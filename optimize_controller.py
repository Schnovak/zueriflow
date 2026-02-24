"""
ZüriFlow Controller Optimizer
=============================
Uses Optuna (Bayesian optimization with TPE) to find optimal GLOSA parameters.

Run with: py -3.13 optimize_controller.py

Requirements: pip install optuna
"""

import optuna
from optuna.samplers import TPESampler
import traci
import sys
import os
import json
from dataclasses import dataclass

sys.stdout.reconfigure(line_buffering=True)

os.environ['SUMO_HOME'] = os.environ.get('SUMO_HOME', r'C:\Program Files (x86)\Eclipse\Sumo')


@dataclass
class ControllerConfig:
    """Parameters to optimize - matches run_demo.py structure"""
    min_speed: float         # m/s - minimum advisory speed
    max_speed: float         # m/s - maximum advisory speed
    lookahead: float         # meters - distance to start advising
    buffer_distance: float   # meters - release control this far from light
    green_buffer: float      # seconds after green start to target
    coast_divisor: float     # controls how gradually to slow down


class VehicleStats:
    """
    Physics-based statistics tracking (same as compare_scenarios.py)
    - Base fuel = distance * base_consumption
    - Stop penalty = kinetic energy lost at each FULL stop
    """
    def __init__(self):
        self.total_distance = 0.0
        self.total_time = 0.0
        self.total_stopped_time = 0.0
        self.num_full_stops = 0
        self.prev_moving = {}
        self.speed_before_stop = {}
        self.energy_wasted_J = 0.0
        self.mass = 1400  # kg

    def update(self, veh_id, step_length=0.1):
        speed = traci.vehicle.getSpeed(veh_id)
        self.total_distance += speed * step_length
        self.total_time += step_length

        is_moving = speed > 0.5
        was_moving = self.prev_moving.get(veh_id, True)

        if is_moving:
            self.speed_before_stop[veh_id] = max(
                speed, self.speed_before_stop.get(veh_id, 0)
            )

        if not is_moving:
            self.total_stopped_time += step_length
            if was_moving:
                v = self.speed_before_stop.get(veh_id, 10)
                ke_lost = 0.5 * self.mass * v**2
                self.energy_wasted_J += ke_lost
                self.num_full_stops += 1
                self.speed_before_stop[veh_id] = 0

        self.prev_moving[veh_id] = is_moving

    def results(self):
        total_km = self.total_distance / 1000
        base_fuel_L = total_km * 0.06
        extra_fuel_L = (self.energy_wasted_J / 0.25) / 34_200_000
        fuel_L = base_fuel_L + extra_fuel_L
        co2_kg = fuel_L * 2.31

        return {
            'co2_kg': co2_kg,
            'fuel_L': fuel_L,
            'stopped_min': self.total_stopped_time / 60,
            'num_stops': self.num_full_stops,
            'avg_speed_kmh': (self.total_distance / self.total_time * 3.6) if self.total_time > 0 else 0,
        }


class GreenWaveController:
    """GLOSA controller - same structure as run_demo.py"""

    def __init__(self, config: ControllerConfig):
        self.cfg = config

    def get_green_phases(self, tls_id, link_index, current_time):
        program = traci.trafficlight.getAllProgramLogics(tls_id)[0]
        phases = [(p.duration, p.state[link_index] if link_index < len(p.state) else p.state[0])
                  for p in program.phases]
        current_phase_idx = traci.trafficlight.getPhase(tls_id)
        next_switch = traci.trafficlight.getNextSwitch(tls_id)
        green_windows = []
        time_in_current = phases[current_phase_idx][0] - (next_switch - current_time)
        t = current_time - time_in_current
        idx = current_phase_idx
        while t < current_time + 120:
            duration, state = phases[idx]
            if state in ('G', 'g') and t + duration > current_time:
                green_windows.append((max(t, current_time), t + duration))
            t += duration
            idx = (idx + 1) % len(phases)
        return green_windows

    def apply_control(self, veh_id, current_time):
        speed = traci.vehicle.getSpeed(veh_id)
        road = traci.vehicle.getRoadID(veh_id)
        if road.startswith(':'):
            return False

        tls_list = traci.vehicle.getNextTLS(veh_id)
        if not tls_list:
            traci.vehicle.setSpeed(veh_id, -1)
            return False

        tls_id, link_index, distance, _ = tls_list[0]

        # Too far or already close - let SUMO drive
        if distance > self.cfg.lookahead or distance < self.cfg.buffer_distance:
            traci.vehicle.setSpeed(veh_id, -1)
            return False

        green_windows = self.get_green_phases(tls_id, link_index, current_time)
        if not green_windows:
            traci.vehicle.setSpeed(veh_id, -1)
            return False

        # Check if we'll arrive at buffer zone during green - release control
        if speed > 1.0:
            arrival = current_time + (distance - self.cfg.buffer_distance) / speed
            for gs, ge in green_windows:
                if gs <= arrival <= ge - 2.0:
                    traci.vehicle.setSpeed(veh_id, -1)
                    return False

        # Find speed to reach buffer zone when green starts
        for gs, ge in green_windows:
            if ge <= current_time:
                continue
            target = gs + self.cfg.green_buffer
            time_avail = target - current_time
            if time_avail <= 0:
                continue
            travel_distance = distance - self.cfg.buffer_distance
            req_speed = travel_distance / time_avail
            if self.cfg.min_speed <= req_speed <= self.cfg.max_speed:
                current = traci.vehicle.getSpeed(veh_id)
                if req_speed < current:
                    coast_time = (current - req_speed) / self.cfg.coast_divisor
                    traci.vehicle.slowDown(veh_id, req_speed, max(3.0, coast_time))
                else:
                    traci.vehicle.setSpeed(veh_id, -1)
                return True

        # No feasible speed - slow down to wait
        traci.vehicle.slowDown(veh_id, self.cfg.min_speed, 3.0)
        return True


def run_simulation(config: ControllerConfig = None, seed: int = 42) -> dict:
    """Run simulation with optional controller, return physics-based metrics"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cfg_file = os.path.join(script_dir, "zueriflow.sumocfg")

    if sys.platform == "win32":
        sumo_home = os.environ.get("SUMO_HOME", r"C:\Program Files (x86)\Eclipse\Sumo")
        sumo_bin = os.path.join(sumo_home, "bin", "sumo.exe")
    else:
        sumo_bin = "sumo"

    cmd = [sumo_bin, "-c", cfg_file, "--seed", str(seed), "--no-warnings", "true"]
    traci.start(cmd)

    controller = GreenWaveController(config) if config else None
    stats = VehicleStats()
    step_length = 0.1

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        t = traci.simulation.getTime()

        for veh_id in traci.vehicle.getIDList():
            if traci.vehicle.getTypeID(veh_id) == "zueriflow":
                stats.update(veh_id, step_length)
                if controller:
                    controller.apply_control(veh_id, t)

    traci.close()
    return stats.results()


# Cache baseline results
_baseline_cache = {}

def get_baseline(seed: int = 42) -> dict:
    if seed not in _baseline_cache:
        print(f"Running baseline (seed={seed})...")
        _baseline_cache[seed] = run_simulation(config=None, seed=seed)
        b = _baseline_cache[seed]
        print(f"  CO2={b['co2_kg']:.2f}kg, Stops={b['num_stops']}")
    return _baseline_cache[seed]


def objective(trial: optuna.Trial) -> float:
    """Optuna objective - maximize CO2 and stop reduction"""

    config = ControllerConfig(
        min_speed=trial.suggest_float('min_speed', 0.3, 3.0),
        max_speed=trial.suggest_float('max_speed', 12.0, 16.0),
        lookahead=trial.suggest_float('lookahead', 200.0, 400.0),
        buffer_distance=trial.suggest_float('buffer_distance', 15.0, 50.0),
        green_buffer=trial.suggest_float('green_buffer', 0.3, 2.0),
        coast_divisor=trial.suggest_float('coast_divisor', 0.5, 2.0),
    )

    seeds = [42, 123, 456]
    total_score = 0.0

    for seed in seeds:
        baseline = get_baseline(seed)
        results = run_simulation(config, seed)

        co2_improvement = (baseline['co2_kg'] - results['co2_kg']) / baseline['co2_kg']
        stop_improvement = (baseline['num_stops'] - results['num_stops']) / baseline['num_stops'] if baseline['num_stops'] > 0 else 0

        # 60% CO2, 40% stops
        score = 0.60 * co2_improvement + 0.40 * stop_improvement
        total_score += score

        trial.report(score, seed)
        if trial.should_prune():
            raise optuna.TrialPruned()

    return total_score / len(seeds)


def main():
    print("=" * 60)
    print("ZüriFlow Controller Optimization")
    print("=" * 60)
    print()

    # Pre-compute baselines
    for seed in [42, 123, 456]:
        get_baseline(seed)

    print()
    print("Starting optimization (50 trials)...")
    print("-" * 60)

    sampler = TPESampler(n_startup_trials=10, seed=42)
    study = optuna.create_study(
        direction='maximize',
        sampler=sampler,
        pruner=optuna.pruners.MedianPruner(n_warmup_steps=1),
    )

    study.optimize(objective, n_trials=50, show_progress_bar=True, gc_after_trial=True)

    print()
    print("=" * 60)
    print("RESULTS")
    print("=" * 60)

    best = study.best_trial
    print(f"\nBest score: {best.value:.4f}")
    print(f"\nOptimal parameters:")
    for key, value in best.params.items():
        print(f"  {key}: {value:.4f}")

    # Final comparison
    optimal_config = ControllerConfig(**best.params)
    baseline = get_baseline(42)
    optimized = run_simulation(optimal_config, 42)

    print(f"\n{'Metric':<20} {'Baseline':>12} {'Optimized':>12} {'Change':>12}")
    print("-" * 60)
    for metric in ['co2_kg', 'fuel_L', 'num_stops', 'avg_speed_kmh']:
        b, o = baseline[metric], optimized[metric]
        pct = (o - b) / b * 100 if b != 0 else 0
        print(f"{metric:<20} {b:>12.2f} {o:>12.2f} {pct:>+11.1f}%")

    # Save results
    results_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'optimization_results.json')
    with open(results_file, 'w') as f:
        json.dump({'best_params': best.params, 'best_score': best.value}, f, indent=2)

    print(f"\nSaved to: {results_file}")

    # Code snippet for run_demo.py
    print()
    print("=" * 60)
    print("UPDATE run_demo.py WITH:")
    print("=" * 60)
    print(f"""
class GreenWaveController:
    def __init__(self):
        self.min_speed = {best.params['min_speed']:.2f}
        self.max_speed = {best.params['max_speed']:.2f}
        self.lookahead = {best.params['lookahead']:.1f}

    # In apply_control():
    #   buffer_distance = {best.params['buffer_distance']:.1f}
    #   target = gs + {best.params['green_buffer']:.2f}
    #   coast_time = (current - req_speed) / {best.params['coast_divisor']:.2f}
""")


if __name__ == "__main__":
    main()

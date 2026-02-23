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
from typing import Optional

sys.stdout.reconfigure(line_buffering=True)

# Suppress SUMO warnings during optimization
os.environ['SUMO_HOME'] = os.environ.get('SUMO_HOME', r'C:\Program Files (x86)\Eclipse\Sumo')


@dataclass
class ControllerConfig:
    """Parameters to optimize"""
    min_speed: float        # m/s - minimum advisory speed
    max_speed: float        # m/s - maximum advisory speed
    lookahead: float        # meters - distance to start advising
    max_accel: float        # m/s per step - max speed increase
    max_decel: float        # m/s per step - max speed decrease
    green_buffer_start: float  # seconds after green start to target
    green_buffer_end: float    # seconds before green end as buffer
    speed_threshold: float     # m/s - below this, consider stopped


class OptimizedController:
    """GLOSA controller with configurable parameters"""

    def __init__(self, config: ControllerConfig):
        self.cfg = config

    def get_green_phases(self, tls_id, link_index, current_time, horizon=120):
        program = traci.trafficlight.getAllProgramLogics(tls_id)[0]
        phases = [(p.duration, p.state[link_index] if link_index < len(p.state) else p.state[0])
                  for p in program.phases]

        current_phase_idx = traci.trafficlight.getPhase(tls_id)
        next_switch = traci.trafficlight.getNextSwitch(tls_id)

        green_windows = []
        time_in_current = phases[current_phase_idx][0] - (next_switch - current_time)
        t = current_time - time_in_current
        idx = current_phase_idx

        while t < current_time + horizon:
            duration, state = phases[idx]
            phase_start = t
            phase_end = t + duration

            if state in ('G', 'g'):
                if phase_end > current_time:
                    green_windows.append((max(phase_start, current_time), phase_end))

            t = phase_end
            idx = (idx + 1) % len(phases)

        return green_windows

    def compute_advisory(self, veh_id, current_time) -> Optional[float]:
        speed = traci.vehicle.getSpeed(veh_id)
        road = traci.vehicle.getRoadID(veh_id)

        if road.startswith(':'):
            return None

        tls_list = traci.vehicle.getNextTLS(veh_id)
        if not tls_list:
            return None

        tls_id, link_index, distance, current_state = tls_list[0]

        if distance > self.cfg.lookahead or distance < 5:
            return None

        green_windows = self.get_green_phases(tls_id, link_index, current_time)
        if not green_windows:
            return None

        # Check if we'll make a green at current speed
        if speed > 0.5:
            current_arrival = current_time + distance / speed
        else:
            current_arrival = current_time + 999

        for green_start, green_end in green_windows:
            if green_start <= current_arrival <= green_end - self.cfg.green_buffer_end:
                return None  # Will make this green

        # Find best green to target
        for green_start, green_end in green_windows:
            if green_end <= current_time:
                continue

            target_arrival = green_start + self.cfg.green_buffer_start
            if target_arrival > green_end - self.cfg.green_buffer_end:
                target_arrival = (green_start + green_end) / 2

            time_available = target_arrival - current_time
            if time_available <= 0:
                continue

            required_speed = distance / time_available

            if self.cfg.min_speed <= required_speed <= self.cfg.max_speed:
                return required_speed

        return None

    def apply_control(self, veh_id, current_time) -> bool:
        advisory = self.compute_advisory(veh_id, current_time)

        if advisory is None:
            traci.vehicle.setSpeed(veh_id, -1)
            return False

        current_speed = traci.vehicle.getSpeed(veh_id)
        diff = advisory - current_speed

        # Asymmetric acceleration/deceleration limits
        if diff > 0:
            new_speed = min(advisory, current_speed + self.cfg.max_accel)
        else:
            new_speed = max(advisory, current_speed - self.cfg.max_decel)

        new_speed = max(self.cfg.min_speed, min(self.cfg.max_speed, new_speed))
        traci.vehicle.setSpeed(veh_id, new_speed)
        return True


class SimulationRunner:
    """Run simulation and collect metrics"""

    def __init__(self, config: ControllerConfig):
        self.config = config
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.cfg_file = os.path.join(self.script_dir, "zueriflow.sumocfg")

    def run(self, seed: int = 42) -> dict:
        """Run simulation and return metrics"""
        if sys.platform == "win32":
            sumo_home = os.environ.get("SUMO_HOME", r"C:\Program Files (x86)\Eclipse\Sumo")
            sumo_bin = os.path.join(sumo_home, "bin", "sumo.exe")
        else:
            sumo_bin = "sumo"

        cmd = [sumo_bin, "-c", self.cfg_file, "--seed", str(seed), "--no-warnings", "true"]
        traci.start(cmd)

        controller = OptimizedController(self.config)

        # Metrics
        total_co2_mg = 0.0
        total_fuel_mg = 0.0
        total_stopped_time = 0.0
        total_distance = 0.0
        total_time = 0.0
        num_stops = 0
        prev_moving = {}

        step_length = 0.1

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            t = traci.simulation.getTime()

            for veh_id in traci.vehicle.getIDList():
                vtype = traci.vehicle.getTypeID(veh_id)

                if vtype == "zueriflow":
                    speed = traci.vehicle.getSpeed(veh_id)
                    co2 = traci.vehicle.getCO2Emission(veh_id)
                    fuel = traci.vehicle.getFuelConsumption(veh_id)

                    total_co2_mg += co2 * step_length
                    total_fuel_mg += fuel * step_length
                    total_distance += speed * step_length
                    total_time += step_length

                    is_moving = speed > self.config.speed_threshold
                    was_moving = prev_moving.get(veh_id, True)

                    if not is_moving:
                        total_stopped_time += step_length
                        if was_moving:
                            num_stops += 1

                    prev_moving[veh_id] = is_moving
                    controller.apply_control(veh_id, t)

        traci.close()

        fuel_density = 750_000  # mg/L

        return {
            'co2_kg': total_co2_mg / 1_000_000,
            'fuel_L': total_fuel_mg / fuel_density,
            'stopped_min': total_stopped_time / 60,
            'num_stops': num_stops,
            'avg_speed_kmh': (total_distance / total_time * 3.6) if total_time > 0 else 0,
        }


def run_baseline(seed: int = 42) -> dict:
    """Run baseline simulation without controller"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cfg_file = os.path.join(script_dir, "zueriflow.sumocfg")

    if sys.platform == "win32":
        sumo_home = os.environ.get("SUMO_HOME", r"C:\Program Files (x86)\Eclipse\Sumo")
        sumo_bin = os.path.join(sumo_home, "bin", "sumo.exe")
    else:
        sumo_bin = "sumo"

    cmd = [sumo_bin, "-c", cfg_file, "--seed", str(seed), "--no-warnings", "true"]
    traci.start(cmd)

    total_co2_mg = 0.0
    total_fuel_mg = 0.0
    total_stopped_time = 0.0
    total_distance = 0.0
    total_time = 0.0
    num_stops = 0
    prev_moving = {}
    step_length = 0.1

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        for veh_id in traci.vehicle.getIDList():
            vtype = traci.vehicle.getTypeID(veh_id)

            if vtype == "zueriflow":
                speed = traci.vehicle.getSpeed(veh_id)
                co2 = traci.vehicle.getCO2Emission(veh_id)
                fuel = traci.vehicle.getFuelConsumption(veh_id)

                total_co2_mg += co2 * step_length
                total_fuel_mg += fuel * step_length
                total_distance += speed * step_length
                total_time += step_length

                is_moving = speed > 0.1
                was_moving = prev_moving.get(veh_id, True)

                if not is_moving:
                    total_stopped_time += step_length
                    if was_moving:
                        num_stops += 1

                prev_moving[veh_id] = is_moving

    traci.close()

    return {
        'co2_kg': total_co2_mg / 1_000_000,
        'fuel_L': total_fuel_mg / 750_000,
        'stopped_min': total_stopped_time / 60,
        'num_stops': num_stops,
        'avg_speed_kmh': (total_distance / total_time * 3.6) if total_time > 0 else 0,
    }


# Cache baseline results
_baseline_cache = {}

def get_baseline(seed: int = 42) -> dict:
    if seed not in _baseline_cache:
        print(f"Running baseline simulation (seed={seed})...")
        _baseline_cache[seed] = run_baseline(seed)
        print(f"Baseline: CO2={_baseline_cache[seed]['co2_kg']:.3f}kg, "
              f"Stops={_baseline_cache[seed]['num_stops']}")
    return _baseline_cache[seed]


def objective(trial: optuna.Trial) -> float:
    """
    Optuna objective function.

    We want to MINIMIZE a combined score that penalizes:
    - High CO2 emissions
    - Many stops
    - Long stopped time

    Returns a score where LOWER is BETTER.
    """

    # Sample hyperparameters
    config = ControllerConfig(
        min_speed=trial.suggest_float('min_speed', 2.0, 8.0),
        max_speed=trial.suggest_float('max_speed', 11.0, 16.0),
        lookahead=trial.suggest_float('lookahead', 80.0, 250.0),
        max_accel=trial.suggest_float('max_accel', 0.1, 0.8),
        max_decel=trial.suggest_float('max_decel', 0.1, 0.8),
        green_buffer_start=trial.suggest_float('green_buffer_start', 0.5, 4.0),
        green_buffer_end=trial.suggest_float('green_buffer_end', 0.5, 3.0),
        speed_threshold=trial.suggest_float('speed_threshold', 0.05, 0.5),
    )

    # Run simulation
    runner = SimulationRunner(config)

    # Test on multiple seeds for robustness
    seeds = [42, 123, 456]

    total_score = 0.0

    for seed in seeds:
        baseline = get_baseline(seed)
        results = runner.run(seed)

        # Calculate relative improvements (positive = better)
        co2_improvement = (baseline['co2_kg'] - results['co2_kg']) / baseline['co2_kg']
        stop_improvement = (baseline['num_stops'] - results['num_stops']) / baseline['num_stops'] if baseline['num_stops'] > 0 else 0
        stopped_time_improvement = (baseline['stopped_min'] - results['stopped_min']) / baseline['stopped_min'] if baseline['stopped_min'] > 0 else 0

        # Weighted score: prioritize CO2 reduction, then stops, then stopped time
        # Higher score = better
        score = (
            0.50 * co2_improvement +      # 50% weight on CO2
            0.35 * stop_improvement +      # 35% weight on stops
            0.15 * stopped_time_improvement  # 15% weight on stopped time
        )

        total_score += score

        # Report intermediate value for pruning
        trial.report(score, seed)

        if trial.should_prune():
            raise optuna.TrialPruned()

    avg_score = total_score / len(seeds)

    # We want to MAXIMIZE improvement, so return negative (Optuna minimizes by default)
    # Or we can set direction='maximize' in create_study
    return avg_score


def main():
    print("=" * 70)
    print("ZüriFlow Controller Optimization")
    print("Using Optuna with TPE (Tree-structured Parzen Estimator)")
    print("=" * 70)
    print()

    # Pre-compute baseline for all seeds
    for seed in [42, 123, 456]:
        get_baseline(seed)

    print()
    print("Starting optimization...")
    print("-" * 70)

    # Create study with TPE sampler
    sampler = TPESampler(
        n_startup_trials=10,  # Random trials before TPE kicks in
        seed=42,
    )

    study = optuna.create_study(
        direction='maximize',  # We want to maximize improvement score
        sampler=sampler,
        pruner=optuna.pruners.MedianPruner(n_warmup_steps=1),
    )

    # Run optimization
    n_trials = 50  # Adjust based on available time

    study.optimize(
        objective,
        n_trials=n_trials,
        show_progress_bar=True,
        gc_after_trial=True,
    )

    print()
    print("=" * 70)
    print("OPTIMIZATION COMPLETE")
    print("=" * 70)

    # Best trial
    best = study.best_trial
    print(f"\nBest trial #{best.number}:")
    print(f"  Score: {best.value:.4f} (higher = better)")
    print(f"\nOptimal parameters:")

    for key, value in best.params.items():
        print(f"  {key}: {value:.4f}")

    # Create optimal config
    optimal_config = ControllerConfig(**best.params)

    # Run final comparison
    print()
    print("=" * 70)
    print("FINAL COMPARISON WITH OPTIMAL CONFIG")
    print("=" * 70)

    baseline = get_baseline(42)
    runner = SimulationRunner(optimal_config)
    optimized = runner.run(42)

    print(f"\n{'Metric':<20} {'Baseline':>12} {'Optimized':>12} {'Change':>12}")
    print("-" * 60)

    for metric in ['co2_kg', 'fuel_L', 'stopped_min', 'num_stops', 'avg_speed_kmh']:
        b_val = baseline[metric]
        o_val = optimized[metric]
        change = (o_val - b_val) / b_val * 100 if b_val != 0 else 0
        print(f"{metric:<20} {b_val:>12.3f} {o_val:>12.3f} {change:>+11.1f}%")

    # Save results
    results = {
        'best_params': best.params,
        'best_score': best.value,
        'baseline': baseline,
        'optimized': optimized,
    }

    results_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'optimization_results.json')
    with open(results_file, 'w') as f:
        json.dump(results, f, indent=2)

    print(f"\nResults saved to: {results_file}")

    # Print code snippet for compare_scenarios.py
    print()
    print("=" * 70)
    print("UPDATE compare_scenarios.py WITH THESE VALUES:")
    print("=" * 70)
    print(f"""
    def __init__(self):
        self.min_speed = {best.params['min_speed']:.4f}
        self.max_speed = {best.params['max_speed']:.4f}
        self.lookahead = {best.params['lookahead']:.4f}
        self.max_speed_change = {best.params['max_accel']:.4f}  # For acceleration
        # Note: Also add max_decel = {best.params['max_decel']:.4f}
        # green_buffer_start = {best.params['green_buffer_start']:.4f}
        # green_buffer_end = {best.params['green_buffer_end']:.4f}
""")


if __name__ == "__main__":
    main()

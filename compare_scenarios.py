"""
ZüriFlow Comparison: With vs Without Controller
================================================
Run with: py -3.13 compare_scenarios.py

This script runs two identical simulations and compares the results.
"""

import traci
import sys
import os

sys.stdout.reconfigure(line_buffering=True)

def log(msg):
    print(msg, flush=True)


# =============================================================================
# STATISTICS TRACKING
# =============================================================================

class VehicleStats:
    """
    Track statistics using simple physics:
    - Base fuel = distance * base_consumption
    - Stop penalty = kinetic energy lost at each FULL stop

    Key insight: only full stops waste significant energy.
    Gradual speed changes are efficient (coasting uses no fuel).
    """
    def __init__(self, name):
        self.name = name
        self.total_distance = 0.0
        self.total_time = 0.0
        self.total_stopped_time = 0.0
        self.num_full_stops = 0
        self.prev_moving = {}
        self.speed_before_stop = {}  # track speed when vehicle starts stopping

        # Track kinetic energy lost at full stops
        self.energy_wasted_J = 0.0
        self.mass = 1400  # kg

    def update(self, veh_id, step_length=0.1):
        speed = traci.vehicle.getSpeed(veh_id)

        self.total_distance += speed * step_length
        self.total_time += step_length

        is_moving = speed > 0.5
        was_moving = self.prev_moving.get(veh_id, True)

        if is_moving:
            # Track highest speed while moving (for when we stop)
            self.speed_before_stop[veh_id] = max(
                speed,
                self.speed_before_stop.get(veh_id, 0)
            )

        if not is_moving:
            self.total_stopped_time += step_length

            if was_moving:
                # Full stop! Calculate kinetic energy wasted
                v = self.speed_before_stop.get(veh_id, 10)  # speed before stopping
                ke_lost = 0.5 * self.mass * v**2
                self.energy_wasted_J += ke_lost
                self.num_full_stops += 1
                self.speed_before_stop[veh_id] = 0

        self.prev_moving[veh_id] = is_moving

    def results(self):
        total_km = self.total_distance / 1000

        # Base consumption: 6 L/100km at steady speed
        base_fuel_L = total_km * 0.06

        # Extra fuel to regenerate kinetic energy lost at stops
        # Engine efficiency ~25%
        # Gasoline: 34.2 MJ/L
        extra_fuel_L = (self.energy_wasted_J / 0.25) / 34_200_000

        fuel_L = base_fuel_L + extra_fuel_L
        co2_kg = fuel_L * 2.31

        return {
            'co2_kg': co2_kg,
            'fuel_L': fuel_L,
            'stopped_min': self.total_stopped_time / 60,
            'num_stops': self.num_full_stops,
            'avg_speed_kmh': (self.total_distance / self.total_time * 3.6) if self.total_time > 0 else 0,
            'total_km': total_km,
            'total_time_min': self.total_time / 60,
            'co2_per_km': (co2_kg / total_km * 1000) if total_km > 0 else 0,
        }


# =============================================================================
# GREEN WAVE CONTROLLER - DETAILED EXPLANATION
# =============================================================================

class GreenWaveController:
    """
    GLOSA (Green Light Optimal Speed Advisory) Controller

    CORE PRINCIPLE:
    ---------------
    When approaching a traffic light, we want to avoid stopping.
    Stopping wastes energy because:
      1. Kinetic energy is lost to brakes (heat)
      2. Must accelerate again (fuel burn)
      3. Creates traffic waves behind

    STRATEGY:
    ---------
    1. Look at upcoming traffic light
    2. Predict: "At my current speed, what will the light be when I arrive?"
    3. If GREEN → do nothing, coast through
    4. If RED → calculate: "What speed would let me arrive when it turns green?"
    5. Apply that speed gradually (no sudden changes)

    KEY INSIGHT:
    ------------
    It's better to drive at 30 km/h continuously than to drive at 50 km/h
    and then stop. The slow-but-steady approach:
      - Uses less fuel (no acceleration from zero)
      - Keeps traffic flowing (no backup behind stopped cars)
      - Reduces stress (no hard braking)
    """

    def __init__(self):
        # Configuration for GLOSA - tuned for zero braking at lights
        self.min_speed = 2.0        # m/s - can go very slow to time perfectly
        self.max_speed = 13.89      # m/s (50 km/h) - speed limit
        self.lookahead = 350.0      # meters - start very early
        self.max_accel = 0.4        # m/s per step
        self.max_decel = 0.15       # m/s per step - very gentle
        self.green_buffer_start = 1.0
        self.green_buffer_end = 1.5

    def get_green_phases(self, tls_id, link_index, current_time, horizon=120):
        """
        Returns list of (start_time, end_time) for all green phases
        within the next 'horizon' seconds.

        This is better than just knowing "time to next switch" because
        we get the FULL picture of when greens occur.

        Args:
            tls_id: Traffic light ID
            link_index: The signal index for this vehicle's lane
            current_time: Current simulation time
            horizon: How far ahead to look (seconds)
        """
        program = traci.trafficlight.getAllProgramLogics(tls_id)[0]
        # Use the correct signal index for this vehicle's lane
        phases = [(p.duration, p.state[link_index] if link_index < len(p.state) else p.state[0])
                  for p in program.phases]

        current_phase_idx = traci.trafficlight.getPhase(tls_id)
        next_switch = traci.trafficlight.getNextSwitch(tls_id)

        # Build timeline of green phases
        green_windows = []

        # Start from beginning of current phase
        time_in_current = phases[current_phase_idx][0] - (next_switch - current_time)
        t = current_time - time_in_current

        idx = current_phase_idx

        while t < current_time + horizon:
            duration, state = phases[idx]
            phase_start = t
            phase_end = t + duration

            if state in ('G', 'g'):
                # Only include future portions
                if phase_end > current_time:
                    green_windows.append((
                        max(phase_start, current_time),
                        phase_end
                    ))

            t = phase_end
            idx = (idx + 1) % len(phases)

        return green_windows

    def compute_advisory(self, veh_id, current_time):
        """
        Compute speed advisory for a vehicle.

        Returns:
            float: target speed in m/s, or None if no advisory needed

        LOGIC FLOW:
        -----------
        1. Get distance to next traffic light
        2. Get all upcoming green phases
        3. For each green phase, calculate:
           - What speed would make me arrive at the START of this green?
           - What speed would make me arrive at the END of this green?
        4. Find a green phase where the required speed is reasonable
        5. Return that speed (or None if already fine)
        """
        # Get vehicle state
        speed = traci.vehicle.getSpeed(veh_id)
        road = traci.vehicle.getRoadID(veh_id)

        # Don't control in junctions (internal edges start with ':')
        if road.startswith(':'):
            return None

        # Get next traffic light
        tls_list = traci.vehicle.getNextTLS(veh_id)
        if not tls_list:
            return None

        tls_id, link_index, distance, current_state = tls_list[0]

        # Only advise within lookahead range, and not too close
        if distance > self.lookahead or distance < 5:
            return None

        # Get all green phases for this vehicle's specific lane
        green_windows = self.get_green_phases(tls_id, link_index, current_time)
        if not green_windows:
            return None

        # Calculate arrival time at current speed
        if speed > 0.5:
            current_arrival = current_time + distance / speed
        else:
            current_arrival = current_time + 999

        # Check: will I arrive during any green phase?
        for green_start, green_end in green_windows:
            # Use optimized buffer at end
            if green_start <= current_arrival <= green_end - self.green_buffer_end:
                # I'll make this green! No intervention needed.
                return None

        # I won't make a green at current speed.
        # Find the best green phase to target.

        for green_start, green_end in green_windows:
            # Skip greens that already ended
            if green_end <= current_time:
                continue

            # Target: arrive shortly after green starts (optimized buffer)
            target_arrival = green_start + self.green_buffer_start

            # But don't target past the green end
            if target_arrival > green_end - self.green_buffer_end:
                target_arrival = (green_start + green_end) / 2

            # Calculate required speed
            time_available = target_arrival - current_time

            if time_available <= 0:
                continue

            required_speed = distance / time_available

            # Is this speed reasonable?
            if self.min_speed <= required_speed <= self.max_speed:
                return required_speed

        # Couldn't find a suitable speed - let SUMO handle it
        return None

    def apply_control(self, veh_id, current_time):
        """
        Apply speed control with smooth transitions.

        Returns True if actively controlling, False otherwise.
        """
        advisory = self.compute_advisory(veh_id, current_time)

        if advisory is None:
            # Release control - let SUMO drive normally
            traci.vehicle.setSpeed(veh_id, -1)
            return False

        # Apply advisory speed directly
        current_speed = traci.vehicle.getSpeed(veh_id)

        # Smooth transition to target speed
        if advisory < current_speed:
            new_speed = max(advisory, current_speed - self.max_decel)
        else:
            new_speed = min(advisory, current_speed + self.max_accel)

        # Enforce bounds
        new_speed = max(self.min_speed, min(self.max_speed, new_speed))

        traci.vehicle.setSpeed(veh_id, new_speed)
        return True


# =============================================================================
# SIMULATION RUNNER
# =============================================================================

def run_simulation(with_controller: bool, gui: bool = False):
    """
    Run simulation and return statistics for both vehicle types.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cfg = os.path.join(script_dir, "zueriflow.sumocfg")

    # Find SUMO binary
    if sys.platform == "win32":
        sumo_home = os.environ.get("SUMO_HOME", r"C:\Program Files (x86)\Eclipse\Sumo")
        binary = "sumo-gui.exe" if gui else "sumo.exe"
        sumo_bin = os.path.join(sumo_home, "bin", binary)
    else:
        sumo_bin = "sumo-gui" if gui else "sumo"

    cmd = [sumo_bin, "-c", cfg, "--seed", "42"]
    if gui:
        cmd.extend(["--start", "--delay", "50"])  # Slower for visibility

    traci.start(cmd)

    # Separate stats for each vehicle type
    stats_normal = VehicleStats("normal")
    stats_zueriflow = VehicleStats("zueriflow")

    controller = GreenWaveController()
    step = 0

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        t = traci.simulation.getTime()

        for veh_id in traci.vehicle.getIDList():
            vtype = traci.vehicle.getTypeID(veh_id)

            if vtype == "zueriflow":
                stats_zueriflow.update(veh_id)

                if with_controller:
                    is_controlling = controller.apply_control(veh_id, t)
                    # Dynamic color based on control state:
                    #   GREEN  = actively adjusting speed for traffic light
                    #   CYAN   = no adjustment needed (will make green naturally)
                    if is_controlling:
                        traci.vehicle.setColor(veh_id, (0, 255, 0, 255))  # Bright green
                    else:
                        traci.vehicle.setColor(veh_id, (0, 200, 255, 255))  # Cyan
            else:
                stats_normal.update(veh_id)

        if step % 100 == 0:
            n_total = len(traci.vehicle.getIDList())
            log(f"  t={t:.0f}s, {n_total} vehicles")

        step += 1

    traci.close()

    return {
        'normal': stats_normal.results(),
        'zueriflow': stats_zueriflow.results(),
    }


# =============================================================================
# MAIN
# =============================================================================

def print_comparison(label, wo, w):
    """Print a metric comparison"""
    diff = w - wo
    pct = (diff / wo * 100) if wo != 0 else 0
    log(f"{label:<25} {wo:>12.2f} {w:>12.2f} {diff:>+12.2f} ({pct:+.1f}%)")


def main():
    log("=" * 70)
    log("ZUERIFLOW COMPARISON TEST")
    log("=" * 70)
    log("")

    log("SCENARIO 1: BASELINE (no controller)")
    log("-" * 40)
    results_without = run_simulation(with_controller=False)
    log("")

    log("SCENARIO 2: WITH ZüriFlow Controller")
    log("-" * 40)
    results_with = run_simulation(with_controller=True)
    log("")

    # Compare ZüriFlow vehicles specifically
    log("=" * 70)
    log("ZUERIFLOW VEHICLES ONLY (the ones receiving advice)")
    log("=" * 70)
    log(f"{'Metric':<25} {'Baseline':>12} {'Controlled':>12} {'Difference':>12}")
    log("-" * 70)

    zf_wo = results_without['zueriflow']
    zf_w = results_with['zueriflow']

    print_comparison("Total Distance (km)", zf_wo['total_km'], zf_w['total_km'])
    print_comparison("Total Time (min)", zf_wo['total_time_min'], zf_w['total_time_min'])
    print_comparison("CO2 (kg)", zf_wo['co2_kg'], zf_w['co2_kg'])
    print_comparison("CO2 (g/km)", zf_wo['co2_per_km'], zf_w['co2_per_km'])
    print_comparison("Fuel (L)", zf_wo['fuel_L'], zf_w['fuel_L'])
    print_comparison("Time Stopped (min)", zf_wo['stopped_min'], zf_w['stopped_min'])
    print_comparison("Full Stops (#)", zf_wo['num_stops'], zf_w['num_stops'])
    print_comparison("Avg Speed (km/h)", zf_wo['avg_speed_kmh'], zf_w['avg_speed_kmh'])

    log("")

    # Compare Normal vehicles (should be similar - they're not controlled)
    log("=" * 70)
    log("NORMAL VEHICLES (not controlled - sanity check)")
    log("=" * 70)
    log(f"{'Metric':<25} {'Baseline':>12} {'With ZF':>12} {'Difference':>12}")
    log("-" * 70)

    n_wo = results_without['normal']
    n_w = results_with['normal']

    print_comparison("CO2 (kg)", n_wo['co2_kg'], n_w['co2_kg'])
    print_comparison("Full Stops (#)", n_wo['num_stops'], n_w['num_stops'])
    print_comparison("Avg Speed (km/h)", n_wo['avg_speed_kmh'], n_w['avg_speed_kmh'])

    log("")
    log("=" * 70)
    log("SUMMARY")
    log("=" * 70)

    # ZüriFlow impact (positive = improvement/reduction, negative = worse)
    co2_change = (zf_w['co2_kg'] - zf_wo['co2_kg']) / zf_wo['co2_kg'] * 100 if zf_wo['co2_kg'] > 0 else 0
    stop_change = (zf_w['num_stops'] - zf_wo['num_stops']) / zf_wo['num_stops'] * 100 if zf_wo['num_stops'] > 0 else 0

    log(f"ZüriFlow vehicles:")
    log(f"  CO2:   {co2_change:+.1f}% {'(worse)' if co2_change > 0 else '(better)' if co2_change < 0 else ''}")
    log(f"  Stops: {stop_change:+.1f}% {'(worse)' if stop_change > 0 else '(better)' if stop_change < 0 else ''}")
    log("")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="ZüriFlow GLOSA Controller Comparison")
    parser.add_argument("--gui", action="store_true", help="Run with SUMO GUI (visual)")
    parser.add_argument("--demo", action="store_true", help="Run controlled scenario with GUI")
    parser.add_argument("--baseline", action="store_true", help="Run baseline (no control) with GUI")
    args = parser.parse_args()

    if args.baseline:
        # Run baseline without controller
        log("=" * 50)
        log("BASELINE (no controller)")
        log("=" * 50)
        log("All vehicles controlled by SUMO only.")
        log("Watch vehicles stop at red lights.")
        log("=" * 50)
        run_simulation(with_controller=False, gui=True)
    elif args.demo:
        # Just run the controlled scenario with GUI for demonstration
        log("=" * 50)
        log("ZüriFlow GLOSA Demo")
        log("=" * 50)
        log("")
        log("VEHICLE COLORS:")
        log("  GREEN = ZüriFlow adjusting speed for light")
        log("  CYAN  = ZüriFlow coasting (will make green)")
        log("  GRAY  = Normal traffic (no control)")
        log("")
        log("Watch GREEN vehicles slow down early to avoid stopping!")
        log("=" * 50)
        run_simulation(with_controller=True, gui=True)
    elif args.gui:
        # Run both scenarios with GUI
        log("Running comparison with GUI (will run TWO simulations)...")
        results_without = run_simulation(with_controller=False, gui=True)
        results_with = run_simulation(with_controller=True, gui=True)
    else:
        main()

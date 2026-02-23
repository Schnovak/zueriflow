"""
ZüriFlow Visual Demo
====================
Runs a nice visual demo you can screen record.

Usage:
    py -3.13 run_demo.py           # Controlled demo (with GLOSA)
    py -3.13 run_demo.py --baseline # Baseline demo (no control)

To record: Press Win+G and click Record, or use OBS
"""

import traci
import sys
import os

class GreenWaveController:
    def __init__(self):
        self.min_speed = 0.5       # Can creep very slowly to time it perfectly
        self.max_speed = 13.89
        self.lookahead = 350.0     # Start very early

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
        buffer_distance = 30.0

        # Too far or already close - let SUMO drive
        if distance > self.lookahead or distance < buffer_distance:
            traci.vehicle.setSpeed(veh_id, -1)
            return False

        green_windows = self.get_green_phases(tls_id, link_index, current_time)
        if not green_windows:
            traci.vehicle.setSpeed(veh_id, -1)
            return False

        # Check if we'll arrive at buffer zone during green - release control
        if speed > 1.0:
            arrival = current_time + (distance - buffer_distance) / speed
            for gs, ge in green_windows:
                if gs <= arrival <= ge - 2.0:
                    traci.vehicle.setSpeed(veh_id, -1)
                    return False

        # Find speed to reach buffer zone when green starts
        for gs, ge in green_windows:
            if ge <= current_time:
                continue
            target = gs + 0.5
            time_avail = target - current_time
            if time_avail <= 0:
                continue
            travel_distance = distance - buffer_distance
            req_speed = travel_distance / time_avail
            if self.min_speed <= req_speed <= self.max_speed:
                current = traci.vehicle.getSpeed(veh_id)
                if req_speed < current:
                    coast_time = (current - req_speed) / 1.0
                    traci.vehicle.slowDown(veh_id, req_speed, max(3.0, coast_time))
                else:
                    traci.vehicle.setSpeed(veh_id, -1)
                return True

        # No feasible speed - slow down to wait for next window
        traci.vehicle.slowDown(veh_id, self.min_speed, 3.0)
        return True


def run_demo(with_controller: bool):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cfg = os.path.join(script_dir, "zueriflow.sumocfg")

    sumo_home = os.environ.get("SUMO_HOME", r"C:\Program Files (x86)\Eclipse\Sumo")
    sumo_bin = os.path.join(sumo_home, "bin", "sumo-gui.exe")

    view_file = os.path.join(script_dir, "zueriflow_speed.xml")
    cmd = [
        sumo_bin, "-c", cfg,
        "--seed", "42",
        "--start", "true",
        "--delay", "30",
        "--window-size", "1920,1080",
        "--window-pos", "0,0",
        "--gui-settings-file", view_file,
    ]

    traci.start(cmd)

    controller = GreenWaveController() if with_controller else None
    stops = 0
    prev_moving = {}

    print()
    if with_controller:
        print("  ZUERIFLOW DEMO - Watch the GREEN vehicles adjust speed!")
    else:
        print("  BASELINE DEMO - Watch vehicles STOP at red lights")
    print()
    print("  Press Win+G to open Game Bar and record")
    print("  Close the SUMO window when done")
    print()

    # Set up camera view (uses ZueriFlow Demo scheme from demo_view.xml)
    traci.gui.setZoom("View #0", 1500)
    tracking_vehicle = None

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        t = traci.simulation.getTime()

        for veh_id in traci.vehicle.getIDList():
            vtype = traci.vehicle.getTypeID(veh_id)
            speed = traci.vehicle.getSpeed(veh_id)


            # Set speed as displayable parameter (parameter name "v" for velocity)
            speed_kmh = int(speed * 3.6)
            traci.vehicle.setParameter(veh_id, "v", str(speed_kmh))

            if controller:
                # Controlled mode - color ZüriFlow vehicles
                if vtype == "zueriflow":
                    is_active = controller.apply_control(veh_id, t)
                    if is_active:
                        traci.vehicle.setColor(veh_id, (50, 255, 50, 255))  # Green = adjusting
                    else:
                        traci.vehicle.setColor(veh_id, (0, 220, 255, 255))  # Cyan = cruising
                else:
                    traci.vehicle.setColor(veh_id, (80, 80, 90, 255))  # Gray
            else:
                # Baseline mode - all cars same color
                traci.vehicle.setColor(veh_id, (80, 80, 90, 255))

            # Count stops
            is_moving = speed > 0.5
            was_moving = prev_moving.get(veh_id, True)
            if not is_moving and was_moving:
                stops += 1
            prev_moving[veh_id] = is_moving

        # Track one ZüriFlow vehicle until it leaves, then pick the newest one
        if tracking_vehicle not in traci.vehicle.getIDList():
            # Find the newest ZüriFlow vehicle (lowest x position)
            tracking_vehicle = None
            best_pos = float('inf')
            for vid in traci.vehicle.getIDList():
                if traci.vehicle.getTypeID(vid) == "zueriflow":
                    pos = traci.vehicle.getPosition(vid)[0]
                    if pos < best_pos:
                        best_pos = pos
                        tracking_vehicle = vid

            if tracking_vehicle:
                traci.gui.trackVehicle("View #0", tracking_vehicle)
                traci.gui.setZoom("View #0", 600)

    traci.close()
    print(f"  Total stops: {stops}")


if __name__ == "__main__":
    baseline = "--baseline" in sys.argv

    print("=" * 50)
    if baseline:
        print("  BASELINE MODE (no speed control)")
    else:
        print("  ZUERIFLOW MODE (green wave active)")
    print("=" * 50)

    run_demo(with_controller=not baseline)

# ZüriFlow

GLOSA (Green Light Optimal Speed Advisory) traffic simulation for Zürich.

Vehicles receive speed recommendations to pass through traffic lights on green, reducing stops and emissions.

**[Watch the demo video](https://youtu.be/EHPM1QoO9jc)**

## Results

### ZüriFlow-equipped vehicles

| Metric | Baseline | With GLOSA | Improvement |
|--------|----------|------------|-------------|
| Full stops | 128 | 30 | **-77%** |
| Time stopped | 22 min | 10 min | **-55%** |
| CO2 emissions | 11.6 kg | 8.5 kg | **-27%** |
| Fuel consumption | 5.0 L | 3.7 L | **-27%** |

### Bonus: Normal vehicles also benefit

Other vehicles in traffic benefit from smoother flow caused by ZüriFlow vehicles:

| Metric | Without ZüriFlow | With ZüriFlow | Improvement |
|--------|------------------|---------------|-------------|
| Full stops | 211 | 63 | **-70%** |
| CO2 emissions | 24.4 kg | 19.6 kg | **-20%** |

### Key Benefits

- **Fewer stops** — Vehicles glide through green lights instead of waiting at red
- **Lower emissions** — Less braking and accelerating means less fuel burned
- **Smoother traffic** — Even non-equipped vehicles benefit from improved flow
- **Same travel time** — No increase in journey duration

*Note: Results from simulation under ideal conditions (no pedestrians, no cross-traffic, perfect V2I communication, consistent signal timing). Real-world results may vary.*

## References

- [Green Light Optimal Speed Advisory (GLOSA) - ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0001457524000794)

## Contact

novak@kalaj.org

---

© 2025 Novak Tomic. All rights reserved.

## Requirements

- SUMO 1.26.0+ (Simulation of Urban Mobility)
- Python 3.13+
- TraCI (included with SUMO)

## Files

### Core Scripts

| File | Description |
|------|-------------|
| `run_demo.py` | Visual demo with GLOSA controller. Run with `--baseline` to compare without control. |
| `compare_scenarios.py` | Runs both scenarios and outputs metrics (stops, fuel, CO2). |
| `optimize_controller.py` | Optuna-based parameter optimization for the GLOSA controller. |

### SUMO Configuration

| File | Description |
|------|-------------|
| `zueriflow.sumocfg` | Main SUMO configuration file. |
| `zueriflow.net.xml` | Road network: 1.2km corridor with 4 traffic lights, 300m apart. |
| `zueriflow.rou.xml` | Vehicle types and traffic flows. 30% ZüriFlow adoption rate. |
| `zueriflow_speed.xml` | GUI view settings with speed display on vehicles. |

## Usage

### Visual Demo (for screen recording)

```bash
# With GLOSA control - watch green vehicles adjust speed
py -3.13 run_demo.py

# Without control - watch vehicles stop at red lights
py -3.13 run_demo.py --baseline
```

### Compare Scenarios

```bash
# Headless comparison with metrics
py -3.13 compare_scenarios.py

# Visual comparison
py -3.13 compare_scenarios.py --gui
```

### Optimize Controller

```bash
py -3.13 optimize_controller.py
```

## How It Works

The GLOSA controller:

1. Detects traffic lights up to 350m ahead
2. Calculates green phase timing
3. Adjusts vehicle speed to arrive ~30m before the light when it turns green
4. Releases control near the light, letting vehicles accelerate through naturally

Vehicle colors in demo:
- **Green**: Actively adjusting speed
- **Cyan**: Cruising (no intervention needed)
- **Gray**: Normal vehicles (no GLOSA)

## Network Layout

```
[Entry] --300m-- [Light1] --300m-- [Light2] --300m-- [Light3] --300m-- [Light4] --300m-- [Exit]
```

Traffic light cycle: 25s green, 4s yellow, 31s red (60s total)
Offsets: 0, 35, 15, 50 seconds (intentionally imperfect coordination)

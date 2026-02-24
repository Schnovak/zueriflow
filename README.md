# ZüriFlow

GLOSA (Green Light Optimal Speed Advisory) traffic simulation for Zürich.

Vehicles receive speed recommendations to pass through traffic lights on green, reducing stops and emissions.

## Videos

**SUMO Simulation Demo:**

[![SUMO Simulation Demo](https://img.youtube.com/vi/dTFsEfYSqCg/0.jpg)](https://youtu.be/dTFsEfYSqCg)

**Project Explanation:**

[![Project Explanation](https://img.youtube.com/vi/EHPM1QoO9jc/0.jpg)](https://youtu.be/EHPM1QoO9jc)

## Presentation

[Download PowerPoint Presentation](ZueriFlow_Presentation.pptx)

## Results

### ZüriFlow-equipped vehicles

| Metric | Baseline | With GLOSA | Improvement |
|--------|----------|------------|-------------|
| Full stops | 128 | 30 | **-77%** |
| Time stopped | 22 min | 10 min | **-55%** |
| CO2 emissions | 11.6 kg | 8.5 kg | **-27%** |
| Fuel consumption | 5.0 L | 3.7 L | **-27%** |

### Normal vehicles also benefit

Other vehicles in traffic benefit from smoother flow:

| Metric | Without ZüriFlow | With ZüriFlow | Improvement |
|--------|------------------|---------------|-------------|
| Full stops | 211 | 63 | **-70%** |
| CO2 emissions | 24.4 kg | 19.6 kg | **-20%** |

### Key Benefits

- **Fewer stops** — Vehicles glide through green lights instead of waiting at red
- **Lower emissions** — Less braking and accelerating means less fuel burned
- **Smoother traffic** — Even non-equipped vehicles benefit from improved flow
- **Same travel time** — No increase in journey duration

*Note: Simulation under ideal conditions (no pedestrians, no cross-traffic, perfect V2I communication). Real-world results may vary.*

## How It Works

1. Detects traffic lights up to 350m ahead
2. Calculates green phase timing
3. Adjusts vehicle speed to arrive ~30m before the light when it turns green
4. Releases control near the light, letting vehicles accelerate through naturally

**Vehicle colors in demo:**
- **Green** = Actively adjusting speed
- **Cyan** = Cruising (no intervention needed)
- **Gray** = Normal vehicles (no GLOSA)

## Requirements

- SUMO 1.26.0+
- Python 3.13+

## Quick Start

```bash
# Run visual demo with GLOSA
py -3.13 run_demo.py

# Run baseline (no control)
py -3.13 run_demo.py --baseline

# Compare metrics
py -3.13 compare_scenarios.py
```

## Files

| File | Description |
|------|-------------|
| `run_demo.py` | Visual demo with GLOSA controller |
| `compare_scenarios.py` | Metrics comparison (stops, fuel, CO2) |
| `optimize_controller.py` | Parameter optimization with Optuna |
| `zueriflow.sumocfg` | SUMO configuration |
| `zueriflow.net.xml` | Road network (1.2km, 4 traffic lights) |
| `zueriflow.rou.xml` | Vehicle types and flows |

## References

- [Green Light Optimal Speed Advisory (GLOSA) - ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0001457524000794)

## Contact

novak@kalaj.org

---

© 2025 Novak Tomic. All rights reserved.

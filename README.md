[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)


# Scenario Module
This packages implements Scenario-Based Model Predictive Control by providing a module for `mpc_planner` (see https://github.com/tud-amr/mpc_planner). 

It currently supports only ROS1.


**Journal Paper:** O. de Groot, L. Ferranti, D. M. Gavrila, and J. Alonso-Mora, *Scenario-Based Trajectory Optimization with Bounded Probability of Collision.* **International Journal of Robotics Research**, 2024. Preprint: https://arxiv.org/pdf/2307.01070

# Connecting to MPC Planner
The `scenario_constraints` module in `mpc_planner` connects to this module. When using the `scenario_constraints` module, please make sure to load its parameter in the launch file, e.g.:

```xml
<rosparam command="load" file="$(find scenario_module)/config/params.yaml"/>
```

The scenario constraints are included in provided configurations in `mpc_planner` when generating a solver. E.g., in `jackalsimulator`, by uncommenting:

```python
model, modules = configuration_safe_horizon(settings)
```

Finally, make sure the predictions include noise, for example by setting in `pedestrian_simulator`, `pedestrians/process_noise` to a non-zero value.

> **Note:** Multiple scenario solvers can be run in parallel by setting `scenario_constraints/parallel_solvers` to a value between `1` and `4` (`4` being the configured maximum number of threads).

---

### Examples

In the examples below about `400` samples are drawn for `12` obstacles and the MPC is updating its plan at `30 Hz` providing a safety guarantee on its probability of collision (keeping it below `10%`)

**Jackal Simulator**

<img src="https://imgur.com/tGkrvta.gif" width="80%">

**ROS Navigation**

<img src="https://imgur.com/4sY15bM.gif" width="80%">

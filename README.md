[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)


# Scenario Module
This packages implements Scenario-Based Model Predictive Control by providing a module for `mpc_planner` (see https://github.com/tud-amr/mpc_planner). 


**Journal Paper:** O. de Groot, L. Ferranti, D. M. Gavrila, and J. Alonso-Mora, *Scenario-Based Trajectory Optimization with Bounded Probability of Collision.* **International Journal of Robotics Research**, 2024. Preprint: https://arxiv.org/pdf/2307.01070

---

### Examples

In the examples below about `400` samples are drawn for `12` obstacles and the MPC is updating its plan at `30 Hz` providing a safety guarantee on its probability of collision (keeping it below `10%`)

**Jackal Simulator**

<img src="https://imgur.com/tGkrvta.gif" width="100%">

**ROS Navigation**
<img src="https://imgur.com/4sY15bM.gif" width="100%"> |

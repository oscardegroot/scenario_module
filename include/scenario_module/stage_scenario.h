/**
 * @file stage_scenario.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Implements S-MPCC (https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9410362)
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef STAGE_SCENARIO_H
#define STAGE_SCENARIO_H

#include "scenario/scenario_manager.h"

#include "scenario/safety_certifier.h"
#include "scenario/polygon_search.h"
#include "lmpcc_tools/helpers.h"
#include "scenario/sampler.h"

#include <Eigen/Eigen>
#include <Eigen/Cholesky>

/** Set to 0 to compile faster without debug information */
#define SCENARIO_DEBUG 1
#if SCENARIO_DEBUG == 1
#define SCENARIO_INFO(msg)                \
  if (config_->debug_output_)             \
  {                                       \
    ROS_INFO_STREAM("[S-MPCC]: " << msg); \
  }
#define SCENARIO_INFO_STREAM(msg)         \
  if (config_->debug_output_)             \
  {                                       \
    ROS_INFO_STREAM("[S-MPCC]: " << msg); \
  }
#else
#define SCENARIO_INFO(msg)
#define SCENARIO_INFO_STREAM(msg)
#endif
namespace ScenarioModule
{
  class StageScenario : public ScenarioManager
  {
  public:
    StageScenario(ros::NodeHandle &nh, ConfigurationMPC *config, const VehicleDisc &vehicle_disc,
                  GaussianSampler *sampler, bool enable_visualization, int solver_id);

  private:
    // Parameters that will be set via config
    u_int S; // sampling count
    u_int N; // Prediction horizon

    int R_; // Scenario removal
    int l_;

    // Received from constructor
    ConfigurationMPC *config_;
    bool enable_visualization_;

    // The visualisation class
    std::unique_ptr<ROSMarkerPublisher> ros_markers_;

    std::unique_ptr<VehicleDisc> disc_;
    // New
    // A pointer to the scenarios
    std::vector<trajectory_sample> *scenarios_;

    // Intermediate distance computation (reuse for obstacles, but threaded for k)
    std::vector<Eigen::ArrayXd> diffs_x_;
    std::vector<Eigen::ArrayXd> diffs_y_;
    std::vector<Eigen::ArrayXd> distances_;

    bool is_feasible_;

    std::vector<int> sort_indices_;

    // These are for all obstacles as well
    std::vector<Eigen::ArrayXd> a1_;
    std::vector<Eigen::ArrayXd> a2_;
    std::vector<Eigen::ArrayXd> b_;

    std::vector<PolygonSearch> polygon_constructors_;

    // A vector with the sample and obstacle indices per scenario
    std::vector<std::vector<Scenario>> scenario_indices_;

    // Constraints constructed from the scenarios
    std::vector<std::vector<ScenarioConstraint>> constraints_;

    // The x, y value of all constraints at a left and right point from the vehicle (for the polygon search)
    std::vector<Eigen::ArrayXd> y_left_;
    std::vector<Eigen::ArrayXd> y_right_;
    std::vector<double> x_left_, x_right_;

    // Obstacle msgs
    std::vector<DynamicObstacle> *obstacles_;
    std::vector<double> radii_;

    // Threads for multithreading per stage
    std::vector<std::thread> scenario_threads_;

    // Active obstacles
    std::vector<std::vector<int>> active_obstacle_indices_;
    std::vector<int> active_obstacle_count_;

    // Vehicle poses when project outside of obstacles
    std::vector<Eigen::Vector2d> projected_poses_;

    // Areas of the regions
    std::vector<double> areas_;

    /**
     * @brief Convert scenarios to constraints
     *
     * @param k time step k
     * @param halfspaces external static half-spaces
     */
    void scenariosToConstraints(int k, const std::vector<Halfspace> &halfspaces);

    /**
     * @brief Remove scenarios based on distance
     *
     * @param k time step k
     */
    void removeScenariosBySorting(int k);

    /**
     * @brief Check feasibility of the scenarios
     *
     * @param k time step k
     */
    void feasibilityCheck(int k);

    /**
     * @brief Projection to obtain a feasible initial plan
     *
     * @param solver_interface solver interface
     */
    void PushAlgorithm(BaseModel *solver_interface);

    /**
     * @brief Getter to get a scenario position
     *
     * @param k time step k
     * @param obstacle_index the obstacle index
     * @param scenario_index the scenario index
     * @return Eigen::Vector3d output position vector in 3D
     */
    Eigen::Vector3d getScenarioLocation(const int &k, const int &obstacle_index, const int &scenario_index);

    // Visualisation methods
    void visualiseConstraints();
    void visualiseScenarios();
    void visualiseRemovedScenarios(const std::vector<int> &indices_to_draw);
    void visualiseSelectedScenarios(const std::vector<int> &indices_to_draw);
    void visualisePolygons();
    void visualisePolygonScenarios();
    void visualiseProjectedPosition();

    // Call to publish all scenario visuals
    void publishVisuals();

  public:
    /**
     * @brief Updates inequality constraints defined by S-MPCC
     *
     * @param solver_interface the solver interface
     * @param dynamic_obstacles dynamic obstacle positions and predictions
     * @param halfspaces static obstacles as linear constraints
     */
    void Update(BaseModel *solver_interface, RealTimeData &data) override;

    /**
     * @brief Visualize the constraints
     */
    void Visualize();

    /**
     * @brief Insert constraints into the solver
     *
     * @param solver_interface the solver interface
     * @param k time step k
     * @param param_idx parameter index (increased internally)
     */
    void SetParameters(BaseModel *solver_interface, const RealTimeData &data, int k, int &param_idx) override;

    /**
     * @brief Use a backup plan when infeasible
     *
     * @param solver_interface the solver interface
     * @param vel the velocity
     * @param deceleration the deceleration
     */
    void useBackupPlan(BaseModel *solver_interface, double vel, double deceleration) override;

    // Get the area of a particular polytope
    double getArea(int k)
    {
      return areas_[k];
    };
  };
};
#endif
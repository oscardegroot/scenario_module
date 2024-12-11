/**
 * @file trajectory_disc.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Code for disc-wise computations of Safe Horizon MPC (SH-MPC)
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef SAFE_HORIZON_H
#define SAFE_HORIZON_H

#include <scenario_module/scenario_base.h>
#include <scenario_module/safety_certifier.h>
#include <scenario_module/polygon_search.h>

#include <mpc_planner_solver/solver_interface.h>

#include <ros_tools/data_saver.h>
#include <ros_tools/ros_visuals.h>

using namespace MPCPlanner;

namespace ScenarioModule
{
  class SafeHorizon : public ScenarioBase
  {
  public:
    SafeHorizon(int disc_id, std::shared_ptr<Solver> solver, Sampler &sampler);

    SafeHorizon(const SafeHorizon &other) = delete;

  public:
    void update(const RealTimeData &data, const ModuleData &module_data);

    /** @note The update computes the constraints for k = 0, ..., N-1 (with predictions 0, ..., N-1)*/
    /** However, k=0 of the optimization is the initial state. We therefore insert k-1 when prompted with k*/
    void setParameters(const RealTimeData &data, int k);

    bool computeActiveConstraints(SupportSubsample &active_constraints_aggregate,
                                  SupportSubsample &infeasible_scenarios) override;

    void Visualize(const RealTimeData &data) override;

  private:
    // Parameters that will be set via config
    u_int S; // sampling count
    u_int N; // Prediction horizon

    int disc_id_;

    // Received from constructor
    bool enable_visualization_; // Visualization is enabled if TRUE

    std::shared_ptr<Solver> _solver;

    bool is_feasible_;

    // A pointer to the scenarios
    std::vector<trajectory_sample> *scenarios_;

    // Intermediate distance computation (reuse for obstacles, but threaded for k)
    std::vector<Eigen::ArrayXd> diffs_x_;
    std::vector<Eigen::ArrayXd> diffs_y_;
    std::vector<Eigen::ArrayXd> distances_;

    // Constraint vectors Ax <= b
    std::vector<Eigen::ArrayXd> a1_;
    std::vector<Eigen::ArrayXd> a2_;
    std::vector<Eigen::ArrayXd> b_;

    // The x, y value of all constraints at a left and right point from the vehicle (for the polygon search)
    std::vector<Eigen::ArrayXd> y_left_, y_right_;
    std::vector<double> x_left_, x_right_;

    // Classes for computing the minimal polygon
    std::vector<PolygonSearch> polytopes_;

    // A vector with the sample and obstacle indices per scenario
    std::vector<std::vector<Scenario>> scenario_indices_;

    // Meta-data of constructed constraints
    std::vector<std::vector<ScenarioConstraint>> constraints_;

    // Joint radii w.r.t. each obstacle (including vehicle radius)
    std::vector<double> radii_;
    double robot_radius_;

    // Vehicle poses used in verification
    std::vector<Eigen::Vector2d> verify_poses_;

    // Old intersects used in removal
    std::vector<std::vector<Eigen::Vector2d *>> old_intersects_;

    // Tracking of infeasible scenarios
    std::vector<bool> distances_feasible_;
    std::vector<std::vector<Eigen::Vector2d>> infeasible_scenario_poses_;
    std::vector<std::vector<int>> infeasible_scenario_idxs_;

    /**
     * @brief Clear data from previous computations
     */
    void clearAll();

    /**
     * @brief Load external data, retrieve scenarios and prepare for the update
     *
     * @param solver_interface
     * @param dynamic_obstacles
     * @param halfspaces
     */
    void LoadData(const RealTimeData &data);

    // COMPUTATIONS FOR EACH K, OBSTACLE
    /**
     * @brief Compute distances to all scenarios
     *
     * @param k time index
     * @param obstacle_id obstacle index
     */
    void computeDistances(const RealTimeData &data, int k, int obstacle_id);

    /**
     * @brief Check feasibility based on computed distances
     *
     * @param k time index
     * @param obstacle_id obstacle index
     */
    void checkFeasibilityByDistance(int k, int obstacle_id);

    /**
     * @brief Use distances, diff_x_ and diff_y_ to compute constraints for all scenarios
     *
     * @param k time index
     * @param obstacle_id obstacle index
     */
    void computeHalfspaces(int k, int obstacle_id);

    /**
     * @brief Construct the polytope given all computed constraints
     *
     * @param k time index
     * @param obstacle_id obstacle index
     */
    void constructPolytopes(int k, const RealTimeData &data, const ModuleData &module_data);

    // PROJECTION SCHEMES
    /** @brief Push the initial plan away from scenarios if infeasible (orthogonal to the vehicle plan) */
    void PushAlgorithm(const RealTimeData &data);

    /** @brief Project the plan to feasibility w.r.t. to the scenario constraints using (Cyclic) Douglas-Rachford Splitting */
    void DRProjection(const RealTimeData &data);

    // Convert samples to vectors for visuals
    Eigen::Vector3d getScenarioLocation(const int &k, const int &obstacle_index, const int &scenario_index);
    Eigen::Vector2d getScenarioLocation2D(const int &k, const int &obstacle_index, const int &scenario_index);

    // Visualisation methods
    void visualiseEllipsoidConstraints() {};

    void visualizeAllConstraints();                            // Visualize all scenario constraints
    void visualiseScenarios(const RealTimeData &data);         // Visualize all scenarios (very slow)
    void visualiseSelectedScenarios(const RealTimeData &data); // Visualize support scenarios
    void visualisePolygonScenarios();                          // Visualize all scenarios that contribute to the final constraints (slow)
    void visualisePolygons();                                  // Visualize the final constraints

    std::vector<int> getDrawIndices();
  };
};
#endif
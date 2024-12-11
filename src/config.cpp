#include <scenario_module/config.h>

#include <mpc_planner_util/parameters.h>

#ifndef MPC_PLANNER_ROS
#include <ros_tools/ros2_wrappers.h>
#endif

namespace ScenarioModule
{

  bool Config::Init()
  {
    if (Initialized())
      return true;

#ifdef MPC_PLANNER_ROS
    ros::NodeHandle node;
#else
    rclcpp::Node *node = GET_STATIC_NODE_POINTER();
#endif

    max_obstacles_ = CONFIG["max_obstacles"].as<int>();
    vehicle_width_ = CONFIG["robot_radius"].as<double>() * 2.;
    draw_every_ = CONFIG["visualization"]["draw_every"].as<int>();

    risk_ = CONFIG["probabilistic"]["risk"].as<double>() * 2.;

    retrieveParameter(node, "scenario_module/safe_sampling/confidence", confidence_);

    retrieveParameter(node, "scenario_module/debug_output", debug_output_, false);

    retrieveParameter(node, "target_frame", target_frame_, std::string("map"));

    retrieveParameter(node, "scenario_module/use_real_samples", use_real_samples_, false);
    retrieveParameter(node, "scenario_module/sample_distribution/binomial_distribution", binomial_distribution_);

    // retrieveParameter(node, "static_obstacles/n_halfspaces", n_halfspaces_, 0);
    n_halfspaces_ = CONFIG["linearized_constraints"]["add_halfspaces"].as<int>();
    static_obstacles_enabled_ = n_halfspaces_ > 0;

    // retrieveParameter(node, "visuals/indices_to_draw", indices_to_draw_, std::vector<int>{});
    // retrieveParameter(node, "visuals/nr_stages_to_draw", nr_stages_to_draw_, 0);
    retrieveParameter(node, "visuals/ellipsoids", draw_ellipsoids_, false);

    // Sample parameters
    retrieveParameter(node, "scenario_module/safe_sampling/compute_automatically", automatically_compute_sample_size_);

    retrieveParameter(node, "scenario_module/safe_sampling/removal_count", removal_count_);
    retrieveParameter(node, "scenario_module/safe_sampling/sample_size", sample_size_);

    retrieveParameter(node, "scenario_module/safe_horizon/sqp_iterations", max_iterations_);
    retrieveParameter(node, "scenario_module/safe_horizon/n_bar", n_bar_);
    retrieveParameter(node, "scenario_module/safe_horizon/terminate_equality_tolerance", terminate_eq_tol_);
    retrieveParameter(node, "scenario_module/safe_horizon/enable_termination", enable_termination_);
    retrieveParameter(node, "scenario_module/scale_reference_velocity", scale_reference_velocity_, false);

    retrieveParameter(node, "scenario_module/sample_distribution/propagate_covariance", propagate_covariance_);

    retrieveParameter(node, "scenario_module/enable_safe_horizon", enable_safe_horizon_);
    retrieveParameter(node, "scenario_module/safe_horizon/use_json_trajectory", use_json_trajectory, false);
    retrieveParameter(node, "scenario_module/safe_horizon/enable_scenario_removal", enable_scenario_removal_);

    retrieveParameter(node, "scenario_module/s_mpcc/database/truncated", truncated_);
    retrieveParameter(node, "scenario_module/s_mpcc/database/truncated_radius", truncated_radius_);
    retrieveParameter(node, "scenario_module/s_mpcc/database/build_database", build_database_);
    retrieveParameter(node, "scenario_module/s_mpcc/database/size", batch_count_);

    retrieveParameter(node, "scenario_module/polygon/range", polygon_range_);
    retrieveParameter(node, "scenario_module/polygon/activation_range", activation_range_);

    retrieveParameter(node, "scenario_module/visualisation/all_scenarios", draw_all_scenarios_);
    retrieveParameter(node, "scenario_module/visualisation/support_scenarios", draw_selected_scenarios_);
    retrieveParameter(node, "scenario_module/visualisation/polygon_scenarios", draw_polygon_scenarios_);
    retrieveParameter(node, "scenario_module/visualisation/removed_scenarios", draw_removed_scenarios_);
    retrieveParameter(node, "scenario_module/visualisation/constraints", draw_constraints_);
    retrieveParameter(node, "scenario_module/visualisation/draw_disc", draw_disc_, 0);
    retrieveParameter(node, "scenario_module/visualisation/scale", visuals_scale_);

    retrieveParameter(node, "scenario_module/s_mpcc/seed", seed_, -1);
    retrieveParameter(node, "scenario_module/s_mpcc/checked_constraints", polygon_checked_constraints_);

    N_ = CONFIG["N"].as<int>();

    success_ = true;
    return true;
  }
};
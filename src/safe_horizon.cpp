#include "scenario_module/safe_horizon.h"

#include <scenario_module/logging.h>

#include <mpc_planner_util/parameters.h>

#include <ros_tools/profiling.h>
#include <ros_tools/visuals.h>

namespace ScenarioModule
{
  SafeHorizon::SafeHorizon(int disc_id, std::shared_ptr<Solver> solver, Sampler &sampler)
      : disc_id_(disc_id), _solver(solver)
  {
    LOG_INITIALIZE("SH-MPC Disc");

    enable_visualization_ = SCENARIO_CONFIG.draw_disc_ == disc_id;

    S = SafetyCertifier::Get().GetSampleSize();
    N = SCENARIO_CONFIG.N_;
    sampler_ = &sampler;

    // Resize over the horizon
    verify_poses_.resize(N); // Active constraints checking
    old_intersects_.resize(N);

    // Keeping track of the location of infeasible scenarios per k
    infeasible_scenario_poses_.resize(N);
    infeasible_scenario_idxs_.resize(N);
    for (size_t k = 0; k < infeasible_scenario_poses_.size(); k++)
    {
      infeasible_scenario_poses_[k].reserve(20);
      infeasible_scenario_idxs_[k].reserve(20);
    }

    robot_radius_ = CONFIG["robot_radius"].as<double>();

    // Dynamic + Range + Static
    int constraint_size = S * SCENARIO_CONFIG.max_obstacles_ + 4 + SCENARIO_CONFIG.n_halfspaces_;

    // Arrays for constraint construction
    diffs_x_.resize(N);
    diffs_y_.resize(N);
    distances_.resize(N);

    a1_.resize(N);
    a2_.resize(N);
    b_.resize(N);
    scenario_indices_.resize(N);

    x_left_.resize(N);
    x_right_.resize(N);

    for (size_t k = 0; k < N; k++)
    {
      diffs_x_[k] = Eigen::ArrayXd(S);
      diffs_y_[k] = Eigen::ArrayXd(S);
      distances_[k] = Eigen::ArrayXd(S);

      a1_[k] = Eigen::ArrayXd(constraint_size);
      a2_[k] = Eigen::ArrayXd(constraint_size);
      b_[k] = Eigen::ArrayXd(constraint_size);

      // Populate the scenario indices
      scenario_indices_[k].resize(constraint_size);

      // Dynamic
      for (int v = 0; v < SCENARIO_CONFIG.max_obstacles_; v++)
      {
        scenario_indices_[k].resize(S);
        for (int s = 0; s < (int)S; s++)
          scenario_indices_[k][v * S + s] = Scenario{s, v};
      }

      // Range / Static
      int index;
      for (int i = 0; i < SCENARIO_CONFIG.n_halfspaces_ + 4; i++)
      {
        index = SCENARIO_CONFIG.max_obstacles_ * S + i;
        scenario_indices_[k][index] = Scenario{index, -1};
      }
    }

    // Reserve space for the constraints and the y evaluations
    constraints_.resize(N);
    y_left_.resize(N);
    y_right_.resize(N);

    for (size_t k = 0; k < N; k++)
    {
      y_left_[k] = Eigen::ArrayXd(constraint_size);
      y_right_[k] = Eigen::ArrayXd(constraint_size);

      // Populate the constraints
      constraints_[k].reserve(constraint_size);

      // Dynamic constraints
      for (int v = 0; v < SCENARIO_CONFIG.max_obstacles_; v++)
      {
        for (size_t s = 0; s < S; s++)
        {
          constraints_[k].emplace_back(&scenario_indices_[k][v * S + s], ObstacleType::DYNAMIC,
                                       ConstraintSide::UNDEFINED);
        }
      }

      // External static constraints
      for (int i = 0; i < SCENARIO_CONFIG.n_halfspaces_; i++)
        constraints_[k].emplace_back(&scenario_indices_[k][SCENARIO_CONFIG.max_obstacles_ * S + i], ObstacleType::STATIC,
                                     ConstraintSide::UNDEFINED);

      // Range constraints
      for (size_t i = 0; i < 4; i++)
        constraints_[k].emplace_back(&scenario_indices_[k][SCENARIO_CONFIG.max_obstacles_ * S + SCENARIO_CONFIG.n_halfspaces_ + i],
                                     ObstacleType::RANGE, ConstraintSide::UNDEFINED);
    }

    // Initialize polygon constructors
    for (u_int k = 0; k < N; k++)
    {
      polytopes_.emplace_back(&constraints_[k], &y_left_[k], &y_right_[k], constraint_size, SCENARIO_CONFIG.polygon_range_);
    }

    // Variable initialisation
    radii_.resize(SCENARIO_CONFIG.max_obstacles_);

    // Support subsample (size arguments are not critical)
    support_subsample_.support_indices_.resize(50);
    support_subsample_.scenarios_.resize(50);

    LOG_INITIALIZED();
  }

  void SafeHorizon::LoadData(const RealTimeData &data)
  {
    SCENARIO_INFO("Safe Horizon: LoadData()");

    clearAll(); // Clear data from previous runs

    status_ = ScenarioStatus::SUCCESS;
    is_feasible_ = true;

    SCENARIO_INFO("Safe Horizon: Retrieving scenarios");
    scenarios_ = sampler_->GetSamples();

    // Store the radii of all obstacles
    for (size_t v = 0; v < data.dynamic_obstacles.size(); v++)
      radii_[v] = data.dynamic_obstacles[v].radius + robot_radius_;

    SCENARIO_INFO("Safe Horizon: Projecting");
    PushAlgorithm(data); // Orthogonal push
    DRProjection(data);  // Douglas-Rachford splitting

    SCENARIO_INFO("Safe Horizon: Checking feasibility");
    for (size_t k = 0; k < N; k++)
    {
      for (int obst_id = 0; obst_id < SCENARIO_CONFIG.max_obstacles_; obst_id++) // For all obstacles
      {
        computeDistances(data, k, obst_id);     // Computes for the scenarios of this obstacle the distances to the vehicle
        checkFeasibilityByDistance(k, obst_id); // We can then check if the scenarios are all feasible
      }
    }

    SCENARIO_INFO("Safe Horizon: Data Loaded");
  }

  // Todo: dynamic_obstacles should just be a pointer
  void SafeHorizon::update(const RealTimeData &data, const ModuleData &module_data)

  {
    PROFILE_SCOPE("Safe Horizon Update");
    SCENARIO_INFO("Safe Horizon: Update");

    LoadData(data);

    SCENARIO_INFO("Constructing polytopes");
    for (size_t k = 0; k < N; k++)
    {
      for (int obst_id = 0; obst_id < SCENARIO_CONFIG.max_obstacles_; obst_id++) // For all obstacles
      {
        computeDistances(data, k, obst_id);     // Computes for the scenarios of this obstacle the distances to the vehicle
        checkFeasibilityByDistance(k, obst_id); // We can then check if the scenarios are all feasible
        computeHalfspaces(k, obst_id);          // Then, we construct halfspaces for all scenarios
      }

      // Finally we intersect all constraints for all obstacles to obtain a safe polytope
      constructPolytopes(k, data, module_data);
    }

    if (!is_feasible_)
      status_ = ScenarioStatus::INFEASIBLE;

    SCENARIO_INFO_STREAM("Safe Horizon: Update Done! Status: " << (int)status_);
  }

  // Clear data from previous iterations
  void SafeHorizon::clearAll()
  {
    SCENARIO_INFO("Safe Horizon: Clear All");
    for (size_t k = 0; k < N; k++)
      old_intersects_[k].clear();

    support_subsample_.Reset();

    // Reset polygon computations
    for (size_t k = 0; k < N; k++)
      polytopes_[k].Reset();

    // Feasibility variables
    distances_feasible_ = std::vector<bool>(N, true);

    for (size_t k = 0; k < N; k++)
    {
      infeasible_scenario_poses_[k].clear();
      infeasible_scenario_idxs_[k].clear();
    }
  }

  void SafeHorizon::computeDistances(const RealTimeData &data, int k, int obstacle_id)
  {
    Eigen::Vector2d pose = data.robot_area[disc_id_].getPosition(
        _solver->getEgoPredictionPosition(k + 1),
        _solver->getEgoPrediction(k + 1, "psi"));

    diffs_x_[k] = (*scenarios_)[k][obstacle_id][0].array() - pose(0); // Note that the samples are already in a data structure that is fast to handle here
    diffs_y_[k] = (*scenarios_)[k][obstacle_id][1].array() - pose(1);

    distances_[k] = (diffs_x_[k] * diffs_x_[k] + diffs_y_[k] * diffs_y_[k]).sqrt();
  }

  void SafeHorizon::checkFeasibilityByDistance(int k, int obstacle_id)
  {
    // Check for all samples if they are feasible
    for (int s = 0; s < distances_[k].size(); s++)
    {
      if (distances_[k](s) < radii_[obstacle_id] - 1e-3) // If distance to scenario < r, then infeasible
      {
        infeasible_scenario_poses_[k].push_back(
            Eigen::Vector2d((*scenarios_)[k][obstacle_id][0](s), (*scenarios_)[k][obstacle_id][1](s)));

        infeasible_scenario_idxs_[k].push_back(obstacle_id * S + s);

        distances_feasible_[k] = false;
        is_feasible_ = false;
      }
    }
  }

  void SafeHorizon::computeHalfspaces(int k, int obstacle_id)
  {
    // Compute the components of A for this obstacle (normalized normal vector)
    a1_[k].block(obstacle_id * S, 0, S, 1) = diffs_x_[k] / distances_[k];
    a2_[k].block(obstacle_id * S, 0, S, 1) = diffs_y_[k] / distances_[k];

    // Compute b (evaluate point on the collision circle)
    b_[k].block(obstacle_id * S, 0, S, 1) =
        a1_[k].block(obstacle_id * S, 0, S, 1) * (*scenarios_)[k][obstacle_id][0].array() +
        a2_[k].block(obstacle_id * S, 0, S, 1) * (*scenarios_)[k][obstacle_id][1].array() - radii_[obstacle_id];
  }

  void SafeHorizon::constructPolytopes(int k, const RealTimeData &data, const ModuleData &module_data)
  {
    // Add range constraints
    const Eigen::Vector2d center_position = _solver->getEgoPredictionPosition(k + 1);
    double angle = _solver->getEgoPrediction(k + 1, "psi");
    const Eigen::Vector2d position = data.robot_area[disc_id_].getPosition(center_position, angle);

    polytopes_[k].AddRangeConstraints(position, angle, a1_[k], a2_[k], b_[k]);

    // Add external halfspace constraints
    double a0_ext, a1_ext, b_ext;
    for (int i = 0; i < SCENARIO_CONFIG.n_halfspaces_; i++)
    {
      // Guard against 0 constraints by slightly altering the constraints
      a0_ext = module_data.static_obstacles[k][i].A(0);
      if (std::abs(a0_ext) < 1e-4)
        a0_ext = 1e-4;

      a1_ext = module_data.static_obstacles[k][i].A(1);
      if (std::abs(a1_ext) < 1e-4)
        a1_ext = 1e-4;

      b_ext = module_data.static_obstacles[k][i].b;

      a1_[k](SCENARIO_CONFIG.max_obstacles_ * S + i) = a0_ext;
      a2_[k](SCENARIO_CONFIG.max_obstacles_ * S + i) = a1_ext;
      b_[k](SCENARIO_CONFIG.max_obstacles_ * S + i) = b_ext;
    }

    // Compute the extreme search x's for each vehicle position
    x_left_[k] = position(0) - 1e-8 - std::sqrt(2) * SCENARIO_CONFIG.polygon_range_;
    x_right_[k] = position(0) + 1e-8 + std::sqrt(2) * SCENARIO_CONFIG.polygon_range_;

    // Compute all y at the left and the right for this k
    y_left_[k] = (b_[k] - a1_[k] * x_left_[k]) / a2_[k];
    y_right_[k] = (b_[k] - a1_[k] * x_right_[k]) / a2_[k];

    // Assign the sides of the constraints based on a2
    for (size_t c = 0; c < constraints_[k].size(); c++)
      constraints_[k][c].side_ = a2_[k](c) > 0 ? ConstraintSide::TOP : ConstraintSide::BOTTOM;

    // Construct the polytopes
    bool success;
    success = polytopes_[k].Search(position, angle, x_left_[k], x_right_[k]);

    if (!success)
      polygon_failed_ = true;

    if (k == 0)
      SCENARIO_INFO("Constructed polytopes (first polytope has " << polytopes_[k].polygon_out_.size() << " constraints)");

    // Polytopes are stored in polytopes_[k]
  }

  void SafeHorizon::DRProjection(const RealTimeData &data)
  {
    int iterates = 5;

    for (size_t k = 0; k < SCENARIO_CONFIG.N_; k++) // For each k
    {

      auto &disc = data.robot_area[disc_id_];

      Eigen::Vector2d center_pose = _solver->getEgoPredictionPosition(k + 1);
      double angle = _solver->getEgoPrediction(k + 1, "psi");

      Eigen::Vector2d pose = disc.getPosition(center_pose, angle); // Translated disc position

      Eigen::Vector2d start_pose = pose;
      if (k > 0)
      {
        start_pose = disc.getPosition(
            _solver->getEgoPredictionPosition(k),
            _solver->getEgoPrediction(k, "psi")); // Start pose refers to the previous pose
      }

      Eigen::Vector2d prev_pose = pose;

      for (int iterate = 0; iterate < iterates; iterate++) // At most iterates iterations
      {
        Eigen::Vector2d anchor((*scenarios_)[k][0][0](0),
                               (*scenarios_)[k][0][1](0)); // Iterations are anchored at some random constraint

        for (int obstacle_id = 0; obstacle_id < SCENARIO_CONFIG.max_obstacles_; obstacle_id++) // For all obstacles
        {
          double r = radii_[obstacle_id] + 1e-3; // Add some margin to the projection radius

          for (size_t s = 0; s < S; s++)
          {
            if (s == 0 && obstacle_id == 0) // The first constraint is the anchor
              continue;

            // Current constraint
            Eigen::Vector2d delta((*scenarios_)[k][obstacle_id][0](s), (*scenarios_)[k][obstacle_id][1](s));
            Eigen::Vector2d update_pose = pose;

            // Reflection Operator on the anchor
            if (std::sqrt((update_pose - anchor).transpose() * (update_pose - anchor)) < r)
              update_pose =
                  2.0 * (anchor - (anchor - update_pose) /
                                      (std::sqrt((update_pose - anchor).transpose() * (update_pose - anchor))) * r) -
                  update_pose;

            // Reflection Operator on the current constraint set
            if (std::sqrt((update_pose - delta).transpose() * (update_pose - delta)) < r)
              update_pose = 2.0 * (delta - (delta - start_pose) /
                                               (std::sqrt((start_pose - delta).transpose() * (start_pose - delta))) * r) -
                            update_pose;

            // Douglas rachford operator
            pose = (pose + update_pose) / 2.0;
          }
        }

        // STATIC CONSTRAINTS (works but may not always converge to a dynamically feasible position)
        // for (auto &halfspace : halfspaces[k].halfspaces)
        // {
        //     double r = radii_[0] + 1e-3; // Add some margin to the projection radius

        //     // Reflection Operator on the anchor
        //     Eigen::Vector2d update_pose = pose;
        //     if (std::sqrt((update_pose - anchor).transpose() * (update_pose - anchor)) < r)
        //         update_pose = 2.0 * (anchor - (anchor - update_pose) / (std::sqrt((update_pose - anchor).transpose() *
        //         (update_pose - anchor))) * r) - update_pose;

        //     // Reflection Operator on a halfspace constraint (projection/reflection onto a line)
        //     if (halfspace.A[0] * pose(0) + halfspace.A[1] * pose(1) > halfspace.b)
        //     {
        //         Eigen::Vector2d Ah(halfspace.A[0], halfspace.A[1]);
        //         update_pose = 2.0 * (update_pose + Ah * (-Ah.transpose() * update_pose - halfspace.b)) - update_pose;
        //     }

        //     // Douglas rachford operator
        //     pose = (pose + update_pose) / 2.0;
        // }
        // Stop if the change in position is close to zero
        if (RosTools::distance(prev_pose, pose) < 1e-5)
          break;

        if (iterate == iterates - 1)
        {
          LOG_ERROR("Reached max iterations in DR project.");
        }

        prev_pose = pose;
      }
      _solver->setEgoPredictionPosition(k + 1, disc.toRobotCenter(pose, angle));
    }
  }

  void SafeHorizon::PushAlgorithm(const RealTimeData &data) // std::vector<VehicleRegion> &plan)
  {
    PROFILE_FUNCTION();

    for (size_t k = 0; k < N; k++) // For all k
    {

      auto &disc = data.robot_area[disc_id_];

      // Get the vehicle position and angle at k
      Eigen::Vector2d center_pose = _solver->getEgoPredictionPosition(k + 1);
      double angle = _solver->getEgoPrediction(k + 1, "psi");

      Eigen::Vector2d pose = disc.getPosition(center_pose, angle); // Translated disc position

      for (int obstacle_id = 0; obstacle_id < SCENARIO_CONFIG.max_obstacles_; obstacle_id++) // For each obstacle
      {
        Eigen::Vector2d vector_sum(0., 0.);

        // Compute a weighted direction from the scenarios, pointing outwards
        for (u_int s = 0; s < S; s += 5) // We do not really need to add every sample
        {
          Eigen::Vector2d scenario_pose((*scenarios_)[k][obstacle_id][0](s), (*scenarios_)[k][obstacle_id][1](s));
          vector_sum += pose - scenario_pose;
        }

        vector_sum.normalize();

        // Compute orthogonal vectors and check which one lines up with the vector sum better
        Eigen::Vector2d orientation_vec(std::cos(angle), std::sin(angle));
        Eigen::Vector2d orientation_orth1(-orientation_vec(1), orientation_vec(0));
        Eigen::Vector2d orientation_orth2(orientation_vec(1), -orientation_vec(0));

        double val_1 = std::abs(std::acos(vector_sum.transpose() * orientation_orth1));
        double val_2 = std::abs(std::acos(vector_sum.transpose() * orientation_orth2));
        Eigen::Vector2d orth_vec = val_1 <= val_2 ? orientation_orth1 : orientation_orth2;

        vector_sum = orth_vec; // That one is our push direction

        // Multiply this vector with the distances, i.e., V^T d
        Eigen::VectorXd distances = (((*scenarios_)[k][obstacle_id][0].array() - pose(0)).square() + ((*scenarios_)[k][obstacle_id][1].array() - pose(1)).square()).sqrt();
        Eigen::VectorXd d_vec = vector_sum(0) * ((*scenarios_)[k][obstacle_id][0].array() - pose(0)) + vector_sum(1) * ((*scenarios_)[k][obstacle_id][1].array() - pose(1));

        // Keep only distances that are smaller than r
        for (size_t s = 0; s < S; s++)
        {
          if (distances(s) >= radii_[obstacle_id])
            d_vec(s) = -20.0;
        }

        // Find the worst
        double max_d_vec = d_vec.maxCoeff();

        if (max_d_vec + radii_[obstacle_id] > 0.)
          pose += vector_sum * (radii_[obstacle_id] + max_d_vec); // Add a push to the pose of this disc
      }

      // Set the robot position (translated from the projected disc position)
      _solver->setEgoPredictionPosition(k + 1, disc.toRobotCenter(pose, angle));
    }
  }

  // Check for the x's at all k if Ax = b
  bool SafeHorizon::computeActiveConstraints(SupportSubsample &active_constraints_aggregate,
                                             SupportSubsample &infeasible_scenarios)
  {
    // Compute a vector of the values
    bool feasible = true;
    int infeasible_count = 0;

    for (size_t k = 0; k < N - 1; k++)
    {
      // infeasible_count = 0;
      double slack = _solver->getOutput(k + 1, "slack"); // Note: slack should be the same for all k
      // For each constraints in the polygon
      for (auto &constraint : polytopes_[k].polygon_out_)
      {
        // Obtain the absolute scenario index
        int index = constraint->GetHalfspaceIndex(S);

        // Evaluate the constraint (Ax - b (<= slack))
        double constraint_value = a1_[k](index) * _solver->getOutput(k + 1, "x") +
                                  a2_[k](index) * _solver->getOutput(k + 1, "y") -
                                  (b_[k](index) + slack);

        // std::cout << constraint_value << std::endl;
        if (constraint_value > 1e-3) // If it is "Infeasible"
        {
          // Mark the result as infeasible
          infeasible_count++;
          if (constraint->type_ == ObstacleType::DYNAMIC)
            infeasible_scenarios.Add(*constraint->scenario_);

          feasible = false;
          // LOG_MARK("Infeasible Constraint (Value = " << constraint_value << ", index = " << constraint->scenario_->idx_ << ")");

          // Behavior: If they are dynamic constraints, add infeasible constraints to the support to mark them for removal
          if (constraint->type_ == ObstacleType::DYNAMIC)
            active_constraints_aggregate.Add(*constraint->scenario_);
        }
        else if (constraint->type_ == ObstacleType::DYNAMIC && constraint_value > -1e-7) // If it is "active"
        {
          // Mark the constraint as "active"
          active_constraints_aggregate.Add(*constraint->scenario_);
        }
      }
    }

    return feasible;
  }

  void SafeHorizon::setParameters(const RealTimeData &data, int k)
  {
    if (k == 0)
      return;

    // Insert all active constraints in this stage
    for (int l = 0; l < std::min(24, (int)polytopes_[k - 1].polygon_out_.size()); l++)
    {
      ScenarioConstraint *&scenario_constraint = polytopes_[k - 1].polygon_out_[l];
      int cur_index = scenario_constraint->GetHalfspaceIndex(S);
      // std::cout << a1_[k](cur_index) << ", " << a2_[k](cur_index) << ", " << b_[k](cur_index) << std::endl;
      _solver->setParameter(k, "disc_0_scenario_constraint_" + std::to_string(l) + "_a1", a1_[k - 1](cur_index));
      _solver->setParameter(k, "disc_0_scenario_constraint_" + std::to_string(l) + "_a2", a2_[k - 1](cur_index));
      _solver->setParameter(k, "disc_0_scenario_constraint_" + std::to_string(l) + "_b", b_[k - 1](cur_index));
    }

    // Insert dummies on other spots (relative to vehicle position)
    for (int l = polytopes_[k - 1].polygon_out_.size(); l < 20 + 4; l++)
    {
      _solver->setParameter(k, "disc_0_scenario_constraint_" + std::to_string(l) + "_a1", 1.);
      _solver->setParameter(k, "disc_0_scenario_constraint_" + std::to_string(l) + "_a2", 0.);
      _solver->setParameter(k, "disc_0_scenario_constraint_" + std::to_string(l) + "_b", _solver->getEgoPrediction(0, "x") + 100.);
    }
  }

  //-------------------------------- VISUALIZATION -----------------------------------//
  void SafeHorizon::Visualize(const RealTimeData &data)
  {
    if (enable_visualization_)
    {
      PROFILE_SCOPE("Visualization");

      // Visualise scenarios
      if (SCENARIO_CONFIG.draw_all_scenarios_)
        visualiseScenarios(data);

      if (SCENARIO_CONFIG.draw_selected_scenarios_)
        visualiseSelectedScenarios(data);

      if (SCENARIO_CONFIG.draw_polygon_scenarios_)
        visualisePolygonScenarios();

      if (SCENARIO_CONFIG.draw_constraints_)
        visualisePolygons();
    }
  }

  // Visualise the predictions
  void SafeHorizon::visualiseScenarios(const RealTimeData &data)
  {
    SCENARIO_INFO("SH-MPC: Visualizing All Scenarios");
    bool draw_line = true;
    bool draw_circle = true;
    bool draw_points = false;

    double alpha = 1.0 / SCENARIO_CONFIG.sample_size_ * 2.0;

    // auto &scenario_visuals = VISUALS.getPublisher("scenario_constraints/" + std::to_string(_solver->_solver_id) + "_scenarios");
    auto &scenario_visuals = VISUALS.getPublisher("scenario_constraints/scenarios");
    RosTools::ROSMultiplePointMarker &scenario_points = scenario_visuals.getNewMultiplePointMarker("POINTS");
    scenario_points.setScale(0.05, 0.05, 0.05);

    RosTools::ROSPointMarker &scenario_circles = scenario_visuals.getNewPointMarker("CYLINDER");

    RosTools::ROSLine &selected_scenario_trajectories = scenario_visuals.getNewLine();
    selected_scenario_trajectories.setScale(0.15, 0.1);

    Eigen::Vector3d point_prev, point_cur;
    for (size_t v = 0; v < data.dynamic_obstacles.size(); v++) // For all obstacles
    {
      auto &obs = data.dynamic_obstacles[v];
      for (size_t s = 0; s < S; s++) // For all samples
      {
        for (uint k = 0; k < N; k += SCENARIO_CONFIG.draw_every_) // For all drawn indices
        {
          point_cur = getScenarioLocation(k, v, s);
          point_cur(2) = -0.1;

          if (draw_points)
          {
            scenario_points.setColorInt(k, N, 1.0);
            scenario_points.addPointMarker(point_cur); // Add a point at the scenario
          }
          if (draw_circle)
          {
            scenario_circles.setColorInt(k, N, alpha);
            scenario_circles.setScale(2 * (obs.radius),
                                      2 * (obs.radius), 0.01);
            scenario_circles.addPointMarker(point_cur); // Add a circle with the collision radius
          }
          if (draw_line && k != 0)
          {
            selected_scenario_trajectories.setColorInt(k, N, 1.0);
            selected_scenario_trajectories.addLine(point_prev, point_cur);
          }
          point_prev = point_cur;
        }
      }
    }

    if (draw_points)
      scenario_points.finishPoints();

    scenario_visuals.publish();
  }

  void SafeHorizon::visualiseSelectedScenarios(const RealTimeData &data)
  {
    bool draw_circles = true;

    SCENARIO_INFO("SH-MPC: Visualizing Selected Scenarios");

    auto &scenario_visuals = VISUALS.getPublisher("scenario_constraints/support");

    RosTools::ROSPointMarker &scenario_points = scenario_visuals.getNewPointMarker("CYLINDER");
    scenario_points.setScale(0.15, 0.15, 0.1e-3);

    RosTools::ROSPointMarker &scenario_circles = scenario_visuals.getNewPointMarker("CYLINDER");

    RosTools::ROSLine &selected_scenario_trajectories = scenario_visuals.getNewLine();
    selected_scenario_trajectories.setScale(0.07 * SCENARIO_CONFIG.visuals_scale_,
                                            0.07 * SCENARIO_CONFIG.visuals_scale_);

    Eigen::Vector3d scenario_location, prev_location;
    std::vector<int> draw_indices = getDrawIndices();

    // Plot support scenarios (and removed scenarios with a different color)
    // For all scenarios of support -> Plots support subsample, which includes removed scenarios!
    for (Scenario &scenario : support_subsample_.scenarios_)
    {

      // To plot the whole obstacle, we need to find all the indices that belong to the same obstacle
      const DynamicObstacle *scenario_obstacle = nullptr;

      int disc_id = 0;
      int obs_id = 0;

      // For each obstacle
      for (auto &obstacle : data.dynamic_obstacles)
      {

        int temp_obs_id = obs_id; // This is the index starting with this obstacle
        if (scenario.obstacle_idx_ == obstacle.index)
        {
          scenario_obstacle = &obstacle;
        }
        disc_id++;
        temp_obs_id++;

        if (scenario_obstacle)
          break;

        obs_id = temp_obs_id; // If it wasn't our obstacle, keep counting onwards
      }

      if (scenario_obstacle == nullptr)
        continue;

      for (int k : draw_indices)
      {

        selected_scenario_trajectories.setColorInt(k, (int)N);

        // Retrieve the scenario location
        scenario_location = getScenarioLocation(k, scenario.obstacle_idx_, scenario.idx_);
        scenario_location(2) = -((double)k) * 0.1e-3;

        // Draw a broken line
        if (k > 0)
        {
          // selected_scenario_trajectories.addBrokenLine(prev_location, scenario_location, 0.2);
          selected_scenario_trajectories.addLine(prev_location, scenario_location);
        }
        prev_location = scenario_location;

        // Draw a circle for each disc of the obstacle where the scenario is of support
        scenario_location = getScenarioLocation(k, scenario_obstacle->index, scenario.idx_);
        scenario_location(2) = -((double)k) * 0.1e-3;

        scenario_circles.setScale(2 * (radii_[scenario.obstacle_idx_] - robot_radius_),
                                  2 * (radii_[scenario.obstacle_idx_] - robot_radius_), 0.1);

        if (draw_circles)
        {
          scenario_circles.setColorInt(k, N, 0.4);
          scenario_points.setColorInt(k, N, 1.0);

          scenario_circles.addPointMarker(scenario_location);
          scenario_points.addPointMarker(scenario_location);
        }
      }
    }
    scenario_visuals.publish();
  }

  std::vector<int> SafeHorizon::getDrawIndices()
  {
    std::vector<int> draw_indices;
    for (size_t k = 0; k < N - 1; k += SCENARIO_CONFIG.draw_every_)
      draw_indices.emplace_back(k);
    if (draw_indices.back() != N - 2) // Ensure that the last polygon is drawn
      draw_indices.emplace_back(N - 2);

    return draw_indices;
  }

  // Visualizes scenarios that make up the polygon, not necessarily of support
  void SafeHorizon::visualisePolygonScenarios()
  {
    SCENARIO_INFO("SH-MPC: Visualizing Polygon Scenarios");

    bool plot_line = true;
    bool plot_positions = false;

    auto &scenario_visuals = VISUALS.getPublisher("scenario_constraints/polygon_scenarios");

    std::vector<int> draw_indices = getDrawIndices();

    std::vector<RosTools::ROSMultiplePointMarker *> polygon_positions; // Vector necessary to maintain coloring per k
    if (plot_positions)
    {
      int ii = 0;
      for (auto &draw_index : draw_indices)
      {
        polygon_positions.push_back(&scenario_visuals.getNewMultiplePointMarker("SPHERE"));
        polygon_positions[ii]->setScale(0.1, 0.1, 0.01);
        ii++;
      }
    }

    RosTools::ROSLine &polygon_scenario_trajectories = scenario_visuals.getNewLine();
    polygon_scenario_trajectories.setScale(0.05 * SCENARIO_CONFIG.visuals_scale_, 0.1);

    Eigen::Vector3d scenario_location, prev_location;

    SupportSubsample polygon_constraints;

    // Aggregate constraints along the horizon
    for (size_t k = 0; k < N; k++)
    {
      // Get the polygon for this stage
      std::vector<ScenarioConstraint *> &constraints_ = polytopes_[k].polygon_out_;

      for (size_t i = 0; i < constraints_.size(); i++)
      {
        if (constraints_[i]->type_ == ObstacleType::DYNAMIC)
          polygon_constraints.Add(*constraints_[i]->scenario_);
      }
    }

    // For all scenarios of support -> Plots support subsample, which includes removed scenarios!
    for (int i = 0; i < polygon_constraints.size_; i++)
    {
      int iii = 0;
      for (int k : draw_indices)
      {

        scenario_location = getScenarioLocation(k,
                                                polygon_constraints.scenarios_[i].obstacle_idx_,
                                                polygon_constraints.scenarios_[i].idx_);
        scenario_location(2) = -((double)k) * 0.1;

        // Draw the point
        if (plot_positions)
        {
          polygon_positions[iii]->setColorInt(k, N, 0.5);
          polygon_positions[iii]->addPointMarker(scenario_location);
        }

        // Draw a line
        if (k > 0 && plot_line)
        {
          polygon_scenario_trajectories.setColorInt(k, N, 0.3);
          polygon_scenario_trajectories.addBrokenLine(prev_location, scenario_location, 0.2);
        }

        prev_location = scenario_location;
      }
    }

    if (plot_positions)
    {
      for (auto &marker : polygon_positions)
        marker->finishPoints();
    }

    scenario_visuals.publish();
  }

  void SafeHorizon::visualisePolygons()
  {
    SCENARIO_INFO("SH-MPC: Visualizing Polygons");

    auto &scenario_visuals = VISUALS.getPublisher("scenario_constraints/polygons");

    RosTools::ROSLine &line = scenario_visuals.getNewLine();
    line.setScale(0.1 * SCENARIO_CONFIG.visuals_scale_, 0.1 * SCENARIO_CONFIG.visuals_scale_);

    geometry_msgs::Point p1, p2;
    p1.z = 0.3e-3;
    p2.z = 0.3e-3;

    bool visualize_points = false;
    RosTools::ROSPointMarker &intersections = scenario_visuals.getNewPointMarker("CYLINDER");
    intersections.setScale(0.1 * SCENARIO_CONFIG.visuals_scale_, 0.1 * SCENARIO_CONFIG.visuals_scale_, 1e-3);

    std::vector<int> draw_indices = getDrawIndices();
    for (int k : draw_indices)
    {
      line.setColorInt(k, (int)N);
      intersections.setColorInt(k, (int)N);

      p1.z = ((double)k) * 0.2e-2; // Above all other scenario visuals
      p2.z = ((double)k) * 0.2e-2;

      std::vector<Eigen::Vector2d *> &intersects = polytopes_[k].intersects_out_;

      for (size_t i = 0; i < intersects.size(); i++)
      {
        // Draw lines between consecutive intersections
        if (i == 0)
        {
          p1.x = (*intersects[intersects.size() - 1])(0);
          p1.y = (*intersects[intersects.size() - 1])(1);
        }
        else
        {
          p1.x = (*intersects[i - 1])(0);
          p1.y = (*intersects[i - 1])(1);
        }

        p2.x = (*intersects[i])(0);
        p2.y = (*intersects[i])(1);

        line.addLine(p1, p2);

        if (visualize_points) // If true, draw the corners as well
        {
          intersections.addPointMarker(p1);
        }
      }
    }
    scenario_visuals.publish();

    // visualizeAllConstraints();
  }

  // Debug functionality
  void SafeHorizon::visualizeAllConstraints()
  {
    SCENARIO_INFO("SH-MPC: Visualizing All Constraints");

    auto &scenario_visuals = VISUALS.getPublisher("scenario_constraints/constraints");

    RosTools::ROSLine &line = scenario_visuals.getNewLine();
    geometry_msgs::Point p1, p2;
    p1.z = 0.5;
    p2.z = 0.5; // 2D points

    int k = 19; // draw only for this k
    bool plot_all = false;
    bool plot_top_bottom = true;
    bool plot_intersects = true;

    if (plot_all)
    {
      for (size_t i = 0; i < constraints_[k].size(); i++)
      {
        p1.x = x_left_[k];
        p1.y = y_left_[k](i);

        p2.x = x_right_[k];
        p2.y = y_right_[k](i);

        line.addLine(p1, p2);
      }
    }

    if (plot_top_bottom)
    {
      line.setColor((double)1.0 / 3.0 / 4.0);
      for (size_t i = 0; i < polytopes_[k].result_indices_bot_.size(); i++)
      {
        // Index of a selected line
        int index = polytopes_[k].result_indices_bot_[i];

        p1.x = x_left_[k];
        p1.y = y_left_[k](index);

        p2.x = x_right_[k];
        p2.y = y_right_[k](index);

        line.addLine(p1, p2);
      }

      line.setColor((double)2.0 / 3.0 / 4.0);

      for (size_t i = 0; i < polytopes_[k].result_indices_top_.size(); i++)
      {
        // Index of a selected line
        int index = polytopes_[k].result_indices_top_[i];

        p1.x = x_left_[k];
        p1.y = y_left_[k](index);

        p2.x = x_right_[k];
        p2.y = y_right_[k](index);

        line.addLine(p1, p2);
      }
    }

    if (plot_intersects)
    {
      RosTools::ROSPointMarker &intersections = scenario_visuals.getNewPointMarker("CUBE");
      intersections.setScale(0.4, 0.4, 0.1);
      intersections.setColor(1, 0, 0);
      for (size_t i = 0; i < polytopes_[k].result_intersects_bot_.size(); i++)
      {
        // Index of a selected line
        p1.x = polytopes_[k].result_intersects_bot_[i](0);
        p1.y = polytopes_[k].result_intersects_bot_[i](1);

        intersections.addPointMarker(p1);
      }
      for (size_t i = 0; i < polytopes_[k].result_intersects_top_.size(); i++)
      {
        // Index of a selected line
        p1.x = polytopes_[k].result_intersects_top_[i](0);
        p1.y = polytopes_[k].result_intersects_top_[i](1);

        intersections.addPointMarker(p1);
      }
    }
    scenario_visuals.publish();
  }

  Eigen::Vector3d SafeHorizon::getScenarioLocation(const int &k, const int &obstacle_index, const int &scenario_index)
  {
    Eigen::Vector2d loc((*scenarios_)[k][obstacle_index][0](scenario_index),
                        (*scenarios_)[k][obstacle_index][1](scenario_index));
    return Eigen::Vector3d(loc(0), loc(1), 0.2);
  }

  Eigen::Vector2d SafeHorizon::getScenarioLocation2D(const int &k, const int &obstacle_index,
                                                     const int &scenario_index)
  {
    return Eigen::Vector2d((*scenarios_)[k][obstacle_index][0](scenario_index),
                           (*scenarios_)[k][obstacle_index][1](scenario_index));
  }

};
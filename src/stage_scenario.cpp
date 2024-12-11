#include "scenario/stage_scenario.h"

namespace ScenarioModule
{

  StageScenario::StageScenario(ros::NodeHandle &nh, ConfigurationMPC *config, const VehicleDisc &vehicle_disc,
                               GaussianSampler *sampler, bool enable_visualization = false, int solver_id = 0)
      : config_(config), enable_visualization_(enable_visualization)
  {
    SCENARIO_INFO("Initializing Scenario Manager");

    disc_.reset(new VehicleDisc(vehicle_disc));

    // Initialise the visualisation
    ros_markers_.reset(
        new ROSMarkerPublisher(nh, (std::string("scenario_constraints/markers_") + std::to_string(solver_id)).c_str(),
                               config_->target_frame_, 1800));

    // Save some useful variables
    sampler_ = sampler;
    S = sampler_->SampleSize();
    N = config_->N_;

    R_ = config_->removal_count_;
    l_ = config_->polygon_checked_constraints_;

    // Resize over the horizon
    poses_.resize(N);
    orientations_.resize(N);
    projected_poses_.resize(N);
    scenario_threads_.resize(N);

    int constraint_size = config_->max_obstacles_ * S + config_->n_halfspaces_ + 4;

    // Indices used for finding the closest scenarios to remove
    sort_indices_.resize(S * config_->max_obstacles_);

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
      // Cosntraint size!
      diffs_x_[k] = Eigen::ArrayXd(S);
      diffs_y_[k] = Eigen::ArrayXd(S);
      distances_[k] = Eigen::ArrayXd(S * config_->max_obstacles_); // * max obstacles gives much more freedom

      // Constraint variables
      a1_[k] = Eigen::ArrayXd(constraint_size);
      b_[k] = Eigen::ArrayXd(constraint_size);
      a2_[k] = Eigen::ArrayXd(constraint_size);

      // Populate the scenario indices
      scenario_indices_[k].resize(constraint_size);

      // Dynamic
      for (int v = 0; v < config_->max_obstacles_; v++)
      {
        scenario_indices_[k].resize(S);
        for (int s = 0; s < (int)S; s++)
          scenario_indices_[k][v * S + s] = Scenario{s, v};
      }

      // Range / Static
      int index;
      for (int i = 0; i < config_->n_halfspaces_ + 4; i++)
      {
        index = config_->max_obstacles_ * S + i;
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
      for (int v = 0; v < config_->max_obstacles_; v++)
      {
        for (size_t s = 0; s < S; s++)
        {
          constraints_[k].emplace_back(&scenario_indices_[k][v * S + s], ObstacleType::DYNAMIC,
                                       ConstraintSide::UNDEFINED);
        }
      }

      // External static constraints
      for (int i = 0; i < config_->n_halfspaces_; i++)
        constraints_[k].emplace_back(&scenario_indices_[k][config_->max_obstacles_ * S + i], ObstacleType::STATIC,
                                     ConstraintSide::UNDEFINED);

      // Range constraints
      for (size_t i = 0; i < 4; i++)
        constraints_[k].emplace_back(&scenario_indices_[k][config_->max_obstacles_ * S + config_->n_halfspaces_ + i],
                                     ObstacleType::RANGE, ConstraintSide::UNDEFINED);
    }

    // Initialize polygon constructors
    for (u_int k = 0; k < N; k++)
      polygon_constructors_.emplace_back(&constraints_[k], &y_left_[k], &y_right_[k], constraint_size,
                                         config_->polygon_range_);

    active_obstacle_count_.resize(N, 0);
    active_obstacle_indices_.resize(N);
    for (uint k = 0; k < N; k++)
      active_obstacle_indices_[k].resize(config_->max_obstacles_);

    areas_.resize(N);
    for (u_int k = 0; k < N; k++)
      areas_[k] = 0.0;

    radii_.resize(config_->max_obstacles_);

    SCENARIO_INFO("\tDone");
  }

  void StageScenario::Update(BaseModel *solver_interface, RealTimeData &data)
  {
    SCENARIO_INFO("Update");

    obstacles_ = (std::vector<DynamicObstacle> *)(&data.dynamic_obstacles_);

    // No objects received
    if (data.dynamic_obstacles_.size() == 0 || data.dynamic_obstacles_[0].prediction_.gaussians[0].mean.poses.size() == 0)
    {
      if (config_->debug_output_)
        ROS_WARN("Stage Scenario: No obstacles received yet. Update skipped!");

      return;
    }

    status_ = ScenarioStatus::SUCCESS;

    // Translate Gaussian samples to the observations and save the pointer
    SCENARIO_INFO("Preparing Samples");

    scenarios_ = sampler_->TranslateToMeanAndVariance(data.dynamic_obstacles_);

    // Compute the poses and orientation for this disc
    SCENARIO_INFO("Retrieving Vehicle Trajectory");

    // retrievePoses(solver_interface, true);
    disc_->retrievePoses(solver_interface);

    // Get the radii for each of the obstacles
    int disc_id = 0;
    for (size_t v = 0; v < data.dynamic_obstacles_.size(); v++)
    {
      for (size_t d = 0; d < data.dynamic_obstacles_[v].discs_.size(); d++)
      {
        radii_[disc_id] = data.dynamic_obstacles_[v].discs_[d].radius_ + config_->vehicle_width_ / 2.;
        disc_id++;
      }
    }

    SCENARIO_INFO("Constructing Constraints From Scenarios");
    PushAlgorithm(solver_interface);

    if (config_->multithread_scenarios_)
    {
      // multithread
      for (size_t k = 0; k < N; k++)
      {
        // Create a thread for stage k
        scenario_threads_[k] = std::thread(&StageScenario::scenariosToConstraints, this, k, data.halfspaces_[k]);
      }
      // Wait for all threads to finish
      for (auto &t : scenario_threads_)
        t.join();
    }
    else
    {
      // single thread
      for (size_t k = 0; k < N; k++)
        scenariosToConstraints(k, data.halfspaces_[k]);
    }

    SCENARIO_INFO("Done");
  }

  void StageScenario::Visualize()
  {
    PROFILE_FUNCTION();

    if (enable_visualization_)
    {
      // Visualise scenarios
      if (config_->draw_all_scenarios_)
        visualiseScenarios();

      if (config_->draw_polygon_scenarios_)
        visualisePolygonScenarios();

      if (config_->draw_removed_scenarios_)
        visualiseRemovedScenarios(config_->indices_to_draw_);

      if (config_->draw_constraints_)
      {
        // Draw the polygons using the lines from the constructors
        visualisePolygonScenarios();

        visualisePolygons();
      }

      publishVisuals();
    }
  }

  // Constructs N x m linear constraints for all stages and the given scenarios_ (result = scenario_constraints_)
  void StageScenario::scenariosToConstraints(int k, const std::vector<Halfspace> &halfspaces)
  {
    // Reset the polygon class
    polygon_constructors_[k].Reset();

    // Make a local copy that changes
    Eigen::Vector2d pose = disc_->poses_[k];

    //------------------- Verify the relevance of obstacles ----------------------------------//
    active_obstacle_count_[k] = 0;

    // Check all obstacles
    for (uint v = 0; v < (unsigned int)config_->max_obstacles_; v++)
    {
      auto &obstacle = (*obstacles_)[v];

      // If the vehicle is near one of the means
      for (auto &gaussian : obstacle.prediction_.gaussians)
      {
        Eigen::Vector2d obst(gaussian.mean.poses[k].pose.position.x, gaussian.mean.poses[k].pose.position.y);
        // If this obstacle is relevant
        if (Helpers::dist(obst, pose) <= config_->activation_range_)
        {
          // Save that this obstacle is active for this stage, increase the count
          active_obstacle_indices_[k][active_obstacle_count_[k]] = v;
          active_obstacle_count_[k]++;
        }

        break;
      }
    }

    SCENARIO_INFO("Computing Distances");

    //--------------------- Construct constraints for all non-removed points ----------------------//
    {
      PROFILE_SCOPE("Constructing Halfspaces");

      // Here we compute the constraints for all obstacles
      for (int v = 0; v < active_obstacle_count_[k]; v++)
      {
        int obst = active_obstacle_indices_[k][v];

        // MOVED
        // // Compute distances
        diffs_x_[k] = (*scenarios_)[k][obst][0].array() - pose(0); // Requires modifications to the sampler
        diffs_y_[k] = (*scenarios_)[k][obst][1].array() - pose(1);

        //.array()?
        distances_[k].block(v * S, 0, S, 1) = (diffs_x_[k] * diffs_x_[k] + diffs_y_[k] * diffs_y_[k]).sqrt();

        // Feasibility is assumed under the ellipsoidal projection

        // Compute the components of A for this obstacle
        a1_[k].block(v * S, 0, S, 1) = diffs_x_[k] / distances_[k].block(v * S, 0, S, 1);
        a2_[k].block(v * S, 0, S, 1) = diffs_y_[k] / distances_[k].block(v * S, 0, S, 1);

        // Compute b
        b_[k].block(v * S, 0, S, 1) = a1_[k].block(v * S, 0, S, 1) * (*scenarios_)[k][obst][0].array() +
                                      a2_[k].block(v * S, 0, S, 1) * (*scenarios_)[k][obst][1].array() -
                                      radii_[v]; //(a1_[k].block(v * S, 0, S, 1).square() + a2_[k].block(v * S, 0, S,
                                                 // 1).square()) * combined_radius_;
      } // End of obstacles

      {
        PROFILE_SCOPE("Removing Scenarios");

        // Sort on distance and remove the closest scenarios
        if (R_ > 0)
          removeScenariosBySorting(k);
      }

      SCENARIO_INFO("Finalizing Constraints");

      {
        PROFILE_SCOPE("Creating Objects");

        // Add range constraints
        polygon_constructors_[k].AddRangeConstraints(pose, disc_->orientations_[k], a1_[k], a2_[k], b_[k]);

        // Add external halfspace constraints
        double a0_ext, a1_ext, b_ext;
        for (int i = 0; i < config_->n_halfspaces_; i++)
        {
          // Guard against 0 constraints by slightly altering them
          a0_ext = halfspaces[i].A_[0];
          if (std::abs(a0_ext) < 1e-5)
            a0_ext = 1e-5;

          a1_ext = halfspaces[i].A_[1];
          if (std::abs(a1_ext) < 1e-5)
            a1_ext = 1e-5;

          b_ext = halfspaces[i].b_;

          a1_[k](config_->max_obstacles_ * S + i) = a0_ext;
          a2_[k](config_->max_obstacles_ * S + i) = a1_ext;
          b_[k](config_->max_obstacles_ * S + i) = b_ext;
        }

        // Computations on all constraints
        x_left_[k] = pose(0) - 1e-8 - std::sqrt(2) * config_->polygon_range_;
        x_right_[k] = pose(0) + 1e-8 + std::sqrt(2) * config_->polygon_range_;

        // Compute all y at the left and the right for this k
        y_left_[k] = (b_[k] - a1_[k] * x_left_[k]) / a2_[k];
        y_right_[k] = (b_[k] - a1_[k] * x_right_[k]) / a2_[k];

        // Bot top check by fieasiblil
        for (size_t c = 0; c < constraints_[k].size(); c++)
          constraints_[k][c].side_ = a2_[k](c) > 0 ? ConstraintSide::TOP : ConstraintSide::BOTTOM;
      }
    }

    SCENARIO_INFO("Constructing Polygons");

    {
      PROFILE_SCOPE("Recursive Polygon");

      polygon_constructors_[k].Search(pose, disc_->orientations_[k], x_left_[k], x_right_[k]);
      // polygon_out_ has the scenario constraints
    }

    areas_[k] = 0;
  }

  void StageScenario::removeScenariosBySorting(int k)
  {
    std::iota(sort_indices_.begin(), sort_indices_.end(), 0);

    // Removal based on distance to VRU mean!
    std::sort(sort_indices_.begin(), sort_indices_.end(),
              [&](const int &a, const int &b)
              { return distances_[k](a) < distances_[k](b); });

    // Remove the first R_ constraints
    for (int i = 0; i < R_; i++)
      polygon_constructors_[k].RemoveConstraint(scenario_indices_[k][sort_indices_[i]]);
  }

  void StageScenario::feasibilityCheck(int k)
  {
    Eigen::Vector2d pose = disc_->poses_[k];

    Eigen::ArrayXd constraint_values = a1_[k] * pose(0) + a2_[k] * pose(1) - b_[k];

    for (u_int s = 0; s < S; s++)
    {
      if (constraint_values(s) > 1e-3)
      {
        is_feasible_ = false;
        return;
      }
    }
  }

  void StageScenario::PushAlgorithm(BaseModel *solver_interface)
  {
    PROFILE_FUNCTION();

    for (size_t k = 0; k < N; k++)
    {
      Eigen::Vector2d pose = disc_->poses_[k]; // Pose of this disc

      for (int obstacle_id = 0; obstacle_id < config_->max_obstacles_; obstacle_id++)
      {
        Eigen::Vector2d vector_sum(0., 0.);

        for (u_int s = 0; s < S; s += 5) // We do not really need to add every sample
        {
          Eigen::Vector2d scenario_pose((*scenarios_)[k][obstacle_id][0](s), (*scenarios_)[k][obstacle_id][1](s));
          vector_sum += pose - scenario_pose;
        }
        vector_sum.normalize();

        double biggest_push = -radii_[obstacle_id];
        // V^T d
        Eigen::VectorXd d_vec = vector_sum(0) * ((*scenarios_)[k][obstacle_id][0].array() - pose(0)) +
                                vector_sum(1) * ((*scenarios_)[k][obstacle_id][1].array() - pose(1));

        // Find the worst
        double max_d_vec = d_vec.maxCoeff();
        if (max_d_vec > biggest_push)
          biggest_push = max_d_vec;

        pose += vector_sum * (biggest_push + radii_[obstacle_id]); // Add a push to the pose of this disc
      }

      // pose += vector_sum * (biggest_push + config_->ego_w_ / 2.); // Add a push

      disc_->poses_[k] = pose; // Set the new disc pose
      // Eigen::Vector2d veh_pose = disc_->DiscPoseToVehiclePoseAt(k); // Translate to the vehicle pose
      // solver_interface->InitialPlan(k).set_x(veh_pose(0));
      // solver_interface->InitialPlan(k).set_y(veh_pose(1));
    }
  }

  void StageScenario::useBackupPlan(BaseModel *solver_interface, double vel, double deceleration)
  {
    LMPCC_INFO("Stage Scenario: Finding a backup plan");

    // First, try to find out what deceleration is sufficient (For the front disc!)
    // double v = 0.; // solver_interface->State().get_v();
    double angle = solver_interface->State().psi();

    // // Current disc position
    Eigen::Vector2d cur_pose(solver_interface->State().x(), solver_interface->State().y());
    cur_pose += Eigen::Vector2d(std::cos(angle) * disc_->offset_,
                                std::sin(angle) * disc_->offset_); // Account for the disc offset

    // // Function to compute vehicle pose under constant velocity
    auto get_vehicle_pose = [&](const Eigen::Vector2d &start_pose, int T, double velocity, Eigen::Vector2d &pose_out)
    {
      pose_out = (start_pose +
                  Eigen::Vector2d(std::cos(angle), std::sin(angle)) * (velocity * ((double)(T)) * solver_interface->DT));
    };

    // Is still questionable
    solver_interface->InitialPlan(0) =
        solver_interface->State(); // Set the initial state to match the current measurements

    // More involved: compute deceleration for safety
    // double deceleration = getSafeDeceleration(solver_interface);
    // Simple: specified value
    // double deceleration = config_->deceleration_at_infeasible_;

    // double angle = solver_interface->psi(0);
    // Is it correct that cur_pose is the current state?
    Eigen::Vector2d pose_k, prev_pose;
    for (size_t k = 0; k < N; k++)
    {
      double velocity = /*solver_interface->v()*/ vel - deceleration * solver_interface->DT * k; // v -= a*T, T = k*dt
      if (velocity < 0.)                                                                         // No rear driving
        velocity = 0;

      if (k == 0)
      {
        pose_k = cur_pose;
      }
      else
      {
        solver_interface->predicted_v(k) = velocity;

        // Set the solver values (Note: reverts back to disc positions)
        get_vehicle_pose(prev_pose, 1, velocity, pose_k);
        auto &state_k = solver_interface->InitialPlan(k);
        state_k.set_x(pose_k(0));
        state_k.set_y(pose_k(1));
        state_k.set_psi(pose_k(angle));
      }
      prev_pose = pose_k;
    }
  }

  // Insert chance constraints into the optimisation
  void StageScenario::SetParameters(BaseModel *solver_interface, const RealTimeData &data, int k, int &param_idx)
  {
    // Insert constraints A_l * x <= b_l
    assert(k > 0);

    // Insert all active constraints in this stage
    if ((int)polygon_constructors_[k - 1].polygon_out_.size() > 24)
    {
      ROS_ERROR_STREAM("Not enough support (24). Polygon had " << (int)polygon_constructors_[k - 1].polygon_out_.size()
                                                               << " edges.");
    }
    for (int l = 0; l < std::min(24, (int)polygon_constructors_[k - 1].polygon_out_.size()); l++)
    {
      ScenarioConstraint *&scenario_constraint = polygon_constructors_[k - 1].polygon_out_[l];

      int lin_offset = param_idx + l * 3;
      int cur_index = scenario_constraint->GetHalfspaceIndex(S);
      solver_interface->setParameter(k, lin_offset, a1_[k - 1](cur_index));
      solver_interface->setParameter(k, lin_offset + 1, a2_[k - 1](cur_index));
      solver_interface->setParameter(k, lin_offset + 2, b_[k - 1](cur_index));
    }

    // Insert dummies on other spots (relative to vehicle position)
    for (int l = polygon_constructors_[k - 1].polygon_out_.size(); l < 20 + 4; l++)
    {
      int lin_offset = param_idx + l * 3;
      solver_interface->setParameter(k, lin_offset, 1.0);
      solver_interface->setParameter(k, lin_offset + 1, 0.0);
      solver_interface->setParameter(k, lin_offset + 2, solver_interface->InitialPlan(k - 1).x() + 100.0);
    }
    param_idx += (20 + 4) * 3;
  }

  //-------------------------------- VISUALIZATION -----------------------------------//
  // Visualise the predictions
  void StageScenario::visualiseScenarios()
  {
    if (obstacles_->size() < 1)
      return;

    bool draw_circle = false;
    ROSMultiplePointMarker &scenario_points = ros_markers_->getNewMultiplePointMarker("POINTS");
    scenario_points.setScale(0.05, 0.05, 0.05);
    scenario_points.setColor(0, 0, 0, 0.4); // alpha 0.25 shows the spread a bit better

    ROSPointMarker &scenario_circles = ros_markers_->getNewPointMarker("CYLINDER");
    scenario_circles.setColor(1, 0, 0, 0.05);

    Eigen::Vector3d point_cur;
    for (int v = 0; v < config_->max_obstacles_; v++)
    {
      for (size_t s = 0; s < S; s++)
      {
        for (size_t k = 0; k < config_->indices_to_draw_.size(); k++)
        {
          int index = config_->indices_to_draw_[k];
          point_cur = getScenarioLocation(index, v, s);
          point_cur(2) = 0.05;
          scenario_points.addPointMarker(point_cur);
          if (draw_circle)
          {
            scenario_circles.setColorInt(k, (int)config_->indices_to_draw_.size(), 0.05);
            scenario_circles.setScale(2 * radii_[v] - config_->vehicle_width_ / 2.,
                                      2 * radii_[v] - config_->vehicle_width_ / 2., 0.01);
            scenario_circles.addPointMarker(point_cur);
          }
        }
      }
    }

    scenario_points.finishPoints();
  }

  void StageScenario::visualiseRemovedScenarios(const std::vector<int> &indices_to_draw)
  {
    ROSMultiplePointMarker &removed_scenarios_ = ros_markers_->getNewMultiplePointMarker("POINTS");
    removed_scenarios_.setScale(0.15, 0.15, 0.15);
    removed_scenarios_.setColor(0, 0.7, 0);

    for (size_t k = 0; k < indices_to_draw.size(); k++)
    {
      const int &index = indices_to_draw[k];

      // Draw removed and selected scenarios
      for (size_t i = 0; i < S; i++)
      {
        for (int v = 0; v < active_obstacle_count_[index]; v++)
        {
          // Get the index of this obstacle
          int obst = active_obstacle_indices_[index][v];

          // Plot removed constraints
          if (!polygon_constructors_[index].WasRemoved(scenario_indices_[k][obst * S + i]))
            continue;

          Eigen::Vector3d scenario_location = getScenarioLocation(index, obst, i);

          // Draw a point
          removed_scenarios_.addPointMarker(scenario_location);
        }
      }
    }

    removed_scenarios_.finishPoints();
  }

  void StageScenario::visualiseSelectedScenarios(const std::vector<int> &indices_to_draw)
  {
    // ROSPointMarker &selected_scenario_points = ros_markers_->getNewPointMarker("CUBE");
    // selected_scenario_points.setScale(0.1, 0.1, 0.1);
    // selected_scenario_points.setColor(1, 1, 0);

    // ROSPointMarker &selected_scenario_circles = ros_markers_->getNewPointMarker("CYLINDER"); // Just the obstacles
    // themselves selected_scenario_circles.setScale(2 * (config_->r_VRU_), 2 * (config_->r_VRU_), 0.01);
    // selected_scenario_circles.setColor(1, 1, 0, 0.025);
    // ROSPointMarker &selected_full_circles = ros_markers_->getNewPointMarker("CYLINDER"); // Includes the vehicle!
    // selected_scenario_circles.setColor(1, 1, 0, 0.025);

    // for (uint k = 0; k < indices_to_draw.size(); k++)
    // {
    //     const int &index = indices_to_draw[k];
    //     selected_scenario_points.setColor((double)k / (double)indices_to_draw.size() / 4.0);
    //     selected_scenario_circles.setColor((double)k / (double)indices_to_draw.size() / 4.0, 0.1);

    //     // Get lines in the polygon
    //     std::vector<ScenarioConstraint *> &constraints = polygon_constructors_[index].polygon_out_;

    //     for (size_t i = 0; i < constraints.size(); i++)
    //     {

    //         if (constraints[i]->type_ != ObstacleType::DYNAMIC)
    //             continue;

    //         Eigen::Vector3d scenario_location = getScenarioLocation(
    //             index,
    //             constraints[i]->scenario_->obstacle_idx_,
    //             constraints[i]->scenario_->idx_);

    //         // Draw a yellow point
    //         scenario_location(2) = 0.5;
    //         selected_scenario_points.addPointMarker(scenario_location);

    //         // Draw a circle around it
    //         if (config_->r_VRU_ > 0.0)
    //         {
    //             selected_scenario_circles.addPointMarker(scenario_location);
    //         }
    //         selected_full_circles.addPointMarker(scenario_location);
    //     }
    // }
  }

  // Visualizes which scenarios make up the polygon, not necessarily of support
  void StageScenario::visualisePolygonScenarios()
  {
    ROSPointMarker &polygon_scenario_points = ros_markers_->getNewPointMarker("CYLINDER");
    polygon_scenario_points.setScale(0.15, 0.15, 0.1);

    ROSPointMarker &selected_scenario_circles = ros_markers_->getNewPointMarker("CYLINDER");
    selected_scenario_circles.setColor(1, 1, 0, 0.1);

    ROSPointMarker &selected_full_circles = ros_markers_->getNewPointMarker("CYLINDER"); // Includes the vehicle!
    selected_full_circles.setColor(1, 1, 0, 0.01);
    Eigen::Vector3d scenario_location;

    SupportSubsample polygon_constraints;

    // Aggregate constraints along the horizon
    for (size_t k = 0; k < N; k++)
    {
      // Get the polygon for this stage
      std::vector<ScenarioConstraint *> &constraints_ = polygon_constructors_[k].polygon_out_;

      for (size_t i = 0; i < constraints_.size(); i++)
      {
        Scenario &scenario = *constraints_[i]->scenario_;

        if (constraints_[i]->type_ == ObstacleType::DYNAMIC && !polygon_constraints.ContainsScenario(scenario))
          polygon_constraints.Add(scenario);
      }
    }

    // For all scenarios of support -> Plots support subsample, which includes removed scenarios!
    for (int i = 0; i < polygon_constraints.size_; i++)
    {
      // Draw each constraints along the horizon
      Scenario *&scenario = polygon_constraints.scenarios_[i];

      for (size_t k = 0; k < config_->indices_to_draw_.size(); k++)
      {
        const int &index = config_->indices_to_draw_[k];

        polygon_scenario_points.setColorInt(k, (int)config_->indices_to_draw_.size(), 1.0);
        selected_scenario_circles.setColorInt(k, (int)config_->indices_to_draw_.size(), 0.15);
        // selected_full_circles.setColor((double)k / (double)config_->indices_to_draw_.size() / config_->color_range_,
        // 0.1);

        double r = radii_[scenario->obstacle_idx_] - config_->vehicle_width_ / 2.;
        // selected_full_circles.setScale(2 * r, 2 * r, 0.01);
        selected_scenario_circles.setScale(2 * r, 2 * r, 0.01); // + config_->ego_w_ / 2.0

        // k, obs, scenario
        // Get the location of this
        scenario_location = getScenarioLocation(index, scenario->obstacle_idx_, scenario->idx_);

        // Draw a yellow point
        scenario_location(2) = k * 0.1;
        polygon_scenario_points.addPointMarker(scenario_location);

        // Draw a circle around it
        if (config_->r_VRU_ > 0.0)
          selected_scenario_circles.addPointMarker(scenario_location);

        // selected_full_circles.addPointMarker(scenario_location);
      }
    }
  }

  void StageScenario::visualisePolygons()
  {
    ROSLine &line = ros_markers_->getNewLine();
    geometry_msgs::Point p1, p2;
    p1.z = 0.3;
    p2.z = 0.3; // 2D points
    // ROSPointMarker &intersections = ros_markers_->getNewPointMarker("CUBE");
    // intersections.setScale(0.25, 0.25, 0.1);
    // intersections.setColor(0, 0, 0);

    // line.setLifetime(1.0);

    for (size_t k = 0; k < config_->indices_to_draw_.size(); k++)
    {
      int &index = config_->indices_to_draw_[k];
      line.setColorInt(k, (int)config_->indices_to_draw_.size());
      // line.setColor(polygon_constructors_[index].peel_level_/ 4.0 / 4.0);
      line.setScale(0.1, 0.1);
      p1.z = 0.3;
      p2.z = 0.3;
      std::vector<Eigen::Vector2d *> &intersects = polygon_constructors_[index].intersects_out_;

      for (size_t i = 0; i < intersects.size(); i++)
      {
        // Consecutive intersections
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
        // std::cout << "Drawing line from " << p1.x << ", " << p1.y << " to " << p2.x << ", " << p2.y << std::endl;

        line.addLine(p1, p2);

        // intersections.addPointMarker(p1);
      }
    }
  }

  void StageScenario::visualiseProjectedPosition()
  {
    ROSPointMarker &selected_scenario_circles = ros_markers_->getNewPointMarker("CYLINDER");
    selected_scenario_circles.setScale(1.0, 1.0, 0.01); // + config_->ego_w_ / 2.0
    selected_scenario_circles.setColor(1, 1, 1, 0.7);

    for (uint k = 0; k < config_->indices_to_draw_.size(); k++)
    {
      const int &index = config_->indices_to_draw_[k];

      Eigen::Vector3d scenario_location(projected_poses_[index](0), projected_poses_[index](1), 0.2);

      selected_scenario_circles.addPointMarker(scenario_location);
    }
  }
  Eigen::Vector3d StageScenario::getScenarioLocation(const int &k, const int &obstacle_index, const int &scenario_index)
  {
    Eigen::Vector2d loc((*scenarios_)[k][obstacle_index][0](scenario_index),
                        (*scenarios_)[k][obstacle_index][1](scenario_index));
    return Eigen::Vector3d(loc(0), loc(1), 0.2);
  }

  void StageScenario::publishVisuals()
  {
    // Draws all the markers added to ros_markers_ and resets
    ros_markers_->publish();
  }
};
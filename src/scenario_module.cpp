#include "scenario_module/scenario_module.h"

#include <scenario_module/safety_certifier.h>
#include <scenario_module/safe_horizon.h>
#include <scenario_module/logging.h>

#include <mpc_planner_solver/solver_interface.h>
#include <mpc_planner_util/parameters.h>

#include <ros_tools/data_saver.h>
#include <ros_tools/profiling.h>

namespace ScenarioModule
{
  ScenarioModule::ScenarioModule()
  {
  }

  void ScenarioModule::initialize(std::shared_ptr<Solver> solver)
  {
    _solver = solver;
    status_ = ScenarioStatus::RESET;

    SCENARIO_CONFIG.Init();
    SafetyCertifier::Get().Init();

    // Initialize the scenario handlers for the discs
    int n_discs = CONFIG["n_discs"].as<int>();
    disc_manager_.resize(n_discs);

    if (SCENARIO_CONFIG.enable_safe_horizon_)
    {
      sampler_.Init();

      for (int i = 0; i < n_discs; i++)
        disc_manager_[i].reset(new SafeHorizon(i, _solver, sampler_));
    }
  }

  bool ScenarioModule::isDataReady(const RealTimeData &data, std::string &missing_data)
  {
    bool data_ready = (SCENARIO_CONFIG.enable_safe_horizon_ && sampler_.SamplesReady());
    if (!data_ready)
    {
      missing_data += " Samples";
      status_ = ScenarioStatus::DATA_MISSING;
      return false;
    }

    return true;
  }

  void ScenarioModule::update(const RealTimeData &data, const ModuleData &module_data)
  {
    SCENARIO_INFO("ScenarioConstraints::Update()");

    for (auto &disc : disc_manager_)
      disc->update(data, module_data);

    // Checks if there were any problems in constructing the polytopes
    if (IsScenarioStatus(ScenarioStatus::SUCCESS, status_))
      return;

    SCENARIO_INFO("Not all discs are feasible");
  }

  // How to allow for two statuses.
  bool ScenarioModule::IsScenarioStatus(ScenarioStatus &&expected_status, ScenarioStatus &status_out)
  {
    status_out = expected_status;

    for (auto &disc : disc_manager_)
    {
      if (disc->status_ != expected_status)
      {
        status_out = disc->status_;
        return false;
      }
    }

    return true;
  }

  void ScenarioModule::setParameters(const RealTimeData &data, int k)
  {
    for (auto &disc : disc_manager_)
      disc->setParameters(data, k);
  }

  int ScenarioModule::optimize(const RealTimeData &data)
  {
    LOG_MARK("Scenario optimization");
    RosTools::Benchmarker iteration_timer("iteration");
    RosTools::Timer timeout_timer(_solver->_params.solver_timeout);
    timeout_timer.start();

    int exit_code = -1; // Returned exit code
    bool feasible = true;

    int prev_support_estimate = 0;
    int new_support = 0;

    SupportSubsample support = SupportSubsample(SafetyCertifier::Get().GetMaxSupport());
    SupportSubsample prev_support;

    _solve_status = ScenarioSolveStatus::SUCCESS;

    // Every iteration has variables (x*, cost, support, risk)
    double iteration_time = 0.;
    for (int iteration = 0; iteration < SCENARIO_CONFIG.max_iterations_; iteration++)
    {
      iteration_timer.start();

      if (SCENARIO_CONFIG.debug_output_ && (iteration == 0 || new_support > 0))
        support.PrintUpdate(_solver->_solver_id, SafetyCertifier::Get().GetSafeSupportBound(), iteration);

      // Set the initial guess for the first iteration and use the saved trajectory for consecutive iterations
      // _solver->setReinitialize(iteration == 0); // @TODO
      if (iteration == 0)
        _solver->initializeOneIteration();

      // OPTIMIZE
      {
        PROFILE_SCOPE("Solver Iteration");
        // exit_code = _solver->solve();
        exit_code = _solver->solveOneIteration();
        exit_code = _solver->completeOneIteration();
      }

      // DETECT SUPPORT (over all discs)
      feasible = true;
      SupportSubsample infeasible_scenarios;
      for (auto &disc : disc_manager_)
        feasible = feasible & disc->computeActiveConstraints(support, infeasible_scenarios);

      if (exit_code != 1) // stop (it cannot recover)
      {
        SCENARIO_WARN_STREAM("[Solver " << _solver->_solver_id << "] SQP(" << iteration
                                        << ") iterate became infeasible (exit_code = " << exit_code << ")");

        _solve_status = ScenarioSolveStatus::INFEASIBLE;
        _solver->completeOneIteration();
        return exit_code;
      }

      // Insert the support constraints to the discs for visualization (Note the last disc already has this information)
      for (auto &disc : disc_manager_)
        disc->support_subsample_.MergeWith(support);

      if (support.size_ > SafetyCertifier::Get().GetMaxSupport())
        break;

      // Update the support parameters
      new_support = support.size_ - prev_support_estimate;
      prev_support_estimate = support.size_;

      // Timing (stop when over time)
      iteration_time += iteration_timer.stop();
      double avg_iteration_time = iteration_time / ((double)(iteration + 1));

      // Stop iterating if we ran out of time
      if (timeout_timer.currentDuration() + avg_iteration_time >= _solver->_params.solver_timeout)
      {
        LOG_WARN_THROTTLE(500., "Timeout is enabled. Stopping after " << iteration + 1 << " iterations because planning time is exceeded");
        break;
      }
    }

    _solver->completeOneIteration();

    // Validate the trajectory
    if (!feasible) // Check feasibility w.r.t. the relaxed inequality constraints
    {
      _solve_status = ScenarioSolveStatus::INFEASIBLE;
      SCENARIO_WARN_ALWAYS("Optmized trajectory was not provably safe: Constraints were not satisfied");
      // return -1;
    }
    else if (support.size_ > SafetyCertifier::Get().GetMaxSupport()) // Support exceeded the limit
    {
      _solve_status = ScenarioSolveStatus::SUPPORT_EXCEEDED;
      SCENARIO_WARN_ALWAYS("Optmized trajectory was not provably safe: Support bound exceeded (" << support.size_ << " > " << SafetyCertifier::Get().GetMaxSupport() << ")");
      // return -1;
    }
    else if (_solver->getOutput(1, "slack") > 1e-3) // Slack was necessary to find a solution
    {
      _solve_status = ScenarioSolveStatus::NONZERO_SLACK;
      SCENARIO_WARN_ALWAYS("Optmized trajectory was not provably safe: Slack variable was not zero (value: " << _solver->getOutput(1, "slack") << ")");
      // return -1;
    }

    SafetyCertifier::Get().LogSupport(support.size_);

    return exit_code; // Return the solution if it is feasible
  }

  bool ScenarioModule::ComputeActiveConstraints(SupportSubsample &active_constraints_aggregate,
                                                SupportSubsample &infeasible_scenarios)
  {
    bool feasible = true;
    for (auto &disc : disc_manager_)
      feasible = feasible & disc->computeActiveConstraints(active_constraints_aggregate, infeasible_scenarios);

    return feasible;
  }

  void ScenarioModule::MergeSupport(SupportSubsample &support_estimate)
  {
    for (auto &disc : disc_manager_)
      disc->support_subsample_.MergeWith(support_estimate);
  }

  void ScenarioModule::ExportData(RosTools::DataSaver &data_saver)
  {
    if (first_run_)
    {
      data_saver.AddData("sample_size", SafetyCertifier::Get().GetSampleSize());
      data_saver.AddData("max_support", SafetyCertifier::Get().GetMaxSupport());
      data_saver.AddData("max_iterations", SCENARIO_CONFIG.max_iterations_);
      data_saver.AddData("risk", SCENARIO_CONFIG.risk_);
      data_saver.AddData("confidence", SCENARIO_CONFIG.confidence_);
      first_run_ = false;
    }
  }

  void ScenarioModule::visualize(const RealTimeData &data)
  {
    SCENARIO_INFO("Visualize");
    for (auto &disc : disc_manager_)
      disc->Visualize(data);
  }
};
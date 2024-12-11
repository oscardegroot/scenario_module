/**
 * @file scenario_base.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Base class for S-MPCC and SH-MPC (mostly virtual)
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef SCENARIO_BASE_H
#define SCENARIO_BASE_H

#include <scenario_module/sampler.h>

#include <scenario_module/types.h>

#include <mpc_planner_types/realtime_data.h>
#include <mpc_planner_types/module_data.h>

using namespace MPCPlanner;

// Forward declare
namespace RosTools
{
  struct Halfspace;
}

namespace ScenarioModule
{
  enum class ScenarioStatus
  {
    SUCCESS = 0,
    PROJECTED_SUCCESS,
    BACKUP_PLAN,
    INFEASIBLE,
    DATA_MISSING,
    RESET
  };

  class ScenarioBase
  {
  public:
    virtual void update(const RealTimeData &data, const ModuleData &module_data) = 0;

    virtual void SetSampler(Sampler *sampler) { sampler_ = sampler; }

    virtual void setParameters(const RealTimeData &data, int k) = 0;

    virtual void Visualize(const RealTimeData &data) = 0;

    virtual bool computeActiveConstraints(SupportSubsample &active_constraints_aggregate,
                                          SupportSubsample &infeasible_scenarios)
    {
      (void)active_constraints_aggregate;
      (void)infeasible_scenarios;
      return false;
    }

  public:
    // To unify for trajectory / marginal, can add k to Scenario structure
    SupportSubsample support_subsample_;

    Sampler *sampler_ = nullptr;

    ScenarioStatus status_;
    bool polygon_failed_;
  };
} // namespace ScenarioModule

#endif
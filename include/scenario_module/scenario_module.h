/**
 * @file lmpcc_scenario_module.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief This class is the main interface from the scenario computations to the MPC
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef LMPCC_SCENARIO_MODULE_H
#define LMPCC_SCENARIO_MODULE_H

#include <scenario_module/config.h>
#include <scenario_module/scenario_base.h>
#include <scenario_module/sampler.h>
#include <scenario_module/types.h>

// Forward declare
namespace MPCPlanner
{
  class Solver;
}

namespace RosTools
{
  class DataSaver;
}

namespace ScenarioModule
{
  class ScenarioModule
  {

  public:
    ScenarioModule();
    void initialize(std::shared_ptr<Solver> solver);

  public:
    void update(const RealTimeData &data, const ModuleData &module_data);

    bool isDataReady(const RealTimeData &data, std::string &missing_data);

    void setParameters(const RealTimeData &data, int k);

    int optimize(const RealTimeData &data);

    void visualize(const RealTimeData &data);

    bool ComputeActiveConstraints(
        SupportSubsample &active_constraints_aggregate,
        SupportSubsample &infeasible_scenarios);

    void MergeSupport(SupportSubsample &support_estimate);

    void ExportData(RosTools::DataSaver &data_saver);

    /**
     * @brief Check if the status of a scenario class is the same for all discs
     *
     * @param expected_status The status to compare with
     * @param status_out The actual status
     * @return true If the status matched the expected one
     * @return false otherwise
     */
    bool IsScenarioStatus(ScenarioStatus &&expected_status, ScenarioStatus &status_out);

    Config &GetConfig()
    {
      return SCENARIO_CONFIG;
    }
    Sampler &GetSampler()
    {
      return sampler_;
    }

  private:
    std::shared_ptr<Solver> _solver;

    ScenarioStatus status_;
    ScenarioSolveStatus _solve_status;
    std::vector<std::unique_ptr<ScenarioBase>> disc_manager_;

    Sampler sampler_;

    bool first_run_;
  };
}

#endif
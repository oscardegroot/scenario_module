/**
 * @file config.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Loads parameters for PRM
 * @version 0.1
 * @date 2022-07-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef SCENARIO_CONFIGURATION_H
#define SCENARIO_CONFIGURATION_H

#include <ros_tools/base_configuration.h>

namespace ScenarioModule
{
#define _SCENARIO_CONFIG ScenarioModule::Config::Get()
#define SCENARIO_CONFIG Config::Get()

  class Config : public RosTools::BaseConfiguration
  {

  public:
    // Singleton function
    static Config &Get()
    {

      static Config instance_;

      return instance_;
    }

    Config() { success_ = false; };
    Config(const Config &) = delete;

    ~Config(){};

    bool Init();
    bool Initialized() const { return success_; }

    /************ CONFIGURATION VARIABLES **************/
  public:
    bool success_;

    // Debug
    bool debug_output_;

    unsigned int N_;
    int max_obstacles_;

    std::string target_frame_;

    double vehicle_width_;
    int draw_disc_;

    bool use_json_trajectory;
    bool scale_reference_velocity_;

    bool static_obstacles_enabled_;

    int n_halfspaces_;
    int sample_size_;
    int batch_count_;
    int removal_count_;
    bool automatically_compute_sample_size_;
    int draw_every_;

    bool draw_all_scenarios_, draw_selected_scenarios_, draw_removed_scenarios_, draw_ellipsoids_, draw_constraints_;
    bool draw_projected_position_, draw_polygon_scenarios_, draw_initial_guess_;
    double visuals_scale_;

    int seed_;
    double r_vehicle_;
    bool truncated_;
    double truncated_radius_;
    bool enable_safe_horizon_;
    int parallel_scenario_planners_;
    bool enable_termination_;
    double terminate_eq_tol_;
    double n_bar_;
    bool use_real_samples_;
    bool halfspaces_from_spline_;

    bool build_database_;

    bool binomial_distribution_;

    // Polygons
    int polygon_checked_constraints_;
    bool propagate_covariance_;
    double polygon_range_;
    double activation_range_;

    // Probabilistic scenario properties
    double risk_;
    double confidence_;
    double received_object_sigma_;

    bool enable_scenario_removal_;
    int max_iterations_;
  };
}

#endif

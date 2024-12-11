/**
 * @file sampler.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Sampling of Gaussian distributions or Gaussian Mixture Models
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SAMPLER_H__
#define __SAMPLER_H__

#include <scenario_module/partition_sampler.h>
#include <scenario_module/config.h>
#include <scenario_module/types.h>

#include <ros_tools/random_generator.h>
#include <ros_tools/math.h>
#include <ros_tools/data_saver.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Dense>

using namespace MPCPlanner;

namespace ScenarioModule
{
  /**
   * @brief Class for sampling Gaussian distributions
   */
  class Sampler
  {
  public:
    Sampler() { samples_ready_ = false; };

  public:
    bool standard_samples_ready_; // Internally: have the standard samples been prepared
    bool samples_ready_;          // Are the samples of the real distribution ready?

    /** @brief Initialization. Reads or generates standard normal samples.  */
    void Init();

    /**
     * @brief Translate standard normal samples to the mean and variances of obstacle predictions
     *
     * @param obstacles Dynamic obstacles with populated predictions
     * @return std::vector<trajectory_sample>* Pointer to the scenarios
     */
    std::vector<trajectory_sample> *TranslateToMeanAndVariance(const std::vector<DynamicObstacle> &obstacles, double dt);

    /**
     * @brief Propagate samples over a horizon
     *
     * @param msg GMMs for all obstacles
     * @param dt integration step
     * @return std::vector<trajectory_sample>* Pointer to the scenarios
     */
    std::vector<trajectory_sample> *IntegrateAndTranslateToMeanAndVariance(const std::vector<DynamicObstacle> &_obstacles, const double dt);

    void SamplePartitionedScenarios(const std::vector<DynamicObstacle> &obstacles,
                                    std::unordered_map<int, Partition> partitions,
                                    int partition_id, int num_partitions, double dt);

    /**
     * @brief Load samples from an external message
     *
     * @param obstacle_msgs The message with samples (obstacle x sample x time x (x, y))
     * @param obstacles Obstacles for translating discs if necessary
     * @param sample_size_is_correct True if sample size is okay
     * @param sample_size_msg Message to send if sample size is not okay
     */
    void LoadSamples(const std_msgs::Float64MultiArray::ConstPtr &obstacle_msgs,
                     const std::vector<DynamicObstacle> &obstacles, bool &sample_size_is_correct,
                     std_msgs::Int32 &sample_size_ms);

    std::vector<trajectory_sample> *GetSamples() { return &samples_; }; // Get the samples

    int SampleSize() const { return standard_samples_[0].size(); }; // Returns the sample size of the first batch (do not use when sample size is varying)
    bool SamplesReady() const { return samples_ready_; };           // True if samples are prepared

  private:
    RosTools::RandomGenerator random_generator_;

    PartitionSampler partition_sampler_;

    // These are variables used in translating the samples online
    std::vector<std::vector<std::vector<Eigen::Matrix2d>>> R_, SVD_, Sigma_, A_;
    std::vector<std::vector<double>> sum_of_probabilities_;

    RosTools::DataSaver data_saver_; /* For saving and loading */

    std::vector<std::vector<Eigen::Vector2d>> standard_samples_; /* Batches of samples with mean 0 and variance of 1. */
    std::vector<trajectory_sample> samples_;                     /* Output samples, translated */

    /** @brief Sample standard normal samples (i.e., mean 0, variance 1) */
    void SampleStandardNormal();

    /** @brief Sample truncated standard normal samples (i.e., mean 0, variance 1) */
    void SampleTruncatedStandardNormal();

    /** @brief Prune samples in the center of the distribution (only used in S-MPCC) */
    void Prune();

    void SortSamples();   /** @brief Sort samples on distance from the center */
    void ResizeSamples(); /** @brief Resize samples based on pruned sample size */

    /**
     * @brief Fit an ellipse that contains all samples
     *
     * @param samples the samples to fit on
     * @return Eigen::Vector2d axes of the ellipse
     */
    Eigen::Vector2d FitMaximumEllipse(const std::vector<Eigen::Vector2d> &samples);

    /** @brief Save standard normal samples */
    void Save();

    /**
     * @brief Load standard normal samples
     *
     * @return true If file exists
     * @return false If no file exists
     */
    bool Load();

    /**
     * @brief Construct a file path for the samples
     *
     * @return std::string the file path
     */
    std::string GetFilePath();

    /**
     * @brief Construct a file name for the samples
     *
     * @return std::string the file name
     */
    std::string GetFileName();
  };
}
#endif // __SAMPLER_H__
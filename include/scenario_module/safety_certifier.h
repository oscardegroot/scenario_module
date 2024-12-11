/**
 * @file safety_certifier.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Uses the scenario approach to certify safety of a trajectory
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef SAFETY_CERTIFIER_H
#define SAFETY_CERTIFIER_H

#include <ros_tools/data_saver.h>

// Numerical tolerance used in the bisection algorithm
#define BISECTION_TOLERANCE 1e-6

namespace ScenarioModule
{
  class SupportSubsample;
  class SafetyCertifier
  {
  public:
    // Singleton (Use SafetyCertifier::Get(). to access this class)
    static SafetyCertifier &Get()
    {
      static SafetyCertifier instance_;

      return instance_;
    }

    SafetyCertifier(const SafetyCertifier &) = delete;

  public:
    void Init();

    static double CertifySupportProblem(double epsilon, double n_collected) { return std::pow(1. - epsilon, (double)n_collected); }

    /**
     * @brief Find the sample size that is safe for the given configuration
     *
     * @param risk Probability of constraint violation (0 - 1)
     * @param confidence Confidence with which risk < specified risk (0 - 1) - Fairly "cheap" in terms of sample size
     * @param max_support The maximum support to be encountered
     * @return int The sample size
     */
    int FindSampleSize(double risk, double confidence, int max_support);

    /** Getters */
    int GetSampleSize() { return sample_size_; }
    int GetMaxSupport() const { return max_support_; }

    /** @brief Returns the maximum safe support */
    int GetSafeSupportBound();

    /**
     * @brief Returns the risk for a particular support
     *
     * @param s support (integer)
     * @return double The risk at support = s
     */
    double GetRiskForSupport(int s);

    /**
     * @brief Returns the risk for a particular support (supplied as support subsample object)
     *
     * @param support_subsample support
     * @return double The risk at the given support
     */
    double GetRiskForSupport(const SupportSubsample &support_subsample);

    void LogSupport(int support_size);

  private:
    /** @brief Class for managing the Convex SP setting the support maximum */
    struct SupportData
    {
      int n_collected;
      int max_support;
      RosTools::DataSaver support_saver;

      std::string name;

      SupportData(const std::string &_name);

      void Add(int support);

      void Save();
      bool Load();

      std::string GetFileName() const { return name + "_support-data"; }
    };
    std::unique_ptr<SupportData> support_data_;

    SafetyCertifier() {};

    // Main parameters
    int max_support_;
    int sample_size_;
    double risk_;
    double confidence_;

    bool initialized_ = false;

    // Look up table
    std::vector<double> risk_lut_;

    // Mathematical helper functions //
    /**
     * @brief Compute N choose k inside of a root numerically
     *
     * @param N
     * @param k
     * @param root
     * @return * double
     */
    double rootedNChooseK(double N, double k, double root);

    /**
     * @brief Apply bisection on the given function
     *
     * @param low minimum output value
     * @param high maximum output value
     * @param func function to bisect on
     * @return int result
     */
    int Bisection(double low, double high, std::function<double(int)> func);

    /**
     * @brief Compute the risk for all support levels < max_support_
     */
    void ComputeRiskForSupportRange();

    /**
     * @brief The risk function
     *
     * @param sample_size S
     * @param support n
     * @return double resulting risk
     */
    double Epsilon(int sample_size, int support);

    /**
     * @brief The risk function, but with support = max_support
     *
     * @param sample_size S
     * @return double resulting risk
     */
    double EpsilonForMaxSupport(int sample_size);
  };
};
#endif
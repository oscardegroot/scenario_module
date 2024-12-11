#include "scenario_module/safety_certifier.h"

#include <scenario_module/config.h>
#include <scenario_module/types.h>
#include <scenario_module/logging.h>

#include <ros_tools/math.h>

namespace ScenarioModule
{
  void SafetyCertifier::Init()
  {
    if (initialized_)
      return;
    // Load settings
    risk_ = SCENARIO_CONFIG.risk_;
    confidence_ = SCENARIO_CONFIG.confidence_;

    SCENARIO_WARN_ALWAYS("Initializing Safety Certifier");

    if (SCENARIO_CONFIG.enable_safe_horizon_)
    {
      max_support_ = SCENARIO_CONFIG.n_bar_ + SCENARIO_CONFIG.removal_count_;
    }
    else
    {
      max_support_ = 20; // Number of constraints in the polygon (maximum)
    }

    // Find the sample size
    if (SCENARIO_CONFIG.automatically_compute_sample_size_)
    {
      SCENARIO_CONFIG.sample_size_ = FindSampleSize(risk_, confidence_, max_support_); // Overwrites the sample size
    }
    else
    {
      LOG_INFO("Safety Certifier: Using static sample size (S = " << SCENARIO_CONFIG.sample_size_ << ")");
    }

    sample_size_ = SCENARIO_CONFIG.sample_size_; // save in this class

    // Compute the risk for all allowed support values
    ComputeRiskForSupportRange();
    support_data_.reset(new SupportData("debug"));

    initialized_ = true;

    SCENARIO_WARN_ALWAYS("Safety Certifier Initialized");
  }

  int SafetyCertifier::Bisection(double low, double high, std::function<double(int)> func)
  {
    if (low > high)
      throw std::runtime_error("Safety Certifier: Bisection low value was higher than the high value!");

    double value_low = func(low);

    double mid;
    double value_mid;

    for (int iterations = 0; iterations < 1000; iterations++)
    {
      mid = (low + high) / 2.0;
      value_mid = func(mid);

      if (std::abs(value_mid) < BISECTION_TOLERANCE || (high - low) / 2.0 < BISECTION_TOLERANCE)
        return mid;

      // Extra check because of integers
      if (high - low == 1)
        return high;

      if (RosTools::sgn(value_mid) == RosTools::sgn(value_low))
      {
        low = mid;
        value_low = value_mid;
      }
      else
      {
        high = mid;
      }
      // std::cout << "low: " << low << ", mid: " << mid << ", high: " << high << std::endl;
    }

    throw std::runtime_error("Safety Certifier: Bisection failed!");
  }

  int SafetyCertifier::FindSampleSize(double risk, double confidence, int max_support)
  {
    int low = max_support + 1;
    int start_high = 500000;
    int high = start_high;

    // Do a bisection search on the function of epsilon
    sample_size_ = Bisection(low, high, std::bind(&SafetyCertifier::EpsilonForMaxSupport, this, std::placeholders::_1));
    sample_size_ = std::ceil(sample_size_) + 1; // Correct safely

    LOG_INFO("Safety Certifier:\n\tRisk: " << risk << "\n\tConfidence: " << confidence_
                                           << "\n\tMaximum Support (Removed): " << max_support << " ("
                                           << SCENARIO_CONFIG.removal_count_ << ")"
                                           << "\n\tComputed Sample Size: " << sample_size_);

    if (sample_size_ == start_high)
    {
      ROS_WARN_STREAM("Safety Certifier: Sample size maximum in bisection was reached!");
    }

    return sample_size_;
  }

  int SafetyCertifier::GetSafeSupportBound()
  {
    return max_support_;
  }

  double SafetyCertifier::Epsilon(int sample_size, int support)
  {
    if (SCENARIO_CONFIG.enable_safe_horizon_)
    {
      return 1.0 - std::pow(confidence_ / (double)sample_size, 1.0 / ((double)(sample_size - support))) *
                       (1.0 / rootedNChooseK(sample_size, support, sample_size - support));
    }
    else
    {
      return 1.0 - std::pow(confidence_ / (double)max_support_, 1.0 / ((double)(sample_size - support))) *
                       (1.0 / rootedNChooseK(sample_size, support, sample_size - support));
    }
  }

  double SafetyCertifier::EpsilonForMaxSupport(int sample_size)
  {
    return risk_ - Epsilon(sample_size, max_support_);
  }

  void SafetyCertifier::ComputeRiskForSupportRange()
  {
    risk_lut_.resize(sample_size_);

    for (int k = 0; k < sample_size_; k++)
      risk_lut_[k] = 1.0 - std::pow(confidence_ / ((double)sample_size_), 1.0 / ((double)(sample_size_ - k))) *
                               (1.0 / rootedNChooseK(sample_size_, k, sample_size_ - k));
  }

  double SafetyCertifier::rootedNChooseK(double N, double k, double root)
  {
    double result = 1.0;
    for (int i = 1; i <= k; i++)
    {
      result *= std::pow((N - (k - i)) / (k - i + 1), 1.0 / root);
    }

    return result;
  }

  double SafetyCertifier::GetRiskForSupport(int support)
  {
    return risk_lut_[support];
  }

  double SafetyCertifier::GetRiskForSupport(const SupportSubsample &support_subsample)
  {
    return GetRiskForSupport(support_subsample.size_);
  }

  void SafetyCertifier::LogSupport(int support_size)
  {
    support_data_->Add(support_size);
  }

  SafetyCertifier::SupportData::SupportData(const std::string &_name)
  {
    name = _name;
    bool data_existed = Load();
    if (!data_existed)
    {
      n_collected = 0;
      max_support = 0;
    }
  }

  void SafetyCertifier::SupportData::Add(int support)
  {
    if (support > max_support)
    {
      max_support = support;
      n_collected = 0;
      LOG_WARN("Support Logging - New maximum support: " << max_support);
    }
  }

  void SafetyCertifier::SupportData::Save()
  {
    support_saver.Clear();
    support_saver.AddData("n_collected", n_collected);
    support_saver.AddData("max_support", max_support);
    LOG_INFO("Max Support: " << max_support);
    support_saver.SaveData(GetFileName());
  }

  bool SafetyCertifier::SupportData::Load()
  {
    std::map<std::string, std::vector<double>> data;
    bool success = support_saver.LoadData<double>(GetFileName(), data);

    if (!success)
      return false;

    n_collected = data["n_collected"][0];
    max_support = data["max_support"][0];
    return true;
  }
};
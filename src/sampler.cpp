#include "scenario_module/sampler.h"

#include <scenario_module/logging.h>

#include <mpc_planner_types/data_types.h>

#include <ros_tools/convertions.h>
#include <ros_tools/profiling.h>
#include <ros_tools/logging.h>
#include <ros_tools/paths.h>

#include <filesystem>

namespace ScenarioModule
{
  /** @todo replace "SCENARIO_CONFIG.sample_size" with input parameter */
  void Sampler::Init()
  {
    SCENARIO_WARN("Gaussian Sampler: Initializing");

    // Allocate space for standard samples
    standard_samples_.resize(SCENARIO_CONFIG.batch_count_);
    for (int b = 0; b < SCENARIO_CONFIG.batch_count_; b++)
      standard_samples_[b].resize(SCENARIO_CONFIG.sample_size_);

    // Allocate space for scenarios
    samples_.resize(SCENARIO_CONFIG.N_);
    for (size_t k = 0; k < SCENARIO_CONFIG.N_; k++)
    {
      samples_[k].resize(SCENARIO_CONFIG.max_obstacles_);
      for (int v = 0; v < SCENARIO_CONFIG.max_obstacles_; v++)
      {
        samples_[k][v].resize(2);
        samples_[k][v][0] = Eigen::VectorXd::Ones(SCENARIO_CONFIG.sample_size_) * 100.0;
        samples_[k][v][1] = Eigen::VectorXd::Ones(SCENARIO_CONFIG.sample_size_) * 100.5;
      }
    }

    if (SCENARIO_CONFIG.use_real_samples_)
      partition_sampler_.UpdatePartitionData(1);

    bool load_successful = Load();
    if (!load_successful)
    {
      SCENARIO_WARN("Sample database does not exist. Generating new samples...");
      SCENARIO_CONFIG.truncated_ ? SampleTruncatedStandardNormal() : SampleStandardNormal();

      if (!SCENARIO_CONFIG.enable_safe_horizon_)
        Prune();

      // Save();
    }

    ResizeSamples();

    standard_samples_ready_ = true;
    samples_ready_ = false;

    // Initialize matrices
    R_.resize(SCENARIO_CONFIG.max_obstacles_);
    SVD_.resize(SCENARIO_CONFIG.max_obstacles_);
    Sigma_.resize(SCENARIO_CONFIG.max_obstacles_);
    A_.resize(SCENARIO_CONFIG.max_obstacles_);
    sum_of_probabilities_.resize(SCENARIO_CONFIG.max_obstacles_);
    int num_modes = SCENARIO_CONFIG.binomial_distribution_ ? SCENARIO_CONFIG.N_ + 1 : 1;

    for (size_t v = 0; v < R_.size(); v++)
    {
      R_[v].resize(SCENARIO_CONFIG.N_);
      SVD_[v].resize(SCENARIO_CONFIG.N_);
      Sigma_[v].resize(SCENARIO_CONFIG.N_);
      A_[v].resize(SCENARIO_CONFIG.N_);

      sum_of_probabilities_.reserve(num_modes); // Reserve not resize

      for (size_t k = 0; k < SCENARIO_CONFIG.N_; k++)
      {
        R_[v][k].resize(num_modes);
        SVD_[v][k].resize(num_modes);
        Sigma_[v][k].resize(num_modes);
        A_[v][k].resize(num_modes);
      }
    }

    SCENARIO_SUCCESS_ALWAYS("Gaussian Sampler: Ready");
  }

  void Sampler::SampleStandardNormal()
  {
    // Draw scenarios
    for (int b = 0; b < SCENARIO_CONFIG.batch_count_; b++)
    {
      for (int s = 0; s < SCENARIO_CONFIG.sample_size_; s++)
      {
        // Generate uniform random numbers in 2D
        standard_samples_[b][s] = Eigen::Vector2d(random_generator_.Double(), random_generator_.Double());

        // Convert them to a Gaussian
        RosTools::RandomGenerator::uniformToGaussian2D(standard_samples_[b][s]);
      }
    }
  }

  void Sampler::SampleTruncatedStandardNormal()
  {
    // Initialize variables
    double truncated_cap = std::exp(-std::pow(SCENARIO_CONFIG.truncated_radius_, 2.0) / 2.0);

    // Draw scenarios
    for (int b = 0; b < SCENARIO_CONFIG.batch_count_; b++)
    {
      for (int s = 0; s < SCENARIO_CONFIG.sample_size_; s++)
      {
        // Generate uniform random numbers in 2D
        standard_samples_[b][s] = Eigen::Vector2d(random_generator_.Double(), random_generator_.Double());

        // In the case of truncated gaussian distribution, we modify the random sampled variables
        standard_samples_[b][s](0) = standard_samples_[b][s](0) * (1.0 - truncated_cap) + truncated_cap;

        // Convert them to a Gaussian
        RosTools::RandomGenerator::uniformToGaussian2D(standard_samples_[b][s]);
      }
    }
  }

  // Sort the standard samples on their distance from the middle
  void Sampler::SortSamples()
  {
    for (int batch = 0; batch < SCENARIO_CONFIG.batch_count_; batch++)
    {
      std::sort(standard_samples_[batch].begin(), standard_samples_[batch].end(),
                [](const Eigen::Vector2d &a, const Eigen::Vector2d &b)
                { return a.squaredNorm() > b.squaredNorm(); });
    }
  }

  void Sampler::ResizeSamples()
  {
    // Resize the actual samples
    for (size_t k = 0; k < SCENARIO_CONFIG.N_; k++)
    {
      for (int v = 0; v < SCENARIO_CONFIG.max_obstacles_; v++)
      {
        samples_[k][v].resize(2);
        samples_[k][v][0] = Eigen::VectorXd::Ones(standard_samples_[0].size()) * 100.0;
        samples_[k][v][1] =
            Eigen::VectorXd::Ones(standard_samples_[0].size()) * 100.5; // Resizing depends on the batch...
      }
    }
  }

  void Sampler::Prune()
  {
    SortSamples(); // Necessary for pruning

    int max_size = 0;

    for (int b = 0; b < SCENARIO_CONFIG.batch_count_; b++)
    {
      // General idea: sample points in a circle around the distribution, check which ones are never close enough
      // Initialize vectors that we need
      std::vector<double> distances;
      distances.resize(SCENARIO_CONFIG.sample_size_);

      // Vector of indices that are close enough at some point
      std::vector<bool> used_indices(SCENARIO_CONFIG.sample_size_, false);

      std::vector<int> all_indices;
      all_indices.resize(SCENARIO_CONFIG.sample_size_);

      // Fit an ellipse to know where to compute from when pruning
      Eigen::Vector2d max_ellipse_r = FitMaximumEllipse(standard_samples_[b]);

      // Idea: Sample a few points around the circle, check if the scenario is used in that case
      for (int point_it = 0; point_it < 50; point_it++)
      {
        double angle = 2.0 * M_PI / 50.0 * (double)point_it; // Space the angle

        // Compute the associated point
        Eigen::Vector2d point(max_ellipse_r(0) * std::cos(angle),
                              max_ellipse_r(1) * std::sin(angle)); // rx * cos(psi) , ry * sin(psi)

        // For all samples, compute the distance
        for (int s = 0; s < SCENARIO_CONFIG.sample_size_; s++)
        {
          distances[s] = RosTools::distance(point, standard_samples_[b][s]);
          all_indices[s] = s;
        }

        // Here we have [distances, 1, ..., S]

        // Sort to find the smallest distances to our point outside (keep track of indices!)
        std::sort(all_indices.begin(), all_indices.end(),
                  [&distances](const int &a, const int &b)
                  { return distances[a] < distances[b]; });

        // Indices, sorted on their distance (starting low)

        // The closest l+R are marked as used
        for (int i = 0; i < SCENARIO_CONFIG.polygon_checked_constraints_ + SCENARIO_CONFIG.removal_count_; i++)
        {
          used_indices[all_indices[i]] = true;
        }
      }

      int batch_prune_count = 0;

      // Find the first index marked as used, starting from the end
      // (remember sorted from large distance to small distance to mean)
      for (size_t s = SCENARIO_CONFIG.sample_size_ - 1; s >= 0; s--)
      {
        if (used_indices[s] == true)
        {
          batch_prune_count = s;

          break;
        }
      }

      if (batch_prune_count > max_size)
        max_size = batch_prune_count;
    }

    SCENARIO_WARN_ALWAYS("Gaussian Samples pruned (for all batches): "
                         << SCENARIO_CONFIG.sample_size_ - max_size << "/" << SCENARIO_CONFIG.sample_size_ << " ("
                         << (double)(SCENARIO_CONFIG.sample_size_ - max_size) / (double)SCENARIO_CONFIG.sample_size_ * 100.0 << ")");

    for (int b = 0; b < SCENARIO_CONFIG.batch_count_; b++)
    {
      // Resize the samples to those not pruned
      standard_samples_[b].resize(max_size);

      std::random_shuffle(standard_samples_[b].begin(),
                          standard_samples_[b].end()); // Make sure samples are not sorted anymore
    }
  }

  Eigen::Vector2d Sampler::FitMaximumEllipse(const std::vector<Eigen::Vector2d> &samples)
  {
    double max_x = 0.0;
    double max_y = 0.0;

    // Go through all samples
    double abs_current_x, abs_current_y;
    for (int s = 0; s < SCENARIO_CONFIG.sample_size_; s++)
    {
      // Check if this is the furthest x
      abs_current_x = std::abs(samples[s](0));
      if (abs_current_x > max_x)
        max_x = abs_current_x;

      // Check if this is the furthest y
      abs_current_y = std::abs(samples[s](1));
      if (abs_current_y > max_y)
        max_y = abs_current_y;
    }

    return Eigen::Vector2d(max_x, max_y);
  }

  /** @note Real-time sample functions */

  std::vector<trajectory_sample> *Sampler::TranslateToMeanAndVariance(const std::vector<DynamicObstacle> &obstacles, double dt)
  {
    if (!standard_samples_ready_)
      ROS_WARN_ONCE("Call Sampler::Get().Init() before sampling!");

    SCENARIO_INFO("Gaussian Sampler: TranslateToMeanAndVariance()");

    // Initialize variables
    Eigen::Matrix<double, 2, 2> A, Sigma, R, SVD;
    double psi;

    // We also construct a list for the probabilities in the GMM to check which one to sample from
    std::vector<double> sum_of_probabilities_;
    sum_of_probabilities_.reserve(SCENARIO_CONFIG.N_ + 1);

    // Decide which batch to samples from (for each k)
    std::vector<int> batch_pick;
    for (size_t k = 0; k < SCENARIO_CONFIG.N_; k++)
      batch_pick.push_back(random_generator_.Int(SCENARIO_CONFIG.batch_count_ - 1));

    for (size_t v = 0; v < obstacles.size(); v++)
    {
      const DynamicObstacle &obstacle = obstacles[v];     // The current obstacle
      const Prediction &prediction = obstacle.prediction; // Get the predictions

      // Construct a list of summed probabilities
      double p_summed = 0.;
      sum_of_probabilities_.clear();
      sum_of_probabilities_.resize(prediction.probabilities.size());
      for (size_t mode = 0; mode < prediction.probabilities.size(); mode++)
      {
        p_summed += prediction.probabilities[mode];
        sum_of_probabilities_[mode] = p_summed;
      }

      double mode_random;
      int mode_pick = -1;
      for (int s = 0; s < SampleSize(); s++)
      {
        // Decide what mode to sample from
        mode_random = random_generator_.Double();
        for (size_t mode = 0; mode < sum_of_probabilities_.size(); mode++)
        {
          if (mode_random <= sum_of_probabilities_[mode])
          {
            mode_pick = mode;
            break;
          }
        }

        // Easy reference for the path
        const Mode &gaussian = prediction.modes[mode_pick];

        // For all stages
        double major = 0.; // Integrate (sigma_x_k+1 = sigma_x_k + sigma_x@(k+1)^2 dt^2)
        double minor = 0.;
        for (uint k = 0; k < gaussian.size(); k++)
        {
          // Get the rotation matrix
          psi = gaussian[k].angle; // RosTools::quaternionToAngle(path.poses[k].pose);
          R = RosTools::rotationMatrixFromHeading(-psi);

          // Convert the semi axes back to gaussians
          if (SCENARIO_CONFIG.propagate_covariance_)
          {
            major += std::pow(gaussian[k].major_radius * dt, 2);
            minor += std::pow(gaussian[k].minor_radius * dt, 2);
          }
          else
          {
            major = std::pow(gaussian[k].major_radius, 2.);
            minor = std::pow(gaussian[k].minor_radius, 2.);
          }

          SVD << major, 0., 0., minor;

          // Compute Sigma and cholesky decomposition
          Sigma = R * SVD * R.transpose();
          A = Sigma.llt().matrixL();

          // Adapt the standard samples to this sigma and mu
          samples_[k][v][0](s) = A(0, 0) * standard_samples_[batch_pick[k]][s](0) +
                                 A(0, 1) * standard_samples_[batch_pick[k]][s](1) +
                                 gaussian[k].position(0);
          samples_[k][v][1](s) = A(1, 0) * standard_samples_[batch_pick[k]][s](0) +
                                 A(1, 1) * standard_samples_[batch_pick[k]][s](1) +
                                 gaussian[k].position(1);
        }
      }
    }

    // Return the samples
    samples_ready_ = true;
    return &samples_;
  }

  std::vector<trajectory_sample> *Sampler::IntegrateAndTranslateToMeanAndVariance(const std::vector<DynamicObstacle> &_obstacles,
                                                                                  const double dt)
  {
    if (!standard_samples_ready_)
      ROS_WARN_ONCE("Call Sampler::Get().Init() before sampling!");

    SCENARIO_INFO("Gaussian Sampler: IntegrateAndTranslateToMeanAndVariance()");

    const auto obstacles = _obstacles; // Copy!

    ROSTOOLS_ASSERT(obstacles.size() == (unsigned int)SCENARIO_CONFIG.max_obstacles_,
                    "Received number of obstacles does not match the expectation"); // Ensure obstacles are all present
    for (auto &obstacle : obstacles)
    {
      ROSTOOLS_ASSERT(obstacle.prediction.modes.size() == obstacle.prediction.probabilities.size(),
                      "Number of modes and probabilities do not match");
    }
    PROFILE_FUNCTION();
    auto rng = std::mt19937{std::random_device{}()};
    // Pick a batch to sample from for each k
    std::vector<int> batch_pick;
    for (u_int k = 0; k < SCENARIO_CONFIG.N_; k++)
      batch_pick.push_back(random_generator_.Int(SCENARIO_CONFIG.batch_count_ - 1));

    // We are using the Gaussian standard samples here to generate Gaussian bivariates
    // To ensure the gaussian is different for different k, we shuffle the sample picks
    std::vector<std::vector<int>> shuffle_indices;
    shuffle_indices.resize(SCENARIO_CONFIG.N_);
    for (size_t k = 0; k < shuffle_indices.size(); k++)
    {
      shuffle_indices[k].resize(SCENARIO_CONFIG.sample_size_); // k, s
      std::iota(shuffle_indices[k].begin(), shuffle_indices[k].end(), 0);
    }

    // Reshuffle indices for all k
    // #pragma omp parallel for
    for (size_t k = 0; k < shuffle_indices.size(); k++)
    {
      std::shuffle(shuffle_indices[k].begin(), shuffle_indices[k].end(),
                   rng); // Shuffle scenario indices (COULD BE OPTIMIZED A LOT)
    }

    // #pragma omp parallel for
    for (size_t v = 0; v < obstacles.size(); v++) // For all obstacles
    {
      // Construct a list of summed probabilities over the modes
      double p_summed = 0.;
      sum_of_probabilities_[v].clear();
      for (size_t mode = 0; mode < obstacles[v].prediction.probabilities.size(); mode++)
      {
        p_summed += obstacles[v].prediction.probabilities[mode];
        sum_of_probabilities_[v].push_back(p_summed);
      }

      // Compute the covariance matrix (ASSUMED SAME NOISE FOR ALL PEDESTRIANS)
      for (size_t k = 0; k < SCENARIO_CONFIG.N_; k++)
      {
        for (size_t mode = 0; mode < obstacles[v].prediction.modes.size(); mode++)
        {
          const auto &cur_mode = obstacles[v].prediction.modes[mode];

          // Get the angle of the path
          double psi = cur_mode[k].angle; // RosTools::quaternionToAngle(cur_mode.mean.poses[k].pose);

          // Get the rotation matrix and construct the covariance matrix
          R_[v][k][mode] = RosTools::rotationMatrixFromHeading(-psi);
          SVD_[v][k][mode] = Eigen::Matrix2d::Zero();
          SVD_[v][k][mode] << std::pow(cur_mode[k].major_radius, 2.), 0.0, 0.0,
              std::pow(cur_mode[k].minor_radius, 2.);

          Sigma_[v][k][mode] = R_[v][k][mode] * SVD_[v][k][mode] * R_[v][k][mode].transpose();

          // Take the square root for translation to the covariance (which is assumed constant over the horizon)
          A_[v][k][mode] = Sigma_[v][k][mode].llt().matrixL();
        }
      }

      // To facilitate multiple modes, we sample from each mode in proportional amounts
      for (int s = 0; s < SampleSize(); s++)
      {
        // Decide what mode to sample from
        double mode_random = random_generator_.Double();
        int mode_pick = -1;

        for (size_t mode = 0; mode < sum_of_probabilities_[v].size(); mode++)
        {
          if (mode_random <= sum_of_probabilities_[v][mode])
          {
            mode_pick = mode;
            break;
          }
        }

        // Initialize the aggregated noise at zero
        Eigen::Vector2d bivariate_gaussian(0., 0.);

        for (uint k = 0; k < SCENARIO_CONFIG.N_; k++)
        {
          const Eigen::Vector2d &cur_pose =
              obstacles[v].prediction.modes[mode_pick][k].position; // Use the mean from the selected mode

          int pick = shuffle_indices[k][s]; // Pick a random standard normal sample from the pre-generated samples

          bivariate_gaussian += Eigen::Vector2d(
              A_[v][k][mode_pick](0, 0) * standard_samples_[batch_pick[k]][pick](0) +
                  A_[v][k][mode_pick](0, 1) * standard_samples_[batch_pick[k]][pick](1),
              A_[v][k][mode_pick](1, 0) * standard_samples_[batch_pick[k]][pick](0) +
                  A_[v][k][mode_pick](1, 1) * standard_samples_[batch_pick[k]][pick](1)); // Get a translated random number and add it to the sum

          samples_[k][v][0](s) = cur_pose(0) + bivariate_gaussian(0) * dt; // x_k+1 = mean + uncertainty*dt
          samples_[k][v][1](s) = cur_pose(1) + bivariate_gaussian(1) * dt;
        }
      }
    }

    // Return samples
    samples_ready_ = true;
    return &samples_;
  }

  void Sampler::SamplePartitionedScenarios(const std::vector<DynamicObstacle> &obstacles,
                                           std::unordered_map<int, Partition> partitions,
                                           int partition_id, int num_partitions, double dt)
  {
    partition_sampler_.SamplePartitionedScenarios(obstacles, partitions, partition_id, num_partitions, dt, samples_);
    samples_ready_ = true;
  }

  void Sampler::LoadSamples(const std_msgs::Float64MultiArray::ConstPtr &obstacle_msgs,
                            const std::vector<DynamicObstacle> &obstacles, bool &sample_size_is_correct,
                            std_msgs::Int32 &sample_size_msg)
  {
    PROFILE_FUNCTION();

    int temp_expected_discs = 1; // ASSUMED: ALL OBSTACLES HAVE ONE DISC

    // Check the sample size and return a message if necessary
    if (obstacle_msgs->data.size() != SCENARIO_CONFIG.max_obstacles_ * SCENARIO_CONFIG.sample_size_ * SCENARIO_CONFIG.N_ * 2 / temp_expected_discs)
    {
      ROS_ERROR_STREAM("Sampler: LoadSamples received "
                       << obstacle_msgs->data.size() << " samples\n"
                       << "instead of (Obstacle (discs) x sample size x horizon length)\n"
                       << SCENARIO_CONFIG.max_obstacles_ << " x " << SCENARIO_CONFIG.sample_size_ << " x " << SCENARIO_CONFIG.N_ << " = "
                       << SCENARIO_CONFIG.max_obstacles_ * SCENARIO_CONFIG.sample_size_ * SCENARIO_CONFIG.N_ * 2 / temp_expected_discs);

      // Communicate with the node to change to the correct sample size
      sample_size_msg.data = SCENARIO_CONFIG.sample_size_;
      sample_size_is_correct = false;
      return;
    }
    else
    {
      sample_size_is_correct = true;
    }

    // Message layout: V x S x N x 2
    int id = 0;
    int disc_id;
    for (size_t v = 0; v < (unsigned int)SCENARIO_CONFIG.max_obstacles_ / temp_expected_discs; v++) // For each obstacle
    {
      // double orientation;
      for (int s = 0; s < SampleSize(); s++) // Each sample
      {
        // For all stages
        for (size_t k = 0; k < SCENARIO_CONFIG.N_; k++)
        {
          // Orientation per sample
          // if (k > 0)
          // {
          //   orientation = std::atan2(
          //       obstacle_msgs->data[id + 1] - samples_[k - 1][v][1](s), /* y difference between this k and the previous */
          //       obstacle_msgs->data[id] - samples_[k - 1][v][0](s)      /* x difference between this k and the previous */
          //   );
          // }
          // else
          // {
          //   orientation = obstacles[v].angle;
          // }

          // Offset samples with the disc offset and their orientation
          for (disc_id = 0; disc_id < temp_expected_discs; disc_id++)
          {
            samples_[k][v + disc_id][0](s) =
                obstacle_msgs->data[id]; // + obstacles[v].discs_[disc_id].offset * std::cos(orientation);
          }
          id++;

          for (disc_id = 0; disc_id < temp_expected_discs; disc_id++)
          {
            samples_[k][v + disc_id][1](s) =
                obstacle_msgs->data[id]; // + obstacles[v].discs_[disc_id].offset * std::sin(orientation);
          }
          id++;
        }
      }
    }
    samples_ready_ = true;
  }

  void Sampler::Save()
  {
    for (int b = 0; b < SCENARIO_CONFIG.batch_count_; b++)
    {
      for (int s = 0; s < SCENARIO_CONFIG.sample_size_; s++)
        data_saver_.AddData("standard_samples_" + std::to_string(b), standard_samples_[b][s]);
    }

    data_saver_.SaveData(GetFilePath(), GetFileName());
  }

  bool Sampler::Load()
  {
    std::map<std::string, std::vector<Eigen::Vector2d>> data;

    if (!data_saver_.LoadData(GetFilePath(), GetFileName(), data))
      return false;

    for (int b = 0; b < SCENARIO_CONFIG.batch_count_; b++)
      standard_samples_[b] = data["standard_samples_" + std::to_string(b)];

    return true;
  }

  std::string Sampler::GetFilePath()
  {
    // Get the package path
    std::string path = getPackagePath("scenario_module");

    path += "/samples";
    if (std::filesystem::create_directories(path))
      LOG_VALUE("File directory", path);

    return path;
  }

  std::string Sampler::GetFileName()
  {
    std::string file_name = "samples_";

    if (SCENARIO_CONFIG.truncated_)
      file_name += "truncated_at_" + std::to_string((int)SCENARIO_CONFIG.truncated_radius_) + "_";

    if (!SCENARIO_CONFIG.enable_safe_horizon_)
      file_name += "pruned_";

    file_name += "S" + std::to_string(SCENARIO_CONFIG.sample_size_) + "_R" + std::to_string(SCENARIO_CONFIG.removal_count_) + ".txt";

    return file_name;
  }
}
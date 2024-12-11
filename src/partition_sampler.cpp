#include "scenario_module/partition_sampler.h"

#include <scenario_module/config.h>
#include <scenario_module/logging.h>

#include <ros_tools/paths.h>
#include <ros_tools/profiling.h>

#include <third_party/json.hpp>

namespace ScenarioModule
{

    PartitionSampler::PartitionSampler()
    {
        partition_read_id_ = 0;
        thread_pool_.reset(new ThreadPool(PARTITION_READ_THREADS));
    }

    void PartitionSampler::UpdatePartitionData(int partition_size)
    {
        SCENARIO_INFO_ALWAYS("Updating partitions...");
        // Find the number of partitions
        // std::string meta_path = ros::package::getPath("lmpcc") + "/scripts/cluster/meta.txt";
        // std::ifstream meta_file(meta_path);

        // int partition_size;
        // meta_file >> partition_size;
        SCENARIO_INFO_ALWAYS("Number of Partitions: " << partition_size);

        // Create a new set of partitions
        std::vector<PartitionReader> new_partitions;
        new_partitions.resize(partition_size);

        // Read the new partition
        std::vector<bool> success(partition_size, false);

#pragma omp parallel for num_threads(PARTITION_READ_THREADS)
        for (size_t i = 0; i < new_partitions.size(); i++)
        {
            // if (ros::ok())                              // To prevent slow stopping
            success[i] = new_partitions[i].Init(i); // Read real samples (velocity samples!)
        }

        for (size_t i = 0; i < success.size(); i++)
        {
            if (!success[i])
                return;
        }

        int min_size = 1e5;
        int max_size = -1;
        for (size_t i = 0; i < new_partitions.size(); i++)
        {
            if (new_partitions[i].n_samples_ < min_size)
                min_size = new_partitions[i].n_samples_;

            if (new_partitions[i].n_samples_ > max_size)
                max_size = new_partitions[i].n_samples_;
        }

        SCENARIO_INFO_ALWAYS("Min partition size: " << min_size << ", max partition size: " << max_size);

        // Only copy if the data is not being used
        mutex_.lock();
        partitions_.resize(partition_size);
        partitions_ = new_partitions;
        largest_sample_size_ = SCENARIO_CONFIG.sample_size_;
        mutex_.unlock();

        SCENARIO_INFO_ALWAYS("New partitions loaded!");
    }

    // void Sampler::SampleJsonRealTrajectories(const mpc_msgs::lmpcc_obstacle_array &obstacle_msgs, const
    // mpc_msgs::observation_partitioning &obs)
    void PartitionSampler::SamplePartitionedScenarios(const std::vector<DynamicObstacle> &obstacles,
                                                      std::unordered_map<int, Partition> partitions,
                                                      int partition_id, int num_partitions, double dt,
                                                      std::vector<trajectory_sample> &output_scenarios)
    {
        PROFILE_FUNCTION();
        SCENARIO_INFO("Using partitioned database to generate scenarios");

        // If we made a new thread at least once
        bool thread_ready = true;
        if (partition_read_id_ > 0)
        {
            auto partition_status = partition_future_.wait_for(std::chrono::duration<short, std::nano>::zero());
            thread_ready = partition_status == std::future_status::ready; // Check if we can spawn a new thread
        }

        // Check if we need to start loading new partitions in a new thread
        // If the ID of the partitions has changed OR if this is not the first partition change but the thread is still running.
        if (partition_id != partition_read_id_ && thread_ready)
        {
            partition_read_id_ = partition_id;
            partition_future_ = thread_pool_->enqueue(&PartitionSampler::UpdatePartitionData, this, num_partitions);
        }

        int N = SCENARIO_CONFIG.N_;

        mutex_.lock(); // We are using the partitions, do not load new partitions here...

        // For each obstacle
        for (size_t v = 0; v < obstacles.size(); v++)
        {

            // THIS CODE IS COPIED CURRENTLY
            const auto &obstacle = obstacles[v];
            double angle = obstacle.angle; // Helpers::quaternionToAngle(obstacle.pose_.orientation);

            //  Find the partition through the map (or set to 0 if not found)
            Partition partition;
            if (partitions.find(obstacle.index) != partitions.end())
                partition = partitions.at(obstacle.index);
            else
                partition = {0, 0.};

            if (SCENARIO_CONFIG.debug_output_ && partitions.find(obstacle.index) == partitions.end())
                SCENARIO_WARN("Sample Partition not found! (ID = " << obstacle.index << ")");

            // Retrieve the x y and observables for the partition as reference
            int safe_id = std::min(partition.id, (int)partitions_.size() - 1); // Sometimes we may not have loaded the new
                                                                               // partitions, so be safe here
            // SCENARIO_HOOK;

            auto &batch_x = partitions_[safe_id].GetSampleBatchX(); // [s, k]
            auto &batch_y = partitions_[safe_id].GetSampleBatchY();
            auto &batch_o = partitions_[safe_id].GetSampleBatchO(); // Observables

            // Retrieve the samples closest to the current observable within the selected partition
            online_partition_x_.clear();
            online_partition_y_.clear();
            online_partition_obs_.clear();

            SampleTrajectories(batch_x, batch_y, batch_o, &online_partition_x_, &online_partition_y_, &online_partition_obs_,
                               SCENARIO_CONFIG.sample_size_ + SCENARIO_CONFIG.sample_size_ % 2, // If odd add one (this fixes the rounding
                                                                                                // later)
                               (float)partition.velocity);

            output_scenarios[0][v][0] = Eigen::VectorXd::Ones(SCENARIO_CONFIG.sample_size_) * obstacle.position(0);
            output_scenarios[0][v][1] = Eigen::VectorXd::Ones(SCENARIO_CONFIG.sample_size_) * obstacle.position(1);

            // SCENARIO_HOOK;

            for (int k = 0; k < N; k++) // For all steps
            {
                int prev_k = k == 0 ? 0 : k - 1; // At 0 we use the initialized value, otherwise we use the previous positions
                                                 // in the trajectory

                for (int s = 0; s < SCENARIO_CONFIG.sample_size_; s++) // For all samples
                {
                    // Euler integrate the velocity to obtain a trajectory (taking the rotation of the pedestrian into account)
                    output_scenarios[k][v][0](s) =
                        output_scenarios[prev_k][v][0](s) + ((double)online_partition_x_[s][k] * std::cos(angle) -
                                                             (double)online_partition_y_[s][k] * std::sin(angle)) *
                                                                dt;

                    output_scenarios[k][v][1](s) =
                        output_scenarios[prev_k][v][1](s) + ((double)online_partition_x_[s][k] * std::sin(angle) +
                                                             (double)online_partition_y_[s][k] * std::cos(angle)) *
                                                                dt;
                }
            }
        }

        mutex_.unlock();
        SCENARIO_INFO_STREAM("Sampler: Real scenarios ready, format: [ k = "
                             << output_scenarios.size() << " | v = " << output_scenarios[0].size()
                             << " | s = " << output_scenarios[0][0][0].rows() << "]");
    }

    bool PartitionReader::Init(int index)
    {
        batch_x_.resize(0);
        batch_y_.resize(0);

        SCENARIO_WARN("Initializing JSON Trajectory Sampler [" << index << "]");

        file_path_ = getPackagePath("scenario_replay") + "/partitions/partition-" + std::to_string(index) + ".json"; // path to json samples

        return ReadSamples();
    }

    bool PartitionReader::ReadSamples()
    {
        // using namespace nlohmann;
        nlohmann::json j;
        int window = SCENARIO_CONFIG.N_;

        batch_x_.clear();
        batch_y_.clear();

        SCENARIO_INFO("\tJSON File: " << file_path_);
        std::ifstream file_read(file_path_);
        if (!file_read)
        {
            SCENARIO_ERROR_STREAM("Error in reading partitions from " << file_path_);
            return false;
        }

        // Start the file reading
        file_read >> j;
        std::vector<float> v2_x(window);
        std::vector<float> v2_y(window);

        // stores the observables, and the x, y trajectories of each sample in a separate vector
        // Oscar: Up until sample size for faster loading
        // for (int s = 0; s < std::min((int)j.size(), config_->sample_size_); s++)
        for (int s = 0; s < (int)j.size(); s++)
        {
            float observable_value = roundf(float(j[std::to_string(s)]["Observable"]) * 10) / 10;

            batch_o_.push_back(observable_value);
            for (int k = 0; k < window; k++)
            {
                v2_x[k] = (float(j[std::to_string(s)]["Trajectory X"]["x" + std::to_string(k)]));
                v2_y[k] = (float(j[std::to_string(s)]["Trajectory Y"]["y" + std::to_string(k)]));
            }

            batch_x_.push_back(v2_x);
            batch_y_.push_back(v2_y);
        }
        n_samples_ = batch_x_.size();
        SCENARIO_INFO("\tSamples: " << n_samples_);

        if (n_samples_ < SCENARIO_CONFIG.sample_size_)
        {
            // SCENARIO_ERROR("The sample size is larger than the samples in this partition!");

            // Make sure that we can still run if this happens, by copying the existing data until we have enough
            int start_size = (int)n_samples_;
            for (int i = start_size; i < SCENARIO_CONFIG.sample_size_; i++)
            {
                batch_x_.push_back(batch_x_[i % start_size]);
                batch_y_.push_back(batch_y_[i % start_size]);
                batch_o_.push_back(batch_o_[i % start_size]);
            }
        }
        return true;
    }

    template <typename A, typename B, typename C>
    void PartitionSampler::SampleTrajectories(std::vector<std::vector<A>> &a, std::vector<std::vector<B>> &b, std::vector<C> &c,
                                              std::vector<std::vector<A>> *ac, std::vector<std::vector<B>> *bc, std::vector<C> *cc,
                                              int samples, float Observable)
    {
        if ((int)c.size() < samples)
        {
            copy(a.begin(), a.end(), back_inserter(*ac));

            copy(b.begin(), b.end(), back_inserter(*bc));
            copy(c.begin(), c.end(), back_inserter(*cc));
        }
        else
        {
            // How does this set the upper bound?
            auto low_bound = std::lower_bound(c.begin(), c.end(), Observable);
            auto index_to_start_x = low_bound - c.begin();
            auto index_to_end_x = c.end() - low_bound;

            if (index_to_end_x >= samples / 2 && index_to_start_x >= samples / 2)
            {
                copy(&a[index_to_start_x - samples / 2], &a[index_to_start_x + samples / 2],
                     back_inserter(*ac)); // This is not safe, sample count may be odd
                copy(&b[index_to_start_x - samples / 2], &b[index_to_start_x + samples / 2], back_inserter(*bc));
                copy(&c[index_to_start_x - samples / 2], &c[index_to_start_x + samples / 2], back_inserter(*cc));
            }

            else if (index_to_end_x < samples / 2)
            {
                int samples_before = samples / 2 + (samples / 2 - index_to_end_x);
                int samples_after = samples / 2 - (samples / 2 - index_to_end_x);
                copy(&a[index_to_start_x - samples_before], &a[index_to_start_x + samples_after], back_inserter(*ac));
                copy(&b[index_to_start_x - samples_before], &b[index_to_start_x + samples_after], back_inserter(*bc));
                copy(&c[index_to_start_x - samples_before], &c[index_to_start_x + samples_after], back_inserter(*cc));
            }
            else if (index_to_start_x < samples / 2)
            {
                int samples_before = samples / 2 - (samples / 2 - index_to_start_x);

                int samples_after = samples / 2 + (samples / 2 - index_to_start_x);
                copy(&a[index_to_start_x - samples_before], &a[index_to_start_x + samples_after], back_inserter(*ac));
                copy(&b[index_to_start_x - samples_before], &b[index_to_start_x + samples_after], back_inserter(*bc));
                copy(&c[index_to_start_x - samples_before], &c[index_to_start_x + samples_after], back_inserter(*cc));
            }
        }
    }
};
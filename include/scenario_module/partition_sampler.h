#ifndef __PARTITION_SAMPLER_H__
#define __PARTITION_SAMPLER_H__

#include <scenario_module/types.h>

#include <mpc_planner_types/data_types.h>

#include <ros_tools/random_generator.h>

#include <third_party/ThreadPool.h>

#include <vector>

#include <unordered_map>
#include <future>
#include <mutex>

using namespace MPCPlanner;

#define PARTITION_READ_THREADS 2
namespace ScenarioModule
{
    // typedef std::vector<std::vector<Eigen::VectorXd>> SampleVector; // k | x/y | (s) location per obstacle and time step -X- -> No interaction: position along the trajectory, x/y

    /** @brief Class for reading partitions from JSON format. */
    class PartitionReader
    {
    public:
        PartitionReader() {};

    public:
        int n_samples_;

        bool Init(int index);

        std::vector<std::vector<float>> &GetSampleBatchX() { return batch_x_; }
        std::vector<std::vector<float>> &GetSampleBatchY() { return batch_y_; };
        std::vector<float> &GetSampleBatchO() { return batch_o_; };

    private:
        int data_size_;
        std::string file_path_;
        RosTools::RandomGenerator rand_;

        trajectory_sample batches_; // Subset of the database with the right size (batch, sample) (needs to copy because of the vectorxd class)

        int small;
        std::vector<float> batch_o_;
        std::vector<std::vector<float>> batch_x_;
        std::vector<std::vector<float>> batch_y_;

        bool ReadSamples();
    };

    class PartitionSampler
    {

    public:
        PartitionSampler();

    public:
        void UpdatePartitionData(int partition_size);

        void SamplePartitionedScenarios(const std::vector<DynamicObstacle> &obstacles,
                                        std::unordered_map<int, Partition> partitions,
                                        int partition_id, int num_partitions, double dt,
                                        std::vector<trajectory_sample> &output_scenarios);

    private:
        std::vector<PartitionReader> partitions_;

        int largest_sample_size_;

        std::mutex mutex_;
        std::unique_ptr<ThreadPool> thread_pool_;
        std::future<void> partition_future_;
        int partition_read_id_;

        std::vector<std::vector<float>> online_partition_x_, online_partition_y_;
        std::vector<float> online_partition_obs_;

        template <typename A, typename B, typename C>
        void SampleTrajectories(std::vector<std::vector<A>> &a, std::vector<std::vector<B>> &b, std::vector<C> &c,
                                std::vector<std::vector<A>> *ac, std::vector<std::vector<B>> *bc, std::vector<C> *cc,
                                int samples, float Observable);
    };

}

#endif // __PARTITION_SAMPLER_H__
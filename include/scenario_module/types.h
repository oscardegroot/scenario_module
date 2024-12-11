#ifndef __SCENARIO_TYPES_H__
#define __SCENARIO_TYPES_H__

#include <ros_tools/logging.h>

#include <mpc_planner_types/data_types.h>

#include <Eigen/Dense>

#include <vector>

using namespace MPCPlanner;

namespace ScenarioModule
{
    typedef std::vector<std::vector<Eigen::VectorXd>> trajectory_sample; // location per obstacle and time step

    enum class ScenarioSolveStatus
    {
        SUCCESS = 0,
        INFEASIBLE = 1,
        SUPPORT_EXCEEDED = 2,
        NONZERO_SLACK = 3
    };

    enum class ObstacleType
    {
        STATIC,
        DYNAMIC,
        RANGE
    };
    enum class ConstraintSide
    {
        BOTTOM,
        TOP,
        UNDEFINED
    };

    struct Scenario
    {
        int idx_;
        int obstacle_idx_;
    };

    struct ScenarioConstraint
    {
        // LinearConstraint2D constraint_; // Improve later

        Scenario *scenario_;

        ObstacleType type_;
        ConstraintSide side_;

        ScenarioConstraint(){};

        ScenarioConstraint(Scenario *scenario, const ObstacleType &type, const ConstraintSide &side)
        {
            scenario_ = scenario;
            type_ = type;
            side_ = side;
        }

        int GetHalfspaceIndex(int sample_size)
        {
            return type_ == ObstacleType::DYNAMIC ? sample_size * scenario_->obstacle_idx_ + scenario_->idx_ : scenario_->idx_;
        }
    };

    struct SupportSubsample
    {
        std::vector<int> support_indices_;
        std::vector<Scenario> scenarios_;

        int size_;

        SupportSubsample(int initial_size = 150)
        {
            size_ = 0;
            support_indices_.reserve(initial_size);
            scenarios_.reserve(initial_size);
        }

        void Add(const Scenario &scenario)
        {
            // No duplicates
            if (ContainsScenario(scenario))
                return;

            // Note: will allocate if above size!
            support_indices_.push_back(scenario.idx_);
            scenarios_.push_back(scenario);
            size_++;
        }

        void Reset()
        {
            size_ = 0;
            support_indices_.clear();
            scenarios_.clear();
        }

        bool ContainsScenario(const Scenario &scenario)
        {
            return (std::find(support_indices_.begin(), support_indices_.begin() + size_, scenario.idx_) !=
                    support_indices_.begin() + size_);
        }

        // Aggregate vector 2 into vector 1
        void MergeWith(const SupportSubsample &other)
        {
            for (int i = 0; i < other.size_; i++)
            {
                if (!ContainsScenario(other.scenarios_[i]))
                {
                    Add(other.scenarios_[i]);
                }
            }
        }

        void Print()
        {
            LOG_DIVIDER();
            LOG_INFO("Support Subsample");
            for (int i = 0; i < size_; i++)
            {
                LOG_VALUE("Scenario", scenarios_[i].idx_);
                LOG_VALUE("Obstacle", scenarios_[i].obstacle_idx_);
            }
            LOG_DIVIDER();
        }

        void PrintUpdate(int solver_id, int bound, int iterations)
        {
            LOG_INFO("[Solver " << solver_id << "] SQP (" << iterations << "): Support = " << size_ << "/" << bound);
        }
    };

    struct Partition
    {
        int id;
        double velocity;
    };
}

#endif // __TYPES_H__
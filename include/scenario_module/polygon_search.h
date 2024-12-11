/**
 * @file polygon_search.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Class for searching for an inner polytope from a large set of linear constraints
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef POLYGON_SEARCH_H
#define POLYGON_SEARCH_H

#include <scenario_module/types.h>

#include <Eigen/Dense>

#include <vector>
#include <memory>

#define POLYGON_BOTTOM_RANGE_CONSTRAINTS 2
#define POLYGON_TOP_RANGE_CONSTRAINTS 2

#define POLYGON_DEBUG 0
#if POLYGON_DEBUG == 1
#define POLYGON_LOG(x) std::cout << x << std::endl
#else
#define POLYGON_LOG(x) //
#endif
namespace ScenarioModule
{
    // Ideally: we have y here as array and we compute lambda * y_left + (1 - lambda * y_right)
    // The problem, currently, is that we need to sort the indices and if we then sort y, then we lose our map from indices to y.
    // So either we have some sort of object that holds y and indices, but also allows to use y as array, or we need some additional structure
    // to preserve the map.
    // Next idea to try: have an extra array for y that can be reused and holds every value where it needs to be! and a sorted version as second
    struct VerticalEvaluation
    {

        std::vector<int> indices_;
        int top_idx_;
        double top_y_;

        void AllocateSpace(int size)
        {
            indices_.reserve(size);
        }

        void Clear()
        {
            indices_.clear();
            top_idx_ = -1;
        }
    };

    /* Class for memory handling during recursion */
    /* Since we need the evaluate y values throughout, but their length decreases over evaluations we need some class that can provide preallocated memory
        on command, but in such a way that we do not allocate excessive amounts of data. This implementation has "steps" levels that decreases in size by
        a factor 4 per step to save data. The input size is directly converted to the required index */
    class RecursionMemory
    {

    public:
        RecursionMemory(int max_constraints, int steps, int expected_constraints);

    private:
        std::vector<int> usage_indices_; // To indicate how far we are in the data
        std::vector<std::vector<VerticalEvaluation>> data_;

        int max_constraints_, steps_, expected_constraints_;

    public:
        void Reset();
        VerticalEvaluation &GetStorage(int size);
        VerticalEvaluation &GetCopy(VerticalEvaluation &data);
    };

    // Note: The search uses a formulation in lambda. We compute y_left and y_right. All constraints between have
    // y = (1 - lambda) * y_left + lambda * y_right,
    // x = lambda * x_left + (1 - lambda) * x_right

    class PolygonSearch
    {
    public:
        PolygonSearch(std::vector<ScenarioConstraint> *halfspaces, Eigen::ArrayXd *y_left, Eigen::ArrayXd *y_right, int n_halfspaces, double range);

        int n_halfspaces_;
        double range_;
        double check_range_;

    private:
        // Memory
        std::unique_ptr<RecursionMemory> recursion_memory_;

        // Input constraints
        std::vector<ScenarioConstraint> *halfspaces_;

        // Peeling variable
        std::vector<bool> is_removed_;

        // initial index sets
        std::vector<int> indices_top_;
        std::vector<int> indices_bot_;

        // Y data initially
        Eigen::ArrayXd *y_left_;
        Eigen::ArrayXd *y_right_;

        double x_left_;
        double x_right_;

        // Intersections initially
        Eigen::Vector2d right_intersect_, left_intersect_;

        // Range variables
        Eigen::MatrixXd A_range_;
        Eigen::VectorXd b_range_;

        // Intersection variables
        Eigen::MatrixXd intersection_matrix_;
        double det_denom_;

        int max_recursions_;

        // Halfspaces to add
        /* Compute Ax - b for all halfspaces at the given x and sort their indices on dominance */
        VerticalEvaluation &EvaluateVerticallyAt(const double x, const std::vector<int> &indices_in, const ConstraintSide &side);

        // Same but where we only want indices with a value above y
        VerticalEvaluation &EvaluateVerticallyAt(const double x, const double y, const VerticalEvaluation &indices_in, const ConstraintSide &side);

        // Find the top constraints initially when given the y values
        VerticalEvaluation &FindTopConstraint(const std::vector<int> &indices_in, const Eigen::ArrayXd *y_values, const ConstraintSide &side);

        // Find the intersection between two lines
        bool findIntersect(const int &a, const int &b, Eigen::Vector2d &intersect_out);

        // Main search function. A recursive search tree that splits the space to find all intersections.
        void RecursiveSearch(int top_index_left, int top_index_right, const VerticalEvaluation &indices, const Eigen::Vector2d &intersect_left, const Eigen::Vector2d &intersect_right,
                             int n, const ConstraintSide &side, std::vector<int> &indices_out, std::vector<Eigen::Vector2d> &intersects_out);

        // Search on the top or bottom (calls recursivesearch)
        void SearchSide(const Eigen::Vector2d &pose, const ConstraintSide &side);

        // Connect top and bottom lines
        bool ConstructPolygon();
        bool SweepTopAndBottom(bool sweep_from_left, Eigen::Vector2d &intersect_out, int &top_idx, int &bot_idx);

    public:
        // The scenario constraints that make up the inner polygon and its intersections
        bool polygon_is_empty_;
        std::vector<ScenarioConstraint *> polygon_out_;
        std::vector<Eigen::Vector2d *> intersects_out_;

        // Debug public -> move to private
        // Resulting constraints and intersects
        std::vector<int> result_indices_top_;
        std::vector<int> result_indices_bot_;
        std::vector<Eigen::Vector2d> result_intersects_top_;
        std::vector<Eigen::Vector2d> result_intersects_bot_;

        // Main function that does the search, after this the polygon and its intersections are available in polygon_out_ and intersects_out_
        void Reset();

        // Remove the given constraint from the search (not efficiently implemented yet!)
        void RemoveConstraint(const Scenario &scenario);
        bool WasRemoved(const Scenario &scenario) { return is_removed_[scenario.idx_]; };

        // Search function called by first search and peel
        bool Search(const Eigen::Vector2d &pose, double orientation, double x_left, double x_right);

        // Add a set of box constraints around the pose
        void AddRangeConstraints(const Eigen::Vector2d &pose, const double orientation, Eigen::ArrayXd &a1, Eigen::ArrayXd &a2, Eigen::ArrayXd &b);

        void PrintResult();
    };
};
#endif
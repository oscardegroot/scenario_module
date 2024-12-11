#include "scenario_module/polygon_search.h"
#include <scenario_module/logging.h>

#include <ros_tools/math.h>

namespace ScenarioModule
{
  // There should be only one of these classes for all calls to it!

  /* Class for memory handling */
  // Todo: This should be one class for all polygon searches!
  RecursionMemory::RecursionMemory(int max_constraints, int steps, int expected_constraints)
      : max_constraints_(max_constraints), steps_(steps), expected_constraints_(expected_constraints)
  {
    // Data[Storage Level][Storage Index] holds a vector of SweepValue data of at most storage level entries
    data_.resize(steps);
    for (size_t i = 0; i < data_.size(); i++)
    {
      if (i == 0)
        data_[i].resize(2 * expected_constraints);
      else if (i == data_.size() - 1)
        data_[i].resize(2 * expected_constraints);
      else
        data_[i].resize(expected_constraints);

      for (size_t j = 0; j < data_[i].size(); j++)
      {
        data_[i][j].AllocateSpace(std::ceil(max_constraints / (std::pow(2, 2 * (double)i)))); // Heuristic
      }
    }
    usage_indices_.resize(steps);
    for (int j = 0; j < steps; j++)
      usage_indices_[j] = 0;
  }

  void RecursionMemory::Reset()
  {
    for (size_t i = 0; i < data_.size(); i++)
    {
      for (size_t j = 0; j < data_[i].size(); j++)
        data_[i][j].Clear();
    }

    for (size_t i = 0; i < usage_indices_.size(); i++)
      usage_indices_[i] = 0;
  }

  VerticalEvaluation &RecursionMemory::GetStorage(int size)
  {
    // Seems complicated, but is just a one way map to get the index
    int storage_level = std::floor(0.5 * std::log2(max_constraints_ / (double)size));
    storage_level = std::min(storage_level, (int)usage_indices_.size() - 1); // Cap

    // std::cout << "Requesting storage for " << size << " stored at level " << storage_level << ", usage index: " <<
    // usage_indices_[storage_level] << std::endl;

    // Increase the usage counter
    usage_indices_[storage_level]++;

    // If we have exceeded our current size
    if (usage_indices_[storage_level] - 1 > (int)data_[storage_level].size() - 1)
    {
      // Give a warning and use a different level (will cause an allocation)
      SCENARIO_WARN_STREAM("Polygon Search: Exceeded allocated size at storage level "
                           << storage_level << " of (" << data_[storage_level].size() << ")"
                           << "! Returning a slot at a lower level, please increase storage size to speed up the program.");
      return GetStorage(size * 2);
      // usage_indices_[storage_level - 1]++;
      // return data_[storage_level - 1][usage_indices_[storage_level - 1] - 1]; // We return storage in the previous
      // level
    }

    // Return the data
    return data_[storage_level][usage_indices_[storage_level] - 1];
  }

  // Copy data from one set into another
  VerticalEvaluation &RecursionMemory::GetCopy(VerticalEvaluation &data)
  {
    VerticalEvaluation &new_data = GetStorage(data.indices_.size());

    // Copy data
    new_data.indices_ = data.indices_;
    new_data.top_y_ = data.top_y_;
    new_data.top_idx_ = data.top_idx_;

    return new_data;
  }

  PolygonSearch::PolygonSearch(std::vector<ScenarioConstraint> *halfspaces, Eigen::ArrayXd *y_left,
                               Eigen::ArrayXd *y_right, int n_halfspaces, double range)
      : n_halfspaces_(n_halfspaces), range_(range)
  {
    halfspaces_ = halfspaces;

    // Initialize data storage
    recursion_memory_.reset(new RecursionMemory(n_halfspaces_, 4, 50));

    // Limit recursions internally
    max_recursions_ = 14;

    is_removed_.resize(n_halfspaces_, false);

    indices_top_.reserve(n_halfspaces); // Includes space for range
    indices_bot_.reserve(n_halfspaces);

    // Reserve space for the output
    int output_size = 250;
    result_indices_top_.reserve(output_size);
    result_indices_bot_.reserve(output_size);
    result_intersects_top_.reserve(output_size - 1);
    result_intersects_bot_.reserve(output_size - 1);
    polygon_out_.reserve(output_size);
    intersects_out_.reserve(output_size - 1);

    // Resize to the maximum possible size
    y_left_ = y_left;
    y_right_ = y_right;

    // Variables for range constraints
    A_range_ = Eigen::MatrixXd::Zero(4, 2);
    b_range_ = Eigen::VectorXd::Zero(4);
    check_range_ = range_ * std::sqrt(2);

    polygon_is_empty_ = false;

    intersection_matrix_ = Eigen::MatrixXd(2, 2);
  }

  void PolygonSearch::AddRangeConstraints(const Eigen::Vector2d &pose, const double orientation, Eigen::ArrayXd &a1,
                                          Eigen::ArrayXd &a2, Eigen::ArrayXd &b)
  {
    // Ensure non zero orientation
    double nonzero_orientation = std::abs(orientation) < 1e-5 ? 1e-5 : orientation;

    Eigen::MatrixXd rotation_matrix = RosTools::rotationMatrixFromHeading(nonzero_orientation);

    // Setup the direction of the range constraints to align with the vehicle
    A_range_ << rotation_matrix * Eigen::MatrixXd::Identity(2, 2), -rotation_matrix * Eigen::MatrixXd::Identity(2, 2);

    // Define their offset
    b_range_ << pose(0), pose(1), -pose(0), -pose(1);
    b_range_.block<2, 1>(0, 0) = rotation_matrix * b_range_.block<2, 1>(0, 0) + Eigen::Vector2d(range_, range_);
    b_range_.block<2, 1>(2, 0) = rotation_matrix * b_range_.block<2, 1>(2, 0) + Eigen::Vector2d(range_, range_);

    for (u_int i = 0; i < 4; i++)
    {
      int index = a1.size() - i - 1; // Assume range at the end -> REPLACE BY INPUT

      a1(index) = A_range_(i, 0);
      a2(index) = A_range_(i, 1);
      b(index) = b_range_(i);
    }
  }

  // Search on the top or bottom side for the dominant line
  void PolygonSearch::SearchSide(const Eigen::Vector2d &pose, const ConstraintSide &side)
  {
    // Pick the correct variables based on this being top or bottom
    std::vector<int> &indices = side == ConstraintSide::BOTTOM ? indices_bot_ : indices_top_;

    std::vector<int> &result_indices = side == ConstraintSide::BOTTOM ? result_indices_bot_ : result_indices_top_;
    std::vector<Eigen::Vector2d> &result_intersects =
        side == ConstraintSide::BOTTOM ? result_intersects_bot_ : result_intersects_top_;

    /* Clear values from previous iterations */
    indices.clear();
    result_indices.clear();
    result_intersects.clear();

    recursion_memory_->Reset();

    // Initialize the indices -> which refer to the scenario definition
    for (size_t i = 0; i < halfspaces_->size(); i++)
    {
      if ((*halfspaces_)[i].side_ == side /* Only use constraints that are on the considered side */
          && !((*halfspaces_)[i].type_ == ObstacleType::DYNAMIC &&
               is_removed_[(*halfspaces_)[i].scenario_->idx_])) /* Do not add removed constraints */
      {
        indices.push_back(i);
      }
    }

    // std::cout << "(size = " << indices.size() << ")" << std::endl;
    // Compute the values on the left and right of the vehicle at a distance of range
    VerticalEvaluation &values_left = FindTopConstraint(indices, y_left_, side);
    VerticalEvaluation &values_right = FindTopConstraint(indices, y_right_, side);

    // The initial intersections are at the evaluated x and the y of the most dominant lines
    Eigen::Vector2d intersect_left(pose(0) - check_range_ - 1e-8, values_left.top_y_);
    Eigen::Vector2d intersect_right(pose(0) + check_range_ + 1e-8, values_right.top_y_);

    // If there is only one line, stop immediately
    if (values_left.top_idx_ == values_right.top_idx_)
    {
      POLYGON_LOG("There is only one relevant constraint, returning that constraint and no intersections!");
      result_indices.push_back(values_left.top_idx_);
    }
    else
    {
      POLYGON_LOG("Starting Recursive Search");
      // Otherwise we recursively search for intersections and their lines
      RecursiveSearch(values_left.top_idx_, values_right.top_idx_, values_left, intersect_left, intersect_right, 0, side,
                      result_indices, result_intersects);

      // std::cout << "Results: \n";
      // for (size_t i = 0; i < result_indices.size(); i++)
      // {
      //     std::cout << "Line: " << result_indices[i] << "\n";

      //     if (i != result_indices.size() - 1)
      //         std::cout << "Intersection: " << result_intersects[i](0) << ", " << result_intersects[i](1) << std::endl;
      // }

      // // For matlab debug
      // std::cout << "intersections = [";

      // for (size_t i = 0; i < result_intersects.size(); i++){
      //     if(i != result_intersects.size() - 1)
      //         std::cout << result_intersects[i](0) << ", ";
      //     else
      //         std::cout << result_intersects[i](0) << ";";
      // }
      // for (size_t i = 0; i < result_intersects.size(); i++)
      // {
      //     if (i != result_intersects.size() - 1)
      //         std::cout << result_intersects[i](1) << ", ";
      //     else
      //         std::cout << result_intersects[i](1) << "]\n";
      // }
    }
  }

  // Remove a scenario from the search (works via a flag)
  void PolygonSearch::RemoveConstraint(const Scenario &scenario)
  {
    is_removed_[scenario.idx_] = true;
  }

  // Reset the removed constraints
  void PolygonSearch::Reset()
  {
    polygon_is_empty_ = false;

    std::fill(is_removed_.begin(), is_removed_.end(), false);
  }

  bool PolygonSearch::Search(const Eigen::Vector2d &pose, double orientation, double x_left, double x_right)
  {
    x_left_ = x_left;
    x_right_ = x_right;

    // Search for the lines on the bottom part of the polygon
    POLYGON_LOG("Bottom search");
    SearchSide(pose, ConstraintSide::BOTTOM);
    POLYGON_LOG("Found " << result_indices_bot_.size() << " bottom lines and " << result_intersects_bot_.size()
                         << " intersects");

    // Search for the lines on the top part of the polygon
    POLYGON_LOG("Top search");
    SearchSide(pose, ConstraintSide::TOP);
    POLYGON_LOG("Found " << result_indices_top_.size() << " top lines and " << result_intersects_top_.size()
                         << " intersects");

    // Combine top and bottom
    bool success = ConstructPolygon();
    return success;
  }

  bool PolygonSearch::ConstructPolygon()
  {
    polygon_out_.clear();
    intersects_out_.clear();
    recursion_memory_->Reset();

    int left_top, left_bot, right_top, right_bot;
    bool success = true;
    bool temp_success;

    // Sweep from the left, finding the intersection where the bottom and top lines should start
    POLYGON_LOG("Left Sweep with " << result_intersects_bot_.size() << " bottom intersects, and "
                                   << result_intersects_top_.size() << " top intersects!\n");
    temp_success = SweepTopAndBottom(true, left_intersect_, left_top, left_bot);
    success = success && temp_success;
    // Sweep from the right, finding the intersection where the bottom and top lines should end
    POLYGON_LOG("Right Sweep with " << result_intersects_bot_.size() << " bottom intersects, and "
                                    << result_intersects_top_.size() << " top intersects!\n");
    temp_success = SweepTopAndBottom(false, right_intersect_, right_top, right_bot);
    success = success && temp_success;

    // Construct the final polygon by connecting the parts by the found intersections
    intersects_out_.push_back(&right_intersect_); // Right

    for (int i = right_top + 1; i >= left_top; i--)
    { /* Top (note flipped!)*/

      polygon_out_.push_back(&(*halfspaces_)[result_indices_top_[i]]);
      if (i != right_top + 1)
        intersects_out_.push_back(&result_intersects_top_[i]);
    }

    intersects_out_.push_back(&left_intersect_); // Left

    for (int i = left_bot; i <= right_bot + 1; i++)
    { /* Bottom */
      polygon_out_.push_back(&(*halfspaces_)[result_indices_bot_[i]]);
      if (i != right_bot + 1)
        intersects_out_.push_back(&result_intersects_bot_[i]);
    }

    return success;
    // polygon_out_ should now contain the ScenarioConstraints that span the polygon
    // intersects_out_ should have all the intersects of the polygon, mostly for visual purposes

    // Debug output
    // std::cout << "==== Result ====\n";
    // std::cout << "   Polygon:\n";
    // for(size_t i = 0; i < polygon_out_.size(); i++){
    //     if (polygon_out_[i]->type_ == ObstacleType::DYNAMIC)
    //         std::cout << "   Scenario: " << polygon_out_[i]->scenario_->idx_ << std::endl;
    //     else
    //         std::cout << "   Scenario: " << "RANGE" << std::endl;
    // }
    // std::cout << "   Intersections:\n";
    // for(size_t i = 0; i < intersects_out_.size(); i++){
    //     std::cout << "   Intersect: (" << (*intersects_out_[i])(0) << ", " << (*intersects_out_[i])[1] << ")" <<
    //     std::endl;
    // }
  }

  // direction 1 is right, -1 is left
  // Goal of this function is to connect the top lines and bottom lines
  // This is done by sweeping from left to right and right to left and checking when one of the lines exceeds
  // one of the other lines on the opposite side, IN the search domain.
  bool PolygonSearch::SweepTopAndBottom(bool sweep_from_left, Eigen::Vector2d &intersect_out, int &top_idx, int &bot_idx)
  {
    // Go through all intersections
    int direction;
    double next_x;
    bool top_intersect_is_next;
    double tolerance = 1e-8;
    // Two cases for the sweep
    if (sweep_from_left)
    {
      // Start from the start of the indices
      top_idx = 0;
      bot_idx = 0;
      direction = 1;
    }
    else
    {
      // Start from the end of the indices
      top_idx = result_intersects_top_.size() - 1;
      bot_idx = result_intersects_bot_.size() - 1;
      direction = -1;
    }

    // Go through all intersects? -> shouldn't this be lines? (2)

    for (size_t i = 0; i < result_intersects_top_.size() + result_intersects_bot_.size(); i++)
    {
      // If there are no more top indices, we shouldn't check the top indices
      bool no_top_indices = (sweep_from_left && top_idx >= (int)result_intersects_top_.size()) ||
                            (!sweep_from_left && top_idx < 0); // Then we should go to bottom
                                                               // I should also have this condition for the bottom case?
      // If there are no more top indices, we shouldn't check the top indices
      bool no_bot_indices = (sweep_from_left && bot_idx >= (int)result_intersects_bot_.size()) ||
                            (!sweep_from_left && bot_idx < 0); // Then we should go to bottom

      // If the next intersection on top is first on the x axis (i.e., the axis that we are sweeping)
      if (no_bot_indices ||
          (!no_top_indices && direction * (result_intersects_top_[top_idx](0) - result_intersects_bot_[bot_idx](0)) < 0))
      {
        // The next x is at the top intersection
        next_x = result_intersects_top_[top_idx](0) - (tolerance * direction);
        top_idx += direction;
        top_intersect_is_next = true;
      }
      else
      { /* The bottom axis has the next intersection */

        // The next x is at the bottom intersection
        next_x = result_intersects_bot_[bot_idx](0) - (tolerance * direction);
        bot_idx += direction;
        top_intersect_is_next = false;
      }

      // Evaluate the y values of all found lines on the top and the bottom at the given x
      // Note that we evaluate just before the intersection (tolerance) to ensure that the order is consistent (at the
      // intersection two lines have the same value)
      VerticalEvaluation &top_values = EvaluateVerticallyAt(next_x, result_indices_top_, ConstraintSide::TOP);
      VerticalEvaluation &bot_values = EvaluateVerticallyAt(next_x, result_indices_bot_, ConstraintSide::BOTTOM);

      // If we have entered the polygon (i.e., the top line is greater than the bottom line),
      // then the intersection that connects the two is the intersection between the bottom and top lines that we have
      // currently Note that top y is just the highest or lowest value depending on bot or top
      if (top_values.top_idx_ == -1)
        ROS_ERROR_STREAM("Top idx value was not right!");
      else if (bot_values.top_idx_ == -1)
        ROS_ERROR_STREAM("Bot idx value was not right!");

      // std::cout << top_values.top_y_ << " > " <<  bot_values.top_y_ << "?" << std::endl;
      if (top_values.top_y_ > bot_values.top_y_)
      {
        // Find the intersection
        findIntersect(top_values.top_idx_, bot_values.top_idx_, intersect_out);

        // Reset the last increase of bot or top idx
        if (top_intersect_is_next)
          top_idx -= direction;
        else
          bot_idx -= direction;

        return true;
      }
    }

    // Error handling for when the top and bottom line cannot be connected (happens when the vehicle is not an interior
    // point!)
    //    throw std::runtime_error("Polygon Search: Could not connect top and bottom lines in the polygon!");
    // ROS_ERROR("Polygon Search: Could not connect top and bottom lines in the polygon!");
    polygon_is_empty_ = true;

    if (sweep_from_left)
    {
      // Start from the start of the indices
      top_idx = 0;
      bot_idx = 0;
    }
    else
    {
      // Start from the end of the indices
      top_idx = result_intersects_top_.size() - 1;
      bot_idx = result_intersects_bot_.size() - 1;
    }
    intersect_out = Eigen::Vector2d(0, 0);
    return false;
  }

  // Main search
  // Fix left / right by heaving the top idxes instead of all indices and only one indices array.
  void PolygonSearch::RecursiveSearch(int top_index_left, int top_index_right, const VerticalEvaluation &indices,
                                      const Eigen::Vector2d &intersect_left, const Eigen::Vector2d &intersect_right,
                                      int n, const ConstraintSide &side, std::vector<int> &indices_out,
                                      std::vector<Eigen::Vector2d> &intersects_out)
  {
    POLYGON_LOG("Node [" << n << "] " << top_index_left << "-" << top_index_right);

    //   if (top_index_left == top_index_right)
    //   {
    // if (!(indices_out.size() > 0 && indices_out.back() == top_index_left))
    //   indices_out.push_back(top_index_left);  // Check if it is in the list

    // return;
    //   }

    // Intersect the left and right line
    Eigen::Vector2d intersect;
    bool do_intersect = findIntersect(top_index_left, top_index_right, intersect);

    // If the lines are parallel and do not intersect (can happen when scenarios have duplicates)
    if (!do_intersect)
    {
      // SCENARIO_WARN_STREAM("Lines are parallel!");
      // std::cout << top_index_left << ", " << top_index_right << std::endl;
      // std::cout << (*y_left_)(top_index_left) << ", " << (*y_left_)(top_index_right) << std::endl; // Debug
    }
    else
    {
      POLYGON_LOG("intersection: (" << intersect(0) << ", " << intersect(1) << ")");
    }

    // If the recursion does not terminate, terminate it after max_recursion steps
    if (n > max_recursions_ || !do_intersect)
    {
      SCENARIO_WARN("PolygonSearch: Recursion was terminated early!");

      // Add as if this is a leave node (note that this can cause errors if max_recursion is too small)
      // Maybe simply throw an error here
      if (!(indices_out.size() > 0 && indices_out.back() == top_index_left))
        indices_out.push_back(top_index_left);

      indices_out.push_back(top_index_right);
      intersects_out.push_back(intersect);
      return;
    }

    // Now evaluate the y values of relevant indices at the x of the intersection
    VerticalEvaluation &values =
        EvaluateVerticallyAt(intersect(0), intersect(1), indices, side); // Todo: fix n_top also in the data management

    int line_count = values.indices_.size();

    POLYGON_LOG("Line count: " << line_count);

    // If we are not terminating, then recursively call this function again for the left and/or right respectively
    if (line_count != 0)
    {
      // Search to the left first
      RecursiveSearch(top_index_left, values.top_idx_, values, intersect_left, intersect, n + 1, side, indices_out,
                      intersects_out);

      // Search to the right
      RecursiveSearch(values.top_idx_, top_index_right, values, intersect, intersect_right, n + 1, side, indices_out,
                      intersects_out);
    }
    else
    {
      // If this is a leaf node

      // Save the 2 lines and their intersect (Note: push_back is okay because we search left first (depth))
      // For the lines, we expect overlap, so if our left line is already in the list, we can skip it
      if (!(indices_out.size() > 0 && indices_out.back() == top_index_left))
        indices_out.push_back(top_index_left);

      indices_out.push_back(top_index_right);
      intersects_out.push_back(intersect);
    }
  }

  VerticalEvaluation &PolygonSearch::EvaluateVerticallyAt(const double x, const std::vector<int> &indices_in,
                                                          const ConstraintSide &side)
  {
    // PROFILE_FUNCTION();

    VerticalEvaluation &data = recursion_memory_->GetStorage(indices_in.size());

    double value; // Current y
    data.top_idx_ = -1;

    Eigen::ArrayXd &y_left = *y_left_;
    Eigen::ArrayXd &y_right = *y_right_;

    double range_full = x_right_ - x_left_;
    double lambda = (x - x_left_) / range_full;

    if (side == ConstraintSide::BOTTOM)
    {
      data.top_y_ = -1e10;

      // Compute y = -(b + a1*x)/a2, for all indices (this is relatively expensive but only happens twice)
      for (size_t i = 0; i < indices_in.size(); i++)
      {
        int idx = indices_in[i];
        value = (1.0 - lambda) * y_left(idx) + lambda * y_right(idx);
        // Only ush back indices if they lie above the y of the intersection

        data.indices_.push_back(idx);
        if (value > data.top_y_)
        {
          data.top_y_ = value;
          data.top_idx_ = idx;
        }
      }
    }
    else
    {
      data.top_y_ = 1e10;
      // Compute y = -(b + a1*x)/a2, for all indices (this is relatively expensive but only happens twice)
      for (size_t i = 0; i < indices_in.size(); i++)
      {
        int idx = indices_in[i];
        value = (1.0 - lambda) * y_left(idx) + lambda * y_right(idx);

        data.indices_.push_back(idx);
        if (value < data.top_y_)
        {
          data.top_y_ = value;
          data.top_idx_ = idx;
        }
      }
    }

    return data;
  }

  // THIS IS THE CONTINUOUS EVALUATION
  VerticalEvaluation &PolygonSearch::EvaluateVerticallyAt(const double x, const double y,
                                                          const VerticalEvaluation &indices_in,
                                                          const ConstraintSide &side)
  {
    // PROFILE_FUNCTION();
    VerticalEvaluation &data = recursion_memory_->GetStorage(indices_in.indices_.size());

    // Fill left and right
    double value; // Current y
    data.top_idx_ = -1;

    Eigen::ArrayXd &y_left = *y_left_;
    Eigen::ArrayXd &y_right = *y_right_;

    double range_full = x_right_ - x_left_;
    double lambda = (x - x_left_) / range_full;

    if (side == ConstraintSide::BOTTOM)
    {
      data.top_y_ = -1e10;

      // Compute y = -(b + a1*x)/a2, for all indices (this is relatively expensive but only happens twice)
      for (size_t i = 0; i < indices_in.indices_.size(); i++)
      {
        int idx = indices_in.indices_[i];
        value = (1.0 - lambda) * y_left(idx) + lambda * y_right(idx);
        // Only ush back indices if they lie above the y of the intersection
        if (value > y + 1e-6)
        {
          data.indices_.push_back(idx);
          if (value > data.top_y_)
          {
            data.top_y_ = value;
            data.top_idx_ = idx;
          }
        }
      }
    }
    else
    {
      data.top_y_ = 1e10;
      // Compute y = -(b + a1*x)/a2, for all indices (this is relatively expensive but only happens twice)
      for (size_t i = 0; i < indices_in.indices_.size(); i++)
      {
        int idx = indices_in.indices_[i];
        value = (1.0 - lambda) * y_left(idx) + lambda * y_right(idx);

        // Only ush back indices if they lie above the y of the intersection
        if (value < y - 1e-6)
        {
          data.indices_.push_back(idx);
          if (value < data.top_y_)
          {
            data.top_y_ = value;
            data.top_idx_ = idx;
          }
        }
      }
    }

    POLYGON_LOG("TOP Value: " << data.top_idx_ << " -> " << data.top_y_);

    // SortVerticalEvaluation(data, is_bottom);

    return data;
  }

  // THIS IS THE INITIAL EVALUATION!
  VerticalEvaluation &PolygonSearch::FindTopConstraint(const std::vector<int> &indices_in, const Eigen::ArrayXd *y_values,
                                                       const ConstraintSide &side)
  {
    // PROFILE_SCOPE("Initial Y Evaluation");
    VerticalEvaluation &data = recursion_memory_->GetStorage(indices_in.size());

    double value;
    if (side == ConstraintSide::BOTTOM)
    {
      data.top_y_ = -1e10;
      // Compute y = -(b + a1*x)/a2, for all indices (this is relatively expensive but only happens twice)
      int idx;
      for (size_t i = 0; i < indices_in.size(); i++)
      {
        idx = indices_in[i];
        // std::cout << "idx: " << idx << std::endl;
        value = (*y_values)(idx);
        // std::cout << "value: " << value << std::endl;

        // Add all indices, and find the top value
        data.indices_.push_back(idx);
        if (value > data.top_y_)
        {
          data.top_y_ = value;
          data.top_idx_ = idx;
        }
      }
    }
    else
    {
      data.top_y_ = 1e10;

      // Compute y = -(b + a1*x)/a2, for all indices (this is relatively expensive but only happens twice)
      int idx;
      for (size_t i = 0; i < indices_in.size(); i++)
      {
        idx = indices_in[i];
        value = (*y_values)(idx);

        // Add all indices, and find the top value
        data.indices_.push_back(idx);
        if (value < data.top_y_)
        {
          data.top_y_ = value;
          data.top_idx_ = idx;
        }
      }
    }

    // Return the sorted result
    return data;
  }

  // Find an intersection using 2D matrix inversion
  bool PolygonSearch::findIntersect(const int &a, const int &b, Eigen::Vector2d &intersect_out)
  {
    // std::cout << "intersect: " <<  (*y_left_)(a) << " | " <<   (*y_left_)(b) << " | " << (*y_right_)(a) << " | " <<
    // (*y_right_)(b) << std::endl; double r = (*y_right_)(a) - (*y_right_)(b);
    double r = (*y_left_)(a) - (*y_left_)(b);
    det_denom_ = r + (*y_right_)(b) - (*y_right_)(a);

    // Lines are parallell, there are no intersects
    if (std::abs(det_denom_) < 1e-8)
    {
      // std::cout << det_denom_ << std::endl;
      return false;
    }
    // Find at what lambda the lines meet
    double lambda = r / det_denom_; // std::cout << "lambda in intersect: " << lambda << std::endl;
    // double range_full = x_left_ - x_right_;
    // double lambda = (x - x_right_) / range_full;
    intersect_out(0) = (1.0 - lambda) * x_left_ + lambda * x_right_;
    intersect_out(1) = (1.0 - lambda) * ((*y_left_)(a)) + lambda * ((*y_right_)(a));

    return true;
  }

  void PolygonSearch::PrintResult()
  {
    std::cout << "Polygon: ";
    for (ScenarioConstraint *constraint : polygon_out_)
    {
      std::cout << constraint->scenario_->idx_ << ", ";
    }

    std::cout << ".\n";
  }
}
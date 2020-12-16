#include <ino_planner/ino_planner.hpp>
#include <ino_planner/simple_costmap_model.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt64.h>


PLUGINLIB_EXPORT_CLASS(ino_planner::InoPlanner, nav_core::BaseGlobalPlanner)
using namespace ino_planner;
using namespace dynamic_reconfigure;
using namespace std;


/**
 * @brief Prepare this planner for planning
 * @param The costmap to use for planning
 */
void InoPlanner::initialize(std::string, costmap_2d::Costmap2DROS* costmap_ros)
{
  // Check if the planner was previously initialized
  if (initialized_)
  {
    ROS_WARN("This planner is already initialized.");
    return;
  }

  // Create publisher for published produced path for information purpose
  ros::NodeHandle nh("ino_planner");
  path_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1, true);

  // Gather costmap and create world model
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  world_model_ = std::make_unique<SimpleCostmapModel>(*costmap_);

  // Setup dynamic reconfigure
  dsrv_ = make_unique<Server<InoPlannerConfig>>(nh);
  Server<InoPlannerConfig>::CallbackType callback = boost::bind(&InoPlanner::reconfigureCallback, this, _1, _2);
  dsrv_->setCallback(callback);

  initialized_ = true;
  ROS_INFO("Initialized ino_planner.");
}


/**
 * @brief Applies the received configuration to this planner
 *
 * @param config The new configuration
 * @param level Indicate which parameter changed
 */
void InoPlanner::reconfigureCallback(InoPlannerConfig &config, uint32_t level)
{
  config_ = config;
}


/**
 * @brief Update the simplified footprint from the costmap's footprint
 *
 * This function assumes a rectangular footprint with the longer side alongs the x axis.
 * The footprint must be symetrical along the x axis (-y = y).
 */
void InoPlanner::updateFootprint()
{
    geometry_msgs::Point point;
    simplified_footprint_.clear();

    // Find inscribed radius from footprint points
    costmap_2d::calculateMinAndMaxDistances(costmap_ros_->getRobotFootprint(), inscribed_radius_, circumscribed_radius_);

    // Simplified points are at (+-inscribed_radius, 0)
    point.x = circumscribed_radius_ - inscribed_radius_;
    point.y = 0;
    simplified_footprint_.push_back(point);

    point.x = -point.x;
    simplified_footprint_.push_back(point);
}


/**
 * @brief Create a plan from start to goal using the A* algorithm
 * @param start Start pose of the robot in world coordinates
 * @param goal Goal pose of the robot in world coordinates
 * @param plan The plan computed by the planner
 * @return true is the planning succeeded
 */
bool InoPlanner::makePlan(
  const geometry_msgs::PoseStamped& start,
  const geometry_msgs::PoseStamped& goal,
  std::vector<geometry_msgs::PoseStamped>& plan)
{
  ROS_INFO("Got goal.");
  updateFootprint();

  unsigned int mx, my;

  // Check if starting pose is inside the map
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my))
  {
    ROS_WARN("The starting pose is outside of the map. Planning will always fail.");
    return false;
  }

  // Extract yaw from start pose then create matching graph location
  tf::Quaternion q(
    start.pose.orientation.x,
    start.pose.orientation.y,
    start.pose.orientation.z,
    start.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  int start_yaw = (static_cast<int>(yaw * 180.0 / M_PI) + 360) % 360;

  ROS_INFO("Starting yaw is %d", start_yaw);

  GridPose grid_start(GridLocation(static_cast<int>(mx), static_cast<int>(my)), start_yaw, 9, 0);
  grid_start.is_start_ = true;

  // Check if goal pose is inside the map
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my))
  {
    ROS_WARN("The goal pose is outside of the map. Planning will always fail.");
    return false;
  }

  // Extract yaw from goal pose then create matching graph location
  q = tf::Quaternion(
    goal.pose.orientation.x,
    goal.pose.orientation.y,
    goal.pose.orientation.z,
    goal.pose.orientation.w);
  m = tf::Matrix3x3(q);
  m.getRPY(roll, pitch, yaw);
  int goal_yaw = (static_cast<int>(yaw * 180.0 / M_PI) + 360) % 360;

  ROS_INFO("Goal yaw is %d", goal_yaw);

  GridPose grid_end(GridLocation(static_cast<int>(mx), static_cast<int>(my)), goal_yaw, 9, 0);
  grid_end.is_goal_ = true;

  // Call A* to do the actual planning
  ROS_INFO("Planning path.");
  bool succeeded = aStar(grid_start, grid_end);

  // A* found a path, we can continue
  if (succeeded)
  {
    ROS_INFO("Found path.");
    reconstructPath(grid_start);
    plan.push_back(start);

    geometry_msgs::PoseStamped step; // Step is in world coordinates
    step.header = start.header;

    tf2::Quaternion last_quat; // Orientation of the last step in world coordinates
    tf2::convert(start.pose.orientation, last_quat);

    tf2::Quaternion goal_quat;
    tf2::convert(goal.pose.orientation, goal_quat);

    tf2::Quaternion reversing_quat;
    reversing_quat.setRPY(0.0, 0.0, M_PI);

    tf2::Quaternion reversed_goal_quat;
    reversed_goal_quat = goal_quat * reversing_quat;

    // We traverse the path to convert map coordinates path to world coordinates plan
    for (unsigned long i = 0; i < path_.size(); i++)
    {
      // Convert map coordinates to world coordinates
      GridPose pose = path_[i];
      costmap_->mapToWorld(
            pose.location().x(), pose.location().y(),
            step.pose.position.x, step.pose.position.y);

      // We are far enough from the end of the path to look ahead
      if ( i+20 < path_.size())
      {
        GridPose ahead = path_[i+20];   // Pose ahead is hardcoded at 20 index farther for now
        double wx, wy;

        // Get ahead coordinates in world coordinates
        costmap_->mapToWorld(
              ahead.location().x(), ahead.location().y(),
              wx, wy);

        // Compute orientation tangent to the current motion moving forward
        tf2::Quaternion tangent_quat;
        double tangent_rad = atan2(
              wy - step.pose.position.y,
              wx - step.pose.position.x);
        tangent_rad = fmod(tangent_rad + 2.0 * M_PI, 2.0*M_PI);
        int tangent_deg = static_cast<int>(tangent_rad * 180.0 / M_PI);
        tangent_quat.setRPY(0.0, 0.0, tangent_rad);

        // Compute orientation tangent to the current motion in reverse
        tf2::Quaternion reverse_quat;
        int reverse_deg = (tangent_deg + 180) % 360;
        double reverse_rad = fmod(tangent_rad + M_PI, 2.0*M_PI);
        reverse_quat.setRPY(0.0, 0.0, reverse_rad);

        // Compute orientation in the middle of the pose free orientation interval
        tf2::Quaternion interval_quat;
        double interval_deg = pose.theta_start() + static_cast<double>(pose.theta_length())/2.0;
        double interval_rad = interval_deg * M_PI / 180.0;
        interval_quat.setRPY(0.0, 0.0, interval_rad);

        // Last orientation is closer to forward motion tangent (we are probably moving forward)
        if(fabs(last_quat.angleShortestPath(tangent_quat)) <= fabs(last_quat.angleShortestPath(reverse_quat)))
        {
          // The formward tangent orientation is allowed by the pose free interval
          if (pose.theta_is_free(tangent_deg)) {
            tf2::convert(tangent_quat, step.pose.orientation);
          } else { // We fall back to the center of the free interval to maximise clearance
            tf2::convert(interval_quat, step.pose.orientation);
          }
        }
        else // Last orientation is closer to reverse motion tangent (we are probably reversing)
        {
          // The reverse tangent orientation is allowed by the pose free interval
          if (pose.theta_is_free(reverse_deg)) {
            tf2::convert(reverse_quat, step.pose.orientation);
          } else {  // We fall back to the center of the free interval to maximise clearance
            tf2::convert(interval_quat, step.pose.orientation);
          }
        }
      }
      else // We can't look ahead so we adopt the goal orientation
      {
        bool reverse_is_closer = fabs(last_quat.angleShortestPath(reversed_goal_quat)) <= fabs(last_quat.angleShortestPath(goal_quat));
        if(reverse_is_closer && config_.simetrical_goal) {
          tf2::convert(reversed_goal_quat, step.pose.orientation);
        }
        else {
          tf2::convert(goal_quat, step.pose.orientation);
        }
      }

      // Add the step to the plan then convert from tf2 quaternion to geometry_msgs quaternion
      plan.push_back(step);
      tf2::convert(step.pose.orientation, last_quat);
    }

    // Publish the complete plan
    nav_msgs::Path path_msg;
    path_msg.header = start.header;
    path_msg.poses = plan;
    path_pub_.publish(path_msg);
  }

  // No path found, nothing to do
  else
  {
    ROS_WARN("No path to goal found.");
  }

  return succeeded;
}


/**
 * @brief A* algorithm implementation
 *
 * The actual implementation of the A* algorithm
 * The logic is mostly from here: https://www.redblobgames.com/pathfinding/a-star/implementation.html#cplusplus
 *
 * @param start Start pose of the robot in map coordinates
 * @param goal Goal pose of the robot in map coordinates
 * @return true if a path has been found
 */
bool InoPlanner::aStar(GridPose start, GridPose goal)
{
  // Clears queues and frontier for new run
  came_from_.clear();
  cost_so_far_.clear();
  frontier_ = PQ();

  // Start A* with a single node
  frontier_.emplace(0, start);
  came_from_[start] = start;
  cost_so_far_[start] = 0.0;

  GridPose current;
  double new_cost;

  // Setup coefficients for octile distance see reference
  // http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#diagonal-distance
  double d = 1.0;
  double d2 = sqrt(2.0);

  // Setup tie breaker for heuristic function see reference
  // http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#breaking-ties
  double p = 0.5 / static_cast<double>(costmap_->getSizeInCellsX() + costmap_->getSizeInCellsY());

  // There is still nodes to expand and ROS didn't request shutdown
  while (!frontier_.empty() && ros::ok())
  {
    // Get the highest priority node from the queue
    current = frontier_.top().second;
    frontier_.pop();

    // We reached the goal! Our work is here is done
    if (current == goal)
    {
      path_end_ = current;
      return true;
    }

    // Retreive neighborings pose from the graph. Thoses poses are all reachable from the current pose.
    auto neighbors = graph_.neighbors(costmap_, *world_model_, simplified_footprint_, inscribed_radius_, circumscribed_radius_, current);

    // Each neighbor could be the next node
    for (GridPose next : neighbors)
    {
      // Cost to reach the neighbor is current cost + the cost to travel to the neighbor
      new_cost = cost_so_far_[current] + current.costTo(next);

      // The neighbor was never visited or a cheaper way to reach it is found
      if (cost_so_far_.find(next) == cost_so_far_.end() || new_cost < cost_so_far_[next])
      {
        // We update the cost to reach the neighbor and add it to the priority queue
        cost_so_far_[next] = new_cost;
        came_from_[next] = current;
        frontier_.emplace(new_cost + next.heuristic(goal, d, d2, p), next);
      }
    }
  }

  // We didn't break from the search loop, so no plan exists or we were interrupted
  return false;
}


/**
 * @brief Traverse back the graph to reconstruct the path found by A*
 * @param Start pose of the robot in map coordinates passed to the A* algorithm
 */
void InoPlanner::reconstructPath(GridPose start)
{
  ROS_INFO("reconstructing...");

  // Clear the path from previous run
  path_.clear();

  // Starting from the end of the path, which match the goal
  // We use path_end_ rather than the goal because we need the exact pose object for the unordered_map lookups to work!
  GridPose current = path_end_;

  // While we didn't reached the start and ROS is not quitting
  while (current != start && ros::ok())
  {
    path_.push_back(current);
    current = came_from_[current];
  }

  // We reverse the path to have the end at the end of the vector as expected
  std::reverse(path_.begin(), path_.end());
  ROS_INFO("path complete.");
}

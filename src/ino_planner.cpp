#include <ino_planner/ino_planner.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>


PLUGINLIB_EXPORT_CLASS(ino_planner::InoPlanner, nav_core::BaseGlobalPlanner)


using namespace ino_planner;


InoPlanner::InoPlanner()
{
  geometry_msgs::Point point;

  point.x = 1.57;
  point.y = -1.02;
  footprint_.push_back(point);

  point.x = 1.57;
  point.y = 1.02;
  footprint_.push_back(point);

  point.x = -1.57;
  point.y = 1.02;
  footprint_.push_back(point);

  point.x = -1.57;
  point.y = -1.02;
  footprint_.push_back(point);
}


void InoPlanner::initialize(std::string, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("This planner is already initialized.");
  }
  costmap_ = costmap_ros->getCostmap();
  worldModel_ = std::make_unique<base_local_planner::CostmapModel>(*costmap_);

  ros::NodeHandle nh;
  path_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1, true);

  initialized_ = true;
  ROS_INFO("Initialized ino_planner.");
}


bool InoPlanner::makePlan(
  const geometry_msgs::PoseStamped& start,
  const geometry_msgs::PoseStamped& goal,
  std::vector<geometry_msgs::PoseStamped>& plan)
{
  ROS_INFO("Got goal.");
  unsigned int mx, my;

  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my))
  {
    ROS_WARN("The starting pose is outside of the map. Planning will always fail.");
    return false;
  }
  GridPose grid_start(GridLocation(static_cast<int>(mx), static_cast<int>(my)), 0, 359, 0);

  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my))
  {
    ROS_WARN("The goal pose is outside of the map. Planning will always fail.");
    return false;
  }
  GridPose grid_end(GridLocation(static_cast<int>(mx), static_cast<int>(my)), 0, 359, 0);

  ROS_INFO("Building the graph...");
  graph_.rebuild(costmap_, *worldModel_, footprint_);
  ROS_INFO("Graph built. Planning path.");
  bool succeeded = dijkstra(grid_start, grid_end);

  if (succeeded)
  {
    ROS_INFO("Found path.");
    reconstructPath(grid_start, grid_end);

    geometry_msgs::PoseStamped step = start;
    plan.push_back(start);
    for (GridPose pose: path_)
    {
      costmap_->mapToWorld(
            pose.location().x(), pose.location().y(),
            step.pose.position.x, step.pose.position.y);

      double yaw_deg = pose.theta_start() + pose.theta_length()/2.0;
      double yaw = yaw_deg * M_PI / 180.0;

      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, yaw);
      tf2::convert(orientation, step.pose.orientation);

      plan.push_back(step);
    }
    plan.push_back(goal);

    nav_msgs::Path path_msg;
    path_msg.header = start.header;
    path_msg.poses = plan;
    path_pub_.publish(path_msg);
  }

  else
  {
    ROS_WARN("No path to goal found.");
  }

  return succeeded;
}


bool InoPlanner::dijkstra(GridPose start, GridPose goal)
{
  came_from_.clear();
  cost_so_far_.clear();
  frontier_ = PQ();

  frontier_.emplace(0, start);

  came_from_[start] = start;
  cost_so_far_[start] = 0.0;

  GridPose current;
  double new_cost;

  while (!frontier_.empty() && ros::ok())
  {
    ROS_INFO_THROTTLE(1, "frontier has %lu nodes.", frontier_.size());
    current = frontier_.top().second;
    frontier_.pop();

    if (current == goal)
    {
      return true;
    }

    ROS_INFO_THROTTLE(1, "node has %lu neighbors.", graph_.neighbors(current).size());
    for (GridPose next : graph_.neighbors(current))
    {
      new_cost = cost_so_far_[current] + current.costTo(next);
      if (cost_so_far_.find(next) == cost_so_far_.end() || new_cost < cost_so_far_[next])
      {
        cost_so_far_[next] = new_cost;
        came_from_[next] = current;
        frontier_.emplace(new_cost, next);
      }
    }
  }

  return false;
}


void InoPlanner::reconstructPath(GridPose start, GridPose goal)
{
  path_.clear();

  GridPose current = goal;
  while (current != start)
  {
    path_.push_back(current);
    current = came_from_[current];
  }

  path_.push_back(start);
  std::reverse(path_.begin(), path_.end());
}


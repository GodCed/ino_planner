#include <ino_planner/ino_planner.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf/tf.h>
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

  visited_grid_.header.frame_id = costmap_ros->getGlobalFrameID();

  ros::NodeHandle nh("~");
  path_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1, true);
  grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("visited_cells", 10, false);

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

  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my))
  {
    ROS_WARN("The goal pose is outside of the map. Planning will always fail.");
    return false;
  }

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

  if (!graphBuilt_)
  {
    ROS_INFO("Building the graph...");
    graph_.rebuild(costmap_, *worldModel_, footprint_);

    graphBuilt_ = true;
  }

  ROS_INFO("Planning path.");
  bool succeeded = dijkstra(grid_start, grid_end);

  if (succeeded)
  {
    ROS_INFO("Found path.");
    reconstructPath(grid_start);

    plan.push_back(start);

    geometry_msgs::PoseStamped step;
    step.header = start.header;

    tf2::Quaternion last_quat;
    tf2::convert(start.pose.orientation, last_quat);

    for (unsigned long i = 0; i < path_.size(); i++)
    {
      GridPose pose = path_[i];
      ROS_INFO_THROTTLE(1,"Adding yaw to %04lu/%04lu", i, path_.size());

      costmap_->mapToWorld(
            pose.location().x(), pose.location().y(),
            step.pose.position.x, step.pose.position.y);

      if ( i+20 < path_.size())
      {
        GridPose ahead = path_[i+20];
        double wx, wy;

        costmap_->mapToWorld(
              ahead.location().x(), ahead.location().y(),
              wx, wy);

        tf2::Quaternion tangent_quat;
        double tangent_rad = atan2(
              wy - step.pose.position.y,
              wx - step.pose.position.x);
        tangent_rad = fmod(tangent_rad + 2.0 * M_PI, 2.0*M_PI);
        int tangent_deg = static_cast<int>(tangent_rad * 180.0 / M_PI);
        tangent_quat.setRPY(0.0, 0.0, tangent_rad);

        tf2::Quaternion reverse_quat;
        int reverse_deg = (tangent_deg + 180) % 360;
        double reverse_rad = fmod(tangent_rad + M_PI, 2.0*M_PI);
        reverse_quat.setRPY(0.0, 0.0, reverse_rad);

        tf2::Quaternion interval_quat;
        double interval_deg = pose.theta_start() + static_cast<double>(pose.theta_length())/2.0;
        double interval_rad = interval_deg * M_PI / 180.0;
        interval_quat.setRPY(0.0, 0.0, interval_rad);

        if(fabs(last_quat.angleShortestPath(tangent_quat)) <= fabs(last_quat.angleShortestPath(reverse_quat)))
        {
          if (pose.theta_is_free(tangent_deg))
          {
            tf2::convert(tangent_quat, step.pose.orientation);
          }
          else
          {
            tf2::convert(interval_quat, step.pose.orientation);
          }
        }
        else
        {
          if (pose.theta_is_free(reverse_deg))
          {
            tf2::convert(reverse_quat, step.pose.orientation);
          }
          else
          {
            tf2::convert(interval_quat, step.pose.orientation);
          }
        }
      }

      else
      {
        step.pose.orientation = goal.pose.orientation;
      }

      plan.push_back(step);
      tf2::convert(step.pose.orientation, last_quat);
    }
    plan.push_back(goal);


    //
    // ===
    // We need to offset the yaw for 60 samples!
    // ===
    //

    /*for(unsigned long i = plan.size()-2; i > 0; i--)
    {
      if (i > 59)
      {
        plan[i].pose.orientation = plan[i-60].pose.orientation;
      }
      else
      {
        plan[i].pose.orientation = start.pose.orientation;
      }
    }*/

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

  visited_grid_.info.origin.position.x = costmap_->getOriginX();
  visited_grid_.info.origin.position.y = costmap_->getOriginY();
  visited_grid_.info.resolution = static_cast<float>(costmap_->getResolution());
  visited_grid_.info.width = costmap_->getSizeInCellsX();
  visited_grid_.info.height = costmap_->getSizeInCellsY();
  visited_grid_.data = std::vector<int8_t>(visited_grid_.info.width * visited_grid_.info.height, 0);
  grid_pub_.publish(visited_grid_);

  double d = 1.0;
  double d2 = sqrt(2.0);

  while (!frontier_.empty() && ros::ok())
  {
    current = frontier_.top().second;
    frontier_.pop();

    visited_grid_.data[current.location().x() + current.location().y()*visited_grid_.info.width] = -1;
    grid_pub_.publish(visited_grid_);

    if (current == goal)
    {
      path_end_ = current;
      return true;
    }

    for (GridPose next : graph_.neighbors(current))
    {
      new_cost = cost_so_far_[current] + current.costTo(next);
      if (cost_so_far_.find(next) == cost_so_far_.end() || new_cost < cost_so_far_[next])
      {
        cost_so_far_[next] = new_cost;
        came_from_[next] = current;
        frontier_.emplace(new_cost + next.heuristic(goal, d, d2), next);
      }
    }
  }

  return false;
}


void InoPlanner::reconstructPath(GridPose start)
{
  path_.clear();

  GridPose current = path_end_;

  ROS_INFO("reconstructing...");
  while (current != start && ros::ok())
  {
    if (current != path_end_)
    {
      path_.push_back(current);
    }
    current = came_from_[current];
  }

  ROS_INFO("path complete.");
  std::reverse(path_.begin(), path_.end());
}


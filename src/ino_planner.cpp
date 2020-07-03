#include <ino_planner/ino_planner.hpp>
#include <pluginlib/class_list_macros.hpp>


PLUGINLIB_EXPORT_CLASS(ino_planner::InoPlanner, nav_core::BaseGlobalPlanner)


using namespace ino_planner;


InoPlanner::InoPlanner()
{

}


void InoPlanner::initialize(std::string, costmap_2d::Costmap2DROS* costmap_ros)
{
  costmap_ = costmap_ros;
  initialized_ = true;
}


bool InoPlanner::makePlan(
  const geometry_msgs::PoseStamped& start,
  const geometry_msgs::PoseStamped& goal,
  std::vector<geometry_msgs::PoseStamped>& plan)
{
  GridPose grid_start;
  GridPose grid_end;
  bool succeeded;

  succeeded = dijkstra(grid_start, grid_end);

  if (succeeded)
  {
    reconstructPath(grid_start, grid_end);
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

  while (!frontier_.empty())
  {
    current = frontier_.top().second;
    frontier_.pop();

    if (current == goal)
    {
      return true;
    }

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



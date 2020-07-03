#include <ino_planner/ino_planner.hpp>
#include <pluginlib/class_list_macros.hpp>


PLUGINLIB_EXPORT_CLASS(ino_planner::InoPlanner, nav_core::BaseGlobalPlanner)


using namespace ino_planner;


InoPlanner::InoPlanner()
{

}


void InoPlanner::initialize(std::string, costmap_2d::Costmap2DROS* costmap_ros)
{
  costmap_ = costmap_ros->getCostmap();
  initialized_ = true;
}


bool InoPlanner::makePlan(
  const geometry_msgs::PoseStamped& start,
  const geometry_msgs::PoseStamped& goal,
  std::vector<geometry_msgs::PoseStamped>& plan)
{
  unsigned int mx, my;

  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  GridPose grid_start(GridLocation(static_cast<int>(mx), static_cast<int>(my)), 0, 359);

  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  GridPose grid_end(GridLocation(static_cast<int>(mx), static_cast<int>(my)), 0, 359);

  bool succeeded = dijkstra(grid_start, grid_end);

  if (succeeded)
  {
    reconstructPath(grid_start, grid_end);

    geometry_msgs::PoseStamped step = start;
    for (GridPose pose: path_)
    {
      costmap_->mapToWorld(
            pose.location().x(), pose.location().y(),
            step.pose.position.x, step.pose.position.y);
      plan.push_back(step);
    }
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



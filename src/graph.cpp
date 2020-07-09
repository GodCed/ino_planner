#include <ino_planner/graph.hpp>
#include <cmath>
#include <algorithm>


using namespace ino_planner;


GridLocation::GridLocation() {}


GridLocation::GridLocation(int x, int y): x_(x), y_(y) {}


int GridLocation::hash() const
{
    return x_ ^ (y_ << 1);
}


double GridLocation::costTo(GridLocation location)
{
    int dx = location.x_ - x_;
    int dy = location.y_ - y_;

    return dx*dx + dy*dy;
}


GridPose::GridPose() {}


GridPose::GridPose(GridLocation location, int free_theta_start, int free_theta_length, int cost):
    location_(location),
    free_theta_start_(free_theta_start),
    free_theta_length_(free_theta_length),
    cost_(cost) {}


int GridPose::thetaOverlapWith(GridPose pose) const
{
    if (free_theta_start_ <= pose.free_theta_start_)
    {
        return free_theta_length_ - (pose.free_theta_start_ - free_theta_start_);
    }
    else
    {
        return pose.free_theta_length_ - (free_theta_start_ - pose.free_theta_start_);
    }
}


bool GridPose::canReachTo(GridPose pose) const
{
  int overlap = thetaOverlapWith(pose);
  return overlap >= 0;
}


double GridPose::costTo(GridPose pose)
{
    return location_.costTo(pose.location_)
        + (35.9 / (double)thetaOverlapWith(pose))
        + cost_;
}


int GridPose::hash() const
{
    return location_.hash() ^ free_theta_start_ << 1;
}


std::vector<GridPose> Graph::neighbors(GridPose pose)
{
    std::vector<GridPose> neighbors;
    for(auto move: moves_)
    {
        GridLocation location = pose.offsetLocation(move);
        if (location.x() > 0 && location.x() < size_x_ && location.y() > 0 && location.y() < size_y_)
        {
          auto potential_poses = free_grid_.equal_range(location);
          std::for_each(
              potential_poses.first,
              potential_poses.second,
              [pose, &neighbors](auto potential_pair) mutable{
              if (pose.canReachTo(potential_pair.second)) {
                  neighbors.push_back(potential_pair.second);
              }
          });
        }
    }
    return neighbors;
}


void Graph::rebuild(costmap_2d::Costmap2D *costmap, base_local_planner::WorldModel &world_model, std::vector<geometry_msgs::Point> &footprint)
{
  free_grid_.clear();

  size_x_ = costmap->getSizeInCellsX();
  size_y_ = costmap->getSizeInCellsY();

  for (unsigned int mx = 0; mx < size_x_; mx++)
  {
    for (unsigned int my = 0; my < size_y_; my++)
    {
      unsigned char cost = costmap->getCost(mx, my);
      if (cost == 0) // Definitly not in a collision
      {
        GridLocation loc(static_cast<int>(mx), static_cast<int>(my));
        GridPose pose(loc, 0, 359, cost);
        free_grid_.emplace(loc, pose);
      }
      else if (cost < 253) // Maybe in a collision
      {
        GridLocation loc(static_cast<int>(mx), static_cast<int>(my));
        int safe_begin = 0;
        bool was_safe = false;
        for (int theta = 0; theta < 359; theta+=10) // We check orientations...
        {
          double wx, wy;
          costmap->mapToWorld(mx, my, wx, wy);
          double theta_rad = static_cast<double>(theta) * M_PI / 180.0;
          bool colliding = world_model.footprintCost(wx, wy, theta_rad, footprint) < 0;

          if (!colliding && !was_safe) // Begining of safe zone
          {
            safe_begin = theta;
            was_safe = true;
          }
          else if (colliding && was_safe) // End of safe zone
          {
            GridPose pose(loc, safe_begin, theta-safe_begin-1, cost);
            free_grid_.emplace(loc, pose);
            was_safe = false;
          }
        }
        if (was_safe) // Last interval was open
        {
          GridPose pose(loc, safe_begin, 359-safe_begin, cost);
          free_grid_.emplace(loc, pose);
        }
      }
      // We ignore 253-254 because we are definitly in a collision.
      if (!ros::ok())
      {
        break;
      }
    }
    if (!ros::ok())
    {
      break;
    }
  }
}


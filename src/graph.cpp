#include <ino_planner/graph.hpp>
#include <costmap_2d/cost_values.h>
#include <cmath>
#include <algorithm>


using namespace ino_planner;
using namespace std;


GridLocation::GridLocation() {}


GridLocation::GridLocation(int x, int y): x_(x), y_(y) {}


int GridLocation::hash() const
{
    return x_ ^ (y_ << 1);
}


void GridLocation::mapToWorld(const costmap_2d::Costmap2D *costmap, double &x, double &y) const
{
    costmap->mapToWorld(x_, y_, x, y);
}


unsigned char GridLocation::cost(const costmap_2d::Costmap2D *costmap) const
{
    return costmap->getCost(x_, y_);
}


double GridLocation::costTo(GridLocation location)
{
    int dx = location.x_ - x_;
    int dy = location.y_ - y_;

    return std::sqrt(dx*dx + dy*dy);
}


void GridLocation::deltasTo(GridLocation goal, double &dx, double &dy)
{
    dx = abs(x_ - goal.x_);
    dy = abs(y_ - goal.y_);
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
       + static_cast<double>(cost_) / 255.0
       + (1.0 - thetaOverlapWith(pose) / 360.0);
}


double GridPose::heuristic(GridPose goal, double d, double d2, double p)
{
  double dx, dy;
  location_.deltasTo(goal.location_, dx, dy);

  double h = d * (dx + dy) + (d2 - 2 * d) * std::min(dx, dy);
  return h * (1.0 + p);
}


int GridPose::hash() const
{
    return location_.hash() ^ free_theta_start_ << 1;
}


std::vector<GridPose> Graph::neighbors(
        const costmap_2d::Costmap2D *costmap,
        base_local_planner::WorldModel &world_model,
        const std::vector<geometry_msgs::Point> &footprint,
        const double inscribed_radius,
        const double circumscribed_radius,
        GridPose pose)
{
    size_x_ = costmap->getSizeInCellsX();
    size_y_ = costmap->getSizeInCellsY();

    std::vector<GridPose> neighbors;
    for(auto move: moves_)
    {
        GridLocation location = pose.offsetLocation(move);

        if (location.inGrid(size_x_, size_y_))
        {
          std::vector<GridPose> potentials;
          posesForLocation(costmap, world_model, footprint, inscribed_radius, circumscribed_radius, location, potentials);

          for (GridPose potential: potentials) {

              if ( pose.canReachTo(potential) ) {
                  neighbors.push_back(potential);
              }
          }

        }
    }

    return neighbors;
}


void Graph::posesForLocation(
        const costmap_2d::Costmap2D *costmap,
        base_local_planner::WorldModel &world_model,
        const std::vector<geometry_msgs::Point> &footprint,
        const double inscribed_radius,
        const double circumscribed_radius,
        const GridLocation loc,
        std::vector<GridPose> &poses)
{
    unsigned char cost = loc.cost(costmap);

    if (cost == costmap_2d::FREE_SPACE) // Definitly not in a collision
    {
      GridPose pose(loc, 0, 359, cost);
      poses.push_back(pose);
    }

    else if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) // Maybe in a collision
    {
      int safe_begin = 0;
      bool was_safe = false;

      double wx, wy;
      loc.mapToWorld(costmap, wx, wy);

      for (int theta = 0; theta < 359; theta+=36) // We check orientations...
      {
        double theta_rad = static_cast<double>(theta) * M_PI / 180.0;
        bool colliding = world_model.footprintCost(wx, wy, theta_rad, footprint, inscribed_radius, circumscribed_radius) < 0;

        if (!colliding && !was_safe) // Begining of safe zone
        {
          safe_begin = theta;
          was_safe = true;
        }
        else if (colliding && was_safe) // End of safe zone
        {
          GridPose pose(loc, safe_begin, theta-safe_begin-1, cost);
          poses.push_back(pose);

          was_safe = false;
        }
      }

      if (was_safe) // Last interval was open
      {
        GridPose pose(loc, safe_begin, 359-safe_begin, cost);
        poses.push_back(pose);
      }
    }

    // We ignore 253-254 because we are definitly in a collision.
}

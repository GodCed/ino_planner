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


GridPose::GridPose(GridLocation location, int free_theta_start, int free_theta_length):
    location_(location),
    free_theta_start_(free_theta_start),
    free_theta_length_(free_theta_length) {}


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
    return thetaOverlapWith(pose) > 0;
}


double GridPose::costTo(GridPose pose)
{
    return location_.costTo(pose.location_) + (35.9 / (double)thetaOverlapWith(pose));
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
        auto potential_poses = free_grid_.equal_range(location);
        std::for_each(
            potential_poses.first,
            potential_poses.second,
            [pose, neighbors](auto potential_pair) mutable{
            if (pose.canReachTo(potential_pair.second)) {
                neighbors.push_back(potential_pair.second);
            }
        });
    }
    return neighbors;
}
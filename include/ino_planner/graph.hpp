#ifndef ino_planner_GRAPH_HPP
#define ino_planner_GRAPH_HPP

#include <functional>
#include <vector>
#include <array>
#include <unordered_map>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/world_model.h>

namespace costmap_2d
{
  static const unsigned char POSSIBLY_CIRCUMSCRIBED_INFLATED_OBSTACLE = 128;
}

namespace ino_planner
{

  class GridLocation
  {
  public:
    GridLocation();
    GridLocation(int x, int y);

    int hash() const;

    unsigned char cost(const costmap_2d::Costmap2D *costmap) const;
    double costTo(GridLocation location);

    void mapToWorld(const costmap_2d::Costmap2D *costmap, double &x, double &y) const;

    inline int x()
    {
      return x_;
    }

    inline int y()
    {
      return y_;
    }

    inline bool inGrid(int x_dim, int y_dim)
    {
      return x_ >= 0 && x_ < x_dim && y_ >= 0 && y_ < y_dim;
    }

    void deltasTo(GridLocation goal, double &dx, double &dy);

    friend inline GridLocation operator+(const GridLocation &a, const GridLocation &b)
    {
      return GridLocation(a.x_ + b.x_, a.y_ + b.y_);
    }

    friend inline bool operator==(const GridLocation &a, const GridLocation &b)
    {
      return a.x_ == b.x_ && a.y_ == b.y_;
    }

  private:
    int x_ = 0;
    int y_ = 0;
  };

  class GridPose
  {
  public:
    bool is_start_ = false;
    bool is_goal_ = false;
    GridPose();
    GridPose(GridLocation location, int free_theta_start, int free_theta_length, int cost);

    bool canReachTo(GridPose pose) const;
    double costTo(GridPose pose);
    double heuristic(GridPose pose, double d, double d2, double p);

    inline GridLocation offsetLocation(GridLocation offset)
    {
      return location_ + offset;
    }

    inline GridLocation location()
    {
      return location_;
    }

    inline int theta_start() { return free_theta_start_; }
    inline int theta_length() { return free_theta_length_; }

    inline bool theta_is_free(int theta)
    {
      return (free_theta_start_ <= theta) && (theta <= (free_theta_start_ + free_theta_length_));
    }

    friend inline bool operator==(const GridPose &a, const GridPose &b)
    {
      return a.location_ == b.location_ && a.canReachTo(b);
    }

    friend inline bool operator!=(const GridPose &a, const GridPose &b)
    {
      return !(a == b);
    }

    int hash() const;

  private:
    GridLocation location_;
    int free_theta_start_;
    int free_theta_length_;
    int cost_;

    int thetaOverlapWith(GridPose pose) const;
  };

} // namespace ino_planner

namespace std
{

  template <>
  struct hash<ino_planner::GridLocation>
  {
    inline std::size_t operator()(const ino_planner::GridLocation &id) const noexcept
    {
      return std::hash<int>()(id.hash());
    }
  };

  template <>
  struct hash<ino_planner::GridPose>
  {
    inline std::size_t operator()(const ino_planner::GridPose &id) const noexcept
    {
      return std::hash<int>()(id.hash());
    }
  };
} // namespace std

namespace ino_planner
{

  class Graph
  {
  public:
    std::vector<GridPose> neighbors(
        const costmap_2d::Costmap2D *costmap,
        base_local_planner::WorldModel &world_model,
        const std::vector<geometry_msgs::Point> &footprint,
        const double inscribed_radius,
        const double circumscribed_radius,
        GridPose pose);

  private:
    std::array<GridLocation, 8> moves_ = {
        GridLocation{1, 0}, GridLocation{0, 1},
        GridLocation{-1, 0}, GridLocation{0, -1},
        GridLocation{1, 1}, GridLocation{1, -1},
        GridLocation{-1, -1}, GridLocation{-1, 1}};

    void posesForLocation(
        const costmap_2d::Costmap2D *costmap,
        base_local_planner::WorldModel &world_model,
        const std::vector<geometry_msgs::Point> &footprint,
        const double inscribed_radius,
        const double circumscribed_radius,
        const GridLocation loc,
        std::vector<GridPose> &poses);

    int size_x_;
    int size_y_;
  };

} // namespace ino_planner

#endif // ino_planner_GRAPH_HPP

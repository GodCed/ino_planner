#ifndef ino_planner_GRAPH_HPP
#define ino_planner_GRAPH_HPP


#include <functional>
#include <vector>
#include <array>
#include <unordered_map>


namespace ino_planner
{


  class GridLocation
  {
  public:
    GridLocation();
    GridLocation(int x, int y);
    
    int hash() const;
    double costTo(GridLocation location);

    inline unsigned int x()
    {
      return static_cast<unsigned int>(x_);
    }

    inline unsigned int y()
    {
      return static_cast<unsigned int>(y_);
    }

    friend inline GridLocation operator+(const GridLocation& a, const GridLocation& b)
    {
      return GridLocation(a.x_ + b.x_, a.y_ + b.y_);
    }

    friend inline bool operator==(const GridLocation& a, const GridLocation& b)
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
    GridPose();
    GridPose(GridLocation location, int free_theta_start, int free_theta_length);

    bool canReachTo(GridPose pose) const;
    double costTo(GridPose pose);

    inline GridLocation offsetLocation(GridLocation offset)
    {
      return location_ + offset;
    }

    inline GridLocation location()
    {
      return location_;
    }

    friend inline bool operator==(const GridPose& a, const GridPose& b)
    {
      return a.location_ == b.location_ && a.canReachTo(b);
    }

    friend inline bool operator!=(const GridPose& a, const GridPose& b)
    {
      return !(a == b);
    }

    int hash() const;

  private:
    GridLocation location_;
    int free_theta_start_;
    int free_theta_length_;

    int thetaOverlapWith(GridPose pose) const;
  };

  
}


namespace std
{


  template <> struct hash<ino_planner::GridLocation>
  {
    inline std::size_t operator()(const ino_planner::GridLocation& id) const noexcept
    {
      return std::hash<int>()(id.hash());
    }
  };
  

  template <> struct hash<ino_planner::GridPose>
  {
    inline std::size_t operator()(const ino_planner::GridPose& id) const noexcept
    {
      return std::hash<int>()(id.hash());
    }
  };
}


namespace ino_planner
{


  class Graph
  {
  public:
    std::vector<GridPose> neighbors(GridPose pose);

  private:
    std::unordered_multimap<GridLocation, GridPose> free_grid_;

    std::array<GridLocation, 4> moves_ = {
      GridLocation{1, 0}, GridLocation{0, 1},
      GridLocation{-1, 0}, GridLocation{0, -1}
    };
  };


}


#endif // ino_planner_GRAPH_HPP

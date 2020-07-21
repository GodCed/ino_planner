#ifndef ino_planner_INO_PLANNER_HPP
#define ino_planner_INO_PLANNER_HPP


#include <nav_core/base_global_planner.h>
#include <base_local_planner/costmap_model.h>
#include <ino_planner/graph.hpp>
#include <queue>
#include <vector>


namespace ino_planner {

  typedef std::pair<double, GridPose> PQItem;

  inline bool operator<(const PQItem& a, const PQItem& b)
  {
    return a.first < b.first;
  }

  typedef std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> PQ;

  class InoPlanner: public nav_core::BaseGlobalPlanner
  {
  public:
    InoPlanner();

    void initialize(std::string, costmap_2d::Costmap2DROS* costmap_ros);

    bool makePlan(
        const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan);

    bool dijkstra(GridPose start, GridPose goal);
    void reconstructPath(GridPose start);

  private:
    bool initialized_ = false;
    bool graphBuilt_ = false;

    std::vector<geometry_msgs::Point> footprint_;
    costmap_2d::Costmap2D* costmap_ = nullptr;
    std::unique_ptr<base_local_planner::CostmapModel> worldModel_;

    Graph graph_;
    std::unordered_map<GridPose, GridPose> came_from_;
    std::unordered_map<GridPose, double> cost_so_far_;
    std::vector<GridPose> path_;
    GridPose path_end_;

    ros::Publisher path_pub_;

    ros::Publisher grid_pub_;
    nav_msgs::OccupancyGrid visited_grid_;

    PQ frontier_;
  };


}


#endif // ino_planner_INO_PLANNER_HPP

#ifndef ino_planner_INO_PLANNER_HPP
#define ino_planner_INO_PLANNER_HPP


#include <nav_core/base_global_planner.h>
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
    void reconstructPath(GridPose start, GridPose goal);

  private:
    bool initialized_ = false;
    costmap_2d::Costmap2DROS* costmap_ = nullptr;
    
    Graph graph_;
    std::unordered_map<GridPose, GridPose> came_from_;
    std::unordered_map<GridPose, double> cost_so_far_;
    std::vector<GridPose> path_;

    PQ frontier_;
  };


}


#endif // ino_planner_INO_PLANNER_HPP

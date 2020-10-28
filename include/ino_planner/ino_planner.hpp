#ifndef ino_planner_INO_PLANNER_HPP
#define ino_planner_INO_PLANNER_HPP


#include <nav_core/base_global_planner.h>
#include <base_local_planner/costmap_model.h>
#include <ino_planner/graph.hpp>
#include <queue>
#include <vector>


namespace ino_planner {


  /**
   * @brief An item of the priority queue
   *
   * It contains a GridPose and the cost to reach it
   */
  typedef std::pair<double, GridPose> PQItem;


  /**
   * @brief Priority queue item comparison operator
   * @param a First item
   * @param b Second item
   * @return true if the first item cost is less than the second item cost
   */
  inline bool operator<(const PQItem& a, const PQItem& b)
  {
    return a.first < b.first;
  }


  /**
   * @brief Syntaxic sugar to define the priority queue
   */
  typedef std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> PQ;


  /**
   * @brief A ROS global planner which finds path without assuming a round robot
   *
   * The planner use the A* algorithm to find the shortest path to a goal in the costmap.
   * The robot is not assumed round, each node in the graph is a pose with a (x,y) location and an admissible range of orientation (theta).
   */
  class InoPlanner: public nav_core::BaseGlobalPlanner
  {
  public:
    InoPlanner() {};
    void initialize(std::string, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(
        const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan);

  private:
    // ROS stuff
    bool initialized_ = false;
    ros::Publisher path_pub_;

    // Footprint
    std::vector<geometry_msgs::Point> simplified_footprint_;
    double inscribed_radius_, circumscribed_radius_;

    // Costmap
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    std::unique_ptr<base_local_planner::WorldModel> world_model_;

    // A* algorithm stuff
    Graph graph_;
    std::unordered_map<GridPose, GridPose> came_from_;
    std::unordered_map<GridPose, double> cost_so_far_;
    std::vector<GridPose> path_;
    GridPose path_end_;
    PQ frontier_;

    // Utility methods
    void updateFootprint();
    bool aStar(GridPose start, GridPose goal);
    void reconstructPath(GridPose start);
  };


}


#endif // ino_planner_INO_PLANNER_HPP

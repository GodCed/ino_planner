#include <ino_planner/simple_costmap_model.hpp>
#include <costmap_2d/cost_values.h>


using namespace ino_planner;


/**
* @brief  Checks if the footprint cover an obstacle in the costmap
*
* The model first check if the center of the footprint lays in a inscribed inflated obstacle cell of the costmap.
* If this check pass it checks whether any point of the footprint is in a lethal cell.
*
* @param  position Footprint center in world coordinates
* @param  footprint Oriented footprint points in world coordinates
* @param  inscribed_radius The radius of the inscribed circle of the robot
* @param  circumscribed_radius The radius of the circumscribed circle of the robot
* @return Footprint center costmap cell cost if all the points lie outside the footprint, -1 otherwise
*/
double SimpleCostmapModel::footprintCost(
        const geometry_msgs::Point& position,
        const std::vector<geometry_msgs::Point>& footprint,
        double,
        double)
{
    // Get the footprint center cost from the costmap
    unsigned int map_x, map_y;
    costmap_.worldToMap(position.x, position.y, map_x, map_y);
    double center_cost = costmap_.getCost(map_x, map_y);

    // The footprint center is within an inscribed inflated obstacles
    // This means the footprint is definitly colliding with an obstacle
    if (center_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
        return -1;
    }
    // The footprint center is in free space
    // This means the footprint is definitly not colliding with an obstacle
    else if (center_cost == costmap_2d::FREE_SPACE)
    {
        return center_cost;
    }
    // We are in unknown territory
    // We need to check every points
    else
    {
        // Iterate over every provided footprint point
        std::vector<double> point_costs;
        for (auto point : footprint)
        {
            // Convert point to map coordinate and append its cost to the cost array
            unsigned int point_mx, point_my;
            costmap_.worldToMap(point.x, point.y, point_mx, point_my);
            point_costs.push_back(costmap_.getCost(point_mx, point_my));
        }

        // We check if any point lies in the inscribed radius, meaning an outside edge of the footprint is colliding
        // Else we can return the footprint center cost as the current position cost
        return *(std::max_element(point_costs.begin(), point_costs.end())) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE ? -1 : center_cost;
    }
}

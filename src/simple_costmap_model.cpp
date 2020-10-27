#include <ino_planner/simple_costmap_model.hpp>
#include <costmap_2d/cost_values.h>

using namespace ino_planner;

double SimpleCostmapModel::footprintCost(
        const geometry_msgs::Point& position,
        const std::vector<geometry_msgs::Point>& footprint,
        double,
        double)
{
    unsigned int map_x, map_y;
    costmap_.worldToMap(position.x, position.y, map_x, map_y);

    double center_cost = costmap_.getCost(map_x, map_y);

    if (center_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
        return -1;
    }
    else if (center_cost <= 127)
    {
        return center_cost;
    }
    else
    {
        std::vector<double> corner_costs;
        for (auto corner : footprint)
        {
            unsigned int corner_mx, corner_my;
            costmap_.worldToMap(corner.x, corner.y, corner_mx, corner_my);
            corner_costs.push_back(costmap_.getCost(corner_mx, corner_my));
        }
        return *(std::max_element(corner_costs.begin(), corner_costs.end())) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE ? -1 : center_cost;
    }

    return -1;
}

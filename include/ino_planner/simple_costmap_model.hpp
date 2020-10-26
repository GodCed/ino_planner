#ifndef SIMPLE_COSTMAP_MODEL_HPP
#define SIMPLE_COSTMAP_MODEL_HPP

#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>

namespace ino_planner {

    class SimpleCostmapModel: public base_local_planner::WorldModel
    {
    public:
        SimpleCostmapModel(const costmap_2d::Costmap2D& costmap): costmap_(costmap) {};
        virtual ~SimpleCostmapModel(){}

        using WorldModel::footprintCost;
        /**
        * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
        * @param  position The position of the robot in world coordinates
        * @param  footprint The specification of the footprint of the robot in world coordinates
        * @param  inscribed_radius The radius of the inscribed circle of the robot
        * @param  circumscribed_radius The radius of the circumscribed circle of the robot
        * @return Positive if all the points lie outside the footprint, negative otherwise:
        *            -1 if footprint covers at least a lethal obstacle cell, or
        *            -2 if footprint covers at least a no-information cell, or
        *            -3 if footprint is [partially] outside of the map
        */
        virtual double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
          double inscribed_radius, double circumscribed_radius);

    private:
        const costmap_2d::Costmap2D& costmap_;

    };
}

#endif // SIMPLE_COSTMAP_MODEL_HPP

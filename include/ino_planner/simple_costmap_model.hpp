#ifndef SIMPLE_COSTMAP_MODEL_HPP
#define SIMPLE_COSTMAP_MODEL_HPP


#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>


namespace ino_planner
{


    /**
     * @brief A world model that evaluate a footprint with a centerpoint and peripheral points against a costmap
     */
    class SimpleCostmapModel: public base_local_planner::WorldModel
    {
    public:
        SimpleCostmapModel(const costmap_2d::Costmap2D& costmap): costmap_(costmap) {};
        virtual ~SimpleCostmapModel(){}

        using WorldModel::footprintCost;
        virtual double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
          double inscribed_radius, double circumscribed_radius);

    private:
        const costmap_2d::Costmap2D& costmap_;
    };


}


#endif // SIMPLE_COSTMAP_MODEL_HPP

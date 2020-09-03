#ifndef COPY_LAYER_H
#define COPY_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/costmap_2d.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <memory>

namespace copylayer
{

struct CellCost {
    double x;
    double y;
    unsigned int mx;
    unsigned int my;
    unsigned char cost;
};

class CopyLayer : public costmap_2d::Layer
{
public:
    CopyLayer();

    virtual void onInitialize();

    virtual void updateBounds(
        double robot_x, double robot_y, double robot_yaw, 
        double* min_x, double* min_y, 
        double* max_x, double* max_y);

    virtual void updateCosts(
        costmap_2d::Costmap2D& master_grid,
        int min_i, int min_j, 
        int max_i, int max_j);

private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    void costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    
    bool m_source_received = false;

    double m_min_x, m_max_x;
    double m_min_y, m_max_y;
    std::vector<CellCost> m_new_cell_costs;
    std::vector<CellCost> m_old_cell_costs;

    bool m_set = false;
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

    std::string m_source_costmap_name;
    bool m_persistent;

    ros::Subscriber m_sub_costmap;
    std::shared_ptr<ros::NodeHandle> m_nh;
    std::shared_ptr<ros::NodeHandle> m_nh_pub;

    unsigned char m_occ_to_cost[256];
};
} // namespace copylayer
#endif // COPY_LAYER_H

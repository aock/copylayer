#include <copylayer/CopyLayer.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(copylayer::CopyLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace copylayer
{

CopyLayer::CopyLayer() 
{
    // // special values:
    // cost_translation_table_[0] = 0;  // NO obstacle
    // cost_translation_table_[253] = 99;  // INSCRIBED obstacle
    // cost_translation_table_[254] = 100;  // LETHAL obstacle
    // cost_translation_table_[255] = -1;  // UNKNOWN

    // // regular cost values scale the range 1 to 252 (inclusive) to fit
    // // into 1 to 98 (inclusive).
    // for (int i = 1; i < 253; i++)
    // {
    //   cost_translation_table_[ i ] = char(1 + (97 * (i - 1)) / 251);
    // }

    m_occ_to_cost[0] = costmap_2d::FREE_SPACE;
    m_occ_to_cost[99] = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    m_occ_to_cost[100] = costmap_2d::LETHAL_OBSTACLE;
    m_occ_to_cost[255] = costmap_2d::NO_INFORMATION;

    // lethal obstical is inflated. we dont want this so
    m_occ_to_cost[100] = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

    // y = char(1 + (97 * (x - 1)) / 251);
    // int(y) = 1 + (97 * (x - 1)) / 251
    // int(y) - 1 = (97 * (x - 1)) / 251
    // (int(y) - 1) * 251 = 97 * (x - 1)
    // ((int(y) - 1) * 251) / 97 = x - 1
    // ((int(y) - 1) * 251) / 97 + 1 = x

    for(int i=1; i<99; i++)
    {
        m_occ_to_cost[i] = static_cast<unsigned char>(((i - 1) * 251) / 97 + 1);
    }
    // m_occ_to_cost[0] = 0;
    // m_occ_to_cost[99] = 253;
    // m_occ_to_cost[100] = 
}

void CopyLayer::onInitialize()
{
    // ros::NodeHandle nh("~/" + name_);
    m_nh.reset(new ros::NodeHandle("~/" + name_));
    m_nh_pub.reset(new ros::NodeHandle("~"));

    ROS_WARN("onInitialize");
    current_ = true;
    if(!m_nh->getParam("source", m_source_costmap_name))
    {
        throw std::runtime_error("'source' Parameter not set");
    }

    ROS_INFO_STREAM("CopyLayer using '" << m_source_costmap_name << "' as source");

    if(!m_nh->getParam("persistent", m_persistent))
    {
        m_persistent = false;
    }

    m_sub_costmap = m_nh_pub->subscribe<nav_msgs::OccupancyGrid>(m_source_costmap_name + "/costmap", 1, &CopyLayer::costmapCB, this);

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(*m_nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
        &CopyLayer::reconfigureCB, this, _1, _2);
    
    dsrv_->setCallback(cb);
}


void CopyLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
}

void CopyLayer::costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if (!enabled_)
        return;
    
    // add new data

    m_min_x = std::numeric_limits<double>::max();
    m_max_x = std::numeric_limits<double>::lowest();
    m_min_y = std::numeric_limits<double>::max();
    m_max_y = std::numeric_limits<double>::lowest();

    // transform map into target frame
    // reconstruct points + weights
    // transform to target layers frame

    for(size_t i = 0; i<msg->info.width; i++)
    {
        for(size_t j=0; j<msg->info.height; j++)
        {
            CellCost cell;
            // TODO: what if different frames? local costmap fram_id != global
            // transformation is not correct. Works only with same rotation
            cell.x = static_cast<float>(i) * msg->info.resolution + msg->info.origin.position.x;
            cell.y = static_cast<float>(j) * msg->info.resolution + msg->info.origin.position.y;
            
            char occ_cost = msg->data[msg->info.width * j + i];
            cell.cost = m_occ_to_cost[occ_cost];

            m_min_x = std::min(m_min_x, cell.x);
            m_min_y = std::min(m_min_y, cell.y);
            m_max_x = std::max(m_max_x, cell.x);
            m_max_y = std::max(m_max_y, cell.y);

            m_new_cell_costs.push_back(cell);
        }
    }

    m_source_received = true;
}

void CopyLayer::updateBounds(
    double robot_x, double robot_y, double robot_yaw, 
    double* min_x, double* min_y, 
    double* max_x, double* max_y)
{
    if (!enabled_ || !m_source_received)
        return;

    // CellCost cell_new;
    // cell_new.x = robot_x + cos(robot_yaw);
    // cell_new.y = robot_y + sin(robot_yaw);
    // cell_new.cost = 253;

    *min_x = std::min(*min_x, m_min_x);
    *min_y = std::min(*min_y, m_min_y);
    *max_x = std::max(*max_x, m_max_x);
    *max_y = std::max(*max_y, m_max_y);
}

void CopyLayer::updateCosts(
    costmap_2d::Costmap2D& master_grid, 
    int min_i, int min_j, 
    int max_i, int max_j)
{
    if (!enabled_ || !m_source_received)
        return;
    
    // if not persistent
    // -- find cells for remaining old transformed points
    // -- add weights

    // take transformed points + weights
    // find target cells
    // if weight target cell < transformed point weight, swap:
    // -- cell weight -> transformed points weight
    // -- transformed -> weight

    // dont use LETHAL_OBSTACLE: this would be inflated

    if(!m_persistent)
    {
        // ROS_INFO_STREAM("resetting " << m_old_cell_costs.size() << " cells");
        for(const CellCost& cell_old: m_old_cell_costs)
        {
            master_grid.setCost(cell_old.mx, cell_old.my, cell_old.cost);
        }
        m_old_cell_costs.resize(0);
    }

    // ROS_INFO("add new");
    
    // ROS_INFO_STREAM("adding " << m_new_cell_costs.size() << " cells");
    for(CellCost& cell_new : m_new_cell_costs)
    {
        if(master_grid.worldToMap(cell_new.x, cell_new.y, cell_new.mx, cell_new.my))
        {
            unsigned char current_cost = master_grid.getCost(cell_new.mx, cell_new.my);
            if(current_cost != LETHAL_OBSTACLE)
            {
                // swap
                master_grid.setCost(cell_new.mx, cell_new.my, cell_new.cost);
                if(!m_persistent)
                {
                    CellCost old_cell = cell_new;
                    old_cell.cost = current_cost;
                    m_old_cell_costs.push_back(old_cell);
                }
            }
        } 
    }

    m_new_cell_costs.resize(0);
    m_source_received = false;
}

} // namespace copylayer

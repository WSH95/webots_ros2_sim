#include "webots_sim_data_publisher/real_base_pose_broadcaster.hpp"

namespace webots_sim_data_publisher
{
    void WebotsBasePoseBroadcaster::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
    {
        mNode = node;
    }

    void WebotsBasePoseBroadcaster::step()
    {

    }
}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(webots_sim_data_publisher::WebotsBasePoseBroadcaster, webots_ros2_driver::PluginInterface)
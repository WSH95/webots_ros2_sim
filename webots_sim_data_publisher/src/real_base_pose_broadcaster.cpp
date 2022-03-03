#include "webots_sim_data_publisher/real_base_pose_broadcaster.hpp"

namespace webots_sim_data_publisher
{
    void WebotsBasePoseBroadcaster::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
    {
        mNode_ = node;
        mRobot_ = mNode_->robot();
        baseNode_ = mRobot_->getFromDef("ROBOT");
    }

    void WebotsBasePoseBroadcaster::step()
    {
        auto tmp_position_matrix = baseNode_->getPosition();
        std::cout << tmp_position_matrix[0] << ' ' << tmp_position_matrix[1] << ' ' << tmp_position_matrix[2] << std::endl;
        auto tmp_orientation_matrix = baseNode_->getOrientation();
        tf2::Quaternion q;

    }
}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(webots_sim_data_publisher::WebotsBasePoseBroadcaster, webots_ros2_driver::PluginInterface)
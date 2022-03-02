#ifndef REAL_BASE_POSE_BROADCASTER_HPP
#define REAL_BASE_POSE_BROADCASTER_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace webots_sim_data_publisher
{
    class WebotsBasePoseBroadcaster : public webots_ros2_driver::PluginInterface
    {
    public:
        void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
        void step() override;
    };
}

#endif // REAL_BASE_POSE_BROADCASTER_HPP
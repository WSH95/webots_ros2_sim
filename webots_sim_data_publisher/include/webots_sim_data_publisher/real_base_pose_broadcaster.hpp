#ifndef REAL_BASE_POSE_BROADCASTER_HPP
#define REAL_BASE_POSE_BROADCASTER_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>

namespace webots_sim_data_publisher
{
    class WebotsBasePoseBroadcaster : public webots_ros2_driver::PluginInterface
    {
    public:
        void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
        void step() override;

    private:
        webots_ros2_driver::WebotsNode *mNode_;
        webots::Supervisor *mRobot_;
        webots::Node *baseNode_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_{nullptr};
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        geometry_msgs::msg::TransformStamped transformStamped_;
        geometry_msgs::msg::Pose base_pose_;
    };
}

#endif // REAL_BASE_POSE_BROADCASTER_HPP
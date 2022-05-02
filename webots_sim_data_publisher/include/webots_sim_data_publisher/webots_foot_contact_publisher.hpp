#ifndef WEBOTS_FOOT_CONTACT_PUBLISHER_HPP
#define WEBOTS_FOOT_CONTACT_PUBLISHER_HPP

#include <unordered_map>
#include <array>
#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <webots/TouchSensor.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace webots_sim_data_publisher
{
    class WebotsFootContactPublisher : public webots_ros2_driver::Ros2SensorPlugin
    {
    public:
        void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
        void step() override;

    private:
        void readTouchSensors();
        void publishForceMagnitude();
        void publishForceVisual();
        void enable();
        void disable();

        int num_touchSensor;
        std::vector<webots::TouchSensor *> touchSensorList;
        std::vector<std::string> touchSensorNameList;
        std::vector<std::string> touchSensorFrameList;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mForceMagnitudePublisher;
        std_msgs::msg::Float32MultiArray mForceMagnitudeMessage;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mForceVisualPublisher;
        visualization_msgs::msg::MarkerArray mForceVisualMessage;

        std::vector<std::array<double, 3>> tmpForceList;
        rclcpp::Time currentTimeStamp;

        bool mIsEnabled;
    };
}

#endif // WEBOTS_FOOT_CONTACT_PUBLISHER_HPP
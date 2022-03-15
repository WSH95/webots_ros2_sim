#include <webots_sim_data_publisher/webots_foot_contact_publisher.hpp>
#include <regex>

namespace webots_sim_data_publisher
{
    void WebotsFootContactPublisher::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
    {
        Ros2SensorPlugin::init(node, parameters);
        mIsEnabled = false;

        if (!(parameters.count("touchSensorName") && parameters.count("touchSensorFrameName")))
            throw std::runtime_error("The WebotsFootContactPublisher plugins has to contain <touchSensorName> and <touchSensorFrameName>.");

        std::regex ws_re("\\s+"); // whitespace
        auto name_string = parameters["touchSensorName"];
        touchSensorNameList = std::vector<std::string>(std::sregex_token_iterator(name_string.begin(), name_string.end(), ws_re, -1), std::sregex_token_iterator());
        auto frame_string = parameters["touchSensorFrameName"];
        touchSensorFrameList = std::vector<std::string>(std::sregex_token_iterator(frame_string.begin(), frame_string.end(), ws_re, -1), std::sregex_token_iterator());

        num_touchSensor = touchSensorNameList.size();
        tmpForceList.resize(num_touchSensor);

        for (auto &&s : touchSensorNameList)
        {
            auto tmpTouchSensor = mNode->robot()->getTouchSensor(s);
            if (tmpTouchSensor == nullptr)
                throw std::runtime_error("Cannot find TouchSensor with name " + s);
            touchSensorList.push_back(tmpTouchSensor);
        }

        mForceMagnitudePublisher = mNode->create_publisher<std_msgs::msg::Int16MultiArray>(mTopicName + "/force_magnitude", rclcpp::SensorDataQoS().reliable());

        mForceVisualPublisher = mNode->create_publisher<visualization_msgs::msg::MarkerArray>(mTopicName + "/force_vector_visual", rclcpp::SensorDataQoS().reliable());
        mForceVisualMessage.markers.resize(num_touchSensor);
        int tmp_marker_index = 0;
        for (auto &&marker : mForceVisualMessage.markers)
        {
            marker.header.frame_id = touchSensorFrameList[tmp_marker_index];
            marker.ns = touchSensorNameList[tmp_marker_index];
            marker.type = marker.ARROW;
            marker.pose.position.x = 1;
            marker.pose.position.y = 1;
            marker.pose.position.z = 1;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = tmp_marker_index / 3.0;
            marker.color.g = tmp_marker_index / 3.0;
            marker.color.b = tmp_marker_index / 3.0;

            tmp_marker_index++;
        }

        if (mAlwaysOn)
        {
            enable();
            mIsEnabled = true;
        }
    }

    void WebotsFootContactPublisher::step()
    {
        if (!preStep())
            return;

        if (mIsEnabled)
        {
            readTouchSensors();
            publishForceMagnitude();
            // publishForceVisual();
        }

        if (mAlwaysOn)
            return;

        // Enable/Disable sensor
        const bool shouldBeEnabled = mForceMagnitudePublisher->get_subscription_count() > 0;
        if (shouldBeEnabled != mIsEnabled)
        {
            if (shouldBeEnabled)
                enable();
            else
                disable();
            mIsEnabled = shouldBeEnabled;
        }

        // // debug info
        // for (auto &&s : touchSensorNameList)
        //     std::cout << "********************************************" << s << std::endl;
        // std::cout << std::endl;
        // for (auto &&s : touchSensorFrameList)
        //     std::cout << "********************************************" << s << std::endl;
        // std::cout << std::endl;
        // std::cout << std::endl;
    }

    void WebotsFootContactPublisher::readTouchSensors()
    {
        currentTimeStamp = mNode->get_clock()->now();

        for (int i = 0; i < num_touchSensor; i++)
        {
            auto ts = touchSensorList[i];
            if (ts)
            {
                const double *values = ts->getValues();
                tmpForceList[i][0] = values[0];
                tmpForceList[i][1] = values[1];
                tmpForceList[i][2] = values[2];
            }
        }
    }

    void WebotsFootContactPublisher::publishForceMagnitude()
    {
        mForceMagnitudeMessage = std_msgs::msg::Int16MultiArray();
        for (auto &&forces : tmpForceList)
        {
            auto force_magnitude = sqrt(forces[0] * forces[0] + forces[1] * forces[1] + forces[2] * forces[2]);
            mForceMagnitudeMessage.data.push_back(force_magnitude);
        }

        mForceMagnitudePublisher->publish(mForceMagnitudeMessage);
    }

    void WebotsFootContactPublisher::publishForceVisual()
    {
        static long long marker_id_index = 0;
        int tmp_marker_index = 0;
        for (auto &&marker : mForceVisualMessage.markers)
        {
            if (mForceMagnitudeMessage.data[tmp_marker_index] < 0.001)
            {
                marker.header.stamp = currentTimeStamp;
                marker.action = marker.DELETEALL;
            }
            else
            {
                marker.header.stamp = currentTimeStamp;
                marker.id = marker_id_index;
                marker.action = marker.ADD;
                marker.lifetime = rclcpp::Duration(1);
                marker.frame_locked = true;
            }

            tmp_marker_index++;
        }

        mForceVisualPublisher->publish(mForceVisualMessage);
    }

    void WebotsFootContactPublisher::enable()
    {
        for (auto &&ts : touchSensorList)
        {
            if (ts)
                ts->enable(mPublishTimestepSyncedMs);
        }
    }

    void WebotsFootContactPublisher::disable()
    {
        for (auto &&ts : touchSensorList)
        {
            if (ts)
                ts->disable();
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(webots_sim_data_publisher::WebotsFootContactPublisher, webots_ros2_driver::PluginInterface)
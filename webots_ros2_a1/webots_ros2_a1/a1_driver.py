import rclpy
from rclpy.time import Time
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
from std_msgs.msg import Float32
import numpy as np
from rclpy.parameter import Parameter


class A1Driver:
    def init(self, webots_node, properties): 
        self.__robot = webots_node.robot

        self.__robot_node = self.__robot.getFromDef("ROBOT")
        # self.__timestep = int(self.__robot.getBasicTimeStep())

        self.body_pose = PoseWithCovarianceStamped()

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('a1_driver')
        self.__publisher = self.__node.create_publisher(
            PoseWithCovarianceStamped, 'sim_body_pose', 1)
        # self.__publisher_estimate_pose = self.__node.create_publisher(
        #     PoseWithCovarianceStamped, 'estimate_body_pose', 1)
        # self.estimate_body_pose = PoseWithCovarianceStamped()

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self.__node)

    def step(self):
        current_stamp = Time(seconds=self.__robot.getTime()).to_msg()
        self.body_pose.header.stamp = current_stamp
        self.body_pose.header.frame_id = 'world'
        # print(f"self.__robot: {self.__robot}")
        # print(f"self.__robot_node: {self.__robot_node}")
        tmp_body_pose_matrix = self.__robot_node.getPose()
        # print(tmp_body_pose_matrix)
        # tmp_body_pose_matrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 0]], dtype="float64")
        self.body_pose.pose.pose.position.x = tmp_body_pose_matrix[3]
        self.body_pose.pose.pose.position.y = tmp_body_pose_matrix[7]
        self.body_pose.pose.pose.position.z = tmp_body_pose_matrix[11]

        tmp_body_orientation_quaternion = tf_transformations.quaternion_from_matrix(
            np.resize(np.array([tmp_body_pose_matrix]), (4, 4)))
        # print(tmp_body_orientation_quaternion)
        self.body_pose.pose.pose.orientation.x = tmp_body_orientation_quaternion[0]
        self.body_pose.pose.pose.orientation.y = tmp_body_orientation_quaternion[1]
        self.body_pose.pose.pose.orientation.z = tmp_body_orientation_quaternion[2]
        self.body_pose.pose.pose.orientation.w = tmp_body_orientation_quaternion[3]

        self.body_pose.pose.covariance = [0.0] * 36

        self.__publisher.publish(self.body_pose)

        t = TransformStamped()
        t.header.stamp = current_stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'trunk'
        t.transform.translation.x = self.body_pose.pose.pose.position.x
        t.transform.translation.y = self.body_pose.pose.pose.position.y
        t.transform.translation.z = self.body_pose.pose.pose.position.z
        t.transform.rotation.x = self.body_pose.pose.pose.orientation.x
        t.transform.rotation.y = self.body_pose.pose.pose.orientation.y
        t.transform.rotation.z = self.body_pose.pose.pose.orientation.z
        t.transform.rotation.w = self.body_pose.pose.pose.orientation.w

        # Send the transformation
        # self.br.sendTransform(t)

        # publish estimate body pose
        # self.estimate_body_pose.header.stamp = Time(seconds=self.__robot.getTime()).to_msg()
        # self.estimate_body_pose.header.frame_id = 'world'
        # self.estimate_body_pose.pose.pose = self.body_pose
        # self.estimate_body_pose.pose.covariance = [0.0] * 36
        # self.__publisher_estimate_pose.publish(self.estimate_body_pose)

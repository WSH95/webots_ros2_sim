from sympy import im
import rclpy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import tf_transformations
from std_msgs.msg import Float32
import numpy as np

class A1Driver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        
        self.__robot_node = self.__robot.getFromDef("UNITREE_A1")
        # self.__timestep = int(self.__robot.getBasicTimeStep())

        self.body_pose = Pose()

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('a1_driver')
        self.__publisher = self.__node.create_publisher(Pose, 'sim_body_pose', 1)
    
    def step(self):
        print(f"self.__robot: {self.__robot}")
        print(f"self.__robot_node: {self.__robot_node}")
        # tmp_body_pose_matrix = self.__robot_node.getPose()
        # # tmp_body_pose_matrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 0]], dtype="float64")
        # self.body_pose.position.x = tmp_body_pose_matrix[3][0]
        # self.body_pose.position.y = tmp_body_pose_matrix[3][1]
        # self.body_pose.position.z = tmp_body_pose_matrix[3][2]

        # tmp_body_orientation_quaternion = tf_transformations.quaternion_from_matrix(tmp_body_pose_matrix)
        # self.body_pose.orientation.x = tmp_body_orientation_quaternion[0]
        # self.body_pose.orientation.y = tmp_body_orientation_quaternion[1]
        # self.body_pose.orientation.z = tmp_body_orientation_quaternion[2]
        # self.body_pose.orientation.w = tmp_body_orientation_quaternion[3]
        
        # self.__publisher.publish(self.body_pose)
#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose, WrenchStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from hector_uav_msgs.msg import MotorCommand


class DronePID:
    def __init__(self):
        rospy.init_node('pid_controller', anonymous=True)

        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/command/wrench', WrenchStamped, queue_size=10)

        self.state = Pose()
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

        self.thrust_kp, self.thrust_kd, self.thrust_ki = 5.0, 0.0, 0.0

        self.target_height = 10.0
        self.prev_height_error = 0.0
        self.height_error_integral = 0.0

    def odom_callback(self, data):
        self.state.position = data.pose.pose.position
        self.state.orientation = data.pose.pose.orientation

        quat = [self.state.orientation.x,
                self.state.orientation.y,
                self.state.orientation.z,
                self.state.orientation.w]
        
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quat)

        #rospy.loginfo(f"Current position (x, y, z): {self.state.position.x:.1f}, {self.state.position.y:.1f}, {self.state.position.z:.1f}")
        #rospy.loginfo(f"Current orientation (roll, pitch, yaw): {self.roll:.1f}, {self.pitch:.1f}, {self.yaw:.1f}")

        self.hover()

    def hover(self):
        height_error = self.target_height - self.state.position.z

        height_error_derivative = height_error - self.prev_height_error
        self.height_error_integral += height_error

        thrust = (self.thrust_kp * height_error +
                  self.thrust_kd * height_error_derivative +
                  self.thrust_ki * self.height_error_integral)
        
        self.prev_height_error = height_error

        command = WrenchStamped()
        command.header.frame_id = 'base_link'
        command.header.stamp = rospy.Time.now()

        command.wrench.force.z = thrust

        command.wrench.force.x = 0.0
        command.wrench.force.y = 0.0
        command.wrench.torque.x = 0.0
        command.wrench.torque.y = 0.0
        command.wrench.torque.z = 0.0

        self.cmd_pub.publish(command)


if __name__ == "__main__":
    try:
        drone = DronePID()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass        
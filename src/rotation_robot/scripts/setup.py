import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, pi
import numpy as np


class TurtleBot3:
    def __init__(self):
        rospy.init_node('move_to_goal', anonymous=True)
        self.sub = rospy.Subscriber('/odom', Odometry, self.get_odom)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # Turtlebot3 pose
        self.pos_x_2d = 0.0
        self.pos_y_2d = 0.0
        self.theta_2d = 0.0
        self.prev_theta_2d = 0.0
        self.theta_2d_sum = 0.0

        # List of goal poses
        self.goal_poses = []

    def get_odom(self, msg):
        pos_x, pos_y, theta = self.get_pose(msg)
        self.pos_x_2d = pos_x
        self.pos_y_2d = pos_y
        self.theta_2d = theta

        if (self.theta_2d - self.prev_theta_2d) > 5.:
            d_theta = (self.theta_2d - self.prev_theta_2d) - 2 * pi
        elif (self.theta_2d - self.prev_theta_2d) < -5.:
            d_theta = (self.theta_2d - self.prev_theta_2d) + 2 * pi
        else:
            d_theta = (self.theta_2d - self.prev_theta_2d)

        self.theta_2d_sum = self.theta_2d_sum + d_theta
        self.prev_theta_2d = self.theta_2d
        self.theta_2d = self.theta_2d_sum

    def get_pose(self, data):
        q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        )
        theta = euler_from_quaternion(q)[2]
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2
        pos_x = data.pose.pose.position.x
        pos_y = data.pose.pose.position.y
        return pos_x, pos_y, theta

    def get_dist(self, goal_pose):
        return sqrt(pow((goal_pose[0] - self.pos_x_2d), 2) + pow((goal_pose[1] - self.pos_y_2d), 2))

    def get_lin_x(self, goal_pose, constant=1.15):
        return constant * self.get_dist(goal_pose)

    def get_angle(self, goal_pose):
        return atan2(goal_pose[1] - self.pos_y_2d, goal_pose[0] - self.pos_x_2d)

    def get_ang_z(self, goal_pose, constant=1.15):
        return constant * (self.get_angle(goal_pose) - self.theta_2d)

    def add_goal_poses_from_matrix(self, goal_poses_matrix):
        self.goal_poses = goal_poses_matrix.tolist()

    def move_to_next_goal(self):
        if not self.goal_poses:
            rospy.loginfo("No goal poses defined.")
            return

        goal_pose = self.goal_poses.pop(0)
        rospy.loginfo("Moving to goal: %s", str(goal_pose))

        tolerance = 0.1  # Tolerance for reaching the goal
        t = Twist()
        cnt4print = 0  # Counter for print pose every second(1Hz)

        while self.get_dist(goal_pose) >= tolerance:
            if cnt4print >= 10:
                cnt4print = 0
                self.print_pose()
            cnt4print = cnt4print + 1
            t.linear.x = self.get_lin_x(goal_pose)
            t.linear.y = t.linear.z = 0
            t.angular.x = t.angular.y = 0
            t.angular.z = self.get_ang_z(goal_pose)
            self.pub.publish(t)
            self.rate.sleep()

        t.linear.x = t.angular.z = 0
        self.pub.publish(t)
        rospy.loginfo("Robot has reached the goal: %s", str(goal_pose))

    def print_pose(self):
        rospy.loginfo("p.x: %f,  p.y: %f,  th: %f", self.pos_x_2d, self.pos_y_2d, self.theta_2d)


if __name__ == '__main__':
    try:
        tb3 = TurtleBot3()
        goal_poses_matrix = np.array([[2.0, 0.0], [-3.0, 0.0], [4.0, 0.0]])  # Define goal poses as a matrix
        tb3.add_goal_poses_from_matrix(goal_poses_matrix)

        while not rospy.is_shutdown():
            tb3.move_to_next_goal()

    except rospy.ROSInterruptException:
        pass

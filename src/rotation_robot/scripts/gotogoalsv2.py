import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, pi
import numpy as np

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPD = MAX_LIN_SPEED * 0.125
ANG_SPD = MAX_ANG_SPEED * 0.125

class TurtleBot3:

    def __init__(self):
        rospy.init_node('move_to_goal', anonymous = True)
        self.sub  = rospy.Subscriber('/odom', Odometry, self.get_odom  )
        self.pub  = rospy.Publisher( '/cmd_vel', Twist, queue_size = 10)
        self.rate = rospy.Rate(10)
        
        # Turtlebot3 pose #
        self.pos_x_2d = 0.0
        self.pos_y_2d = 0.0
        self.theta_2d = 0.0
        ###################
        self.prev_theta_2d = 0.0
        self.theta_2d_sum  = 0.0
        
        self.goal_poses = []
        
        
    def get_odom(self, msg):
        pos_x, pos_y, theta = self.get_pose(msg)
        
        self.pos_x_2d = pos_x
        self.pos_y_2d = pos_y
        self.theta_2d = theta
        
    def get_pose(self, data):
        q = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
             data.pose.pose.orientation.z, data.pose.pose.orientation.w)
             
        theta = euler_from_quaternion(q)[2]	# euler_from_quaternion(q)[0] - roll
		                                    # euler_from_quaternion(q)[1] - pitch
		                                    # euler_from_quaternion(q)[2] - yaw <---
     
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2

        pos_x = data.pose.pose.position.x
        pos_y = data.pose.pose.position.y

        return pos_x, pos_y, theta
        
    def get_dist(self, goal_pose):
        return sqrt(pow((goal_pose[0] - self.pos_x_2d), 2) + pow((goal_pose[1] - self.pos_y_2d), 2))
        

        
    def get_angle(self, goal_pose):
        return atan2(goal_pose[1] - self.pos_y_2d, goal_pose[0] - self.pos_x_2d)
            

    
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

            if(cnt4print >= 10):
                cnt4print = 0
                self.print_pose()
            
            cnt4print   = cnt4print + 1
            
            angle = self.get_angle(goal_pose) - self.theta_2d
            if abs(angle) > 0.3 or abs (angle) > 5:
            	t.linear.x = 0
            	if angle >= pi:
            		t.angular.z = -0.5
            	if angle > 0 and angle < pi:
            		t.angular.z = 0.5
            	if angle < 0 and angle > -pi:
            		t.angular.z = -0.5
            	if angle <= -pi:
            		t.angular.z = 0.5
            else:
            	t.angular.z = 0
            	t.linear.x = 0.22
            self.pub.publish(t)
            self.rate.sleep()
            
        t.linear.x = t.angular.z = 0
        self.pub.publish(t)
        rospy.loginfo("Robot has reached the goal: %s", str(goal_pose))

                
	
                
    def print_pose(self):
        print("p.x: %f,  p.y: %f,  th: %f" %(self.pos_x_2d, self.pos_y_2d, self.theta_2d))
        

if __name__ == '__main__':
    try:
        tb3 = TurtleBot3()
        goal_poses_matrix = np.array([[6.80, 0.0], [7.51, 0.13], [8.07, 0.31], [8.68, 0.56], [9.20, 0.82], [9.88, 1.27], [10.33, 1.66], [10.77, 2.20], [11.12, 2.82], [11.33, 3.53], [11.49, 4.29]]) # Define goal poses as a matrix
        tb3.add_goal_poses_from_matrix(goal_poses_matrix)

        while not rospy.is_shutdown():
            tb3.move_to_next_goal()
    except rospy.ROSInterruptException:  pass


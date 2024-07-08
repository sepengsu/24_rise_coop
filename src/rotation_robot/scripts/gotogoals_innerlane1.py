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
LIN_SPD = 0.3
ANG_SPD = MAX_ANG_SPEED * 0.125

class TurtleBot3:

    def __init__(self):
        rospy.init_node('move_to_goal', anonymous = True)
        self.sub  = rospy.Subscriber('robot1/odom', Odometry, self.get_odom  )
        self.pub  = rospy.Publisher( 'robot1/cmd_vel', Twist, queue_size = 10)
        self.rate = rospy.Rate(15)
        
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
        if theta < -pi *2:
        	theta = theta + pi *2
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
        	rospy.loginfo("All goals reached. Resetting goal poses...")
        	self.add_goal_poses_from_matrix(np.array([[0, 0.3], [6.8, 0.3], [7.44, 0.42], [7.98, 0.6], [8.56, 0.84], [9.03, 1.07], [9.69, 1.51], [10.53, 2.36], [10.84, 2.93], [11.04, 3.61], [11.19, 4.32], [11.28, 5.01], [11.28, 12.76], [11.19, 13.31], [8.95, 15.53], [8.52, 15.66], [-0.94, 15.66], [-1.58, 15.71], [-1.91, 15.49], [-2.09, 15.2], [-2.09, 13.45], [-1.96, 12.86], [-1.68, 12.56], [-1.24, 12.41], [2.25, 12.41], [3.09, 12.2], [3.72, 11.68], [3.89, 10.78], [3.98, 7.72], [3.91, 6.95], [3.64, 6.26], [3.16, 5.8], [2.41, 5.57], [1.66, 5.65], [1.14, 5.94], [-9.64, 12.12], [-10.2, 12.54], [-12.43, 14.69], [-12.93, 15.07], [-13.49, 15.37], [-14.11, 15.62], [-14.78, 15.71], [-15.42, 15.76], [-15.95, 15.71], [-16.46, 15.5], [-17.0, 15.19], [-17.51, 14.67], [-17.86, 14.19], [-17.95, 13.75], [-18.01, 12.98], [-18.06, 11.08], [-17.99, 10.6], [-17.13, 7.57], [-17.02, 5.49], [-16.85, 4.77], [-16.62, 4.12], [-16.35, 3.56], [-16.02, 3.05], [-15.66, 2.64], [-15.1, 2.1], [-14.47, 1.83], [-13.84, 1.7], [-13.22, 1.5], [-12.49, 1.27], [-11.99, 1.27], [-8.67, 1.27], [-8.12, 1.13], [-7.59, 0.92], [-7.02, 0.71], [-6.4, 0.54], [-5.71, 0.34], [-5.27, 0.3], [-3.54, 0.3]]))

        tolerance = 0.12
        goal_pose = self.goal_poses.pop(0)
        rospy.loginfo("Moving to goal: %s", str(goal_pose))

        t = Twist()
        cnt4print = 0  # Counter for print pose every second(1Hz)

        distance = self.get_dist(goal_pose)
        while self.get_dist(goal_pose) >= tolerance:

            	if(cnt4print >= 10):
                	cnt4print = 0
                	self.print_pose()
            
            	cnt4print   = cnt4print + 1
            
            	angle = self.get_angle(goal_pose) - self.theta_2d
            	if abs(angle) > 0.1 and abs (angle) < 6.18:
            		t.linear.x = 0
            		if angle >= pi:
            			t.angular.z = -0.3
            		if angle >= 0 and angle < pi:
            			t.angular.z = 0.3
            		if angle < 0 and angle > -pi:
            			t.angular.z = -0.3
            		if angle <= -pi:
            			t.angular.z = 0.3
            	else:
            		t.angular.z = 0
            		t.linear.x = 0.2
            	
            		
                		
            	self.pub.publish(t)
            	self.rate.sleep()
            	distance = self.get_dist(goal_pose)
        t.linear.x = t.angular.z = 0
        self.pub.publish(t)
        rospy.loginfo("Robot has reached the goal: %s", str(goal_pose))
        	
                
	
                
    def print_pose(self):
        print("p.x: %f,  p.y: %f,  th: %f" %(self.pos_x_2d, self.pos_y_2d, self.theta_2d))
        

if __name__ == '__main__':
    try:
        tb3 = TurtleBot3()
        goal_poses_matrix = np.array([[0, 0.3], [6.8, 0.3], [7.44, 0.42], [7.98, 0.6], [8.56, 0.84], [9.03, 1.07], [9.69, 1.51], [10.53, 2.36], [10.84, 2.93], [11.04, 3.61], [11.19, 4.32], [11.28, 5.01], [11.28, 12.76], [11.19, 13.31], [8.95, 15.53], [8.52, 15.66], [-0.94, 15.66], [-1.58, 15.71], [-1.91, 15.49], [-2.09, 15.2], [-2.09, 13.45], [-1.96, 12.86], [-1.68, 12.56], [-1.24, 12.41], [2.25, 12.41], [3.09, 12.2], [3.72, 11.68], [3.89, 10.78], [3.98, 7.72], [3.91, 6.95], [3.64, 6.26], [3.16, 5.8], [2.41, 5.57], [1.66, 5.65], [1.14, 5.94], [-9.64, 12.12], [-10.2, 12.54], [-12.43, 14.69], [-12.93, 15.07], [-13.49, 15.37], [-14.11, 15.62], [-14.78, 15.71], [-15.42, 15.76], [-15.95, 15.71], [-16.46, 15.5], [-17.0, 15.19], [-17.51, 14.67], [-17.86, 14.19], [-17.95, 13.75], [-18.01, 12.98], [-18.06, 11.08], [-17.99, 10.6], [-17.13, 7.57], [-17.02, 5.49], [-16.85, 4.77], [-16.62, 4.12], [-16.35, 3.56], [-16.02, 3.05], [-15.66, 2.64], [-15.1, 2.1], [-14.47, 1.83], [-13.84, 1.7], [-13.22, 1.5], [-12.49, 1.27], [-11.99, 1.27], [-8.67, 1.27], [-8.12, 1.13], [-7.59, 0.92], [-7.02, 0.71], [-6.4, 0.54], [-5.71, 0.34], [-5.27, 0.3], [-3.54, 0.3]]) 
        tb3.add_goal_poses_from_matrix(goal_poses_matrix)

        while not rospy.is_shutdown():
        	tb3.move_to_next_goal()
		
    except rospy.ROSInterruptException:
    	rospy.loginfo("Ctrl-C caught. Exiting.")

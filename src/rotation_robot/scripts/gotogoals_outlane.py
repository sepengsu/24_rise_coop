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
        self.sub  = rospy.Subscriber('/odom', Odometry, self.get_odom  )
        self.pub  = rospy.Publisher( '/cmd_vel', Twist, queue_size = 10)
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
        	self.add_goal_poses_from_matrix(np.array([[0, -0.3], [6.8, -0.3], [7.58, -0.16], [8.18, 0.04], [8.8, 0.28], [9.37, 0.57], [10.07, 1.03], [11.03, 2.04], [11.4, 2.71], [11.62, 3.47], [11.79, 4.26], [11.88, 5.01], [11.88, 12.76], [11.73, 13.57], [9.27, 16.05], [8.52, 16.26], [-0.94, 16.26], [-1.72, 16.29], [-2.37, 15.87], [-2.69, 15.2], [-2.69, 13.45], [-2.52, 12.64], [-2.0, 12.06], [-1.24, 11.81], [2.25, 11.81], [2.83, 11.66], [3.22, 11.36], [3.29, 10.78], [3.38, 7.72], [3.33, 7.11], [3.16, 6.6], [2.82, 6.3], [2.35, 6.17], [1.92, 6.19], [1.44, 6.46], [-9.34, 12.64], [-9.78, 12.96], [-12.01, 15.11], [-12.61, 15.59], [-13.25, 15.93], [-13.99, 16.2], [-14.72, 16.31], [-15.4, 16.36], [-16.07, 16.29], [-16.74, 16.04], [-17.38, 15.65], [-17.97, 15.05], [-18.4, 14.45], [-18.55, 13.79], [-18.61, 13.0], [-18.66, 11.1], [-18.57, 10.44], [-17.71, 7.41], [-17.6, 5.35], [-17.43, 4.63], [-17.16, 3.86], [-16.87, 3.28], [-16.5, 2.69], [-16.1, 2.24], [-15.3, 1.7], [-14.8, 1.32], [-14.25, 1.09], [-13.66, 0.88], [-13.1, 0.76], [-12.51, 0.7], [-12.01, 0.7], [-8.77, 0.67], [-8.28, 0.55], [-7.85, 0.38], [-7.18, 0.13], [-6.56, -0.04], [-5.97, -0.2], [-5.27, -0.3], [-3.54, -0.3]]))

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
        goal_poses_matrix = np.array([[0, -0.3], [6.8, -0.3], [7.58, -0.16], [8.18, 0.04], [8.8, 0.28], [9.37, 0.57], [10.07, 1.03], [11.03, 2.04], [11.4, 2.71], [11.62, 3.47], [11.79, 4.26], [11.88, 5.01], [11.88, 12.76], [11.73, 13.57], [9.27, 16.05], [8.52, 16.26], [-0.94, 16.26], [-1.72, 16.29], [-2.37, 15.87], [-2.69, 15.2], [-2.69, 13.45], [-2.52, 12.64], [-2.0, 12.06], [-1.24, 11.81], [2.25, 11.81], [2.83, 11.66], [3.22, 11.36], [3.29, 10.78], [3.38, 7.72], [3.33, 7.11], [3.16, 6.6], [2.82, 6.3], [2.35, 6.17], [1.92, 6.19], [1.44, 6.46], [-9.34, 12.64], [-9.78, 12.96], [-12.01, 15.11], [-12.61, 15.59], [-13.25, 15.93], [-13.99, 16.2], [-14.72, 16.31], [-15.4, 16.36], [-16.07, 16.29], [-16.74, 16.04], [-17.38, 15.65], [-17.97, 15.05], [-18.4, 14.45], [-18.55, 13.79], [-18.61, 13.0], [-18.66, 11.1], [-18.57, 10.44], [-17.71, 7.41], [-17.6, 5.35], [-17.43, 4.63], [-17.16, 3.86], [-16.87, 3.28], [-16.5, 2.69], [-16.1, 2.24], [-15.3, 1.7], [-14.8, 1.32], [-14.25, 1.09], [-13.66, 0.88], [-13.1, 0.76], [-12.51, 0.7], [-12.01, 0.7], [-8.77, 0.67], [-8.28, 0.55], [-7.85, 0.38], [-7.18, 0.13], [-6.56, -0.04], [-5.97, -0.2], [-5.27, -0.3], [-3.54, -0.3]]) 
        tb3.add_goal_poses_from_matrix(goal_poses_matrix)

        while not rospy.is_shutdown():
        	tb3.move_to_next_goal()
		
    except rospy.ROSInterruptException:
    	rospy.loginfo("Ctrl-C caught. Exiting.")

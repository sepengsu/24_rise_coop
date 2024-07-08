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
        	self.add_goal_poses_from_matrix(np.array([[0.0, 0.0], [6.80, 0.0], [8.07, 0.31], [9.20, 0.9], [10.33, 1.66], [11.12, 2.82], [11.49, 4.29],[11.58, 5.01],[11.58, 12.75],[11.45, 13.43],[9.11, 15.78],[8.52260, 15.95],[-0.94, 15.95],[-1.65, 16.00],[-2.14, 15.67],[-2.39, 15.20],[-2.39, 13.45],[-2.24, 12.74],[-1.84, 12.31],[-1.24, 12.11],[2.0, 12.11],[3.5, 11.00],[3.59, 7.73],[3.60, 7.72],[3.62, 7.03],[3.40, 6.43],[2.99, 6.05],[2.38, 5.87],[1.80, 5.92],[1.29, 6.19],[-9.50, 12.38],[-9.99, 12.75],[-12.22, 14.90],[-13.37, 15.64],[-14.75, 16.01],[-16.01, 16.00],[-17.19, 15.41],[-18.13, 14.32],[-18.31, 12.99],[-18.36, 11.09],[-18.28, 10.52],[-17.42, 7.49],[-17.31, 5.42],[-17.14, 4.70],[-16.90, 3.99],[-16.61, 3.42],[-16.26, 2.87],[-15.88, 2.43],[-15.49, 2.03],[-14.95, 1.66],[-14.36, 1.41],[-13.75, 1.19],[-13.16, 1.07],[-12.50, 0.99],[-12.00, 0.99],[-8.72, 0.96],[-8.20, 0.84],[-7.72, 0.65],[-7.10, 0.42],[-6.48, 0.25],[-5.85, 0.07],[-5.27, 0.0]]))

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
        goal_poses_matrix = np.array([[6.80, 0.0], [8.07, 0.31], [9.20, 0.9], [10.33, 1.66], [11.12, 2.82], [11.49, 4.29],[11.58, 5.01],[11.58, 12.75],[11.45, 13.43],[9.11, 15.78],[8.52260, 15.95],[-0.94, 15.95],[-1.65, 16.00],[-2.14, 15.67],[-2.39, 15.20],[-2.39, 13.45],[-2.24, 12.74],[-1.84, 12.31],[-1.24, 12.11],[2.0, 12.11],[3.5, 11.00],[3.59, 7.73],[3.60, 7.72],[3.62, 7.03],[3.40, 6.43],[2.99, 6.05],[2.38, 5.87],[1.80, 5.92],[1.29, 6.19],[-9.50, 12.38],[-9.99, 12.75],[-12.22, 14.90],[-13.37, 15.64],[-14.75, 16.01],[-16.01, 16.00],[-17.19, 15.41],[-18.13, 14.32],[-18.31, 12.99],[-18.36, 11.09],[-18.28, 10.52],[-17.42, 7.49],[-17.31, 5.42],[-17.14, 4.70],[-16.90, 3.99],[-16.61, 3.42],[-16.26, 2.87],[-15.88, 2.43],[-15.49, 2.03],[-14.95, 1.66],[-14.36, 1.41],[-13.75, 1.19],[-13.16, 1.07],[-12.50, 0.99],[-12.00, 0.99],[-8.72, 0.96],[-8.20, 0.84],[-7.72, 0.65],[-7.10, 0.42],[-6.48, 0.25],[-5.85, 0.07],[-5.27, 0.0]]) # Define goal poses as a matrix
        tb3.add_goal_poses_from_matrix(goal_poses_matrix)

        while not rospy.is_shutdown():
        	tb3.move_to_next_goal()
		
    except rospy.ROSInterruptException:
    	rospy.loginfo("Ctrl-C caught. Exiting.")

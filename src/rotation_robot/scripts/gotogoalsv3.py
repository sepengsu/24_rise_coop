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
        	self.add_goal_poses_from_matrix(np.array([[0.00, 0.25], [6.80, 0.25], [7.76, 0.13], [8.33, 0.32],[8.93, 0.56], [9.45, 0.82], [10.13, 1.27], [11.03, 2.20],[11.37, 2.83], [11.58, 3.55], [11.74, 4.30], [11.83, 5.02],[11.83, 12.77], [11.71, 13.45], [9.36, 15.80], [8.77, 15.97],[-0.69, 15.97], [-1.40, 16.01], [-1.89, 15.70], [-2.14, 15.22],[-2.14, 13.47], [-1.99, 12.77], [-1.59, 12.33], [-1.24, 12.36],[2.25, 12.36], [3.21, 11.93], [3.72, 11.52], [3.84, 10.80],[3.93, 7.74], [3.87, 7.05], [3.65, 6.45], [3.24, 6.07],[2.63, 5.88], [2.04, 5.93], [1.54, 6.21], [-9.24, 12.39],[-9.74, 12.76], [-11.97, 14.91], [-12.52, 15.34], [-13.12, 15.66],[-13.80, 15.92], [-14.50, 16.02], [-15.16, 16.07], [-15.76, 16.01],[-16.35, 15.79], [-16.94, 15.44], [-17.49, 14.88], [-17.88, 14.34],[-18.00, 13.79], [-18.06, 13.01], [-18.11, 11.11], [-18.03, 10.54],[-17.17, 7.51], [-17.06, 5.44], [-16.89, 4.72], [-16.64, 4.01],[-16.36, 3.44], [-16.01, 2.89], [-15.63, 2.46], [-15.24, 2.05],[-14.70, 1.67], [-14.11, 1.42], [-13.50, 1.20], [-12.91, 1.08],[-12.25, 1.01], [-11.75, 1.01], [-8.47, 0.97], [-7.95, 0.84],[-7.47, 0.65], [-6.85, 0.42], [-6.23, 0.25], [-5.59, 0.07],[-5.27, 0.25], [-3.54, 0.25]]))

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
        goal_poses_matrix = np.array([[0.00, 0.25], [6.80, 0.25], [7.76, 0.13], [8.33, 0.32],[8.93, 0.56], [9.45, 0.82], [10.13, 1.27], [11.03, 2.20],[11.37, 2.83], [11.58, 3.55], [11.74, 4.30], [11.83, 5.02],[11.83, 12.77], [11.71, 13.45], [9.36, 15.80], [8.77, 15.97],[-0.69, 15.97], [-1.40, 16.01], [-1.89, 15.70], [-2.14, 15.22],[-2.14, 13.47], [-1.99, 12.77], [-1.59, 12.33], [-1.24, 12.36],[2.25, 12.36], [3.21, 11.93], [3.72, 11.52], [3.84, 10.80],[3.93, 7.74], [3.87, 7.05], [3.65, 6.45], [3.24, 6.07],[2.63, 5.88], [2.04, 5.93], [1.54, 6.21], [-9.24, 12.39],[-9.74, 12.76], [-11.97, 14.91], [-12.52, 15.34], [-13.12, 15.66],[-13.80, 15.92], [-14.50, 16.02], [-15.16, 16.07], [-15.76, 16.01],[-16.35, 15.79], [-16.94, 15.44], [-17.49, 14.88], [-17.88, 14.34],[-18.00, 13.79], [-18.06, 13.01], [-18.11, 11.11], [-18.03, 10.54],[-17.17, 7.51], [-17.06, 5.44], [-16.89, 4.72], [-16.64, 4.01],[-16.36, 3.44], [-16.01, 2.89], [-15.63, 2.46], [-15.24, 2.05],[-14.70, 1.67], [-14.11, 1.42], [-13.50, 1.20], [-12.91, 1.08],[-12.25, 1.01], [-11.75, 1.01], [-8.47, 0.97], [-7.95, 0.84],[-7.47, 0.65], [-6.85, 0.42], [-6.23, 0.25], [-5.59, 0.07],[-5.27, 0.25], [-3.54, 0.25]])  # Define goal poses as a matrix
        tb3.add_goal_poses_from_matrix(goal_poses_matrix)

        while not rospy.is_shutdown():
        	tb3.move_to_next_goal()
		
    except rospy.ROSInterruptException:
    	rospy.loginfo("Ctrl-C caught. Exiting.")

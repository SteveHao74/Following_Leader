import rospy
import math
import numpy as np
import cv2


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from dynamic_window_approch import DynamicWindowApproch
    
points_cloud = []
sub_goal = [0,0]
current_global_yaw = 0
current_pose_x = 0
current_pose_y = 0
current_linear_speed = 0
current_angular_speed = 0

def calc_yaw(orientation):
    """
    from quaternion to angle in rad / deg
    """
    atan2_y = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    atan2_x = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    yaw = math.atan2(atan2_y , atan2_x)
    # print("yaw : ", np.rad2deg(yaw))    
    return yaw

def callback_laser(self, data):        
    points_cloud = []
    collision_distance = data.range_max
    angle_min = data.angle_min
    angle_increment = data.angle_increment 

    for i in range(len(data.ranges)):
        if data.ranges[i] == float('Inf'):
            continue
        elif np.isnan(data.ranges[i]):
            continue
        else:
            angle = angle_min + i*angle_increment

            points_cloud.append([np.cos(angle)*data.ranges[i],np.sin(angle)*data.ranges[i]])

        if (collision_distance > data.ranges[i] > 0):
            collision_distance = data.ranges[i]

    print("points_cloud_num",len(points_cloud))

def callback_odom(self, data):    

    current_global_yaw = calc_yaw(data.pose.pose.orientation)
    current_pose_x = data.pose.pose.position.x
    current_pose_y = data.pose.pose.position.y
    current_pose_z = data.pose.pose.position.z
    current_linear_speed = data.twist.twist.linear.x
    current_angular_speed = data.twist.twist.angular.z
        



if __name__ == "__main__":
    # Configure depth and color streams
    rospy.init_node('dwa_planner', anonymous=True)
    rospy.Subscriber('/robot_0/scan', LaserScan, callback_laser, queue_size=1)
    rospy.Subscriber('/robot_0/odom', Odometry, callback_odom, queue_size=1)
    rospy.Subscriber('/robot_0/odom', Odometry, callback_odom, queue_size=1)

    pub_cmd_vel = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
    cmd_vel_value = Twist()

    DWA_Planner = DynamicWindowApproch()
    DWA_Planner.config(
            max_speed=3.0,
            max_yawrate=np.radians(40.0),
            base=self.base)
    rospy.spin()
    rate = rospy.Rate(1000)
    #try:
    while not rospy.is_shutdown():

        vel_vector = self.Planner.planning(
                        [current_pose_x,current_pose_y,current_global_yaw], current_linear_speed, self.goal, points_cloud)
        cmd_vel_value.linear.x, cmd_vel_value.angular.z = vel_vector[0], vel_vector[1]
        self.pub_cmd_vel.publish(cmd_vel_value)
        rate.sleep()
import rospy
import math
import numpy as np
import cv2


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from dynamic_window_approch import DynamicWindowApproch



def calc_yaw(orientation):
    """
    from quaternion to angle in rad / deg
    """
    atan2_y = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    atan2_x = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    yaw = math.atan2(atan2_y , atan2_x)
    # print("yaw : ", np.rad2deg(yaw))    
    return yaw

def global_points_publish(input_points_cloud):
    global pub_points_cloud

    msg = PointCloud2()
    msg.header.stamp = rospy.Time().now()
    msg.header.frame_id = "/robot_0_tf/odom"

    if len(input_points_cloud.shape) == 3:
        msg.height = input_points_cloud.shape[1]
        msg.width = input_points_cloud.shape[0]
    else:
        msg.height = 1
        msg.width = len(input_points_cloud)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * input_points_cloud.shape[0]
    msg.is_dense = False
    msg.data = np.asarray(input_points_cloud, np.float32).tostring()

    pub_points_cloud.publish(msg)


def callback_laser(data):   
    global points_cloud
    temp_points_cloud = []
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
            temp_points_cloud.append([np.cos(angle)*data.ranges[i],np.sin(angle)*data.ranges[i],1])#增广

        if (collision_distance > data.ranges[i] > 0):
            collision_distance = data.ranges[i]

    if len(temp_points_cloud) != 0:
        temp_points_cloud = np.array(temp_points_cloud)

        robot2world = np.identity(3)
        robot2world[:2,0] = np.array([np.cos(current_pose[2]),np.sin(current_pose[2])])
        robot2world[:2,1] = np.array([-np.sin(current_pose[2]),np.cos(current_pose[2])])
        robot2world[:2,2] = np.array(current_pose[:2])
        # print("robot2world",current_pose,robot2world)
        temp_points_cloud = (robot2world.dot(temp_points_cloud.T)).T
        points_cloud = temp_points_cloud[:,:2].tolist() 
        global_points_publish(temp_points_cloud)
    else:
        points_cloud = []
    # points_cloud = temp_points_cloud
    # print("points_cloud",points_cloud)



                
    # print("points_cloud_num",len(points_cloud))

def callback_odom(data):    
    global current_pose,current_vel
    current_pose[0] = data.pose.pose.position.x
    current_pose[1] = data.pose.pose.position.y
    current_pose[2] = calc_yaw(data.pose.pose.orientation)

    # current_pose_z = data.pose.pose.position.z
    current_vel[0] = np.linalg.norm(np.array([data.twist.twist.linear.x,data.twist.twist.linear.y])) 
    current_vel[1] = data.twist.twist.angular.z
        
def callback_subgoal(data):   
    global sub_goal 
    sub_goal[0] = data.pose.position.x 
    sub_goal[1] = data.pose.position.y 
    sub_goal[2] = calc_yaw(data.pose.orientation)

        



if __name__ == "__main__":
    # Configure depth and color streams
    rospy.init_node('dwa_planner', anonymous=True)
    rospy.Subscriber('/robot_0/scan', LaserScan, callback_laser, queue_size=1)
    rospy.Subscriber('/robot_0/odom', Odometry, callback_odom, queue_size=1)
    rospy.Subscriber('/subgoal', PoseStamped, callback_subgoal, queue_size=1)

    pub_cmd_vel = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
    pub_tag_cmd_vel = rospy.Publisher('/robot_tag/cmd_vel', Twist, queue_size=10)
    pub_points_cloud = rospy.Publisher('pointcloud_topic', PointCloud2, queue_size=5)
    cmd_vel_value = Twist()

    ###global variable init### 
    points_cloud = []
    current_pose = [0,0,0]
    current_vel = [0,0]#v,w
    sub_goal = [0,0,0]



    DWA_Planner = DynamicWindowApproch()
    DWA_Planner.config(
            max_speed=1.0,
            max_yawrate=np.radians(10.0),#
            base=0.5)
    # rospy.spin()
    rate = rospy.Rate(10)
    points_cloud = []
    #try:
    while not rospy.is_shutdown():
        # import pdb;pdb.set_trace()
        # cmd_vel_value.linear.x = rospy.get_param("/turtlebot_teleop_keyboard/scale_linear")
        # cmd_vel_value.angular.z = rospy.get_param("/turtlebot_teleop_keyboard/scale_angular")
        # print("tag_current_vel",cmd_vel_value.linear.x,cmd_vel_value.angular.z)
        # pub_tag_cmd_vel.publish(cmd_vel_value)
        # vel_vector = [0.1,0.1]
        if np.linalg.norm(np.array(current_pose[:2])-np.array(sub_goal[:2]))< 0.5: 
            kp = 0.1
            cmd_vel_value.linear.x,cmd_vel_value.angular.z = 0,0
            cmd_vel_value.angular.z = kp*(sub_goal[2] - current_pose[2]) 
        else:    
            
            vel_vector = DWA_Planner.planning(current_pose, current_vel, sub_goal[:2], points_cloud)
            cmd_vel_value.linear.x,cmd_vel_value.angular.z = vel_vector[0], vel_vector[1]
        pub_cmd_vel.publish(cmd_vel_value)
        print("sug_goal",sub_goal)
        print("cmd_vel_value",cmd_vel_value.linear.x,cmd_vel_value.angular.z)
        # print(points_cloud)
        print("current_pose",current_pose,"current_vel",current_vel)
        rate.sleep()

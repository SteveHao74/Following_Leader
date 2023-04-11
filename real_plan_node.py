import rospy
import math
import numpy as np
import cv2
import open3d


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from dynamic_window_approch import DynamicWindowApproch
from scipy.spatial.transform import Rotation as R

##global param
track_distacne = 1
camera_extrinsic_matrix = np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]]) #相机坐标系:z轴光轴,x右,y下
lidar_extrinsic_matrix = np.identity(4)
robot2world =  np.identity(4)

adjust_heading_threshold = 0.2
if_only_forward_pc = False 

SCOUT_LENGTH = 2
SCOUT_WIDTH = 1
SCOUT_MAX_VEL = 1
SCOUT_MAX_ANG_VEL = 0.2
FEASIBLE_REGION_RANGE_SCOUT = 8
####



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
    msg.header.frame_id = "/camera_init"

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

def down_sampling(in_points_cloud, voxel_size=0.1):
    
    down_points_cloud = open3d.geometry.PointCloud()
    down_points_cloud.points = open3d.utility.Vector3dVector(in_points_cloud)
    down_points_cloud = down_points_cloud.voxel_down_sample(voxel_size=voxel_size)
    
    return np.asarray(down_points_cloud.points)

def Lidar_Preprocess(lidar_data):
    global robot2world
        # start = time.time()
        # Crop
        # lidar_data = self.lidar_data
    range_mask = np.logical_and(
                    np.abs(lidar_data[:, 0]) < FEASIBLE_REGION_RANGE_SCOUT,
                    np.abs(lidar_data[:, 1]) < FEASIBLE_REGION_RANGE_SCOUT
                    )
    if if_only_forward_pc:
        range_mask = np.logical_and(
                    range_mask,
                    lidar_data[:, 0] > 0
                    )
    lidar_data = lidar_data[range_mask] 
    
    height_mask = np.logical_and(
                    lidar_data[:, 2] < 0.5,#1.5
                    lidar_data[:, 2] > -0.5#1.5
                    )
    lidar_data = lidar_data[height_mask]
    
    ego_scale = 1 # 2
    ego_mask = np.logical_or(
                    np.abs(lidar_data[:, 0]) > SCOUT_LENGTH / 2 * ego_scale, 
                    np.abs(lidar_data[:, 1]) > SCOUT_WIDTH / 2 * ego_scale
                    )
    lidar_data = lidar_data[ego_mask]
    
    # DownSampling
    points = down_sampling(lidar_data)#, None
    lidar2robot = lidar_extrinsic_matrix
    lidar2world = robot2world.dot(lidar2robot)

    points = (lidar2world[:3,:3].dot(points.T).T+lidar2world[3,:3])

    return points


def callback_obstacle_points(msg):
    global points_cloud
    # temp = []
    # points = point_cloud2.read_points_list(msg, field_names=("x", "y", "z"))
    # num = len(points)
    # for i in range(num):
    #     temp.append([points[i].x,points[i].y])
    # points_cloud = temp
    
    # print("shahao",num,points_cloud)
    cloud_gen = point_cloud2.read_points(msg)
    points_cloud = np.array(list(cloud_gen))[:, 0:3] # about 0.06s - 0.07s   




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
    global current_pose,current_vel,robot2world
    current_pose[0] = data.pose.pose.position.x
    current_pose[1] = data.pose.pose.position.y
    current_pose[2] = calc_yaw(data.pose.pose.orientation)

    robot2world[:2,0] = np.array([np.cos(current_pose[2]),np.sin(current_pose[2])])
    robot2world[:2,1] = np.array([-np.sin(current_pose[2]),np.cos(current_pose[2])])
    robot2world[:2,3] = np.array(current_pose[:2])
    # current_pose_z = data.pose.pose.position.z
    current_vel[0] = np.linalg.norm(np.array([data.twist.twist.linear.x,data.twist.twist.linear.y])) 
    current_vel[1] = data.twist.twist.angular.z
        
# def callback_subgoal(data):   
#     global sub_goal 
#     sub_goal[0] = data.pose.position.x 
#     sub_goal[1] = data.pose.position.y 
#     sub_goal[2] = calc_yaw(data.pose.orientation) 
     
def callback_subgoal(data):   
    ###recieve subgoal_s2c, transform into subgoal_s2w, and publish it.
    global sub_goal,pub_subgoal_s2w,robot2world,subgoal2robot,subgoal2world,subgoal2camera
    subgoal2camera = np.identity(4)

    r3 = R.from_quat(np.array([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]))#subgoal2world
    subgoal2camera[:3,:3] = r3.as_matrix()

    subgoal2camera[:3,3] = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z])
    # subgoal2camera[:3,3] = subgoal2camera[:3,3]# - subgoal2camera[:3,0] * track_distacne
    
    camera2robot = camera_extrinsic_matrix
    subgoal2robot = camera2robot.dot(subgoal2camera)
    subgoal2robot[:3,3] = subgoal2robot[:3,3] - subgoal2robot[:3,3]/np.linalg.norm(subgoal2robot[:3,3]) * track_distacne
    subgoal2world = robot2world.dot(subgoal2robot)
    # print("subgoal2robot:",subgoal2robot,"subgoal2world:",subgoal2world)


    sub_goal[0] = subgoal2world[0,3] 
    sub_goal[1] = subgoal2world[1,3] 
    r3 = R.from_matrix(subgoal2world[:3,:3])
    qua = r3.as_quat()# (x,
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "/camera_init"#
    msg.pose.position.x = subgoal2world[0,3]#subgoal2world
    msg.pose.position.y = subgoal2world[1,3]#subgoal2world
    msg.pose.position.z = subgoal2world[2,3]
    
    msg.pose.orientation.x = qua[0]
    msg.pose.orientation.y = qua[1]
    msg.pose.orientation.z = qua[2]
    msg.pose.orientation.w = qua[3]

    sub_goal[2] = calc_yaw(msg.pose.orientation)

    pub_subgoal_s2w.publish(msg)


        



if __name__ == "__main__":
    # Configure depth and color streams
    rospy.init_node('real_dwa_planner', anonymous=True)
    # rospy.Subscriber('/robot_0/scan', LaserScan, callback_laser, queue_size=1)
    rospy.Subscriber('/Odometry', Odometry, callback_odom, queue_size=1)
    rospy.Subscriber('/subgoal_s2c', PoseStamped, callback_subgoal, queue_size=1)
    rospy.Subscriber('/os_cloud_node/points', PointCloud2, callback_obstacle_points, queue_size=1)

    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # pub_tag_cmd_vel = rospy.Publisher('/robot_tag/cmd_vel', Twist, queue_size=10)
    pub_points_cloud = rospy.Publisher('/pointcloud_p2w', PointCloud2, queue_size=5)
    pub_subgoal_s2w = rospy.Publisher('/subgoal_s2w',PoseStamped, queue_size=10)
    
    cmd_vel_value = Twist()

    ###global variable init### 
    points_cloud = []
    current_pose = [0,0,0]
    current_vel = [0,0]#v,w
    sub_goal = [0,0,0]
    subgoal2robot = []
    subgoal2world = []
    subgoal2camera = []



    DWA_Planner = DynamicWindowApproch()
    DWA_Planner.config(
            max_speed=SCOUT_MAX_VEL,#1
            max_yawrate=SCOUT_MAX_ANG_VEL,#np.radians(10.0)
            base=SCOUT_WIDTH*0.5)#0.5
    # rospy.spin()
    rate = rospy.Rate(80)
    points_cloud = np.array([])
    points_cloud_p2w = []
    dwa_obs_points = []
    #try:
    while not rospy.is_shutdown():
        # temp_points_cloud = np.ones((4,3))
        # global_points_publish(temp_points_cloud)
        if points_cloud.shape[0]>0:

            points_cloud_p2w = Lidar_Preprocess(points_cloud)
            # print("points_cloud:",points_cloud_p2w.shape)
            global_points_publish(points_cloud_p2w)
            dwa_obs_points = points_cloud_p2w[:,:2]
        # import pdb;pdb.set_trace()
        # cmd_vel_value.linear.x = rospy.get_param("/turtlebot_teleop_keyboard/scale_linear")
        # cmd_vel_value.angular.z = rospy.get_param("/turtlebot_teleop_keyboard/scale_angular")
        # print("tag_current_vel",cmd_vel_value.linear.x,cmd_vel_value.angular.z)
        # pub_tag_cmd_vel.publish(cmd_vel_value)
        # vel_vector = [0.1,0.1]
        if np.linalg.norm(np.array(current_pose[:2])-np.array(sub_goal[:2]))< adjust_heading_threshold: 
            kp = 0.1
            cmd_vel_value.linear.x,cmd_vel_value.angular.z = 0,0
            cmd_vel_value.angular.z =0# kp*(sub_goal[2] - current_pose[2]) 
        else:    
            # print("dwa_obs_points",dwa_obs_points)
            vel_vector = DWA_Planner.planning(current_pose, current_vel, sub_goal[:2], dwa_obs_points)
            cmd_vel_value.linear.x,cmd_vel_value.angular.z = vel_vector[0]/3, vel_vector[1]/1.5
        pub_cmd_vel.publish(cmd_vel_value)
        print("sug_goal",sub_goal,"current_pose",current_pose)
        # print("subgoal2robot:",subgoal2robot,"subgoal2camera:",subgoal2camera)
        print("cmd_vel_value",cmd_vel_value.linear.x,cmd_vel_value.angular.z)
        # sub_goal[:] = current_pose[:]
        # # print(points_cloud)
        # print("current_pose",current_pose,"current_vel",current_vel)
        rate.sleep()

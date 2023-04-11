import rospy
import math
import numpy as np
import cv2


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from dynamic_window_approch import DynamicWindowApproch
import open3d
##global param
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
        


        



if __name__ == "__main__":
    # Configure depth and color streams
    rospy.init_node('preprocess_pc', anonymous=True)
    rospy.Subscriber('/Odometry', Odometry, callback_odom, queue_size=1)
    rospy.Subscriber('/os_cloud_node/points', PointCloud2, callback_obstacle_points, queue_size=1)

    pub_points_cloud = rospy.Publisher('/pointcloud_p2w', PointCloud2, queue_size=5)

    ###global variable init### 
    points_cloud = np.array([])
    current_pose = [0,0,0]
    current_vel = [0,0]#v,w
    # rospy.spin()
    rate = rospy.Rate(80)
    #try:
    while not rospy.is_shutdown():
        if points_cloud.shape[0]>0:
            points_cloud_p2w = Lidar_Preprocess(points_cloud)
            # print("points_cloud:",points_cloud_p2w.shape)
            global_points_publish(points_cloud_p2w)

        rate.sleep()

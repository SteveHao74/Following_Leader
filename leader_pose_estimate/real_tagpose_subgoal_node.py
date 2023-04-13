import numpy as np
import apriltag
import cv2
import math
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

#####param####
extrinsic_matrix = np.array([[0,0,1,-0.24],[-1,0,0,0.0061277],[0,-1,0,-0.112],[0,0,0,1]]) #相机坐标系:z轴光轴,x右,y下,
cam_params = np.array([385.45458984375, 321.5283203125, 385.1853332519531, 244.05418395996094])#fx, fy, cx, cy,realsense on 5g vehicle
# cam_params = np.array([554.254691191187, 554.254691191187, 320.5, 240.5]) #fx, fy, cx, cy
tag_len = 0.2#0.360
tag_family = 'tag36h11'
track_distacne = 2#0.3

#####initialize####
robot2world =  np.identity(4)
current_pose = [0,0,0]

def aprilTag_pose_estimate(img):
    global subgoal_pub
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    at_detector = apriltag.Detector(apriltag.DetectorOptions(families=tag_family))
    tags = at_detector.detect(gray)
    #print("tags: {}\n".format(tags))


    #48    #tag的边长（单位：mm）
    # obj2world_T = np.array([0, 0, 0])    #物体中心在世界坐标系中的坐标值
    # robot2world =  np.identity(4)
    camera2robot = extrinsic_matrix
    camera2world = robot2world.dot(camera2robot)#

    for tag in tags:       
        if tag.tag_id == 1: #世界坐标系原点为id=10的tag中心
            M, e1, e2 = at_detector.detection_pose(tag, cam_params)
            M[:3,3:] *= tag_len
            obj2camera = M
            
            subgoal2camera = np.identity(4)
            subgoal2camera[:3,0] =  obj2camera[:3,2]
            subgoal2camera[:3,1] =  -obj2camera[:3,0]
            subgoal2camera[:3,2] =  -obj2camera[:3,1]
            subgoal2camera[:3,3] =  obj2camera[:3,3] #- obj2camera[:3,2] * track_distacne

            obj2world = camera2world.dot(obj2camera)
            subgoal2world = camera2world.dot(subgoal2camera)
            print("subgoal2world:\n", subgoal2world)
            print("subgoal2camera:\n", subgoal2camera)
            print("obj2camera:\n", obj2camera)
            print("subgoal2robot:\n", camera2robot.dot(subgoal2camera))

            # subgoal2world = camera2robot.dot(subgoal2camera)
            subgoal2world[:3,3] =  subgoal2world[:3,3]# - subgoal2world[:3,0] * track_distacne
            subgoal = PoseStamped()
            subgoal.header.stamp = rospy.Time.now()
            subgoal.header.frame_id = "/robot_1/camera_init"#/base2odometry
            subgoal.pose.position.x = subgoal2camera[0,3]#subgoal2world
            subgoal.pose.position.y = subgoal2camera[1,3]#subgoal2world
            subgoal.pose.position.z = subgoal2camera[2,3]#subgoal2world
            r3 = R.from_matrix(subgoal2camera[:3,:3])#subgoal2world
            qua = r3.as_quat()# (x, y, z, w) format
            subgoal.pose.orientation.x = qua[0]
            subgoal.pose.orientation.y = qua[1]
            subgoal.pose.orientation.z = qua[2]
            subgoal.pose.orientation.w = qua[3]
            
            subgoal_pub.publish(subgoal)
            # for i in range(4):
            #     cv2.circle(img, tuple(tag.corners[i].astype(int)), 4, (255, 0, 0), 2)
            # cv2.circle(img, tuple(tag.center.astype(int)), 4, (2, 180, 200), 4)
            # cv2.imwrite("mark.png",img)


def show_colorizer_depth_img(color_image):
    # colorizer = rs.colorizer()
    # hole_filling = rs.hole_filling_filter()
    # filled_depth = hole_filling.process(depth_frame)
    # colorized_depth = np.asanyarray(colorizer.colorize(filled_depth).get_data())
    cv2.imshow('color_image',color_image)
    

def callback_img(ros_img):
    img = np.ndarray(shape=(480, 640, 3), dtype=np.dtype("uint8"), buffer=ros_img.data)
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    print("shahao")
    aprilTag_pose_estimate(img)

def calc_yaw(orientation):
    """
    from quaternion to angle in rad / deg
    """
    atan2_y = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    atan2_x = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    yaw = math.atan2(atan2_y , atan2_x)
    # print("yaw : ", np.rad2deg(yaw))    
    return yaw

def callback_odom(data):    
    current_pose[0] = data.pose.pose.position.x
    current_pose[1] = data.pose.pose.position.y
    current_pose[2] = calc_yaw(data.pose.pose.orientation)
    robot2world[:2,0] = np.array([np.cos(current_pose[2]),np.sin(current_pose[2])])
    robot2world[:2,1] = np.array([-np.sin(current_pose[2]),np.cos(current_pose[2])])
    robot2world[:2,3] = np.array(current_pose[:2])


    # current_pose_z = data.pose.pose.position.z
    # current_vel[0] = data.twist.twist.linear.x
    # current_vel[1] = data.twist.twist.angular.z
        



if __name__ == "__main__":
    # Configure depth and color streams
    rospy.init_node('tag_pose_estimate', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, callback_img)    
    rospy.Subscriber('/robot_1/Odometry', Odometry, callback_odom, queue_size=1)
    subgoal_pub = rospy.Publisher('/subgoal_s2c',PoseStamped, queue_size=10)
    rospy.spin()
    rate = rospy.Rate(1000)
    #try:
    while not rospy.is_shutdown():
        rate.sleep()



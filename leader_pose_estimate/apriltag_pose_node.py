import numpy as np
import apriltag
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

extrinsic_matrix = np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]]) #相机坐标系:z轴光轴,x右,y下
cam_params = np.array([554.254691191187, 554.254691191187, 320.5, 240.5]) #fx, fy, cx, cy
tag_len = 0.360
tag_family = 'tag36h11'
track_distacne = 2


def aprilTag_pose_estimate(img):
    # img = cv2.imread("color.png")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    at_detector = apriltag.Detector(apriltag.DetectorOptions(families=tag_family))
    tags = at_detector.detect(gray)
    #print("tags: {}\n".format(tags))


    #48    #tag的边长（单位：mm）
    # obj2world_T = np.array([0, 0, 0])    #物体中心在世界坐标系中的坐标值
    robot2world =  np.identity(4)
    camera2robot = extrinsic_matrix
    camera2world = robot2world.dot(camera2robot)#

    for tag in tags:       
        if tag.tag_id == 0: #世界坐标系原点为id=10的tag中心
            M, e1, e2 = at_detector.detection_pose(tag, cam_params)
            M[:3,3:] *= tag_len
            obj2camera = M
            
            subgoal2camera = np.identity(4)
            subgoal2camera[:3,0] =  -obj2camera[:3,0]
            subgoal2camera[:3,1] =  obj2camera[:3,2]
            subgoal2camera[:3,2] =  obj2camera[:3,1]
            subgoal2camera[:3,3] =  obj2camera[:3,3] + obj2camera[:3,0] * track_distacne

            obj2world = camera2world.dot(obj2camera)
            subgoal2world = camera2world.dot(subgoal2camera)
            print("subgoal2world:\n", subgoal2world)
            print("obj2world:\n", obj2world)
            for i in range(4):
                cv2.circle(img, tuple(tag.corners[i].astype(int)), 4, (255, 0, 0), 2)
            cv2.circle(img, tuple(tag.center.astype(int)), 4, (2, 180, 200), 4)
            cv2.imwrite("mark.png",img)


def show_colorizer_depth_img(color_image):
    # colorizer = rs.colorizer()
    # hole_filling = rs.hole_filling_filter()
    # filled_depth = hole_filling.process(depth_frame)
    # colorized_depth = np.asanyarray(colorizer.colorize(filled_depth).get_data())
    cv2.imshow('color_image',color_image)
    

def callback_img(ros_img):
    img = np.ndarray(shape=(480, 640, 3), dtype=np.dtype("uint8"), buffer=ros_img.data)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    print("shahao")
    aprilTag_pose_estimate(img)
    



if __name__ == "__main__":
    # Configure depth and color streams
    rospy.init_node('tag_pose_estimate', anonymous=True)
    rospy.Subscriber('/robot_0/camera/rgb/image_raw', Image, callback_img)    
    pose_pub = rospy.Publisher('/tag_pose',PoseStamped, queue_size=10)
    rospy.spin()
    rate = rospy.Rate(1000)
    #try:
    while not rospy.is_shutdown():
        rate.sleep()



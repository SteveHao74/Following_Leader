import numpy as np
import apriltag
import cv2
import pyrealsense2 as rs
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R


extrinsic_matrix = np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]]) #相机坐标系:z轴光轴,x右,y下
cam_params = np.array([554.254691191187, 554.254691191187, 320.5, 240.5]) 
tag_len = 0.360
tag_family = 'tag36h11'
track_distacne = 2


def aprilTag_pose_estimate(img):
    global subgoal_pub
    # img = cv2.imread("color.png")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    at_detector = apriltag.Detector(apriltag.DetectorOptions(families=tag_family))
    tags = at_detector.detect(gray)
    #print("tags: {}\n".format(tags))

    # cam_params = np.array([612.345825, 613.81781, 318.473251, 237.981806]) #fx, fy, cx, cy
    # tag_len = 48    #tag的边长（单位：mm）
    # obj2world_T = np.array([20, 95, 50])    #物体中心在世界坐标系中的坐标值

    for tag in tags:       
        if tag.tag_id == 0: #世界坐标系原点为id=10的tag中心
            M, e1, e2 = at_detector.detection_pose(tag, cam_params)
            M[:3,3:] *= tag_len
            obj2camera = M


            # obj2world = np.identity(4)
            # obj2world[:3, 3] = obj2world_T
            # world2camera = M
            # obj2camera = world2camera.dot(obj2world)
            # print("obj2camera:\n", obj2camera)

            # for i in range(4):
            #     cv2.circle(img, tuple(tag.corners[i].astype(int)), 4, (255, 0, 0), 2)
            # cv2.circle(img, tuple(tag.center.astype(int)), 4, (2, 180, 200), 4)
            # cv2.imwrite("mark.png",img)
            camera2robot = extrinsic_matrix
            subgoal2world = camera2robot.dot(obj2camera)
            subgoal = PoseStamped()
            subgoal.header.stamp = rospy.Time.now()
            subgoal.header.frame_id = "map"#
            subgoal.pose.position.x = subgoal2world[0,3]
            subgoal.pose.position.y = subgoal2world[1,3]
            subgoal.pose.position.z = subgoal2world[2,3]
            r3 = R.from_matrix(subgoal2world[:3,:3])
            qua = r3.as_quat()# (x, y, z, w) format
            subgoal.pose.orientation.x = qua[0]
            subgoal.pose.orientation.y = qua[1]
            subgoal.pose.orientation.z = qua[2]
            subgoal.pose.orientation.w = qua[3]
            
            subgoal_pub.publish(subgoal)
            print("obj2camera:\n", obj2camera)




def show_colorizer_depth_img(color_image):
    # colorizer = rs.colorizer()
    # hole_filling = rs.hole_filling_filter()
    # filled_depth = hole_filling.process(depth_frame)
    # colorized_depth = np.asanyarray(colorizer.colorize(filled_depth).get_data())
    cv2.imshow('color_image',color_image)


if __name__ == "__main__":
    # Configure depth and color streams
    rospy.init_node('tag_pose_estimate', anonymous=True)
    subgoal_pub = rospy.Publisher('/subgoal',PoseStamped, queue_size=10)
    # a = {"m":1}
    # b= "shahao"
    # for char in b:
    #     try:
    #         a[char] += 1
    #     except:
    #         a[char] = 1
    #     print(char)
    # # print(a)
    # import pdb;pdb.set_trace()

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Start streaming
    pipeline.start(config)
    #深度图像向彩色对齐
    align_to_color=rs.align(rs.stream.color)
 
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            frames = align_to_color.process(frames)
 
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # show_colorizer_depth_img(color_image)
            aprilTag_pose_estimate(color_image)

            # Press esc or 'q' to close the image window
            # key = cv2.waitKey(1)
            # if key & 0xFF == ord('q') or key == 27:
            #     cv2.destroyAllWindows()
            #     break
    

    
    finally:
        # Stop streaming
        pipeline.stop()


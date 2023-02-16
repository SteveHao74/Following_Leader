# Following_Leader


# 仿真demo
## 1.启动仿真环境:
cd ~/Shahao/navigation_research
source devel/setup.bash
roslaunch navigation_research controller_project.launch

## 2.启动领航员位姿估计和目标发布节点:
cd ~/Shahao/Following_Leader/leader_pose_estimate
python t_subgoal_node.py

## 3.启动DWA局部规划器节点:
cd ~/Shahao/Following_Leader
python local_planner_node.py

## 4.启动键盘控制领航员:
cd ~/Shahao/navigation_research
source devel/setup.bash
roslaunch turtlebot_teleop keyboard_teleop.launch 

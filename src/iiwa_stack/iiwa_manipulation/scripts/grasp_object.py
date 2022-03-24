#!/usr/bin/env python
import sys
import rospy
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown, MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
import message_filters
from gpd_ros.srv import detect_grasps_poses, detect_grasps_posesResponse
from gpd_ros.msg import GraspConfigListPose


rospy.init_node('grasp_object')
robot = RobotCommander()
rospy.sleep(1)

rospy.wait_for_service('/detect_grasps/detect_grasps_poses')

poses_from_srv = rospy.ServiceProxy('/detect_grasps/detect_grasps_poses', detect_grasps_poses)
resp = poses_from_srv()

arm = MoveGroupCommander("manipulator")
grp = MoveGroupCommander("gripper")

arm.set_named_target('camera_start1')
arm.go(wait=True)


p = PoseStamped()
p.header.frame_id = "camera_depth_optical_frame"


# print calculated poses
# print(resp.grasp_configs_poses.poses)

# choose best pose
p.pose = resp.grasp_configs_poses.poses[0].pose

# search for pose, that achievavle
# for pose in resp.grasp_configs_poses.poses:
# 	p.pose = pose
# 	print(p.pose)
# 	arm.go(p, wait=True)

# best position for grasping black box

# p.pose.position.x = 0.03134943137811154
# p.pose.position.y = 0.03338680987336645
# p.pose.position.z = 0.46800237081813543
# p.pose.orientation.w = 1
arm.go(p, wait=True)

grp.go(wait=True)
grp.set_named_target('close')
grp.go(wait=True)

rospy.sleep(15)

arm.go(wait=True)
arm.set_named_target('to_basket')
arm.go(wait=True)

grp.go(wait=True)
grp.set_named_target('open')
grp.go(wait=True)

rospy.spin()

roscpp_shutdown()

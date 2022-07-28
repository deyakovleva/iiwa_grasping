#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown, MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotState, DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Pose
import message_filters
from gpd_ros.srv import detect_grasps_poses, detect_grasps_posesResponse
from gpd_ros.msg import GraspConfigListPose
from contact_graspnet_planner.srv import ContactGraspNetPlanner, ContactGraspNetAnswer, ContactGraspNetAnswerResponse
from contact_graspnet_planner.msg import ContactGraspVect
from contact_graspnet_planner.msg import ContactGrasp

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('grasp_object')

robot = RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    DisplayTrajectory,
    queue_size=20,
)

rospy.sleep(1)

arm = MoveGroupCommander("manipulator")
arm.clear_pose_targets()
grp = MoveGroupCommander("gripper")
grp.clear_pose_targets()

arm.set_named_target('camera_start1')
arm.go(wait=True)

# for GPD
# rospy.wait_for_service('/detect_grasps/detect_grasps_poses')

rospy.wait_for_service('/responce')
print('..........waiting for service..........')

# for GPD
# poses_from_srv = rospy.ServiceProxy('/detect_grasps/detect_grasps_poses', detect_grasps_poses)
# resp = poses_from_srv()

poses_from_srv = rospy.ServiceProxy('/responce', ContactGraspNetAnswer)
resp = poses_from_srv()



p = PoseStamped()
p.header.frame_id = "world"


##########################################################################################################################################
# without segmentation

# while (True):

#     poses_from_srv = rospy.ServiceProxy('/responce', ContactGraspNetAnswer)
#     resp = poses_from_srv()

#     p.pose.position.x = resp.grasps[-1].pose.position.x 
#     p.pose.position.y = resp.grasps[-1].pose.position.y 
#     p.pose.position.z = resp.grasps[-1].pose.position.z - 0.08

#     p.pose.orientation.x = resp.grasps[-1].pose.orientation.x 
#     p.pose.orientation.y = resp.grasps[-1].pose.orientation.y 
#     p.pose.orientation.z = resp.grasps[-1].pose.orientation.z 
#     p.pose.orientation.w = resp.grasps[-1].pose.orientation.w

#     print(p.pose)
#     arm.set_pose_target(p)
#     plan = arm.go(wait=True)       

#     rospy.sleep(1)

#     grp.set_named_target('preclose')
#     grp.go(wait=True)

#     rospy.sleep(1)

#     grp.set_named_target('close')
#     grp.go(wait=True)
        
#     print('wait for grasp plugin...')
#     rospy.sleep(10)

#     arm.set_named_target('to_basket')
#     arm.go(wait=True)

#     grp.set_named_target('open')
#     grp.go(wait=True)

#     arm.set_named_target('camera_start1')
#     arm.go(wait=True)

#     rospy.wait_for_service('/responce')
##########################################################################################################################################

##########################################################################################################################################
# with segmentation

while (True):

    for i in resp.grasps:


        p.pose.position.x = i.pose.position.x 
        p.pose.position.y = i.pose.position.y 
        p.pose.position.z = i.pose.position.z - 0.06

        p.pose.orientation.x = i.pose.orientation.x 
        p.pose.orientation.y = i.pose.orientation.y 
        p.pose.orientation.z = i.pose.orientation.z 
        p.pose.orientation.w = i.pose.orientation.w

        print(p.pose)
        arm.set_pose_target(p)
        plan = arm.go(wait=True)
       

        rospy.sleep(1)
        
        grp.set_named_target('preclose')
        grp.go(wait=True)

        rospy.sleep(1)

        grp.set_named_target('close')
        grp.go(wait=True)
            
        print('wait for grasp plugin...')
        rospy.sleep(10)

        arm.set_named_target('to_basket')
        arm.go(wait=True)

        grp.set_named_target('open')
        grp.go(wait=True)

        arm.set_named_target('camera_start1')
        arm.go(wait=True)

        rospy.wait_for_service('/responce')

    poses_from_srv = rospy.ServiceProxy('/responce', ContactGraspNetAnswer)
    resp = poses_from_srv()

##########################################################################################################################################
               
rospy.spin()

roscpp_shutdown()
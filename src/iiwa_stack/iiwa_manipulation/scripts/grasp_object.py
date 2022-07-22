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

# rospy.wait_for_service('/detect_grasps/detect_grasps_poses')

rospy.wait_for_service('/responce')
print('..........waiting for service..........')

# poses_from_srv = rospy.ServiceProxy('/detect_grasps/detect_grasps_poses', detect_grasps_poses)
# resp = poses_from_srv()

poses_from_srv = rospy.ServiceProxy('/responce', ContactGraspNetAnswer)
resp = poses_from_srv()

##########################################################################################################################################

# print calculated poses
# print(resp.grasps[0].pose) # the fifth pose of the first obj

p = PoseStamped()
p.header.frame_id = "world"

# print(resp.grasps)
# print(resp.grasps[0].pose.position.x) # the fifth pose of the first obj

# без сегментации

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
       

#         # чтоб не сбивать объекты - приходим в точку сверху


#         # arm.set_joint_value_target

#     rospy.sleep(1)

#     # grp.set_named_target('preclose')
#     # grp.go(wait=True)

#     # rospy.sleep(1)

#     # grp.set_named_target('close')
#     # grp.go(wait=True)
        
#     print('sleep')
#     rospy.sleep(10)

#     arm.set_named_target('to_basket')
#     arm.go(wait=True)

#     # grp.set_named_target('open')
#     # grp.go(wait=True)

#     arm.set_named_target('camera_start1')
#     arm.go(wait=True)

#     rospy.wait_for_service('/responce')



# с сегментацией

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
        
        print('sleep')
        rospy.sleep(2)

        # arm.set_named_target('to_basket')
        # arm.go(wait=True)

        # grp.set_named_target('open')
        # grp.go(wait=True)

        arm.set_named_target('camera_start1')
        arm.go(wait=True)

        rospy.wait_for_service('/responce')

    poses_from_srv = rospy.ServiceProxy('/responce', ContactGraspNetAnswer)
    resp = poses_from_srv()



##########################################################################################################################################




# для дебага

#########################################
# p = PoseStamped()
# # p = arm.get_current_pose()
# p.header.frame_id = "world"
# # arm.go(wait=True)

# print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! get_current_pose')
# print(p)

# rospy.sleep(2)


# # pose_goal = arm.get_current_pose().pose

# # p.pose = arm.get_current_pose()

# p.pose.position.x = 0.6924505190165611
# p.pose.position.y = -0.09880217902145214
# p.pose.position.z = 0.17666847615883274 - 0.12

# p.pose.orientation.x = -0.5787131672742809
# p.pose.orientation.y = 0.7507499095400031
# p.pose.orientation.z = -0.2546187264158741
# p.pose.orientation.w = 0.1914025796780333


# arm.set_pose_target(p)
# plan = arm.go(wait=True)

#########################################





#########################################
# rospy.sleep(3)

# grp.go(wait=True)
# grp.set_named_target('cup')
# grp.go(wait=True)

# rospy.sleep(10)

# arm.go(wait=True)
# arm.set_named_target('to_basket')
# arm.go(wait=True)

# grp.go(wait=True)
# grp.set_named_target('open')
# grp.go(wait=True)

# rospy.sleep(3)

# arm.set_named_target('camera_start1')
# arm.go(wait=True)

# # cola

# print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! get_current_pose')
# print(p)

# rospy.sleep(2)

# p.pose.position.x = 0.5660452669983723
# p.pose.position.y = -0.017855429606792407
# p.pose.position.z = 0.11084236006673412
# p.pose.orientation.x = -0.6643711558253111
# p.pose.orientation.y = 0.7465573255745492
# p.pose.orientation.z = 0.005399485343913623
# p.pose.orientation.w = 0.035127944664286925

# arm.set_pose_target(p)
# plan = arm.go(wait=True)

# rospy.sleep(1)

# grp.go(wait=True)
# grp.set_named_target('cola')
# grp.go(wait=True)

# rospy.sleep(10)

# arm.go(wait=True)
# arm.set_named_target('to_basket')
# arm.go(wait=True)

# grp.go(wait=True)
# grp.set_named_target('open')
# grp.go(wait=True)

# rospy.sleep(3)

# arm.set_named_target('camera_start1')
# arm.go(wait=True)

# # p.pose.position.z = 0.09825430692239813

# # arm.set_pose_target(p)
# # plan = arm.go(wait=True)

# # arm.stop()
# # arm.clear_pose_targets()
# print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! target pose')
# print(arm.get_current_pose())
#########################################




#########################################
# # print calculated poses
# # print(resp.grasp_configs_poses.poses)

# # choose best pose             
# p.pose = resp.grasp_configs_poses.poses[0].pose

# # search for pose, that achievavle

# p = PoseStamped()
# p.header.frame_id = "world"
# for pose in resp.grasp_configs_poses.poses:
# 	p.pose = pose
# 	print(p.pose)
# 	arm.go(p, wait=True)
###########################################


                               

rospy.spin()

roscpp_shutdown()

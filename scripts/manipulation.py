import math
import moveit_commander
import rospy
import tf
from utils import *
import smach
import smach_ros

class PoseConvertions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["in_pose"], output_keys=["pg_pose","g_pose"])
    def execute(self, userdata):
        #Get grasp position relative to the 'odom' frame
        final_pose = tf2gm.PoseStamped()
        final_pose.pose = get_pose_relative_coordinate('odom',userdata.in_pose)
        final_pose.header = 'odom'

        #Get PreGrasping position
        pg_pose = tf2gm.PoseStamped()
        pg_pose = pregrasp_calc(userdata.in_pose) #has to be "tf2gm.PoseStamped()"

        #Get pregrasp position relative to the 'odom' frame
        obj_pose = tf2gm.PoseStamped()
        obj_pose.pose = get_pose_relative_coordinate('odom',pg_pose)
        obj_pose.header = 'odom'

        return "succeeded"

class PreGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["in_pose"])
    def execute(self, userdata):
        #Robot movement to pregrasping by IK
        whole_body.set_pose_target(userdata.in_pose)
        whole_body.go()

        return "succeeded"

class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["in_pose"])
    def execute(self, userdata):
        #Robot grasps
        move_hand(0.8)
        whole_body.set_pose_target(userdata.in_pose)
        whole_body.go()
        move_hand(0.8)

        return "succeeded"

class GraspCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"])
    def execute(self, userdata):
        #NOT IMPLEMENTED
        return "succeeded"

class Neutral(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"])
    def execute(self, userdata):
        #Robot to Neutral
        move_arm_neutral()

        return "succeeded"


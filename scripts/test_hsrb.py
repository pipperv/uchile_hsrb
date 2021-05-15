import math
import moveit_commander
import rospy
import tf
from utils import *

def main():
    rospy.init_node("hsrb_test")
    rospy.loginfo(arm.get_active_joints())
    value = arm.set_joint_value_target([0, 0, 0, 500, 0, 0])
    rospy.loginfo(value) 
    final = arm.go()
    rospy.loginfo(final)
    rospy.spin()

if __name__=="__main__":
    main()
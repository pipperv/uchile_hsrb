import math
import moveit_commander
import rospy
import tf
from utils import *

def main():
    rospy.init_node("hsrb_test")
    rospy.loginfo(arm.get_active_joints())
    move_hand(0.5)
    move_wholebody_ik(0.1, 1.6, 0.65, -90, 0, -90)
    move_hand(0.0)
    #rospy.spin()

if __name__=="__main__":
    main()
import math
import moveit_commander
import rospy
import tf
from utils import *

rospy.init_node("position_test")
get_relative_coordinate('odom','hand_palm_link')
rospy.loginfo(get_relative_coordinate('odom','hand_palm_link'))
import math
import moveit_commander
import rospy
import tf
from utils import *

rospy.init_node("position_test")

rel_cord = get_relative_coordinate('base_link','hand_palm_link')

p = tf2gm.PoseStamped()

p.header.frame_id = "base_link"

p.pose.position = rel_cord.translation
p.pose.orientation = rel_cord.rotation

#p.pose.position.x = 0.56185
#p.pose.position.y = 0.103366
#p.pose.position.z = 0.6501218
#p.pose.orientation.x = -0.70401285
#p.pose.orientation.y = -0.0639018
#p.pose.orientation.z = -0.70438379
#p.pose.orientation.w = 0.06425529

#p.pose.orientation = quaternion_from_euler(180, 90, 0)

#get_relative_coordinate('base_link','hand_palm_link')
rospy.loginfo(get_relative_coordinate('odom','base_link'))
rospy.loginfo(get_relative_coordinate('odom','hand_palm_link'))
rospy.loginfo(get_pose_relative_coordinate('odom',p))
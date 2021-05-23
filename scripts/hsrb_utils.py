import actionlib
import cv2
import glob
import math
import moveit_commander
import numpy as np
import os
import rospy
import ros_numpy
import subprocess
import tf
import tf2_ros
import time

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from IPython.display import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan, PointCloud2

# crea un publicador sobre el topico /hsrb/command_velocity
base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)


def move_base_vel(vx, vy, vw):

    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw / 180.0 * math.pi
    base_vel_pub.publish(twist)  # Publica en el topico de /hsrb/command_velocity

def get_current_time_sec():

    return rospy.Time.now().to_sec()

def quaternion_from_euler(roll, pitch, yaw):

    q = tf.transformations.quaternion_from_euler(roll / 180.0 * math.pi,
                                                 pitch / 180.0 * math.pi,
                                                 yaw / 180.0 * math.pi, 'rxyz')
    return Quaternion(q[0], q[1], q[2], q[3])

navclient = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

def move_base_goal(x, y, theta):

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"


    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y


    goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, theta)

    navclient.send_goal(goal)
    navclient.wait_for_result()
    state = navclient.get_state()
    # retorna True si se llegó al Goal (state = 3)
    return True if state == 3 else False

def get_relative_coordinate(parent, child):

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    trans = TransformStamped()
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(parent, child,
                                              rospy.Time().now(),
                                              rospy.Duration(4.0))
            break
        except (tf2_ros.ExtrapolationException):
            pass

    return trans.transform

whole_body = moveit_commander.MoveGroupCommander("whole_body_light")
# whole_body = moveit_commander.MoveGroupCommander("whole_body_weighted")
whole_body.allow_replanning(True)
whole_body.set_workspace([-3.0, -3.0, 3.0, 3.0])


def move_wholebody_ik(x, y, z, roll, pitch, yaw):

    p = PoseStamped()

    p.header.frame_id = "/odom"

    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z

    p.pose.orientation = quaternion_from_euler(roll, pitch, yaw)

    whole_body.set_pose_target(p)
    return whole_body.go()
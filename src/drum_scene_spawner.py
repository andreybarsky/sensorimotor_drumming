#!/usr/bin/python

# this script spawns a bunch of boxes for MoveIt's RViz interface that correspond to drum surfaces in simulation

from drum_surfaces import drum_poses, drum_names, drum_sizes
import rospy
import moveit_commander
import numpy as np
from tf.transformations import quaternion_from_euler as e2q
from geometry_msgs.msg import PoseStamped, Quaternion

rospy.init_node('drum_scene_spawner')

rospy.sleep(5) # let rviz boot up

scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

rospy.sleep(1) # let moveit latch on to everything

def spawn_box(size, pos, rpy, name):
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = pose[0]
    p.pose.position.y = pose[1]
    p.pose.position.z = pose[2] - 0.93
    quat = e2q(*rpy)
    p.pose.orientation = Quaternion(*quat)
    print('Spawning box %s with position:\n%s \nand orientation:\n%s' % (name, p.pose.position, p.pose.orientation))
    scene.add_box(name, p, size)

if __name__ == '__main__':
    for d in drum_names:
        pose = drum_poses[d]
        pos = pose[:3]
        rpy = pose[3:]
        radius = drum_sizes[d] * 1.05
        size = [radius*2 , radius*2, 0.1]
        spawn_box(size, pos, rpy, d)
else:
    lgroup = moveit_commander.MoveGroupCommander('left_arm')
    rgroup = moveit_commander.MoveGroupCommander('right_arm')

    target_drum = 'snare'
    pose_target = geometry_msgs.msg.Pose()
    drum_pose = drum_poses[target_drum]

    pose_target.orientation.w = 1.0
    pose_target.position.x = drum_pose[0]
    pose_target.position.y = drum_pose[1]
    pose_target.position.z = drum_pose[2] + 0.93 + 0.15 
    rgroup.set_pose_target(pose_target)
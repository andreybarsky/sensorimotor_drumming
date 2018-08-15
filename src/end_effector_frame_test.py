import rospy
import baxter_interface as bax
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
from baxter_core_msgs.msg import EndpointState
from gazebo_msgs.srv import SpawnModel, DeleteModel
import transform_utils as tu
from tf.transformations import quaternion_from_euler as e2q, euler_from_quaternion as q2e
import math

rospy.init_node('frame_test')

use_drumsticks = True

obj_name='right_sphere'

sphere_sdf = """<?xml version='1.0'?>
<sdf version="1.4">
<model name="%s">
  <pose>0 0 0 0 0 0</pose>
  <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.02</size>
          </sphere>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>""" % obj_name

left_pose = rospy.wait_for_message('robot/limb/left/endpoint_state', EndpointState, 1).pose
right_pose = rospy.wait_for_message('robot/limb/right/endpoint_state', EndpointState, 1).pose

def spawn_sdf(sdf_string, pose, name=obj_name):
    """takes series of (path,pose) pairs and spawns them"""
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    sdf_string = sdf_string.replace('\n', '')
    try:
        resp_sdf = spawn_sdf(name, sdf_string, "/",
                            pose, "base")
        rospy.loginfo('Successfully spawned %s' % name)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

print('Detected left end effector at: ')
print(left_pose)

print('Detected right end effector at: ')
print(right_pose)

# right_pose.position.z += 0.93

i = 0

# spawn_sdf(sphere_sdf, left_pose)
drumstick_params = [0.165, 0, 0.15, 0, 0, 0]
right_pose = rospy.wait_for_message('robot/limb/right/endpoint_state', EndpointState, 1).pose

# drumstick_params = [0.11,0.2,0,0,0,0]

# drumstick_params = [0,0,0,0,0,0]

right_eef_transform = tu.pose_to_matrix(right_pose)

rstick_transform = tu.transform_matrix(*drumstick_params)

rstick_end_transform = np.dot(right_eef_transform, rstick_transform)

rstick_p, rstick_q = tu.extract(rstick_end_transform, rot_type='quat')

right_stick_pose = Pose(position=Point(*rstick_p), orientation=Quaternion(*rstick_q))

i += 1
spawn_sdf(sphere_sdf, right_stick_pose, obj_name + str(i))

left_pose = rospy.wait_for_message('robot/limb/left/endpoint_state', EndpointState, 1).pose
left_eef_transform = tu.pose_to_matrix(left_pose)

lstick_transform = tu.transform_matrix(*drumstick_params)

lstick_end_transform = np.dot(left_eef_transform, np.asarray(drumstick_params[:3] + [1]))

lstick_p, lstick_q = tu.extract(lstick_end_transform, rot_type='quat')

left_stick_pose = Pose(position=Point(*lstick_p), orientation=Quaternion(*lstick_q))

i += 1
spawn_sdf(sphere_sdf, left_stick_pose, 'left_sphere' + str(i))
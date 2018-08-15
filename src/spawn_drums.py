#!/usr/bin/python

import rospy

from gazebo_msgs.srv import SpawnModel, DeleteModel

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Empty

from tf.transformations import quaternion_from_euler as e2q

pi = 3.14159265

model_path = '/home/andrey/.gazebo/models/'

table_path = model_path + 'table/model.sdf'
table_pose = Pose(position=Point(x=1.0, y=0.0, z=0.0),
                  orientation=Quaternion(*e2q(0.0, 0.0, -pi/2)))

drum1_path = model_path + 'disk_part/static_model.sdf'
drum1_pose = Pose(position=Point(x=0.79, y=0.42, z=1.02))

drum2_path = model_path + 'cinder_block_2/static_model.sdf'
drum2_pose = Pose(position=Point(x=0.89, y=0.00, z=1.02))

drum3_path = model_path + 'pulley_part/static_model.sdf'
drum3_pose = Pose(position=Point(x=0.76, y=-0.40, z=1.02))

def spawn_models(*model_paths_and_poses):
    """takes series of (path,pose) pairs and spawns them"""
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    for tup in model_paths_and_poses:
        path, pose = tup
        model_name = path.split('/')[-2]
        model_xml = ''
        with open(path) as model_file:
            model_xml = model_file.read().replace('\n', '')
        try:
            resp_sdf = spawn_sdf(model_name, model_xml, "/",
                                pose, "world")
            rospy.loginfo('Successfully spawned %s' % model_name)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))

print('Initialising node...')
rospy.init_node("drum_spawner")
# Wait for the All Clear from emulator startup
rospy.wait_for_message("/robot/sim/started", Empty)

# Load Gazebo Models via Spawning Services
# Note that the models reference is the /world frame
# and the IK operates with respect to the /base frame
models = [(table_path, table_pose), (drum1_path, drum1_pose),
(drum2_path, drum2_pose), (drum3_path, drum3_pose)]
print('Spawning models...')
spawn_models(*models)


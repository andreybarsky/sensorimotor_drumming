#!/usr/bin/python
 
import rospy
from drum_surfaces import drum_poses, drum_sizes, drum_names
from baxter_core_msgs.msg import EndpointState
import std_msgs
from drumming.msg import collision
from transform_utils import transform_matrix, invert_transform, pose_to_matrix
import numpy as np
 
rospy.init_node('drum_collision_detector')
 
col_height = 0.2 # length of collision cylinder
 
# the gazebo origin is different to the robot origin, so we have to correct for it
# and for some reason I cannot find the tf transform between robot base and gazebo world
robot_pos = (0, 0, 0.93, 0, 0, 0) # fixed robot pose in simulation
 
use_drumsticks = True
# x,y,z,1 offstick of drumstick tip relative to end effectors:
drumstick_offset = np.asarray([0.165, 0, 0.15, 1]) # the 1 makes this a homogenous position vector
 
# we do it here rather than to the end effector pose at runtime, to save cycles
 
for d in drum_names:
    drum_poses[d] = tuple(drum_poses[d][n] - robot_pos[n] for n in range(len(robot_pos)))
 
left_topic = 'robot/limb/left/endpoint_state'
right_topic = 'robot/limb/right/endpoint_state'
col_topic = 'drum_collision'
 
class DrumContactListener(object):
    def __init__(self, arm, endpoint_topic, pub_topic, drum_names, drum_poses, drum_sizes, col_height, rate, use_drumsticks):
        self.arm = arm
        self.endpoint_topic = endpoint_topic
        self.pub_topic = pub_topic
        self.drum_names = drum_names
        self.num_drums = len(drum_names)
        self.drum_poses = drum_poses # dict of drum_name: (x,y,z,r,p,y) cylinder poses
        self.drum_sizes = drum_sizes # dict of form {drum_name: cylinder_radius}
        self.col_height = col_height # height of collision cylinders
        self.use_drumsticks = use_drumsticks # calculate end effector position from hand or from drumsticks?
        self.rate = rospy.Rate(rate) # update rate in hz
 
        self.enabled = False
        self.rested = True # we use this to keep the update rate to some manageable number
        self.prev_pos = None
 
        # this will keep track of whether we're touching a drum, for on/off collisions:
        self.is_touching = np.ones(self.num_drums)
 
        if arm is 'left':
            self.arm_num = 0
        elif arm is 'right':
            self.arm_num = 1
 
        self.drum_transforms = []
        self.inv_transforms = []
        self.drum_radius_list = []
        for d in self.drum_names:
            px,py,pz,r,p,y = drum_poses[d]
            transform = transform_matrix(px, py, pz, r, p, y)
            self.drum_transforms.append(transform)
            self.inv_transforms.append(invert_transform(transform))
 
            self.drum_radius_list.append(self.drum_sizes[d])
 
        self.drum_radius_list = np.asarray(self.drum_radius_list)
             
        # stack them up as one matrix for efficiency:
        self.all_inv_transforms = np.concatenate(self.inv_transforms)
        # (shape is [4*6,4]), and when we multiply it by a homogenous
        # 4d position vector, the output is a single long vector of
        # that position vector in each reference frame
        # so we have to slice out the positions for each drum:
        self.pos_indices = np.arange(4,4*self.num_drums,4)
 
        self.sub = rospy.Subscriber(self.endpoint_topic, EndpointState, self.endpoint_callback)
        self.pub = rospy.Publisher(self.pub_topic, collision, queue_size=10)
 
    def endpoint_callback(self, data):
        if self.enabled:
            # if self.rested is True or self.rate is None:
            start_time = rospy.get_time()
            # rospy.loginfo('Checking %s collisions...' % self.arm)
            self.rested = False
            if self.use_drumsticks:
                # take position and orientation of end effector, apply drumstick offset
                end_transform = pose_to_matrix(data.pose)
                p = np.dot(end_transform, drumstick_offset)
            else:
                # just take position of end effector
                endpos = data.pose.position
                p = np.ones(4) # homogenous position vector
                p[0], p[1], p[2] = endpos.x, endpos.y, endpos.z

            # endpoint pose relative to drum centres:
            self.rel_endpos_vector = np.dot(self.all_inv_transforms, p)
            self.homogenous_endpos = np.asarray(np.split(self.rel_endpos_vector, self.pos_indices))
            self.rel_endpos_matrix = self.homogenous_endpos[:,:3]

            z_heights = self.rel_endpos_matrix[:,2]
            # check if hand is near the drum plane
            # by getting the list of drums whose z distance is less than
            # half the total height of the collision cylinder
            # i.e. if the hand is on the infinite x,y plane around the drum
            plane_contacts = np.where(np.abs(z_heights) < (self.col_height/2))[0]
            # then check if it's near enough the centre according to circle radius:
            if len(plane_contacts) > 0:
                plane_xy = self.rel_endpos_matrix[plane_contacts,:2]
                plane_distances = np.linalg.norm(plane_xy, axis=1)

                # radiuses for drums on the contact plane:
                contact_drum_radiuses = self.drum_radius_list[plane_contacts]
                contact_collisions = np.where(np.abs(plane_distances) < contact_drum_radiuses)[0]
                if len(contact_collisions) > 0:
                    drum_collisions = plane_contacts[contact_collisions]
                    for c in drum_collisions: # c is the index of the drum that was hit
                        # only register a beat if we were not already on the drumskin
                        if not self.is_touching[c]:
                            # get endpoint velocity:
                            if self.prev_pos is not None:
                                lin_twist = (self.prev_pos - p)[:3] # difference in x,y,z coordinates
                                endpoint_vel = np.linalg.norm(lin_twist)
                            else:
                                endpoint_vel = 0.0
                            col_msg = collision(drum=drum_names[c], drum_num=c, arm=self.arm, arm_num=self.arm_num, velocity=endpoint_vel)
                            rospy.loginfo('\n' + str(col_msg))
                            self.pub.publish(col_msg)

                # update touching:
            self.is_touching[:] = 0
            self.is_touching[plane_contacts] = 1
            self.prev_pos = p

            # self.rate.sleep()
            # self.rested = True

 
    def enable(self, on=True):
        self.enabled=on
             
update_rate = 20           
left_contact = DrumContactListener(arm='left', 
                                    endpoint_topic=left_topic, 
                                    pub_topic=col_topic,
                                    drum_names=drum_names, 
                                    drum_poses=drum_poses, 
                                    drum_sizes=drum_sizes, 
                                    col_height=col_height,
                                    rate=update_rate,
                                    use_drumsticks=use_drumsticks)
 
# stagger the updates for each arm:
rospy.sleep((1.0 /update_rate) /2)
 
right_contact = DrumContactListener(arm='right', 
                                    endpoint_topic=right_topic, 
                                    pub_topic=col_topic,
                                    drum_names=drum_names, 
                                    drum_poses=drum_poses, 
                                    drum_sizes=drum_sizes, 
                                    col_height=col_height,
                                    rate=update_rate,
                                    use_drumsticks=use_drumsticks)
 
left_contact.enable()
right_contact.enable()
 
 
rospy.loginfo('Collision detection node online at %sHz.' % update_rate)
rospy.spin()
import baxter_interface as bax
import rospy
from geometry_msgs.msg import Pose

rospy.init_node('waypoint_finder')


def get_current_pose(arm):
	"""returns current endpoint pose for given arm as a dict.
	arm should be a string in ('left', 'right')"""
	pose = bax.Limb(arm).endpoint_pose() # a dict of position and orientation
	return pose

def find_waypoints():
	"""uses command line user interface to get a list of waypoints for each arm"""
	arm_poses = {'left':[], 'right':[]}
	while True:
		print('Please move arm to the desired position.')
		print("Then type 'left' or 'right' to extract the position of that arm")
		print("Or type 'exit' to finish and return the positions.")
		arm = raw_input('> ')
		if arm == 'l':
			arm = 'left'
		elif arm == 'r':
			arm = 'right'		
		if arm in ('left', 'right'):
			pose = bax.Limb(arm).endpoint_pose()
			arm_poses[arm].append(pose)
			print('Position saved:')
			print(pose)
		elif arm == 'exit':
			return arm_poses
		else:
			print("Invalid input, type 'left' or 'right'.")

# for debugging: assume we have the following positions specified:

def record_points(num_points=3, arm='left', names=None):
	waypoints = []
	for p in range(num_points):
		if names is None:
			print('Move the %s arm to position %d' % (arm, p))
		else:
			print('Move the %s arm to %s position' % (arm, names[p]))
		print('and press enter to save.')
		raw_input('\n')
		pose = bax.Limb(arm).endpoint_pose()
		waypoints.append(pose)
	return waypoints

lrest, ldrum1, ldrum2 = record_points(3, arm='left', names=('rest', 'drum1', 'drum2'))
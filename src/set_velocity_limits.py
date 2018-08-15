#!/usr/bin/python

from baxter_base import urdf_str
import argparse

# first argument to program changes the multiplier to the base velocity/effort limits
parser = argparse.ArgumentParser()
parser.add_argument('multiplier')

args = parser.parse_args()
multiplier = float(args.multiplier)

urdf_filename = '../models/custom_baxter_base.urdf.xacro'

# change these values to increase velocity and effort limits:
vel_limit_multiplier = multiplier
eff_limit_multiplier = multiplier

# limits for shoulder, elbow and wrist velocities:
# defaults are: 1.5, 1.5, 4.0
default_vels = [2.5, 1.5, 4.0]
default_effs = [50.0, 50.0, 15.0]

vel_limits = [v*vel_limit_multiplier for v in default_vels]
eff_limits = [e*eff_limit_multiplier for e in default_effs]

new_urdf = urdf_str.format(shoulder_velocity=vel_limits[0], 
    elbow_velocity=vel_limits[1], 
    wrist_velocity=vel_limits[2],
    shoulder_effort=eff_limits[0],
    elbow_effort=eff_limits[1],
    wrist_effort=eff_limits[2])

with open(urdf_filename, 'w') as file:
    file.write(new_urdf)
print('Velocity limits set to %.2f times normal, saved new %s' % (multiplier, urdf_filename))
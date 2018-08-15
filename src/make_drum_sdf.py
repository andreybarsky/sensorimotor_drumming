#!/usr/bin/python

from drum_surfaces import drum_names, drum_poses, drum_sizes, drum_colours, colour_rgb
import rospkg

def cylinder_model(name, pose, visual_length, collision_length, radius, static=True, colour=None, alpha=1):
    """outputs an sdf model string for a cylinder parameterised by
    size, pose, and collision/visual geometry. 
        args:
    name:: a string, should be different for each model
    pose:: a pair of (x,y,z), (r,p,y) tuples for the cylinder origin
    visual_length:: total length in metres of the visual geometry of the cylinder
        (the shape will extend half this amount in either direction from the origin,
        can be None for no visual)
    collision_length:: total length of the collision geometry of the cylinder
        (can be None for no collision at all)
    radius:: cylinder radius from centre, in metres (assumed same for visual and collision)
    colour:: a tuple of rgb values, or None for no material at all
    """ 

    # unpack pose tuples to space-separated string
    # pose_str = '%s %s %s %s %s %s' % (pose[0][0], pose[0][1], pose[0][2],
    #     pose[1][0], pose[1][1], pose[1][2])
    pose_str = ' '.join(str(p) for p in pose)

    static_str = str(static).lower()

    collision_str = ""
    if collision_length is not None:
        collision_str = """
            <collision name="collision">
                <geometry>
                  <cylinder>
                    <length>%.3f</length>
                    <radius>%.3f</radius>
                  </cylinder>
                </geometry>
            </collision>""" % (collision_length, radius)

    material_str = ""
    if colour is not None:
        rgb = '%s %s %s' % (colour[0], colour[1], colour[2])
        material_str = """
                <material>
                    <ambient>%s %s</ambient>
                    <diffuse>%s %s</diffuse>
                    <specular>0.1 0.1 0.1 %s</specular>
                    <emissive>0 0 0 0</emissive>
                </material>""" % (rgb, alpha, rgb, alpha, alpha)

    visual_str = ""
    if visual_length is not None:
        visual_str = """
            <visual name="visual">
                <geometry>
                  <cylinder>
                    <length>%.3f</length>
                    <radius>%.3f</radius>
                  </cylinder>
                </geometry>
                %s
              </visual>""" % (visual_length, radius, material_str)

    model_sdf = """
        <model name="%s">
          <pose>%s</pose>
          <static>%s</static>
            <link name="body">
                %s
                %s
            </link>
          </model>""" % (name, pose_str, static_str, visual_str, collision_str)
    return model_sdf


def mesh_model(filename, pose=(0,0,0, 0,0,0), scale=1.0, collision=True, static=True):
    """Returns an sdf model string for a mesh loaded from file,
    optionally with collision same as visual geometry.
    args:
    filename:: an absolute path directory to a .dae file
        (which apparently must be collada ver 1.4 or 1.4.1)
    pose:: a pair of (x,y,z), (r,p,y) tuples for the model origin.
    scale:: a single float that scales the model in all directions
    collision:: if True, uses a collision mesh as well. else it is non-colliding
    static:: if True, model is immovable and has no gravity"""

    name = '/'.split(filename)[-1]
    visual_str = """
            <visual name="visual">
              <geometry>
                <mesh><uri>file://%s</uri>
                <scale>%s %s %s </scale></mesh>
              </geometry>
            </visual>""" % (filename, scale, scale, scale)

    collision_str = ""
    if collision is True:
        collision_str = """
            <collision name="collision">
              <geometry>
                <mesh><uri>file://%s</uri>
                <scale>%s %s %s</scale></mesh>
              </geometry>
            </collision>""" % (filename, scale, scale, scale)

    # unpack pose tuples to space-separated string
    # pose_str = '%s %s %s %s %s %s' % (pose[0][0], pose[0][1], pose[0][2],
    #     pose[1][0], pose[1][1], pose[1][2])
    pose_str = ' '.join(str(p) for p in pose)

    static_str = str(static).lower()

    model_sdf = """
        <model name="%s">
          <pose>%s</pose>
          <static>%s</static>
          <link name="body">
            %s
            %s
          </link>
        </model>""" % (name, pose_str, static_str, visual_str, collision_str)

    return model_sdf

def make_world_string(models):
    """takes sdf model specifications and inserts them into a
    string that can be saved as a world file
    """
    world_sdf = """
<?xml version="1.0" ?>
    <sdf version="1.4">
      <world name="default">
        <include>
          <uri>model://ground_plane</uri>
        </include>

        <include>
          <uri>model://sun</uri>
        </include>

        %s

        <physics type="ode">
          <real_time_update_rate>1000.0</real_time_update_rate>
          <gravity>
        0.0 0.0 -9.81
          </gravity>
        </physics>
      </world>
    </sdf>""" % models
    return world_sdf

def main():
    world_filename = 'baxter_drumset.world'

    pkg_path = rospkg.RosPack().get_path('drumming')

    world_path = pkg_path + '/world/'
    drumset_path = 'models/drumset.dae'

    full_world_path = world_path + world_filename
    full_drumset_path = world_path + drumset_path

    drum_mesh_str = mesh_model(full_drumset_path, 
        pose=(1.054, -5.454, 0.727, 1.57, 0, 1.57), 
        scale=0.005, 
        collision=False,
        static=True)

    length = 0.05
    visual_length_multiplier = 1 # cylinder visual is this many times the cylinder collision
    collision_length_multiplier = 1.1

    cylinder_models = []

    for name in drum_names:
        pose = drum_poses[name]
        radius = drum_sizes[name]
        rgb = colour_rgb[drum_colours[name]]
        # we may want the visual to extend a little bit past the collision model:
        # or vice versa
        collision_length = length * collision_length_multiplier
        visual_length = length * visual_length_multiplier 

        model_str = cylinder_model(name, 
            pose, 
            visual_length, 
            collision_length, 
            radius, 
            colour=rgb, 
            static=True)
        cylinder_models.append(model_str)

    cylinders_str = '\n\n'.join(cylinder_models)

    models = drum_mesh_str + '\n\n' + cylinders_str
    world_sdf = make_world_string(models)

    with open(full_world_path, 'w') as file:
        file.write(world_sdf)
    print('Successfully created %s\n at: %s' % (world_filename, full_world_path))

if __name__ == '__main__':
    main()
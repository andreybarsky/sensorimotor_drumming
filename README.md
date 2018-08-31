# sensorimotor_drumming
Open source framework for multimodal data collection on a simulated Baxter robot

Prerequisites:
Follow the steps for the Baxter Simulator setup guide at http://sdk.rethinkrobotics.com/wiki/Simulator_Installation
(this requires Ubuntu 14 and ROS Indigo)

Installation:
Clone this repository into ROS_DIRECTORY/src/drumming/
and run catkin_make from the ROS_DIRECTORY. 

To launch the simulator with the drum setup, use:
roslaunch drumming baxter_drums.launch

To run the collision detection node in a separate process, use:
rosrun drumming collision_detection_node.py

To run the data collection framework with default parameters, use:
rosrun drumming trajectory_mmd_recorder.py

Data are automatically saved to the trials/ directory as a .mmd file for each trial. This format can be read using the load_mmd_obj() function from the baxter_mmd module, which loads it as an object of class MultimodalData, containing image, audio, and joint attributes, as well as several visualisation methods. 

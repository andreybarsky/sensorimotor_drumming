# this is just a formattable string that can be modified
# using set_velocity_limits.py to alter the velocity/effort limits of the robot

urdf_str = """<?xml version="1.0" ?>
<robot name="baxter" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="gazebo" default="false"/>
  <xacro:if value="$(arg gazebo)">
    <!-- Gazebo Tags -->
    <xacro:include filename="$(find baxter_description)/urdf/baxter_base/baxter_base.gazebo.xacro" />
  </xacro:if>
  <link name="base">
  </link>
  <link name="torso">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/torso/base_link.DAE"/>
      </geometry>
      <material name="darkgray">
        <color rgba=".2 .2 .2 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/torso/base_link_collision.DAE"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.000000 0.000000 0.000000"/>
      <mass value="35.336455"/>
      <inertia ixx="1.849155" ixy="-0.000354" ixz="-0.154188" iyy="1.662671" iyz="0.003292" izz="0.802239"/>
    </inertial>
  </link>
  <link name="left_torso_itb">
    <inertial>
      <origin rpy="0 0 0" xyz="0.000000 0.000000 0.000000"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="right_torso_itb">
    <inertial>
      <origin rpy="0 0 0" xyz="0.000000 0.000000 0.000000"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="head">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 .00953"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/head/H0.DAE"/>
      </geometry>
      <material name="darkgray">
        <color rgba=".2 .2 .2 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.0 0 0.0"/>
      <geometry>
        <sphere radius="0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.547767"/>
      <inertia ixx="0.004641" ixy="0.000159" ixz="0.000242" iyy="0.003295" iyz="-0.001324" izz="0.003415"/>
    </inertial>
  </link>
  <link name="sonar_ring">
    <visual>
      <origin rpy="0 0 0" xyz="-.0347 0 .00953"/>
      <geometry>
        <cylinder length="0.01" radius="0.085"/>
      </geometry>
      <material name="darkgray">
        <color rgba=".2 .2 .2 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.0 0 0.0"/>
      <geometry>
        <sphere radius="0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="screen">
    <visual>
      <origin rpy="0 -1.57079632679 0" xyz="0 -.00953 -.0347"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/head/H1.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.0 0 0.0"/>
      <geometry>
        <sphere radius="0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.440171"/>
      <inertia ixx="0.004006" ixy="0.000230" ixz="0.000002" iyy="0.002800" iyz="0.000029" izz="0.001509"/>
    </inertial>
  </link>
  <link name="display">
    <visual>
      <origin rpy="0.2617993877991494 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.218 0.16 0.001"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="head_camera">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="dummyhead1">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="collision_head_link_1">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.001"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.3 0.3 0.3"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0.0" xyz="-0.07 -0.04 0.0"/>
      <geometry>
        <sphere radius="0.22"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="collision_head_link_2">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.001"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.3 0.3 0.3"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="-0.07 0.04 0.00"/>
      <geometry>
        <sphere radius="0.22"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <joint name="collision_head_1" type="fixed">
    <origin rpy="0 0 0" xyz="0.11 0 0.75"/>
    <parent link="base"/>
    <child link="collision_head_link_1"/>
  </joint>
  <joint name="collision_head_2" type="fixed">
    <origin rpy="0 0 0" xyz="0.11 0 0.75"/>
    <parent link="base"/>
    <child link="collision_head_link_2"/>
  </joint>
  <joint name="dummy" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="head"/>
    <child link="dummyhead1"/>
  </joint>
  <joint name="torso_t0" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base"/>
    <child link="torso"/>
    <limit effort="50000" lower="-3.01" upper="3.01" velocity="10000"/>
  </joint>
  <joint name="left_torso_itb_fixed" type="fixed">
    <origin rpy="-1.57079632679 3.1415 0" xyz="-0.08897 0.15593 0.389125"/>
    <parent link="torso"/>
    <child link="left_torso_itb"/>
  </joint>
  <joint name="right_torso_itb_fixed" type="fixed">
    <origin rpy="1.57079632679 0 0" xyz="-0.08897 -0.15593 0.389125"/>
    <parent link="torso"/>
    <child link="right_torso_itb"/>
  </joint>
  <joint name="head_pan" type="revolute">
    <origin rpy="0 0 0" xyz="0.06 0 0.686"/>
    <axis xyz="0 0 1"/>
    <parent link="torso"/>
    <child link="head"/>
    <limit effort="50000" lower="-1.3963" upper="1.3963" velocity="10000"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="head_nod" type="fixed">
    <origin rpy="1.75057 0 1.57079632679" xyz=".1227 0 0"/>
    <parent link="head"/>
    <child link="screen"/>
  </joint>
  <joint name="head_camera" type="fixed">
    <origin rpy="1.75057 0 1.57079632679" xyz="0.12839 0 0.06368"/>
    <parent link="head"/>
    <child link="head_camera"/>
  </joint>
  <joint name="display_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0.0 -0.016 0.0"/>
    <parent link="screen"/>
    <child link="display"/>
  </joint>
  <joint name="sonar_s0" type="fixed">
    <origin rpy="0 0 0" xyz="0.0947 0 .817"/>
    <axis xyz="0 0 1"/>
    <parent link="torso"/>
    <child link="sonar_ring"/>
  </joint>
  <link name="right_arm_mount">
    <!-- all defaults -->
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="right_upper_shoulder">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/upper_shoulder/S0.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.1361"/>
      <geometry>
        <cylinder length="0.2722" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.01783 0.00086 0.19127"/>
      <mass value="5.70044"/>
      <inertia ixx="0.04709102262" ixy="0.00012787556" ixz="0.00614870039" iyy="0.03766976455" iyz="0.00078086899" izz="0.03595988478"/>
    </inertial>
  </link>
  <link name="right_lower_shoulder">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/lower_shoulder/S1.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.12" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.06845 0.00269 -0.00529"/>
      <mass value="3.22698"/>
      <inertia ixx="0.01175209419" ixy="-0.00030096398" ixz="0.00207675762" iyy="0.0278859752" iyz="-0.00018821993" izz="0.02078749298"/>
    </inertial>
  </link>
  <link name="right_upper_elbow">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/upper_elbow/E0.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 -0.0535"/>
      <geometry>
        <cylinder length="0.107" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00276 0.00132 0.18086"/>
      <mass value="4.31272"/>
      <inertia ixx="0.02661733557" ixy="0.00029270634" ixz="0.00392189887" iyy="0.02844355207" iyz="0.0010838933" izz="0.01248008322"/>
    </inertial>
  </link>
  <link name="right_upper_elbow_visual">
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.1365"/>
      <geometry>
        <cylinder length="0.273" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="right_lower_elbow">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/lower_elbow/E1.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <geometry>
        <cylinder length="0.10" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.02611 0.00159 -0.01117"/>
      <mass value="2.07206"/>
      <inertia ixx="0.00711582686" ixy="0.00036036173" ixz="0.0007459496" iyy="0.01318227876" iyz="-0.00019663418" izz="0.00926852064"/>
    </inertial>
  </link>
  <link name="right_upper_forearm">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/upper_forearm/W0.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 -0.044"/>
      <geometry>
        <cylinder length="0.088" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00168 0.0046 0.13952"/>
      <mass value="2.24665"/>
      <inertia ixx="0.01667742825" ixy="0.00018403705" ixz="0.00018657629" iyy="0.01675457264" iyz="-0.00064732352" izz="0.0037463115"/>
    </inertial>
  </link>
  <link name="right_upper_forearm_visual">
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.136"/>
      <geometry>
        <cylinder length="0.272" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="right_arm_itb">
    <inertial>
      <origin rpy="0 0 0" xyz="0.000000 0.000000 0.000000"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="right_lower_forearm">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/lower_forearm/W1.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.10" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.06041 0.00697 0.006"/>
      <mass value="1.60979"/>
      <inertia ixx="0.00387607152" ixy="-0.00044384784" ixz="-0.00021115038" iyy="0.00700537914" iyz="0.00015348067" izz="0.0055275524"/>
    </inertial>
  </link>
  <link name="right_wrist">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/wrist/W2.DAE"/>
      </geometry>
      <material name="lightgrey">
        <color rgba=".1 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.165" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.00198 0.00125 0.01855"/>
      <mass value="0.35093"/>
      <inertia ixx="0.00025289155" ixy="0.00000575311" ixz="-0.00000159345" iyy="0.0002688601" iyz="-0.00000519818" izz="0.0003074118"/>
    </inertial>
  </link>
  <link name="right_hand">
    <collision>
      <origin rpy="0 0 0" xyz="0 0 -0.0232"/>
      <geometry>
        <cylinder length="0.464" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.01093 0.00042 -0.01532"/>
      <mass value="0.19125"/>
      <inertia ixx="0.00017588" ixy="0.00000147073" ixz="0.0000243633" iyy="0.00021166377" iyz="0.00000172689" izz="0.00023745397"/>
    </inertial>
  </link>
  <link name="right_hand_camera">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.01" radius="0.02"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="right_hand_camera_axis">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="right_hand_range">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.005 .02 .005"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="right_hand_accelerometer">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <inertial>
      <origin rpy="0 0 0" xyz="0.000000 0.000000 0.000000"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>

  <joint name="right_torso_arm_mount" type="fixed">
    <origin rpy="0 0 -0.7854" xyz="0.024645 -0.219645 0.118588"/>
    <parent link="torso"/>
    <child link="right_arm_mount"/>
  </joint>
  <joint name="right_s0" type="revolute">
    <origin rpy="0 0 0" xyz="0.055695 0 0.011038"/>
    <axis xyz="0 0 1"/>
    <parent link="right_arm_mount"/>
    <child link="right_upper_shoulder"/>
    <limit effort="{shoulder_effort}" lower="-1.70167993878" upper="1.70167993878" velocity="{shoulder_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="right_s1" type="revolute">
    <origin rpy="-1.57079632679 0 0" xyz="0.069 0 0.27035"/>
    <axis xyz="0 0 1"/>
    <parent link="right_upper_shoulder"/>
    <child link="right_lower_shoulder"/>
    <limit effort="{shoulder_effort}" lower="-2.147" upper="1.047" velocity="{shoulder_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="right_e0" type="revolute">
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.102 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="right_lower_shoulder"/>
    <child link="right_upper_elbow"/>
    <limit effort="{elbow_effort}" lower="-3.05417993878" upper="3.05417993878" velocity="{elbow_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="right_e0_fixed" type="fixed">
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.107 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="right_lower_shoulder"/>
    <child link="right_upper_elbow_visual"/>
  </joint>
  <joint name="right_e1" type="revolute">
    <origin rpy="-1.57079632679 -1.57079632679 0" xyz="0.069 0 0.26242"/>
    <axis xyz="0 0 1"/>
    <parent link="right_upper_elbow"/>
    <child link="right_lower_elbow"/>
    <limit effort="{elbow_effort}" lower="-0.05" upper="2.618" velocity="{elbow_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="right_w0" type="revolute">
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.10359 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="right_lower_elbow"/>
    <child link="right_upper_forearm"/>
    <limit effort="{wrist_effort}" lower="-3.059" upper="3.059" velocity="{wrist_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="right_w0_fixed" type="fixed">
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.088 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="right_lower_elbow"/>
    <child link="right_upper_forearm_visual"/>
  </joint>
  <joint name="right_w0_to_itb_fixed" type="fixed">
    <origin rpy="-1.57079632679 0 1.57079632679" xyz="-0.0565 0 0.12"/>
    <axis xyz="0 0 1"/>
    <parent link="right_upper_forearm"/>
    <child link="right_arm_itb"/>
  </joint>
  <joint name="right_w1" type="revolute">
    <origin rpy="-1.57079632679 -1.57079632679 0" xyz="0.01 0 0.2707"/>
    <axis xyz="0 0 1"/>
    <parent link="right_upper_forearm"/>
    <child link="right_lower_forearm"/>
    <limit effort="{wrist_effort}" lower="-1.57079632679" upper="2.094" velocity="{wrist_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="right_w2" type="revolute">
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.115975 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="right_lower_forearm"/>
    <child link="right_wrist"/>
    <limit effort="{wrist_effort}" lower="-3.059" upper="3.059" velocity="{wrist_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="right_hand" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0.11355"/>
    <axis xyz="0 0 1"/>
    <parent link="right_wrist"/>
    <child link="right_hand"/>
  </joint>
  <joint name="right_hand_camera" type="fixed">
    <origin rpy="0 0 -1.57079633" xyz="0.03825 0.012 0.015355"/>
    <parent link="right_hand"/>
    <child link="right_hand_camera"/>
  </joint>
  <joint name="right_hand_camera_axis" type="fixed">
    <origin rpy="0 0 0" xyz="0.03825 0.012 0.015355"/>
    <parent link="right_hand"/>
    <child link="right_hand_camera_axis"/>
  </joint>
  <joint name="right_hand_range" type="fixed">
    <origin rpy="0 -1.57079632679 -1.57079632679" xyz="0.032 -0.020245 0.0288"/>
    <parent link="right_hand"/>
    <child link="right_hand_range"/>
  </joint>
  <joint name="right_hand_accelerometer" type="fixed">
    <origin rpy="0 0 0" xyz="0.00198 0.000133 -0.0146"/>
    <parent link="right_hand"/>
    <child link="right_hand_accelerometer"/>
  </joint>
  <link name="left_arm_mount">
    <!-- all defaults -->
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="left_upper_shoulder">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/upper_shoulder/S0.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.1361"/>
      <geometry>
        <cylinder length="0.2722" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.01783 0.00086 0.19127"/>
      <mass value="5.70044"/>
      <inertia ixx="0.04709102262" ixy="0.00012787556" ixz="0.00614870039" iyy="0.03766976455" iyz="0.00078086899" izz="0.03595988478"/>
    </inertial>
  </link>
  <link name="left_lower_shoulder">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/lower_shoulder/S1.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.12" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.06845 0.00269 -0.00529"/>
      <mass value="3.22698"/>
      <inertia ixx="0.01175209419" ixy="-0.00030096398" ixz="0.00207675762" iyy="0.0278859752" iyz="-0.00018821993" izz="0.02078749298"/>
    </inertial>
  </link>
  <link name="left_upper_elbow">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/upper_elbow/E0.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 -0.0535"/>
      <geometry>
        <cylinder length="0.107" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00276 0.00132 0.18086"/>
      <mass value="4.31272"/>
      <inertia ixx="0.02661733557" ixy="0.00029270634" ixz="0.00392189887" iyy="0.02844355207" iyz="0.0010838933" izz="0.01248008322"/>
    </inertial>
  </link>
  <link name="left_upper_elbow_visual">
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.1365"/>
      <geometry>
        <cylinder length="0.273" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="left_lower_elbow">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/lower_elbow/E1.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <geometry>
        <cylinder length="0.10" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.02611 0.00159 -0.01117"/>
      <mass value="2.07206"/>
      <inertia ixx="0.00711582686" ixy="0.00036036173" ixz="0.0007459496" iyy="0.01318227876" iyz="-0.00019663418" izz="0.00926852064"/>
    </inertial>
  </link>
  <link name="left_upper_forearm">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/upper_forearm/W0.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 -0.044"/>
      <geometry>
        <cylinder length="0.088" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00168 0.0046 0.13952"/>
      <mass value="2.24665"/>
      <inertia ixx="0.01667742825" ixy="0.00018403705" ixz="0.00018657629" iyy="0.01675457264" iyz="-0.00064732352" izz="0.0037463115"/>
    </inertial>
  </link>
  <link name="left_upper_forearm_visual">
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.136"/>
      <geometry>
        <cylinder length="0.272" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="left_arm_itb">
    <inertial>
      <origin rpy="0 0 0" xyz="0.000000 0.000000 0.000000"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="left_lower_forearm">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/lower_forearm/W1.DAE"/>
      </geometry>
      <material name="darkred">
        <color rgba=".5 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.10" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.06041 0.00697 0.006"/>
      <mass value="1.60979"/>
      <inertia ixx="0.00387607152" ixy="-0.00044384784" ixz="-0.00021115038" iyy="0.00700537914" iyz="0.00015348067" izz="0.0055275524"/>
    </inertial>
  </link>
  <link name="left_wrist">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://baxter_description/meshes/wrist/W2.DAE"/>
      </geometry>
      <material name="lightgrey">
        <color rgba=".1 .1 .1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.165" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.00198 0.00125 0.01855"/>
      <mass value="0.35093"/>
      <inertia ixx="0.00025289155" ixy="0.00000575311" ixz="-0.00000159345" iyy="0.0002688601" iyz="-0.00000519818" izz="0.0003074118"/>
    </inertial>
  </link>
  <link name="left_hand">
    <collision>
      <origin rpy="0 0 0" xyz="0 0 -0.0232"/>
      <geometry>
        <cylinder length="0.464" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.01093 0.00042 -0.01532"/>
      <mass value="0.19125"/>
      <inertia ixx="0.00017588" ixy="0.00000147073" ixz="0.0000243633" iyy="0.00021166377" iyz="0.00000172689" izz="0.00023745397"/>
    </inertial>
  </link>
  <link name="left_hand_camera">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.01" radius="0.02"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="left_hand_camera_axis">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="left_hand_range">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.005 .02 .005"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <link name="left_hand_accelerometer">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <inertial>
      <origin rpy="0 0 0" xyz="0.000000 0.000000 0.000000"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
    </inertial>
  </link>
  <joint name="left_torso_arm_mount" type="fixed">
    <origin rpy="0 0 0.7854" xyz="0.024645 0.219645 0.118588"/>
    <parent link="torso"/>
    <child link="left_arm_mount"/>
  </joint>
  <joint name="left_s0" type="revolute">
    <origin rpy="0 0 0" xyz="0.055695 0 0.011038"/>
    <axis xyz="0 0 1"/>
    <parent link="left_arm_mount"/>
    <child link="left_upper_shoulder"/>
    <limit effort="{shoulder_effort}" lower="-1.70167993878" upper="1.70167993878" velocity="{shoulder_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="left_s1" type="revolute">
    <origin rpy="-1.57079632679 0 0" xyz="0.069 0 0.27035"/>
    <axis xyz="0 0 1"/>
    <parent link="left_upper_shoulder"/>
    <child link="left_lower_shoulder"/>
    <limit effort="{shoulder_effort}" lower="-2.147" upper="1.047" velocity="{shoulder_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="left_e0" type="revolute">
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.102 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="left_lower_shoulder"/>
    <child link="left_upper_elbow"/>
    <limit effort="{elbow_effort}" lower="-3.05417993878" upper="3.05417993878" velocity="{elbow_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="left_e0_fixed" type="fixed">
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.107 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="left_lower_shoulder"/>
    <child link="left_upper_elbow_visual"/>
  </joint>
  <joint name="left_e1" type="revolute">
    <origin rpy="-1.57079632679 -1.57079632679 0" xyz="0.069 0 0.26242"/>
    <axis xyz="0 0 1"/>
    <parent link="left_upper_elbow"/>
    <child link="left_lower_elbow"/>
    <limit effort="{elbow_effort}" lower="-0.05" upper="2.618" velocity="{elbow_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="left_w0" type="revolute">
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.10359 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="left_lower_elbow"/>
    <child link="left_upper_forearm"/>
    <limit effort="{wrist_effort}" lower="-3.059" upper="3.059" velocity="{wrist_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="left_w0_fixed" type="fixed">
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.088 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="left_lower_elbow"/>
    <child link="left_upper_forearm_visual"/>
  </joint>
  <joint name="left_w0_to_itb_fixed" type="fixed">
    <origin rpy="-1.57079632679 0 1.57079632679" xyz="-0.0565 0 0.12"/>
    <axis xyz="0 0 1"/>
    <parent link="left_upper_forearm"/>
    <child link="left_arm_itb"/>
  </joint>
  <joint name="left_w1" type="revolute">
    <origin rpy="-1.57079632679 -1.57079632679 0" xyz="0.01 0 0.2707"/>
    <axis xyz="0 0 1"/>
    <parent link="left_upper_forearm"/>
    <child link="left_lower_forearm"/>
    <limit effort="{wrist_effort}" lower="-1.57079632679" upper="2.094" velocity="{wrist_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="left_w2" type="revolute">
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.115975 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="left_lower_forearm"/>
    <child link="left_wrist"/>
    <limit effort="{wrist_effort}" lower="-3.059" upper="3.059" velocity="{wrist_velocity}"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  <joint name="left_hand" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0.11355"/>
    <axis xyz="0 0 1"/>
    <parent link="left_wrist"/>
    <child link="left_hand"/>
  </joint>
  <joint name="left_hand_camera" type="fixed">
    <origin rpy="0 0 -1.57079633" xyz="0.03825 0.012 0.015355"/>
    <parent link="left_hand"/>
    <child link="left_hand_camera"/>
  </joint>
  <joint name="left_hand_camera_axis" type="fixed">
    <origin rpy="0 0 0" xyz="0.03825 0.012 0.015355"/>
    <parent link="left_hand"/>
    <child link="left_hand_camera_axis"/>
  </joint>
  <joint name="left_hand_range" type="fixed">
    <origin rpy="0 -1.57079632679 -1.57079632679" xyz="0.032 -0.020245 0.0288"/>
    <parent link="left_hand"/>
    <child link="left_hand_range"/>
  </joint>
  <joint name="left_hand_accelerometer" type="fixed">
    <origin rpy="0 0 0" xyz="0.00198 0.000133 -0.0146"/>
    <parent link="left_hand"/>
    <child link="left_hand_accelerometer"/>
  </joint>

<!-- drumsticks: -->


  <link name="left_drumstick">
    <visual>
      <origin rpy="0 0.78 0" xyz="0.06 0 0.2"/>
      <geometry>
        <cylinder length="0.3" radius="0.012"/>
      </geometry>
      <material name="darkred">
        <color rgba=".87 .8 .72 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0.78 0" xyz="0.06 0 0.2"/>
      <geometry>
        <cylinder length="0.3" radius="0.012"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0.78 0" xyz="0.06 0 0.2"/>
      <mass value="0.00001"/>
      <inertia ixx="1e-09" ixy="0" ixz="0" iyy="1e-09" iyz="0" izz="1e-09"/>
    </inertial>
  </link>
  <joint name="left_drumstick_hold" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="left_hand"/>
    <child link="left_drumstick"/>
  </joint>


  <link name="right_drumstick">
    <visual>
      <origin rpy="0 0.78 0" xyz="0.06 0 0.2"/>
      <geometry>
        <cylinder length="0.3" radius="0.012"/>
      </geometry>
      <material name="darkred">
        <color rgba=".87 .8 .72 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0.78 0" xyz="0.06 0 0.2"/>
      <geometry>
        <cylinder length="0.3" radius="0.012"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0.78 0" xyz="0.06 0 0.2"/>
      <mass value="0.00001"/>
      <inertia ixx="1e-09" ixy="0" ixz="0" iyy="1e-09" iyz="0" izz="1e-09"/>
    </inertial>
  </link>
  <joint name="right_drumstick_hold" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="right_hand"/>
    <child link="right_drumstick"/>
  </joint>
</robot>
"""
#!/usr/bin/env python3

import sys

import rospy

import numpy as np
import time

import PyQt5.QtWidgets as qt_widgets
import PyQt5.QtCore as qt_core
from PyQt5.QtCore import Qt

from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from fabric_simulator.msg import SegmentStateArray
from fabric_simulator.msg import ChangeParticleDynamicity

from fabric_simulator.srv import GetFabricStretchingCompliance
from fabric_simulator.srv import GetFabricBendingCompliance

from fabric_simulator.srv import EnableCollisionHandling

from std_msgs.msg import Float32

import math
import tf.transformations as tf_trans

"""
Author: Burak Aksoy

The test_gui_node simulates the publishing of odometry and/or twist data for 
a set of particles within a ROS network. Utilizing the PyQt5 library, the node
provides a GUI interface which displays a series of buttons representing each 
particle, allowing the user to manually toggle the publishing state of 
individual particles.
"""

# Velocity commands will only be considered if they are spaced closer than MAX_TIMESTEP
MAX_TIMESTEP = 0.04  # Set it to ~ twice of pub rate odom

class TestGUI(qt_widgets.QWidget):
    def __init__(self):
        super(TestGUI, self).__init__()
        self.shutdown_timer = qt_core.QTimer()

        self.pub_rate_odom = rospy.get_param("~pub_rate_odom", 50)

        self.initial_values_set = False  # Initialization state variable

        self.particles = []  # All particles that is union of controllable and uncontrollable particles
        self.binded_particles = []  # Particles that are uncontrollable and binded to the leader frame

        self.odom_topic_prefix = None
        self.cmd_vel_topic_prefix = None
        self.odom_topic_leader = None
        
        simulator_node_name = "/fabric_simulator_node"
        # simulator_node_name = ""
        
        while not self.particles:
            try:
                self.particles = rospy.get_param(simulator_node_name + "/custom_static_particles")
                self.odom_topic_prefix = rospy.get_param(simulator_node_name + "/custom_static_particles_odom_topic_prefix")
                self.cmd_vel_topic_prefix = rospy.get_param(simulator_node_name + "/custom_static_particles_cmd_vel_topic_prefix")
                self.odom_topic_leader = rospy.get_param("/odom_topic_leader", "/odom_leader")
            except:
                rospy.logwarn("No particles obtained from ROS parameters. GUI will be empty.")
                time.sleep(0.5)

        self.binded_particles = list(set(self.particles))

        self.odom_publishers = {}
        self.info_binded_pose_publishers = {}

        self.particle_positions = {}
        self.particle_orientations = {}
        self.particle_twists = {}

        self.initialize_leader_position()

        # Stores Pose() msg of ROS geometry_msgs (Pose.position and Pose.orientation)
        self.binded_relative_poses = {}

        # Service clients and publishers for modulus values
        
        self.initialize_modulus_services_and_publishers(simulator_node_name)
        # self.initialize_modulus_services_and_publishers(simulator_node_name="/fabric_simulator")

        self.createUI()

        self.spacenav_twist = Twist()  # Set it to an empty twist message
        self.last_spacenav_twist_time = rospy.Time.now()  # For timestamping of the last twist msg
        self.spacenav_twist_wait_timeout = rospy.Duration(1.0)  # Timeout duration to wait twist msg before zeroing in seconds

        self.sub_twist = rospy.Subscriber("/spacenav/twist", Twist, self.spacenav_twist_callback, queue_size=1)
        self.sub_state_array = rospy.Subscriber("/fabric_state", SegmentStateArray, self.state_array_callback, queue_size=1)

        self.pub_change_dynamicity = rospy.Publisher("/change_particle_dynamicity", ChangeParticleDynamicity, queue_size=1)

        self.last_timestep_requests = {}

        self.odom_pub_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.odom_pub_timer_callback)

    def initialize_modulus_services_and_publishers(self, simulator_node_name=""):
        # Service clients
        self.get_stretching_compliance_service_name = simulator_node_name + "/get_fabric_stretching_compliance"
        self.get_bending_compliance_service_name = simulator_node_name + "/get_fabric_bending_compliance"
        self.enable_collision_handling_service_name = simulator_node_name + "/enable_collision_handling"

        # Wait for services to be available
        print("Waiting for services...")
        rospy.wait_for_service(self.get_stretching_compliance_service_name)
        rospy.wait_for_service(self.get_bending_compliance_service_name)
        print("Services are available.")

        self.get_stretching_compliance_service_client = rospy.ServiceProxy(self.get_stretching_compliance_service_name, GetFabricStretchingCompliance)
        self.get_bending_compliance_service_client = rospy.ServiceProxy(self.get_bending_compliance_service_name, GetFabricBendingCompliance)
        self.enable_collision_handling_service_client = rospy.ServiceProxy(self.enable_collision_handling_service_name, EnableCollisionHandling)

        # Publishers
        self.change_stretching_compliance_publisher = rospy.Publisher("/change_fabric_stretching_compliance", Float32, queue_size=1)
        self.change_bending_compliance_publisher = rospy.Publisher("/change_fabric_bending_compliance", Float32, queue_size=1)

    def createUI(self):
        self.layout = qt_widgets.QVBoxLayout(self)

        self.buttons_manual = {}  # To enable/disable manual control
        self.start_controller_buttons = {}  # For leader particle only
        self.bind_to_leader_buttons = {}  # For binded particles

        self.text_inputs_pos = {}  # To manually set x y z positions of the particles
        self.text_inputs_ori = {}  # To manually set x y z orientations of the particles (Euler RPY, degree input)

        # Add modulus controls at the top
        self.add_sim_controls()
        
        # Add a horizontal line here
        h_line = qt_widgets.QFrame()
        h_line.setFrameShape(qt_widgets.QFrame.HLine)
        h_line.setFrameShadow(qt_widgets.QFrame.Sunken)
        self.layout.addWidget(h_line)  # Assuming 'self.layout' is your main layout
        
        # Combine the lists with boundary markers
        combined_particles = (["leader"] + ["_boundary_marker_"] +
                              self.binded_particles + ["_boundary_marker_"])

        # Create rows for the (custom static) particles and the leader
        for particle in combined_particles:
            # Insert a horizontal line between each list (["leader"], self.binded_particles)
            if particle == "_boundary_marker_":
                # Add a horizontal line here
                h_line = qt_widgets.QFrame()
                h_line.setFrameShape(qt_widgets.QFrame.HLine)
                h_line.setFrameShadow(qt_widgets.QFrame.Sunken)
                self.layout.addWidget(h_line)  # Assuming 'self.layout' is your main layout
                continue

            # Create QHBoxLayout for each row
            row_layout = qt_widgets.QHBoxLayout()

            manual_control_button = qt_widgets.QPushButton()
            manual_control_button.setText("Manually Control " + str(particle))
            manual_control_button.setCheckable(True)  # Enables toggle behavior
            manual_control_button.setChecked(False)
            manual_control_button.clicked.connect(lambda _, p=particle: self.manual_control_button_pressed_cb(p))
            row_layout.addWidget(manual_control_button)  # Add button to row layout

            self.buttons_manual[particle] = manual_control_button

            # ------------------------------------------------
            # Add a separator vertical line here
            separator = qt_widgets.QFrame()
            separator.setFrameShape(qt_widgets.QFrame.VLine)
            separator.setFrameShadow(qt_widgets.QFrame.Sunken)
            row_layout.addWidget(separator)
            # ------------------------------------------------

            # Add button to get current pose to GUI for easy set position and orientation operations
            get_pose_button = qt_widgets.QPushButton()
            get_pose_button.setText("Get Pose")
            get_pose_button.clicked.connect(lambda _, p=particle: self.get_pose_button_pressed_cb(p))
            row_layout.addWidget(get_pose_button)

            # ------------------------------------------------
            # Add a separator vertical line here
            separator = qt_widgets.QFrame()
            separator.setFrameShape(qt_widgets.QFrame.VLine)
            separator.setFrameShadow(qt_widgets.QFrame.Sunken)
            row_layout.addWidget(separator)
            # ------------------------------------------------

            # Create LineEdits and Add to row layout
            self.text_inputs_pos[particle] = {}
            for axis in ['x', 'y', 'z']:
                label = qt_widgets.QLabel(axis + ':')
                line_edit = qt_widgets.QLineEdit()

                if axis == 'x':
                    line_edit.setText(str(0.0))
                elif axis == 'y':
                    line_edit.setText(str(0.0))
                elif axis == 'z':
                    line_edit.setText(str(0.0))

                row_layout.addWidget(label)
                row_layout.addWidget(line_edit)
                self.text_inputs_pos[particle][axis] = line_edit

            # Create Set Position button
            set_pos_button = qt_widgets.QPushButton()
            set_pos_button.setText("Set Position")
            # set_pos_button.clicked.connect(lambda _, p=particle: self.set_position_cb_basic(p))
            set_pos_button.clicked.connect(lambda _, p=particle: self.set_position_cb(p))
            row_layout.addWidget(set_pos_button)

            # ------------------------------------------------
            # Add a separator vertical line here
            separator = qt_widgets.QFrame()
            separator.setFrameShape(qt_widgets.QFrame.VLine)
            separator.setFrameShadow(qt_widgets.QFrame.Sunken)
            row_layout.addWidget(separator)
            # ------------------------------------------------

            # Add Set orientation button text inputs
            self.text_inputs_ori[particle] = {}
            for axis in ['x', 'y', 'z']:
                label = qt_widgets.QLabel(axis + ':')
                line_edit = qt_widgets.QLineEdit()

                if axis == 'x':
                    line_edit.setText(str(0.0))
                elif axis == 'y':
                    line_edit.setText(str(0.0))
                elif axis == 'z':
                    line_edit.setText(str(0.0))

                row_layout.addWidget(label)
                row_layout.addWidget(line_edit)
                self.text_inputs_ori[particle][axis] = line_edit

            # Create Set Orientation button
            set_ori_button = qt_widgets.QPushButton()
            set_ori_button.setText("Set Orientation")
            # set_ori_button.clicked.connect(lambda _, p=particle: self.set_orientation_cb_basic(p))
            set_ori_button.clicked.connect(lambda _, p=particle: self.set_orientation_cb(p))
            row_layout.addWidget(set_ori_button)

            # ------------------------------------------------
            # Add a separator vertical line here
            separator = qt_widgets.QFrame()
            separator.setFrameShape(qt_widgets.QFrame.VLine)
            separator.setFrameShadow(qt_widgets.QFrame.Sunken)
            row_layout.addWidget(separator)
            # ------------------------------------------------

            # Create Pause Controller Button or Enable Disable Particle Button
            if particle == "leader":
                start_controller_button = qt_widgets.QPushButton()
                start_controller_button.setText("Start Controller")
                start_controller_button.setEnabled(False)  # Disable the button
                row_layout.addWidget(start_controller_button)

                self.start_controller_buttons[particle] = start_controller_button
            if particle in self.binded_particles:
                bind_to_leader_button = qt_widgets.QPushButton()
                bind_to_leader_button.setText("Bind to Leader")
                bind_to_leader_button.clicked.connect(lambda _, p=particle: self.bind_to_leader_button_cb(p))
                bind_to_leader_button.setCheckable(True)  # Set the pause button as checkable

                row_layout.addWidget(bind_to_leader_button)

                self.bind_to_leader_buttons[particle] = bind_to_leader_button

            # Create Reset fabric positions button
            if particle in self.binded_particles:
                reset_button = qt_widgets.QPushButton()
                reset_button.setText("Reset Controller Position")
                reset_button.setEnabled(False)  # Disable the button
                row_layout.addWidget(reset_button)
            else:  # Leader particle
                # Send leader particle to the centroid of the particles
                send_to_centroid_button = qt_widgets.QPushButton()
                send_to_centroid_button.setText("Center Leader Frame")
                send_to_centroid_button.clicked.connect(lambda _, p=particle: self.send_to_centroid_cb(p))
                row_layout.addWidget(send_to_centroid_button)

            # Create Make Dynamic Button
            if particle in self.binded_particles:
                make_dynamic_button = qt_widgets.QPushButton()
                make_dynamic_button.setText("Make Dynamic")
                make_dynamic_button.clicked.connect(lambda _, p=particle: self.change_dynamicity_cb(p, True))
                row_layout.addWidget(make_dynamic_button)

            # Create Make Static Button
            if particle in self.binded_particles:
                make_static_button = qt_widgets.QPushButton()
                make_static_button.setText("Make Static")
                make_static_button.clicked.connect(lambda _, p=particle: self.change_dynamicity_cb(p, False))
                row_layout.addWidget(make_static_button)

            # Add row layout to the main layout
            self.layout.addLayout(row_layout)

            if particle == "leader":
                self.odom_publishers[particle] = rospy.Publisher(self.odom_topic_leader, Odometry, queue_size=1)
            else:
                self.odom_publishers[particle] = rospy.Publisher(self.odom_topic_prefix + str(particle), Odometry, queue_size=1)

            if particle in self.binded_particles:
                self.info_binded_pose_publishers[particle] = rospy.Publisher("fake_odom_publisher_gui_info_" + str(particle) + "_target_pose", Marker, queue_size=1)

        self.setLayout(self.layout)

        self.shutdown_timer.timeout.connect(self.check_shutdown)
        self.shutdown_timer.start(1000)  # Timer triggers every 1000 ms (1 second)

    def add_sim_controls(self):
        # Create a horizontal layout for the Compliance controls
        compliance_layout = qt_widgets.QHBoxLayout()

        # Create 'Get Stretching Compliance' button
        get_stretching_compliance_button = qt_widgets.QPushButton()
        get_stretching_compliance_button.setText("Get Stretching Compliance")
        get_stretching_compliance_button.clicked.connect(self.get_stretching_compliance_button_pressed_cb)
        compliance_layout.addWidget(get_stretching_compliance_button)

        # Create text field for displaying Stretching Compliance with label
        stretching_compliance_label = qt_widgets.QLabel("Stretching Compliance:")
        compliance_layout.addWidget(stretching_compliance_label)

        self.stretching_compliance_text_input = qt_widgets.QLineEdit()
        self.stretching_compliance_text_input.setPlaceholderText("Stretching Compliance")
        self.stretching_compliance_text_input.setMinimumWidth(100)
        compliance_layout.addWidget(self.stretching_compliance_text_input)

        # Create 'Set Stretching Compliance' button
        set_stretching_compliance_button = qt_widgets.QPushButton()
        set_stretching_compliance_button.setText("Set Stretching Compliance")
        set_stretching_compliance_button.clicked.connect(self.set_stretching_compliance_button_pressed_cb)
        compliance_layout.addWidget(set_stretching_compliance_button)

        # Separator
        separator = qt_widgets.QFrame()
        separator.setFrameShape(qt_widgets.QFrame.VLine)
        separator.setFrameShadow(qt_widgets.QFrame.Sunken)
        compliance_layout.addWidget(separator)

        # Create 'Get Bending Compliance' button
        get_bending_compliance_button = qt_widgets.QPushButton()
        get_bending_compliance_button.setText("Get Bending Compliance")
        get_bending_compliance_button.clicked.connect(self.get_bending_compliance_button_pressed_cb)
        compliance_layout.addWidget(get_bending_compliance_button)

        # Create text field for displaying Bending Compliance with label
        bending_compliance_label = qt_widgets.QLabel("Bending Compliance:")
        compliance_layout.addWidget(bending_compliance_label)

        self.bending_compliance_text_input = qt_widgets.QLineEdit()
        self.bending_compliance_text_input.setPlaceholderText("Bending Compliance")
        self.bending_compliance_text_input.setMinimumWidth(100)
        compliance_layout.addWidget(self.bending_compliance_text_input)

        # Create 'Set Bending Compliance' button
        set_bending_compliance_button = qt_widgets.QPushButton()
        set_bending_compliance_button.setText("Set Bending Compliance")
        set_bending_compliance_button.clicked.connect(self.set_bending_compliance_button_pressed_cb)
        compliance_layout.addWidget(set_bending_compliance_button)
        
        # Separator
        separator = qt_widgets.QFrame()
        separator.setFrameShape(qt_widgets.QFrame.VLine)
        separator.setFrameShadow(qt_widgets.QFrame.Sunken)
        compliance_layout.addWidget(separator)
        
        # Create 'Enable Collision Handling' button
        enable_collision_button = qt_widgets.QPushButton()
        enable_collision_button.setText("Enable Collision Handling")
        enable_collision_button.clicked.connect(self.enable_collision_button_pressed_cb)
        compliance_layout.addWidget(enable_collision_button)

        # Create 'Disable Collision Handling' button
        disable_collision_button = qt_widgets.QPushButton()
        disable_collision_button.setText("Disable Collision Handling")
        disable_collision_button.clicked.connect(self.disable_collision_button_pressed_cb)
        compliance_layout.addWidget(disable_collision_button)

        # Add the compliance_layout to the main layout at the top
        self.layout.addLayout(compliance_layout)

    def get_stretching_compliance_button_pressed_cb(self):
        try:
            # Call the get_stretching_compliance service
            response = self.get_stretching_compliance_service_client()
            stretching_compliance = response.stretching_compliance
            # Set the value in the text input
            self.stretching_compliance_text_input.setText(str(stretching_compliance))
        except rospy.ServiceException as e:
            rospy.logerr("Service call to get_fabric_stretching_compliance failed: %s", e)

    def get_bending_compliance_button_pressed_cb(self):
        try:
            # Call the get_bending_compliance service
            response = self.get_bending_compliance_service_client()
            bending_compliance = response.bending_compliance
            # Set the value in the text input
            self.bending_compliance_text_input.setText(str(bending_compliance))
        except rospy.ServiceException as e:
            rospy.logerr("Service call to get_fabric_bending_compliance failed: %s", e)

    def set_stretching_compliance_button_pressed_cb(self):
        try:
            # Get the value from the text input
            value_str = self.stretching_compliance_text_input.text()
            # Convert to float (should accept '2e+10' etc.)
            stretching_compliance = float(value_str)
            # Publish to the change topic
            msg = Float32()
            msg.data = stretching_compliance
            self.change_stretching_compliance_publisher.publish(msg)
            rospy.loginfo("Published new stretching compliance: %s", stretching_compliance)
        except ValueError:
            rospy.logerr("Invalid value for stretching compliance: %s", value_str)

    def set_bending_compliance_button_pressed_cb(self):
        try:
            # Get the value from the text input
            value_str = self.bending_compliance_text_input.text()
            # Convert to float (should accept '2e+10' etc.)
            bending_compliance = float(value_str)
            # Publish to the change topic
            msg = Float32()
            msg.data = bending_compliance
            self.change_bending_compliance_publisher.publish(msg)
            rospy.loginfo("Published new Bending Compliance: %s", bending_compliance)
        except ValueError:
            rospy.logerr("Invalid value for Bending Compliance: %s", value_str)

    def format_number(self,num, digits=4):
        # When digits = 4, look at the affect of the function
        # print(format_number(5))         # Output: '5.0'
        # print(format_number(5.0))       # Output: '5.0'
        # print(format_number(5.12345))   # Output: '5.1235'
        # print(format_number(5.1000))    # Output: '5.1'
        # print(format_number(123.456789))# Output: '123.4568'

        rounded_num = round(float(num), digits)  # Ensure num is treated as a float
        # Check if the rounded number is an integer
        if rounded_num.is_integer():
            return f'{int(rounded_num)}.0'
        else:
            return f'{rounded_num:.4f}'.rstrip('0').rstrip('.')

    def get_pose_button_pressed_cb(self, particle):
        """
        Gets the current pose of the particle 
        and fills the text_inputs for pos and ori accordingly
        """

        # Check if the particle pose is set
        if (particle in self.particle_positions) and (particle in self.particle_orientations):
            # Get Current Pose of the particle in world frame
            pos = self.particle_positions[particle] # Point() msg of ROS geometry_msgs
            ori = self.particle_orientations[particle] # Quaternion() msg of ROS geometry_msgs

            # Fill the position text inputs with the current position
            self.text_inputs_pos[particle]['x'].setText(self.format_number(pos.x,digits=3)) 
            self.text_inputs_pos[particle]['y'].setText(self.format_number(pos.y,digits=3)) 
            self.text_inputs_pos[particle]['z'].setText(self.format_number(pos.z,digits=3)) 

            # Convert quaternion orientation to RPY (Roll-pitch-yaw) Euler Angles (degrees)
            rpy = np.rad2deg(tf_trans.euler_from_quaternion([ori.x,ori.y,ori.z,ori.w]))

            # Fill the orientation text  inputs with the current RPY orientation
            self.text_inputs_ori[particle]['x'].setText(self.format_number(rpy[0],digits=1))
            self.text_inputs_ori[particle]['y'].setText(self.format_number(rpy[1],digits=1))
            self.text_inputs_ori[particle]['z'].setText(self.format_number(rpy[2],digits=1))
        else:
            rospy.logwarn(f"Key '{particle}' not found in the particle_positions and particle_orientations dictionaries.")

    def set_position_cb_basic(self, particle):
        pose = Pose()
        pose.position.x = float(self.text_inputs_pos[particle]['x'].text())
        pose.position.y = float(self.text_inputs_pos[particle]['y'].text())
        pose.position.z = float(self.text_inputs_pos[particle]['z'].text())

        # Keep the same orientation
        pose.orientation = self.particle_orientations[particle]

        # if particle == "leader":
        self.particle_positions[particle] = pose.position
        self.particle_orientations[particle] = pose.orientation

        # Prepare Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map" 
        odom.pose.pose = pose

        self.odom_publishers[particle].publish(odom)
    
    def set_position_cb(self, particle):
        if particle not in self.particle_positions:
            return
        
        # Current pose of the particle
        current_position = self.particle_positions[particle]
        current_orientation = self.particle_orientations[particle]  # Assuming quaternion
        
        # Target pose of the particle
        pose = Pose()
        pose.position.x = float(self.text_inputs_pos[particle]['x'].text())
        pose.position.y = float(self.text_inputs_pos[particle]['y'].text())
        pose.position.z = float(self.text_inputs_pos[particle]['z'].text())

        # Keep the same orientation
        pose.orientation = self.particle_orientations[particle]

        target_position = pose.position
        target_orientation = pose.orientation  # Assuming quaternion
        
        # Send the particle to the target pose
        self.send_to_target_poses(current_position, current_orientation,
                                    target_position, target_orientation, particle)
        
    def set_orientation_cb_basic(self,particle):
        pose = Pose()

        # Keep the same position
        pose.position = self.particle_positions[particle]

        # Update the orientation with RPY degree input
        th_x = np.deg2rad(float(self.text_inputs_ori[particle]['x'].text())) # rad
        th_y = np.deg2rad(float(self.text_inputs_ori[particle]['y'].text())) # rad
        th_z = np.deg2rad(float(self.text_inputs_ori[particle]['z'].text())) # rad

        q = tf_trans.quaternion_from_euler(th_x, th_y,th_z)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        # if particle == "leader":
        self.particle_positions[particle] = pose.position
        self.particle_orientations[particle] = pose.orientation

        # Prepare Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map" 
        odom.pose.pose = pose

        self.odom_publishers[particle].publish(odom)

    def set_orientation_cb(self,particle):
        if particle not in self.particle_positions:
            return
        
        # Current pose of the particle
        current_position = self.particle_positions[particle]
        current_orientation = self.particle_orientations[particle]  # Assuming quaternion
        
        pose = Pose()
        pose.position = self.particle_positions[particle] # Keep the same position
        # Update the orientation with RPY degree input
        th_x = np.deg2rad(float(self.text_inputs_ori[particle]['x'].text())) # rad
        th_y = np.deg2rad(float(self.text_inputs_ori[particle]['y'].text())) # rad
        th_z = np.deg2rad(float(self.text_inputs_ori[particle]['z'].text())) # rad

        q = tf_trans.quaternion_from_euler(th_x, th_y,th_z)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        target_position = pose.position
        target_orientation = pose.orientation  # Assuming quaternion

        # Send the particle to the target pose
        self.send_to_target_poses(current_position, current_orientation,
                                    target_position, target_orientation, particle)

    def enable_collision_button_pressed_cb(self):
        try:
            # Call the enable_collision_handling service with is_enable=True
            response = self.enable_collision_handling_service_client(is_enable=True)
            if response.success:
                rospy.loginfo("Collision handling enabled successfully.")
            else:
                rospy.logerr("Failed to enable collision handling.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call to enable_collision_handling failed: %s", e)

    def disable_collision_button_pressed_cb(self):
        try:
            # Call the enable_collision_handling service with is_enable=False
            response = self.enable_collision_handling_service_client(is_enable=False)
            if response.success:
                rospy.loginfo("Collision handling disabled successfully.")
            else:
                rospy.logerr("Failed to disable collision handling.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call to enable_collision_handling failed: %s", e)
    # ----------------------------------------------------------------------------------    
    def send_to_target_poses(self, current_position, current_orientation,
                                    target_position, target_orientation, particle,
                                    v_max=3.0, a_max=1.0, alpha_max=6.0, omega_max=1.0,
                                    rate=100, speedup=1.0):
        # Gradually change the position and orientation of the particles 
        # to the target pose (self.planned_path_current_target_poses_of_particles[particle]) 
        # based on a trapezoidal velocity profile described with
        # given self.a_max, self.v_max, self.alpha_max, self.omega_max
        
        rate = rospy.Rate(rate)  # Control frequency

        # Initialize variables
        max_time = 0
        acc_time = 0 # desired acceleration time
        velocity_profiles = {}
        
        # Compute 3D distance and orientation difference as angle (based on axis-angle representation)
        segment_distance, segment_direction = self.compute_distance(current_position, target_position)  # 3D distance
        segment_rotation, segment_rot_axis = self.compute_orientation_difference_axis_angle(current_orientation, target_orientation)

        # Compute trapezoidal profile for position and orientation
        t_pos, t_pos_acc = self.compute_min_time_needed_with_trapezoidal_profile(segment_distance, a_max*speedup, v_max*speedup)
        t_ori, t_ori_acc = self.compute_min_time_needed_with_trapezoidal_profile(segment_rotation, alpha_max*speedup, omega_max*speedup)

        # Store velocity profile components
        velocity_profiles[particle] = {
            't_tot': 0.0,  # Total time (placeholder)
            't_a_p': 0.0,  # position acceleration duration (placeholder), assumed same as deceleration duration
            't_a_o': 0.0,  # orientation acceleration duration (placeholder), assumed same as deceleration duration
            'a_p': 0.0,  # position acceleration (placeholder)
            'a_o': 0.0,  # orientation acceleration (placeholder)
            'd_p': segment_distance,  # 3D distance to the target
            'd_o': segment_rotation,  # Orientation difference (angle in radians)
            'dir_p': segment_direction,  # Unit Vector to target position (zeros vector if d_p=0)
            'dir_o': segment_rot_axis,  # Unit Axis orientation vector (unit vector in z direction if d_o=0)
            'start_p': current_position,  # Starting position (Point)
            'start_o': current_orientation,  # Starting orientation (Quaternion)
            'target_p': target_position,  # Target position (Point)
            'target_o': target_orientation,  # Target orientation (Quaternion)
        }

        # Find the maximum time needed across all particles and corresponding acceleration times
        # max_time = max(max_time, t_pos, t_ori)
        if t_pos > max_time:
            max_time = t_pos
            acc_time = t_pos_acc
        if t_ori > max_time:
            max_time = t_ori
            acc_time = t_ori_acc
            
        # print(velocity_profiles[particle])

        # Re-iterate to update the total time needed for each particle
        # and to update the velocity profiles
        for particle, profile in velocity_profiles.items():
            velocity_profiles[particle]['t_tot'] = max_time
            
            # Find the acceleration time for position and orientation based on the max time and the desired acceleration time
            if profile['d_p'] > 0.0:
                a_p, t_a_p = self.compute_fixed_time_trapezoidal_profile_params(max_time, acc_time, profile['d_p'], a_max*speedup, v_max*speedup)
            else:
                a_p = 0.0
                t_a_p = 0.0
            
            if profile['d_o'] > 0.0:
                a_o, t_a_o = self.compute_fixed_time_trapezoidal_profile_params(max_time, acc_time, profile['d_o'], alpha_max*speedup, omega_max*speedup)
            else:
                a_o = 0.0
                t_a_o = 0.0
            
            # Update the velocity profiles
            velocity_profiles[particle]['t_a_p'] = t_a_p
            velocity_profiles[particle]['t_a_o'] = t_a_o
            velocity_profiles[particle]['a_p'] = a_p
            velocity_profiles[particle]['a_o'] = a_o
        
        # Main loop to update particles simultaneously
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            elapsed_time = current_time - start_time

            for particle, profile in velocity_profiles.items():
                if particle not in self.particle_positions:
                    continue

                # Update position and orientation based on trapezoidal profile
                new_position = self.compute_position_from_trapezoidal_profile(profile, elapsed_time)
                new_orientation = self.compute_orientation_from_trapezoidal_profile(profile, elapsed_time)

                # Compute twist (linear and angular velocity) for the particle
                linear_velocity = self.compute_linear_velocity_from_profile(profile, elapsed_time)
                angular_velocity = self.compute_angular_velocity_from_profile(profile, elapsed_time)
                
                if particle == "leader":
                    self.particle_positions[particle] = new_position
                    self.particle_orientations[particle] = new_orientation

                # Prepare odometry message
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()
                odom.header.frame_id = "map"
                
                odom.pose.pose.position = new_position
                odom.pose.pose.orientation = new_orientation
                
                odom.twist.twist.linear.x = linear_velocity[0]
                odom.twist.twist.linear.y = linear_velocity[1]
                odom.twist.twist.linear.z = linear_velocity[2]
                odom.twist.twist.angular.x = angular_velocity[0]
                odom.twist.twist.angular.y = angular_velocity[1]
                odom.twist.twist.angular.z = angular_velocity[2]

                # Publish odometry and twist
                self.odom_publishers[particle].publish(odom)

            if elapsed_time > max_time:
                break  # Stop if all particles have completed their movements
        
            rate.sleep()  # Maintain the control loop rate
    
    def compute_distance(self, current_position, target_position):
        # Compute 3D distance between two points
        vec = np.array([target_position.x - current_position.x, target_position.y - current_position.y, target_position.z - current_position.z])
        dist = np.linalg.norm(vec)
        
        if dist > 0.0:
            return dist, vec
        else:
            return 0.0, np.zeros_like(vec)
    
    def compute_orientation_difference_axis_angle(self, current_orientation, target_orientation):
        current_orientation = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        target_orientation = [target_orientation.x, target_orientation.y, target_orientation.z, target_orientation.w]
        
        # Relative rotation quaternion from current to target
        quaternion_error = tf_trans.quaternion_multiply(target_orientation, tf_trans.quaternion_inverse(current_orientation))

        # Normalize the quaternion to avoid numerical issues
        quaternion_error = self.normalize_quaternion(quaternion_error)

        # AXIS-ANGLE ORIENTATION ERROR/DIFFERENCE DEFINITION
        # Convert quaternion difference to rotation vector (axis-angle representation)
        rotation_vector = self.quaternion_to_rotation_vec(quaternion_error)
        
        angle = np.linalg.norm(rotation_vector)
        
        # Wrap the angle to [-pi, pi]
        angle = np.arctan2(np.sin(angle), np.cos(angle))
        
        if angle > 0.0:
            axis = rotation_vector / angle
        else:
            axis = np.array([0.0, 0.0, 1.0])
            
        return angle, axis

    def quaternion_to_rotation_vec(self, quaternion):
        """
        Converts a quaternion to axis-angle representation.
        Minimal representation of the orientation error. (3,) vector.
        Arguments:
            quaternion: The quaternion to convert as a numpy array in the form [x, y, z, w].
        Returns:
            rotation_vector: Minimal axis-angle representation of the orientation error. (3,) vector.
            Norm if the axis_angle is the angle of rotation.
        """
        angle = 2 * np.arccos(quaternion[3])
        
        # Wrap the angle to [-pi, pi]
        angle = np.arctan2(np.sin(angle), np.cos(angle))

        # Handling small angles with an approximation
        small_angle_threshold = 1e-6
        if np.abs(angle) < small_angle_threshold:
            # Use small angle approximation
            rotation_vector = np.array([0.,0.,0.])
            
            # # Also calculate the representation Jacobian inverse
            # J_inv = np.eye((3,3))
        else:
            # Regular calculation for larger angles
            axis = quaternion[:3] / np.sin(angle/2.0)
            # Normalize the axis
            axis = axis / np.linalg.norm(axis)
            rotation_vector = angle * axis 
            
            # # Also calculate the representation Jacobian inverse
            # k_hat = self.hat(axis)
            # cot = 1.0 / np.tan(angle/2.0)
            # J = -angle/2.0 * (k_hat + cot*(k_hat @ k_hat)) + np.outer(axis, axis)
            # J_inv = np.linalg.inv(J)
            
        return rotation_vector # , J_inv

    def normalize_quaternion(self, quaternion):
        norm = np.linalg.norm(quaternion)
        if norm == 0:
            raise ValueError("Cannot normalize a quaternion with zero norm.")
        return quaternion / norm

    def compute_min_time_needed_with_trapezoidal_profile(self, delta, a_max, v_max):
        # Check if it's a full trapezoidal profile or triangular profile
        if (v_max ** 2) / a_max < delta:  # Full trapezoidal profile
            t_acc = v_max / a_max
            t_const = (delta - (v_max ** 2) / a_max) / v_max
            t_total = 2 * t_acc + t_const
        else:  # Triangular profile (cannot reach max velocity)
            t_acc = np.sqrt(delta / a_max)
            t_total = 2 * t_acc
            t_const = 0

        # Return total needed time and the acceleration time
        return t_total, t_acc

    def compute_fixed_time_trapezoidal_profile_params(self, t_total, t_acc_d, delta, a_max, v_max):
        """ Calculate the trapezoidal profile parameters (acc and t_acc) for a known 
            time duration and distance (delta) of the profile, given the max acceleration and velocity.
            Assuming the acceleration and deceleration times are the same.
            Assuming the acceleration values are fixed with values either acc, -acc, or 0,
            which is needed for Trapezoidal velocity profile.
            
        """
        acc = delta / (t_acc_d*(t_total - t_acc_d))
        
        EPSILON = 1e-9  # Set a small threshold for precision errors

        if acc <= a_max + EPSILON and acc * t_acc_d <= v_max + EPSILON:
            return acc, t_acc_d
        else:
            # Find the closest valid acceleration and acceleration time
            t_acc = self.find_closest_t_acc(x0=t_acc_d, 
                                            a=-t_total, 
                                            b=-delta/a_max, 
                                            c=t_total - delta/v_max, 
                                            d=t_total/2)
            acc = delta / (t_acc*(t_total - t_acc)) 
            return acc, t_acc
            
    def find_closest_t_acc(self, x0, a, b, c, d):
        # Solve the quadratic inequality x^2 + ax <= b
        # Find the roots of x^2 + ax - b = 0
        discriminant = a**2 + 4*b
        
        if discriminant < 0:
            rospy.logwarn(f"Discriminant = {discriminant} < 0. No real solution for the quadratic inequality.")

        root1 = (-a + np.sqrt(discriminant)) / 2
        root2 = (-a - np.sqrt(discriminant)) / 2

        # The solution for x^2 + ax <= b is between the roots root1 and root2
        lower_bound = min(root1, root2)
        upper_bound = max(root1, root2)

        # Now, intersect with other constraints: x <= c, 0 < x <= d
        feasible_lower_bound = max(0, lower_bound)
        feasible_upper_bound = min(upper_bound, c, d)

        # Check if the feasible region is valid
        if feasible_lower_bound >= feasible_upper_bound:
            raise ValueError("No feasible solution due to constraints.")

        # Now find the value of x in the feasible region that is closest to x0
        if x0 < feasible_lower_bound:
            return feasible_lower_bound
        elif x0 > feasible_upper_bound:
            return feasible_upper_bound
        else:
            return x0

    def compute_linear_velocity_from_profile(self, profile, elapsed_time):
        if elapsed_time >= profile['t_tot']:
            return np.zeros(3)  # Stop if time exceeds
        v_norm =  self.v_from_trap_vel_profile(t=elapsed_time, a=profile['a_p'], t_acc=profile['t_a_p'], t_total=profile['t_tot'])
        v_dir = profile['dir_p']
        return v_norm * v_dir

    def compute_angular_velocity_from_profile(self, profile, elapsed_time):
        if elapsed_time >= profile['t_tot']:
            return np.zeros(3)  # Stop if time exceeds
        w_norm = self.v_from_trap_vel_profile(t=elapsed_time, a=profile['a_o'], t_acc=profile['t_a_o'], t_total=profile['t_tot'])
        w_dir = profile['dir_o']
        return w_norm * w_dir
    
    def compute_position_from_trapezoidal_profile(self, profile, elapsed_time):
        if elapsed_time >= profile['t_tot']:
            return profile['target_p']

        start_position = np.array([profile['start_p'].x, profile['start_p'].y, profile['start_p'].z])

        # Compute the distance moved based on the trapezoidal velocity profile
        p = self.p_from_trap_vel_profile(t=elapsed_time, a=profile['a_p'], t_acc=profile['t_a_p'], t_total=profile['t_tot'], delta=profile['d_p'])        
        if profile['d_p'] > 0.0:
            new_position = start_position +  profile['dir_p']*(p/profile['d_p'])
        else:
            new_position = start_position

        return Point(x=new_position[0], y=new_position[1], z=new_position[2])

    def compute_orientation_from_trapezoidal_profile(self, profile, elapsed_time):
        if elapsed_time >= profile['t_tot']:
            return profile['target_o']

        # Compute rotated angle on the trapezoidal velocity profile
        p = self.p_from_trap_vel_profile(elapsed_time, a=profile['a_o'], t_acc=profile['t_a_o'], t_total=profile['t_tot'], delta=profile['d_o'])
        
        # Compute the new orientation
        new_orientation = self.apply_axis_angle_rotation(profile['start_o'], profile['dir_o'], p)
        
        return Quaternion(x=new_orientation[0], y=new_orientation[1], z=new_orientation[2], w=new_orientation[3])

    def p_from_trap_vel_profile(self, t, a, t_acc, t_total, delta):
        if t >= t_total:
            return delta
        
        if t < t_acc:  # Acceleration phase
            return 0.5 * a * t**2
        elif t < t_total - t_acc:  # Constant velocity phase
            return (0.5 * a*t_acc**2) + (a*t_acc) * (t - t_acc)
        else:  # Deceleration phase
            return delta - 0.5*a*(t_total - t)**2

    def v_from_trap_vel_profile(self, t, a, t_acc, t_total):
        # t: current time
        # a: acceleration
        # t_acc: time to reach max velocity
        # t_total: total time of the profile    
        
        if t >= t_total:
            return 0.0
        
        if t < t_acc:  # Acceleration phase
            return a*t
        elif t < t_total - t_acc:  # Constant velocity phase
            return a*t_acc
        else:  # Deceleration phase
            return a*t_acc - a*(t - (t_total-t_acc))    
    
    def apply_axis_angle_rotation(self, start_orientation, axis, angle):
        # Convert the start orientation to a numpy array
        start_quat = [start_orientation.x, start_orientation.y, start_orientation.z, start_orientation.w]

        delta_orientation = tf_trans.quaternion_about_axis(angle, axis)
        
        # Apply the rotation to the start orientation
        new_orientation = tf_trans.quaternion_multiply(delta_orientation, start_quat)
        return new_orientation
    # ----------------------------------------------------------------------------------
    

    def reset_position_cb(self, particle):
        # Call reset positions service
        service_name = '/fabric_velocity_controller/reset_positions' 
        rospy.wait_for_service(service_name, timeout=2.0)
        try:
            reset_positions_service = rospy.ServiceProxy(service_name, ResetParticlePosition)
            # Create a request to the service
            request = ResetParticlePositionRequest()
            request.particle_id = particle

            reset_positions_service(request)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def bind_to_leader_button_cb(self, particle):
        if self.bind_to_leader_buttons[particle].isChecked():
            # Button is currently pressed, no need to set it to False
            print(f"Button for binding particle {particle} to leader is now pressed.")

            # Store binded particle's current pose relative to the leader frame
            if (particle in self.particle_positions) and ("leader" in self.particle_positions):                
                # Current Pose of the leader in world frame
                pos_leader = self.particle_positions["leader"] # Point() msg of ROS geometry_msgs
                ori_leader = self.particle_orientations["leader"] # Quaternion() msg of ROS geometry_msgs

                # Current Pose of the binded particle in world frame
                pos_binded_world = self.particle_positions[particle] # Point() msg of ROS geometry_msgs
                ori_binded_world = self.particle_orientations[particle] # Quaternion() msg of ROS geometry_msgs

                # Convert quaternions to rotation matrices
                rotation_matrix_leader = tf_trans.quaternion_matrix([ori_leader.x, ori_leader.y, ori_leader.z, ori_leader.w])

                # Calculate the relative position of the binded particle in the leader's frame
                relative_pos = np.dot(rotation_matrix_leader.T, np.array([pos_binded_world.x - pos_leader.x, pos_binded_world.y - pos_leader.y, pos_binded_world.z - pos_leader.z, 1]))[:3]

                # Calculate the relative orientation of the binded particle in the leader's frame
                inv_ori_leader = tf_trans.quaternion_inverse([ori_leader.x, ori_leader.y, ori_leader.z, ori_leader.w])
                relative_ori = tf_trans.quaternion_multiply(inv_ori_leader, [ori_binded_world.x, ori_binded_world.y, ori_binded_world.z, ori_binded_world.w])

                # Store the relative position and orientation
                pose = Pose()
                pose.position = Point(*relative_pos)
                pose.orientation = Quaternion(*relative_ori)
                self.binded_relative_poses[particle] = pose # stores Pose() msg of ROS geometry_msgs (Pose.position and Pose.orientation)

                # Make the manual control button unpressed
                self.buttons_manual[particle].setChecked(False)
            else:
                rospy.logwarn("Initial pose values of the object particles or leader is not yet set")
                self.bind_to_leader_buttons[particle].setChecked(False)

        else:
            # Button is currently not pressed, no need to set it to True
            print(f"Button for binding particle {particle} to leader is now NOT pressed.")   

            # Unbind particle from leader by deleting the relative pose from the dictionary
            if particle in self.binded_relative_poses:
                del self.binded_relative_poses[particle]
            else:
                rospy.logerr(f"Key '{particle}' not found in the binded_relative_poses dictionary.")

    def send_to_centroid_cb(self,particle_to_send):
        # Calculate the centroid of the other particles
        centroid_pos = np.zeros(3)
        n_particle = 0 # number of particles to calculate the centroid
        for particle in ["leader"] + self.particles:
            if particle != particle_to_send:
                # Check if the particle pose is set
                if (particle in self.particle_positions) and (particle in self.particle_orientations):
                    # Get Current Pose of the particle in world frame
                    pos = self.particle_positions[particle] # Point() msg of ROS geometry_msgs
                    # ori = self.particle_orientations[particle] # Quaternion() msg of ROS geometry_msgs

                    centroid_pos = centroid_pos + np.array([pos.x,pos.y,pos.z])
                    n_particle = n_particle + 1
                else:
                    rospy.logwarn(f"Key '{particle}' not found in the particle_positions and particle_orientations dictionaries.")

        # Send to centroid
        if n_particle > 0:
            centroid_pos = centroid_pos/n_particle # Take the mean

            pose = Pose()
            pose.position.x = centroid_pos[0]
            pose.position.y = centroid_pos[1]
            pose.position.z = centroid_pos[2]

            # Keep the same orientation
            pose.orientation = self.particle_orientations[particle_to_send]

            if particle_to_send == "leader":
                self.particle_positions[particle_to_send] = pose.position
                self.particle_orientations[particle_to_send] = pose.orientation

            # Prepare Odometry message
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map" 
            odom.pose.pose = pose

            self.odom_publishers[particle_to_send].publish(odom)
        else:
            rospy.logwarn("There is no particle to calculate the centroid.")

    def odom_pub_timer_callback(self,event):
        # Reset spacenav_twist to zero if it's been long time since the last arrived
        self.check_spacenav_twist_wait_timeout()

        # Handle manual control of each particle and the leader
        for particle in ["leader"] + self.particles:
            # Do not proceed with the "non-leader" particles until the initial values have been set
            if (particle != "leader") and not((particle in self.particle_positions) and ("leader" in self.particle_positions)):
                continue

            if self.buttons_manual[particle].isChecked():
                dt = self.get_timestep(particle)   
                # dt = 0.01
                # dt = 1. / self.pub_rate_odom

                # simple time step integration using Twist data
                pose = Pose()
                pose.position.x = self.particle_positions[particle].x + dt*self.spacenav_twist.linear.x
                pose.position.y = self.particle_positions[particle].y + dt*self.spacenav_twist.linear.y
                pose.position.z = self.particle_positions[particle].z + dt*self.spacenav_twist.linear.z

                # # --------------------------------------------------------------
                # # To update the orientation with the twist message
                # Calculate the magnitude of the angular velocity vector
                omega_magnitude = math.sqrt(self.spacenav_twist.angular.x**2 + 
                                            self.spacenav_twist.angular.y**2 + 
                                            self.spacenav_twist.angular.z**2)
                
                # print("omega_magnitude: " + str(omega_magnitude))

                pose.orientation = self.particle_orientations[particle]

                if omega_magnitude > 1e-9:  # Avoid division by zero
                    # Create the delta quaternion based on world frame twist
                    delta_quat = tf_trans.quaternion_about_axis(omega_magnitude * dt, [
                        self.spacenav_twist.angular.x / omega_magnitude,
                        self.spacenav_twist.angular.y / omega_magnitude,
                        self.spacenav_twist.angular.z / omega_magnitude
                    ])
                    
                    # Update the pose's orientation by multiplying delta quaternion with current orientation 
                    # Note the order here. This applies the world frame rotation directly.
                    current_quaternion = (
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w
                    )
                    
                    new_quaternion = tf_trans.quaternion_multiply(delta_quat, current_quaternion)
                    
                    pose.orientation.x = new_quaternion[0]
                    pose.orientation.y = new_quaternion[1]
                    pose.orientation.z = new_quaternion[2]
                    pose.orientation.w = new_quaternion[3]
                # # --------------------------------------------------------------

                if particle == "leader":
                    self.particle_positions[particle] = pose.position
                    self.particle_orientations[particle] = pose.orientation

                # Prepare Odometry message
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()
                odom.header.frame_id = "map" 
                odom.pose.pose = pose
                odom.twist.twist = self.spacenav_twist
                self.odom_publishers[particle].publish(odom)

        # Handle the control of binded particles
        for particle in self.binded_particles:
            # Do not proceed with the "binded" particles until the initial values have been set
            if not (particle in self.particle_positions) and ("leader" in self.particle_positions):
                continue

            if self.bind_to_leader_buttons[particle].isChecked():
                if particle in self.binded_relative_poses:
                    # calculate the pose of the binded particle in world using the relative pose to the leader
                    
                    # Current Pose of the leader in world frame
                    pos_leader = self.particle_positions["leader"] # Point() msg of ROS geometry_msgs
                    ori_leader = self.particle_orientations["leader"] # Quaternion() msg of ROS geometry_msgs

                    # Relative Pose of the binded particle in leader's frame
                    pos_binded_in_leader = self.binded_relative_poses[particle].position # Point() msg of ROS geometry_msgs
                    ori_binded_in_leader = self.binded_relative_poses[particle].orientation # Quaternion() msg of ROS geometry_msgs

                    # Convert quaternions to rotation matrices
                    rotation_matrix_leader = tf_trans.quaternion_matrix([ori_leader.x, ori_leader.y, ori_leader.z, ori_leader.w])

                    # Calculate the position of the binded particle in the world frame
                    pos_binded_in_world = np.dot(rotation_matrix_leader, np.array([pos_binded_in_leader.x, pos_binded_in_leader.y, pos_binded_in_leader.z, 1]))[:3] + np.array([pos_leader.x, pos_leader.y, pos_leader.z])

                    # Calculate the orientation of the binded particle in the world frame
                    ori_binded_in_world = tf_trans.quaternion_multiply([ori_leader.x, ori_leader.y, ori_leader.z, ori_leader.w], [ori_binded_in_leader.x, ori_binded_in_leader.y, ori_binded_in_leader.z, ori_binded_in_leader.w])

                    # Now publish the binded particle's pose as an odom msg
                    pose = Pose()
                    pose.position = Point(*pos_binded_in_world) # pos_binded_world
                    pose.orientation = Quaternion(*ori_binded_in_world) # ori_binded_world

                    # Prepare Odometry message
                    odom = Odometry()
                    odom.header.stamp = rospy.Time.now()
                    odom.header.frame_id = "map" 
                    odom.pose.pose = pose
                    # odom.twist.twist.linear = TODO
                    # odom.twist.twist.angular =  TODO
                    self.odom_publishers[particle].publish(odom)

                    # Also publish an arrow to distinguish the binded particles
                    self.publish_arrow_marker(pos_leader,pose,particle)
                else:
                    rospy.logwarn(f"Key '{particle}' not found in the binded_relative_poses dictionary.")

    def change_dynamicity_cb(self,particle, is_dynamic):
        msg = ChangeParticleDynamicity()
        msg.particle_id = particle
        msg.is_dynamic = is_dynamic
        self.pub_change_dynamicity.publish(msg)

    def publish_arrow_marker(self, leader_position, target_pose, particle):
        # Create a marker for the arrow
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set the scale of the arrow
        marker.scale.x = 0.015  # Shaft diameter
        marker.scale.y = 0.05  # Head diameter
        marker.scale.z = 0.3  # Head length
        
        # Set the color
        marker.color.a = 0.3
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.7
        
        # Set the pose (position and orientation) for the marker
        marker.pose.orientation.w = 1.0  # Orientation (quaternion)
        
        # Set the start and end points of the arrow
        marker.points = []
        start_point = leader_position  # Should be a Point message
        end_point = target_pose.position  # Assuming target_pose is a Pose message
        marker.points.append(start_point)
        marker.points.append(end_point)
        
        # Publish the marker
        self.info_binded_pose_publishers[particle].publish(marker)


    def spacenav_twist_callback(self, twist):
        # self.spacenav_twist.linear.x = twist.linear.x # because we are in YZ plane
        # self.spacenav_twist.linear.y = twist.linear.y
        # self.spacenav_twist.linear.z = twist.linear.z
        # self.spacenav_twist.angular.x = twist.angular.x
        # self.spacenav_twist.angular.y = twist.angular.y  # twist.angular.y because we are in YZ plane
        # self.spacenav_twist.angular.z = twist.angular.z  # twist.angular.z because we are in YZ plane
        
        self.spacenav_twist = twist

        self.last_spacenav_twist_time = rospy.Time.now()

    def check_spacenav_twist_wait_timeout(self):
        if (rospy.Time.now() - self.last_spacenav_twist_time) > self.spacenav_twist_wait_timeout:
            # Reset spacenav_twist to zero after timeout
            self.spacenav_twist = Twist()

            rospy.loginfo_throttle(2.0,"spacenav_twist is zeroed because it's been long time since the last msg arrived..")

    def state_array_callback(self, states_msg):
        # Positions
        for particle in self.particles:
            # if not self.buttons_manual[particle].isChecked(): # Needed To prevent conflicts in pose updates when applying manual control
                self.particle_positions[particle] = states_msg.states[particle].pose.position
                self.particle_orientations[particle] = states_msg.states[particle].pose.orientation
                self.particle_twists[particle] = states_msg.states[particle].twist


    def initialize_leader_position(self):
        # Initialize to zero vector
        position = Point()
        position.x = 0
        position.y = 0
        position.z = 0

        # Initialize to identity rotation
        quaternion = Quaternion()
        quaternion.x = 0
        quaternion.y = 0
        quaternion.z = 0
        quaternion.w = 1

        self.particle_positions["leader"] = position
        self.particle_orientations["leader"] = quaternion

    def manual_control_button_pressed_cb(self, particle):
        if self.buttons_manual[particle].isChecked():
            # Button is currently pressed, no need to set it to False
            print(f"Button for manual control of particle {particle} is now pressed.")

            # Do not proceed with the "non-leader" particles until the initial values have been set
            if (particle != "leader") and not ((particle in self.particle_positions) and ("leader" in self.particle_positions)):
                # Make the manual control button unpressed
                self.buttons_manual[particle].setChecked(False)

                rospy.logwarn("Initial pose values of the object particles or the leader is not yet set")
            else:
                if particle in self.binded_particles:
                    # Unbind particle from leader by deleting the relative pose from the dictionary
                    if particle in self.binded_relative_poses:
                        del self.binded_relative_poses[particle]
                    # else:
                    #     rospy.loginfo(f"Key '{particle}' not found in the binded_relative_poses dictionary.")

                    # Make the bind to leader button unpressed
                    self.bind_to_leader_buttons[particle].setChecked(False)

        else:
            # Button is currently not pressed, no need to set it to True
            print(f"Button for manual control of particle {particle} is now NOT pressed.")    

    def get_timestep(self, integrator_name):
        current_time = rospy.Time.now().to_time()
        if integrator_name in self.last_timestep_requests:
            dt = current_time - self.last_timestep_requests[integrator_name]
            self.last_timestep_requests[integrator_name] = current_time
            if dt > MAX_TIMESTEP:
                dt = 0.0
            return dt
        else:
            self.last_timestep_requests[integrator_name] = current_time
            return 0.0

    def check_shutdown(self):
        if rospy.is_shutdown():
            qt_widgets.QApplication.quit()

    

if __name__ == "__main__":
    rospy.init_node('test_gui_node', anonymous=False)

    app = qt_widgets.QApplication(sys.argv)

    gui = TestGUI()
    gui.show()

    sys.exit(app.exec_())

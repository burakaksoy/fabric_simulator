#!/usr/bin/env python3

import sys
import os
import rospy

import numpy as np
import time
import threading

import PyQt5.QtWidgets as qt_widgets
import PyQt5.QtCore as qt_core
from PyQt5.QtCore import Qt

from geometry_msgs.msg import Twist, Point, Quaternion, Pose, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from fabric_simulator.msg import SegmentStateArray
from fabric_simulator.msg import ChangeParticleDynamicity
from fabric_simulator.msg import AttachExternalOdomFrameRequest
from fabric_simulator.msg import FixNearestFabricParticleRequest

from fabric_simulator.srv import GetFabricStretchingCompliance
from fabric_simulator.srv import GetFabricBendingCompliance

from fabric_simulator.srv import EnableCollisionHandling

from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Empty, EmptyResponse

from std_msgs.msg import Float32

import math
import tf.transformations as tf_trans

from voice_recognition import VoiceCommandThread
import pyttsx3 # For text-to-speech - OFFLINE
from gtts import gTTS # For text-to-speech - Google's ONLINE

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

def run_in_thread(callback):
    def wrapper(*args, **kwargs):
        thread = threading.Thread(target=callback, args=args, kwargs=kwargs)
        thread.daemon = True
        thread.start()
    return wrapper

class TestGUI(qt_widgets.QWidget):
    def __init__(self):
        super(TestGUI, self).__init__()
        
        self.lock = threading.Lock() # Lock for thread safety
        
        self.shutdown_timer = qt_core.QTimer()

        self.pub_rate_odom = rospy.get_param("~pub_rate_odom", 100)

        self.initial_values_set = False  # Initialization state variable

        self.custom_static_particles = []  # Particles that are controllable attached to the fabric 
        self.binded_particles = []  # Particles that are uncontrollable and binded to other sensors such as human hand tracking

        self.odom_topic_prefix = None
        self.cmd_vel_topic_prefix = None
        self.binded_particles_odom_topic_prefix = None
        
        mode = "simulation_test"  # Default mode. Use this mode for default fabric simulation test
        # mode = "composite_sheet_application_test"  # Use this mode for testing the composite sheet application
        
        self.mode = rospy.get_param("~mode", mode)
        
        simulator_node_name = "" # Default value
        if self.mode == "simulation_test":
            simulator_node_name = "/fabric_simulator_node"
        elif self.mode == "composite_sheet_application_test":
            simulator_node_name = "/fabric_simulator"
            # simulator_node_name = ""
        
        while not self.custom_static_particles:
            try:
                self.custom_static_particles = rospy.get_param(simulator_node_name + "/custom_static_particles")
                self.odom_topic_prefix = rospy.get_param(simulator_node_name + "/custom_static_particles_odom_topic_prefix")
                self.cmd_vel_topic_prefix = rospy.get_param(simulator_node_name + "/custom_static_particles_cmd_vel_topic_prefix")

            except:
                rospy.logwarn("No custom static particles obtained from ROS parameters. That part of the GUI will be empty.")
                time.sleep(0.5)
                
        while not self.binded_particles:
            try:
                binded_particles_default = ["hand_1", "hand_2"]
                binded_particles_odom_topic_prefix_default = "/odom_"
                
                self.binded_particles = rospy.get_param(simulator_node_name + "/binded_particles", binded_particles_default)
                self.binded_particles_odom_topic_prefix = rospy.get_param(simulator_node_name + "/binded_particles_odom_topic_prefix", binded_particles_odom_topic_prefix_default)
                
            except:
                rospy.logwarn("No binded particles obtained from ROS parameters. That part of the GUI will be empty.")
                time.sleep(0.5)

        # Remove duplicates from the list of custom static particles and binded particles
        self.custom_static_particles = list(set(self.custom_static_particles)) 
        self.binded_particles = list(set(self.binded_particles)) 
        
        # Sorting the list of particles in ascending order
        self.custom_static_particles.sort()
        self.binded_particles.sort()
        
        self.combined_particles = self.binded_particles + self.custom_static_particles

        self.odom_publishers = {}

        self.particle_positions = {}
        self.particle_orientations = {}
        self.particle_twists = {}

        self.initialize_binded_particle_positions()

        # Service clients and publishers for modulus values
        
        # Initialize the modulus services and publishers
        if self.mode == "simulation_test":
            self.initialize_modulus_services_and_publishers(simulator_node_name)
        elif self.mode == "composite_sheet_application_test":
            self.initialize_modulus_services_and_publishers(simulator_node_name)
            # self.initialize_modulus_services_and_publishers(simulator_node_name="/fabric_simulator")
            self.initialize_controller_services_and_publishers(controller_node_name="/composite_layup_velocity_controller")

        # Flag to check if orientation control is enabled, GUI is created accordingly
        self.is_orientation_control_enabled = False
        
        self.createUI()

        # ---------------------------------------------------------
        ## Voice commands related initializations
        self.voice_offline_mode = False  # Set it to True for offline mode
        
        if self.voice_offline_mode:
            # Initialize the text-to-speech engine for getting feedback from the voice commands
            self.tts_engine = pyttsx3.init()
            # Optionally set voice properties like rate or voice ID
            self.tts_engine.setProperty('rate', 100)
            # self.tts_engine.setProperty('voice', 'english-us')
        
        # Create a dictionary for recognized commands -> GUI actions
        # The key is a spoken phrase, the value is a callable (could be a button click or a method)
        self.voice_command_actions = {
            "get stretching compliance": self.get_stretching_compliance_button_click,
            "get bending compliance": self.get_bending_compliance_button_click,
            
            "enable collision handling": self.enable_collision_button_click,
            
            "disable collision handling": self.disable_collision_button_click,
            
            "reset target poses": self.reset_target_poses_button_click,
            
            "enable nominal control": self.enable_nominal_control_button_click,
            
            "disable nominal control": self.disable_nominal_control_button_click,
            
            "enable obstacle avoidance": self.enable_obstacle_avoidance_button_click,
            "enable collision avoidance": self.enable_obstacle_avoidance_button_click,
            
            "disable obstacle avoidance": self.disable_obstacle_avoidance_button_click,
            "disable collision avoidance": self.disable_obstacle_avoidance_button_click,
            
            "enable stress avoidance": self.enable_stress_avoidance_button_click,
            
            "disable stress avoidance": self.disable_stress_avoidance_button_click,
            
            "pause controller": self.pause_controller_button_click,
            
            "resume controller": self.resume_controller_button_click,
            
            "enable controller": self.enable_controller_button_click,
            
            "disable controller": self.disable_controller_button_click,
            
            "attach left hand": self.attach_left_hand_button_click,
            "attached left hand": self.attach_left_hand_button_click,
            
            "detach left hand": self.detach_left_hand_button_click,
            "detached left hand": self.detach_left_hand_button_click,
            
            "attach right hand": self.attach_right_hand_button_click,
            "attached right hand": self.attach_right_hand_button_click,
            
            "detach right hand": self.detach_right_hand_button_click,
            "detached right hand": self.detach_right_hand_button_click,
            
            "stick left": self.stick_left_button_click,
            
            "unstick left": self.unstick_left_button_click,
            
            "stick right": self.stick_right_button_click,
            "unstick right": self.unstick_right_button_click,
            
            "get left corner position": self.get_left_corner_pose_from_particle,
            
            "get right corner position": self.get_right_corner_pose_from_particle,
            
            "set left position": self.set_left_position_button_click,
            
            "set right position": self.set_right_position_button_click,
            # Add more voice commands here ..
        }

        # (important) Start the voice command thread
        self.voice_thread = VoiceCommandThread(offline_mode=self.voice_offline_mode)
        self.voice_thread.command_recognized.connect(self.handle_voice_command)
        self.voice_thread.start()
        # ---------------------------------------------------------        


        self.spacenav_twist = Twist()  # Set it to an empty twist message
        self.last_spacenav_twist_time = rospy.Time.now()  # For timestamping of the last twist msg
        self.spacenav_twist_wait_timeout = rospy.Duration(1.0)  # Timeout duration to wait twist msg before zeroing in seconds
        self.sub_twist = rospy.Subscriber("/spacenav/twist", Twist, self.spacenav_twist_callback, queue_size=1)
        
        # Set it to True if you have a second spacenav twist topic to control the second (left) hand
        self.dual_spacenav_twist = rospy.get_param("~dual_spacenav_twist", False)
        
        if self.dual_spacenav_twist:
            self.spacenav_twist2 = Twist()  # Set it to an empty twist message
            self.last_spacenav_twist2_time = rospy.Time.now()  # For timestamping of the last twist msg
            self.spacenav_twist2_wait_timeout = rospy.Duration(1.0)  # Timeout duration to wait twist msg before zeroing in seconds
            self.sub_twist2 = rospy.Subscriber("/spacenav2/twist", Twist, self.spacenav_twist2_callback, queue_size=1)
        
        
        # self.sub_state_array = rospy.Subscriber("/fabric_state", SegmentStateArray, self.state_array_callback, queue_size=1)
        self.sub_state_array = rospy.Subscriber("/fabric_state" + "_minimal", SegmentStateArray, run_in_thread(self.state_array_callback), queue_size=1)

        self.pub_change_dynamicity = rospy.Publisher("/change_particle_dynamicity", 
                                                        ChangeParticleDynamicity, queue_size=1)
        self.pub_attach_hand_to_fabric = rospy.Publisher("/attach_external_odom_frame_request", 
                                                            AttachExternalOdomFrameRequest, queue_size=1)
        self.pub_stick_nearest_fabric_particle = rospy.Publisher("/fix_nearest_fabric_particle_request", 
                                                                    FixNearestFabricParticleRequest, queue_size=1)

        self.last_timestep_requests = {}


        self.odom_pub_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.odom_pub_timer_callback)

    def initialize_modulus_services_and_publishers(self, simulator_node_name=""):
        # Service clients
        self.get_stretching_compliance_service_name = simulator_node_name + "/get_fabric_stretching_compliance"
        self.get_bending_compliance_service_name = simulator_node_name + "/get_fabric_bending_compliance"
        self.enable_collision_handling_service_name = simulator_node_name + "/enable_collision_handling"

        # Wait for services to be available
        print("Waiting for stiffness services...")
        rospy.wait_for_service(self.get_stretching_compliance_service_name)
        rospy.wait_for_service(self.get_bending_compliance_service_name)
        rospy.wait_for_service(self.enable_collision_handling_service_name)
        print("Stiffness Services are available.")

        self.get_stretching_compliance_service_client = rospy.ServiceProxy(self.get_stretching_compliance_service_name, GetFabricStretchingCompliance)
        self.get_bending_compliance_service_client = rospy.ServiceProxy(self.get_bending_compliance_service_name, GetFabricBendingCompliance)
        self.enable_collision_handling_service_client = rospy.ServiceProxy(self.enable_collision_handling_service_name, EnableCollisionHandling)

        # Publishers
        self.change_stretching_compliance_publisher = rospy.Publisher("/change_fabric_stretching_compliance", Float32, queue_size=1)
        self.change_bending_compliance_publisher = rospy.Publisher("/change_fabric_bending_compliance", Float32, queue_size=1)
        
    def initialize_controller_services_and_publishers(self, controller_node_name=""):        
        # Service clients
        self.set_enable_controller_service_name = controller_node_name + "/set_enable_controller"
        self.set_pause_controller_service_name = controller_node_name + "/set_pause_controller"
        self.set_nominal_control_enabled_service_name = controller_node_name + "/set_nominal_control_enabled"
        self.set_obstacle_avoidance_enabled_service_name = controller_node_name + "/set_obstacle_avoidance_enabled"
        self.set_stress_avoidance_enabled_service_name = controller_node_name + "/set_stress_avoidance_enabled"
        self.reset_target_poses_wrt_leader_service_name = controller_node_name + "/reset_target_poses_wrt_leader"

        # Wait for services to be available
        print("Waiting for controller services...")
        rospy.wait_for_service(self.set_enable_controller_service_name)
        rospy.wait_for_service(self.set_pause_controller_service_name)
        rospy.wait_for_service(self.set_nominal_control_enabled_service_name)
        rospy.wait_for_service(self.set_obstacle_avoidance_enabled_service_name)
        rospy.wait_for_service(self.set_stress_avoidance_enabled_service_name)
        rospy.wait_for_service(self.reset_target_poses_wrt_leader_service_name)
        print("Controller Services are available.")

        self.set_enable_controller_service_client = rospy.ServiceProxy(self.set_enable_controller_service_name, SetBool)
        self.set_pause_controller_service_client = rospy.ServiceProxy(self.set_pause_controller_service_name, SetBool)
        self.set_nominal_control_enabled_service_client = rospy.ServiceProxy(self.set_nominal_control_enabled_service_name, SetBool)
        self.set_obstacle_avoidance_enabled_service_client = rospy.ServiceProxy(self.set_obstacle_avoidance_enabled_service_name, SetBool)
        self.set_stress_avoidance_enabled_service_client = rospy.ServiceProxy(self.set_stress_avoidance_enabled_service_name, SetBool)
        self.reset_target_poses_wrt_leader_service_client = rospy.ServiceProxy(self.reset_target_poses_wrt_leader_service_name, Empty)

        # Publishers
        # Add here the publishers for the controller node ..

    def add_horizontal_line_to_layout(self, layout):
        # Create a horizontal line
        h_line = qt_widgets.QFrame()
        h_line.setFrameShape(qt_widgets.QFrame.HLine)
        h_line.setFrameShadow(qt_widgets.QFrame.Sunken)
        layout.addWidget(h_line)

    def add_vertical_line_to_layout(self, layout):
        # Create a vertical line
        v_line = qt_widgets.QFrame()
        v_line.setFrameShape(qt_widgets.QFrame.VLine)
        v_line.setFrameShadow(qt_widgets.QFrame.Sunken)
        layout.addWidget(v_line)

    def createUI(self):
        self.layout = qt_widgets.QVBoxLayout(self)

        self.buttons_manual = {}  # To enable/disable manual control

        self.text_inputs_pos = {}  # To manually set x y z positions of the particles
        self.text_inputs_ori = {}  # To manually set x y z orientations of the particles (Euler RPY, degree input)

        # Add modulus controls at the top
        self.add_sim_controls()
        
        # Add a horizontal line here
        self.add_horizontal_line_to_layout(self.layout)
        
        # Add controller related interface if the mode is composite_sheet_application_test
        if self.mode == "composite_sheet_application_test":
            # Add controller related interface
            self.add_controller_related_interface()
            
            # Add a horizontal line here
            self.add_horizontal_line_to_layout(self.layout)
        
        # Combine the lists with boundary markers
        combined_particles = (self.binded_particles + ["_boundary_marker_"] +
                              self.custom_static_particles + ["_boundary_marker_"])

        # Create rows for the (custom static) particles and the self.binded_particles
        for particle in combined_particles:
            # Insert a horizontal line between each list (self.binded_particles, self.custom_static_particles)
            if particle == "_boundary_marker_":
                # Add a horizontal line here
                self.add_horizontal_line_to_layout(self.layout)
                continue

            # Create QHBoxLayout for each row
            row_layout = qt_widgets.QHBoxLayout()

            # ------------------------------------------------------------------------------------
            # Create components common to all particles (self.binded_particles and custom static particles)
            
            # Add manual control button for velocity commands to particles 
            manual_control_button = qt_widgets.QPushButton()
            manual_control_button.setText("Manually Control " + str(particle))
            manual_control_button.setCheckable(True)  # Enables toggle behavior
            manual_control_button.setChecked(False)
            manual_control_button.clicked.connect(lambda _, p=particle: self.manual_control_button_pressed_cb(p))
            row_layout.addWidget(manual_control_button)  # Add button to row layout

            self.buttons_manual[particle] = manual_control_button

            # Add a separator vertical line here
            self.add_vertical_line_to_layout(row_layout)

            # Add button to get current pose to GUI for easy set position and orientation operations
            get_pose_button = qt_widgets.QPushButton()
            get_pose_button.setText("Get Pose")
            get_pose_button.clicked.connect(lambda _, p=particle: self.get_pose_button_pressed_cb(p))
            row_layout.addWidget(get_pose_button)

            # Add a separator vertical line here
            self.add_vertical_line_to_layout(row_layout)

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

            if self.is_orientation_control_enabled:
                # ------------------------------------------------
                # Add a separator vertical line here
                self.add_vertical_line_to_layout(row_layout)
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
            # ------------------------------------------------------------------------------------

            # Add a separator vertical line here
            self.add_vertical_line_to_layout(row_layout)

            # ------------------------------------------------------------------------------------
            # Create Buttons dedicated to the binded particles
            if particle in self.binded_particles: 
                # Send a binded particle to the centroid of the custom static particles
                send_to_centroid_button = qt_widgets.QPushButton()
                send_to_centroid_button.setText("Send to Center")
                send_to_centroid_button.clicked.connect(lambda _, p=particle: self.send_to_centroid_cb(p))
                row_layout.addWidget(send_to_centroid_button)
                
                # Create a vertical line here
                self.add_vertical_line_to_layout(row_layout)
                
                # Create Attach hand to fabric button
                attach_to_fabric_button = qt_widgets.QPushButton()
                attach_to_fabric_button.setText("Attach to Fabric")
                attach_to_fabric_button.clicked.connect(lambda _, p=particle: self.attach_hand_to_fabric_cb(p, is_attach=True))
                row_layout.addWidget(attach_to_fabric_button)
                
                # Create Detach hand from fabric button
                detach_from_fabric_button = qt_widgets.QPushButton()
                detach_from_fabric_button.setText("Detach from Fabric")
                detach_from_fabric_button.clicked.connect(lambda _, p=particle: self.attach_hand_to_fabric_cb(p, is_attach=False))
                row_layout.addWidget(detach_from_fabric_button)
                
                # Create a vertical line here
                self.add_vertical_line_to_layout(row_layout)
                
                # Create Stick (object to mandrel) button
                stick_button = qt_widgets.QPushButton()
                stick_button.setText("Stick")
                stick_button.clicked.connect(lambda _, p=particle: self.stick_nearest_fabric_particle_cb(p, is_stick=True))
                row_layout.addWidget(stick_button)
                
                # Create Unstick (object from mandrel) button
                unstick_button = qt_widgets.QPushButton()
                unstick_button.setText("Unstick")
                unstick_button.clicked.connect(lambda _, p=particle: self.stick_nearest_fabric_particle_cb(p, is_stick=False))
                row_layout.addWidget(unstick_button)
                
            # ------------------------------------------------------------------------------------

            # ------------------------------------------------------------------------------------
            # Create Buttons dedicated to the custom static particles
            if particle in self.custom_static_particles:
                # Create Reset fabric positions button
                reset_button = qt_widgets.QPushButton()
                reset_button.setText("Reset Controller Position")
                reset_button.setEnabled(False)  # Disable the button
                row_layout.addWidget(reset_button)

                # Create Make Dynamic Button
                make_dynamic_button = qt_widgets.QPushButton()
                make_dynamic_button.setText("Make Dynamic")
                make_dynamic_button.clicked.connect(lambda _, p=particle: self.change_dynamicity_cb(p, True))
                row_layout.addWidget(make_dynamic_button)

                # Create Make Static Button
                make_static_button = qt_widgets.QPushButton()
                make_static_button.setText("Make Static")
                make_static_button.clicked.connect(lambda _, p=particle: self.change_dynamicity_cb(p, False))
                row_layout.addWidget(make_static_button)
            # ------------------------------------------------------------------------------------

            # Add row layout to the main layout
            self.layout.addLayout(row_layout)

            # Add the odometry publisher for each particle
            if particle in self.binded_particles: 
                self.odom_publishers[particle] = rospy.Publisher(self.binded_particles_odom_topic_prefix + str(particle), 
                                                                    Odometry, queue_size=1)
            if particle in self.custom_static_particles:
                self.odom_publishers[particle] = rospy.Publisher(self.odom_topic_prefix + str(particle), 
                                                                    Odometry, queue_size=1)


        self.setLayout(self.layout)

        self.shutdown_timer.timeout.connect(self.check_shutdown)
        self.shutdown_timer.start(1000)  # Timer triggers every 1000 ms (1 second)

    def add_sim_controls(self):
        if self.is_orientation_control_enabled:
            # Original horizontal layout when orientation control is enabled
            compliance_layout = qt_widgets.QHBoxLayout()

            # Stretching Compliance Controls
            self.get_stretching_compliance_button = qt_widgets.QPushButton("Get Stretching Compliance")
            self.get_stretching_compliance_button.clicked.connect(self.get_stretching_compliance_button_pressed_cb)
            compliance_layout.addWidget(self.get_stretching_compliance_button)

            stretching_compliance_label = qt_widgets.QLabel("Stretching Compliance:")
            compliance_layout.addWidget(stretching_compliance_label)

            self.stretching_compliance_text_input = qt_widgets.QLineEdit()
            self.stretching_compliance_text_input.setPlaceholderText("Stretching Compliance")
            self.stretching_compliance_text_input.setMinimumWidth(100)
            compliance_layout.addWidget(self.stretching_compliance_text_input)

            self.set_stretching_compliance_button = qt_widgets.QPushButton("Set Stretching Compliance")
            self.set_stretching_compliance_button.clicked.connect(self.set_stretching_compliance_button_pressed_cb)
            compliance_layout.addWidget(self.set_stretching_compliance_button)

            self.add_vertical_line_to_layout(compliance_layout)

            # Bending Compliance Controls
            self.get_bending_compliance_button = qt_widgets.QPushButton("Get Bending Compliance")
            self.get_bending_compliance_button.clicked.connect(self.get_bending_compliance_button_pressed_cb)
            compliance_layout.addWidget(self.get_bending_compliance_button)

            bending_compliance_label = qt_widgets.QLabel("Bending Compliance:")
            compliance_layout.addWidget(bending_compliance_label)

            self.bending_compliance_text_input = qt_widgets.QLineEdit()
            self.bending_compliance_text_input.setPlaceholderText("Bending Compliance")
            self.bending_compliance_text_input.setMinimumWidth(100)
            compliance_layout.addWidget(self.bending_compliance_text_input)

            self.set_bending_compliance_button = qt_widgets.QPushButton("Set Bending Compliance")
            self.set_bending_compliance_button.clicked.connect(self.set_bending_compliance_button_pressed_cb)
            compliance_layout.addWidget(self.set_bending_compliance_button)

            self.add_vertical_line_to_layout(compliance_layout)
            
            # Collision Handling Controls
            self.enable_collision_button = qt_widgets.QPushButton("Enable Collision Handling")
            self.enable_collision_button.clicked.connect(self.enable_collision_button_pressed_cb)
            compliance_layout.addWidget(self.enable_collision_button)

            self.disable_collision_button = qt_widgets.QPushButton("Disable Collision Handling")
            self.disable_collision_button.clicked.connect(self.disable_collision_button_pressed_cb)
            compliance_layout.addWidget(self.disable_collision_button)

            # Add the horizontal compliance_layout to the main layout
            self.layout.addLayout(compliance_layout)
        else:
            # Orientation control disabled:
            # Create a two-column layout with a vertical line separator in between.

            # Main horizontal layout that will hold both columns and the separator
            main_layout = qt_widgets.QHBoxLayout()

            # Left column: Compliance controls in a vertical layout
            compliance_layout = qt_widgets.QVBoxLayout()

            # Row 1: Stretching Compliance Controls (arranged horizontally)
            stretching_layout = qt_widgets.QHBoxLayout()
            self.get_stretching_compliance_button = qt_widgets.QPushButton("Get Stretching Compliance")
            self.get_stretching_compliance_button.clicked.connect(self.get_stretching_compliance_button_pressed_cb)
            stretching_layout.addWidget(self.get_stretching_compliance_button)

            stretching_compliance_label = qt_widgets.QLabel("Stretching Compliance:")
            stretching_layout.addWidget(stretching_compliance_label)

            self.stretching_compliance_text_input = qt_widgets.QLineEdit()
            self.stretching_compliance_text_input.setPlaceholderText("Stretching Compliance")
            self.stretching_compliance_text_input.setMinimumWidth(100)
            stretching_layout.addWidget(self.stretching_compliance_text_input)

            self.set_stretching_compliance_button = qt_widgets.QPushButton("Set Stretching Compliance")
            self.set_stretching_compliance_button.clicked.connect(self.set_stretching_compliance_button_pressed_cb)
            stretching_layout.addWidget(self.set_stretching_compliance_button)

            compliance_layout.addLayout(stretching_layout)

            # Row 2: Bending Compliance Controls (arranged horizontally)
            bending_layout = qt_widgets.QHBoxLayout()
            self.get_bending_compliance_button = qt_widgets.QPushButton("Get Bending Compliance")
            self.get_bending_compliance_button.clicked.connect(self.get_bending_compliance_button_pressed_cb)
            bending_layout.addWidget(self.get_bending_compliance_button)

            bending_compliance_label = qt_widgets.QLabel("Bending Compliance:")
            bending_layout.addWidget(bending_compliance_label)

            self.bending_compliance_text_input = qt_widgets.QLineEdit()
            self.bending_compliance_text_input.setPlaceholderText("Bending Compliance")
            self.bending_compliance_text_input.setMinimumWidth(100)
            bending_layout.addWidget(self.bending_compliance_text_input)

            self.set_bending_compliance_button = qt_widgets.QPushButton("Set Bending Compliance")
            self.set_bending_compliance_button.clicked.connect(self.set_bending_compliance_button_pressed_cb)
            bending_layout.addWidget(self.set_bending_compliance_button)

            compliance_layout.addLayout(bending_layout)

            # Right column: Collision Handling Controls in a vertical layout
            collision_layout = qt_widgets.QVBoxLayout()
            self.enable_collision_button = qt_widgets.QPushButton("Enable Collision Handling")
            self.enable_collision_button.clicked.connect(self.enable_collision_button_pressed_cb)
            collision_layout.addWidget(self.enable_collision_button)

            self.disable_collision_button = qt_widgets.QPushButton("Disable Collision Handling")
            self.disable_collision_button.clicked.connect(self.disable_collision_button_pressed_cb)
            collision_layout.addWidget(self.disable_collision_button)

            # Add left column to the main layout
            main_layout.addLayout(compliance_layout)

            # Add vertical line separator that spans the height of both rows
            self.add_vertical_line_to_layout(main_layout)

            # Add right column to the main layout
            main_layout.addLayout(collision_layout)

            # Add the complete two-column layout to the overall layout
            self.layout.addLayout(main_layout)

    def add_controller_related_interface(self):
        # Add the controllers controls here
        # Main horizontal layout that will hold columns and the separators
        main_layout = qt_widgets.QHBoxLayout()
        
        # -------------------------------------------------------------
        # Column 1: Reset Target Poses WRT Leader button layout
        reset_target_poses_layout = qt_widgets.QVBoxLayout()
        self.reset_target_poses_button = qt_widgets.QPushButton("Reset Target Poses")
        self.reset_target_poses_button.clicked.connect(self.reset_target_poses_cb)
        reset_target_poses_layout.addWidget(self.reset_target_poses_button)
        
        # Add the column 1 to the main layout
        main_layout.addLayout(reset_target_poses_layout)
        # Add vertical line separator that spans the height of both columns
        self.add_vertical_line_to_layout(main_layout)
        # -------------------------------------------------------------
        
        # -------------------------------------------------------------
        # Column 2: Enable/Disable Nominal Control
        nominal_control_layout = qt_widgets.QVBoxLayout()
        self.nominal_control_enable_button = qt_widgets.QPushButton("Enable Nominal Control")
        self.nominal_control_enable_button.clicked.connect(lambda _, is_enabled=True: self.nominal_control_toggle_cb(is_enabled))
        nominal_control_layout.addWidget(self.nominal_control_enable_button)
        
        self.nominal_control_disable_button = qt_widgets.QPushButton("Disable Nominal Control")
        self.nominal_control_disable_button.clicked.connect(lambda _, is_enabled=False: self.nominal_control_toggle_cb(is_enabled))
        nominal_control_layout.addWidget(self.nominal_control_disable_button)
        
        # Add the column 2 to the main layout
        main_layout.addLayout(nominal_control_layout)
        # Add vertical line separator that spans the height of both columns
        self.add_vertical_line_to_layout(main_layout)
        # -------------------------------------------------------------
        
        # -------------------------------------------------------------
        # Column 3: Enable/Disable Obstacle Avoidance
        obstacle_avoidance_layout = qt_widgets.QVBoxLayout()
        self.obstacle_avoidance_enable_button = qt_widgets.QPushButton("Enable Obstacle Avoidance")
        self.obstacle_avoidance_enable_button.clicked.connect(lambda _, is_enabled=True: self.obstacle_avoidance_toggle_cb(is_enabled))
        obstacle_avoidance_layout.addWidget(self.obstacle_avoidance_enable_button)
        
        self.obstacle_avoidance_disable_button = qt_widgets.QPushButton("Disable Obstacle Avoidance")
        self.obstacle_avoidance_disable_button.clicked.connect(lambda _, is_enabled=False: self.obstacle_avoidance_toggle_cb(is_enabled))
        obstacle_avoidance_layout.addWidget(self.obstacle_avoidance_disable_button)
        
        # Add the column 3 to the main layout
        main_layout.addLayout(obstacle_avoidance_layout)
        # Add vertical line separator that spans the height of both columns
        self.add_vertical_line_to_layout(main_layout)
        # -------------------------------------------------------------
        
        # -------------------------------------------------------------
        # Column 4: Enable/Disable Stress Avoidance
        stress_avoidance_layout = qt_widgets.QVBoxLayout()
        self.stress_avoidance_enable_button = qt_widgets.QPushButton("Enable Stress Avoidance")
        self.stress_avoidance_enable_button.clicked.connect(lambda _, is_enabled=True: self.stress_avoidance_toggle_cb(is_enabled))
        stress_avoidance_layout.addWidget(self.stress_avoidance_enable_button)
        
        self.stress_avoidance_disable_button = qt_widgets.QPushButton("Disable Stress Avoidance")
        self.stress_avoidance_disable_button.clicked.connect(lambda _, is_enabled=False: self.stress_avoidance_toggle_cb(is_enabled))
        stress_avoidance_layout.addWidget(self.stress_avoidance_disable_button)
        
        # Add the column 4 to the main layout
        main_layout.addLayout(stress_avoidance_layout)
        # Add vertical line separator that spans the height of both columns
        self.add_vertical_line_to_layout(main_layout)
        # -------------------------------------------------------------
        
        # -------------------------------------------------------------
        # Column 5: Pause/Resume Controller
        pause_controller_layout = qt_widgets.QVBoxLayout()
        self.pause_controller_button = qt_widgets.QPushButton("Pause Controller")
        self.pause_controller_button.clicked.connect(lambda _, is_paused=True: self.pause_controller_toggle_cb(is_paused))
        pause_controller_layout.addWidget(self.pause_controller_button)
        
        self.resume_controller_button = qt_widgets.QPushButton("Resume Controller")
        self.resume_controller_button.clicked.connect(lambda _, is_paused=False: self.pause_controller_toggle_cb(is_paused))
        pause_controller_layout.addWidget(self.resume_controller_button)
        
        # Add the column 5 to the main layout
        main_layout.addLayout(pause_controller_layout)
        # -------------------------------------------------------------
        
        # -------------------------------------------------------------
        # Last Column: Enable/Disable Controller
        controller_toggle_layout = qt_widgets.QVBoxLayout()
        self.controller_enable_button = qt_widgets.QPushButton("Enable Controller")
        self.controller_enable_button.clicked.connect(lambda _, is_enabled=True: self.controller_toggle_cb(is_enabled))
        controller_toggle_layout.addWidget(self.controller_enable_button)
        
        self.controller_disable_button = qt_widgets.QPushButton("Disable Controller")
        self.controller_disable_button.clicked.connect(lambda _, is_enabled=False: self.controller_toggle_cb(is_enabled))
        controller_toggle_layout.addWidget(self.controller_disable_button)
        
        # Add the last column to the main layout
        main_layout.addLayout(controller_toggle_layout)
        # -------------------------------------------------------------
        
        # Add the main layout to the overall layout
        self.layout.addLayout(main_layout)
        
        
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

    # -------------------------------------------------------------------------------------
    # Controller related callback functions
    def reset_target_poses_cb(self):
        try:
            # Call the reset_target_poses_wrt_leader service
            response = self.reset_target_poses_wrt_leader_service_client()
            rospy.loginfo("Target Poses reset successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call to reset_target_poses_wrt_leader failed: %s", e)
            
    def nominal_control_toggle_cb(self, is_enabled):
        try:
            # Call the set_nominal_control_enabled service
            response = self.set_nominal_control_enabled_service_client(is_enabled)
            if response.success:
                rospy.loginfo("Nominal Control enabled/disabled successfully.")
            else:
                rospy.logerr("Failed to enable/disable nominal control.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call to set_nominal_control_enabled failed: %s", e)
            
    def obstacle_avoidance_toggle_cb(self, is_enabled):
        try:
            # Call the set_obstacle_avoidance_enabled service
            response = self.set_obstacle_avoidance_enabled_service_client(is_enabled)
            if response.success:
                rospy.loginfo("Obstacle Avoidance enabled/disabled successfully.")
            else:
                rospy.logerr("Failed to enable/disable obstacle avoidance.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call to set_obstacle_avoidance_enabled failed: %s", e)
            
    def stress_avoidance_toggle_cb(self, is_enabled):
        try:
            # Call the set_stress_avoidance_enabled service
            response = self.set_stress_avoidance_enabled_service_client(is_enabled)
            if response.success:
                rospy.loginfo("Stress Avoidance enabled/disabled successfully.")
            else:
                rospy.logerr("Failed to enable/disable stress avoidance.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call to set_stress_avoidance_enabled failed: %s", e)

    def pause_controller_toggle_cb(self, is_paused):
        try:
            # Call the set_pause_controller service
            response = self.set_pause_controller_service_client(is_paused)
            if response.success:
                rospy.loginfo("Controller paused/resumed successfully.")
            else:
                rospy.logerr("Failed to pause/resume controller.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call to set_pause_controller failed: %s", e)

    def controller_toggle_cb(self, is_enabled):
        try:
            # Call the set_enable_controller service
            response = self.set_enable_controller_service_client(is_enabled)
            if response.success:
                rospy.loginfo("Controller enabled/disabled successfully.")
            else:
                rospy.logerr("Failed to enable/disable controller.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call to set_enable_controller failed: %s", e)
    # -------------------------------------------------------------------------------------

    # -------------------------------------------------------------------------------------
    # Voice recognition related functions
    # Provide small wrapper methods you can call from voice_command_actions:
    def reset_target_poses_button_click(self):
        self.reset_target_poses_button.click()

    def enable_nominal_control_button_click(self):
        self.nominal_control_enable_button.click()

    def disable_nominal_control_button_click(self):
        self.nominal_control_disable_button.click()
        
    def get_stretching_compliance_button_click(self):
        self.get_stretching_compliance_button.click()

    def get_bending_compliance_button_click(self):
        self.get_bending_compliance_button.click()

    def enable_collision_button_click(self):
        self.enable_collision_button.click()

    def disable_collision_button_click(self):
        self.disable_collision_button.click()

    def enable_obstacle_avoidance_button_click(self):
        self.obstacle_avoidance_enable_button.click()

    def disable_obstacle_avoidance_button_click(self):
        self.obstacle_avoidance_disable_button.click()

    def enable_stress_avoidance_button_click(self):
        self.stress_avoidance_enable_button.click()

    def disable_stress_avoidance_button_click(self):
        self.stress_avoidance_disable_button.click()

    def pause_controller_button_click(self):
        self.pause_controller_button.click()

    def resume_controller_button_click(self):
        self.resume_controller_button.click()

    def enable_controller_button_click(self):
        self.controller_enable_button.click()

    def disable_controller_button_click(self):
        self.controller_disable_button.click()
        
    def attach_left_hand_button_click(self):
        # left hand is "hand_2"
        self.attach_hand_to_fabric_cb("hand_2", is_attach=True)

    def detach_left_hand_button_click(self):
        self.attach_hand_to_fabric_cb("hand_2", is_attach=False)

    def attach_right_hand_button_click(self):
        # right hand is "hand_1"
        self.attach_hand_to_fabric_cb("hand_1", is_attach=True)

    def detach_right_hand_button_click(self):
        self.attach_hand_to_fabric_cb("hand_1", is_attach=False)

    def stick_left_button_click(self):
        # again, left is "hand_2"
        self.stick_nearest_fabric_particle_cb("hand_2", is_stick=True)

    def unstick_left_button_click(self):
        self.stick_nearest_fabric_particle_cb("hand_2", is_stick=False)

    def stick_right_button_click(self):
        # right is "hand_1"
        self.stick_nearest_fabric_particle_cb("hand_1", is_stick=True)

    def unstick_right_button_click(self):
        self.stick_nearest_fabric_particle_cb("hand_1", is_stick=False)
        
    def get_left_corner_pose_from_particle(self):
        # Assuming 4 custom static particles are present, the left corner particle is the second last particle
        particle = self.custom_static_particles[-2] # left_hand_corresponding_particle
        text_input_particle = "hand_2" # left hand
        self.get_pose_common(particle, text_input_particle)
        
    def get_right_corner_pose_from_particle(self):
        # Assuming 4 custom static particles are present, the right corner particle is the last particle
        particle = self.custom_static_particles[-1]
        text_input_particle = "hand_1" # right hand
        self.get_pose_common(particle, text_input_particle)
        
    def set_left_position_button_click(self):
        self.set_position_cb_basic("hand_2")
        
    def set_right_position_button_click(self):
        self.set_position_cb_basic("hand_1")


    # Slot that is called whenever we get recognized text
    def handle_voice_command(self, recognized_text):
        # See if recognized_text matches a known command
        if recognized_text in self.voice_command_actions:
            rospy.loginfo(f"Executing command: {recognized_text}")
            
            # Call the associated action
            self.voice_command_actions[recognized_text]()
            
            # Provide audible feedback
            # self.say_text(f"I heard {recognized_text}.")
            # self.say_text(f"{recognized_text}.")
            filename="/usr/share/sounds/Yaru/stereo/complete.oga"
            os.system(f"ffplay -nodisp -autoexit {filename}")
            
        else:
            rospy.logwarn(f"No matching command for: {recognized_text}")
            
            # Provide audible feedback
            # self.say_text(f"I did not understand.")
            filename="/usr/share/sounds/Yaru/stereo/dialog-error.oga"
            os.system(f"ffplay -nodisp -autoexit {filename}")

    def say_text(self, text):
        if self.voice_offline_mode:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        else:
            self.speak_online(text)
        
    def speak_online(self, text, filename="temp.mp3"):
        """Convert text to speech and play the sound."""
        try:
            # Generate speech
            tts = gTTS(text=text, lang="en", slow=False)
            tts.save(filename)

            # Play the saved audio file
            # os.system(f"start {filename}")  # Windows
            # os.system(f"afplay {filename}")  # macOS
            # os.system(f"mpg321 {filename}")  # Linux
            os.system(f"ffplay -nodisp -autoexit {filename}")

        except Exception as e:
            rospy.logerr(f"Error in text-to-speech: {e}")
    # -------------------------------------------------------------------------------------

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

    def get_pose_common(self, particle, text_input_particle=None):
        """
        Gets the current pose of the particle 
        and fills the text_inputs for pos and ori accordingly
        """
        if text_input_particle is None:
            text_input_particle = particle
        
        # Check if the particle pose is set
        if (particle in self.particle_positions) and (particle in self.particle_orientations):
            # Get Current Pose of the particle in world frame
            pos = self.particle_positions[particle] # Point() msg of ROS geometry_msgs
            ori = self.particle_orientations[particle] # Quaternion() msg of ROS geometry_msgs

            # Fill the position text inputs with the current position
            self.text_inputs_pos[text_input_particle]['x'].setText(self.format_number(pos.x,digits=3)) 
            self.text_inputs_pos[text_input_particle]['y'].setText(self.format_number(pos.y,digits=3)) 
            self.text_inputs_pos[text_input_particle]['z'].setText(self.format_number(pos.z,digits=3)) 

            if self.is_orientation_control_enabled:
                # Convert quaternion orientation to RPY (Roll-pitch-yaw) Euler Angles (degrees)
                rpy = np.rad2deg(tf_trans.euler_from_quaternion([ori.x,ori.y,ori.z,ori.w]))

                # Fill the orientation text  inputs with the current RPY orientation
                self.text_inputs_ori[text_input_particle]['x'].setText(self.format_number(rpy[0],digits=1))
                self.text_inputs_ori[text_input_particle]['y'].setText(self.format_number(rpy[1],digits=1))
                self.text_inputs_ori[text_input_particle]['z'].setText(self.format_number(rpy[2],digits=1))
        else:
            rospy.logwarn(f"Key '{particle}' not found in the particle_positions and particle_orientations dictionaries.")

    def get_pose_button_pressed_cb(self, particle):
        self.get_pose_common(particle)

    def set_position_cb_basic(self, particle):
        pose = Pose()
        pose.position.x = float(self.text_inputs_pos[particle]['x'].text())
        pose.position.y = float(self.text_inputs_pos[particle]['y'].text())
        pose.position.z = float(self.text_inputs_pos[particle]['z'].text())

        # Keep the same orientation
        pose.orientation = self.particle_orientations[particle]

        # if particle in self.binded_particles:
        self.particle_positions[particle] = pose.position
        self.particle_orientations[particle] = pose.orientation

        # Prepare Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map" 
        
        if particle in self.binded_particles:
            odom.child_frame_id = f"{particle}_odom"
        if particle in self.custom_static_particles:
            odom.child_frame_id = f"particle_{particle}_odom"
        
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
        if not self.is_orientation_control_enabled:
            return
        
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

        # if particle in self.binded_particles:
        self.particle_positions[particle] = pose.position
        self.particle_orientations[particle] = pose.orientation

        # Prepare Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map" 
        
        if particle in self.binded_particles:
            odom.child_frame_id = f"{particle}_odom"
        if particle in self.custom_static_particles:
            odom.child_frame_id = f"particle_{particle}_odom"
        
        odom.pose.pose = pose

        self.odom_publishers[particle].publish(odom)

    def set_orientation_cb(self,particle):
        if not self.is_orientation_control_enabled:
            return
        
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
                
                if particle in self.binded_particles:
                    self.particle_positions[particle] = new_position
                    self.particle_orientations[particle] = new_orientation

                # Prepare odometry message
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()
                odom.header.frame_id = "map"
                
                if particle in self.binded_particles:
                    odom.child_frame_id = f"{particle}_odom"
                if particle in self.custom_static_particles:
                    odom.child_frame_id = f"particle_{particle}_odom"
                
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
    

    def send_to_centroid_cb(self,particle_to_send):
        # Calculate the centroid of the other particles
        centroid_pos = np.zeros(3)
        n_particle = 0 # number of particles to calculate the centroid
        for particle in self.custom_static_particles:
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

            if particle_to_send in self.binded_particles:
                self.particle_positions[particle_to_send] = pose.position
                self.particle_orientations[particle_to_send] = pose.orientation

            # Prepare Odometry message
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map" 
            
            if particle in self.binded_particles:
                odom.child_frame_id = f"{particle}_odom"
            if particle in self.custom_static_particles:
                odom.child_frame_id = f"particle_{particle}_odom"
            
            odom.pose.pose = pose

            self.odom_publishers[particle_to_send].publish(odom)
        else:
            rospy.logwarn("There is no particle to calculate the centroid.")

    def attach_hand_to_fabric_cb(self, particle, is_attach):
        rospy.loginfo(f"Attach hand odometry frame to fabric: {is_attach}")
        rospy.loginfo(f"particle: {particle}")
        rospy.loginfo(f"odom_topic: {self.odom_publishers[particle].resolved_name}")
        
        # Publish the topic    
        msg = AttachExternalOdomFrameRequest()
        msg.is_attach = is_attach
        msg.odom_topic = self.odom_publishers[particle].resolved_name
        self.pub_attach_hand_to_fabric.publish(msg)

    def stick_nearest_fabric_particle_cb(self, particle, is_stick):
        rospy.loginfo(f"Stick nearest fabric particle: {is_stick}")
        rospy.loginfo(f"particle: {particle}")
        rospy.loginfo(f"position: {self.particle_positions[particle]}")
        rospy.loginfo(f"orientation: {self.particle_orientations[particle]}")
        
        # Publish the topic    
        msg = FixNearestFabricParticleRequest()
        msg.is_fix = is_stick        
        msg.pose.header.frame_id = "map" # Note that pose is poseStamped msg type
        msg.pose.header.stamp = rospy.Time.now()
        msg.pose.pose.position = self.particle_positions[particle]
        msg.pose.pose.orientation = self.particle_orientations[particle]
        
        self.pub_stick_nearest_fabric_particle.publish(msg)
        
    def odom_pub_timer_callback(self,event):
        # Reset spacenav_twist to zero if it's been long time since the last arrived
        self.check_spacenav_twist_wait_timeout()
        
        if self.dual_spacenav_twist:
            self.check_spacenav_twist2_wait_timeout()
        
        with self.lock:
            # Handle manual control of each custom static particle and the binded particles
            for particle in self.combined_particles:
                # Do not proceed with the particles until the initial position values have been set
                if not (particle in self.particle_positions):
                    continue

                if self.buttons_manual[particle].isChecked():
                    dt = self.get_timestep(particle)   
                    # dt = 0.01
                    # dt = 1. / self.pub_rate_odom
                    
                    # If second spacenav twist is enabled, use the second spacenav twist for hand_2 particle
                    if self.dual_spacenav_twist and particle == "hand_2":
                        twist_cmd = self.spacenav_twist2
                    else:
                        twist_cmd = self.spacenav_twist

                    # simple time step integration using Twist data
                    pose = Pose()
                    pose.position.x = self.particle_positions[particle].x + dt*twist_cmd.linear.x
                    pose.position.y = self.particle_positions[particle].y + dt*twist_cmd.linear.y
                    pose.position.z = self.particle_positions[particle].z + dt*twist_cmd.linear.z

                    # # --------------------------------------------------------------
                    # # To update the orientation with the twist message
                    # Calculate the magnitude of the angular velocity vector
                    omega_magnitude = math.sqrt(twist_cmd.angular.x**2 + 
                                                twist_cmd.angular.y**2 + 
                                                twist_cmd.angular.z**2)
                    
                    # print("omega_magnitude: " + str(omega_magnitude))

                    pose.orientation = self.particle_orientations[particle]

                    if omega_magnitude > 1e-9:  # Avoid division by zero
                        # Create the delta quaternion based on world frame twist
                        delta_quat = tf_trans.quaternion_about_axis(omega_magnitude * dt, [
                            twist_cmd.angular.x / omega_magnitude,
                            twist_cmd.angular.y / omega_magnitude,
                            twist_cmd.angular.z / omega_magnitude
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

                    if particle in self.binded_particles:
                        self.particle_positions[particle] = pose.position
                        self.particle_orientations[particle] = pose.orientation

                    # Prepare Odometry message
                    odom = Odometry()
                    odom.header.stamp = rospy.Time.now()
                    odom.header.frame_id = "map" 
                    
                    if particle in self.binded_particles:
                        odom.child_frame_id = f"{particle}_odom"
                    if particle in self.custom_static_particles:
                        odom.child_frame_id = f"particle_{particle}_odom"
                    
                    odom.pose.pose = pose
                    odom.twist.twist = twist_cmd
                    self.odom_publishers[particle].publish(odom)

    def change_dynamicity_cb(self,particle, is_dynamic):
        msg = ChangeParticleDynamicity()
        msg.particle_id = particle
        msg.is_dynamic = is_dynamic
        self.pub_change_dynamicity.publish(msg)

    def spacenav_twist_callback(self, twist):
        self.spacenav_twist = twist

        self.last_spacenav_twist_time = rospy.Time.now()
        
    def spacenav_twist2_callback(self, twist):
        self.spacenav_twist2 = twist

        self.last_spacenav_twist2_time = rospy.Time.now()

    def check_spacenav_twist_wait_timeout(self):
        if (rospy.Time.now() - self.last_spacenav_twist_time) > self.spacenav_twist_wait_timeout:
            # Reset spacenav_twist to zero after timeout
            self.spacenav_twist = Twist()

            rospy.loginfo_throttle(2.0,"spacenav_twist is zeroed because it's been long time since the last msg arrived..")
            
    def check_spacenav_twist2_wait_timeout(self):
        if (rospy.Time.now() - self.last_spacenav_twist2_time) > self.spacenav_twist2_wait_timeout:
            # Reset spacenav_twist to zero after timeout
            self.spacenav_twist2 = Twist()

            rospy.loginfo_throttle(2.0,"spacenav_twist2 is zeroed because it's been long time since the last msg arrived..")

    def state_array_callback(self, states_msg):
        with self.lock:
            # for particle in self.custom_static_particles:
            #     # if not self.buttons_manual[particle].isChecked(): # Needed To prevent conflicts in pose updates when applying manual control
            #         self.particle_positions[particle] = states_msg.states[particle].pose.position
            #         self.particle_orientations[particle] = states_msg.states[particle].pose.orientation
            #         self.particle_twists[particle] = states_msg.states[particle].twist
            
            # Convert to a set for faster membership checks
            custom_particle_ids = set(self.custom_static_particles)

            for segment_state in states_msg.states:
                seg_id = segment_state.id
                # Only update if this ID is in the custom_static_particles
                if seg_id in custom_particle_ids:
                    self.particle_positions[seg_id] = segment_state.pose.position
                    self.particle_orientations[seg_id] = segment_state.pose.orientation
                    self.particle_twists[seg_id] = segment_state.twist


    def initialize_binded_particle_positions(self):
        for particle in self.binded_particles:
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

            self.particle_positions[particle] = position
            self.particle_orientations[particle] = quaternion

    def manual_control_button_pressed_cb(self, particle):
        if self.buttons_manual[particle].isChecked():
            # Button is currently pressed, no need to set it to False
            print(f"Button for manual control of particle {particle} is now pressed.")

            # Do not proceed with the particles until the initial values have been set
            if not (particle in self.particle_positions) :
                # Make the manual control button unpressed
                self.buttons_manual[particle].setChecked(False)

                rospy.logwarn("Initial pose values of the object particles or the binded particle is not yet set")

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
            self.voice_thread.stop()  # Tell the voice thread to stop
            qt_widgets.QApplication.quit()

    

if __name__ == "__main__":
    rospy.init_node('test_gui_node', anonymous=False)

    app = qt_widgets.QApplication(sys.argv)

    gui = TestGUI()
    gui.show()

    sys.exit(app.exec_())

#!/usr/bin/env python

# Copyright (c) 2016, HLP-R
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of hlpr_simulator nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Vivian Chu, vchu@gatech.edu

"""    
vector_control_interface.py  

This script takes commands sent through the "real robot" topics and converts them
into standard ROS messages for Gazebo to interpret

Note: No longer depends on MoveIt! Uses trak_ik instead
"""

import rospy
import tf
import math
from collections import defaultdict
from vector_msgs.msg import LinearActuatorCmd, GripperCmd, JacoCartesianVelocityCmd, GripperStat
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from hlpr_trac_ik.srv import *
import dynamixel_msgs.msg

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


class VectorControllerConverter():
    '''
    This node converts Vector Robot specific messages into standard ROS messages
    '''


    def __init__(self):

        rospy.loginfo("Setting up subscribers and publishers")

        self.RIGHT_KEY = 'right'

        # Specific for the linear actuator
        self.linact_pub = rospy.Publisher('/linear_actuator_controller/command', JointTrajectory, queue_size=10) 
        self.linact_joint_names = get_param('/linear_actuator_controller/joints', '')          

        # Setup pan/tilt controller
        self.pan_pub = rospy.Publisher('/pan_sim_controller/command', JointTrajectory, queue_size=10) 
        self.tilt_pub = rospy.Publisher('/tilt_sim_controller/command', JointTrajectory, queue_size=10) 
        self.pan_names = get_param('/pan_sim_controller/joints', '')          
        self.tilt_names = get_param('/tilt_sim_controller/joints', '')          

        # Get flag for one vs. two arms
        self.two_arms = get_param('two_arms', False)
        # Gets flag for 6-DoF vs. 7-DoF
        self.use_7dof_jaco = get_param('use_7dof_jaco', False)

        # Setup pan/tilt sim to real controller topics to simulate the real robot
        # Needed because ROS controller has a different message than Stanley
        self.pan_state_sub = rospy.Subscriber('/pan_sim_controller/state', JointTrajectoryControllerState, self.panStateCallback, queue_size=1)
        self.tilt_state_sub = rospy.Subscriber('/tilt_sim_controller/state', JointTrajectoryControllerState, self.tiltStateCallback, queue_size=1)
        self.pan_state_pub = rospy.Publisher('/pan_controller/state', dynamixel_msgs.msg.JointState, queue_size=10) 
        self.tilt_state_pub = rospy.Publisher('/tilt_controller/state', dynamixel_msgs.msg.JointState, queue_size=10) 

        # Setup the gripper controller
        self.gripper_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
        self.gripper_name = get_param('/gripper_controller/joints', '')

        # Sets up the arm controller
        if self.use_7dof_jaco:
            self.arm_pub = rospy.Publisher('/j2s7s300/command', JointTrajectory, queue_size=10)
        else:
            self.arm_pub = rospy.Publisher('/vector/right_arm/command', JointTrajectory, queue_size=10)

        # Listens to arm joints states for IK
        if self.use_7dof_jaco:
            self.arm_sub = rospy.Subscriber('/j2s7s300/state', JointTrajectoryControllerState, self.armStateCallback, queue_size=1)
        else:
            self.arm_sub = rospy.Subscriber('/vector/right_arm/state', JointTrajectoryControllerState, self.armStateCallback, queue_size=1)
        
        # Initializes member variables to trak the arm joint state
        self.arm_joint_state = None
        self.arm_joint_names = None

        # Setup subscribers to listen to the commands
        self.linact_command_sub = rospy.Subscriber('/vector/linear_actuator_cmd', LinearActuatorCmd, self.linactCallback, queue_size=10)  
        self.pan_sub = rospy.Subscriber('/pan_controller/command', Float64, self.panCallback, queue_size=1)
        self.tilt_sub = rospy.Subscriber('/tilt_controller/command', Float64, self.tiltCallback, queue_size=1)
        self.gripper_sub = rospy.Subscriber('/vector/right_gripper/cmd', GripperCmd, self.gripperCallback, (self.RIGHT_KEY), queue_size=1)

        # Setup some topics that mimic the real robot state topics
        self.lin_state_sub = rospy.Subscriber('/linear_actuator_controller/state', JointTrajectoryControllerState, self.linStateCallback, queue_size=1)
        self.lin_state_pub = rospy.Publisher('/vector/joint_states', JointState, queue_size=1)
        self.gripper_state_sub = rospy.Subscriber('/gripper_controller/state', JointTrajectoryControllerState, self.gripperStateCallback, (self.RIGHT_KEY), queue_size=1)
        self.gripper_joint_state_pub = rospy.Publisher('/vector/right_gripper/joint_states', JointState, queue_size=1)
        self.gripper_stat_pub = rospy.Publisher('/vector/right_gripper/stat', GripperStat, queue_size=1)

        # Initialize necessary components for TF
        self.listener = tf.TransformListener()
        self.trans = None
        self.rot = None

        # Constants for gripper
        # Fully closed = 0
        # Fully open = 0.085
        # The joint is interval is inverted in that 0 joint position = 0.085 gripper
        # Joint maximum is 0.8
        self.grip_max_width = 0.085
        self.grip_joint_max = 0.8
        self.gripper_cmd = dict()
        self.gripper_cmd[self.RIGHT_KEY] = None

        # Initialize components for Trak IK service
        rospy.logwarn("Waiting for trak_ik for 10 seconds...")
        try:
            rospy.wait_for_service('hlpr_trac_ik', timeout=10.0)  # Wait for 10 seconds and assumes we don't want IK
            self.compute_ik = rospy.ServiceProxy('hlpr_trac_ik', IKHandler)
        except rospy.ROSException, e:
            rospy.logwarn("IKHanlder Service was not loaded and arm teleop will not be available")
            self.compute_ik = None 
        else: 
            rospy.logwarn("IKHandler detected")
            self.eef_sub = rospy.Subscriber('/vector/right_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, self.EEFCallback, queue_size=1)

        rospy.loginfo("Done Init")


    def jointTrajHelper(self, joint_names, positions):

        # Setup the joint trajectory
        jtm = JointTrajectory()
        jtp = JointTrajectoryPoint()
        jtm.joint_names = joint_names
        jtp.time_from_start = rospy.Duration(1.0)
        jtp.positions = [positions]
        jtm.points = [jtp] 
       
        return jtm 

    def linactCallback(self, msg):

        # Create the trajectory
        jtm = self.jointTrajHelper(self.linact_joint_names, msg.desired_position_m)

        # Publish the command        
        self.linact_pub.publish(jtm)

    def panCallback(self, msg):

        # Create the trajectory
        jtm = self.jointTrajHelper(self.pan_names, msg.data)

        # Publish the command        
        self.pan_pub.publish(jtm)

    def tiltCallback(self, msg):

        # Create the trajectory
        jtm = self.jointTrajHelper(self.tilt_names, msg.data)

        # Publish the command        
        self.tilt_pub.publish(jtm)

    def convertJointTrajectoryControllerState(self, msg, motor_ids=""):

        # Generate a fake dynamixel msg 
        sim_data_msg = dynamixel_msgs.msg.JointState()
        sim_data_msg.header = msg.header
        sim_data_msg.name = msg.joint_names[0]
        sim_data_msg.motor_ids = motor_ids
        sim_data_msg.motor_temps = []
        sim_data_msg.goal_pos = msg.desired.positions[0]
        sim_data_msg.current_pos = msg.actual.positions[0]
        sim_data_msg.error = msg.error.positions[0]
        sim_data_msg.velocity = msg.actual.velocities[0]
        return sim_data_msg

    def convertJointTrajectorySensorMsg(self, msg):

        # Generate fake JointState msg
        sim_data_msg = JointState()
        sim_data_msg.header = msg.header
        sim_data_msg.name = msg.joint_names
        sim_data_msg.position = msg.actual.positions
        sim_data_msg.velocity = msg.actual.velocities
        sim_data_msg.effort = msg.actual.effort
        return sim_data_msg

    def convertJointState2GripperWidth(self, pos):
        grip_cmd_pos = ((self.grip_joint_max - pos)/self.grip_joint_max) * self.grip_max_width
        return grip_cmd_pos

    def convertJointTrajectoryGripperStat(self, msg, side):

        sim_data_msg = GripperStat()
        sim_data_msg.header = msg.header
        sim_data_msg.position = self.convertJointState2GripperWidth(msg.actual.positions[0])
        desired_pos = self.convertJointState2GripperWidth(msg.desired.positions[0]) 

        # Replace with commanded value
        if not self.gripper_cmd[side] == None:
            desired_pos = self.gripper_cmd[side]

        sim_data_msg.requested_position = desired_pos

        return sim_data_msg
        
    def gripperStateCallback(self, msg, side):
        
        # Publish the joint state message
        self.gripper_joint_state_pub.publish(self.convertJointTrajectorySensorMsg(msg)) 

        # Publish the gripper stat message
        self.gripper_stat_pub.publish(self.convertJointTrajectoryGripperStat(msg, side))

    def panStateCallback(self, msg):
        self.pan_state_pub.publish(self.convertJointTrajectoryControllerState(msg)) 

    def tiltStateCallback(self, msg):
        self.tilt_state_pub.publish(self.convertJointTrajectoryControllerState(msg)) 

    def linStateCallback(self, msg):
        self.lin_state_pub.publish(self.convertJointTrajectorySensorMsg(msg))

    def gripperCallback(self, msg, side):

        # Store the commanded value
        self.gripper_cmd[side] = msg.position

        # Fully closed = 0
        # Fully open = 0.085
        # The joint is interval is inverted in that 0 joint position = 0.085 gripper
        # Joint maximum is 0.8
        grip_joint_pos = ((self.grip_max_width - msg.position)/self.grip_max_width) * self.grip_joint_max

        # Send the position command for now (does not do force)
        jtm = self.jointTrajHelper(self.gripper_name, grip_joint_pos)
       
        # Send the command
        self.gripper_pub.publish(jtm)

    def armStateCallback(self, msg):
        # gets the current arm state in joint space (joint angles)
        self.arm_joint_state = msg.actual.positions
        self.arm_joint_names = msg.joint_names

    def isCommandAllZero(self, msg):
        return (msg.x + msg.y + msg.z + 
                msg.theta_x + msg.theta_y + msg.theta_z) == 0

    def EEFCallback(self, msg):
        # Check if we have EEF positions yet
        if self.trans == None or self.rot == None:
            return

        # Check if the command is just zero - if so do nothing
        if self.isCommandAllZero(msg):
            return

        # Determine current position of EEF
        position = self.trans

        # Convert from Quaternion to RPY in radians
        rotation = tf.transformations.euler_from_quaternion(self.rot, 'rxyz') 
        rotation = [r * (180/math.pi) for r in rotation] # Convert to degrees

        # Convert msg from kinova convention (xyz) to vector convention
        converted_msg = JacoCartesianVelocityCmd()
        converted_msg.x = msg.z
        converted_msg.y = msg.x
        converted_msg.z = msg.y
        converted_msg.theta_x = msg.theta_z
        converted_msg.theta_y = msg.theta_y
        converted_msg.theta_z = msg.theta_x

        # Propogate the position based on velocity
        # assume a small time dt to compute position and rotation
        pose = defaultdict(dict)
        pose['position']['value'] = dict()
        pose['rotation']['value'] = dict()
        pose['position']['keys'] = ['x','y','z']
        pose['rotation']['keys'] = ['theta_x','theta_y','theta_z'] 
        pose['rotation']['speed'] = "0.075" # rotation speed (degrees)
        pose['position']['speed'] = "0.3" # position speed (cm)

        for val in ['position','rotation']:
            for i in xrange(len(pose[val]['keys'])): 
                field = pose[val]['keys'][i]
                pose[val]['value'][field] = eval('converted_msg.'+field+'*'+pose[val]['speed']+' + '+val+'[i]')
     
        # Pull out values from dictionary 
        new_position = pose['position']['value']
        new_rot = pose['rotation']['value']
        for theta in new_rot:
            new_rot[theta] = new_rot[theta] * (math.pi/180.0) # conver to radians

        # Convert into quaternion
        new_rot = tf.transformations.quaternion_from_euler(new_rot['theta_x'], new_rot['theta_y'],new_rot['theta_z'], 'rxyz')

        # Create a Pose and populate 
        eef_pose = Pose()
        eef_pose.position = Point()
        eef_pose.position.x = new_position['x']
        eef_pose.position.y = new_position['y']
        eef_pose.position.z = new_position['z']
        eef_pose.orientation = Quaternion()
        eef_pose.orientation.x = new_rot[0]
        eef_pose.orientation.y = new_rot[1]
        eef_pose.orientation.z = new_rot[2]
        eef_pose.orientation.w = new_rot[3]

        # Convert EEF position into joint positions
        joint_positions, joint_names = self.computeIK(eef_pose)

        if joint_positions is not None:
            # Send to trajectory controller
            # For now send directly to gazebo
            jtm = JointTrajectory()   
            jtm.joint_names = joint_names
            jtp = JointTrajectoryPoint()
            jtp.positions = joint_positions
            jtp.time_from_start = rospy.Duration(1.0)
            jtm.points = [jtp]

            self.arm_pub.publish(jtm) 

 
    def computeIK(self, pose):
        # Create a Trak IK request
        ik_request = IKHandlerRequest() 
        ik_request.goals = [pose]
        ik_request.tolerance = [0.01,0.01,0.01,0.01,0.01,0.01,0.01]
        ik_request.verbose = False

        # Checks for the current arm state in joint space
        if not self.arm_joint_state == None:
            ik_request.origin = self.arm_joint_state
        else:
            rospy.logerror("IK service request failed: Current arm joint state not read")
            return None,None
        
        # Makes IK request
        try:
            request_value = self.compute_ik(ik_request)
            if request_value.success:
                joint_positions = request_value.poses[0].positions
                joint_names = self.arm_joint_names
                return joint_positions,joint_names
            else:
                rospy.logwarn("Teleop Arm: No IK Solution")
                return None,None

        except rospy.ServiceException, e:
            rospy.logerror("IK service request failed: %s", e)
            return None,None
   
    def updateEEF(self):
        # Selects are type (6-DoF or 7-DoF Jaco)
        if self.use_7dof_jaco:
            end_link = 'j2s7s300_ee_link'
        else:
            end_link = 'right_ee_link'
        rate = rospy.Rate(100.0) # Run at 100hz?
        # Continuously cycles and updates the EEF using tf if available
        while not rospy.is_shutdown():
            try:
                (self.trans,self.rot) = self.listener.lookupTransform('/linear_actuator_link', end_link, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            rate.sleep()


if __name__=='__main__':
    rospy.init_node('VectorController2Gazebo')
    rospy.loginfo("Starting up Vector Controller Converter Node")
    vec = VectorControllerConverter()
    vec.updateEEF() 
    #rospy.spin()


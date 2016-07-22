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

Note: it currently depends on MoveIt! to perform arm manipulation using the joystick
"""

import roslib; roslib.load_manifest('hlpr_gazebo')
import rospy
import tf
import math
from collections import defaultdict
from vector_msgs.msg import LinearActuatorCmd, GripperCmd, JacoCartesianVelocityCmd
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
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

        # Specific for the linear actuator
        self.linact_pub = rospy.Publisher('/linear_actuator_controller/command', JointTrajectory, queue_size=10) 
        self.linact_joint_names = get_param('/linear_actuator_controller/joints', '')          

        # Setup pan/tilt controller
        self.pan_pub = rospy.Publisher('/pan_sim_controller/command', JointTrajectory, queue_size=10) 
        self.tilt_pub = rospy.Publisher('/tilt_sim_controller/command', JointTrajectory, queue_size=10) 
        self.pan_names = get_param('/pan_sim_controller/joints', '')          
        self.tilt_names = get_param('/tilt_sim_controller/joints', '')          

        # Setup pan/tilt sim to real controller topics to simulate the real robot
        # Needed because ROS controller has a different message than Stanley
        self.pan_state_sub = rospy.Subscriber('/pan_sim_controller/state', JointTrajectoryControllerState, self.panStateCallback, queue_size=1)
        self.tilt_state_sub = rospy.Subscriber('/tilt_sim_controller/state', JointTrajectoryControllerState, self.tiltStateCallback, queue_size=1)
        self.pan_state_pub = rospy.Publisher('/pan_controller/state', dynamixel_msgs.msg.JointState, queue_size=10) 
        self.tilt_state_pub = rospy.Publisher('/tilt_controller/state', dynamixel_msgs.msg.JointState, queue_size=10) 

        # Setup the gripper controller
        self.gripper_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
        self.gripper_name = get_param('/gripper_controller/joints', '')

        # Setup the arm controller
        self.arm_pub = rospy.Publisher('/vector/right_arm/command', JointTrajectory, queue_size=10)

        # Setup subscribers to listen to the commands
        self.linact_command_sub = rospy.Subscriber('/vector/linear_actuator_cmd', LinearActuatorCmd, self.linactCallback, queue_size=10)  
        self.pan_sub = rospy.Subscriber('/pan_controller/command', Float64, self.panCallback, queue_size=1)
        self.tilt_sub = rospy.Subscriber('/tilt_controller/command', Float64, self.tiltCallback, queue_size=1)
        self.gripper_sub = rospy.Subscriber('/vector/right_gripper/cmd', GripperCmd, self.gripperCallback, queue_size=1)

        # Initialize necessary components for TF
        self.listener = tf.TransformListener()
        self.trans = None
        self.rot = None

        # Initialize components for moveit IK service
        rospy.logwarn("Waiting for MoveIt! for 10 seconds...")
        try:
            rospy.wait_for_service('compute_ik', timeout=10.0)  # Wait for 10 seconds and assumes we don't want IK
            self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        except rospy.ROSException, e:
            rospy.logwarn("MoveIt was not loaded and arm teleop will not be available")
            self.compute_ik = None 
        else: 
            rospy.logwarn("MoveIt detected")
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

    def panStateCallback(self, msg):
        self.pan_state_pub.publish(self.convertJointTrajectoryControllerState(msg)) 

    def tiltStateCallback(self, msg):
        self.tilt_state_pub.publish(self.convertJointTrajectoryControllerState(msg)) 

    def gripperCallback(self, msg):

        # Send the position command for now (does not do force)
        # Scale the command by 10 - 0.08 gripper width = 0.8 joint position
        jtm = self.jointTrajHelper(self.gripper_name, msg.position*10.0)
       
        # Send the command
        self.gripper_pub.publish(jtm) 

    def EEFCallback(self, msg):

        # Check if we have EEF positions yet
        if self.trans == None or self.rot == None:
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
        pose['position']['speed'] = "0.15" # position speed (cm)

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

        # Create a pose to compute IK for
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link' # Hard coded for now
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose 
        
        # Create a moveit ik request
        ik_request = PositionIKRequest() 
        ik_request.group_name = 'arm' # Hard coded for now
        ik_request.pose_stamped = pose_stamped
        ik_request.timeout.secs = 0.1
        ik_request.avoid_collisions = True 
         
        try:
            request_value = self.compute_ik(ik_request)
            if request_value.error_code.val == -31:
                rospy.logwarn("Teleop Arm: No IK Solution")
            if request_value.error_code.val == 1:
                joint_positions = request_value.solution.joint_state.position[1:7] 
                joint_names = request_value.solution.joint_state.name[1:7]
                return joint_positions,joint_names
            else:
                return None,None
 
        except rospy.ServiceException, e:
            print "IK service request failed: %s" % e
            return None,None
   
    def updateEEF(self):

        rate = rospy.Rate(100.0) # Run at 100hz?
        # Continuously cycles and updates the EEF using tf if available
        while not rospy.is_shutdown():
            try:
                (self.trans,self.rot) = self.listener.lookupTransform('/base_link', 'right_ee_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            rate.sleep()


if __name__=='__main__':
    rospy.init_node('VectorController2Gazebo')
    rospy.loginfo("Starting up Vector Controller Converter Node")
    vec = VectorControllerConverter()
    vec.updateEEF() 
    #rospy.spin()


#!/usr/bin/env python
import roslib; roslib.load_manifest('hlpr_gazebo')
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import sys

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class JointTrajectorySetup():
    def __init__(self, controller='/pan_sim_controller', position=False):
        self.controller = controller
        if position == False:
            self.position = map(float,get_param('~init_position', '0.0, 0.0').split(','))
        else:
            self.position = position
        self.goal_pub = rospy.Publisher(controller+'/command', JointTrajectory, queue_size=1)
        self.state_sub = rospy.Subscriber(controller+'/state', JointTrajectoryControllerState, self.state_cb)
        self.joint_names = None

    def state_cb(self, state_msg):
        if self.joint_names is None:
            self.joint_names = state_msg.joint_names

    def send_cmd_msg(self):
        jtm = JointTrajectory()
        jtm.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()

        # Check if we're dealing with one single float value
        if len(self.joint_names) > 1:
            jtp.positions = self.position
        else:
            jtp.positions = [self.position]*len(self.joint_names)
        jtp.time_from_start = rospy.Duration(1.0)
        jtm.points = [jtp]
        return jtm


    def run(self):
        while self.joint_names is None:
            print "Waiting for joint state information from %s/state topic" %self.controller
            rospy.sleep(2)
        print "Received joint state information. Sending %s to default position (rads)" % self.controller
        print self.position

        cmd_msg = self.send_cmd_msg()
        self.goal_pub.publish(cmd_msg)

if __name__=='__main__':
    rospy.init_node('vector_setup')
    controller = 'pan_sim_controller'
    position = False

    if (len(sys.argv) == 4):
        controller = sys.argv[1]

    elif (len(sys.argv) == 3 or len(sys.argv) > 4):
        controller = sys.argv[1]
        position = float(sys.argv[2])


    JTT = JointTrajectorySetup(controller=controller, position=position)
    JTT.run()

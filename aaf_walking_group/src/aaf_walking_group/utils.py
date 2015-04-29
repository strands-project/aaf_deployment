#!/usr/bin/env python

import rospy
import actionlib
import flir_pantilt_d46.msg
from sensor_msgs.msg import JointState
import strands_gazing.msg
from copy import deepcopy
import subprocess


class Gaze():
    def __init__(self):
        self.people_closest_topic = '/upper_body_detector/closest_bounding_box_centre'
        # Gaze client
        rospy.loginfo("Creating gaze client")
        self.gazeClient = actionlib.SimpleActionClient(
            'gaze_at_pose',
            strands_gazing.msg.GazeAtPoseAction
        )
        self.gazeClient.wait_for_server()
        rospy.loginfo("...done")

    def people(self):
        goal = strands_gazing.msg.GazeAtPoseGoal
        goal.topic_name = self.people_closest_topic
        goal.runtime_sec = 0
        self.gazeClient.send_goal(goal)

    def preempt(self):
        self.gazeClient.cancel_all_goals()


class PTU():
    def __init__(self):
        # PTU client
        rospy.loginfo("Creating PTU client")
        self.ptuClient = actionlib.SimpleActionClient(
            'SetPTUState',
            flir_pantilt_d46.msg.PtuGotoAction
        )
        self.ptuClient.wait_for_server()
        rospy.loginfo("...done")

    def turnPTU(self, pan, tilt):
        goal = flir_pantilt_d46.msg.PtuGotoGoal()
        goal.pan = pan
        goal.tilt = tilt
        goal.pan_vel = 60
        goal.tilt_vel = 60
        self.ptuClient.send_goal(goal)


class Head():
    def __init__(self):
        self.pub = rospy.Publisher('/head/commanded_state', JointState)

    def resetHead(self):
        self.head_command = JointState()
        self.head_command.name = ["HeadPan", "HeadTilt", "EyesTilt", "EyesPan", "EyeLids"]
        self.head_command.position = [0, 0, 0, 0, 100]
        self.pub.publish(self.head_command)

    def closeEyes(self):
        self.head_command = JointState()
        self.head_command.name = ["EyeLids"]
        self.head_command.position = [0]
        self.pub.publish(self.head_command)

    def openEyes(self):
        self.head_command = JointState()
        self.head_command.name = ["EyeLids"]
        self.head_command.position = [100]
        self.pub.publish(self.head_command)

class RecoveryReconfigure():
    SET, RESET = range(2)

    def __init__(self, name, whitelist):
        self.name = name
        self.whitelist = whitelist
        self.save_current_configuration()

    def save_current_configuration(self):
        self.save = rospy.get_param(self.name)

    def reconfigure(self, option):
        if option == RecoveryReconfigure.RESET:
            rospy.set_param(self.name, self.save)
        elif option == RecoveryReconfigure.SET:
            to_set = deepcopy(self.save)
            for k in to_set.keys():
                if not k in self.whitelist.keys():
                    to_set[k][0] = False
                else:
                    to_set[k] = self.whitelist[k]
            rospy.set_param(self.name, to_set)

def get_master_volume():
    proc = subprocess.Popen('/usr/bin/amixer sget Master', shell=True, stdout=subprocess.PIPE)
    amixer_stdout = proc.communicate()[0].split('\n')[4]
    proc.wait()

    find_start = amixer_stdout.find('[') + 1
    find_end = amixer_stdout.find('%]', find_start)

    return float(amixer_stdout[find_start:find_end])

def set_master_volume(val):
    val = val if val <= 100.0 else 100.0
    val = val if val >= 0.0 else 0.0
    val = float(int(val))
    proc = subprocess.Popen('/usr/bin/amixer sset Master ' + str(val) + '%', shell=True, stdout=subprocess.PIPE)
    proc.wait()
    subprocess.call(['/usr/bin/canberra-gtk-play','--id','message'])

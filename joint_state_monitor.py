#!/usr/bin/env python
#!/usr/bin/python

import argparse
import rospy
import baxter_interface
import baxter_external_devices
import thread
import threading
import time
import tty
import Queue
import termios, sys, os
from select import select

from baxter_interface import CHECK_VERSION
import traceback
import threading
import Queue

import rospy
import numpy as np

import cv2
import cv_bridge
import rospkg
import std_msgs
from PyQt4 import QtGui, QtCore

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
)
from sensor_msgs.msg import (
    Image,
    JointState,
)

import baxter_dataflow
import baxter_interface

from baxter_core_msgs.msg import (
    AnalogIOStates,
    EndEffectorState,
)
from baxter_core_msgs.srv import (
    ListCameras,
    SolvePositionIK,
    SolvePositionIKRequest,
)

class Window(QtGui.QWidget):
    def __init__(self):
        QtGui.QWidget.__init__(self)
        self.button = QtGui.QPushButton('Test', self)
        self.button.clicked.connect(self.handleButton)
        layout = QtGui.QVBoxLayout(self)
        layout.addWidget(self.button)

    def handleButton(self):
        modifiers = QtGui.QApplication.keyboardModifiers()
        if modifiers == QtCore.Qt.ShiftModifier:
            print('Shift+Click')
        elif modifiers == QtCore.Qt.ControlModifier:
            print('Control+Click')
        else:
            print('Click')

    def keyPressEvent(self, event):
        self.firstrelease = True
        astr = "pressed: " + str(event.key())
        self.keylist.append(astr)

    def keyReleaseEvent(self, event):
        if self.firstrelease == True: 
            self.processmultikeys(self.keylist)

        self.firstrelease = False
        del self.keylist[-1]

    def processmultikeys(self,keyspressed):
        # your logic here
        print keyspressed

def getch(timeout=0.01):
    # If this is being piped to, ignore non-blocking functionality
    if not sys.stdin.isatty():
        print('not in sys.stdin.isatty()')
        return sys.stdin.read(1)
    fileno = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fileno)
    
    ch = None
    
    try:
        tty.setraw(fileno)
        rlist = [fileno]
        if timeout >= 0:
            [rlist, _, _] = select(rlist, [], [], timeout)
        if fileno in rlist:
            ch = sys.stdin.read(1)
    except Exception as ex:
        print "getch", ex
        raise OSError
    finally:
        termios.tcsetattr(fileno, termios.TCSADRAIN, old_settings)
    
    return ch

def record_thread(type, limb_num, queue, timeout=15.0):
    try:
        limbs = [];
        if (limb_num == 0):
            limbs.append(baxter_interface.Limb('left'))
        elif(limb_num == 1):
            limbs.append(baxter_interface.Limb('right'))
        elif(limb_num == 2):
            limbs.append(baxter_interface.Limb('left'))
            limbs.append(baxter_interface.Limb('right'))

        print ("Please enter filename:")

        filename = raw_input()
        if (type == 'T'): #torque option
            print type
        elif (type == 'A'): #angle option
            print type
        elif (type == 'V'): #velocity option
            print type
        elif (type == 'E'): #endpoint angle option
            print type
        elif (type == 'F'): #endpoint torque option
            print type
        elif (type == 'Y'): #endpoint velocity option
            print type

        queue.put(None)
    except Exception, exception:
        queue.put(traceback.format_exc())
        queue.put(exception)

def map_keyboard():
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)

    bindings = {
        #   key: (function, args, description)
        '9': (set_j, [left, lj[0], 0.1], "left_s0 increase"),
        '6': (set_j, [left, lj[0], -0.1], "left_s0 decrease"),
        '8': (set_j, [left, lj[1], 0.1], "left_s1 increase"),
        '7': (set_j, [left, lj[1], -0.1], "left_s1 decrease"),
        'o': (set_j, [left, lj[2], 0.1], "left_e0 increase"),
        'y': (set_j, [left, lj[2], -0.1], "left_e0 decrease"),
        'i': (set_j, [left, lj[3], 0.1], "left_e1 increase"),
        'u': (set_j, [left, lj[3], -0.1], "left_e1 decrease"),
        'l': (set_j, [left, lj[4], 0.1], "left_w0 increase"),
        'h': (set_j, [left, lj[4], -0.1], "left_w0 decrease"),
        'k': (set_j, [left, lj[5], 0.1], "left_w1 increase"),
        'j': (set_j, [left, lj[5], -0.1], "left_w1 decrease"),
        '.': (set_j, [left, lj[6], 0.1], "left_w2 increase"),
        'n': (set_j, [left, lj[6], -0.1], "left_w2 decrease"),
        ',': (grip_left.close, [], "left: gripper close"),
        'm': (grip_left.open, [], "left: gripper open"),
        '/': (grip_left.calibrate, [], "left: gripper calibrate"),

        '4': (set_j, [right, rj[0], 0.1], "right_s0 increase"),
        '1': (set_j, [right, rj[0], -0.1], "right_s0 decrease"),
        '3': (set_j, [right, rj[1], 0.1], "right_s1 increase"),
        '2': (set_j, [right, rj[1], -0.1], "right_s1 decrease"),
        'r': (set_j, [right, rj[2], 0.1], "right_e0 increase"),
        'q': (set_j, [right, rj[2], -0.1], "right_e0 decrease"),
        'e': (set_j, [right, rj[3], 0.1], "right_e1 increase"),
        'w': (set_j, [right, rj[3], -0.1], "right_e1 decrease"),
        'f': (set_j, [right, rj[4], 0.1], "right_w0 increase"),
        'a': (set_j, [right, rj[4], -0.1], "right_w0 decrease"),
        'd': (set_j, [right, rj[5], 0.1], "right_w1 increase"),
        's': (set_j, [right, rj[5], -0.1], "right_w1 decrease"),
        'v': (set_j, [right, rj[6], 0.1], "right_w2 increase"),
        'z': (set_j, [right, rj[6], -0.1], "right_w2 decrease"),
        'c': (grip_right.close, [], "right: gripper close"),
        'x': (grip_right.open, [], "right: gripper open"),
        'b': (grip_right.calibrate, [], "right: gripper calibrate"),
     }
    while not done and not rospy.is_shutdown():
    
        c = getch()
        if c:
            #catch Esc or ctrl-c
            print "c:",c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))


def main():

    right_filename = 'right_joint_state'
    leftt_filename = 'left_joint_state'

    rf = open(right_filename, 'w')
    lf = open(leftt_filename, 'w')

    epilog = ''
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    if (init_state == False):
        print ("Baxter must be enabled")
        return

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    done = False
    print_mode = False

    i = 0;
    start_force = 0;
    max_force = 0;
    while not done and not rospy.is_shutdown():

	print right.joint_angles()
        #rf.write(right.joint_angles())
        #rf.write(right.joint_velocities())
        #rf.write(right.joint_efforts())
        #rf.write(right.endpoint_pose())
        #rf.write(right.endpoint_effort())
        #rf.write(right.endpoint_velocity())

        #lf.write(left.joint_angles())
        #lf.write(left.joint_velocities())
        #lf.write(left.joint_efforts())
        #lf.write(left.endpoint_pose())
        #lf.write(left.endpoint_effort())
        #lf.write(left.endpoint_velocity())

        #print left.endpoint_effort()
        force = right.endpoint_effort()['force'].x
        if (i == 0):
            start_force = force
        else:
            if  (abs(force) > abs(max_force)):
                max_force = force
        if (np.mod(i,100) == 0):
            print force, abs(force), abs(max_force), abs(force) > abs(max_force)
        i = i + 1
        c = getch()
        if c:
            print "c:",c
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                print "start and max",start_force, max_force
                rospy.signal_shutdown("Example finished.")
            elif c == 't' or c == 'T':
                print left.joint_efforts()
                print right.joint_efforts()
            elif c == 'a' or c == 'A':
                print left.joint_angles()
                print right.joint_angles()
            elif c == 'v' or c == 'V':
                print left.joint_velocities()
                print right.joint_velocities()
            elif c == 'e' or c == 'E':
                print left.endpoint_pose()
                print right.endpoint_pose()
            elif c == 'f' or c == 'F':
                print left.endpoint_effort()
                print right.endpoint_effort()
            elif c == 'y' or c == 'Y':
                print left.endpoint_velocity()
                print right.endpoint_velocity()

if __name__ == '__main__':
    main()
    
    #app = QtGui.QApplication(sys.argv)
    #window = Window()
    #window.show()
    #sys.exit(app.exec_())

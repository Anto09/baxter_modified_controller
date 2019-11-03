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

import cv2
import cv_bridge
import rospkg
import std_msgs

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

def getch(timeout=0.01):
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

def main():

    epilog = ''
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("force_based_movement_test")
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

    i = 0
    while not rospy.is_shutdown():
        c = getch()
        if c and c in ['\x1b', '\x03']:
            rospy.signal_shutdown("Example finished.")

        if (i%500 == 0):
            #print 'left',left.endpoint_effort()
            r_f = right.endpoint_effort()['force']
            r_t = right.endpoint_effort()['torque']
            r_f_mag = r_f[0]**2 + r_f[1]**2 + r_f[2]**2
            r_t_mag = r_t[0]**2 + r_t[1]**2 + r_t[2]**2
            print 'right',r_f_mag,r_t_mag
            print right.joint_efforts()

        i+=1

if __name__ == '__main__':
    main()
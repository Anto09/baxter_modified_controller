#!/usr/bin/env python
#!/usr/bin/python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Example: keyboard
Modified by: Antonio Umali, WPI ARC Lab for simulataneous control
"""
import argparse
import rospy
import baxter_interface
import baxter_external_devices
import thread
import threading
import time
import tty
import limb
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

import pygame


def move_thread(limb, angle, queue):
    """
    Threaded joint movement allowing for simultaneous joint moves.
    """
    print angle
    try:
        limb.set_joint_positions(angle)
        queue.put(None)
    except Exception, exception:
        queue.put(traceback.format_exc())
        queue.put(exception)

def main():

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

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    keyboard_input()
    print("Done.")

def keyboard_input():

    done = False
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = baxter_interface.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        baxter_interface.limb.set_joint_positions(joint_command)

    bindings_two = {
        #   key: (limb,index, amount, description)
        '9': (0, 0, 0.1, "left_s0 increase"),
        '6': (0, 0, -0.1, "left_s0 decrease"),
        '8': (0, 1, 0.1, "left_s1 increase"),
        '7': (0, 1, -0.1, "left_s1 decrease"),
        'o': (0, 2, 0.1, "left_e0 increase"),
        'y': (0, 2, -0.1, "left_e0 decrease"),
        'i': (0, 3, 0.1, "left_e1 increase"),
        'u': (0, 3, -0.1, "left_e1 decrease"),
        'l': (0, 4, 0.1, "left_w0 increase"),
        'h': (0, 4, -0.1, "left_w0 decrease"),
        'k': (0, 5, 0.1, "left_w1 increase"),
        'j': (0, 5, -0.1, "left_w1 decrease"),
        '.': (0, 6, 0.1, "left_w2 increase"),
        'n': (0, 6, -0.1, "left_w2 decrease"),
        ',': (grip_left.close, [], "left: gripper close"),
        'm': (grip_left.open, [], "left: gripper open"),
        '/': (grip_left.calibrate, [], "left: gripper calibrate"),

        '4': (1, 0, 0.1, "right_s0 increase"),
        '1': (1, 0, -0.1, "right_s0 decrease"),
        '3': (1, 1, 0.1, "right_s1 increase"),
        '2': (1, 1, -0.1, "right_s1 decrease"),
        'r': (1, 2, 0.1, "right_e0 increase"),
        'q': (1, 2, -0.1, "right_e0 decrease"),
        'e': (1, 3, 0.1, "right_e1 increase"),
        'w': (1, 3, -0.1, "right_e1 decrease"),
        'f': (1, 4, 0.1, "right_w0 increase"),
        'a': (1, 4, -0.1, "right_w0 decrease"),
        'd': (1, 5, 0.1, "right_w1 increase"),
        's': (1, 5, -0.1, "right_w1 decrease"),
        'v': (1, 6, 0.1, "right_w2 increase"),
        'z': (1, 6, -0.1, "right_w2 decrease"),
        'c': (grip_right.close, [], "right: gripper close"),
        'x': (grip_right.open, [], "right: gripper open"),
        'b': (grip_right.calibrate, [], "right: gripper calibrate"),
    }

    while not done and not rospy.is_shutdown():
        print 'Type input'
        c = getkey()
        print 'Input:', c
        joint_moves_left = []
        joint_moves_right = []
        for i in range(0,len(lj)):
            joint_moves_left.append(left.joint_angle(lj[i]))
            joint_moves_right.append(right.joint_angle(rj[i]))
        if c == '\n':
            break
        elif c in ['\x1b', '\x03']:
            done = True
            rospy.signal_shutdown("Example finished.")
        continue
        left_queue = Queue.Queue()
        right_queue = Queue.Queue()

        #print joint_moves_right
        for char in c:
            cmd = bindings_two[char]
            #print 'cmd:',cmd
            if (cmd[0] == 0):
                joint_moves_left[cmd[1]] += cmd[2]
                #left.set_joint_positions(dict(zip(left.joint_names(), joint_moves_left)))
            elif (cmd[0]==1):
                joint_moves_right[cmd[1]] += cmd[2]
                #right.set_joint_positions(dict(zip(right.joint_names(), joint_moves_right)))
        #print joint_moves_right

        
        try:
            left_thread = threading.Thread(
                target=move_thread,
                args=(left,
                      dict(zip(left.joint_names(), joint_moves_left)),
                      left_queue
                      )
            )
            right_thread = threading.Thread(
                target=move_thread,
                args=(right,
                      dict(zip(right.joint_names(), joint_moves_right)),
                      right_queue
                      )
            )
            
            left_thread.daemon = True
            right_thread.daemon = True
            
            left_thread.start()
            right_thread.start()
            
            baxter_dataflow.wait_for(
                lambda: not (left_thread.is_alive() or
                             right_thread.is_alive()),
                timeout=20.0,
                timeout_msg=("Timeout while waiting for arm move threads"
                             " to finish"),
                rate=10,
            )
            
            left_thread.join()
            right_thread.join()
            result = left_queue.get()

            if not result is None:
                raise left_queue.get()
            result = right_queue.get()

            if not result is None:
                raise right_queue.get()
            rospy.sleep(0.001)
        except Exception:
            print 'Failure'
        
        print "loop done"
        
def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~termios.ICANON & ~termios.ECHO
    new[6][termios.VMIN] = 1
    new[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSADRAIN, new)
    c = None
    try:
        tty.setraw(fd)
        #c = os.read(fd, 4) #maximum of 4 simultaneous joint control inputs
        c = sys.stdin.read(1)
    except Exception as ex:
        print "Exception", ex
        raise OSError
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, old)
    return c


if __name__ == '__main__':
    main()


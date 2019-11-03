#!/usr/bin/env python
#!/usr/bin/python

import argparse
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
import roslib
import tf

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
import baxter_pykdl
import baxter_tools
import numpy as np

def move_thread(limb, angle, queue, timeout=15.0):
        """
        Threaded joint movement allowing for simultaneous joint moves.
        """
        try:
            limb.move_to_joint_positions(angle, timeout)
            queue.put(None)
        except Exception, exception:
            queue.put(traceback.format_exc())
            queue.put(exception)

def sampleMove(left, right):
    kin = baxter_pykdl.baxter_kinematics('left')
    left_effort = np.matrix(left.joint_efforts().values())
    left_J = kin.jacobian()
    left_JT = kin.jacobian_transpose()
    left_J_inverse = kin.jacobian_pseudo_inverse()
    left_JT_inverse = kin.jacobian_pseudo_inverse_transpose()

    print 'POSITIONS'
    print kin.forward_position_kinematics()
    print 'JOINT ANGLES'
    print np.matrix(right.joint_angles().values())
    print 'END EFFECTOR POSITION AND ORIENTATION'
    print left.endpoint_pose()
    print 'JACOBIAN'
    print left_J
    print 'JACOBIAN INVERSE'
    print left_J_inverse
    print 'JACOBIAN TRANSPOSE'
    print left_JT
    print 'JACOBIAN TRANSPOSE INVERSE'
    print left_JT_inverse
    print 'JOINT TORQUES'
    print left_effort

    end_effector_force = []
    for row in left_JT_inverse: #equivalent to columns of JT inverse, which is what we want for torque-force relationships
        end_effector_force.append(np.dot(row,np.transpose(left_effort[0])))

    #print 'FORCES'
    print end_effector_force
    #print right.endpoint_effort().values()
    eeForceArray = []
    for elem in left.endpoint_effort().values():
        for comp in elem:
            eeForceArray.append(comp)
    #print 'FORCE ARRAY',np.matrix(eeForceArray)

    joint_torques = []
    for row in left_JT:
        joint_torques.append(np.dot(row,np.transpose(eeForceArray)))
    print 'SOLVED JOINT TORQUES', joint_torques

    set_vals = [0.48668815, -3.14447154, 1.08002909, -0.61671138, -0.03555108, -0.11155558, 0.00262258]

    left.set_joint_torques(dict(zip(left.joint_names(),set_vals)))
    #left.set_joint_torques(left.joint_efforts())

    print '\n'

    left_J = kin.jacobian()
    left_JT = kin.jacobian_transpose()
    left_J_inverse = kin.jacobian_pseudo_inverse()
    left_JT_inverse = kin.jacobian_pseudo_inverse_transpose()


    print 'NEW POSITIONS'
    print kin.forward_position_kinematics()
    print 'NEW JOINT ANGLES'
    print np.matrix(right.joint_angles().values())
    print 'NEW END EFFECTOR POSITION AND ORIENTATION'
    print left.endpoint_pose()
    print 'NEW JACOBIAN'
    print left_J
    print 'NEW JACOBIAN INVERSE'
    print left_J_inverse
    print 'NEW JACOBIAN TRANSPOSE'
    print left_JT
    print 'NEW JACOBIAN TRANSPOSE INVERSE'
    print left_JT_inverse
    print 'NEW JOINT TORQUES'
    print left_effort

    end_effector_force = []
    for row in left_JT_inverse: #equivalent to columns of JT inverse, which is what we want for torque-force relationships
        end_effector_force.append(np.dot(row,np.transpose(left_effort[0])))

    print 'NEW FORCES'
    print end_effector_force
    eeForceArray = []
    for elem in left.endpoint_effort().values():
        for comp in elem:
            eeForceArray.append(comp)
    print 'NEW FORCE ARRAY',np.matrix(eeForceArray)

    joint_torques = []
    for row in left_JT:
        joint_torques.append(np.dot(row,np.transpose(eeForceArray)))
    print 'NEW SOLVED JOINT TORQUES', joint_torques

def move_thread(limb, angle, queue):
    """
    Threaded joint movement allowing for simultaneous joint moves.
    """
    try:
        limb.set_joint_positions(angle)
        queue.put(None)
    except Exception, exception:
        queue.put(traceback.format_exc())
        queue.put(exception)

def gripper_thread(gripper, move):
    try:
        queue.put(None)
    except Exception, exception:
        queue.put(traceback.format_exc())
        queue.put(exception)

def sampleGripper(limb):
    gripper = baxter_interface.Gripper(limb)
    if (gripper.type() == 'suction'):
        print 'SUCTION'
        gripper.command_suction()
        print gripper.vacuum_sensor(), gripper.missed()
    elif (gripper.type() == 'electric'):
        #print gripper.calibrate()
        print gripper.open()
        #print gripper.close()
        print gripper.valid_parameters_text()
        print gripper.valid_parameters()
        #print gripper.command_position(50.0)
        print gripper.missed(), gripper.position(), gripper.force(), gripper.gripping()


def sampleTf():
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)


    br.sendTransform((0.0, 2.0, 0.0),
                     (0.0, 0.0, 0.0, 1.0),
                     rospy.Time.now(),
                     "carrot1",
                     "turtle1")

def obtainTorque():

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    sampleGripper('left')
    return
    
    move = [0.5, -0.8, 2.8, 0.15, 0.0, 1.9, 2.8]
    left_queue = Queue.Queue()

    try:
        #print "move:",move
        #print "Test: Moving to Joint Positions: ",
        #print ", ".join("%.2f" % x for x in move)
        #print dict(zip(left.joint_names(), move))
        left_thread = threading.Thread(
            target=move_thread,
            args=(left,
                  dict(zip(left.joint_names(), move)),
                  left_queue
                  )
        )
        left_thread.daemon = True
        left_thread.start()
        baxter_dataflow.wait_for(
            lambda: not (left_thread.is_alive()),
            timeout=20.0,
            timeout_msg=("Timeout while waiting for arm move threads"
                         " to finish"),
            rate=10)
        left_thread.join()
        result = left_queue.get()
        #print result
        if not result is None:
            raise left_queue.get()
        rospy.sleep(1.0)

    except Exception:
        print 'Failure' 

    print kin.forward_position_kinematics()
    print kin.jacobian_transpose()


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

    obtainTorque()

    print("Done.")

if __name__ == '__main__':
    main()
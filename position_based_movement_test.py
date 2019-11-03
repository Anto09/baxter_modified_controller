#!/usr/bin/env python
#!/usr/bin/python

import argparse
import rospy
import thread
import threading
import time
import traceback
import tty
import Queue
import collections
import cv2
import cv_bridge
import rospkg
import std_msgs
import termios, sys, os
from select import select

import baxter_interface
import baxter_external_devices
import baxter_dataflow
from baxter_interface import CHECK_VERSION

import geometry_msgs

from std_msgs.msg import Header

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

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint

from baxter_core_msgs.msg import (
    AnalogIOStates,
    EndEffectorState,
)
from baxter_core_msgs.srv import (
    ListCameras,
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_pykdl import baxter_kinematics

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def move_thread(limb, angle, queue, write = True, timeout=15.0):
    try:
        limb.move_to_joint_positions(angle, timeout, write)
        queue.put(None)
    except Exception, exception:
        queue.put(traceback.format_exc())
        queue.put(exception)

def GetEndEffectorPos(limb):
    return limb.endpoint_pose()

def MoveToJointPositions(limb, moves, queue, write = True):
    try:
        for move in moves:
            thread = threading.Thread(
                target=move_thread,
                args=(limb,move, queue, write)
            )
            if (move.values()):
                thread.daemon = True
                thread.start()
                baxter_dataflow.wait_for(
                    lambda: not (thread.is_alive()),
                    timeout=20.0,
                    timeout_msg=("Timeout while waiting for %s move thread"
                                 " to finish" % limb.name),
                    rate=10,
                )
                thread.join()
                result = queue.get()
                if not result is None:
                    raise queue.get()
                rospy.sleep(1.0)
    except Exception, exception:
        queue.put(traceback.format_exc())
        queue.put(exception)

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

def ObtainCartesianPos(limb, pos):
    try:
        srv = '/ExternalTools/' + limb.name + '/PositionKinematicsNode/IKService'
        #print "Test: Service availability: %s" % srv
        rospy.wait_for_service(srv, 5.0)
        iksvc = rospy.ServiceProxy(srv, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        pose = PoseStamped(
            header=std_msgs.msg.Header(
            stamp=rospy.Time.now(), frame_id='base'),
            pose=Pose(
                position = pos[0],
                orientation = pos[1]
            )
        )
        ikreq.pose_stamp.append(pose)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))

    if (resp.isValid[0]):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return False

def PrintFunction(left, right):
    i = 0
    cur_time = time.time()
    while not rospy.is_shutdown():
        c = getch()
        if c:
            if c in ['\x1b', '\x03']:
                rospy.signal_shutdown("Example finished.")
            elif c == 'e':
                r_p = right.endpoint_pose()
                l_p = left.endpoint_pose()
                print 'right',r_p
                print 'time', time.time() - cur_time
                print 'left',l_p

def MoveToInitialPos(left, right):
    left_start_angles = [-0.7800292297485352, -0.5296068664123535, -0.243519449798584, 1.5428011756530762,
                        -2.7496605591430665, 2.0095148298339844, 1.7610099424804688]
    right_start_angles = [0.7861651528930664, -0.5710243476379395, 0.2442864401916504, 1.5401167092773438, 
                           2.7580974534667972, 1.9796022045043946,-1.755641009729004]

    '''
    according to:
    self._joint_names = {
            'left': ['left_s0', 'left_s1', 'left_e0', 'left_e1',
                     'left_w0', 'left_w1', 'left_w2'],
            'right': ['right_s0', 'right_s1', 'right_e0', 'right_e1',
                      'right_w0', 'right_w1', 'right_w2']
            }
    }
    '''

    left_queue = Queue.Queue()
    right_queue = Queue.Queue()

    try:
        left_thread = threading.Thread(
            target=move_thread,
            args=(left,
                  dict(zip(left.joint_names(), left_start_angles)),
                  left_queue
                  )
        )
        right_thread = threading.Thread(
            target=move_thread,
            args=(right,
                  dict(zip(right.joint_names(), right_start_angles)),
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
        rospy.sleep(5.0)

    except Exception:
        print 'Failure'

def ThreadMovements(left, right, left_queue, right_queue, left_angles, right_angles):
    try:
        left_thread = threading.Thread(
            target=move_thread,
            args=(left,left_angles,left_queue)
        )
        right_thread = threading.Thread(
            target=move_thread,
            args=(right,right_angles,right_queue)
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

def IKWithSeed(limb, target, seed_angles):
    try:
        srv = '/ExternalTools/' + limb.name + '/PositionKinematicsNode/IKService'
        #print "Test: Service availability: %s" % srv
        rospy.wait_for_service(srv, 5.0)
        iksvc = rospy.ServiceProxy(srv, SolvePositionIK)

        ikreq = SolvePositionIKRequest()

        hdr = Header(seq = 0, stamp=rospy.Time.now(), frame_id='base')
        pose = PoseStamped(
            header=hdr,
            pose=Pose(
                position = target[0],
                orientation = target[1]
            )
        )

        #ikreq.seed_angles = [JointState(header = hdr,
        #                                name = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2'], 
        #                                position = seed_angles)]
        #ikreq.seed_mode = 0
        ikreq.pose_stamp.append(pose)

        #print ikreq

        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))

    if (resp.isValid[0]):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return False


def WaypointMovement(left, right):
      right_waypoints = (
          {'position': Point(x=0.8377640450864112, y=-0.11582423065828584, z=0.4923458650955586), 
           'orientation': Quaternion(x=0.13226666836055648, y=0.24912446543847738, z=0.7655199816521918, w=0.5782920428745492)},
          {'position': Point(x=0.8417443069502412, y=-0.11891448683402965, z=0.49310363781328503), 
           'orientation': Quaternion(x=0.19271115075795434, y=0.21201941305761618, z=0.8836993992373301, w=0.37011559362965346)},
          {'position': Point(x=0.840731485609212, y=-0.1355324000805379, z=0.49206996435617234), 
           'orientation': Quaternion(x=0.248754473835434, y=0.14780500803538485, z=0.9570681654018451, w=0.017187731696260092)},
          {'position': Point(x=0.8473055130584894, y=-0.14468189857022432, z=0.4925367919941549), 
           'orientation': Quaternion(x=0.2924606076954246, y=0.023934196335161563, z=0.8821665197334683, w=-0.36834247468022646)},
          {'position': Point(x=0.8161129521166166, y=-0.14931163702751613, z=0.40302195439947613), 
           'orientation': Quaternion(x=-0.23214268252782502, y=0.009677291479408516, z=-0.7865809015792901, w=0.5721071667521037)},
          {'position': Point(x=0.815765891909716, y=-0.1720765374811499, z=0.36796937823448433), 
           'orientation': Quaternion(x=-0.24874519018671293, y=0.004858072181530482, z=-0.789911732474219, w=0.5604834381078829)},
          {'position': Point(x=0.7670019015722332, y=-0.19590723658453468, z=0.33864541613689475), 
           'orientation': Quaternion(x=-0.25711378939083646, y=-0.014619384754647285, z=-0.7620628895641606, w=0.5940866310931447)},
          {'position': Point(x=0.7340879224275777, y=-0.16928858942083452, z=0.37266208371773457), 
           'orientation': Quaternion(x=-0.2816125105518511, y=0.006937882496422352, z=-0.7086736440314365, w=0.6468600512803014)},
          {'position': Point(x=0.698768015436055, y=-0.21135327950402172, z=0.5050995173912183), 
           'orientation': Quaternion(x=-0.35034180144486377, y=0.10605618547935249, z=-0.6535370331348659, w=0.6624968332024042)},
          {'position': Point(x=0.6946636676306267, y=-0.17043574327656708, z=0.6919001894469808), 
           'orientation': Quaternion(x=-0.3513286280063439, y=0.13464823864637682, z=-0.5817368595873038, w=0.7211243118700894)},
          {'position': Point(x=0.6180997554988862, y=-0.2051381882734812, z=0.8430459129606265), 
           'orientation': Quaternion(x=-0.1777313686112402, y=-0.00066452747654752, z=-0.598096718077658, w=0.7814674880246415)},
          {'position': Point(x=0.5723182491641119, y=-0.16321379474415168, z=0.8602984049144161), 
           'orientation': Quaternion(x=-0.1525795000200966, y=-0.2393503353302891, z=-0.5382789170277538, w=0.7935280213290039)},
          {'position': Point(x=0.591255154823128, y=-0.168294272015567, z=0.8596023289642115), 
           'orientation': Quaternion(x=-0.12400569686253145, y=-0.28006287093223076, z=-0.5332593041427981, w=0.7885568400666964)}
    )

    left_waypoints = (
         {'position': Point(x=0.8303014991452027, y=0.10958267293977095, z=0.4618150075107597), 
          'orientation': Quaternion(x=-0.13056732158295478, y=0.23737067778535645, z=-0.7713864446527938, w=0.5758214036212063)},
         {'position': Point(x=0.8360943368432537, y=0.13032771143609928, z=0.4597747597087926), 
          'orientation': Quaternion(x=0.1970585322380517, y=-0.1906565720325663, z=0.8978826029989767, w=-0.3444195662919829)},
         {'position': Point(x=0.8343417140099081, y=0.13195840639235246, z=0.44909317715990604), 
          'orientation': Quaternion(x=0.24793848387568648, y=-0.13038124979899385, z=0.9598861077170286, w=-0.012078829667287846)},
         {'position': Point(x=0.8403985663974468, y=0.13856360047761537, z=0.4489607867243851), 
          'orientation': Quaternion(x=0.28646866455444847, y=0.0015433612749044233, z=0.8694973102524979, w=0.4023776208091566)},
         {'position': Point(x=0.8005423040065784, y=0.15076830590128623, z=0.34667613333289926), 
          'orientation': Quaternion(x=0.22281271353007828, y=0.02120250430781421, z=0.7881979277358514, w=0.5732791424893724)},
         {'position': Point(x=0.794886462157506, y=0.16097195728292196, z=0.3200114207909129), 
          'orientation': Quaternion(x=0.23694599119415338, y=0.016863132674314125, z=0.7850741186736484, w=0.572040960248672)},
         {'position': Point(x=0.7480122083156326, y=0.19587597452461614, z=0.3150269244610346), 
          'orientation': Quaternion(x=0.22823098028996608, y=0.009732745590434213, z=0.7625921954439023, w=0.605201649656709)},
         {'position': Point(x=0.7124907252266404, y=0.1915478798411454, z=0.3842717926656125), 
          'orientation': Quaternion(x=0.2553837365069832, y=0.025239497749872563, z=0.7141228633210988, w=0.6512838482283598)},
         {'position': Point(x=0.6826868081790027, y=0.19910758867189324, z=0.5091675605347953), 
          'orientation': Quaternion(x=0.3320007298254199, y=0.10841833267513816, z=0.6408780772399449, w=0.6835907186676563)},
         {'position': Point(x=0.6944262408338714, y=0.16051125604772684, z=0.7139270369292335), 
          'orientation': Quaternion(x=0.30636137214303927, y=0.1154735180016041, z=0.5731122927855326, w=0.7512329040697548)},
         {'position': Point(x=0.594905394744783, y=0.1960222465776685, z=0.8538914058392645), 
          'orientation': Quaternion(x=0.10288793860333992, y=-0.07366289860628404, z=0.5733456786455119, w=0.80948291040478)},
         {'position': Point(x=0.5857400942894155, y=0.16429567620918273, z=0.8275134157410283), 
          'orientation': Quaternion(x=0.1030785682607772, y=-0.2925131598974558, z=0.4955458966371058, w=0.8113230702859624)},
         {'position': Point(x=0.5990926952184676, y=0.17690387915465824, z=0.8240704203723417), 
          'orientation': Quaternion(x=0.09389157660653147, y=-0.2970183695170658, z=0.5063954105529815, w=0.8040697408699328)}
    )

    times = (4.59733510017, 12.2072281837, 13.2064771652, 15.7105820179, 18.0389580727, 19.1579041481, 21.2389450073, 22.2226121426,
             23.2304651737, 24.5033130646, 25.6616361141, 26.4380722046, 28.1029679775)

    for i in range(0,len(times)):
        continue
        r_w = right_waypoints[i]
        l_w = left_waypoints[i]
        c_t = times[i] - default_time

        right_point = Point(r_w['position'].x, r_w['position'].y, r_w['position'].z)
        right_orient = Quaternion(r_w['orientation'].x, r_w['orientation'].y, r_w['orientation'].z, r_w['orientation'].z)
        right_pose = [right_point, right_orient]
        right_angles = ObtainCartesianPos(right, right_pose)

        left_point = Point(l_w['position'].x, l_w['position'].y, l_w['position'].z)
        left_orient = Quaternion(l_w['orientation'].x, l_w['orientation'].y, l_w['orientation'].z, l_w['orientation'].z)
        left_pose = [left_point, left_orient]
        left_angles = ObtainCartesianPos(left, left_pose)

        right_queue = Queue.Queue()
        left_queue = Queue.Queue()

        ThreadMovements(left, right, left_queue, right_queue, left_angles, right_angles)

    target = [Point(x=0.657579481614, y=0.851981417433, z=0.0388352386502), 
              Quaternion(x=-0.366894936773, y=0.885980397775, z=0.108155782462, w=0.262162481772)]

    print 'CUR POS', left.endpoint_pose()['position'], left.endpoint_pose()['orientation']
    print 'TARGET', target

    #['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'],
    seed_angles = [0.4371845240478516, 1.8419274289489747, 0.4981602602966309, -1.3483691110107423, -0.11850001572875977, 1.18768462366333, -0.002300971179199219]
    sol = IKWithSeed(left, target, seed_angles)

    left_queue = Queue.Queue()
    
    print 'before',left.joint_angles()

    try:
        left_thread = threading.Thread(
            target=move_thread,
            args=(left,sol,left_queue, False)
        )
        
        left_thread.daemon = True
        
        left_thread.start()
        
        baxter_dataflow.wait_for(
            lambda: not (left_thread.is_alive()),
            timeout=20.0,
            timeout_msg=("Timeout while waiting for arm move threads"
                         " to finish"),
            rate=10,
        )
        
        left_thread.join()

        result = left_queue.get()
        if not result is None:
            raise left_queue.get()
        rospy.sleep(5.0)

    except Exception:
        print 'Failure'

    print 'second',left.joint_angles()

    left_queue = Queue.Queue()
    try:
        left_thread = threading.Thread(
            target=move_thread,
            args=(left,dict(zip(left.joint_names(), seed_angles)),left_queue, False)
        )
        
        left_thread.daemon = True
        
        left_thread.start()
        
        baxter_dataflow.wait_for(
            lambda: not (left_thread.is_alive()),
            timeout=20.0,
            timeout_msg=("Timeout while waiting for arm move threads"
                         " to finish"),
            rate=10,
        )
        
        left_thread.join()

        result = left_queue.get()
        if not result is None:
            raise left_queue.get()
        rospy.sleep(5.0)

    except Exception:
        print 'Failure'

    print 'last',left.joint_angles()

def main():

    epilog = ''
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")

    rospy.init_node("position_based_movement_test", anonymous=True)
    #rospy.Subscriber("chatter", String, callback)

    '''
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    if (init_state == False):
        print ("Baxter must be enabled")
        return
    '''

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #MoveToInitialPos(left, right)

    default_time = 4.59733510017

  

if __name__ == '__main__':
    main()
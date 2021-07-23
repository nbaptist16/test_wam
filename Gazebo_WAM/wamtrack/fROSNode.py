#!/usr/bin/env python3

'''
combines IK with Jacobian.py with original exampleROSNode.py to command wam
    - given some random initial configuration, find goal pose via ik
'''

# added for ros connection
# from roboticstoolbox.mobile import animations
import rospy
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# end

from roboticstoolbox import *
import numpy as np
np.set_printoptions(linewidth=np.inf, precision=4)

""" Defines the robot arm using DH parameters """
robot = DHRobot([                                                                                                                       # wam frame parameters (7)
RevoluteDH(             alpha=-np.pi/2,                 I=[0.13488033, 0.11328369, 0.09046330, -0.00213041, 0.00068555 , -0.00012485],  qlim=[-2.60, 2.60]),    # i = 1
RevoluteDH(             alpha=np.pi/2,                  I=[0.02140958, 0.01377875, 0.01558906, 0.00027172, -0.00181920, 0.00002461],    qlim=[-1.97, 1.97]),    # i = 2
RevoluteDH(a=0.045,     alpha=-np.pi/2,     d=0.55,     I=[0.05911077, 0.00324550, 0.05927043, -0.00249612, -0.00001767, 0.00000738],   qlim=[-2.80, 2.80]),    # i = 3
RevoluteDH(a=-0.045,    alpha=np.pi/2,                  I=[0.01491672, 0.01482922, 0.00294463, 0.00001741, -0.00002109, -0.00150604],   qlim=[-0.90, 3.10]),    # i = 4
RevoluteDH(             alpha=-np.pi/2,     d=0.3,      I=[0.00005029, 0.00007582, 0.00006270, 0.00000020, -0.00000359, -0.00000005],   qlim=[-4.76, 1.24]),    # i = 5
RevoluteDH(             alpha=np.pi/2,                  I=[0.00055516, 0.00024367, 0.00045358, 0.00000061, -0.00004590, -0.00000074],   qlim=[-1.50, 1.50]),    # i = 6
RevoluteDH(d=0.060,                                     I=[0.00003773, 0.00003806, 0.00007408, -0.00000019, 0, 0],                      qlim=[-2.20, 2.20]),    # i = 7
], name="WAM")
# inertia tensor of link with respect to center of mass:
# I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
# above from: https://github.com/petercorke/robotics-toolbox-python/blob/71183f3221cf9ced69420f07ade06f4514c256ba/roboticstoolbox/models/DH/README.md
# values from: http://www.me.unm.edu/~starr/research/WAM_MassParams_AA-00.pdf

# """ start at random configuration (ignoring collision) """
# np.random.seed(0)
# # angles_start = np.random.uniform(0, 2*np.pi, (7,))
# angles_start = np.zeros(7) # to start from upright position
# # angles_start = np.ones(7) # to start from upright position
# # print("angles_start", angles_start) #added for checking
# T_start = robot.fkine(angles_start)

# """ find reachable desired end configuation for joints """
# # angles_end_hidden = np.random.uniform(0, 2*np.pi, (7,)) # this is not actually known
# angles_end_hidden = np.ones(7)
# # angles_end_hidden = np.zeros(7)
# # print("angles_end_hidden", angles_end_hidden) #added for checking
# T_end = robot.fkine(angles_end_hidden)

""" iterate joints to find path """
from scipy.spatial.transform import Slerp, Rotation # use this for linear interpolation of rotation


# TODO: make class to incorporate subscriber for goal input
def callback(data):
    rospy.loginfo("I heard %s",data.data)
    
def listener():
    rospy.init_node('node_name')
    rospy.Subscriber("chatter", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



def talker():
    hold = False

    rospy.init_node('trajectory_giver')
    pub = rospy.Publisher('joint_traj', JointTrajectory, queue_size=10)
    # sub = rospy.Subscriber()

    """ start at random configuration (ignoring collision) """
    np.random.seed(0)
    # angles_start = np.random.uniform(0, 2*np.pi, (7,))
    angles_start = np.zeros(7) # to start from upright position
    # print("angles_start", angles_start) #added for checking
    T_start = robot.fkine(angles_start)

    """ find reachable desired end configuation for joints """
    # angles_end_hidden = np.random.uniform(0, 2*np.pi, (7,)) # this is not actually known
    angles_end_hidden = np.ones(7) # placeholder end pose (make subscriber to topic)
    # print("angles_end_hidden", angles_end_hidden) #added for checking
    T_end = robot.fkine(angles_end_hidden)

    while not rospy.is_shutdown():
        gain = 0.01
        T_current = T_start.copy()
        q_current = angles_start

        if(hold != True):
            for i in range(10):
            # for i in range(100):
                # find desired omega
                q1 = Rotation.from_matrix(T_current.R)
                q2 = Rotation.from_matrix(T_end.R)
                aa_diff = (q1.inv() * q2).as_rotvec()
                omega = aa_diff*gain

                # find desired velocity
                x1 = T_current.t
                x2 = T_end.t
                velocity = (x2 - x1)*gain

                # find a solution to [v, w]' = J dq/dt
                J = robot.jacob0(angles_end_hidden)
                qdot = np.linalg.pinv(J) @ np.hstack([omega, velocity])
                # qdot = np.zeros((7,))
                q_new = q_current + qdot

                # update
                q_current = q_new.copy()
                T_current = robot.fkine(q_current)

                corz = robot.coriolis(q_current, qdot)
                # print("corz", corz)
                print("q_current", q_current)


                # # Publish on ROS topic "joint_traj"
                # pub = rospy.Publisher('joint_traj', JointTrajectory, queue_size=10)
                # rospy.init_node('trajectory_giver')


                # Seven joints on a Barrettt WAM Arm.
                number_of_joints = 7
                all_sets_of_joint_coordinates = JointTrajectory()
                # Each joint coordinate/joint point consists of desired positions for each 
                # joint and a time for when the joints should assume their positions.
                one_set_of_joint_coordinates = JointTrajectoryPoint()
                all_sets_of_joint_coordinates.header.stamp = rospy.Time.now()
                # Create  J1 through J7
                for joint_number in range (1, number_of_joints + 1):  
                    all_sets_of_joint_coordinates.joint_names.append("J%i"%joint_number)
                # Usually you'd be feeding the joint trajectory in from somewhere intelligent. Here we create a hundred points.
                # Each point is going to move all 7 joints an additional .01 degrees.
                # Each point takes place .01 seconds after the other.
                number_of_boring_points = 100
                # initial starting position for all joints
                joint_coord = .1
                # time at which to take initial positions
                time_for_move = .01

                for joint_number in range (1, number_of_joints + 1):
                # for one point, set all desired joint positions equal to joint_coord
                    one_set_of_joint_coordinates.positions.append(q_current[joint_number-1])
                # And set the time for the desired joint positions
                one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(time_for_move)
                # append that joint coordinate to the list of joint coordinates
                all_sets_of_joint_coordinates.points.append(one_set_of_joint_coordinates)
                # Reset the holding variable for one joint coordinate
                one_set_of_joint_coordinates = None
                one_set_of_joint_coordinates = JointTrajectoryPoint()
                # Increment the desired position.
                joint_coord = joint_coord + .01
                # Increment the time.
                time_for_move += .01
                
                str = "Published a joint trajectory at time %s" % rospy.get_time()
                #Send a comment to the terminal stating the time the trajectory was published
                rospy.loginfo(str)
                #Publish the set of joint coordinates
                pub.publish(all_sets_of_joint_coordinates)
                print("pubd: ", q_current)
                #Wait.
                # rospy.sleep(1.0)
                rospy.sleep(0.05)

                hold = True
        else:
           pub.publish(all_sets_of_joint_coordinates) 
           print("pubd: ", q_current)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

'''
combines IK with Jacobian.py with original exampleROSNode.py to command wam
    - given some random initial configuration, find goal pose via ik
'''

# added for ros connection
import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# end

from roboticstoolbox import *
import numpy as np
np.set_printoptions(linewidth=np.inf, precision=4)

""" Defines the robot arm using DH parameters """
# robot = DHRobot([
# RevoluteDH(alpha=-np.pi/2),
# RevoluteDH(alpha=np.pi/2),
# RevoluteDH(a=0.045, alpha=-np.pi/2, d=0.55),
# RevoluteDH(a=-0.045, alpha=np.pi/2),
# RevoluteDH(alpha=-np.pi/2, d=0.3),
# RevoluteDH(alpha=np.pi/2),
# RevoluteDH(d=0.060),
# ], name="WAM")
# robot = DHRobot([                                         # wam frame parameters (7)
# RevoluteDH(alpha=-np.pi/2, I=[0, 0, 0, 0, 0, 0]),                   # i = 1
# RevoluteDH(alpha=np.pi/2, I=[0, 0, 0, 0, 0, 0]),                    # i = 2
# RevoluteDH(a=0.045, alpha=-np.pi/2, d=0.55, I=[0, 0, 0, 0, 0, 0]),  # i = 3
# RevoluteDH(a=-0.045, alpha=np.pi/2, I=[0, 0, 0, 0, 0, 0]),          # i = 4
# RevoluteDH(alpha=-np.pi/2, d=0.3, I=[0, 0, 0, 0, 0, 0]),            # i = 5
# RevoluteDH(alpha=np.pi/2, I=[0, 0, 0, 0, 0, 0]),                    # i = 6
# RevoluteDH(d=0.060, I=[0, 0, 0, 0, 0, 0]),                          # i = 7
# ], name="WAM")
robot = DHRobot([                                         # wam frame parameters (7)
RevoluteDH(alpha=-np.pi/2, I=[0.13488033, 0.11328369, 0.09046330, -0.00213041, 0.00068555 , -0.00012485]),                   # i = 1
RevoluteDH(alpha=np.pi/2, I=[0.02140958, 0.01377875, 0.01558906, 0.00027172, -0.00181920, 0.00002461]),                    # i = 2
RevoluteDH(a=0.045, alpha=-np.pi/2, d=0.55, I=[0.05911077, 0.00324550, 0.05927043, -0.00249612, -0.00001767, 0.00000738]),  # i = 3
RevoluteDH(a=-0.045, alpha=np.pi/2, I=[0.01491672, 0.01482922, 0.00294463, 0.00001741, -0.00002109, -0.00150604]),          # i = 4
RevoluteDH(alpha=-np.pi/2, d=0.3, I=[0.00005029, 0.00007582, 0.00006270, 0.00000020, -0.00000359, -0.00000005]),            # i = 5
RevoluteDH(alpha=np.pi/2, I=[0.00055516, 0.00024367, 0.00045358, 0.00000061, -0.00004590, -0.00000074]),                    # i = 6
RevoluteDH(d=0.060, I=[0.00003773, 0.00003806, 0.00007408, -0.00000019, 0, 0]),                          # i = 7
], name="WAM")
# inertia tensor of link with respect to center of mass:
# I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
# above from: https://github.com/petercorke/robotics-toolbox-python/blob/71183f3221cf9ced69420f07ade06f4514c256ba/roboticstoolbox/models/DH/README.md
# values from: http://www.me.unm.edu/~starr/research/WAM_MassParams_AA-00.pdf

""" start at random configuration (ignoring collision) """
np.random.seed(0)
# angles_start = np.random.uniform(0, 2*np.pi, (7,))
angles_start = np.zeros(7) # to start from upright position
print("angles_start", angles_start) #added for checking
T_start = robot.fkine(angles_start)

""" find reachable desired end configuation for joints """
# angles_end_hidden = np.random.uniform(0, 2*np.pi, (7,)) # this is not actually known
angles_end_hidden = np.ones(7)
print("angles_end_hidden", angles_end_hidden) #added for checking
T_end = robot.fkine(angles_end_hidden)

""" iterate joints to find path """
from scipy.spatial.transform import Slerp, Rotation # use this for linear interpolation of rotation

gain = 0.01
T_current = T_start.copy()
q_current = angles_start
for i in range(10):
    # print(q_current)
    # print(T_end - T_current)
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
    print("corz", corz)
    print("q_current", q_current)
    print("qdot", qdot)

print(T_start)
print(T_current)
print(T_end)

# corz = robot.coriolis(q_current, qdot)
# print(corz)


# added for ros connection
print(q_current)

def talker():
    # Publish on ROS topic "joint_traj"
    # pub = rospy.Publisher('joint_traj', JointTrajectory)
    pub = rospy.Publisher('joint_traj', JointTrajectory, queue_size=10)
    # This node is called trajectory_giver
    rospy.init_node('trajectory_giver')
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
    # for i in range (0, number_of_boring_points): 
    #    for joint_number in range (1, number_of_joints + 1):
	#   # for one point, set all desired joint positions equal to joint_coord
    #       one_set_of_joint_coordinates.positions.append(joint_coord)
    #    # And set the time for the desired joint positions
    #    one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(time_for_move)
    #    # append that joint coordinate to the list of joint coordinates
    #    all_sets_of_joint_coordinates.points.append(one_set_of_joint_coordinates)
    #    # Reset the holding variable for one joint coordinate
    #    one_set_of_joint_coordinates = None
    #    one_set_of_joint_coordinates = JointTrajectoryPoint()
    #    # Increment the desired position.
    #    joint_coord = joint_coord + .01
    #    # Increment the time.
    #    time_for_move += .01
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

    while not rospy.is_shutdown():
        str = "Published a joint trajectory as time %s" % rospy.get_time()
	#Send a comment to the terminal stating the time the trajectory was published
        rospy.loginfo(str)
        #Publish the set of joint coordinates
        pub.publish(all_sets_of_joint_coordinates)
        #Wait.
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
# end
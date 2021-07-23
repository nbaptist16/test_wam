#!/usr/bin/env python3

'''
combines IK with Jacobian.py with original exampleROSNode.py to command wam
    - given some random initial configuration, find goal pose via ik
'''

# added for ros connection
# from roboticstoolbox.mobile import animations
import rospy
from std_msgs.msg import Float64
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

""" start at random configuration (ignoring collision) """
np.random.seed(0)
# angles_start = np.random.uniform(0, 2*np.pi, (7,))
angles_start = np.zeros(7) # to start from upright position
# print("angles_start", angles_start) #added for checking
T_start = robot.fkine(angles_start)

""" find reachable desired end configuation for joints """
# angles_end_hidden = np.random.uniform(0, 2*np.pi, (7,)) # this is not actually known
angles_end_hidden = np.ones(7)
# print("angles_end_hidden", angles_end_hidden) #added for checking
T_end = robot.fkine(angles_end_hidden)

""" iterate joints to find path """
from scipy.spatial.transform import Slerp, Rotation # use this for linear interpolation of rotation

# gain = 0.01
# T_current = T_start.copy()
# q_current = angles_start
# for i in range(10):
#     # print(q_current)
#     # print(T_end - T_current)
#     # find desired omega
#     q1 = Rotation.from_matrix(T_current.R)
#     q2 = Rotation.from_matrix(T_end.R)
#     aa_diff = (q1.inv() * q2).as_rotvec()
#     omega = aa_diff*gain

#     # find desired velocity
#     x1 = T_current.t
#     x2 = T_end.t
#     velocity = (x2 - x1)*gain

#     # find a solution to [v, w]' = J dq/dt
#     J = robot.jacob0(angles_end_hidden)
#     qdot = np.linalg.pinv(J) @ np.hstack([omega, velocity])
#     # qdot = np.zeros((7,))
#     q_new = q_current + qdot

#     # update
#     q_current = q_new.copy()
#     T_current = robot.fkine(q_current)

#     corz = robot.coriolis(q_current, qdot)
#     # print("corz", corz)
#     print("q_current", q_current)
#     # print("qdot", qdot)

# print(T_start)
# print(T_current)
# print(T_end)

# corz = robot.coriolis(q_current, qdot)
# print(corz)


# added for ros connection
# print(q_current)

# def talker():
#     # Publish on ROS topic "joint_traj"
#     # pub = rospy.Publisher('joint_traj', JointTrajectory)
#     pub = rospy.Publisher('joint_traj', JointTrajectory, queue_size=10)
#     # This node is called trajectory_giver
#     rospy.init_node('trajectory_giver')
#     # Seven joints on a Barrettt WAM Arm.
#     number_of_joints = 7   
#     all_sets_of_joint_coordinates = JointTrajectory()
#     # Each joint coordinate/joint point consists of desired positions for each 
#     # joint and a time for when the joints should assume their positions.
#     one_set_of_joint_coordinates = JointTrajectoryPoint()
#     all_sets_of_joint_coordinates.header.stamp = rospy.Time.now()
#     # Create  J1 through J7
#     for joint_number in range (1, number_of_joints + 1):  
#        all_sets_of_joint_coordinates.joint_names.append("J%i"%joint_number)
#     # Usually you'd be feeding the joint trajectory in from somewhere intelligent. Here we create a hundred points.
#     # Each point is going to move all 7 joints an additional .01 degrees.
#     # Each point takes place .01 seconds after the other.
#     number_of_boring_points = 100
#     # initial starting position for all joints
#     joint_coord = .1
#     # time at which to take initial positions
#     time_for_move = .01

#     for joint_number in range (1, number_of_joints + 1):
#     # for one point, set all desired joint positions equal to joint_coord
#         one_set_of_joint_coordinates.positions.append(q_current[joint_number-1])
#     # And set the time for the desired joint positions
#     one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(time_for_move)
#     # append that joint coordinate to the list of joint coordinates
#     all_sets_of_joint_coordinates.points.append(one_set_of_joint_coordinates)
#     # Reset the holding variable for one joint coordinate
#     one_set_of_joint_coordinates = None
#     one_set_of_joint_coordinates = JointTrajectoryPoint()
#     # Increment the desired position.
#     joint_coord = joint_coord + .01
#     # Increment the time.
#     time_for_move += .01

#     while not rospy.is_shutdown():
#         str = "Published a joint trajectory as time %s" % rospy.get_time()
# 	#Send a comment to the terminal stating the time the trajectory was published
#         rospy.loginfo(str)
#         #Publish the set of joint coordinates
#         pub.publish(all_sets_of_joint_coordinates)
#         print("pubd: ", q_current)
#         #Wait.
#         rospy.sleep(1.0)


def talker():


    while not rospy.is_shutdown():
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
            # print("corz", corz)
            print("q_current", q_current)
            # print("qdot", qdot)


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
            
            str = "Published a joint trajectory as time %s" % rospy.get_time()
        #Send a comment to the terminal stating the time the trajectory was published
            rospy.loginfo(str)
            #Publish the set of joint coordinates
            pub.publish(all_sets_of_joint_coordinates)
            # pub.publish(one_set_of_joint_coordinates)
            print("pubd: ", q_current)
            #Wait.
            rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
# end

'''""" iterate joints to find path """
from scipy.spatial.transform import Slerp, Rotation # use this for linear interpolation of rotation

class Controlla():
    def __init__(self):
        
        # initial definition of variables
        
        rospy.init_node('trajectory_giver')        

        # init vars
        self.pub = rospy.Publisher('joint_traj', JointTrajectory, queue_size=10)
        self.pubd = rospy.Publisher('joint_traj', JointTrajectory, queue_size=10)
        self.now = rospy.Time.now().secs + rospy.Time.now().nsecs/1000000000

        self.number_of_joints = 7
        self.all_sets_of_joint_coordinates = JointTrajectory()
        self.one_set_of_joint_coordinates = JointTrajectoryPoint()
        self.joint_coords_cp = JointTrajectory()

        self.all_sets_of_joint_coordinates.header.stamp = rospy.Time.now()
        self.joint_coords_cp.header.stamp = rospy.Time.now()

        for joint_number in range(1, self.number_of_joints + 1):
            self.all_sets_of_joint_coordinates.joint_names.append("J%i"%joint_number)
        for joint_number in range(1, self.number_of_joints + 1):
            self.joint_coords_cp.joint_names.append("J%i"%joint_number)

        self.q_current = 0
    
    def talker(self):
        # moved from above
        """ start at random configuration (ignoring collision) """
        np.random.seed(0)
        # angles_start = np.random.uniform(0, 2*np.pi, (7,))
        angles_start = np.zeros(7) # to start from upright position
        # print("angles_start", angles_start) #added for checking
        T_start = robot.fkine(angles_start)

        """ find reachable desired end configuation for joints """
        # angles_end_hidden = np.random.uniform(0, 2*np.pi, (7,)) # this is not actually known
        angles_end_hidden = np.ones(7)
        # print("angles_end_hidden", angles_end_hidden) #added for checking
        T_end = robot.fkine(angles_end_hidden)

        gain = 0.01
        T_current = T_start.copy()
        self.q_current = angles_start
        for i in range(10):
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
            q_new = self.q_current + qdot

            # update
            self.q_current = q_new.copy()
            # print("updated", q_current)
            T_current = robot.fkine(self.q_current)

            for joint_number in range (1, self.number_of_joints + 1):
                # for one point, set all desired joint positions equal to joint_coord
                self.one_set_of_joint_coordinates.positions.append(self.q_current[joint_number-1])
            print("setting", self.q_current)
            # print("one_set: ", self.one_set_of_joint_coordinates)
            
            # And set the time for the desired joint positions
            # one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(time_for_move)
            self.one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(self.now)
            
            # append that joint coordinate to the list of joint coordinates
            self.all_sets_of_joint_coordinates.points.append(self.one_set_of_joint_coordinates)
            self.pub.publish(self.all_sets_of_joint_coordinates) # moved up from rospy loop
            # self.pubd.publish(self.one_set_of_joint_coordinates)

            # self.pub.publish(self.one_set_of_joint_coordinates)
            self.joint_coords_cp.points.append(self.one_set_of_joint_coordinates)
            self.pubd.publish(self.joint_coords_cp)
            print("pubd", self.all_sets_of_joint_coordinates)
            
            # Reset the holding variable for one joint coordinate
            self.one_set_of_joint_coordinates = None
            self.one_set_of_joint_coordinates = JointTrajectoryPoint()
        # end
    
    # def pubvel(self, vee, ome):
    def pubvel(self):
        
        # publishing function (to keep main bit clean, sorta)
        #     arguments: vee -- linear velocity (float) in +x-direction of the robot (m/s)
        #                ome -- angular velocity (float) of the robot (rad/s)
        #     returns: none
        #     exceptions: none
        
        # rate = rospy.Rate(self.freq)
        rate = rospy.Rate(100)
        # trajectory = Twist(Vector3(vee, 0, 0), Vector3(0, 0, ome))
        # self.pubc.publish(trajectory)
        print("pubd", self.one_set_of_joint_coordinates)
        self.pub.publish(self.all_sets_of_joint_coordinates)
        rate.sleep()


def calc_ellipse(widthin, heightin):
    # to calculate ellipse trajectory
    # x = xin
    # y = yin
    a = widthin/2
    b = heightin/2

    # (x^2/a^2) + (y^2/b^2) = 1
    # x,y = function of time
    # start at top center of ellipse (0, +y)

    # xstart = 0
    # ystart = a


if __name__ == '__main__':
    
    controlz = Controlla()

    # moved down    
    while not rospy.is_shutdown():
        controlz.talker()
        # controlz.pubvel()
        print("rosloop:", controlz.q_current)
        str = "Published a joint trajectory as time %s" % rospy.get_time()
        rospy.loginfo(str)
        # pub.publish(all_sets_of_joint_coordinates)
        # rospy.sleep(1.0)
        rospy.spin()
# end'''
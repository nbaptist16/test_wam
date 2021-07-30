#!/usr/bin/env python3

'''
combines IK with Jacobian.py with original exampleROSNode.py to command wam
    - given some random initial configuration, find goal pose via ik
'''

# added for ros connection
# from roboticstoolbox.mobile import animations
import math
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from spatialmath import SE3
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# end

from roboticstoolbox import *
import numpy as np
np.set_printoptions(linewidth=np.inf, precision=4)

""" Defines the robot arm using DH parameters """
robot = DHRobot([
# PrismaticDH(alpha=np.pi/2, qlim=[-0.8638, 0.8638]),  # added for track, but no likey --> TODO: if trajectory x-pos > range of robot, then adjust track position
                                                                                                                                           # wam frame parameters (7)
RevoluteDH(          alpha=-np.pi/2,         I=[0.13488033, 0.11328369, 0.09046330, -0.00213041, 0.00068555, -0.00012485],  qlim=[-2.60, 2.60], m = 8.393600),  # i = 1
RevoluteDH(          alpha=np.pi/2,          I=[0.02140958, 0.01377875, 0.01558906, 0.00027172, -0.00181920, 0.00002461],   qlim=[-1.97, 1.97], m = 4.848700),  # i = 2
RevoluteDH(a=0.045,  alpha=-np.pi/2, d=0.55, I=[0.05911077, 0.00324550, 0.05927043, -0.00249612, -0.00001767, 0.00000738],  qlim=[-2.80, 2.80], m = 1.725100),  # i = 3
RevoluteDH(a=-0.045, alpha=np.pi/2,          I=[0.01491672, 0.01482922, 0.00294463, 0.00001741, -0.00002109, -0.00150604],  qlim=[-0.90, 3.10], m = 2.172700),  # i = 4
RevoluteDH(          alpha=-np.pi/2, d=0.3,  I=[0.00005029, 0.00007582, 0.00006270, 0.00000020, -0.00000359, -0.00000005],  qlim=[-4.76, 1.24], m = 0.356600),  # i = 5
RevoluteDH(          alpha=np.pi/2,          I=[0.00055516, 0.00024367, 0.00045358, 0.00000061, -0.00004590, -0.00000074],  qlim=[-1.50, 1.50], m = 0.409200),  # i = 6
RevoluteDH(d=0.060,                          I=[0.00003773, 0.00003806, 0.00007408, -0.00000019, 0, 0],                     qlim=[-2.20, 2.20], m = 0.075500),  # i = 7
], name="WAM")
# inertia tensor of link with respect to center of mass:
# I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
# above from: https://github.com/petercorke/robotics-toolbox-python/blob/71183f3221cf9ced69420f07ade06f4514c256ba/roboticstoolbox/models/DH/README.md
# values from: http://www.me.unm.edu/~starr/research/WAM_MassParams_AA-00.pdf

""" iterate joints to find path """
from scipy.spatial.transform import Slerp, Rotation # use this for linear interpolation of rotation


# +++ TODO: ADD CAPABILITIES TO CHECK IF STUFF ALREADY CALLED SO NOT ALWAYS STARTING FROM HOME, BUT CAN ALSO START FROM LAST (goal) POSE?

class Controlla():
    def __init__(self):
        
        # initial definition of variables
        
        rospy.init_node('trajectory_giver')        

        # init vars
        self.pub = rospy.Publisher('joint_traj', JointTrajectory, queue_size=10)
        # self.pubd = rospy.Publisher('joint_traj', JointTrajectory, queue_size=10)
        self.now = rospy.Time.now().secs + rospy.Time.now().nsecs/1000000000

        self.number_of_joints = 7
        # self.number_of_joints = 8
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

        # 
        self.hold = False
        self.angles_start = np.zeros(7)
        self.angles_end_hidden = np.ones(7)
    


    # TODO: make class to incorporate subscriber for goal input
    # http://wiki.ros.org/rospy_tutorials/Tutorials/numpy
    def callback(self, data):
        # rospy.loginfo("I heard %s",data.data)
        # rospy.loginfo("I heard %s", str(data.data))
        # +++ STORE RECEIVED VAR HERE
        '''
        # from: https://github.com/petercorke/robotics-toolbox-python/blob/71183f3221cf9ced69420f07ade06f4514c256ba/roboticstoolbox/examples/readme.py
        T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
        sol = robot.ikine_LMS(T)  # solve IK, ignore additional outputs
        print(sol.q)  # display joint angles
        # FK shows that desired end-effector pose was achieved
        print(robot.fkine(sol.q))
        '''
        # below SE3 from https://petercorke.github.io/spatialmath-python/3d_pose_SE3.html?highlight=oa#spatialmath.pose3d.SE3.OA
        T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])  # right creates an SE(3) pure rotation from 2 vectors
        sol = robot.ikine_LMS(T)  # solve IK, ignore additional outputs
        # also, special thanks to jt for telling me to use this function for ik
        self.angles_end_hidden = sol.q
        print(self.angles_end_hidden)

        self.temp = data.data
        rospy.loginfo("logged data")
        
    def listener(self):
        # rospy.init_node('node_name')
        # rospy.Subscriber("chatter", String, self.callback)
        rospy.Subscriber("chatter", numpy_msg(Floats), self.callback)
        # rospy.spin()

        """"""

        # test for subscriber callback
        T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
        # T = SE3(0.2, 0.5, 0.3) * SE3.OA([0, 1, 0], [0, 0, -1])
        sol = robot.ikine_LMS(T)  # solve IK, ignore additional outputs
        self.angles_end_hidden = sol.q
        print(self.angles_end_hidden)

        ''''''
        
        # # test for ellipse calc
        # # (once fixed, move to separate)
        # a = 2
        # b = 1

        # # (x^2/a^2) + (y^2/b^2) = 1

        # self.x = b*math.cos(self.now)
        # self.y = a*math.sin(self.now)

        # # trajectory in xz plane (z = y; x = x)
        # T = SE3(self.x, 0.1, self.y) * SE3.OA([0, 1, 0], [0, 0, -1])    # TODO: add checking to determine OA axes
        # sol = robot.ikine_LMS(T)
        # self.angles_end_hidden = sol.q
        # print(self.angles_end_hidden)

        """"""

        print("passed")
        rospy.sleep(1) #change to however long buffer for test input
                        # or add srv!
    
    # def calc_ellipse(self, widthin, heightin):
    def calc_ellipse(self):
        # to calculate ellipse trajectory
        # x = xin
        # y = yin
        # a = widthin/2
        # b = heightin/2
        a = 2
        b = 1

        # (x^2/a^2) + (y^2/b^2) = 1
        # x,y = function of time
        # start at top center of ellipse (0, +y)

        # xstart = 0
        # ystart = a

        self.x = b*math.cos(self.now)
        self.y = a*math.sin(self.now)

        # trajectory in xz plane (z = y; x = x)
        T = SE3(self.x, 0.1, self.y) * SE3.OA([0, 1, 0], [0, 0, -1])    # TODO: add checking to determine OA axes
        sol = robot.ikine_LMS(T)
        self.angles_end_hidden = sol.q
        print(self.angles_end_hidden)


    def talker(self):
        # +++ CALLBACK FUNCTION STORES RECEIVED POSE @ angles_end_hidden; NOW FREE TO USE

        rospy.init_node('trajectory_giver')

        """ start at random configuration (ignoring collision) """
        np.random.seed(0)
        # angles_start = np.random.uniform(0, 2*np.pi, (7,))
        # angles_start = np.zeros(7) # to start from upright position
        # print("angles_start", angles_start) #added for checking
        self.T_start = robot.fkine(self.angles_start)

        """ find reachable desired end configuation for joints """
        # angles_end_hidden = np.random.uniform(0, 2*np.pi, (7,)) # this is not actually known
        # angles_end_hidden = np.ones(7) # placeholder end pose (make subscriber to topic)
        # print("angles_end_hidden", angles_end_hidden) #added for checking
        self.T_end = robot.fkine(self.angles_end_hidden)

        # +++ TODO: ADD SWITCH FOR IF BUILT-IN ELLIPSE OR RECEIVING INPUT GOAL COORDS
        while not rospy.is_shutdown():
            self.gain = 0.01
            self.T_current = self.T_start.copy()
            self.q_current = self.angles_start

            if(self.hold != True):
                # for i in range(10):
                for i in range(100):
                    # TODO: EDIT SO NO ITERATIONS; CHECKS IF FINAL POSE == GOAL POSE
                    # find desired omega
                    self.q1 = Rotation.from_matrix(self.T_current.R)
                    self.q2 = Rotation.from_matrix(self.T_end.R)
                    self.aa_diff = (self.q1.inv() * self.q2).as_rotvec()
                    self.omega = self.aa_diff*self.gain

                    # find desired velocity
                    self.x1 = self.T_current.t
                    self.x2 = self.T_end.t
                    self.velocity = (self.x2 - self.x1)*self.gain

                    # find a solution to [v, w]' = J dq/dt
                    self.J = robot.jacob0(self.angles_end_hidden)
                    self.qdot = np.linalg.pinv(self.J) @ np.hstack([self.omega, self.velocity])
                    # qdot = np.zeros((7,))
                    self.q_new = self.q_current + self.qdot

                    # update
                    self.q_current = self.q_new.copy()
                    self.T_current = robot.fkine(self.q_current)

                    self.corz = robot.coriolis(self.q_current, self.qdot)
                    # print("corz", corz)
                    print("q_current", self.q_current)


                    # keep for smooth path (no visual glitch)
                    self.all_sets_of_joint_coordinates = JointTrajectory()
                    # Each joint coordinate/joint point consists of desired positions for each 
                    # joint and a time for when the joints should assume their positions.
                    self.one_set_of_joint_coordinates = JointTrajectoryPoint()
                    # self.all_sets_of_joint_coordinates.header.stamp = rospy.Time.now()


                    # Create  J1 through J7
                    for joint_number in range (1, self.number_of_joints + 1):  
                        self.all_sets_of_joint_coordinates.joint_names.append("J%i"%joint_number)
                    # Usually you'd be feeding the joint trajectory in from somewhere intelligent. Here we create a hundred points.
                    # Each point is going to move all 7 joints an additional .01 degrees.
                    # Each point takes place .01 seconds after the other.
                    # number_of_boring_points = 100
                    # initial starting position for all joints
                    # joint_coord = .1
                    # time at which to take initial positions
                    # time_for_move = .01

                    for joint_number in range (1, self.number_of_joints + 1):
                    # for one point, set all desired joint positions equal to joint_coord
                        self.one_set_of_joint_coordinates.positions.append(self.q_current[joint_number-1])
                    # And set the time for the desired joint positions
                    # self.one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(time_for_move)
                    self.one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(self.now)
                    # append that joint coordinate to the list of joint coordinates
                    self.all_sets_of_joint_coordinates.points.append(self.one_set_of_joint_coordinates)
                    # Reset the holding variable for one joint coordinate
                    self.one_set_of_joint_coordinates = None
                    self.one_set_of_joint_coordinates = JointTrajectoryPoint()
                    # Increment the desired position.
                    # joint_coord = joint_coord + .01
                    # Increment the time.
                    # time_for_move += .01
                    
                    str = "Published a joint trajectory at time %s" % rospy.get_time()
                    #Send a comment to the terminal stating the time the trajectory was published
                    # rospy.loginfo(str)
                    #Publish the set of joint coordinates
                    self.pub.publish(self.all_sets_of_joint_coordinates)
                    # print("pubd: ", self.q_current)
                    #Wait.
                    # rospy.sleep(1.0)
                    rospy.sleep(0.05)

                    self.hold = True
            else:
                # hold final pose
                self.pub.publish(self.all_sets_of_joint_coordinates) 
                # print("pubd: ", self.q_current)


if __name__ == '__main__':
    try:
        # talker()
        ctlr = Controlla()
        ctlr.listener()
        # +++ LISTEN FOR VAR INPUT THEN SEND TO JOINT ANGLE PUBLISHER
        # +++ RECEIVES GOAL FROM JOHN'S (THEORETICALLY)
        ctlr.talker()
    except rospy.ROSInterruptException:
        pass
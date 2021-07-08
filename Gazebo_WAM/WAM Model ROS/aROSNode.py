#!/usr/bin/env python3

#    Example ROS node. Demonstrates how to write a ROS node to control the simulated WAM.
#    Copyright (C) 2013  Benjamin Blumer
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
This script is used to publish a trajectory on the ROS topic joint_traj.

The included WAM plugin subscribes to this topic, and carries out the motion.
The trajectory starts every joint off at a value of .1 and increases the
value by .01 rads every .1 seconds. You can easily replace this with a 
meaningful trajectory by filling one_set_of_joint_coordinates with the
coordinates you want before appending it to all_sets_of_joint_coordinates.

I've heavily commented the code to make it accessible for people who are new
to ROS. In doing so, I've violated the good practice of "Assume the
reader knows Python better than you". I've done so in the hope that it will
help people get their own projects up and running quickly.  If you have any
recomendations on making this more accessible, please let me know
via my github account or make the change yourself and request a "pull".
"""

import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#import _Num.py

# added for storing joint angles
import numpy as np


def talker():

    # init
    rospy.init_node('trajectory_giver')
    pub = rospy.Publisher('joint_traj', JointTrajectory)
    
    # params
    number_of_joints = 7    # Seven joints on a Barrettt WAM Arm.

    all_sets_of_joint_coordinates = JointTrajectory()
    # Each joint coordinate/joint point consists of desired positions for each 
    # joint and a time for when the joints should assume their positions.
    one_set_of_joint_coordinates = JointTrajectoryPoint()
    all_sets_of_joint_coordinates.header.stamp = rospy.Time.now()

    # Create  J1 through J7
    for joint_number in range (1, number_of_joints + 1):  
       all_sets_of_joint_coordinates.joint_names.append("J%i"%joint_number)

    # time at which to take initial positions
    time_for_move = .01

    j1pts = [0.264788565805229, 0.300593407932477, -2.59998455239453]
    j2pts = [0.320104653591992, 0.462606093228363, -0.672710646224523]
    j3pts = [0.171808822192069, 0.111164539316906, 0.15922646198475]
    j4pts = [-0.532532874426464, -0.821081378219505, 1.44168472109312]
    j5pts = [-0.1432024420282, -0.120844695367005, 0.313147557879298]
    j6pts = [0.212605530458411, 0.239272092019939, -0.421765945493522]
    j7pts = [0.065166056892007, 0.066566060521693, 2.56768068058718]

    # added for barrett hand    LIMITS:
    j8pts  = [0, 0, 0]         # fixed
    j9pts  = [0, 0, 0]         # fixed
    j10pts = [1, 3, 1]         # (0, pi)
    j11pts = [-3, 0, 3]        # (-pi, pi)
    j12pts = [-6, -3, -1]      # (-2pi, 0)
    j13pts = [1, 3, 1]         # (0, pi)
    j14pts = [-3, 1, 3]        # (-pi, pi)
    j15pts = [1, 3, 6]         # (0, 2pi)
    j16pts = [1, 3, 1]         # (0, pi)
    j17pts = [-3, -1, 3]       # (-pi, pi)

    # angls = np.matrix([j1pts, j2pts, j3pts, j4pts, j5pts, j6pts, j7pts])
    angls = np.matrix([j1pts, j2pts, j3pts, j4pts, j5pts, j6pts, j7pts, j8pts, j9pts, j10pts, j11pts, j12pts, j13pts, j14pts, j15pts, j16pts, j17pts])

    # for i in range (0, number_of_boring_points): 
    for i in range (0,2):
       for joint_number in range (1, number_of_joints + 1):
        #   one_set_of_joint_coordinates.positions.append(joint_coord)
        one_set_of_joint_coordinates.positions.append(angls[joint_number-1,i])
       one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(time_for_move)
       all_sets_of_joint_coordinates.points.append(one_set_of_joint_coordinates)
       one_set_of_joint_coordinates = None
       one_set_of_joint_coordinates = JointTrajectoryPoint()
    #    joint_coord = joint_coord + .01
    #    time_for_move += .01

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

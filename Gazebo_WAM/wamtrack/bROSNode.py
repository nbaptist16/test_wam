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
    number_of_joints = 8

    all_sets_of_joint_coordinates = JointTrajectory()
    # Each joint coordinate/joint point consists of desired positions for each 
    # joint and a time for when the joints should assume their positions.
    one_set_of_joint_coordinates = JointTrajectoryPoint()
    all_sets_of_joint_coordinates.header.stamp = rospy.Time.now()

    # init joints
    for joint_number in range (1, number_of_joints + 1):  
       all_sets_of_joint_coordinates.joint_names.append("J%i"%joint_number)

    # time at which to take initial positions
    time_for_move = 0.01

    # lower joint limts-ish
    j0pts   = [-0.8, -0.7, -0.6, -0.5, -0.4]
    j1pts   = [-2.5, -2.4, -2.3, -2.2, -2.1]
    j2pts   = [-1.9, -1.8, -1.7, -1.6, -1.5]
    j3pts   = [-2.5, -2.4, -2.3, -2.2, -2.1]
    # j4pts   = [-0.5]
    # j5pts   = [-4.5]
    j4pts   = [1.1, 1.2, 1.3, 1.4, 1.5]
    j5pts   = [-1.75, -1.65, -1.55, -1.45, -1.35]
    j6pts   = [-1.5, -1.4, -1.3, -1.2, -1.1]
    j7pts   = [-2.0, -1.9, -1.8, -1.7, -1.6]
    # j8pts   = [0, 0.1, 0.2, 0.3, 0.4]   #fixed
    # j9pts   = [0, 0.1, 0.2, 0.3, 0.4]   #fixed
    # j10pts  = [0, 0.1, 0.2, 0.3, 0.4]
    # j11pts  = [0, 0.1, 0.2, 0.3, 0.4]
    # j12pts  = [0, 0.1, 0.2, 0.3, 0.4]
    # j13pts  = [0, 0.1, 0.2, 0.3, 0.4]
    # j14pts  = [0, 0.1, 0.2, 0.3, 0.4]
    # j15pts  = [0, 0.1, 0.2, 0.3, 0.4]
    # j16pts  = [0, 0.1, 0.2, 0.3, 0.4]
    # j17pts  = [0, 0.1, 0.2, 0.3, 0.4]
    # j18pts  = [0]
    j8pts     = [-0.8, -0.7, -0.6, -0.5, -0.4]

    # joint_coord = -2
    joint_coord = -1

    # for i in range (0, 150):
    #    for joint_number in range (1, number_of_joints + 1):
    #        one_set_of_joint_coordinates.positions.append(joint_coord)
    #        one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(time_for_move)
    #        all_sets_of_joint_coordinates.points.append(one_set_of_joint_coordinates)
    #        one_set_of_joint_coordinates = None
    #        one_set_of_joint_coordinates = JointTrajectoryPoint()
    #    joint_coord = joint_coord + .01
    #    time_for_move += .01
    for i in range(len(j1pts)):
       for joint_number in range (1, number_of_joints + 1):
           one_set_of_joint_coordinates.positions.append(joint_coord)
           one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(time_for_move)
           all_sets_of_joint_coordinates.points.append(one_set_of_joint_coordinates)
           one_set_of_joint_coordinates = None
           one_set_of_joint_coordinates = JointTrajectoryPoint()
       joint_coord = joint_coord + .01
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
#!/usr/bin/env python
import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String

class Robot:

    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
        """

        self._current =  Pose() # initlize correctly
        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
        self._vel_pub = rospy.Publisher('/cmd_vel_mux', Twist, queue_size=1)
        rospy.Subscriber('YOUR_STRING_HERE', Twist, self.navToPose, queue_size=1) # handle nav goal events


    def navToPose(self,goal):
        """
            This is a callback function. It should exract data from goal, drive in a striaght line to reach the goal and
            then spin to match the goal orientation.
        """

        self._odom_list.waitForTransform('YOUR_STRING_HERE', '/base_link', rospy.Time(0), rospy.Duration(1.0))
        transGoal = self._odom_list.transformPose('YOUR_STRING_HERE', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system

    def executeTrajectory(self):
      """
        See lab manual for the dance the robot has to excute
      """

    def driveStraight(self, speed, distance):
        """
            This method should populate a ??? message type and publish it to ??? in order to move the robot
        """
        origin = copy.deepcopy(self._current) #hint:  use this

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0

        r.rospy.Rate(1)

        traveled_distance = origin.orientation.x - self._current.orientation.x


        while traveled_distance < distance:
            self._vel_pub.publish(twist)
            r.sleep()
            traveled_distance = origin.orientation.x - self._current.orientation.x


    def spinWheels(self, v_left, v_right, time):
        """
           This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
           It should then create a ??? message type, and publish it to ??? in order to move the robot
        """

        diameter = 0.23 # based on wheel track from https://yujinrobot.github.io/kobuki/doxygen/enAppendixKobukiParameters.html

        twist = Twist();
        twist.angular.z = (v_right - v_left) / D;
        r = rospy.Rate(10)

        driveStartTime = rospy.Time.now().secs

        while rospy.Time.now().secs < (driveStartTime + time):
            self._vel_pub.publish(twist)
            r.sleep()

    def rotate(self,angle):
        """
            This method should populate a ??? message type and publish it to ??? in order to spin the robot
        """

        origin = copy.deepcopy(self._current)

        q = [origin.orientation.x,
             origin.orientation.y,
             origin.orientation.z,
             origin.orientation.w] # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)

        twist = Twist()
        twist.angular.z = angle / 10;
        r = rospy.Rate(10)

        current_q = [self._current.orientation.x,
                     self._current.orientation.y,
                     self._current.orientation.z,
                     self._current.orientation.w]

        (current_roll, current_pitch, current_yaw) = euler_from_quaternion(current_q)


        while (current_yaw - yaw) < angle:
            self._vel_pub.publish(twist)
            r.sleep()
            current_q = [self._current.orientation.x,
                     self._current.orientation.y,
                     self._current.orientation.z,
                     self._current.orientation.w]

            (current_roll, current_pitch, current_yaw) = euler_from_quaternion(current_q)

    def timerCallback(self,evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
        """
	# wait for and get the transform between two frames
        self._odom_list.waitForTransform('/odom', '/base_link', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('/odom','/base_link', rospy.Time(0))
	# save the current position and orientation
	self._current.position.x = position[0]
        self._current.position.y = position[1]
        self._current.orientation.x = orientation[0]
        self._current.orientation.y = orientation[1]
        self._current.orientation.z = orientation[2]
        self._current.orientation.w = orientation[3]

    # helper functions
    def planTraj(self, b, t):
        """
            Bonus Question:  compute the coefs for a cubic polynomial (hint:  you did this in 3001)
        """


if __name__ == '__main__':

    rospy.init_node('drive_base')
    turtle = Robot()

    #test function calls here

    while  not rospy.is_shutdown():
        pass
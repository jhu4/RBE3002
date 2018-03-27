#!/usr/bin/env python
import rospy, tf, copy, math, time

from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String

class Robot:

    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
        """
        self.angle_threshold = 0.1
        self.velocity_threshold = 1
        self._current =  Pose() # initlize correctly
        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
	#/cmd_vel_mux/input/teleop
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.navToPose, queue_size=1) # handle nav goal events

    def getYaw(self, orientation):
        q = [orientation.x,
             orientation.y,
             orientation.z,
             orientation.w] # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)
        return yaw

    def navToPose(self,goal):
        """
            This is a callback function. It should exract data from goal, drive in a striaght line to reach the goal and
            then spin to match the goal orientation.
        """
        # self._odom_list.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        rospy.sleep(1)
        transGoal = self._odom_list.transformPose('base_link', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system

        goal_yaw = self.getYaw(transGoal.pose.orientation)


        # origin = copy.deepcopy(self._current)
        dy = transGoal.pose.position.y #- origin.position.y
        dx = transGoal.pose.position.x #- origin.position.x
        align_angle = math.atan(dy/dx)
        distance = math.sqrt(dy**2+dx**2)

        print("dy",dy)
        print("dx",dx)
        print("distance",distance)
        print("a angle",align_angle)
        self.rotate(align_angle)
        self.driveStraight(0.05, distance)

        self.rotate(goal_yaw - align_angle)

    def executeTrajectory(self):
      """
        See lab manual for the dance the robot has to excute
      """
      self.driveStraight(0.1, 6)
      self.rotate(-math.pi/2)
      self.driveStraight(.1, 4.5)
      self.rotate(math.pi/4*3)

    def driveStraight(self, speed, distance):
        """
            This method should populate a ??? message type and publish it to ??? in order to move the robot
        """
        origin = copy.deepcopy(self._current) #hint:  use this

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0

        print("inital x:"+str(origin.position.x))
        r = rospy.Rate(10)

        goal = distance + origin.position.x

        while abs(goal - self._current.position.x) > 0.1:
            self._vel_pub.publish(twist)
            print("current x:"+str(self._current.position.x))
            r.sleep()
        self.stop()

    def spinWheels(self, v_left, v_right, time):
        """
           This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
           It should then create a ??? message type, and publish it to ??? in order to move the robot
        """

        L = .138

        twist = Twist();
        twist.linear.x = (v_left + v_right) / 2
        twist.angular.z = (v_right - v_left) / L
        r = rospy.Rate(10)

        driveStartTime = rospy.Time.now().secs
        while rospy.Time.now().secs < (driveStartTime + time):
            self._vel_pub.publish(twist)
            r.sleep()

        self.stop()

    def convert_angle(self, angle):
        return angle + math.pi

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

        goal_yaw = (self.convert_angle(yaw) + angle) % (2 * math.pi)

        # print("init yaw:"+str(yaw))

        twist = Twist()
        twist.angular.z = angle / 5;
        r = rospy.Rate(10)

        current_q = [self._current.orientation.x,
                     self._current.orientation.y,
                     self._current.orientation.z,
                     self._current.orientation.w]

        (current_roll, current_pitch, current_yaw) = euler_from_quaternion(current_q)

        current_yaw = self.convert_angle(current_yaw)

        while abs(current_yaw - goal_yaw) > self.angle_threshold:
            self._vel_pub.publish(twist)
            r.sleep()
            current_q = [self._current.orientation.x,
                     self._current.orientation.y,
                     self._current.orientation.z,
                     self._current.orientation.w]

            (current_roll, current_pitch, current_yaw) = euler_from_quaternion(current_q)
            current_yaw = self.convert_angle(current_yaw)
            # print("current yaw:" + str(current_yaw))

        self.stop()

    def stop(self):
        twist = Twist()
        self._vel_pub.publish(twist)

    def timerCallback(self,evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
        """
    # wait for and get the transform between two frames
        self._odom_list.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('odom','base_link', rospy.Time(0))
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

    time.sleep(1)

    #test function calls here

    # turtle.rotate(3.14)
    # turtle.spinWheels(-0.2, -0.2, 5)
    # turtle.driveStraight(0.2, 7)
    while  not rospy.is_shutdown():
    # 	# turtle.driveStraight(.2,.6)
    #     turtle.rotate(3.14)
    # turtle.stop()
        pass

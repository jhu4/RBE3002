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
        self.L = .138
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
        goal_yaw = self.getYaw(goal.pose.orientation)


        origin = copy.deepcopy(self._current)
        dy = goal.pose.position.y - origin.position.y
        dx = goal.pose.position.x - origin.position.x
        dtheta = math.atan2(dy,dx)
        distance = math.sqrt(dy**2+dx**2)

        # print("dy",dy)
        # print("dx",dx)
        # print("distance",distance)
        # print("dtheta",dtheta)
        self.rotate(dtheta - self.getYaw(origin.orientation))
        self.driveStraight(0.1, distance)
        self.rotate(goal_yaw - dtheta)

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

        # print("inital x:"+str(origin.position.x))
        r = rospy.Rate(10)

        dy = self._current.position.y - origin. position.y
        dx = self._current.position.x - origin. position.x

        while math.sqrt(dy**2 + dx**2) < abs(distance):
            self._vel_pub.publish(twist)
            # print("current x:",str(self._current.position.x),"y:",abs(self._current.position.y),"ds",math.sqrt(dy**2 + dx**2))
            r.sleep()
            dy = self._current.position.y - origin. position.y
            dx = self._current.position.x - origin. position.x

        self.stop()
    def driveArc(self, radius, speed, angle):
        omega = speed/ radius
        twist = Twist()

        twist.linear.x = speed
        twist.angular.z = omega

        r = rospy.Rate(10)

        dt = float(angle) / omega
        driveStartTime = rospy.get_time()
        while rospy.get_time() < (driveStartTime + dt):
            self._vel_pub.publish(twist)
            r.sleep()
        self.stop()

    def bangbangControl(self, speed, distance):
        origin = copy.deepcopy(self._current) #hint:  use this

        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0

        speed = float(speed)
        # print("inital x:"+str(origin.position.x))
        r = rospy.Rate(10)

        dy = self._current.position.y - origin. position.y
        dx = self._current.position.x - origin. position.x

        while math.sqrt(dy**2 + dx**2) < (abs(distance) * 0.6):
            self._vel_pub.publish(twist)
            # print("speed",twist.linear.x)
            r.sleep()
            if twist.linear.x < speed:
                twist.linear.x += (speed / 10)
            dy = self._current.position.y - origin. position.y
            dx = self._current.position.x - origin. position.x
        while math.sqrt(dy**2 + dx**2) < abs(distance):
            self._vel_pub.publish(twist)
            # print("speed",twist.linear.x)
            r.sleep()
            if twist.linear.x > (speed / 20):
                twist.linear.x -= (speed / 20)
            dy = self._current.position.y - origin. position.y
            dx = self._current.position.x - origin. position.x
        self.stop()

    def spinWheels(self, v_left, v_right, time):
        """
           This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
           It should then create a ??? message type, and publish it to ??? in order to move the robot
        """

        twist = Twist();
        twist.linear.x = (v_left + v_right) / 2
        twist.angular.z = (v_right - v_left) / self.L
        r = rospy.Rate(10)

        driveStartTime = rospy.Time.now().secs
        while rospy.Time.now().secs < (driveStartTime + time):
            self._vel_pub.publish(twist)
            r.sleep()

        self.stop()

    def convertAngle(self, angle):
        return angle + math.pi

    def rotate(self,angle):
        """
            This method should populate a ??? message type and publish it to ??? in order to spin the robot
        """

        origin = copy.deepcopy(self._current)
        yaw = self.getYaw(origin.orientation)

        goal_yaw = (self.convertAngle(yaw) + angle) % (2 * math.pi)

        twist = Twist()
        twist.angular.z = angle / 5;
        r = rospy.Rate(10)

        current_yaw = self.convertAngle(self.getYaw(self._current.orientation))

        while abs(current_yaw - goal_yaw) > abs(angle / 20):
            # print("angle diff:",abs(current_yaw - goal_yaw),"threshold" , abs(angle/40))
            self._vel_pub.publish(twist)
            r.sleep()
            current_yaw = self.convertAngle(self.getYaw(self._current.orientation))
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
    rospy.sleep(2)
    # time.sleep(1)

    #test function calls here

    # turtle.rotate(math.pi)
    turtle.driveArc(.5, 0.1, math.pi)
    # turtle.spinWheels(-0.2, -0.2, 5)
    # turtle.driveStraight(-0.2, 3)
    # turtle.bangbangControl(2, 3)
    # while  not rospy.is_shutdown():
    # 	# turtle.driveStraight(.2,.6)
    #     turtle.rotate(3.14)
    # turtle.stop()
    #     pass

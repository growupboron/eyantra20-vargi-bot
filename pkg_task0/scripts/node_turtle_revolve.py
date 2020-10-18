#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pi


class TurtleBotTask0:
    def __init__(self):
        # starting the node
        rospy.init_node("node_turtle_revolve.py", log_level=rospy.DEBUG)

        # Publish velocities to control the turtlebot
        self.velocity_publisher = rospy.Publisher(
            "/turtle1/cmd_vel", Twist, queue_size=10
        )

        # Listen for current position / velocities
        self.pose_subscriber = rospy.Subscriber("turtle1/pose", Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):

        # Update current position data
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def move_in_circle(self):
        vel_msg = Twist()

        angle = 360

        # Increase angular speed for smaller circle
        angular_speed = 58 * 2 * pi / 360
        radian_angle = angle * 2 * pi / 360

        # Only move forward in linear direction
        vel_msg.linear.x = 1
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Set Z-Axis Angular Speed
        vel_msg.angular.z = abs(angular_speed)

        current_angle = 0
        start_time = rospy.Time.now().to_sec()
        while current_angle < radian_angle:
            # Send velocities to turtlebot
            self.velocity_publisher.publish(vel_msg)

            current_time = rospy.Time.now().to_sec()
            rospy.loginfo("Moving in a circle\n%s" % (current_time - start_time))
            current_angle = angular_speed * (current_time - start_time)

            # Limit the polling rate
            self.rate.sleep()

        # Bring turtlebot to a stop
        vel_msg.angular.z = 0
        vel_msg.linear.x = 0
        rospy.loginfo("Goal reached !")
        self.velocity_publisher.publish(vel_msg)

        # Wait for keyboard interreupt before quitting
        rospy.spin()


if __name__ == "__main__":
    try:
        # Initialize the bot
        turtle = TurtleBotTask0()
        rospy.logdebug("Started to move")

        # Move turtlebot in a circle
        turtle.move_in_circle()

    except rospy.ROSInterruptException:
        pass

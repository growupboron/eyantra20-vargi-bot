#!/usr/bin/env python

import rospy
import actionlib
import math

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from pkg_task1.msg import msgTurtleAction, msgTurtleResult, msgTurtleFeedback


class SimpleActionServerTurtle:
    def __init__(self):
        self._sas = actionlib.SimpleActionServer(
            "/action_turtle",
            msgTurtleAction,
            execute_cb=self.func_on_rx_goal,
            auto_start=False,
        )
        self._config_ros_pub_topic = "/turtle1/cmd_vel"
        self._config_ros_sub_topic = "/turtle1/pose"

        self._curr_x = 0
        self._curr_y = 0
        self._curr_theta = 0

        self._sas.start()
        rospy.loginfo("Started Turtle Simple Action Server.")

    def func_ros_sub_callback(self, pose_message):
        self._curr_x = pose_message.x
        self._curr_y = pose_message.y
        self._curr_theta = pose_message.theta

    def func_move_straight(self, param_dis, param_speed):

        obj_velocity_mssg = Twist()
        Pose()

        start_x = self._curr_x
        start_y = self._curr_y

        handle_pub_vel = rospy.Publisher(
            self._config_ros_pub_topic, Twist, queue_size=10
        )

        var_loop_rate = rospy.Rate(10)
        obj_velocity_mssg.linear.x = abs(int(param_speed))

        dis_moved = 0.0

        while not rospy.is_shutdown() and dis_moved < param_dis:

            obj_msg_feedback = msgTurtleFeedback()
            obj_msg_feedback.cur_x = self._curr_x
            obj_msg_feedback.cur_y = self._curr_y
            obj_msg_feedback.cur_theta = self._curr_theta
            self._sas.publish_feedback(obj_msg_feedback)

            handle_pub_vel.publish(obj_velocity_mssg)
            var_loop_rate.sleep()
            dis_moved = abs(
                math.sqrt(
                    ((self._curr_x - start_x) ** 2) +
                    ((self._curr_y - start_y) ** 2)
                )
            )

        obj_velocity_mssg.linear.x = 0
        handle_pub_vel.publish(obj_velocity_mssg)

    def func_rotate(self, param_degree, param_speed):

        obj_velocity_mssg = Twist()
        Pose()

        start_degree = abs(math.degrees(self._curr_theta))
        current_degree = abs(math.degrees(self._curr_theta))
        handle_pub_vel = rospy.Publisher(
            self._config_ros_pub_topic, Twist, queue_size=10
        )
        var_loop_rate = rospy.Rate(10)
        obj_velocity_mssg.angular.z = math.radians(abs(int(param_speed)))

        degree_rotated = 0.0

        while not rospy.is_shutdown() and round(degree_rotated) < param_degree:

            handle_pub_vel.publish(obj_velocity_mssg)

            var_loop_rate.sleep()

            current_degree = abs(math.degrees(self._curr_theta))
            degree_rotated = abs(current_degree - start_degree)

        obj_velocity_mssg.angular.z = 0
        handle_pub_vel.publish(obj_velocity_mssg)

    def func_on_rx_goal(self, obj_msg_goal):
        rospy.loginfo("Received a Goal from Client.")
        rospy.loginfo(obj_msg_goal)

        if(obj_msg_goal.turn):
            self.func_rotate(60, "10")
        self.func_move_straight(obj_msg_goal.length, "1")


        obj_msg_result = msgTurtleResult()
        obj_msg_result.final_x = self._curr_x
        obj_msg_result.final_y = self._curr_y
        obj_msg_result.final_theta = self._curr_theta

        rospy.loginfo("send goal result to client")
        self._sas.set_succeeded(obj_msg_result)


def main():
    rospy.init_node("node_simple_action_server_turtle")
    obj_server = SimpleActionServerTurtle()
    rospy.Subscriber(
        obj_server._config_ros_sub_topic, Pose, obj_server.func_ros_sub_callback
    )
    rospy.spin()


if __name__ == "__main__":
    main()

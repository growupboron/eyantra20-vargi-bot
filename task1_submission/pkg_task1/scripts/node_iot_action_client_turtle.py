#!/usr/bin/env python
# coding: utf-8
import rospy
import actionlib

from pkg_task1.msg import msgTurtleAction, msgTurtleGoal
from pkg_ros_iot_bridge.msg import msgIotRosAction, msgIotRosGoal, msgIotRosResult, msgMqttSub


class SimpleActionClientTurtle:

    # Constructor
    def __init__(self):
        self._sac = actionlib.SimpleActionClient('/action_turtle',
                                                 msgTurtleAction)
        self._sac.wait_for_server()
        rospy.loginfo("Simple Action server is up, we can send new goals!")
        self._ac = actionlib.ActionClient('/action_ros_iot', msgIotRosAction)
        self._goal_handles = {}
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._ac.wait_for_server()
        rospy.loginfo("Action server up, we can send goals.")
        self.print_banner()

    def trace_hexagon_side(self, arg_dis, turn):
        goal = msgTurtleGoal(length=arg_dis, turn=turn)

        self._sac.send_goal(goal, done_cb=self.done_callback,
                            feedback_cb=self.feedback_callback)

    def done_callback(self, status, result):
        payload = str((result.final_x, result.final_y, result.final_theta))
        self.send_goal("mqtt", "pub", self._config_mqtt_pub_topic, payload)
        

    def feedback_callback(self, feedback):
        pass

    def on_transition(self, goal_handle):
        pass

    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        goal = msgIotRosGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        goal_handle = self._ac.send_goal(goal,
                                         self.on_transition,
                                         None)
        return goal_handle

    def func_callback_start_received(self, payload):
        if(payload.message == "start"):
            self.trace_hexagon()
        

    def trace_hexagon(self):
        first_side_complete = False
        for i in range(6):
            self.trace_hexagon_side(2, first_side_complete)
            first_side_complete = True
            rospy.sleep(10)
    
    def print_banner(self):
        banner = """ 
        ███████╗██╗░░░██╗░█████╗░███╗░░██╗████████╗██████╗░░█████╗░  ████████╗░█████╗░░██████╗██╗░░██╗  ░░███╗░░
        ██╔════╝╚██╗░██╔╝██╔══██╗████╗░██║╚══██╔══╝██╔══██╗██╔══██╗  ╚══██╔══╝██╔══██╗██╔════╝██║░██╔╝  ░████║░░
        █████╗░░░╚████╔╝░███████║██╔██╗██║░░░██║░░░██████╔╝███████║  ░░░██║░░░███████║╚█████╗░█████═╝░  ██╔██║░░
        ██╔══╝░░░░╚██╔╝░░██╔══██║██║╚████║░░░██║░░░██╔══██╗██╔══██║  ░░░██║░░░██╔══██║░╚═══██╗██╔═██╗░  ╚═╝██║░░
        ███████╗░░░██║░░░██║░░██║██║░╚███║░░░██║░░░██║░░██║██║░░██║  ░░░██║░░░██║░░██║██████╔╝██║░╚██╗  ███████╗
        ╚══════╝░░░╚═╝░░░╚═╝░░╚═╝╚═╝░░╚══╝░░░╚═╝░░░╚═╝░░╚═╝╚═╝░░╚═╝  ░░░╚═╝░░░╚═╝░░╚═╝╚═════╝░╚═╝░░╚═╝  ╚══════╝ 

        To pass data to the ROS IOT Bridge Node, MQTT Client should publish on the MQTT Topic : eyrc/sVaPtIeT/iot_to_ros
        To get data from the ROS IOT Bridge Node, MQTT Client should subscribe to the MQTT Topic : eyrc/sVaPtIeT/ros_to_iot

        Publish [start] on [eyrc/sVaPtIeT/iot_to_ros] to start the turtle.


"""
        print(banner)
# Main Function


def main():
    rospy.init_node('node_simple_action_client_turtle')

    obj_client = SimpleActionClientTurtle()

    rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub,
                     obj_client.func_callback_start_received)
    

    # 4. Loop forever
    rospy.spin()


if __name__ == '__main__':
    main()

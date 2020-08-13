#!/usr/bin/env python2
# Send a value to change the opening of the Robotiq gripper using an action

import argparse
import geometry_msgs.msg
import rospy, sys, numpy as np
import actionlib
import control_msgs.msg
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty


rospy.init_node('gripper_command_master')
# Create an action client
client = actionlib.SimpleActionClient(
'/gripper_controller/gripper_cmd',  # namespace of the action topics
control_msgs.msg.GripperCommandAction # action type
)



# Wait until the action server has been started and is listening for goals
client.wait_for_server()
print("here")

# Create a goal to send (to the action server)
goal = control_msgs.msg.GripperCommandGoal()
goal.command.position = 0  # From 0.0 to 0.8
goal.command.max_effort = -1 # Do not limit the effort
client.send_goal(goal)

client.wait_for_result()
print( client.get_result())




rospy.spin()


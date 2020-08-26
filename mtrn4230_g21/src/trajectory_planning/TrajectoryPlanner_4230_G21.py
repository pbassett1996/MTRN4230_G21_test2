#!/usr/bin/env python
#Subject: MTRN4230
#Group: 21
#Date: 30/07/2020
#About: The following program receives coordinates from the "Pick" node and executues a pick and place waypoint using MoveIt.

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose
from copy import deepcopy
from std_msgs.msg import Header
from mtrn4230_g21.msg import Pick
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import control_msgs.msg
import time
bucket_x = -0.4
bucket_y = 0.6
bucket_y = 0.5
travel_height = 0.3
end_effector_length = 0.05


class MoveItCartesianPath:
    def callback(self,data):
        self.moveArm(1, data)

    def moveArm(self, flag, data):
        rospy.loginfo("received the following point " + str(data))
        # Get the current pose so we can add it as a waypoint
        j = 0
        while (j < 3):
            
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose
            print(start_pose)
            
        
            # Initialize the waypoints list
            waypoints = []

            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            waypoints.append(start_pose)

            wpose = deepcopy(start_pose)
            if(flag):
                if (j == 0):
                    eef_step = 0.02
                    self.goal.command.position = 0.0  # From 0.0 to 0.8
                    self.goal.command.max_effort = -1 # Do not limit the effort
                    self.client.send_goal(self.goal)
                    self.client.wait_for_result()
                    time.sleep(0.5)
                    # Set the next waypoint to location of the pick object
                    wpose.position.x = data.x-0.025
                    wpose.position.y = data.y-0.025
                    wpose.position.z = travel_height
                    wpose.orientation.x = -1
                    wpose.orientation.y = 0
                    wpose.orientation.z = 1
                    wpose.orientation.w = 0
                    waypoints.append(deepcopy(wpose))
                elif (j == 1):
                    eef_step = 0.01
                    wpose.position.x = data.x
                    wpose.position.y = data.y
                    wpose.position.z = data.depth + end_effector_length
                    wpose.orientation.x = -1
                    wpose.orientation.y = 0
                    wpose.orientation.z = 1
                    wpose.orientation.w = 0
                    waypoints.append(deepcopy(wpose))

                else:
                    eef_step = 0.02
                    self.goal.command.position = 0.37 # From 0.0 to 0.8
                    self.goal.command.max_effort = -1 # Do not limit the effort
                    self.client.send_goal(self.goal)
                    self.client.wait_for_result()
                    time.sleep(0.5)
                    # raise
                    wpose.position.x = data.x
                    wpose.position.y = data.y
                    wpose.position.z = travel_height
                    waypoints.append(deepcopy(wpose))

                    #move to the bin
                    wpose.position.x = bucket_x
                    wpose.position.y = bucket_y
                    wpose.position.z = travel_height
                    wpose.orientation.x = -1
                    wpose.orientation.y = 0
                    wpose.orientation.z = 0.5
                    wpose.orientation.w = 0
                    waypoints.append(deepcopy(wpose))

            else:
                eef_step = 0.01
                wpose.position.x = start_pose.position.x-0.3
                wpose.position.y = start_pose.position.y+0.1
                wpose.position.z = travel_height
                waypoints.append(deepcopy(wpose))

                wpose.position.x = start_pose.position.x-0.6
                wpose.position.y = start_pose.position.y+0.2
                wpose.position.z = travel_height
                wpose.orientation.x = -1
                wpose.orientation.y = 0
                wpose.orientation.z = 0.5
                wpose.orientation.w = 0
                waypoints.append(deepcopy(wpose))

                wpose.position.x = bucket_x
                wpose.position.y = bucket_y
                wpose.position.z = travel_height
                
                waypoints.append(deepcopy(wpose))
                j = 3

            fraction = 0.0
            maxtries = 100
            attempts = 0
            j = j+1

            # Set the internal state to the current state
            self.arm.set_start_state_to_current_state()

            # Plan the Cartesian path connecting the waypoints
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = self.arm.compute_cartesian_path (waypoints, eef_step, 0.0, True)

                # Increment the number of attempts
                attempts += 1

                # Print out a progress message
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

            # If we have a complete plan, execute the trajectory
            if fraction > 0.8:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                num_pts = len(plan.joint_trajectory.points)

                rospy.loginfo("\n# waypoints: "+str(num_pts))
                waypoints = []
                for i in range(num_pts):
                    waypoints.append(plan.joint_trajectory.points[i].positions)

                #rospy.init_node('send_joints')
                pub = rospy.Publisher('/arm_controller/command',
                                    JointTrajectory,
                                    queue_size=10)

                # Create the topic message
                traj = JointTrajectory()
                traj.header = Header()
                # Joint names for UR5
                traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                    'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                                    'wrist_3_joint']

                rate = rospy.Rate(20)
                cnt = 0
                pts = JointTrajectoryPoint()
                while not rospy.is_shutdown() and cnt < num_pts -1:

                # print(pub.get_Status())

                    cnt += 1
                    traj.header.stamp = rospy.Time.now()

                    #        pts.positions = [0.0, -2.33, 1.57, 0.0, 0.0, 0.0]

                    pts.positions = waypoints[cnt]

                    pts.time_from_start = rospy.Duration(0.001*cnt)

                    # Set the points to the trajectory
                    traj.points = []
                    traj.points.append(pts)
                    # Publish the message
                    pub.publish(traj)

                    rate.sleep()


        # self.arm.execute(plan)
                rospy.loginfo("Path execution complete.")
                
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")
        
        self.goal.command.position = 0.0 # From 0.0 to 0.8
        self.goal.command.max_effort = -1 # Do not limit the effort
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
    

    def __init__(self):
        rospy.init_node("moveit_cartesian_path", anonymous=False)

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        #subscribe to goal updates
        rospy.Subscriber("pick_point_publisher", Pick, self.callback)


        self.client = actionlib.SimpleActionClient('/gripper_controller/gripper_cmd',  # namespace of the action topics
        control_msgs.msg.GripperCommandAction) # action type

        # Wait until the action server has been started and is listening for goals
        self.client.wait_for_server()

        self.goal = control_msgs.msg.GripperCommandGoal()
        self.goal.command.position = 0  # From 0.0 to 0.8
        self.goal.command.max_effort = -1 # Do not limit the effort
        self.client.send_goal(self.goal)


        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        self.reference_frame = "/base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(self.reference_frame)


        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.moveArm(0,0)
        rospy.spin()


    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    

if __name__ == "__main__":
    try:
        MoveItCartesianPath()
    except KeyboardInterrupt:
        print "Shutting down MoveItCartesianPath node."


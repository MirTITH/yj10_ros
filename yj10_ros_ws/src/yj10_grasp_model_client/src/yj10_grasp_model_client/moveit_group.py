#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.robot import RobotCommander

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class MoveGroupPythonInterface:
    def __init__(self, group_name):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = group_name
        self.move_group = MoveGroupCommander(group_name)

        # Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz.
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

        print("============ Reference frame: %s" % self.get_ref_frame())

    def get_ref_frame(self):
        return self.move_group.get_planning_frame()

    def move_to_pose(self, pose: geometry_msgs.msg.Pose):
        self.move_group.set_pose_target(pose)
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

    def move_to_named(self, name: str):
        self.move_group.set_named_target(name)
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()

#!/usr/bin/env python

import enum
import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
# from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class JointsPlan(EventState):
	'''
	Uses dcma planner to plan or plan and move the specified joints to the target configuration.

	-- group_name       string      move group name

	># joint_config		float[]		Target configuration of the joints.
									Same order as their corresponding names in joint_names.

	#> joint_trajectory JointTrajectory  planned or executed trajectory

	<= done 						Target joint configuration has been planned.
	<= failed 				Failed to find a plan to the given joint configuration.
	'''


	def __init__(self, group_name):
		'''
		Constructor
		'''
		super(JointsPlan, self).__init__(outcomes=['failed', 'done'],
											input_keys=['joint_config'],
											output_keys=['joint_trajectory', 'target_joints'])
		# group_name = ""
		self._group_name = group_name
		self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
		self._move_group.set_planner_id("RRTConnectkConfigDefault")
		self._move_group.set_planning_time(1)
		self._result = None

	def execute(self, userdata):
		'''
		Execute this state
		'''
		if len(self._result.joint_trajectory.points) > 0:
			userdata.joint_trajectory = self._result
			userdata.target_joints = userdata.joint_config
			return 'done'
		else:
			return 'failed'

	def on_enter(self, userdata):
		# (success flag : boolean, trajectory message : RobotTrajectory,
 		#  planning time : float, error code : MoveitErrorCodes)
		speed = 0.1
		self._move_group.set_max_velocity_scaling_factor(speed)
		self._move_group.set_max_acceleration_scaling_factor(speed)
		self._result = self._move_group.plan(userdata.joint_config)

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)

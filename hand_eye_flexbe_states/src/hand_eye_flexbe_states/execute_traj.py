#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf import transformations as tf

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

# from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes
# from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryResult

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class TrajectoryExecuteState(EventState):
	'''
	Move robot by planned trajectory.

	-- group_name         string      move group name

	># joint_trajectory             JointTrajectory  planned trajectory

	<= done 						Robot move done.
	<= failed 						Robot move failed.
	<= collision 				    Robot during collision.
	'''


	def __init__(self, group_name):
		'''
		Constructor
		'''
		super(TrajectoryExecuteState, self).__init__(outcomes=['done', 'failed', 'collision'],
											input_keys=['joint_trajectory', 'target_joints'],
											output_keys=['joint_config'])
		# group_name = ""
		self._group_name = group_name
		self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
		self._result = MoveItErrorCodes.FAILURE

	def stop(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''
		print("")
		print("==================================================================")
		print(self._result)
		print("==================================================================")
		print("")
		if self._result == MoveItErrorCodes.SUCCESS:
			return 'done'
		else:
			userdata.joint_config = userdata.target_joints
			return 'collision'
			
		if self._result == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
			userdata.joint_config = userdata.target_joints
			return 'collision'
		else:
			return 'failed'

	def on_enter(self, userdata):
		self._result = self._move_group.execute(userdata.joint_trajectory)

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)

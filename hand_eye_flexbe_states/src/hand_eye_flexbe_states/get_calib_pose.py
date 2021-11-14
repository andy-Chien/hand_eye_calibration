#!/usr/bin/env python

from flexbe_core import EventState, Logger
from math import radians

class GetCalibPoseState(EventState):
	"""
	Output a fixed pose to move.

	<= done									   Pose has been published.
	<= fail									   Task fail and finished

	"""
	
	def __init__(self):
		"""Constructor"""
		super(GetCalibPoseState, self).__init__(outcomes=['done', 'fail'])

	def execute(self, userdata):
		if True:
			return 'done'
		else:
			return 'fail'
	
	def on_enter(self, userdata):
		pass
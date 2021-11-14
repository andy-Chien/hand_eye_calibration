#!/usr/bin/env python

import numpy as np
import tf
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from visp_hand2eye_calibration.srv import compute_effector_camera_quick, compute_effector_camera_quickRequest

class ComputeCalibState(EventState):
	"""
	Output a fixed pose to move.

	<= done									   Pose has been published.
	<= fail									   Task fail and finished

	"""
	
	def __init__(self, eye_in_hand_mode):
		"""Constructor"""
		super(ComputeCalibState, self).__init__(outcomes=['finish'], input_keys=['base_h_tool', 'camera_h_charuco'])
		self.eye_in_hand_mode = eye_in_hand_mode
		self.trans_A_list = TransformArray()
		self.trans_B_list = TransformArray()
		self.calib_compute_client = ProxyServiceCaller({'compute_effector_camera_quick': compute_effector_camera_quick})
	def execute(self, userdata):
		req = compute_effector_camera_quickRequest(self.trans_A_list, self.trans_B_list)
		res = self._check_scene_client.call('/check_state_validity', req)
		
		print('x = ' + str(res.effector_camera.translation.x))
		print('y = ' + str(res.effector_camera.translation.x))
		print('z = ' + str(res.effector_camera.translation.x))
		print('qw = ' + str(res.effector_camera.rotation.w))
		print('qx = ' + str(res.effector_camera.rotation.x))
		print('qy = ' + str(res.effector_camera.rotation.y))
		print('qz = ' + str(res.effector_camera.rotation.z))
		return 'finish'
	
	def on_enter(self, userdata):
		self.trans_A_list = userdata.camera_h_charuco
		if self.eye_in_hand_mode:
			self.trans_B_list = userdata.base_h_tool
		else:
			self.trans_B_list.header = userdata.base_h_tool.header
			for transform in userdata.base_h_tool:
				trans = tf.transformations.quaternion_matrix([transform.rotation.x, transform.rotation.y,
															  transform.rotation.z, transform.rotation.w])
				trans[0:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
				trans = tf.transformations.inverse_matrix(trans)
				trans_B = Transform()
				trans_B.rotation.x, trans_B.rotation.y, trans_B.rotation.z, \
					trans_B.rotation.w = tf.transformations.quaternion_from_matrix(trans)
				self.trans_B_list.transforms.append(trans_B)
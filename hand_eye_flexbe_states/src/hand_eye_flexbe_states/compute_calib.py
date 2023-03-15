#!/usr/bin/env python

import configparser as ConfigParser
import rospkg
import rospy
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
	
	def __init__(self, eye_in_hand_mode, calibration_file_name):
		"""Constructor"""
		super(ComputeCalibState, self).__init__(outcomes=['finish'], input_keys=['base_h_tool', 'camera_h_charuco'])
		self.eye_in_hand_mode = eye_in_hand_mode
		self.calibration_file_name = str(calibration_file_name)
		self.trans_A_list = TransformArray()
		self.trans_B_list = TransformArray()
		self.calib_compute_client = ProxyServiceCaller({'/compute_effector_camera_quick': compute_effector_camera_quick})
    
	def execute(self, userdata):
		req = compute_effector_camera_quickRequest(self.trans_A_list, self.trans_B_list)
		print ("========================================================================================================")
		res = self.calib_compute_client.call('/compute_effector_camera_quick', req)

		base_trans_rgb, base_rot_rgb = res.effector_camera.translation, res.effector_camera.rotation
		
		print('x = '  + str(base_trans_rgb.x))
		print('y = '  + str(base_trans_rgb.y))
		print('z = '  + str(base_trans_rgb.z))
		print('qw = ' + str(base_rot_rgb.w))
		print('qx = ' + str(base_rot_rgb.x))
		print('qy = ' + str(base_rot_rgb.y))
		print('qz = ' + str(base_rot_rgb.z))

		base_h_rgb = tf.transformations.quaternion_matrix([base_rot_rgb.x, base_rot_rgb.y, base_rot_rgb.z, base_rot_rgb.w])
		base_h_rgb[0:3, 3] = [base_trans_rgb.x, base_trans_rgb.y, base_trans_rgb.z]

		tf_listener = tf.TransformListener()
		try:
			(cam_trans_rgb, cam_rot_rgb) = tf_listener.lookupTransform('camera_base', 'rgb_camera_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn('lookupTransform for robot failed!, ' + 'camera_base' + ', ' + 'rgb_camera_link')

		cam_h_rgb = tf.transformations.quaternion_matrix(cam_rot_rgb)
		cam_h_rgb[0:3, 3] = [cam_trans_rgb[0], cam_trans_rgb[1], cam_trans_rgb[2]]
		rgb_h_cam = tf.transformations.inverse_matrix(cam_h_rgb)
		base_h_cam = np.matmul(base_h_rgb, rgb_h_cam)

		base_trans_cam = base_h_cam[:3, 3]
		base_rot_cam = tf.transformations.quaternion_from_matrix(base_h_cam)

		config = ConfigParser.ConfigParser()
		config.optionxform = str #reference: http://docs.python.org/library/configparser.html
		rospack = rospkg.RosPack()
		curr_path = rospack.get_path('charuco_detector')
		config.read(curr_path + '/config/ '+ self.calibration_file_name)
        
		config.add_section("hand_eye_calibration")
		config.set("hand_eye_calibration", "x",  str(base_trans_rgb.x))
		config.set("hand_eye_calibration", "y",  str(base_trans_rgb.y))
		config.set("hand_eye_calibration", "z",  str(base_trans_rgb.z))
		config.set("hand_eye_calibration", "qw", str(base_rot_rgb.w))
		config.set("hand_eye_calibration", "qx", str(base_rot_rgb.x))
		config.set("hand_eye_calibration", "qy", str(base_rot_rgb.y))
		config.set("hand_eye_calibration", "qz", str(base_rot_rgb.z))
		config.add_section("base_cam_trans")
		config.set("base_cam_trans", "x",  str(base_trans_cam[0]))
		config.set("base_cam_trans", "y",  str(base_trans_cam[1]))
		config.set("base_cam_trans", "z",  str(base_trans_cam[2]))
		config.set("base_cam_trans", "qw", str(base_rot_cam[0]))
		config.set("base_cam_trans", "qx", str(base_rot_cam[1]))
		config.set("base_cam_trans", "qy", str(base_rot_cam[2]))
		config.set("base_cam_trans", "qz", str(base_rot_cam[3]))
		
		with open(curr_path + '/config/'+ self.calibration_file_name, 'w') as file:
			config.write(file)
		return 'finish'
	
	def on_enter(self, userdata):
		self.trans_A_list = userdata.camera_h_charuco
		if self.eye_in_hand_mode:
			print("------------------------------------------------------------------")
			self.trans_B_list = userdata.base_h_tool
		else:
			self.trans_B_list.header = userdata.base_h_tool.header
			print ("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
			print (userdata.base_h_tool)
			for transform in userdata.base_h_tool.transforms:
				trans = tf.transformations.quaternion_matrix([transform.rotation.x, transform.rotation.y,
															  transform.rotation.z, transform.rotation.w])
				trans[0:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
				trans = tf.transformations.inverse_matrix(trans)
				trans_B = Transform()
				trans_B.translation.x, trans_B.translation.y, trans_B.translation.z = trans[:3, 3]
				trans_B.rotation.x, trans_B.rotation.y, trans_B.rotation.z, \
					trans_B.rotation.w = tf.transformations.quaternion_from_matrix(trans)
				self.trans_B_list.transforms.append(trans_B)

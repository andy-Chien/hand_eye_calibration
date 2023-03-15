#!/usr/bin/env python
import rospy
import tf
from flexbe_core import EventState
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray

class FindCharucoState(EventState):
	"""
	Output a fixed pose to move.

	<= done									   Charuco pose has been received.
	<= go_compute							   Ready to compute the result.

	"""
	
	def __init__(self, base_link, tip_link):
		"""Constructor"""
		super(FindCharucoState, self).__init__(outcomes=['done', 'go_compute'], input_keys=['result_compute'], 
											   output_keys=['base_h_tool', 'camera_h_charuco'])
		self.base_link = base_link
		self.tip_link = tip_link
		self.tf_listener = tf.TransformListener()
		self.base_h_tool = TransformArray()
		self.camera_h_charuco = TransformArray()
		self.base_h_tool.header.frame_id = self.base_link
		self.camera_h_charuco.header.frame_id = 'calib_camera'
		self.enter_time = rospy.Time.now()

	def execute(self, userdata):
		if (rospy.Time.now() - self.enter_time).to_sec() > 2:
			rospy.logwarn('Can not get charuco board pose, abandon this position')
			return 'done'

		try:
			(base_trans_tool, base_rot_tool) = self.tf_listener.lookupTransform(self.base_link, self.tip_link, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn('lookupTransform for robot failed!, ' + self.base_link + ', ' + self.tip_link)
			return

		try:
			(camera_trans_charuco, camera_rot_charuco) = self.tf_listener.lookupTransform('/calib_camera', '/calib_charuco', rospy.Time.now())
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn('lookupTransform for charuco failed!')
			return

		trans = Transform()
		trans.translation.x = camera_trans_charuco[0]
		trans.translation.y = camera_trans_charuco[1]
		trans.translation.z = camera_trans_charuco[2]
		trans.rotation.x = camera_rot_charuco[0]
		trans.rotation.y = camera_rot_charuco[1]
		trans.rotation.z = camera_rot_charuco[2]
		trans.rotation.w = camera_rot_charuco[3]
		self.camera_h_charuco.transforms.append(trans)
		print(self.camera_h_charuco.transforms)
		print("============================")

		trans = Transform()
		trans.translation.x = base_trans_tool[0]
		trans.translation.y = base_trans_tool[1]
		trans.translation.z = base_trans_tool[2]
		trans.rotation.x = base_rot_tool[0]
		trans.rotation.y = base_rot_tool[1]
		trans.rotation.z = base_rot_tool[2]
		trans.rotation.w = base_rot_tool[3]
		self.base_h_tool.transforms.append(trans)
		print(self.base_h_tool.transforms)
		print("============================")


		if userdata.result_compute:
			userdata.base_h_tool = self.base_h_tool
			userdata.camera_h_charuco = self.camera_h_charuco
			return 'go_compute'
		else:
			return 'done'
			
	def on_enter(self, userdata):
		self.enter_time = rospy.Time.now()

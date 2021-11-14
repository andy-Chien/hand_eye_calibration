#!/usr/bin/env python
import rospy
from flexbe_core import EventState


class MoveRobotManuallyState(EventState):
    '''
    Implements a state that can be used to wait on timed process.

    -- wait_time 	float	Amount of time to wait in seconds.
    -- pose_num     int     Number of pose to calibrate

    #> result_compute bool  Ready to compute the result

    <= done					Indicates that the wait time has elapsed.
    '''

    def __init__(self, wait_time, pose_num):
        super(MoveRobotManuallyState, self).__init__(outcomes=['done'], output_keys=['result_compute'])
        self._wait = wait_time
        self._pose_num = pose_num
        self._pose_count = 0

    def execute(self, userdata):
        elapsed = rospy.get_rostime() - self._start_time
        userdata.result_compute = self._pose_count >= self._pose_num
        if elapsed.to_sec() > self._wait:
            return 'done'

    def on_enter(self, userdata):
        '''Upon entering the state, save the current time and start waiting.'''
        self._start_time = rospy.get_rostime()
        self._pose_count += 1
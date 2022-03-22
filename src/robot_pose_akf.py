#!/usr/bin/env python3

import rospy
from enum import Enum
from math import sqrt, floor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix

from typing import List, Union

__ROS_PARAM__ODOM_INPUTS = "robot_pose_akf/odom_inputs"
__ROS_PARAM__GNSS_INPUTS = "robot_pose_akf/gnss_inputs"
__ROS_PARAM__IMU_INPUTS = "robot_pose_akf/imu_inputs"
__VERY_LARGE_COVARIANCE = 99999


class INPUT_TYPE(Enum):
    ODOM = 0
    GNSS = 1
    IMU = 2
    CMD_VEL = 3

    def type_to_msg(_type):
        if _type == INPUT_TYPE.ODOM:
            return Odometry
        if _type == INPUT_TYPE.GNSS:
            return NavSatFix
        if _type == INPUT_TYPE.IMU:
            return Imu
        if _type == INPUT_TYPE.CMD_VEL:
            return Twist


class RobotInputWrapper:
    def __init__(self, topic, _type: INPUT_TYPE):
        self.topic = topic
        self.type = _type

        rospy.Subscriber(self.topic, INPUT_TYPE.type_to_msg(_type), self._cb)

    def _cb(self, msg):
        if self.type == INPUT_TYPE.ODOM:
            self._cb_odom(msg)
        if self.type == INPUT_TYPE.GNSS:
            self._cb_gnss(msg)
        if self.type == INPUT_TYPE.IMU:
            self._cb_imu(msg)
        if self.type == INPUT_TYPE.CMD_VEL:
            self._cb_cmd_vel(msg)

    def _cb_odom(self, msg: Odometry):
        pass

    def _cb_gnss(self, msg: NavSatFix):
        pass

    def _cb_imu(self, msg: Imu):
        pass

    def _cb_cmd_vel(self, msg: Twist):
        pass


class RobotPoseAkf:
    def __init__(self):
        # Check if there are any inputs configured (or default to
        # robot_pose_ekf interface), and create interfaces given those
        # parameters.
        self.inputs = self._init_inputs()
        self.odom_combined_publisher, self.tf_broadcaster = self._init_outputs()
        self._as_odom = self._init_odom()

    def get_odom(self):
        return self._as_odom

    def _init_odom(self):
        odom = Odometry()
        # 2 dimensional square array represented in 1d, so NxN is sqrt. We know it's 6x6 but this is cleaner I guess
        pose_covar_n = floor(sqrt(len(odom.pose.covariance)))

        # initialize covariance as very high values, since we don't have values for this yet.
        init_covar = [0 for _ in range(0, pose_covar_n * pose_covar_n)]
        for n in range(0, pose_covar_n):
            init_covar[n + (n * pose_covar_n)] = __VERY_LARGE_COVARIANCE
        odom.pose.covariance = init_covar
        odom.twist.covariance = init_covar

    def _init_inputs(self):
        # Any odom inputs?
        odom_input_str = None
        if rospy.has_param(__ROS_PARAM__ODOM_INPUTS):
            odom_input_str = str(rospy.get_param(__ROS_PARAM__ODOM_INPUTS))

        # Any GNSS (GPS/GLONASS/etc) inputs?
        gnss_input_str = None
        if rospy.has_param(__ROS_PARAM__GNSS_INPUTS):
            gnss_input_str = str(rospy.get_param(__ROS_PARAM__GNSS_INPUTS))

        # Any IMU inputs?
        imu_input_str = None
        if rospy.has_param(__ROS_PARAM__IMU_INPUTS):
            imu_input_str = str(rospy.get_param(__ROS_PARAM__IMU_INPUTS))

        odom_inputs = []
        gnss_inputs = []
        imu_inputs = []
        if odom_input_str is None and gnss_input_str is None and imu_input_str is None:
            # No inputs configured, let's assume the 'robot_pose_ekf' interface for legacy purposes
            odom_inputs = ["odom", "vo"]
            imu_inputs = ["imu_data"]
        else:
            odom_inputs = self._parse_topic_str(odom_input_str)
            gnss_inputs = self._parse_topic_str(gnss_input_str)
            imu_inputs = self._parse_topic_str(imu_input_str)

        inputs = []
        for odom_input in odom_inputs:
            inputs.append(RobotInputWrapper(odom_input, INPUT_TYPE.ODOM))
        for gnss_input in gnss_inputs:
            inputs.append(RobotInputWrapper(gnss_input, INPUT_TYPE.GNSS))
        for imu_input in imu_inputs:
            inputs.append(RobotInputWrapper(imu_input, INPUT_TYPE.IMU))
        inputs.append(RobotInputWrapper("cmd_vel", INPUT_TYPE.CMD_VEL))
        return inputs

    def _parse_topic_str(self, input_str: Union[str, None]) -> List[str]:
        if input_str is None:
            return []
        return input_str.strip("'\"").split(",")

    def _init_outputs(self):
        return (
            rospy.Publisher("odom_combined", PoseWithCovarianceStamped, queue_size=5),
            rospy.Publisher("", None, queue_size=5),  # TODO: tf broadcast
        )

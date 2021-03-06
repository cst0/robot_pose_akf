#!/usr/bin/env python3

import rospy
from tf import transformations

from enum import Enum
from math import sqrt, floor
from filterpy.kalman import KalmanFilter

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix

from typing import List, Union

__ROS_PARAM__ODOM_INPUTS = "robot_pose_akf/odom_inputs"
__ROS_PARAM__GNSS_INPUTS = "robot_pose_akf/gnss_inputs"
__ROS_PARAM__IMU_INPUTS = "robot_pose_akf/imu_inputs"
__VERY_LARGE_COVARIANCE = 99999


class INPUT_TYPE(Enum):
    OUTPUT = 0
    ODOM = 1
    GNSS = 2
    IMU = 3
    CMD_VEL = 4

    def type_to_msg(_type):
        if _type == INPUT_TYPE.ODOM:
            return Odometry
        if _type == INPUT_TYPE.GNSS:
            return NavSatFix
        if _type == INPUT_TYPE.IMU:
            return Imu
        if _type == INPUT_TYPE.CMD_VEL:
            return Twist
        if _type == INPUT_TYPE.OUTPUT:
            return None


class DataCollectionWrapper:
    def __init__(self, topic, _type: INPUT_TYPE):
        self.topic = topic
        self.type = _type
        self._as_odom = self._init_odom()
        self._initial_gnss: Union[None, NavSatFix] = None
        self._last_imu = Imu()

        if self.type is not INPUT_TYPE.OUTPUT:
            rospy.Subscriber(self.topic, INPUT_TYPE.type_to_msg(_type), self._cb)

    def get_odom(self):
        return self._as_odom

    def get_odom_as_state(self):
        """
        Return state vector and covariance.
        """
        o = self.get_odom()
        (o_ox, o_oy, o_oz) = transformations.euler_from_quaternion(
            o.pose.pose.orientation
        )
        return (
            [
                o.pose.pose.position.x,
                o.pose.pose.position.y,
                o.pose.pose.position.z,
                o_ox,
                o_oy,
                o_oz,
                o.twist.twist.linear.x,
                o.twist.twist.linear.y,
                o.twist.twist.linear.z,
                o.twist.twist.angular.x,
                o.twist.twist.angular.y,
                o.twist.twist.angular.z,
            ],
            [
                o.pose.covariance[0],
                o.pose.covariance[7],
                o.pose.covariance[14],
                o.pose.covariance[21],
                o.pose.covariance[28],
                o.pose.covariance[35],
                o.twist.covariance[0],
                o.twist.covariance[7],
                o.twist.covariance[14],
                o.twist.covariance[21],
                o.twist.covariance[28],
                o.twist.covariance[35],
            ],
        )

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
        self._as_odom = msg

    def _cb_gnss(self, msg: NavSatFix):
        rospy.logwarn_once("I have no idea if this will work.")
        if (
            msg.status.status >= 0
        ):  # we have some kind of satellite fix, so we can use this message
            if self._initial_gnss is None:
                self._initial_gnss = msg
            self._as_odom.pose.pose.position.x = (
                self._initial_gnss.latitude - msg.latitude
            )
            self._as_odom.pose.pose.position.y = (
                self._initial_gnss.longitude - msg.longitude
            )
            self._as_odom.pose.pose.position.z = (
                self._initial_gnss.altitude - msg.altitude
            )
            self._as_odom.pose.covariance = msg.position_covariance

    def _cb_imu(self, msg: Imu):
        # TODO: set covar off of these vals
        self._as_odom.pose.pose.orientation = msg.orientation
        self._as_odom.twist.twist.angular = msg.angular_velocity
        self._as_odom.twist.twist.linear.x = (
            self._last_imu.linear_acceleration.x + msg.linear_acceleration.x
        ) / 2
        self._as_odom.twist.twist.linear.y = (
            self._last_imu.linear_acceleration.y + msg.linear_acceleration.y
        ) / 2
        self._as_odom.twist.twist.linear.z = (
            self._last_imu.linear_acceleration.z + msg.linear_acceleration.z
        ) / 2
        self._last_imu = msg

    def _cb_cmd_vel(self, msg: Twist):
        # twist is twist
        self._as_odom.twist.twist = msg
        # we have zero uncertainty about this message (it's cmd_vel), so set covariance accordingly
        self._as_odom.twist.covariance = [
            0 for _ in range(0, len(self._as_odom.twist.covariance))
        ]
        # not touching the other values since we don't know anything about position from a twist

    def _init_odom(self) -> Odometry:
        odom = Odometry()
        # 2 dimensional square array represented in 1d, so NxN is sqrt. We know it's 6x6 but this is cleaner I guess
        pose_covar_n = floor(sqrt(len(odom.pose.covariance)))

        # initialize covariance as very high values, since we don't have values for this yet.
        init_covar = [0 for _ in range(0, pose_covar_n * pose_covar_n)]
        for n in range(0, pose_covar_n):
            init_covar[n + (n * pose_covar_n)] = __VERY_LARGE_COVARIANCE
        odom.pose.covariance = init_covar
        odom.twist.covariance = init_covar

        return odom


class RobotPoseAkf:
    def __init__(self):
        # Check if there are any inputs configured (or default to
        # robot_pose_ekf interface), and create interfaces given those
        # parameters.
        self.inputs = self._init_inputs()
        self.odom_combined_publisher, self.tf_broadcaster = self._init_outputs()
        self.state = DataCollectionWrapper(None, INPUT_TYPE.OUTPUT)

        self.timer = rospy.Timer(rospy.Rate(30), self.cb_timer)

    def cb_timer(self, event):
        pass

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
            inputs.append(DataCollectionWrapper(odom_input, INPUT_TYPE.ODOM))
        for gnss_input in gnss_inputs:
            inputs.append(DataCollectionWrapper(gnss_input, INPUT_TYPE.GNSS))
        for imu_input in imu_inputs:
            inputs.append(DataCollectionWrapper(imu_input, INPUT_TYPE.IMU))
        inputs.append(DataCollectionWrapper("cmd_vel", INPUT_TYPE.CMD_VEL))
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

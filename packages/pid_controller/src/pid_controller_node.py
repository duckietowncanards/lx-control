#!/usr/bin/env python3
import os
import time
from typing import Optional

import numpy as np
import rospy
import tf
import yaml
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, EpisodeStart
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from solution.pid_controller import PIDController


class PIDControllerNode(DTROS):
    """
    Computes an estimate of the Duckiebot pose using the wheel encoders.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the ROS node
    Configuration:

    Publisher:
        ~joy_mapper/car_cmd (:obj:`PoseStamped`): The computed position
    Subscribers:
        ~/pose (:obj:`Odometry`):
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(PIDControllerNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.log("Initializing...")
        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")
        self.controller = PIDController()
        self.time_now: float = 0.0
        self.time_last_step: float = 0.0

        # fixed robot linear velocity - starts at zero so the activities start on command
        self.v_0 = 0.0
        # reference y for PID lateral control activity - zero so can set interactively at runtime
        self.y_ref = 0.0

        # initial reference signal for heading control activity
        self.theta_ref = np.deg2rad(0.0)
        # initializing omega command to the robot
        self.omega = 0.0


        # spins only parts of this code depending on the icons pressed on the VNC desktop
        self.PID_HEADING = False
        self.PID_OFFSET = False


        # Defining subscribers:

        # select the current activity
        rospy.Subscriber(f"/{self.veh}/activity_name", String, self.cbActivity, queue_size=1)
        rospy.Subscriber(f"/{self.veh}/PID_parameters", String, self.cbPIDparam, queue_size=1)

        pose_topic = f"/{self.veh}/pose"
        rospy.Subscriber(pose_topic, Odometry, self.cbPose, queue_size=1)

        # Command publisher
        car_cmd_topic = f"/{self.veh}/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic, Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        # Wait until the encoders data is received, then start the controller
        self.STOP = False


        self.log("Initialized.")

    def resetParameters(self):
        # Initializing the PID controller parameters
        self.time_now: float = 0.0
        self.time_last_step: float = 0.0

        # fixed robot linear velocity - starts at zero so the activities start on command
        self.v_0 = 0.0
        # reference y for PID lateral control activity - zero so can set interactively at runtime
        self.y_ref = 0.0


    # Emergency stop / interactive pane for PID activity and exercise
    def cbPIDparam(self, msg):
        PID_parameters = msg.data

        if PID_parameters == "STOP":
            self.publishCmd(0, 0)
            self.STOP = True
            self.log("STOP")
            return

        PID_parameters = PID_parameters.split(";")
        self.log(PID_parameters)

        # ref is angle in activity
        if self.PID_HEADING:
            self.theta_ref = np.deg2rad(float(PID_parameters[0]))

        # ref is lateral position in exercise
        elif self.PID_OFFSET:
            self.y_ref = float(PID_parameters[0])

        self.v_0 = float(PID_parameters[1])
        #update the controller gains
        self.controller.SetGains(kp=float(PID_parameters[2]),
                                 ki=float(PID_parameters[3]),
                                 kd=float(PID_parameters[4])
                                 )
        self.STOP = False

    def cbActivity(self, msg):
        """
        Call the right functions according to desktop icon the parameter.
        """

        self.publishCmd(0, 0)
        self.PID_HEADING = False
        self.PID_OFFSET = False

        self.log("")
        self.log(f"Received activity {msg.data}")
        self.log("")

        self.PID_HEADING = msg.data == "pid_heading"
        self.PID_OFFSET = msg.data == "pid_offset"

    def cbPose(self, pose_msg):
        """


        Args:
            pose_msg: pose as an Odometry message

        Returns:
            publishes control actions

        """
        if self.STOP:
            self.publishCmd(0, 0)
            return

        self.y_curr = pose_msg.pose.pose.position.y
        quat = (
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z,
            pose_msg.pose.pose.orientation.w,
        )
        euler = tf.transformations.euler_from_quaternion(quat)

        # update time
        self.time_now = max(self.time_now, pose_msg.header.stamp.to_sec())


        # We are calling theta the heading here (not the normal psi)
        self.theta_curr = euler[2]

        # run the controller only in appropriate activities
        if self.PID_HEADING or self.PID_OFFSET:
            self.Controller()

    def Controller(self):
        """
        Calculate theta and perform the control actions given by the PID
        """

        delta_time = self.time_now - self.time_last_step
        self.time_last_step = self.time_now

        if self.PID_HEADING:
            v, omega = self.controller.HeadingControl(
                self.v_0, self.theta_ref, self.theta_curr, delta_time
            )

        elif self.PID_OFFSET:
            v, omega = self.controller.OffsetControl(
                self.v_0, self.y_ref, self.y_curr, delta_time
            )
        else:
            v, omega = 0.0 , 0.0

        self.publishCmd(v, omega)

    def publishCmd(self, v, omega):
        """
        Publishes a car command message.

        Args:
            v       (:obj:`double`): linear velocity
            omega   (:obj:`double`): angular velocity
        """

        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.from_sec(self.time_now)

        car_control_msg.v = v
        car_control_msg.omega = omega
        # save omega in case of STOP
        self.omega = omega

        self.pub_car_cmd.publish(car_control_msg)

    def onShutdown(self):
        super(PIDControllerNode, self).on_shutdown()


if __name__ == "__main__":
    # Initialize the node
    pid_controller_node = PIDControllerNode(node_name="pid_controller_node")
    # Keep it spinning
    rospy.spin()

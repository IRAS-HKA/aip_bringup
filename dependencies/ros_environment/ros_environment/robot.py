# -*- coding: utf-8 -*-
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from iras_interfaces.srv import MoveToPose as PoseSrv
from iras_interfaces.srv import MoveToJointPosition as JointPositionSrv
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion, Point
from std_srvs.srv import Trigger
import copy
from manipulation_tasks.transform import Affine

from ros_environment.lib.base_node import BaseNode


# TODO use manipulation_tasks protocol for Robot


class RobotClient:
    """ 
    TODO description
    """

    def __init__(self, node: Node = None, is_simulation: bool = False) -> None:
        """
        TODO docstring

        Returns
        -------
        None.

        """
        if node is None:
            self.node = BaseNode("robot_client", is_simulation)
        else:
            self.node = node
        self.move_cli = self.node.create_client(PoseSrv, "/move_to_pose")
        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("move service not available, waiting some more ...")
        self.node.get_logger().info("move service available")

        self.move_joint_cli = self.node.create_client(JointPositionSrv, "/move_to_joint_position")
        while not self.move_joint_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("move joint service not available, waiting some more ...")
        self.node.get_logger().info("move joint service available")

        self.is_simulation = is_simulation
        if not self.is_simulation:
            self.opem_cli = self.node.create_client(Trigger, "/open_gripper")
            while not self.opem_cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info("opem service not available, waiting again...")
            self.close_cli = self.node.create_client(Trigger, "/close_gripper")
            while not self.close_cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info("close service not available, waiting again...")
        # TODO where to get home pose from
        self.home_pose = Affine((-0.002, -0.052, 1.278), (0, 0.86, 0, 0.511))
        self.home_position = [np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2]

        self.start_servo_client = self.node.create_client(
            Trigger, "servo_node/start_servo")

        self.stop_servo_client = self.node.create_client(
            Trigger, "servo_node/stop_servo")

        while not self.start_servo_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('start_servo_client not available, waiting again...')

        while not self.stop_servo_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('stop_servo_client not available, waiting again...')

    def home(self) -> bool:
        """
        TODO docstring

        Returns
        -------
        None.

        """
        # move_success = self.ptp(self.home_pose)
        move_success = self.ptp_joint(self.home_position)
        # gripper_success = self.open_gripper()
        return move_success  # and gripper_success

    def ptp(self, pose: Affine) -> bool:
        """
        TODO docstring

        Parameters
        ----------
        pose : Affine
            DESCRIPTION.

        Returns
        -------
        None.

        """
        future = self.send_move_request(self.affine_to_pose(pose), False)
        response = self.wait_for_response(future)
        return response.success

    def ptp_joint(self, joint_positions: List[float]) -> bool:
        """
        TODO docstring

        Parameters
        ----------
        pose : Affine
            DESCRIPTION.

        Returns
        -------
        None.
        :param joint_positions:

        """
        future = self.send_move_joint_request(joint_positions)
        response = self.wait_for_response(future)
        return response.success

    def lin(self, pose: Affine) -> bool:
        """
        TODO docstring

        Parameters
        ----------
        pose : Affine
            DESCRIPTION.

        Returns
        -------
        None.

        """

        future = self.send_move_request(self.affine_to_pose(pose), True)
        response = self.wait_for_response(future)
        return response.success

    def open_gripper(self) -> bool:
        """
        TODO docstring

        Returns
        -------
        bool
            DESCRIPTION.

        """
        s = True
        if not self.is_simulation:
            future = self.send_open_request()
            response = self.wait_for_response(future)
            s = response.success
        if not s:
            self.node.get_logger().info("Opening gripper unsuccessful.")
        return s

    def close_gripper(self) -> bool:
        """
        TODO docstring

        Returns
        -------
        bool
            DESCRIPTION.

        """
        s = True
        if not self.is_simulation:
            future = self.send_close_request()
            response = self.wait_for_response(future)
            s = response.success
        if not s:
            self.node.get_logger().info("Closing gripper unsuccessful.")
        return s

    def send_move_request(self, pose, cart=False) -> Future:
        """
        TODO docstring

        Parameters
        ----------
        pose : TYPE
            DESCRIPTION.
        cart : TYPE, optional
            DESCRIPTION. The default is False.

        Returns
        -------
        Future
            DESCRIPTION.

        """
        req = PoseSrv.Request()
        req.pose = pose
        req.cart = cart
        future = self.move_cli.call_async(req)
        return future

    def send_move_joint_request(self, joint_positions) -> Future:
        """
        TODO docstring

        Parameters
        ----------
        pose : TYPE
            DESCRIPTION.
        cart : TYPE, optional
            DESCRIPTION. The default is False.

        Returns
        -------
        Future
            DESCRIPTION.

        """
        req = JointPositionSrv.Request()
        req.joint_position = joint_positions
        future = self.move_joint_cli.call_async(req)
        return future

    def send_open_request(self) -> Future:
        """
        TODO docstring

        Returns
        -------
        Future
            DESCRIPTION.

        """
        req = Trigger.Request()
        future = self.opem_cli.call_async(req)
        return future

    def send_close_request(self) -> Future:
        """
        TODO docstring

        Returns
        -------
        Future
            DESCRIPTION.

        """
        req = Trigger.Request()
        future = self.close_cli.call_async(req)
        return future

    def wait_for_response(self, future):
        """
        TODO docstring

        Parameters
        ----------
        future : TYPE
            DESCRIPTION.

        Returns
        -------
        response : TYPE
            DESCRIPTION.

        """
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.node.get_logger().info(
                        'Service call failed %r' % (e,))
                    return None
                else:
                    return response

    @staticmethod
    def affine_to_pose(affine: Affine) -> PoseMsg:
        """
        TODO docstring

        Parameters
        ----------
        affine : Affine
            DESCRIPTION.

        Returns
        -------
        PoseMsg
            DESCRIPTION.

        """
        # TODO move to a new file util.py?
        t = affine.translation
        r = affine.quat
        msg = PoseMsg(
            position=Point(x=float(t[0]), y=float(t[1]), z=float(t[2])),
            orientation=Quaternion(x=float(r[0]), y=float(r[1]), z=float(r[2]), w=float(r[3])))
        return msg

    @staticmethod
    def pose_to_affine(pose: PoseMsg) -> Affine:
        """
        TODO docstring

        Parameters
        ----------
        pose : PoseMsg
            DESCRIPTION.

        Returns
        -------
        Affine
            DESCRIPTION.
        """
        # TODO move to a new file util.py?
        affine = Affine(
            translation=[pose.position.x, pose.position.y, pose.position.z],
            rotation=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )
        return affine

    def enable_servo(self) -> bool:
        trigger = Trigger.Request()
        future = self.start_servo_client.call_async(trigger)
        response = self.wait_for_response(future)
        return response

    def disable_servo(self) -> bool:
        trigger = Trigger.Request()
        future = self.stop_servo_client.call_async(trigger)
        response = self.wait_for_response(future)
        return response

    def destroy_node(self) -> None:
        self.node.destroy_node()

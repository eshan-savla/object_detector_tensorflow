#!/usr/bin/env python3

import socket

import rclpy
from rclpy.node import Node
from diagnostic_updater import Updater
from diagnostic_msgs.msg import DiagnosticStatus


class Diagnostics(object):

    def __init__(self,
                 node: Node,
                 message: str = "Statistic"):

        def create_diagnostic(status):

            status.summary(DiagnosticStatus.OK, message)

            for key, value in self.diagnostic_dict.items():
                status.add(key, value)

            return status

        self.diagnostic_dict = {}

        self.diagnostic_updater = Updater(node)
        self.diagnostic_updater.setHardwareID(socket.gethostname())

        self.diagnostic_updater.add(node.get_name(), create_diagnostic)

    def update(self,
               key: str,
               value: object):

        self.diagnostic_dict[key] = value
        self.diagnostic_updater.update()

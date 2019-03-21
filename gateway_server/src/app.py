#!/usr/bin/env python
# -*-encoding:utf-8-*-
import ConfigParser
import json
import threading, thread
from Queue import Queue, Empty

import rosnode
import rospy
from enum import IntEnum

# from common import Timer, EventType, Event
from connector import Connector
from monitor import Monitor
from twisted.internet.protocol import Protocol, ReconnectingClientFactory, Factory
from twisted.protocols.basic import LineReceiver
from twisted.internet import reactor
from geometry_msgs.msg import PoseStamped
import json

# HOST = 'localhost'
PORT = 9999


class message_handler():
    def __init__(self):
        self._init_publisher()

    # TODO: 不同的车发布不同的消息,根据car*进行区分
    def _init_publisher(self):
        self._init_current_pose_publisher()

    def _init_current_pose_publisher(self):
        self.pub_car_pose = []
        for i in range(10):
            topic = "/car" + str(i) + "/current_pose"
            pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
            self.pub_car_pose.append(pub)

    def _current_pose_cb(self, data_dict):
        current_pose = PoseStamped()
        current_pose.header.frame_id = "map"
        current_pose.header.stamp.secs = data_dict['timestamp'] / 1000000000
        current_pose.header.stamp.nsecs = data_dict['timestamp'] % 1000000000
        current_pose.pose.position.x = data_dict['data']['pose']['position'][0]
        current_pose.pose.position.y = data_dict['data']['pose']['position'][1]
        current_pose.pose.position.z = data_dict['data']['pose']['position'][2]
        current_pose.pose.orientation.x = data_dict['data']['pose']['orientation'][0]
        current_pose.pose.orientation.y = data_dict['data']['pose']['orientation'][1]
        current_pose.pose.orientation.z = data_dict['data']['pose']['orientation'][2]
        current_pose.pose.orientation.w = data_dict['data']['pose']['orientation'][3]
        car_id = int(data_dict['carId'][-5:])
        self.pub_car_pose[car_id].publish(current_pose)


class MonitorProtocol(LineReceiver):
    def connectionMade(self):
        peer = self.clnt = self.transport.getPeer().host
        print("...connected from : %s", peer)
        self.message_handler = message_handler()

    def lineReceived(self, line):
        data_dict = json.loads(line)
        if data_dict['type'] == "CurrentPose":
            self.message_handler._current_pose_cb(data_dict)

    def connectionLost(self, reason):
        print("connection lost")


class GatewayServer(Factory):
    protocol = MonitorProtocol

    def clientConnectionFailed(self, connector, reason):
        print("Connection Lost, Reason: ", reason)
        ReconnectingClientFactory.clientConnectionLost(self, connector, reason)

    def clientConnectionLost(self, connector, reason):
        print("Connection Failed, Reason: ", reason)
        ReconnectingClientFactory.clientConnectionFailed(self, connector, reason)


class APP():
    def run(self):
        rospy.init_node("gataway_server")
        factory = GatewayServer()
        print("waiting for connection...")
        reactor.listenTCP(PORT, factory)
        reactor.run()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            reactor.stop()
        except Exception as e:
            print e
        return self


if __name__ == "__main__":
    app = APP()
    try:
        app.run()
    except Exception as e:
        print e

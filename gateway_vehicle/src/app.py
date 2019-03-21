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
from twisted.internet.protocol import Protocol, ReconnectingClientFactory, ClientFactory
from twisted.protocols.basic import LineReceiver
from twisted.internet import reactor
from geometry_msgs.msg import PoseStamped
import json


class Config:
    def __init__(self, config_file):
        cf = ConfigParser.ConfigParser()
        cf.read(config_file)
        self.params = dict()
        self.params['version'] = float(cf.get("protocol", "version"))
        self.params['vin'] = cf.get("vehicle", "vin")
        self.params['host'] = cf.get("server", "host")
        self.params['port'] = int(cf.get("server", "port"))

    @property
    def host(self):
        return self.params['host']

    @host.setter
    def host(self, host):
        self.params['host'] = host

    @property
    def port(self):
        return self.params['port']

    @port.setter
    def port(self, port):
        self.params['port'] = port

    @property
    def version(self):
        return self.params['version']

    @version.setter
    def version(self, version):
        self.params['version'] = version

    @property
    def vin(self):
        return self.params['vin']

    @vin.setter
    def vin(self, vin):
        self.params['vin'] = vin

    def to_json(self):
        return self.params


class msg_handler:
    def __init__(self):
        config_file = rospy.get_param("~config_file")
        self.config = Config(config_file)
        rospy.Subscriber("/vehicle_state", Message_type, self._update_state)

    def _update_state(self, msg):
        self.state = msg.
        self.velocity = msg.
        self.angle = msg.

    def _init_data(self):
        data = {}
        data['version'] = self.config.version
        data['crcCode'] = 0x1234
        # data['type'] = self.config['type']
        data['ack'] = 0
        data['requestId'] = -1
        data['carId'] = self.config.vin
        data['data'] = {}
        return data

    def _current_pose_cb(self, msg):
        assert isinstance(msg, PoseStamped)
        data = self._init_data()
        data['type'] = "CurrentPose"
        data['timestamp'] = int(msg.header.stamp.to_nsec())
        pose = {}
        position = msg.pose.position
        orientation = msg.pose.orientation
        pose['position'] = [position.x, position.y, position.z]
        pose['orientation'] = [orientation.x, orientation.y, orientation.z, orientation.w]
        data['data']['pose'] = pose
        return json.dumps(data)


class monitor_protocol(Protocol):
    def connectionMade(self):
        self.buffer = b''
        self.delimiter = b'\r\n'
        self._add_sub()
        self.handler = msg_handler()
        print("connected to server.")

    def _add_sub(self):
        rospy.Subscriber("/ndt/current_pose", PoseStamped,self._current_pose_cb)

    def _current_pose_cb(self,msg):
        data_json = self.handler._current_pose_cb(msg)
        self.sendLine(data_json)

    def lineReceived(self, line):
        print("Line received")
        return

    def sendLine(self, data_json):
        # self.transport.write(data_json+self.delimiter)
        return self.transport.writeSequence((data_json, self.delimiter))


class GatewayClient(ReconnectingClientFactory, LineReceiver):
    protocol = monitor_protocol

    def clientConnectionFailed(self, connector, reason):
        print("Connection Lost, Reason: ", reason)
        ReconnectingClientFactory.clientConnectionLost(self, connector, reason)

    def clientConnectionLost(self, connector, reason):
        print("Connection Failed, Reason: ", reason)
        ReconnectingClientFactory.clientConnectionFailed(self, connector, reason)

class APP:
    def __init__(self):
        rospy.init_node('Gateway', log_level=rospy.DEBUG)
        config_file = rospy.get_param("~config_file")
        self.config = Config(config_file)

    def run(self):
        server_host = self.config.host
        server_port = self.config.port
        print("trying connect to %s:%s", (server_host, server_port))
        reactor.connectTCP(server_host, int(server_port), GatewayClient())
        reactor.run()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        reactor.stop()
        return self

    def shutdown(self):
        try:
            reactor.stop()
        except Exception as e:
            print e


if __name__ == '__main__':
    app = APP()
    try:
        app.run()
    except Exception as e:
        app.shutdown()
        print("gateway shut down")
        rospy.loginfo(e)

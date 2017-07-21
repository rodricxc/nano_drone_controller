#!/usr/bin/env python

import rospy
from nano_drone_controller.msg import Nanoquad
import socket
import time
import math
from threading import Lock


HANDSHAKE_DATA = bytearray([0x49, 0x54, 0x64, 0x00, 0x00, 0x00, 0x5D, 0x00, 0x00, 0x00, 0x81, 0x85, 0xFF, 0xBD, 0x2A, 0x29, 0x5C, 0xAD, 0x67, 0x82, 0x5C, 0x57, 0xBE, 0x41, 0x03, 0xF8, 0xCA, 0xE2, 0x64, 0x30, 0xA3, 0xC1,
            0x5E, 0x40, 0xDE, 0x30, 0xF6, 0xD6, 0x95, 0xE0, 0x30, 0xB7, 0xC2, 0xE5, 0xB7, 0xD6, 0x5D, 0xA8, 0x65, 0x9E, 0xB2, 0xE2, 0xD5, 0xE0, 0xC2, 0xCB, 0x6C, 0x59, 0xCD, 0xCB, 0x66, 0x1E, 0x7E, 0x1E,
            0xB0, 0xCE, 0x8E, 0xE8, 0xDF, 0x32, 0x45, 0x6F, 0xA8, 0x42, 0xEE, 0x2E, 0x09, 0xA3, 0x9B, 0xDD, 0x05, 0xC8, 0x30, 0xA2, 0x81, 0xC8, 0x2A, 0x9E, 0xDA, 0x7F, 0xD5, 0x86, 0x0E, 0xAF, 0xAB, 0xFE,
            0xFA, 0x3C, 0x7E, 0x54, 0x4F, 0xF2, 0x8A, 0xD2, 0x93, 0xCD])

START_DRONE_DATA = bytearray([0xCC, 0x7F, 0x7F, 0x0, 0x7F, 0x0, 0x7F, 0x33])


class NanoDriver(object):
    def __init__(self, show_output=False):
        super(NanoDriver, self).__init__()
        self.load_config()
        self.ros_init()
        self.init_network_config()
        self.init_data()

    def init_data(self):
        """ data_control = {'throttle', 'roll', 'pitch', 'yaw'} """
        self.control = {'throttle': -1., 'roll': 0., 'pitch': 0., 'yaw': 0.}
        self.data_locker = Lock()

    def init_network_config(self):
        self._ip = '172.16.10.1'
        self._tcp_port = 8888
        self._udp_port = 8895

    def load_config(self):
        self.param = dict()
        self.param["frequency"] = rospy.get_param("/nano_drone_driver/frequency")

    def ros_init(self):
        rospy.init_node('nano_drone_driver', anonymous=False)
        self.rate = rospy.Rate(self.param["frequency"])  # 1000hz
        self.subscriber_commander = rospy.Subscriber("/nano_drone_driver/commander", Nanoquad, self.commander_callback)

    def commander_callback(self, data):
        """ data = {'throttle', 'roll', 'pitch', 'yaw'} """
        #print "Command received"
        self.update_control(data)

    def update_control(self, d):
        self.data_locker.acquire()
        self.control["throttle"], self.control["roll"], self.control["pitch"], self.control["yaw"] = d.throttle, -d.roll, d.pitch, -d.yaw
        self.data_locker.release()

    def remap(self, val):
        return min(int((val+1.0)*128.0), 255)

    def get_controls_converted(self):
        self.data_locker.acquire()
        to_return = self.control
        self.data_locker.release()
        # remap the interval [-1.0,1.0] to [0, 254]
        to_return = {key: self.remap(value) for key, value in to_return.items()}
        return to_return


    def connect(self):
        connected = False
        trys = 0

        while not connected and trys < 20 and not rospy.is_shutdown():
            try:
                trys += 1
                self.connect_tcp()
                self.connect_udp()
                connected = True
            except:
                connected = False
                print "ERROR on connecting to Nano Quad at {}. Trying again in 5 seconds".format(self._ip)
                time.sleep(5.0)

        if rospy.is_shutdown():
            exit(0)
        if not connected:
            raise Exception("Network error. Address not reachable")

    def connect_tcp(self):  # handshake
        print("Starting Handshake...")
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self._ip, self._tcp_port))
        self.tcp_socket.send(HANDSHAKE_DATA)
        print("Handshake done!")

    def connect_udp(self):
        print("Starting drone...")
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.connect((self._ip, self._udp_port))
        self.droneCmd = START_DRONE_DATA[:]
        self.udp_socket.send(self.droneCmd)
        print("Drone started!")

    def checksum(self, data):
        return_data = (data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5]) & 0xFF;
        return return_data

    def disconnect(self):
        print("Disconnecting...")
        self.udp_socket.close()
        self.tcp_socket.close()
        print("Disconnected!")

    def cmd(self, r=127, p=127, t=15, y=127):  # roll, pitch, throttle, yaw
        # time.sleep(0.01)
        self.droneCmd[1] = r
        self.droneCmd[2] = p
        self.droneCmd[3] = t
        self.droneCmd[4] = y
        self.droneCmd[6] = self.checksum(self.droneCmd)
        self.udp_socket.send(self.droneCmd)

    def stop(self):
        self.udp_socket.send(START_DRONE_DATA)

    def send_commands(self):
        cmds = self.get_controls_converted()
        self.cmd(r=cmds["roll"], p=cmds["pitch"], t=cmds["throttle"], y=cmds["yaw"])

    def run(self):
        self.connect()
        self.rate.sleep()

        while not rospy.is_shutdown():
            #t = time.time()
            self.send_commands()
            #print "free time: ", time.time() - t
            self.rate.sleep()



if __name__ == "__main__":
    try:
        nano = NanoDriver()
        nano.run()
        print "[Status]: Exiting node 'nano_drone_driver'."
        nano.stop()
        nano.disconnect()
    except rospy.ROSInterruptException:
        pass

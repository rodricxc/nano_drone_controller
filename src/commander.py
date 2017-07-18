#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from nano_drone_controller.msg import Nanoquad


class NanoController(object):
    def __init__(self, show_output=False):
        super(NanoController, self).__init__()
        self.load_config()
        self.ros_init()

    def load_config(self):
        self.param = dict()
        self.param["frequency"] = rospy.get_param("/nano_drone_controller/frequency")
        self.param["channel_throttle"] = rospy.get_param("/nano_drone_controller/channel_throttle")
        self.param["channel_yaw"] = rospy.get_param("/nano_drone_controller/channel_yaw")
        self.param["channel_pitch"] = rospy.get_param("/nano_drone_controller/channel_pitch")
        self.param["channel_roll"] = rospy.get_param("/nano_drone_controller/channel_roll")

        """ ['throttle', 'roll', 'pitch', 'yaw'] """
        self.nano_msg = [-1.0, 0.0, 0.0, 0.0]

    def ros_init(self):
        rospy.init_node('nano_drone_controller', anonymous=False)
        self.rate = rospy.Rate(self.param["frequency"])  # 100hz
        self.subscriber_joy = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.publisher_commander = rospy.Publisher('/nano_drone_driver/commander', Nanoquad, queue_size=1)

    def joy_callback(self, data):
        # print "Joy:", data.axes
        """['throttle', 'roll', 'pitch', 'yaw']"""
        self.nano_msg[0] = data.axes[self.param["channel_throttle"]]
        self.nano_msg[1] = data.axes[self.param["channel_roll"]]
        self.nano_msg[2] = data.axes[self.param["channel_pitch"]]
        self.nano_msg[3] = data.axes[self.param["channel_yaw"]]
        self.publish_commander()

    def publish_commander(self):
        self.publisher_commander.publish(self.nano_msg[0], self.nano_msg[1], self.nano_msg[2], self.nano_msg[3])

    def run(self):

        for i in range(10):
            self.publish_commander()
            self.rate.sleep()

        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":
    try:
        nano = NanoController()
        nano.run()
        print "[Status]: Exiting node 'nano_drone_controller'."
    except rospy.ROSInterruptException:
        pass

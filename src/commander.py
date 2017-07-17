#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy


class NanoController(object):
    def __init__(self, show_output=False):
        super(NanoController, self).__init__()
        self.load_config()
        self.ros_init()

    def load_config(self):
        self.param = dict()
        self.param["frequency"] = rospy.get_param("/nano_drone_controller/frequency")

    def ros_init(self):
        rospy.init_node('nano_drone_controller', anonymous=False)
        self.rate = rospy.Rate(self.param["frequency"])  # 100hz
        self.sub_odom = rospy.Subscriber("/joy", Joy, self.joy_callback)

    def joy_callback(self, data):
        print "Joy:", data.axes

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":
    try:
        nano = NanoController()
        nano.run()
        print "[Status]: Exiting node 'nano_drone_controller'."
    except rospy.ROSInterruptException:
        pass

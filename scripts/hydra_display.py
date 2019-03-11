#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import NavSatFix

class HydraDisplay:
    def __init__(self):
        rospy.init_node('hydra_display')
        self.nav_sat_fix_sub = rospy.Subscriber('gps/data', NavSatFix, self.nav_sat_fix)
        self.imu_sub = rospy.Subscriber('imu/data', Imu, self.imu)
        self.magnetic_field_sub = rospy.Subscriber('imu/mag', MagneticField, self.magnetic_field)
    def nav_sat_fix(self, msg):
        pass
    def imu(self, msg):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (r, p, y) = euler_from_quaternion(q)
        rospy.loginfo('%f, %f, %f' % (r, p, y))
    def magnetic_field(self, msg):
        pass
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        hd = HydraDisplay()
        hd.run()
    except rospy.ROSInterruptException:
        pass

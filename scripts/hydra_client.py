#!/usr/bin/env python


import rospy
import socket
import json
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

class HydraClient:
    def __init__(self):

        rospy.init_node('hydra_client')
        self.host = rospy.get_param("~host", default="127.0.0.1")
        rospy.set_param("~host", self.host)
        self.port = int(rospy.get_param("~port", default=5555))
        rospy.set_param("~port", self.port)
        self.frame = rospy.get_param("~frame", default="gps")
        rospy.set_param("~frame", self.port)
        
        self.nav_sat_fix_pub = rospy.Publisher('gps/data', NavSatFix, queue_size=10)        
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        self.magnetic_field_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
    def run(self):
        rospy.loginfo('Connecting to %s:%d' % (self.host, self.port))
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.host, self.port))
        while not rospy.is_shutdown():
            raw = self.s.recv(1024)
            try:
                data = json.loads(raw)
            except ValueError as e:
                # In this case we probably got more than one json 
                rospy.logwarn('Unable to parse JSON message ' + str(e))
                print(raw)
            else:
                # No exception, continue parsing
                self.parse_general(data)
            
    def parse_general(self, data):
        if ('type' in data):
            if data['type'] == 'imu':
                if ('data' in data):
                    self.parse_imu(data['data'])
                else:
                    rospy.logwarn('Got message with no data')
            elif data['type'] == 'gps':
                if ('data' in data):
                    self.parse_gps(data['data'])
                else:
                    rospy.logwarn('Got message with no data')
            else:
                rospy.logwarn('Unknown message type')
        else:
            rospy.logwarn('Got message with no type')
    def parse_imu(self, data):
        # {
        #  'acc': [0.19589658081531525, -0.18210914731025696, 9.45071792602539], 
        #  'gyro': [0.008880757726728916, 0.012528236024081707, -0.021537888795137405], 
        #  'mag': [-0.00010609201126499102, -0.00013743649469688535, -6.562667840626091e-05], 
        #  'quat': [0.31689611077308655, -0.01368665136396885, 0.002746865153312683, -0.9483575224876404], 
        #  'rpy': [-2.795783042907715, -1.3877670764923096, -143.03599548339844], 
        #  'ts': 6539060
        # }
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame
        msg.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        msg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        msg.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        mag_msg = MagneticField()
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.header.frame_id = self.frame
        mag_msg.magnetic_field_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        try:
            
            msg.angular_velocity.x = data['gyro'][0]
            msg.angular_velocity.y = data['gyro'][1]
            msg.angular_velocity.z = data['gyro'][2]
            msg.angular_velocity_covariance[0]  = 0.02
            msg.angular_velocity_covariance[4]  = 0.02
            msg.angular_velocity_covariance[8]  = 0.02
            msg.linear_acceleration.x = data['acc'][0]
            msg.linear_acceleration.y = data['acc'][1]
            msg.linear_acceleration.z = data['acc'][2]
            msg.linear_acceleration_covariance[0]  = 0.04
            msg.linear_acceleration_covariance[4]  = 0.04
            msg.linear_acceleration_covariance[8]  = 0.04
            msg.orientation.x = data['quat'][0]
            msg.orientation.y = data['quat'][1]
            msg.orientation.z = data['quat'][2]
            msg.orientation.w = data['quat'][3]
            msg.orientation_covariance[0] = 0.0025
            msg.orientation_covariance[1] = 0.0025
            msg.orientation_covariance[2] = 0.0025
            mag_msg.magnetic_field.x = data['mag'][0]
            mag_msg.magnetic_field.y = data['mag'][1]
            mag_msg.magnetic_field.z = data['mag'][2]
            mag_msg.magnetic_field_covariance[0]  = 0.0
            mag_msg.magnetic_field_covariance[4]  = 0.0
            mag_msg.magnetic_field_covariance[8]  = 0.0
        except KeyError as e:
            rospy.logwarn('Unable to IMU parse message ' + str(e))
            print(data)
        except ValueError as e:
            rospy.logwarn('Unable to IMU parse message ' + str(e))
            print(data)
        else:
            # No exception
            self.imu_pub.publish(msg)
            self.magnetic_field_pub.publish(mag_msg)
    def parse_gps(self, data):
        # {
        #   u'status': 
        #   {
        #       u'pDOP': 99.99, 
        #       u'positionLatLonAlt': [u'0.000000000', u'0.000000000', u'0.000000000'], 
        #       u'satellites_dB': u'15,', 
        #       u'gga': u'', 
        #       u'vAcc': 3750111.232, 
        #       u'position': [u'6378137.0000', u'0.0000', u'0.0000'], 
        #       u'estAccuracy': 4294967.295, u'heading': 0, 
        #       u'headingAccuracy': 180, 
        #       u'hAcc': 4294967.295
        #   }, 
        #   u'ts': 77168472, 
        #   u'general': 
        #   {
        #       u'gpsFix': u'no fix/invalid', 
        #       u'mode': u'rover'
        #   }
        # }
        fix = NavSatFix()
        fix.header.stamp = rospy.Time.now()
        fix.header.frame_id = "gps"
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        try:
            fix.latitude = float(data['status']['positionLatLonAlt'][0])
            fix.longitude = float(data['status']['positionLatLonAlt'][1])
            fix.altitude = float(data['status']['positionLatLonAlt'][2])
            fix.position_covariance[0] = float(data['status']['hAcc'])
            fix.position_covariance[4] = float(data['status']['hAcc'])
            fix.position_covariance[8] = float(data['status']['vAcc'])
            if data['general']['gpsFix'] == 'no fix/invalid':
                fix.status.status = NavSatStatus.STATUS_NO_FIX
            else:
                fix.status.status = NavSatStatus.STATUS_FIX
        except KeyError as e:
            rospy.logwarn('Unable to GNSS parse message ' + str(e))
            print(data)
        else:
            self.nav_sat_fix_pub.publish(fix)

if __name__ == '__main__':
    try:
        hc = HydraClient()
        hc.run()
    except rospy.ROSInterruptException:
        pass

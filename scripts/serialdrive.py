#!/usr/bin/env python3
import os
import tf
import rospy
import cv2
from cv_bridge import CvBridge

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

from sensor_msgs.msg import Image
from sensor_msgs.msg import Range


import serial
import time
from datetime import datetime
import struct

from scipy.spatial.transform import Rotation


global ser
ser = serial.Serial()
ser.baudrate = 38400
#ser.port = com_name.get()
ser.port = rospy.get_param('serial_port')
# ser.port = '/dev/ttyUSB1'
# ser.timeout = 20
def crc8(data: bytes) -> int:
    polynomial = 0x07
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1
            crc &= 0xFF  # Ensure CRC remains 8-bit
    return crc

class Serial(object):

    def __init__(self):
        rospy.loginfo("Serial test loaging")
        rospy.on_shutdown(self.shutdown)

        self.tf_publisher = tf.TransformBroadcaster()

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self.cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        self.odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=3)

        # self.left_front_wheel = rospy.Publisher("/state/pos/left_front_wheel", Float64, queue_size=3)
        # self.right_front_wheel = rospy.Publisher("/state/pos/right_front_wheel", Float64, queue_size=3)
        # self.left_rear_wheel = rospy.Publisher("/state/pos/left_rear_wheel", Float64, queue_size=3)
        # self.right_rear_wheel = rospy.Publisher("/state/pos/right_rear_wheel", Float64, queue_size=3)
        # self.range_front_subscriber = rospy.Subscriber("/range/front", Range, self.range_front_callback)

        self.odometry = Odometry()
        self.command = Twist()
        self.new_target_available = False
        self.sonar_data = [0, 0, 0, 0]
        
        # open serial
        ser.open()
        # print(ser.port, ser.is_open)
        # print("Connected to serial port: "+ ser.port)
        if ser.isOpen():
            ser.flushInput() #flush input buffer, discarding all its contents
            ser.flushOutput()#flush output buffer, aborting current output 
            rospy.loginfo("Serial opened: %s", ser.port)
        else:
            rospy.loginfo("Serial failed")

    def __del__(self):
        pass

    def shutdown(self):
        rospy.loginfo("ROS shutdown")
        self.cmd_pub.publish(Twist())


    def cmd_callback(self, msg: Twist):
        self.command = msg
        self.new_target_available = True

    def spin(self):

        rate = rospy.Rate(30)
        t0 = rospy.get_time()

        out = b''
        my_bytes = b''

        while not rospy.is_shutdown():
            t = rospy.get_time() - t0
            
            # IF ROBOT IS SENDING SMTHG -- PARSE INCOMING DATA
            if ser.in_waiting:
                data = ser.readline()
                # integer_value = data.decode('utf-8')
                # print('Logging the response: ', data, len(data), ', content:', data[:len(data)-2], 'check sum:', data[len(data)-2])
                if data != '' and len(data) == 46:
                    res = list(struct.unpack('fffffffffff', data[:len(data)-2])) # for the fynalsystem
                    my_bytes = b''
                    for i in res:
                        my_bytes+= struct.pack('<f', i)
                    if data[len(data)-2] == crc8(my_bytes):
                        # self.left_front_wheel.publish(res[0])
                        # self.right_front_wheel.publish(res[1])
                        # self.left_rear_wheel.publish(res[2])
                        # self.right_rear_wheel.publish(res[3])
                        self.odometry.pose.pose.position.x = res[9]
                        self.odometry.pose.pose.position.y = res[10]

                        rot = Rotation.from_euler('xyz', [0, 0, res[8]], degrees=False)
                        rot_quat = rot.as_quat()
                        self.odometry.pose.pose.orientation.w = rot_quat[3]
                        self.odometry.pose.pose.orientation.x = rot_quat[0]
                        self.odometry.pose.pose.orientation.y = rot_quat[1]
                        self.odometry.pose.pose.orientation.z = rot_quat[2]
                        #print(f'x: {res[9]}, y: {res[10]}, theta: {res[8]}')
                        
                else:
                    ser.flushInput()
            else:
                if ser.is_open and self.new_target_available:
                    #print('time to send data to the robot')
                    my_bytes = b''
                    result = []
                    nums = []
                    my_bytes+= struct.pack('<f', self.command.angular.z)
                    my_bytes+= struct.pack('<f', self.command.linear.x)
                    my_bytes+= struct.pack('<f', self.command.linear.y)
                    #checlsum
                    # data = b"0,1,1"
                    checksum = crc8(my_bytes)
                    # print(f"CRC-8 checksum: {checksum}")
                    # my_bytes+=checksum
                    my_bytes+= struct.pack('<B', checksum)
                    my_bytes+=b'\n'
                    #write to serial
                    res_bytes = ser.write(my_bytes)
                    # print('unpacked: ', list(struct.iter_unpack('fff', my_bytes[:len(my_bytes)-2])))
                    # print('as bytes: ', my_bytes)
                    # rospy.loginfo("target_updated")
                    #self.new_target_available = False
                else:
                    pass
            self.odometry.header.frame_id = 'odom'
            self.odometry.child_frame_id = 'base_link'
            self.odometry.header.stamp = rospy.Time.now()
            self.odom_publisher.publish(self.odometry)
            self.tf_publisher.sendTransform((self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y, self.odometry.pose.pose.position.z),
             (self.odometry.pose.pose.orientation.x, self.odometry.pose.pose.orientation.y, self.odometry.pose.pose.orientation.z, self.odometry.pose.pose.orientation.w),
              rospy.Time.now(), 'base_link', 'odom')
            rate.sleep()


def main(args=None):
    rospy.init_node("serial_test")

    exp = Serial()
    exp.spin()


if __name__ == "__main__":
    main()

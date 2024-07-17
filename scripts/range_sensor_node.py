#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range

import VL53L0X
import RPi.GPIO as GPIO
import time
import numpy as np

I2C_DEFAULT_ADRESS = 0x29
SHUTDOWN_PINS = [18]


class RangeSensorDriver:
    def __init__(self, shutdown_pins: list) -> None:
        self.__shutdown_pins = shutdown_pins
        self.__i2c_adresses = []
        for i in range(0, len(shutdown_pins)):
            self.__i2c_adresses.append(I2C_DEFAULT_ADRESS+i+1)
        self.__sensors = [None]*len(shutdown_pins)
        self.__timing = []
        assert len(self.__shutdown_pins), "Count of shutdown pins can not be 0"
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for p in self.__shutdown_pins:
            GPIO.setup(p, GPIO.OUT)
            GPIO.setup(p, GPIO.OUT)

            # Set all shutdown pins low to turn off each VL53L0X
            GPIO.output(p, GPIO.LOW)
            GPIO.output(p, GPIO.LOW)

        # Keep all low for 500 ms or so to make sure they reset
        time.sleep(0.50)

    def init(self):
        self.__sensors.clear()
        for i in range(0, len(self.__shutdown_pins)):
            tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=I2C_DEFAULT_ADRESS)
            self.__sensors.append(tof)
            # I2C Address can change before tof.open()
            GPIO.output(self.__shutdown_pins[i], GPIO.HIGH)
            time.sleep(0.50)
            tof.change_address(self.__i2c_adresses[i])
            # GPIO.output(self.__shutdown_pins[i], GPIO.LOW)
            time.sleep(0.50)
        
        for i in range(0, len(self.__shutdown_pins)):
            GPIO.output(self.__shutdown_pins[i], GPIO.HIGH)
            self.__sensors[i].open()
            time.sleep(0.50)

    def start(self, mode: int = VL53L0X.Vl53l0xAccuracyMode.LONG_RANGE):
        self.__timing.clear()
        for i in range(0, len(self.__i2c_adresses)):
            self.__sensors[i].start_ranging(mode)
            self.__timing.append(self.__sensors[i].get_timing())
            if self.__timing[-1] < 20000:
                self.__timing[-1] = 20000
            print("Sensor {0}. Timing {1} ms".format(i, (self.__timing[-1]/1000)))
    
    def stop(self):
        for i in range(0, len(self.__i2c_adresses)):
            self.__sensors[i].stop_ranging()
    
    def close(self):
        for i in range(0, len(self.__i2c_adresses)):
            self.__sensors[i].close()
    
    def get_data(self) -> np.ndarray:
        data = []
        for i in range(0, len(self.__i2c_adresses)):
            data.append(self.__sensors[i].get_distance())
            time.sleep(self.__timing[i]/1000000.00)
        return np.array(data)/1000.0
    
    def size(self):
        return len(self.__sensors)

def create_msg(distance: float, frame_id: str):
    range_msg = Range()
    range_msg.radiation_type = 1
    range_msg.header.frame_id =  frame_id
    range_msg.field_of_view = 0.01
    range_msg.min_range = 0.004
    range_msg.max_range = 2.0
    range_msg.range =distance
    range_msg.header.stamp = rospy.get_rostime()
    return range_msg

def main():
    sensors = RangeSensorDriver(SHUTDOWN_PINS)
    publishers = []
    for i in range(0, sensors.size()):
        publishers.append(rospy.Publisher('/range_data', Range, queue_size=2))
    rospy.init_node('range_sensor_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    sensors.init()
    sensors.start()
    while not rospy.is_shutdown():
        distances = sensors.get_data()
        for i in range(0, sensors.size()):
            publishers[i].publish(create_msg(distances[i], '/range_sensor_'+str(i)))
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

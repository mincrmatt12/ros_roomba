#!/usr/bin/env python
import serial
import rospy
import struct
from roomba_msgs.msg import BoolArray

rospy.init_node("passive_bot")
device = serial.Serial(port=rospy.get_param("device", default="/dev/ttyAMA0"), baud=115200)

device.write(chr(128))
bumps = [False, False]

def update_bumps():
    global bumps
    device.write("\x8e\x07")
    data = ord(device.read())

    bumps[0] = data & 2 == 2
    bumps[1] = data & 1 == 1

publisher = rospy.Publisher("/bumper", BoolArray, queue_size=5)
rate = rospy.Rate(50.0)

while not rospy.is_shutdown():
    rate.sleep()
    update_bumps()
    publisher.publish(bumps)

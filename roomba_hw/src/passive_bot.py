#!/usr/bin/env python
import serial
import rospy
import struct
from roomba_msgs.msg import BoolArray
import std_msgs.msg

rospy.init_node("passive_bot")
device = serial.Serial(port=rospy.get_param("device", default="/dev/ttyAMA0"), baud=115200)

device.write(chr(128))
bumps = [False, False]
cliffs = [False, False]
last_time = rospy.Time.now()
vel = 0

def update_bumps():
    global bumps
    device.write("\x8e\x07")
    data = ord(device.read())

    bumps[0] = data & 2 == 2
    bumps[1] = data & 1 == 1
    cliffs[0] = data & 4 == 4
    cliffs[1] = data & 8 == 8

def update_velocity():
    global last_time
    global vel
    device.write("\x8e\x13")
    data = struct.unpack("<h", device.read(2))[0]
    t = rospy.Time.now()
    diff = t - last_time
    last_time = t
    vel = (data / 1000.0) / diff.to_sec()

pub_bump = rospy.Publisher("/bumper", BoolArray, queue_size=5)
pub_clif = rospy.Publisher("/cliffs", BoolArray, queue_size=5)
pub_velo = rospy.Publisher("/velocity", std_msgs.msg.Float32, queue_size=5)
rate = rospy.Rate(50.0)

while not rospy.is_shutdown():
    rate.sleep()
    update_bumps()
    update_velocity()
    pub_bump.publish(bumps)
    pub_clif.publish(cliffs)
    pub_velo.publish(vel)

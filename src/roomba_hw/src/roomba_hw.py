#!/usr/bin/env python

import rospy
from roomba import Roomba
import geometry_msgs.msg
import roomba_msgs.msg

rospy.init_node("roomba_hw")
device_name = rospy.get_param("~dev", default="/dev/ttyACM0")

rba = Roomba(device_name)

def on_cmd_vel(m):
    rba.send_drive(m.linear.x, m.angular.z)

def on_leds(m):
    rba.send_leds(m.check, m.dock, m.spot, m.debris, m.check_color, m.check_intensity)

def on_vacum(m):
    rba.send_vacum(m.vacum, m.main_brush, m.brush != 0, m.brush == 1)

sub_vel   = rospy.Subscriber("/cmd_vel", geometry_msgs.msg.Twist, callback = on_cmd_vel, queue_size = 5)
sub_led   = rospy.Subscriber("/leds", roomba_msgs.msg.LED, callback        = on_leds, queue_size = 5)
sub_vacum = rospy.Subscriber("/vacum", roomba_msgs.msg.Vacum, callback     = on_vacum, queue_size = 5)

while not rospy.is_shutdown():
    rospy.sleep(0.5)

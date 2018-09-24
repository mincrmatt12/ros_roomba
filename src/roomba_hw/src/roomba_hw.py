#!/usr/bin/env python

import rospy
from roomba import Roomba
import geometry_msgs.msg
import roomba_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg

rospy.init_node("roomba_hw")
device_name = rospy.get_param("~dev", default="/dev/ttyACM0")

rba = Roomba(device_name)

drive = [0, 0]
leds = [False, False, False, False, 0, 0]
vacum = [False, False, False, False]

def on_cmd_vel(m):
    global drive
    drive = [m.linear.x, m.angular.z]

def on_leds(m):
    global leds
    leds = [ m.check, m.dock, m.spot, m.debris, m.clean_color, m.clean_intensity ]

def on_vacum(m):
    global vacum
    vacum = [ m.vacum, m.main_brush, m.brush != 0, m.brush == 1 ]

sub_vel   = rospy.Subscriber("/cmd_vel", geometry_msgs.msg.Twist, callback = on_cmd_vel, queue_size = 5)
sub_led   = rospy.Subscriber("/leds", roomba_msgs.msg.LED, callback        = on_leds, queue_size = 5)
sub_vacum = rospy.Subscriber("/vacum", roomba_msgs.msg.Vacum, callback     = on_vacum, queue_size = 5)

rate = rospy.Rate(40)

pub_bumpers     = rospy.Publisher("/bumper_states" , roomba_msgs.msg.BoolArray    , queue_size = 5 )
pub_drops       = rospy.Publisher("/drop_states"   , roomba_msgs.msg.BoolArray    , queue_size = 5 )
pub_wall        = rospy.Publisher("/wall"          , std_msgs.msg.Bool            , queue_size = 5 )
pub_vwall       = rospy.Publisher("/virtual_wall"  , std_msgs.msg.Bool            , queue_size = 5 )
pub_cliff       = rospy.Publisher("/cliff_states"  , roomba_msgs.msg.BoolArray    , queue_size = 5 )
pub_dirt        = rospy.Publisher("/dirt"          , std_msgs.msg.Float32         , queue_size = 5 )
pub_buttons     = rospy.Publisher("/button_states" , roomba_msgs.msg.BoolArray    , queue_size = 5 )
pub_battery     = rospy.Publisher("/battery"       , sensor_msgs.msg.BatteryState , queue_size = 5 )
pub_temperature = rospy.Publisher("/temperature"   , sensor_msgs.msg.Temperature  , queue_size = 5 )
pub_odometry    = rospy.Publisher("/odom/wheel"    , nav_msgs.msg.Odometry        , queue_size = 5 )
pub_states      = rospy.Publisher("/joint_states"  , sensor_msgs.msg.JointState   , queue_size = 5 )

state_message = sensor_msgs.msg.JointState()

state_message.name     = ["wheel_left_spin", "wheel_right_spin", "main_brush_spin", "side_brush_spin"]
state_message.velocity = [0, 0, 0, 0]
state_message.effort   = [0, 0, 0, 0]

last_time = rospy.Time.now()

while not rospy.is_shutdown():
    print("A")
    rate.sleep()
    dur = rospy.Time.now() - last_time
    last_time = rospy.Time.now()

    rba.update_fake_joints(dur.to_sec())
    rba.read_sensor()
    rba.send_drive(*drive)
    rba.send_leds(*leds)
    rba.send_vacum(*vacum)

    pub_bumpers.publish(rba.bumpers)
    pub_drops.publish(rba.drops)
    pub_wall.publish(rba.wall)
    pub_vwall.publish(rba.vwall)
    pub_cliff.publish(rba.cliff)
    pub_dirt.publish(rba.dirt)
    pub_buttons.publish(rba.buttons)

    m = sensor_msgs.msg.BatteryState()
    m.capacity = 3.5
    m.charge = rba.charge_current
    m.voltage = rba.charge_voltage
    m.design_capacity = 3.5
    m.percentage = float('nan')
    m.present = True
    m.power_supply_status = {
            0: m.POWER_SUPPLY_STATUS_DISCHARGING,
            1: m.POWER_SUPPLY_STATUS_CHARGING, 
            2: m.POWER_SUPPLY_STATUS_CHARGING,
            3: m.POWER_SUPPLY_STATUS_CHARGING,
            4: m.POWER_SUPPLY_STATUS_NOT_CHARGING,
            5: m.POWER_SUPPLY_STATUS_UNKNOWN
    }[rba.charging]
    m.power_supply_technology = m.POWER_SUPPLY_TECHNOLOGY_NIMH
    m.power_supply_health = m.POWER_SUPPLY_HEALTH_UNKNOWN

    pub_battery.publish(m)
    m = sensor_msgs.msg.Temperature()
    m.header.frame_id = "base_link"  # todo: add battery_link
    m.temperature = rba.temperature
    pub_temperature.publish(m)

    pub_odometry.publish(rba.odometry.to_msg())

    state_message.position = rba.joint_positions[:]
    state_message.header.stamp = rospy.Time.now()
    state_message.header.frame_id = "base_link"
    pub_states.publish(state_message)

rba.device.write(chr(173))

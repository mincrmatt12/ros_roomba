#!/usr/bin/env python
import rospy
import rospkg
import os
from roomba_msgs.msg import BoolArray
from sound_play.libsoundplay import SoundClient
import Queue
import random
import std_msgs.msg

rospy.init_node("swear_executive")
rospack = rospkg.RosPack()

base_path = rospack.get_path("roomba_swear_executive")

evt_queue = Queue.Queue()

class SwearCategory:
    BUMP = 0
    HIGH_SPEED = 1
    LEDGE = 2

sounds = {
    SwearCategory.BUMP: (
        "sound/ow1.wav",
        "sound/ow2.wav",
        "sound/ow3.wav",
        "sound/ow4.wav",
        "sound/oops1.wav",
        "sound/ohcrapwall.wav",
    ),
    SwearCategory.HIGH_SPEED: (
        "sound/fuck.wav",
        "sound/shit.wav",
        "sound/whatthefuckwasthat.wav",
        "sound/ohforsake.wav"
        "sound/wasthatawall.wav"
    )
}

last_bumper = [False, False]
velocity = [0] * 50

def handle_bumpers(msg):
    global last_bumper

    if any(msg.state) and not any(last_bumper):
        print(velocity)
        if sum(velocity) > 0.03:
            evt_queue.put(SwearCategory.HIGH_SPEED)
        elif random.random() < 0.7:
            evt_queue.put(SwearCategory.BUMP)
        else:
            evt_queue.put(SwearCategory.HIGH_SPEED)

    last_bumper = msg.state[:]

def handle_velocity(msg):
    global velocity

    velocity_c = velocity[1:50]
    velocity = velocity_c + [msg.data]

bumper_sub = rospy.Subscriber("/bumper", BoolArray, callback=handle_bumpers, queue_size=10)
velo_sub = rospy.Subscriber("/velocity", std_msgs.msg.Float32, callback=handle_velocity, queue_size=10)
handle = SoundClient()

while not rospy.is_shutdown():
    a = evt_queue.get()
    sound_name = random.choice(sounds[a])
    path = os.path.join(base_path, sound_name)
    handle.playWave(path)

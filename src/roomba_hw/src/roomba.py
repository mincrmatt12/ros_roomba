import serial
import math
import rospy
import struct
from odometry import Odometry
import threading

encoder_counts = 508.8
wheel_radius = 0.06

class Roomba:
    def __init__(self, device_path):
        self.device = serial.Serial(device_path, 115200, timeout=0.5)
        self.write_lock = threading.Lock()

        self.device.write(chr(128) * 3)
        self.device.write(chr(131))

        self.odometry = Odometry()
        self.bumpers = [False, False]
        self.drops = [False, False]

        self.wall = False
        self.cliff = [False, False, False, False]

        self.vwall = False
        self.dirt = 0

        self.buttons = [False, False, False]

        self.charging = 0
        self.charge_current = 0
        self.charge_voltage = 0

        self._odom_left = 0
        self._odom_right = 0
        self._odom_last = [0, 0]
        self._odom_start = True

        self.joint_positions = [0, 0, 0, 0]

        self.temperature = 0
        self._packets = [7, 8, 9, 10, 11, 12, 13, 14, 15, 18, 43, 44, 21, 22, 24, 25]
        self._i = 0

        self._unmeasured = [False, False]

    def send_drive(self, forward, rotate):
        left  = (forward - rotate * 0.28 / 2.0)
        right = (forward + rotate * 0.28 / 2.0)

        left *= 1000
        right *= 1000

        left = min(500, max(-500, left))
        right = min(500, max(-500, right))
        print(left, right)

        with self.write_lock:
            self.device.write(struct.pack("!Bhh", 145, left, right))

    def send_vacum(self, vacum, brush, side, side_clockwise):
        bitfield = 0
        bitfield += int(vacum) << 1
        bitfield += int(brush) << 2
        bitfield += int(side)  << 0
        bitfield += int(side_clockwise) << 3

        self._unmeasured = [brush, side]

        with self.write_lock:
            self.device.write(chr(138) + chr(bitfield))

    def update_fake_joints(self, duration):
        self.joint_positions[2] += duration * int(self._unmeasured[0]) * 12
        self.joint_positions[3] += duration * int(self._unmeasured[1]) * 20

    def send_leds(self, check, dock, spot, debris, cc, ci):
        bitfield = int(check) << 3
        bitfield += int(dock) << 2
        bitfield += int(debris) << 1
        bitfield += int(debris)

        with self.write_lock:
            self.device.write(struct.pack("!BBBB", 139, bitfield, cc, ci))

    def read_sensor(self):
        with self.write_lock:
            self.device.write(chr(149))
            self.device.write(chr(len(self._packets)))
            for i in self._packets:
                self.device.write(chr(i))
        self._i = 0
        self._read_data()
        while self._i != 0:
            self._read_data()
        print("B")

    def _read_data(self):
        pid = self._packets[self._i]
        self._i += 1
        self._i %= len(self._packets)
        if pid == 7:
            dat = ord(self.device.read(1))
            self.bumpers[1] = (dat & 0x1) != 0
            self.bumpers[0] = (dat & 0x2) != 0
            self.drops[1]   = (dat & 0x4) != 0
            self.drops[0]   = (dat & 0x8) != 0
            return 2
        elif pid == 8:
            self.wall = self.device.read(1) == chr(1)
            return 2
        elif 9 <= pid <= 12:
            self.cliff[pid - 9] = self.device.read(1) == chr(1)
            return 2
        elif pid == 13:
            self.vwall = self.device.read(1) == chr(1)
            return 2
        elif pid == 14 or pid == 16 or pid == 17:
            self.device.read(1)
            return 2
        elif pid == 15:
            self.dirt = ord(self.device.read(1)) / 255.0
            return 2
        elif pid == 18:
            dat = ord(self.device.read(1))
            self.buttons[0] = (dat & 0x1)
            self.buttons[1] = (dat & 0x2)
            self.buttons[2] = (dat & 0x4)
            return 2
        elif pid == 21:
            self.charging = ord(self.device.read(1))
            return 2
        elif pid == 22:
            self.charge_voltage = struct.unpack("!H", self.device.read(2))[0] / 1000.0
            return 3
        elif pid == 23:
            self.device.read(2)
            return 3
        elif pid == 24:
            self.temperature = struct.unpack("!b", self.device.read(1))[0]
            return 2
        elif pid == 25:
            self.charge_current = struct.unpack("!H", self.device.read(2))[0] / 1000.0
            return 3
        elif pid == 26:
            self.device.read(2)
            return 3
        elif pid == 43:
            self._odom_left = struct.unpack("!h", self.device.read(2))[0]
            return 3
        elif pid == 44:
            self._odom_right = struct.unpack("!h", self.device.read(2))[0]

            if self._odom_start:
                self._odom_start = False
                self._odom_last = [self._odom_left, self._odom_right]
                return
            # conversion:
            
            left = self._odom_left - self._odom_last[0]
            right = self._odom_right - self._odom_last[1]

            if left < -32768:
                left += 65536
            elif left > 32767:
                left -= 65536

            if right < -32768:
                right += 65536
            elif right > 32767:
                right -= 65536
            

            distance = (left + right) / 2
            distance = (distance / encoder_counts) * 2*math.pi
            distance *= wheel_radius

            angle = (((right / encoder_counts)*2*math.pi*wheel_radius) - ((left / encoder_counts)*2*math.pi*wheel_radius)) / 0.28
            print("ang", angle)
            self.odometry.integrate(distance, angle)

            self._odom_last = [self._odom_left, self._odom_right]

            self.joint_positions[0] += (left  / encoder_counts) * 2*math.pi
            self.joint_positions[1] += (right / encoder_counts) * 2*math.pi

            return 3
        return 1

import serial
import struct

class Roomba:
    def __init__(self, device_path):
        self.device = serial.Serial(device_path, 115200)

        self.device.write(chr(128) * 3)
        self.device.write(chr(131))

    def send_drive(self, forward, rotate):
        forward = forward * 1000

        left = (rotate * 0.28)/2 + forward
        right = forward*2 - left

        left = min(500, max(-500, left))
        right = min(500, max(-500, right))

        self.device.write(struct.pack("!BHH", 137, left, right))

    def send_vacum(self, vacum, brush, side, side_clockwise):
        bitfield = 0
        bitfield += int(vacum) << 1
        bitfield += int(brush) << 2
        bitfield += int(side)  << 0
        bitfield += int(side_clockwise) << 3

        self.device.write(chr(138) + chr(bitfield))

    def send_leds(self, check, dock, spot, debris, cc, ci):
        bitfield = int(check) << 3
        bitfield += int(dock) << 2
        bitfield += int(debris) << 1
        bitfield += int(debris)

        self.device.write(struct.pack("!BBB", 139, bitfield, cc, ci))

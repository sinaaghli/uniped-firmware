#!/usr/bin/env python3

import sys
import struct
import argparse
import serial
from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, QHBoxLayout,
                             QVBoxLayout, QLabel, QLineEdit)
from PyQt5.QtCore import QBasicTimer
from stm32_crc import stm32_crc



class Controller(QWidget):

    def __init__(self, device, log=False):
        super().__init__()
        self._serial = serial.Serial(device)
        self._red = False
        self._orange = False
        self._green = False
        self._blue = False
        self.init_ui()
        self._timer = QBasicTimer()
        self._timer.start(10, self)
        self._log = log

    def init_ui(self):
        self.setWindowTitle('LED Controller')

        name_width = 120

        red_button = QPushButton('Red', self)
        red_button.setCheckable(True)
        red_button.clicked[bool].connect(self.set_led)

        orange_button = QPushButton('Orange', self)
        orange_button.setCheckable(True)
        orange_button.clicked[bool].connect(self.set_led)

        green_button = QPushButton('Green', self)
        green_button.setCheckable(True)
        green_button.clicked[bool].connect(self.set_led)

        blue_button = QPushButton('Blue', self)
        blue_button.setCheckable(True)
        blue_button.clicked[bool].connect(self.set_led)

        led_box = QHBoxLayout()
        led_box.addWidget(red_button)
        led_box.addWidget(orange_button)
        led_box.addWidget(green_button)
        led_box.addWidget(blue_button)

        # seconds
        seconds_title = QLabel('Seconds:', self)
        seconds_title.setMinimumWidth(name_width)
        self.seconds_value = QLineEdit(self)
        self.seconds_value.setReadOnly(True)
        self.seconds_value.setText('0')
        seconds_box = QHBoxLayout()
        seconds_box.addWidget(seconds_title)
        seconds_box.addWidget(self.seconds_value)

        # hip angle
        hip_angle_title = QLabel('Hip Angle (deg):', self)
        hip_angle_title.setMinimumWidth(name_width)
        self.hip_angle_value = QLineEdit(self)
        self.hip_angle_value.setReadOnly(True)
        self.hip_angle_value.setText('0')
        hip_angle_box = QHBoxLayout()
        hip_angle_box.addWidget(hip_angle_title)
        hip_angle_box.addWidget(self.hip_angle_value)

        # hip current
        hip_current_title = QLabel('Hip Current (mA):', self)
        hip_current_title.setMinimumWidth(name_width)
        self.hip_current_value = QLineEdit(self)
        self.hip_current_value.setReadOnly(True)
        self.hip_current_value.setText('0')
        hip_current_box = QHBoxLayout()
        hip_current_box.addWidget(hip_current_title)
        hip_current_box.addWidget(self.hip_current_value)

        # knee angle
        knee_angle_title = QLabel('Knee Angle (deg):', self)
        knee_angle_title.setMinimumWidth(name_width)
        self.knee_angle_value = QLineEdit(self)
        self.knee_angle_value.setReadOnly(True)
        self.knee_angle_value.setText('0')
        knee_angle_box = QHBoxLayout()
        knee_angle_box.addWidget(knee_angle_title)
        knee_angle_box.addWidget(self.knee_angle_value)

        # knee current
        knee_current_title = QLabel('Hip Current (mA):', self)
        knee_current_title.setMinimumWidth(name_width)
        self.knee_current_value = QLineEdit(self)
        self.knee_current_value.setReadOnly(True)
        self.knee_current_value.setText('0')
        knee_current_box = QHBoxLayout()
        knee_current_box.addWidget(knee_current_title)
        knee_current_box.addWidget(self.knee_current_value)

        # distance
        distance_title = QLabel('Distance (cm):', self)
        distance_title.setMinimumWidth(name_width)
        self.distance_value = QLineEdit(self)
        self.distance_value.setReadOnly(True)
        self.distance_value.setText('0')
        distance_box = QHBoxLayout()
        distance_box.addWidget(distance_title)
        distance_box.addWidget(self.distance_value)

        vbox = QVBoxLayout()
        vbox.addLayout(led_box)
        vbox.addLayout(seconds_box)
        vbox.addLayout(hip_angle_box)
        vbox.addLayout(hip_current_box)
        vbox.addLayout(knee_angle_box)
        vbox.addLayout(knee_current_box)
        vbox.addLayout(distance_box)

        self.setLayout(vbox)
        self.show()

    def set_led(self, pressed):

        sender = self.sender()

        if sender.text() == 'Red':
            self._red = pressed
        if sender.text() == 'Orange':
            self._orange = pressed
        if sender.text() == 'Green':
            self._green = pressed
        if sender.text() == 'Blue':
            self._blue = pressed

        self.send_led_command()

    def send_led_command(self):
        control_byte = 0
        control_byte |= 4 if self._red else 0
        control_byte |= 1 if self._orange else 0
        control_byte |= 2 if self._green else 0
        control_byte |= 8 if self._blue else 0
        packet = b'\x7b\x08\x01\x00' + bytes([control_byte]) + b'\x00\x00\x00'
        signed_packet = (packet + struct.pack('<L', stm32_crc(packet)))
        self._serial.write(signed_packet)
        if self._log:
            print('sending: ' + ' '.join([f'{x:02x}' for x in signed_packet]))

    def timerEvent(self, event):
        if event.timerId() != self._timer.timerId():
            return
        data = self._serial.read_all()
        if len(data) < 4 or not data.startswith(b'\x7b'):
            return
        _, packet_type, length = struct.unpack('<BBH', data[:4])
        padded_length = ((length + 3) // 4)*4
        total_length = 4 + padded_length + 4
        if len(data) < total_length:
            return
        checksum = struct.unpack('<L', data[(total_length-4):])[0]
        if checksum != stm32_crc(data[:(total_length-4)]):
            return
        if self._log:
            print('received: ' + ' '.join([f'{x:02x}' for x in data]))
        # packet is good, now handle it
        if packet_type == 10:
            self.handle_status_packet(length, data[4:(total_length - 4)])

    def handle_status_packet(self, length, data):
        msec, hip_angle, hip_current, knee_angle, knee_current, distance = (
            struct.unpack('<Lfffff', data))
        self.seconds_value.setText(f'{msec/1000:0.1f}')
        self.hip_angle_value.setText(f'{hip_angle:0.1f}')
        self.hip_current_value.setText(f'{hip_current*1000:0.1f}')
        self.knee_angle_value.setText(f'{knee_angle:0.1f}')
        self.knee_current_value.setText(f'{knee_current*1000:0.1f}')
        self.distance_value.setText(f'{distance*100:0.1f}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('device')
    app = QApplication(sys.argv)
    window = Controller(parser.parse_args().device, log=False)
    sys.exit(app.exec_())

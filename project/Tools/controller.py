#!/usr/bin/env python3

import sys
import struct
import argparse
import serial
import math
from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, QHBoxLayout,
                             QVBoxLayout, QLabel, QLineEdit, QTabWidget,
                             QGroupBox, QGridLayout, QSizePolicy)
from PyQt5.QtCore import QBasicTimer
from PyQt5.QtGui import QIntValidator, QDoubleValidator
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

    def add_info_general_group(self, layout):
        grid = QGridLayout()
        grid.addWidget(QLabel('Uptime:', self), 0, 0)
        grid.addWidget(self.uptime, 0, 1)
        grid.addWidget(QLabel('Distance:', self), 1, 0)
        grid.addWidget(self.distance, 1, 1)
        group = QGroupBox('General')
        group.setLayout(grid)
        layout.addWidget(group)

    def add_info_hip_group(self, layout):
        grid = QGridLayout()
        grid.addWidget(QLabel('Angle:', self), 0, 0)
        grid.addWidget(self.hip_angle, 0, 1)
        grid.addWidget(QLabel('RPM:', self), 1, 0)
        grid.addWidget(self.hip_rpm, 1, 1)
        grid.addWidget(QLabel('Current:', self), 2, 0)
        grid.addWidget(self.hip_current, 2, 1)
        group = QGroupBox('Hip')
        group.setLayout(grid)
        layout.addWidget(group)

    def add_info_knee_group(self, layout):
        grid = QGridLayout()
        grid.addWidget(QLabel('Angle:', self), 0, 0)
        grid.addWidget(self.knee_angle, 0, 1)
        grid.addWidget(QLabel('RPM:', self), 1, 0)
        grid.addWidget(self.knee_rpm, 1, 1)
        grid.addWidget(QLabel('Current:', self), 2, 0)
        grid.addWidget(self.knee_current, 2, 1)
        group = QGroupBox('Knee')
        group.setLayout(grid)
        layout.addWidget(group)

    def add_info_tab(self, tabs):
        vbox = QVBoxLayout()
        self.add_info_general_group(vbox)
        hbox = QHBoxLayout()
        self.add_info_hip_group(hbox)
        self.add_info_knee_group(hbox)
        vbox.addLayout(hbox)
        vbox.addStretch(0)
        tab = QWidget()
        tab.setLayout(vbox)
        tabs.addTab(tab, 'Info')

    def add_led_tab(self, tabs):
        red_button = QPushButton('Red', self)
        red_button.setCheckable(True)
        red_button.clicked[bool].connect(self.set_led)
        red_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        red_button.color = 'red'

        orange_button = QPushButton('Orange', self)
        orange_button.setCheckable(True)
        orange_button.clicked[bool].connect(self.set_led)
        orange_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        orange_button.color = 'orange'

        green_button = QPushButton('Green', self)
        green_button.setCheckable(True)
        green_button.clicked[bool].connect(self.set_led)
        green_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        green_button.color = 'green'

        blue_button = QPushButton('Blue', self)
        blue_button.setCheckable(True)
        blue_button.clicked[bool].connect(self.set_led)
        blue_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        blue_button.color = 'blue'

        vbox = QVBoxLayout()
        vbox.addWidget(red_button)
        vbox.addWidget(orange_button)
        vbox.addWidget(green_button)
        vbox.addWidget(blue_button)

        tab = QWidget()
        tab.setLayout(vbox)
        tabs.addTab(tab, 'LEDs')

    def add_position_pid_hip_group(self, layout):
        self.hip_position_proportional_gain = QLineEdit(self)
        self.hip_position_proportional_gain.setText('0.002')
        self.hip_position_proportional_gain.setValidator(QDoubleValidator())

        self.hip_position_integral_gain = QLineEdit(self)
        self.hip_position_integral_gain.setText('0.00')
        self.hip_position_integral_gain.setValidator(QDoubleValidator())

        self.hip_position_derivative_gain = QLineEdit(self)
        self.hip_position_derivative_gain.setText('0.00')
        self.hip_position_derivative_gain.setValidator(QDoubleValidator())

        grid = QGridLayout()
        grid.addWidget(QLabel('Proportional Gain:', self), 0, 0)
        grid.addWidget(self.hip_position_proportional_gain, 0, 1)
        grid.addWidget(QLabel('Integral Gain:', self), 1, 0)
        grid.addWidget(self.hip_position_integral_gain, 1, 1)
        grid.addWidget(QLabel('Derivative Gain:', self), 2, 0)
        grid.addWidget(self.hip_position_derivative_gain, 2, 1)
        group = QGroupBox('Hip (angle)')
        group.setLayout(grid)
        layout.addWidget(group)

    def add_position_pid_knee_group(self, layout):
        self.knee_position_proportional_gain = QLineEdit(self)
        self.knee_position_proportional_gain.setText('0.002')
        self.knee_position_proportional_gain.setValidator(QDoubleValidator())

        self.knee_position_integral_gain = QLineEdit(self)
        self.knee_position_integral_gain.setText('0.00')
        self.knee_position_integral_gain.setValidator(QDoubleValidator())

        self.knee_position_derivative_gain = QLineEdit(self)
        self.knee_position_derivative_gain.setText('0.00')
        self.knee_position_derivative_gain.setValidator(QDoubleValidator())

        grid = QGridLayout()
        grid.addWidget(QLabel('Proportional Gain:', self), 0, 0)
        grid.addWidget(self.knee_position_proportional_gain, 0, 1)
        grid.addWidget(QLabel('Integral Gain:', self), 1, 0)
        grid.addWidget(self.knee_position_integral_gain, 1, 1)
        grid.addWidget(QLabel('Derivative Gain:', self), 2, 0)
        grid.addWidget(self.knee_position_derivative_gain, 2, 1)
        group = QGroupBox('Knee (angle)')
        group.setLayout(grid)
        layout.addWidget(group)

    def add_velocity_pid_hip_group(self, layout):
        self.hip_velocity_proportional_gain = QLineEdit(self)
        self.hip_velocity_proportional_gain.setText('0.01')
        self.hip_velocity_proportional_gain.setValidator(QDoubleValidator())

        self.hip_velocity_integral_gain = QLineEdit(self)
        self.hip_velocity_integral_gain.setText('0.00')
        self.hip_velocity_integral_gain.setValidator(QDoubleValidator())

        self.hip_velocity_derivative_gain = QLineEdit(self)
        self.hip_velocity_derivative_gain.setText('0.00')
        self.hip_velocity_derivative_gain.setValidator(QDoubleValidator())

        grid = QGridLayout()
        grid.addWidget(QLabel('Proportional Gain:', self), 0, 0)
        grid.addWidget(self.hip_velocity_proportional_gain, 0, 1)
        grid.addWidget(QLabel('Integral Gain:', self), 1, 0)
        grid.addWidget(self.hip_velocity_integral_gain, 1, 1)
        grid.addWidget(QLabel('Derivative Gain:', self), 2, 0)
        grid.addWidget(self.hip_velocity_derivative_gain, 2, 1)
        group = QGroupBox('Hip (velocity)')
        group.setLayout(grid)
        layout.addWidget(group)

    def add_velocity_pid_knee_group(self, layout):
        self.knee_velocity_proportional_gain = QLineEdit(self)
        self.knee_velocity_proportional_gain.setText('0.01')
        self.knee_velocity_proportional_gain.setValidator(QDoubleValidator())

        self.knee_velocity_integral_gain = QLineEdit(self)
        self.knee_velocity_integral_gain.setText('0.00')
        self.knee_velocity_integral_gain.setValidator(QDoubleValidator())

        self.knee_velocity_derivative_gain = QLineEdit(self)
        self.knee_velocity_derivative_gain.setText('0.00')
        self.knee_velocity_derivative_gain.setValidator(QDoubleValidator())

        grid = QGridLayout()
        grid.addWidget(QLabel('Proportional Gain:', self), 0, 0)
        grid.addWidget(self.knee_velocity_proportional_gain, 0, 1)
        grid.addWidget(QLabel('Integral Gain:', self), 1, 0)
        grid.addWidget(self.knee_velocity_integral_gain, 1, 1)
        grid.addWidget(QLabel('Derivative Gain:', self), 2, 0)
        grid.addWidget(self.knee_velocity_derivative_gain, 2, 1)
        group = QGroupBox('Knee (velocity)')
        group.setLayout(grid)
        layout.addWidget(group)

    def add_pid_tab(self, tabs):
        vbox = QVBoxLayout()
        hbox = QHBoxLayout()
        self.add_position_pid_hip_group(hbox)
        self.add_position_pid_knee_group(hbox)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        self.add_velocity_pid_hip_group(hbox)
        self.add_velocity_pid_knee_group(hbox)
        vbox.addLayout(hbox)
        submit = QPushButton('Submit', self)
        submit.clicked[bool].connect(self.send_pid_message)
        vbox.addWidget(submit)
        vbox.addStretch(0)
        tab = QWidget()
        tab.setLayout(vbox)
        tabs.addTab(tab, 'PID')

    def send_pid_message(self, pressed):
        hip_position_proportional = float(self.hip_position_proportional_gain.text())
        hip_position_integral = float(self.hip_position_integral_gain.text())
        hip_position_derivative = float(self.hip_position_derivative_gain.text())
        packet = struct.pack('<BBfff', 0, 1,
                             hip_position_proportional,
                             hip_position_integral,
                             hip_position_derivative)
        packet = struct.pack('<BBH', 123, 4, len(packet)) + packet + b'\x00\x00'
        signed_packet = (packet + struct.pack('<L', stm32_crc(packet)))
        self._serial.write(signed_packet)

        knee_position_proportional = float(self.knee_position_proportional_gain.text())
        knee_position_integral = float(self.knee_position_integral_gain.text())
        knee_position_derivative = float(self.knee_position_derivative_gain.text())
        packet = struct.pack('<BBfff', 1, 1,
                             knee_position_proportional,
                             knee_position_integral,
                             knee_position_derivative)
        packet = struct.pack('<BBH', 123, 4, len(packet)) + packet + b'\x00\x00'
        signed_packet = (packet + struct.pack('<L', stm32_crc(packet)))
        self._serial.write(signed_packet)

        hip_velocity_proportional = float(self.hip_velocity_proportional_gain.text())
        hip_velocity_integral = float(self.hip_velocity_integral_gain.text())
        hip_velocity_derivative = float(self.hip_velocity_derivative_gain.text())
        packet = struct.pack('<BBfff', 0, 2,
                             hip_velocity_proportional,
                             hip_velocity_integral,
                             hip_velocity_derivative)
        packet = struct.pack('<BBH', 123, 4, len(packet)) + packet + b'\x00\x00'
        signed_packet = (packet + struct.pack('<L', stm32_crc(packet)))
        self._serial.write(signed_packet)

        knee_velocity_proportional = float(self.knee_velocity_proportional_gain.text())
        knee_velocity_integral = float(self.knee_velocity_integral_gain.text())
        knee_velocity_derivative = float(self.knee_velocity_derivative_gain.text())
        packet = struct.pack('<BBfff', 1, 2,
                             knee_velocity_proportional,
                             knee_velocity_integral,
                             knee_velocity_derivative)
        packet = struct.pack('<BBH', 123, 4, len(packet)) + packet + b'\x00\x00'
        signed_packet = (packet + struct.pack('<L', stm32_crc(packet)))
        self._serial.write(signed_packet)

    def add_power_group(self, layout):
        vbox = QVBoxLayout()
        hbox = QHBoxLayout()

        hbox.addWidget(QLabel('Power:'))
        self.hip_power = QLineEdit(self)
        self.hip_power.setText('100')
        self.hip_power.setValidator(QIntValidator())
        hbox.addWidget(self.hip_power)
        vbox.addLayout(hbox)

        grid = QGridLayout()

        forward = QPushButton('Forward', self)
        forward.clicked[bool].connect(self.send_power_message)
        forward.motor_id = 0
        forward.mode = 'forward'
        grid.addWidget(forward, 0, 1)

        reverse = QPushButton('Reverse', self)
        reverse.clicked[bool].connect(self.send_power_message)
        reverse.motor_id = 0
        reverse.mode = 'reverse'
        grid.addWidget(reverse, 0, 0)

        stop = QPushButton('Stop', self)
        stop.clicked[bool].connect(self.send_power_message)
        stop.motor_id = 0
        stop.mode = 'stop'
        grid.addWidget(stop, 1, 0)

        drift = QPushButton('Drift', self)
        drift.clicked[bool].connect(self.send_power_message)
        drift.motor_id = 0
        drift.mode = 'drift'
        grid.addWidget(drift, 1, 1)

        vbox.addLayout(grid)

        hip_group = QGroupBox('Hip')
        hip_group.setLayout(vbox)

        vbox = QVBoxLayout()
        hbox = QHBoxLayout()

        hbox.addWidget(QLabel('Power:'))
        self.knee_power = QLineEdit(self)
        self.knee_power.setText('100')
        self.knee_power.setValidator(QIntValidator())
        hbox.addWidget(self.knee_power)
        vbox.addLayout(hbox)

        grid = QGridLayout()

        forward = QPushButton('Forward', self)
        forward.clicked[bool].connect(self.send_power_message)
        forward.motor_id = 1
        forward.mode = 'forward'
        grid.addWidget(forward, 0, 1)

        reverse = QPushButton('Reverse', self)
        reverse.clicked[bool].connect(self.send_power_message)
        reverse.motor_id = 1
        reverse.mode = 'reverse'
        grid.addWidget(reverse, 0, 0)

        stop = QPushButton('Stop', self)
        stop.clicked[bool].connect(self.send_power_message)
        stop.motor_id = 1
        stop.mode = 'stop'
        grid.addWidget(stop, 1, 0)

        drift = QPushButton('Drift', self)
        drift.clicked[bool].connect(self.send_power_message)
        drift.motor_id = 1
        drift.mode = 'drift'
        grid.addWidget(drift, 1, 1)

        vbox.addLayout(grid)

        knee_group = QGroupBox('Knee')
        knee_group.setLayout(vbox)

        hbox = QHBoxLayout()
        hbox.addWidget(hip_group)
        hbox.addWidget(knee_group)
        group = QGroupBox('Power')
        group.setLayout(hbox)
        layout.addWidget(group)

    def send_power_message(self, pressed):
        sender = self.sender()
        motor = sender.motor_id
        mode = sender.mode
        if motor == 0:
            power = int(self.hip_power.text())
        else:  # motor == 1:
            power = int(self.knee_power.text())
        mode_map = {
            'forward': 1,
            'reverse': 2,
            'stop': 3,
            'drift': 4}
        packet = struct.pack('<BBB', motor, mode_map[mode], power)
        packet = struct.pack('<BBH', 123, 5, len(packet)) + packet + b'\x00'
        signed_packet = (packet + struct.pack('<L', stm32_crc(packet)))
        self._serial.write(signed_packet)

    def add_angle_group(self, layout):
        vbox = QVBoxLayout()
        hbox = QHBoxLayout()

        hbox.addWidget(QLabel('Angle (deg):'))
        self.hip_angle_target = QLineEdit(self)
        self.hip_angle_target.setText('0')
        self.hip_angle_target.setValidator(QDoubleValidator())
        hbox.addWidget(self.hip_angle_target)
        vbox.addLayout(hbox)

        set = QPushButton('Set Target Angle', self)
        set.clicked[bool].connect(self.send_angle_message)
        set.motor_id = 0
        vbox.addWidget(set)

        hip_group = QGroupBox('Hip')
        hip_group.setLayout(vbox)

        vbox = QVBoxLayout()
        hbox = QHBoxLayout()

        hbox.addWidget(QLabel('Angle (deg):'))
        self.knee_angle_target = QLineEdit(self)
        self.knee_angle_target.setText('0')
        self.knee_angle_target.setValidator(QDoubleValidator())
        hbox.addWidget(self.knee_angle_target)
        vbox.addLayout(hbox)

        set = QPushButton('Set Target Angle', self)
        set.clicked[bool].connect(self.send_angle_message)
        set.motor_id = 1
        vbox.addWidget(set)

        knee_group = QGroupBox('Knee')
        knee_group.setLayout(vbox)

        hbox = QHBoxLayout()
        hbox.addWidget(hip_group)
        hbox.addWidget(knee_group)
        group = QGroupBox('Angle')
        group.setLayout(hbox)
        layout.addWidget(group)

    def send_angle_message(self, pressed):
        sender = self.sender()
        motor = sender.motor_id
        if motor == 0:
            angle = float(self.hip_angle_target.text())
        else:  # motor == 1:
            angle = float(self.knee_angle_target.text())
        packet = struct.pack('<Bf', motor, angle)
        packet = struct.pack('<BBH', 123, 6, len(packet)) + packet + b'\x00\x00\x00'
        signed_packet = (packet + struct.pack('<L', stm32_crc(packet)))
        self._serial.write(signed_packet)

    def add_speed_group(self, layout):
        vbox = QVBoxLayout()
        hbox = QHBoxLayout()

        hbox.addWidget(QLabel('RPM:'))
        self.hip_speed_target = QLineEdit(self)
        self.hip_speed_target.setText('0')
        self.hip_speed_target.setValidator(QDoubleValidator())
        hbox.addWidget(self.hip_speed_target)
        vbox.addLayout(hbox)

        set = QPushButton('Set Target RPM', self)
        set.clicked[bool].connect(self.send_speed_message)
        set.motor_id = 0
        vbox.addWidget(set)

        hip_group = QGroupBox('Hip')
        hip_group.setLayout(vbox)

        vbox = QVBoxLayout()
        hbox = QHBoxLayout()

        hbox.addWidget(QLabel('RPM:'))
        self.knee_speed_target = QLineEdit(self)
        self.knee_speed_target.setText('0')
        self.knee_speed_target.setValidator(QDoubleValidator())
        hbox.addWidget(self.knee_speed_target)
        vbox.addLayout(hbox)

        set = QPushButton('Set Target RPM', self)
        set.clicked[bool].connect(self.send_speed_message)
        set.motor_id = 1
        vbox.addWidget(set)

        knee_group = QGroupBox('Knee')
        knee_group.setLayout(vbox)

        hbox = QHBoxLayout()
        hbox.addWidget(hip_group)
        hbox.addWidget(knee_group)
        group = QGroupBox('Speed')
        group.setLayout(hbox)
        layout.addWidget(group)

    def send_speed_message(self, pressed):
        sender = self.sender()
        motor = sender.motor_id
        if motor == 0:
            rpm = float(self.hip_speed_target.text())
        else:  # motor == 1:
            rpm = float(self.knee_speed_target.text())
        packet = struct.pack('<Bf', motor, rpm)
        packet = struct.pack('<BBH', 123, 7, len(packet)) + packet + b'\x00\x00\x00'
        signed_packet = (packet + struct.pack('<L', stm32_crc(packet)))
        self._serial.write(signed_packet)

    def add_motor_tab(self, tabs):
        vbox = QVBoxLayout()
        self.add_power_group(vbox)
        self.add_angle_group(vbox)
        self.add_speed_group(vbox)
        vbox.addStretch(0)
        tab = QWidget()
        tab.setLayout(vbox)
        tabs.addTab(tab, 'Motors')

    def init_ui(self):
        self.setWindowTitle('Leg Controller')

        self.uptime = QLineEdit(self)
        self.uptime.setReadOnly(True)
        self.uptime.setText('N/A')

        self.distance = QLineEdit(self)
        self.distance.setReadOnly(True)
        self.distance.setText('N/A')

        self.hip_angle = QLineEdit(self)
        self.hip_angle.setReadOnly(True)
        self.hip_angle.setText('N/A')

        self.hip_rpm = QLineEdit(self)
        self.hip_rpm.setReadOnly(True)
        self.hip_rpm.setText('N/A')

        self.hip_current = QLineEdit(self)
        self.hip_current.setReadOnly(True)
        self.hip_current.setText('N/A')

        self.knee_angle = QLineEdit(self)
        self.knee_angle.setReadOnly(True)
        self.knee_angle.setText('N/A')

        self.knee_rpm = QLineEdit(self)
        self.knee_rpm.setReadOnly(True)
        self.knee_rpm.setText('N/A')

        self.knee_current = QLineEdit(self)
        self.knee_current.setReadOnly(True)
        self.knee_current.setText('N/A')

        tabs = QTabWidget()
        self.add_info_tab(tabs)
        self.add_led_tab(tabs)
        self.add_pid_tab(tabs)
        self.add_motor_tab(tabs)

        layout = QVBoxLayout()
        layout.addWidget(tabs)
        self.setLayout(layout)
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
        control_byte |= 1 if self._red else 0
        control_byte |= 2 if self._orange else 0
        control_byte |= 4 if self._green else 0
        control_byte |= 8 if self._blue else 0
        packet = b'\x7b\x03\x01\x00' + bytes([control_byte]) + b'\x00\x00\x00'
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
        if len(data[(total_length-4):]) != 4:
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
        (msec, hip_angle, hip_rpm, hip_current, knee_angle, knee_rpm,
         knee_current, distance) = struct.unpack('<Lfffffff', data)
        seconds = msec//1000
        hours = seconds//3600
        seconds = seconds - hours*3600
        minutes = seconds//60
        seconds = round(seconds - minutes*60)
        self.uptime.setText(f'{hours:02d}:{minutes:02d}:{seconds:02d}')
        self.distance.setText(f'{distance*100:0.1f} cm')
        self.hip_angle.setText(f'{hip_angle:0.1f} deg')
        self.hip_rpm.setText(f'{hip_rpm:0.1f}')
        self.hip_current.setText(f'{hip_current*1000:0.1f} mA')
        self.knee_angle.setText(f'{knee_angle:0.1f} deg')
        self.knee_rpm.setText(f'{knee_rpm:0.1f}')
        self.knee_current.setText(f'{knee_current*1000:0.1f} mA')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('device')
    app = QApplication(sys.argv)
    window = Controller(parser.parse_args().device, log=False)
    sys.exit(app.exec_())

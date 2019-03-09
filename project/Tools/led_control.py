#!/usr/bin/env python3

import sys
import struct
import argparse
import serial
from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, QHBoxLayout,
                             QVBoxLayout, QLabel, QLineEdit)
from PyQt5.QtCore import QBasicTimer
from stm32_crc import stm32_crc



class LEDController(QWidget):

    def __init__(self, device, log=False):
        super().__init__()
        self._serial = serial.Serial(device)
        self._red = False
        self._orange = False
        self._green = False
        self._blue = False
        self.initUI()
        self._timer = QBasicTimer()
        self._timer.start(10, self)
        self._log = log

    def initUI(self):
        self.setWindowTitle('LED Controller')

        red_button = QPushButton('Red', self)
        red_button.setCheckable(True)
        red_button.clicked[bool].connect(self.setLED)

        orange_button = QPushButton('Orange', self)
        orange_button.setCheckable(True)
        orange_button.clicked[bool].connect(self.setLED)

        green_button = QPushButton('Green', self)
        green_button.setCheckable(True)
        green_button.clicked[bool].connect(self.setLED)

        blue_button = QPushButton('Blue', self)
        blue_button.setCheckable(True)
        blue_button.clicked[bool].connect(self.setLED)

        led_box = QHBoxLayout()
        led_box.addWidget(red_button)
        led_box.addWidget(orange_button)
        led_box.addWidget(green_button)
        led_box.addWidget(blue_button)

        msec_title = QLabel('Milliseconds:', self)
        msec_title.setMinimumWidth(100)
        self.msec_value = QLineEdit(self)
        self.msec_value.setReadOnly(True)
        self.msec_value.setText('0')

        msec_box = QHBoxLayout()
        msec_box.addWidget(msec_title)
        msec_box.addWidget(self.msec_value)

        usec_title = QLabel('Microseconds:', self)
        usec_title.setMinimumWidth(100)
        self.usec_value = QLineEdit(self)
        self.usec_value.setReadOnly(True)
        self.usec_value.setText('0')

        usec_box = QHBoxLayout()
        usec_box.addWidget(usec_title)
        usec_box.addWidget(self.usec_value)

        vbox = QVBoxLayout()
        vbox.addLayout(led_box)
        vbox.addLayout(msec_box)
        vbox.addLayout(usec_box)

        self.setLayout(vbox)
        self.show()

    def setLED(self, pressed):

        sender = self.sender()

        if sender.text() == 'Red':
            self._red = pressed
        if sender.text() == 'Orange':
            self._orange = pressed
        if sender.text() == 'Green':
            self._green = pressed
        if sender.text() == 'Blue':
            self._blue = pressed

        self.sendLEDCommand()

    def sendLEDCommand(self):
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
        if event.timerId() == self._timer.timerId():
            data = self._serial.read_all()
            if data.startswith(b'\x7b\x0a\x08\x00'):
                if self._log:
                    print('receiving: ' + ' '.join([f'{x:02x}' for x in data]))
                magic, ptype, length, msec, usec, checksum = struct.unpack(
                    '< B B H L L L', data[:16])
                if checksum == stm32_crc(data[:12]):
                    self.msec_value.setText(f'{msec}')
                    self.usec_value.setText(f'{usec}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('device')
    app = QApplication(sys.argv)
    window = LEDController(parser.parse_args().device)
    sys.exit(app.exec_())

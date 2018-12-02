#!/usr/bin/env python
# coding=utf-8
from __future__ import division

import os
import time
import argparse
import logging

from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder
from pymodbus.constants import Endian
import hal

from borunte_hal.serial_tty_detection import get_tty_for_serial_id

MODBUS_STOPBITS = 1
MODBUS_BYTESIZE = 8
MODBUS_PARITY = 'N'
MODBUS_BAUDRATE = 115200
SERIAL_ID = 'AH06IIBJ'
MODBUS_TIMEOUT_S = 0.2

READING_REGISTER = 0x07D0
WRITING_REGISTER = 0x03E8
DEFAULT_UNIT = 9

ACTION_REQUEST_REGISTER = 0
POSITION_REQUEST_REGISTER = 3
SPEED_REGISTER = 4
FORCE_REGISTER = 5

GRIPPER_STATUS_REGISTER = 0
FAULT_STATUS_REGISTER = 2
POS_REQUEST_ECHO_REGISTER = 3
POSITION_REGISTER = 4
CURRENT_REGISTER = 5


def print_hex_reg(data):
    " ".join("{:02x}".format(ord(c)) for c in data)


class RobotiqHandComponent(object):
    def __init__(self, name, interval_s, usb_serial_id):
        self.name = name
        self.interval_s = interval_s
        self.usb_serial_id = usb_serial_id

        self._last_pos = 0
        self._client = None

    def start(self):
        self._init_comp()

    def loop(self):
        while True:
            start_time = time.time()

            if not self._client:
                connected = self._connect_client()
                self._error_pin.set(not connected)
                if connected:
                    self._init_gripper()

            if not self._error_pin.get():
                self._read_registers()
                if self._activated_pin.get():
                    if self._position_pin.get() != self._last_pos:
                        self._last_pos = self._position_pin.get()
                        self._cmd_gripper()

            self._watchdog_pin.set(not self._watchdog_pin.get())
            time.sleep(max(self.interval_s - (time.time() - start_time), 0.001))

    def stop(self):
        if self._client:
            self._client.close()

    def _init_comp(self):
        self._comp = hal.component(self.name)
        self._error_pin = self._comp.newpin('error', hal.HAL_BIT, hal.HAL_OUT)
        self._watchdog_pin = self._comp.newpin('watchdog', hal.HAL_BIT, hal.HAL_OUT)
        self._position_pin = self._comp.newpin('position', hal.HAL_U32, hal.HAL_IN)
        self._velocity_pin = self._comp.newpin('velocity', hal.HAL_U32, hal.HAL_IN)
        self._force_pin = self._comp.newpin('force', hal.HAL_U32, hal.HAL_IN)
        self._position_fb_pin = self._comp.newpin(
            'position-fb', hal.HAL_U32, hal.HAL_OUT
        )
        self._current_pin = self._comp.newpin('current', hal.HAL_U32, hal.HAL_OUT)
        self._cmd_active_pin = self._comp.newpin('cmd-active', hal.HAL_BIT, hal.HAL_IN)
        self._activated_pin = self._comp.newpin('activated', hal.HAL_BIT, hal.HAL_OUT)
        self._comp.ready()

    @staticmethod
    def _create_action_byte(r_act, r_gto=0, r_ard=0, r_atr=0):
        byte = r_ard << 5 | r_atr << 4 | r_gto << 3 | r_act << 0
        return byte

    def _write_request(
        self, action_byte, position_byte=0x00, speed_byte=0x00, force_byte=0x00
    ):
        builder = BinaryPayloadBuilder(byteorder=Endian.Little, wordorder=Endian.Little)
        builder.add_8bit_uint(action_byte)
        builder.add_8bit_uint(0x00)
        builder.add_8bit_uint(0x00)
        builder.add_8bit_uint(position_byte)
        builder.add_8bit_uint(force_byte)
        builder.add_8bit_uint(speed_byte)
        registers = builder.to_registers()
        return self._client.write_registers(
            WRITING_REGISTER, registers, unit=DEFAULT_UNIT
        )

    def _read_registers(self):
        rr = self._client.read_holding_registers(
            READING_REGISTER, count=3, unit=DEFAULT_UNIT
        )
        decoder = BinaryPayloadDecoder.fromRegisters(
            rr.registers, byteorder=Endian.Little, wordorder=Endian.Little
        )
        gripper_status = decoder.decode_8bit_uint()
        decoder.decode_8bit_uint()  # reserved
        fault_status = decoder.decode_8bit_uint()
        position_request_echo = decoder.decode_8bit_uint()
        position = decoder.decode_8bit_uint()
        current = decoder.decode_8bit_uint()
        g_act = gripper_status & 1
        g_gto = gripper_status >> 3 & 1
        g_sta = gripper_status >> 4 & 0x3
        g_obj = gripper_status >> 6 & 0x3
        g_flt = fault_status & 0xF
        k_flt = fault_status >> 4 & 0xF

        logging.debug(
            'pos echo: {} pos: {} current: {}'.format(
                position_request_echo, position, current
            )
        )
        logging.debug(
            'gACT {}, gGTO {}, gSTA {}, gOBJ {}'.format(
                hex(g_act), hex(g_gto), hex(g_sta), hex(g_obj)
            )
        )
        logging.debug('gFLT {} kFLT {}'.format(g_flt, k_flt))

        self._position_fb_pin.set(position)
        self._current_pin.set(current)
        self._activated_pin.set(g_sta == 0x3)
        self._cmd_active_pin.set(g_obj == 0x0 and g_gto == 0x1)

    def _init_gripper(self):
        rr = self._write_request(
            action_byte=self._create_action_byte(r_act=0, r_gto=0, r_atr=0, r_ard=0)
        )
        if rr.isError():
            return False
        rr = self._write_request(
            action_byte=self._create_action_byte(r_act=1, r_gto=0, r_atr=0, r_ard=0)
        )
        if rr.isError():
            return False

        self._last_pos = 0
        return True

    def _cmd_gripper(self):
        pos = min(self._position_pin.get(), 0xFF)
        speed = min(self._velocity_pin.get(), 0xFF)
        force = min(self._force_pin.get(), 0xFF)
        rr = self._write_request(
            action_byte=self._create_action_byte(r_act=1, r_gto=1),
            position_byte=pos,
            speed_byte=speed,
            force_byte=force,
        )
        success = not rr.isError()
        return success

    def _connect_client(self):
        tty = get_tty_for_serial_id(SERIAL_ID)
        if not tty:
            logging.warn('no matching tty found')
            return False
        if not os.path.exists(tty):
            logging.warn('could not open serial device')
            return False

        self._client = ModbusClient(
            method='rtu',
            timeout=MODBUS_TIMEOUT_S,
            port=tty,
            baudrate=MODBUS_BAUDRATE,
            parity=MODBUS_PARITY,
            bytesize=MODBUS_BYTESIZE,
            stopbits=MODBUS_STOPBITS,
        )
        try:
            return self._client.connect()
        except AttributeError:
            return False


def main():
    parser = argparse.ArgumentParser(
        description='HAL interface for the Robotiq 2 finger gripper'
    )
    parser.add_argument('-n', '--name', help='HAL component name', required=True)
    parser.add_argument('-i', '--interval', help='Update interval', default=0.1)
    parser.add_argument(
        '-s',
        '--serial-id',
        help='Serial ID of the USB-to-RS485 bridge device',
        required=True,
    )
    parser.add_argument('-d', '--debug', help='Enable debug info', action='store_true')
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO)

    comp = RobotiqHandComponent(
        name=args.name, interval_s=float(args.interval), usb_serial_id=args.serial_id
    )

    comp.start()
    try:
        comp.loop()
    except KeyboardInterrupt:
        pass
    finally:
        comp.stop()


if __name__ == '__main__':
    main()

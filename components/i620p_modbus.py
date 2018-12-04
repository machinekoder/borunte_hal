#!/usr/bin/env python
# coding=utf-8
from __future__ import division

import argparse
import logging
import os
import time
from collections import namedtuple
from ctypes import c_int32

from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import hal

from borunte_hal.serial_tty_detection import get_tty_for_serial_id

MODBUS_STOPBITS = 2
MODBUS_BYTESIZE = 8
MODBUS_PARITY = 'N'
MODBUS_BAUDRATE = 57600
MODBUS_TIMEOUT_S = 0.2

ServoPins = namedtuple('ServoPins', 'raw_ticks scale abs_pos')


class IS620Component(object):
    def __init__(self, name, interval_s, num_servos, usb_serial_id):
        self.name = name
        self.interval_s = interval_s
        self.num_servos = num_servos
        self.usb_serial_id = usb_serial_id
        self._client = None

        self._comp = None
        self._error_pin = None
        self._watchdog_pin = None
        self._servo_pins = []

        self._last_time = 0.0

    def start(self):
        self._init_comp()

    def loop(self):
        while True:
            start_time = time.time()
            if not self._client:
                self._error_pin.set(not self._connect_client())

            if not self._error_pin.get():
                for i in range(self.num_servos):
                    try:
                        raw_ticks = self._read_encoder_ticks(i + 1)
                    except AttributeError:
                        self._error_pin.set(True)
                        break
                    scale = self._servo_pins[i].scale.get()
                    if scale >= 0.0:
                        abs_pos = raw_ticks / max(1.0, scale)
                    else:
                        abs_pos = raw_ticks / min(-1.0, scale)
                    self._servo_pins[i].raw_ticks.set(raw_ticks)
                    self._servo_pins[i].abs_pos.set(abs_pos)

            self._watchdog_pin.set(not self._watchdog_pin.get())
            time.sleep(self.interval_s - (time.time() - start_time))

    def stop(self):
        if self._client:
            self._client.close()

    def _init_comp(self):
        self._comp = hal.component(self.name)
        self._error_pin = self._comp.newpin('error', hal.HAL_BIT, hal.HAL_OUT)
        self._watchdog_pin = self._comp.newpin('watchdog', hal.HAL_BIT, hal.HAL_OUT)
        for i in range(self.num_servos):
            servo_pins = ServoPins(
                raw_ticks=self._comp.newpin(
                    '{}.raw-ticks'.format(i + 1), hal.HAL_S32, hal.HAL_OUT
                ),
                scale=self._comp.newpin(
                    '{}.scale'.format(i + 1), hal.HAL_FLOAT, hal.HAL_IN
                ),
                abs_pos=self._comp.newpin(
                    '{}.abs-pos'.format(i + 1), hal.HAL_FLOAT, hal.HAL_OUT
                ),
            )
            servo_pins.scale.set(1.0)
            self._servo_pins.append(servo_pins)
        self._comp.ready()

    def _read_encoder_ticks(self, unit):
        rr = self._client.read_holding_registers(address=0xB07, count=2, unit=unit)
        return c_int32(rr.registers[1] << 16 | rr.registers[0]).value

    def _connect_client(self):
        tty = get_tty_for_serial_id(self.usb_serial_id)
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
    parser = argparse.ArgumentParser(description='HAL interface to the IS620P drive')
    parser.add_argument('-n', '--name', help='HAL component name', required=True)
    parser.add_argument('-c', '--count', help='Number of servos', required=True)
    parser.add_argument('-i', '--interval', help='Update interval', default=1.00)
    parser.add_argument(
        '-s',
        '--serial-id',
        help='Serial ID of the USB-to-RS485 bridge device',
        required=True,
    )
    parser.add_argument('-d', '--debug', help='Enable debug info', action='store_true')
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO)

    comp = IS620Component(
        name=args.name,
        interval_s=float(args.interval),
        num_servos=int(args.count),
        usb_serial_id=args.serial_id,
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

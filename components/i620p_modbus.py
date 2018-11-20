# coding=utf-8
from __future__ import division
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

# HB-07
# 32bit data
# keypad display is decimal

# low before high bits
ENCODER_TICKS = 8388608
TICKS_PER_REV = 10000


def read_absolute_pos():
    client = ModbusClient(
        method='rtu',
        timeout=0.2,
        port='/dev/ttyUSB0',
        baudrate=57600,
        parity='N',
        bytesize=8,
        stopbits=2,
    )
    client.connect()
    rr = client.read_holding_registers(address=0xB07, count=2, unit=6)
    encoder_ticks = rr.registers[1] << 16 | rr.registers[0]
    print(encoder_ticks)
    #servo_ticks = int(encoder_ticks / ENCODER_TICKS * TICKS_PER_REV)
    #print(servo_ticks)

    client.close()


if __name__ == '__main__':
    read_absolute_pos()

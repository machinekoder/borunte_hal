# -*- coding: utf-8 -*-
from subprocess import check_output
import shlex


def get_serial_for_tty(conn):
    cmd = shlex.split('udevadm info -a -n {}'.format(conn))
    output = check_output(cmd).strip()
    for line in output.split('\n'):
        if '{serial}' not in line:
            continue
        key, value = line.split('==')
        value = value.replace('"', '')
        if ':' not in value:
            return value
    return None


def _serial_iterator():
    output = check_output(shlex.split('ls -l /sys/bus/usb-serial/devices')).strip()
    for line in output.split('\n')[1:]:
        chunks = line.split(' ')
        tty = chunks[-3]
        serial = get_serial_for_tty(tty)
        yield tty, serial


def get_tty_for_serial_id(serial_id):
    for tty, serial in _serial_iterator():
        if serial_id == serial:
            return '/dev/{}'.format(tty)
    return None


def list_all_serials():
    for tty, serial in _serial_iterator():
        print(tty, serial)


if __name__ == '__main__':
    list_all_serials()
    tty = get_tty_for_serial_id('AH06IIBJ')
    print(tty)

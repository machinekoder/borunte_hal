# coding=utf-8
from __future__ import division

import os
from machinekit import hal
from machinekit import rtapi as rt

from utils import HalThread

SIM_MODE = bool(os.environ.get('SIM_MODE', 0))
MAIN_THREAD = HalThread(name='main_thread', period_ns=1e8)


if SIM_MODE:
    from sim_hardware import Hardware
else:
    from hardware import Hardware


def setup_threads():
    # read from IO
    rt.newthread(MAIN_THREAD.name, MAIN_THREAD.period_ns, fp=True)


def create_io_remote_component():
    control = hal.RemoteComponent('control', timer=100)
    control.newpin('watchdog_has_bit', hal.HAL_BIT, hal.HAL_IO)
    for board in ('io1', 'io2'):
        for i in range(0, 16):
            control.newpin('{}.in-{}'.format(board, i), hal.HAL_BIT, hal.HAL_IN)
        for i in range(0, 8):
            control.newpin('{}.out-{}'.format(board, i), hal.HAL_BIT, hal.HAL_OUT)
    control.ready()

    control.pin('watchdog_has_bit').link('hm2_7i80.0.watchdog.has_bit')


setup_threads()
hardware = Hardware(MAIN_THREAD)

hardware.read()
hardware.setup()
hardware.write()

create_io_remote_component()

# servo on signals
servo_on = (
    'io2-out-5',
    'io2-out-4',
    'io2-out-3',
    'io2-out-2',
    'io2-out-1',
    'io2-out-0',
)

estop = 'io1-in-14'  # true is estop active

# ready to start the threads
hal.start_threads()

hal.loadusr('haltalk')

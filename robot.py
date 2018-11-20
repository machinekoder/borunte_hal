# coding=utf-8
from __future__ import division

import os
from machinekit import hal
from machinekit import rtapi as rt

from utils import HalThread

SIM_MODE = bool(os.environ.get('SIM_MODE', 0))
MAIN_THREAD = HalThread(name='main_thread', period_ns=1e8)
NUM_JOINTS = 6


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


def create_signals():
    for i in range(1, NUM_JOINTS+1):
        hal.Signal('son-{}', hal.HAL_BIT)
        hal.Signal('brake-{}', hal.HAL_BIT)
    hal.Signal('estop', hal.HAL_BIT)


def setup_offsets():
    home_pos = hal.Signal('joint-0-home-pos', hal.HAL_FLOAT)
    abs_pos = hal.Signal('joint-0-abs-pos', hal.HAL_FLOAT)
    real_pos = hal.Signal('joint-0-real-pos', hal.HAL_FLOAT)
    fb_pos = hal.Signal('joint-0-fb-pos', hal.HAL_FLOAT)
    fb_hw_pos = hal.Signal('joint-0-fb-hw-pos', hal.HAL_FLOAT)
    cmd_pos = hal.Signal('joint-0-cmd-pos', hal.HAL_FLOAT)
    cmd_hw_pos = hal.Signal('joint-0-cmd-hw_pos', hal.HAL_FLOAT)
    pos_offset = hal.Signal('joint-0-offset', hal.HAL_FLOAT)

    offset = rt.newinst('offset', 'offset-joint-0')
    hal.addf('{}.update-output'.format(offset.name), MAIN_THREAD.name)
    hal.addf('{}.update-feedback'.format(offset.name), MAIN_THREAD.name)
    offset.pin('offset').link(pos_offset)
    offset.pin('fb-in').link(fb_hw_pos)
    offset.pin('fb-out').link(fb_pos)
    offset.pin('in').link(cmd_pos)
    offset.pin('out').link(cmd_hw_pos)

    abs_joint = rt.newinst('absolute_joint', 'abs-joint-0')
    hal.addf(abs_joint.name, MAIN_THREAD.name)
    abs_joint.pin('home-pos').link(home_pos)
    abs_joint.pin('abs-pos').link(abs_pos)
    abs_joint.pin('real-pos').link(real_pos)
    abs_joint.pin('fb-pos').link(fb_hw_pos)
    abs_joint.pin('offset').link(pos_offset)

    # feed back the out for now
    sum2 = rt.newinst('sum2', 'sum2-test')
    hal.addf(sum2.name, MAIN_THREAD.name)
    sum2.pin('in0').link(cmd_hw_pos)
    sum2.pin('out').link(fb_hw_pos)


setup_threads()
hardware = Hardware(MAIN_THREAD)

hardware.read()

create_signals()
setup_offsets()
hardware.setup()

hardware.write()

create_io_remote_component()

# ready to start the threads
hal.start_threads()

hal.loadusr('haltalk')

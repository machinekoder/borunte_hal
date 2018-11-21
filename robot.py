# coding=utf-8
from __future__ import division

import os
import sys
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
    for i in range(1, NUM_JOINTS + 1):
        hal.Signal('son-{}'.format(i), hal.HAL_BIT)
        hal.Signal('brake-release-{}'.format(i), hal.HAL_BIT)
    hal.Signal('estop', hal.HAL_BIT)


def setup_joint_offset(nr, thread):
    home_pos = hal.Signal('joint-{}-home-pos'.format(nr), hal.HAL_FLOAT)
    abs_pos = hal.Signal('joint-{}-abs-pos'.format(nr), hal.HAL_FLOAT)
    fb_out_pos = hal.Signal('joint-{}-fb-out-pos'.format(nr), hal.HAL_FLOAT)
    fb_in_pos = hal.Signal('joint-{}-fb-in-pos'.format(nr), hal.HAL_FLOAT)
    cmd_pos = hal.Signal('joint-{}-cmd-pos'.format(nr), hal.HAL_FLOAT)
    cmd_in_pos = hal.Signal('joint-{}-cmd-in-pos'.format(nr), hal.HAL_FLOAT)
    cmd_out_pos = hal.Signal('joint-{}-cmd-out-pos'.format(nr), hal.HAL_FLOAT)
    pos_offset = hal.Signal('joint-{}-pos-offset'.format(nr), hal.HAL_FLOAT)
    limit_min = hal.Signal('joint-{}-limit-min'.format(nr), hal.HAL_FLOAT)
    limit_max = hal.Signal('joint-{}-limit-max'.format(nr), hal.HAL_FLOAT)
    son = hal.Signal('son-{}'.format(nr), hal.HAL_BIT)
    son_not = hal.Signal('son-{}-not'.format(nr), hal.HAL_BIT)
    set_home = hal.Signal('joint-{}-set-home'.format(nr), hal.HAL_BIT)

    offset = rt.newinst('offsetv2', 'offset-joint-{}'.format(nr))
    offset.pin('offset').link(pos_offset)
    offset.pin('fb-in').link(fb_in_pos)
    offset.pin('fb-out').link(fb_out_pos)
    offset.pin('in').link(cmd_in_pos)
    offset.pin('out').link(cmd_out_pos)

    abs_joint = rt.newinst('absolute_joint', 'abs-joint-{}'.format(nr))
    abs_joint.pin('home-pos').link(home_pos)
    abs_joint.pin('abs-pos').link(abs_pos)
    abs_joint.pin('real-pos').link(cmd_pos)
    abs_joint.pin('fb-pos').link(fb_in_pos)
    abs_joint.pin('offset').link(pos_offset)
    abs_joint.pin('set-abs').link(son_not)
    abs_joint.pin('set-home').link(set_home)

    not_son = rt.newinst('notv2', 'not-son-{}'.format(nr))
    not_son.pin('in').link(son)
    not_son.pin('out').link(son_not)

    limit = rt.newinst('limit1v2', 'limit-joint-{}'.format(nr))
    limit.pin('min').link(limit_min)
    limit.pin('max').link(limit_max)
    limit.pin('in').link(cmd_pos)
    limit.pin('out').link(cmd_in_pos)

    hal.addf('{}.update-feedback'.format(offset.name), thread.name)
    hal.addf(not_son.name, thread.name)
    hal.addf(abs_joint.name, thread.name)
    hal.addf(limit.name, thread.name)
    hal.addf('{}.update-output'.format(offset.name), thread.name)


def setup_joint_offsets(thread):
    for nr in range(1, NUM_JOINTS + 1):
        setup_joint_offset(nr=nr, thread=thread)


def create_joint_plumbing():
    for i in range(1, NUM_JOINTS + 1):

        # see init_joints_setup function for overview
        # offset
        rt.newinst('offsetv2', 'joint{}_offset'.format(i))
        rt.newinst('jplan', 'joint{}_jplan'.format(i))


def main():
    setup_threads()
    hardware = Hardware(thread=MAIN_THREAD)

    hardware.read()

    create_signals()
    setup_joint_offsets(thread=MAIN_THREAD)
    hardware.setup()

    hardware.write()

    create_io_remote_component()

    # ready to start the threads
    hal.start_threads()

    hal.loadusr('haltalk')


if not getattr(sys, 'testing', False):
    main()

# coding=utf-8
from __future__ import division

import os
import sys
from machinekit import hal
from machinekit import rtapi as rt

from utils import HalThread, UserComp

SIM_MODE = bool(os.environ.get('SIM_MODE', 0))
MAIN_THREAD = HalThread(name='main_thread', period_ns=1e8)
NUM_JOINTS = 6


if SIM_MODE:
    from sim_hardware import Hardware
else:
    from hardware import Hardware


class BorunteConfig(object):
    def __init__(self):
        self.hardware = None
        self.user_comps = []

    def init(self):
        self.hardware = Hardware(thread=MAIN_THREAD)

    def setup(self):
        self._setup_threads()

        self.hardware.read()

        self._create_signals()
        self._setup_joints(thread=MAIN_THREAD)
        self._create_lamp_control()
        self.hardware.setup()

        self._setup_usrcomp_watchdog(
            comps=self.user_comps + self.hardware.user_comps, thread=MAIN_THREAD
        )

        self.hardware.write()

        # self._create_io_remote_component()

    @staticmethod
    def _setup_threads():
        # read from IO
        rt.newthread(MAIN_THREAD.name, MAIN_THREAD.period_ns, fp=True)

    @staticmethod
    def _create_io_remote_component():
        control = hal.RemoteComponent('control', timer=100)
        control.newpin('watchdog_has_bit', hal.HAL_BIT, hal.HAL_IO)
        for board in ('io1', 'io2'):
            for i in range(0, 16):
                control.newpin('{}.in-{}'.format(board, i), hal.HAL_BIT, hal.HAL_IN)
            for i in range(0, 8):
                control.newpin('{}.out-{}'.format(board, i), hal.HAL_BIT, hal.HAL_OUT)
        control.ready()

        control.pin('watchdog_has_bit').link('hm2_7i80.0.watchdog.has_bit')

    def _create_lamp_control(self):
        blink_interval_s = 0.5
        interval_s = 0.1
        name = 'lamp-control'
        hal.loadusr(
            './components/lamp_control.py -n {} -i {}'.format(name, interval_s),
            wait_name=name,
        )
        lamp = hal.components['lamp-control']
        lamp.pin('power-enable').link('power-enable')
        lamp.pin('estop-active').link('estop-active')
        lamp.pin('blink-interval').set(blink_interval_s)
        lamp.pin('lamp-red').link('lamp-red')
        lamp.pin('lamp-green').link('lamp-green')
        lamp.pin('lamp-yellow').link('lamp-yellow')
        lamp.pin('signal').link('lamp-signal')

        self.user_comps.append(UserComp(name=name, timeout=(interval_s * 2)))

    @staticmethod
    def _setup_usrcomp_watchdog(comps, thread):
        power_enable = hal.Signal('power-enable', hal.HAL_BIT)
        watchdog_ok = hal.Signal('watchdog-ok', hal.HAL_BIT)
        watchdog_error = hal.Signal('watchdog-error', hal.HAL_BIT)

        watchdog = rt.newinst('watchdog', 'watchdog-usrcomp', pincount=len(comps))
        hal.addf('{}.set-timeouts'.format(watchdog.name), thread.name)
        hal.addf('{}.process'.format(watchdog.name), thread.name)
        for n, comp in enumerate(comps):
            sig_in = hal.newsig('{}-watchdog'.format(comp.name), hal.HAL_BIT)
            hal.Pin('{}.watchdog'.format(comp.name)).link(sig_in)
            watchdog.pin('input-{:02}'.format(n)).link(sig_in)
            watchdog.pin('timeout-{:02}'.format(n)).set(comp.timeout)
        watchdog.pin('enable-in').link(power_enable)
        watchdog.pin('ok-out').link(watchdog_ok)

        not_comp = rt.newinst('not', 'not-watchdog-error')
        hal.addf(not_comp.name, thread.name)
        not_comp.pin('in').link(watchdog_ok)
        not_comp.pin('out').link(watchdog_error)

    @staticmethod
    def _create_signals():
        for i in range(1, NUM_JOINTS + 1):
            hal.Signal('son-{}'.format(i), hal.HAL_BIT)
            hal.Signal('brake-release-{}'.format(i), hal.HAL_BIT)
            hal.Signal('drive-alarm-{}'.format(i), hal.HAL_BIT)
        hal.Signal('estop-in', hal.HAL_BIT)
        hal.Signal('estop-active', hal.HAL_BIT)
        hal.Signal('power-enable', hal.HAL_BIT)
        hal.Signal('lamp-red', hal.HAL_BIT)
        hal.Signal('lamp-green', hal.HAL_BIT)
        hal.Signal('lamp-yellow', hal.HAL_BIT)
        hal.Signal('lamp-signal', hal.HAL_BIT)

    @staticmethod
    def _setup_joint_offset(nr, thread):
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

        # setup min/max joint limits
        limit = rt.newinst('limit1v2', 'limit-joint-{}'.format(nr))
        limit.pin('min').link(limit_min)
        limit.pin('max').link(limit_max)
        limit.pin('in').link(cmd_pos)
        limit.pin('out').link(cmd_in_pos)

        # connect functions in correct order
        hal.addf('{}.update-feedback'.format(offset.name), thread.name)
        hal.addf(not_son.name, thread.name)
        hal.addf(abs_joint.name, thread.name)
        hal.addf(limit.name, thread.name)
        hal.addf('{}.update-output'.format(offset.name), thread.name)

    @staticmethod
    def _setup_joint_ferror(nr, thread):
        cmd_fb_pos = hal.Signal('joint-{}-cmd-fb-pos'.format(nr), hal.HAL_FLOAT)
        fb_in_pos = hal.Signal('joint-{}-fb-in-pos'.format(nr), hal.HAL_FLOAT)
        ferror = hal.Signal('joint-{}-ferror'.format(nr), hal.HAL_FLOAT)
        ferror_abs = hal.Signal('joint-{}-ferror-abs'.format(nr), hal.HAL_FLOAT)
        ferror_max = hal.Signal('joint-{}-ferror-max'.format(nr), hal.HAL_FLOAT)
        ferror_active = hal.Signal('joint-{}-ferror-active'.format(nr), hal.HAL_BIT)

        sum_ferror = rt.newinst('sum2v2', 'sum2-joint-{}-ferror'.format(nr))
        sum_ferror.pin('in0').link(cmd_fb_pos)
        sum_ferror.pin('in1').link(fb_in_pos)
        sum_ferror.pin('gain1').set(-1.0)
        sum_ferror.pin('out').link(ferror)

        abs_ferror = rt.newinst('absv2', 'abs-joint-{}-ferror'.format(nr))
        abs_ferror.pin('in').link(ferror)
        abs_ferror.pin('out').link(ferror_abs)

        comp = rt.newinst('compv2', 'comp-joint-{}-ferror'.format(nr))
        comp.pin('in0').link(ferror_max)
        comp.pin('in1').link(ferror_abs)
        comp.pin('out').link(ferror_active)

        hal.addf(sum_ferror.name, thread.name)
        hal.addf(abs_ferror.name, thread.name)
        hal.addf(comp.name, thread.name)

    def _setup_joints(self, thread):
        for nr in range(1, NUM_JOINTS + 1):
            self._setup_joint_offset(nr=nr, thread=thread)
            self._setup_joint_ferror(nr=nr, thread=thread)


def main():
    config = BorunteConfig()
    config.init()
    config.setup()

    # ready to start the threads
    hal.start_threads()

    hal.loadusr('haltalk')


if not getattr(sys, 'testing', False):
    main()

# coding=utf-8
from __future__ import division

import os
from machinekit import hal
from machinekit import rtapi as rt

from .utils import HalThread, UserComp
from .constants import COMPONENT_PATH, TIMEOUT_OVERHEAD

SIM_MODE = bool(os.environ.get('SIM_MODE', 0))
NUM_JOINTS = 6


if SIM_MODE:
    from sim_hardware import Hardware
else:
    from hardware import Hardware


class BorunteConfig(object):
    def __init__(self, thread):
        self.thread = thread
        self.hardware = None
        self.user_comps = []

    def init(self):
        self.hardware = Hardware(thread=self.thread)

    def setup(self):
        self.hardware.read()

        self._create_signals()
        self._setup_joints(thread=self.thread)
        self._create_lamp_control()
        self.hardware.setup()

        self._setup_usrcomp_watchdog(
            comps=self.user_comps + self.hardware.user_comps, thread=self.thread
        )
        self._setup_estop(thread=self.thread)
        self._setup_drive_safety_signals(thread=self.thread)
        self._setup_power_enable(thread=self.thread)

        self.hardware.write()

        self._create_control_remote_component()

    @staticmethod
    def _create_control_remote_component():
        control = hal.RemoteComponent('control', timer=100)
        control.newpin('power-on', hal.HAL_BIT, hal.HAL_IO)
        control.newpin('estop-active', hal.HAL_BIT, hal.HAL_IN)
        control.ready()

        control.pin('power-on').link('power-on')
        control.pin('estop-active').link('estop-active')

    def _create_lamp_control(self):
        blink_interval_s = 0.5
        interval_s = 0.1
        name = 'lamp-control'
        hal.loadusr(
            'lamp_control.py -n {} -i {}'.format(name, interval_s), wait_name=name
        )
        lamp = hal.components[name]
        lamp.pin('power-on').link('power-on')
        lamp.pin('estop-active').link('estop-active')
        lamp.pin('blink-interval').set(blink_interval_s)
        lamp.pin('lamp-red').link('lamp-red')
        lamp.pin('lamp-green').link('lamp-green')
        lamp.pin('lamp-yellow').link('lamp-yellow')
        lamp.pin('signal').link('lamp-signal')

        self.user_comps.append(
            UserComp(name=name, timeout=(interval_s * TIMEOUT_OVERHEAD))
        )

    @staticmethod
    def _setup_usrcomp_watchdog(comps, thread):
        power_on = hal.Signal('power-on', hal.HAL_BIT)
        watchdog_ok = hal.Signal('watchdog-ok', hal.HAL_BIT)
        watchdog_error_raw = hal.Signal('watchdog-error-raw', hal.HAL_BIT)
        watchdog_error = hal.Signal('watchdog-error', hal.HAL_BIT)

        watchdog = rt.newinst('watchdog', 'watchdog.usrcomp', pincount=len(comps))
        hal.addf('{}.set-timeouts'.format(watchdog.name), thread.name)
        hal.addf('{}.process'.format(watchdog.name), thread.name)
        for n, comp in enumerate(comps):
            sig_in = hal.newsig('{}-watchdog'.format(comp.name), hal.HAL_BIT)
            hal.Pin('{}.watchdog'.format(comp.name)).link(sig_in)
            watchdog.pin('input-{:02}'.format(n)).link(sig_in)
            watchdog.pin('timeout-{:02}'.format(n)).set(comp.timeout)
        watchdog.pin('enable-in').link(power_on)
        watchdog.pin('ok-out').link(watchdog_ok)

        not_comp = rt.newinst('not', 'not.watchdog-error')
        hal.addf(not_comp.name, thread.name)
        not_comp.pin('in').link(watchdog_ok)
        not_comp.pin('out').link(watchdog_error_raw)

        and2 = rt.newinst('and2', 'and2.watchdog-error')
        hal.addf(and2.name, thread.name)
        and2.pin('in0').link(watchdog_error_raw)
        and2.pin('in1').link(power_on)
        and2.pin('out').link(watchdog_error)

    @staticmethod
    def _setup_estop_chain(error_signals, thread):
        power_on = hal.Signal('power-on', hal.HAL_BIT)
        estop_active = hal.Signal('estop-active', hal.HAL_BIT)
        estop_error = hal.Signal('estop-error', hal.HAL_BIT)
        estop_in = hal.Signal('estop-in', hal.HAL_BIT)
        ok = hal.Signal('ok', hal.HAL_BIT)

        num = len(error_signals)
        or_comp = rt.newinst('orn', 'or{}.estop-error'.format(num), pincount=num)
        hal.addf(or_comp.name, thread.name)
        for n, sig in enumerate(error_signals):
            or_comp.pin('in{}'.format(n)).link(sig)
        or_comp.pin('out').link(estop_error)

        estop_latch = rt.newinst('estop_latch', 'estop-latch')
        hal.addf(estop_latch.name, thread.name)
        estop_latch.pin('ok-in').link(estop_in)
        estop_latch.pin('fault-in').link(estop_error)
        estop_latch.pin('fault-out').link(estop_active)
        estop_latch.pin('reset').link(power_on)
        estop_latch.pin('ok-out').link(ok)

    def _setup_estop(self, thread):
        error_signals = ['watchdog-error']
        for i in range(1, NUM_JOINTS + 1):
            error_signals.append('drive-alarm-{}'.format(i))
            error_signals.append('joint-{}-ferror-active'.format(i))
        error_signals += self.hardware.error_signals
        self._setup_estop_chain(error_signals, thread)

    @staticmethod
    def _setup_drive_safety_signals(thread):
        for i in range(1, NUM_JOINTS + 1):
            and2_son = rt.newinst('and2', 'and2.son-{}'.format(i))
            hal.addf(and2_son.name, thread.name)
            and2_son.pin('in0').link('son-{}'.format(i))
            and2_son.pin('in1').link('ok')
            and2_son.pin('out').link('son-{}-out'.format(i))

            and2_brake = rt.newinst('and2', 'and2.brake-release-{}'.format(i))
            hal.addf(and2_brake.name, thread.name)
            and2_brake.pin('in0').link('brake-release-{}'.format(i))
            and2_brake.pin('in1').link('ok')
            and2_brake.pin('out').link('brake-release-{}-out'.format(i))

    @staticmethod
    def _setup_power_enable(thread):
        for i in range(1, NUM_JOINTS + 1):
            or1 = rt.newinst('orn', 'pass.son-{}'.format(i), pincount=1)
            hal.addf(or1.name, thread.name)
            or1.pin('in0').link('power-on')
            or1.pin('out').link('son-{}'.format(i))

    @staticmethod
    def _create_signals():
        for i in range(1, NUM_JOINTS + 1):
            hal.Signal('son-{}'.format(i), hal.HAL_BIT)
            hal.Signal('son-{}-out'.format(i), hal.HAL_BIT)
            hal.Signal('brake-release-{}'.format(i), hal.HAL_BIT)
            hal.Signal('brake-release-{}-out'.format(i), hal.HAL_BIT)
            hal.Signal('drive-alarm-{}'.format(i), hal.HAL_BIT)
        hal.Signal('estop-in', hal.HAL_BIT)
        hal.Signal('estop-active', hal.HAL_BIT)
        hal.Signal('power-on', hal.HAL_BIT)
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

        offset = rt.newinst('offset', 'offset.joint-{}'.format(nr))
        offset.pin('offset').link(pos_offset)
        offset.pin('fb-in').link(fb_in_pos)
        offset.pin('fb-out').link(fb_out_pos)
        offset.pin('in').link(cmd_in_pos)
        offset.pin('out').link(cmd_out_pos)

        abs_joint = rt.newinst('absolute_joint', 'abs-joint.{}'.format(nr))
        abs_joint.pin('home-pos').link(home_pos)
        abs_joint.pin('abs-pos').link(abs_pos)
        abs_joint.pin('real-pos').link(cmd_pos)
        abs_joint.pin('fb-pos').link(fb_in_pos)
        abs_joint.pin('offset').link(pos_offset)
        abs_joint.pin('set-abs').link(son_not)
        abs_joint.pin('set-home').link(set_home)

        not_son = rt.newinst('not', 'not.son-{}'.format(nr))
        not_son.pin('in').link(son)
        not_son.pin('out').link(son_not)

        # setup min/max joint limits
        limit = rt.newinst('limit1', 'limit1.joint-{}'.format(nr))
        limit.pin('min').link(limit_min)
        limit.pin('max').link(limit_max)
        limit.pin('in').link(cmd_pos)
        limit.pin('out').link(cmd_in_pos)

        # connect functions in correct order
        hal.addf(not_son.name, thread.name)
        hal.addf(abs_joint.name, thread.name)
        hal.addf(limit.name, thread.name)
        hal.addf('{}.update-feedback'.format(offset.name), thread.name)
        hal.addf('{}.update-output'.format(offset.name), thread.name)

    @staticmethod
    def _setup_joint_ferror(nr, thread):
        cmd_fb_pos = hal.Signal('joint-{}-cmd-fb-pos'.format(nr), hal.HAL_FLOAT)
        fb_in_pos = hal.Signal('joint-{}-fb-in-pos'.format(nr), hal.HAL_FLOAT)
        ferror = hal.Signal('joint-{}-ferror'.format(nr), hal.HAL_FLOAT)
        ferror_abs = hal.Signal('joint-{}-ferror-abs'.format(nr), hal.HAL_FLOAT)
        ferror_max = hal.Signal('joint-{}-ferror-max'.format(nr), hal.HAL_FLOAT)
        ferror_active = hal.Signal('joint-{}-ferror-active'.format(nr), hal.HAL_BIT)

        sum_ferror = rt.newinst('sum2', 'sum2.joint-{}-ferror'.format(nr))
        sum_ferror.pin('in0').link(cmd_fb_pos)
        sum_ferror.pin('in1').link(fb_in_pos)
        sum_ferror.pin('gain1').set(-1.0)
        sum_ferror.pin('out').link(ferror)

        abs_ferror = rt.newinst('abs', 'abs.joint-{}-ferror'.format(nr))
        abs_ferror.pin('in').link(ferror)
        abs_ferror.pin('out').link(ferror_abs)

        comp = rt.newinst('comp', 'comp.joint-{}-ferror'.format(nr))
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


def setup_thread(cgname=None):
    thread_period = 1e8 if SIM_MODE else 1e6
    thread = HalThread(name='main_thread', period_ns=thread_period)
    kwargs = {}
    if cgname and not SIM_MODE:
        kwargs['cgname'] = cgname
    rt.newthread(thread.name, thread.period_ns, fp=True, **kwargs)
    return thread


def configure_hal(thread):
    os.environ['PATH'] = '{}:{}'.format(os.environ['PATH'], COMPONENT_PATH)

    config = BorunteConfig(thread=thread)
    config.init()
    config.setup()

    # ready to start the threads
    hal.start_threads()

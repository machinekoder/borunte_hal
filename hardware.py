# coding=utf-8
from collections import namedtuple
from math import pi

import yaml
from machinekit import hal
from machinekit import rtapi as rt

from utils import HalThread, PinGroup

MESA_BOARD_IP = '192.168.1.121'
NUM_JOINTS = 6


class Hardware(object):
    def __init__(self, thread):
        """
        :type thread: HalThread
        """
        self.thread = thread

        rt.loadrt('hostmot2')
        rt.loadrt(
            'hm2_eth', board_ip=MESA_BOARD_IP, config='"num_encoders=6,num_stepgens=6"'
        )

        hal.Pin('hm2_7i80.0.watchdog.timeout_ns').set(int(self.thread.period_ns * 2))

    def read(self):
        hal.addf('hm2_7i80.0.read', self.thread.name)

    def setup(self):
        self._setup_joints()
        self._setup_io()
        self._setup_brakes()

    def write(self):
        hal.addf('hm2_7i80.0.pet_watchdog', self.thread.name)
        hal.addf('hm2_7i80.0.write', self.thread.name)

    def _setup_joints(self):
        with open('motor_config.yml', 'r') as f:
            config = yaml.safe_load(f)

        for i in range(1, NUM_JOINTS + 1):
            c = config['joint_{}'.format(i)]
            scale = c['gear_ratio'] * c['steps_per_rev'] / pi
            nr = 6 - i
            stepgen = PinGroup('hm2_7i80.0.stepgen.{:02}'.format(nr))
            stepgen['step_type'].set(0)  # 0 = Step/Dir, 1 = Up/Down, 2 = Quadrature
            stepgen['control-type'].set(0)  # position mode
            stepgen['dirhold'].set(c['dirhold_ns'])
            stepgen['dirsetup'].set(c['dirsetup_ns'])
            stepgen['steplen'].set(c['steplen_ns'])
            stepgen['stepspace'].set(c['stepspace_ns'])
            stepgen['position-scale'].set(scale)
            stepgen['maxvel'].set(c['max_vel_rad_s'] / 10)
            stepgen['maxaccel'].set(c['max_accel_rad_s2'] / 10)

            stepgen['enable'].set(True)

            # wire position-cmd, position-fb
            # setup limits

            # encoders
            encoder = PinGroup('hm2_7i80.0.encoder.{:02}'.format(nr))
            encoder['index-enable'].set(False)
            encoder['filter'].set(True)  # use 15 clocks to register change
            encoder['scale'].set(-scale)

        # set rs-485 tx enable pins
        tx0_en = PinGroup('hm2_7i80.0.gpio.071')
        tx0_en['is_output'].set(True)
        tx0_en['out'].set(False)
        tx3_en = PinGroup('hm2_7i80.0.gpio.048')
        tx3_en['is_output'].set(True)
        tx3_en['out'].set(False)

    def _setup_io(self):
        def link_io_pin(name, type_, nr, board):
            pin = PinGroup(name)
            pin['is_output'].set(type_ == hal.HAL_OUT)
            if type_ == hal.HAL_OUT:
                pin['invert_output'].set(True)
            dir_ = 'out' if type_ == hal.HAL_OUT else 'in'
            signal = hal.Signal('{}-{}-{}'.format(board, dir_, nr), hal.HAL_BIT)
            signal.link(pin[dir_])

        # i/o pins
        for nr, i in enumerate(range(16)):
            link_io_pin('hm2_7i80.0.gpio.{:03}'.format(i), hal.HAL_IN, nr, board='io2')
        for nr, i in enumerate(range(16, 24)):
            link_io_pin('hm2_7i80.0.gpio.{:03}'.format(i), hal.HAL_OUT, nr, board='io2')
        for nr, i in enumerate(range(24, 40)):
            link_io_pin('hm2_7i80.0.gpio.{:03}'.format(i), hal.HAL_IN, nr, board='io1')
        for nr, i in enumerate(range(40, 48)):
            link_io_pin('hm2_7i80.0.gpio.{:03}'.format(i), hal.HAL_OUT, nr, board='io1')

    def _setup_brakes(self):
        Brake = namedtuple('Brake', 'input output')
        brakes = (
            Brake(input='io2-in-2', output='io2-out-7'),
            Brake(input='io2-in-6', output='io2-out-6'),
            Brake(input='io2-in-11', output='io1-out-7'),
            Brake(input='io2-in-15', output='io1-out-6'),
            Brake(input='io2-in-10', output='io1-out-5'),
            Brake(input='io2-in-14', output='io1-out-4'),
        )
        for i, brake in enumerate(brakes):
            or2 = rt.newinst('not', 'not-break-{}'.format(i + 1))
            or2.pin('in').link(hal.Signal(brake.input))
            or2.pin('out').link(hal.Signal(brake.output))
            hal.addf(or2.name, self.thread.name)

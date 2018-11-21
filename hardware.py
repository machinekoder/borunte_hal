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
        self._io_pins = {}

        rt.loadrt('hostmot2')
        rt.loadrt(
            'hm2_eth', board_ip=MESA_BOARD_IP, config='"num_encoders=6,num_stepgens=6"'
        )

        hal.loadusr(
            './components/i620p_modbus.py -c {} -n i620p-abs'.format(NUM_JOINTS),
            wait_name='i620p-abs',
        )

        hal.Pin('hm2_7i80.0.watchdog.timeout_ns').set(int(self.thread.period_ns * 2))

    def read(self):
        hal.addf('hm2_7i80.0.read', self.thread.name)

    def setup(self):
        self._setup_joints()
        self._setup_io()
        self._setup_brakes()
        self._setup_servo_on()
        self._setup_estop()

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
            stepgen.pin('step_type').set(0)  # 0 = Step/Dir, 1 = Up/Down, 2 = Quadrature
            stepgen.pin('control-type').set(0)  # position mode
            stepgen.pin('dirhold').set(c['dirhold_ns'])
            stepgen.pin('dirsetup').set(c['dirsetup_ns'])
            stepgen.pin('steplen').set(c['steplen_ns'])
            stepgen.pin('stepspace').set(c['stepspace_ns'])
            stepgen.pin('position-scale').set(scale)
            stepgen.pin('maxvel').set(c['max_vel_rad_s'] / 10)
            stepgen.pin('maxaccel').set(c['max_accel_rad_s2'] / 10)

            stepgen.pin('enable').link('son-{}'.format(i))

            # wire position-cmd, position-fb
            # setup limits

            # encoders
            encoder = PinGroup('hm2_7i80.0.encoder.{:02}'.format(nr))
            encoder.pin('index-enable').set(False)
            encoder.pin('filter').set(True)  # use 15 clocks to register change
            encoder.pin('scale').set(-scale)

            encoder_abs = PinGroup('i620p-abs.{}'.format(i))
            encoder_abs.pin('scale').set(scale)

        # set rs-485 tx enable pins
        tx0_en = PinGroup('hm2_7i80.0.gpio.071')
        tx0_en.pin('is_output').set(True)
        tx0_en.pin('out').set(False)
        tx3_en = PinGroup('hm2_7i80.0.gpio.048')
        tx3_en.pin('is_output').set(True)
        tx3_en.pin('out').set(False)

    def _setup_io(self):
        io_pins = {}

        def link_io_pin(name, type_, nr, board):
            pin = PinGroup(name)
            pin.pin('is_output').set(type_ == hal.HAL_OUT)
            if type_ == hal.HAL_OUT:
                pin.pin('invert_output').set(True)
            dir_ = 'out' if type_ == hal.HAL_OUT else 'in'
            io_pins['{}-{}-{}'.format(board, dir_, nr)] = pin

        # i/o pins
        for nr, i in enumerate(range(16)):
            link_io_pin('hm2_7i80.0.gpio.{:03}'.format(i), hal.HAL_IN, nr, board='io2')
        for nr, i in enumerate(range(16, 24)):
            link_io_pin('hm2_7i80.0.gpio.{:03}'.format(i), hal.HAL_OUT, nr, board='io2')
        for nr, i in enumerate(range(24, 40)):
            link_io_pin('hm2_7i80.0.gpio.{:03}'.format(i), hal.HAL_IN, nr, board='io1')
        for nr, i in enumerate(range(40, 48)):
            link_io_pin('hm2_7i80.0.gpio.{:03}'.format(i), hal.HAL_OUT, nr, board='io1')

        self._io_pins = io_pins

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
            brake_signal = hal.Signal('brake-release-{}'.format(i + 1), hal.HAL_BIT)
            brake_signal.link(self._io_pins[brake.input].pin('in_not'))
            brake_signal.link(self._io_pins[brake.output].pin('out'))

    def _setup_servo_on(self):
        for i in range(NUM_JOINTS):
            self._io_pins['io2-out-{}'.format(5 - i)].pin('out').link(
                'son-{}'.format(i + 1)
            )

    def _setup_estop(self):
        self._io_pins['io1-in-14'].pin('in').link('estop')  # true is estop active

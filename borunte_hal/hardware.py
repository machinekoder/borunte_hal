# coding=utf-8
from collections import namedtuple
from math import pi

import yaml
from machinekit import hal
from machinekit import rtapi as rt

from .mesahandler import MesaHandler
from .utils import HalThread, PinGroup, UserComp
from .constants import JOINT_CONFIG_FILE, MESA_FIRMWARE_FILE, TIMEOUT_OVERHEAD


MESA_BOARD_IP = '192.168.1.121'
NUM_JOINTS = 6


class Hardware(object):
    def __init__(self, thread):
        """
        :type thread: HalThread
        """
        self.thread = thread
        self._io_pins = {}
        self.user_comps = []
        self.error_signals = []

        self._init_hm2()
        self._init_modbus()
        self._init_robotiq()

    def _init_hm2(self):
        mesahandler = MesaHandler(
            device='7I80', address=MESA_BOARD_IP, firmware=MESA_FIRMWARE_FILE
        )
        mesahandler.load_mesacard()

        rt.loadrt('hostmot2')
        rt.loadrt(
            'hm2_eth', board_ip=MESA_BOARD_IP, config='"num_encoders=6,num_stepgens=6"'
        )
        hal.Pin('hm2_7i80.0.watchdog.timeout_ns').set(int(self.thread.period_ns * 2))

        hw_watchdog_signal = hal.Signal('hardware-watchdog', hal.HAL_BIT)
        hal.Pin('hm2_7i80.0.watchdog.has_bit').link(hw_watchdog_signal)
        self.error_signals.append(hw_watchdog_signal)

    def _init_modbus(self):
        name = 'i620p-abs'
        interval_s = 1.0
        hal.loadusr(
            'i620p_modbus.py -c {} -n {} -i {}'.format(NUM_JOINTS, name, interval_s),
            wait_name='i620p-abs',
        )
        self.user_comps.append(
            UserComp(name=name, timeout=(interval_s * TIMEOUT_OVERHEAD))
        )

        comp = hal.components[name]
        error = hal.Signal('{}-error'.format(name), hal.HAL_BIT)
        comp.pin('error').link(error)
        self.error_signals.append(error)

    def _init_robotiq(self):
        name = 'robitiq-gripper'
        interval_s = 0.1
        hal.loadusr(
            'robotiq_modbus.py -n {} -i {}'.format(name, interval_s), wait_name=name
        )
        self.user_comps.append(
            UserComp(name=name, timeout=(interval_s * TIMEOUT_OVERHEAD))
        )

        comp = hal.components[name]
        error = hal.Signal('{}-error'.format(name), hal.HAL_BIT)
        comp.pin('error').link(error)
        self.error_signals.append(error)

    def read(self):
        hal.addf('hm2_7i80.0.read', self.thread.name)

    def setup(self):
        self._setup_joints()
        self._setup_io()
        self._setup_brakes()
        self._setup_servo_on()
        self._setup_estop()
        self._setup_drive_alarm()
        self._setup_lamp()

    def write(self):
        hal.addf('hm2_7i80.0.pet_watchdog', self.thread.name)
        hal.addf('hm2_7i80.0.write', self.thread.name)

    @staticmethod
    def _setup_joints():
        with open(JOINT_CONFIG_FILE, 'r') as f:
            config = yaml.safe_load(f)

        for i in range(1, NUM_JOINTS + 1):
            c = config['joint_{}'.format(i)]
            scale = c['gear_ratio'] * c['steps_per_rev'] / (2.0 * pi)
            nr = 6 - i

            # stepgen
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
            stepgen.pin('enable').link('son-{}-out'.format(i))
            stepgen.pin('position-cmd').link('joint-{}-cmd-out-pos'.format(i))
            stepgen.pin('position-fb').link('joint-{}-cmd-fb-pos'.format(i))

            # encoder
            encoder = PinGroup('hm2_7i80.0.encoder.{:02}'.format(nr))
            encoder.pin('index-enable').set(True)
            encoder.pin('filter').set(True)  # use 15 clocks to register change
            encoder.pin('scale').set(-scale)
            encoder.pin('position').link('joint-{}-fb-in-pos'.format(i))

            # encoder abs
            encoder_abs = PinGroup('i620p-abs.{}'.format(i))
            encoder_abs.pin('scale').set(scale)
            encoder_abs.pin('abs-pos').link('joint-{}-abs-pos'.format(i))
            hal.Signal('joint-{}-home-pos'.format(i)).set(c['home_pos'])

            # setup limits
            hal.Signal('joint-{}-limit-min'.format(i)).set(c['min_limit_rad'])
            hal.Signal('joint-{}-limit-max'.format(i)).set(c['max_limit_rad'])
            hal.Signal('joint-{}-ferror-max'.format(i)).set(c['max_ferror_rad'])

        # set rs-485 tx enable pins
        tx0_en = PinGroup('hm2_7i80.0.gpio.071')
        tx0_en.pin('is_output').set(True)
        tx0_en.pin('out').set(False)
        tx3_en = PinGroup('hm2_7i80.0.gpio.048')
        tx3_en.pin('is_output').set(True)
        tx3_en.pin('out').set(False)

    def _setup_io(self):
        io_pins = {}

        def link_io_pin(name, type_, nr_, board):
            pin = PinGroup(name)
            pin.pin('is_output').set(type_ == hal.HAL_OUT)
            if type_ == hal.HAL_OUT:
                pin.pin('invert_output').set(True)
            dir_ = 'out' if type_ == hal.HAL_OUT else 'in'
            io_pins['{}-{}-{}'.format(board, dir_, nr_)] = pin

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
            self._io_pins[brake.input].pin('in_not').link(
                'brake-release-{}'.format(i + 1)
            )
            self._io_pins[brake.output].pin('out').link(
                'brake-release-{}-out'.format(i + 1)
            )

    def _setup_servo_on(self):
        for i in range(NUM_JOINTS):
            self._io_pins['io2-out-{}'.format(5 - i)].pin('out').link(
                'son-{}-out'.format(i + 1)
            )

    def _setup_drive_alarm(self):
        alarm_ios = (
            'io2-in-0',
            'io2-in-4',
            'io2-in-9',
            'io2-in-13',
            'io2-in-8',
            'io2-in-12',
        )
        for i, io in enumerate(alarm_ios):
            self._io_pins[io].pin('in').link('drive-alarm-{}'.format(i))

    def _setup_lamp(self):
        self._io_pins['io1-out-0'].pin('out').link('lamp-red')
        self._io_pins['io1-out-1'].pin('out').link('lamp-yellow')
        self._io_pins['io1-out-2'].pin('out').link('lamp-green')
        # self._io_pins['io1-out-3'].pin('out').link('lamp-signal')

    def _setup_estop(self):
        self._io_pins['io1-in-14'].pin('in_not').link(
            'estop-in'
        )  # true is estop active

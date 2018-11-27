# coding=utf-8
from machinekit import hal
from machinekit import rtapi as rt
from math import pi

import yaml

NUM_JOINTS = 6
JOINT_CONFIG = 'joint_config.yml'


class Hardware(object):
    def __init__(self, thread):
        self.thread = thread
        self.user_comps = []
        self.error_signals = []

        self._init_stepgens()
        self._init_encoders()

    def read(self):
        for i in range(NUM_JOINTS):
            hal.addf('stepgen-{}.read'.format(i), self.thread.name)
        hal.addf('encoder.capture-position', self.thread.name)
        hal.addf('encoder.update-counters', self.thread.name)

    def setup(self):
        self._setup_joints()
        self._setup_brakes()
        self._setup_estop()

    def write(self):
        for i in range(NUM_JOINTS):
            hal.addf('stepgen-{}.write'.format(i), self.thread.name)

    def _setup_joints(self):
        with open(JOINT_CONFIG, 'r') as f:
            config = yaml.safe_load(f)

        for i in range(1, NUM_JOINTS + 1):
            c = config['joint_{}'.format(i)]
            scale = c['gear_ratio'] * c['steps_per_rev'] / (2.0 * pi)
            nr = 6 - i

            # stepgen
            stepgen = hal.instances['stepgen-{}'.format(nr)]
            stepgen.pin('position-scale').set(scale)
            stepgen.pin('maxvel').set(c['max_vel_rad_s'] / 10)
            stepgen.pin('maxaccel').set(c['max_accel_rad_s2'] / 10)
            stepgen.pin('enable').link('son-{}-out'.format(i))
            stepgen.pin('position-cmd').link('joint-{}-cmd-out-pos'.format(i))
            stepgen.pin('position-fb').link('joint-{}-cmd-fb-pos'.format(i))

            # encoder
            sum2 = rt.newinst('sum2v2', 'sum2-encoder-{}-fb'.format(nr))
            hal.addf(sum2.name, self.thread.name)
            sum2.pin('in0').link('joint-{}-cmd-fb-pos'.format(i))
            sum2.pin('in1').set(0.0)  # can be used to set an artificial offset to trigger the alarm
            sum2.pin('out').link('joint-{}-fb-in-pos'.format(i))

            # encoder abs
            hal.Signal('joint-{}-abs-pos'.format(i)).set(c['home_pos'])
            hal.Signal('joint-{}-home-pos'.format(i)).set(c['home_pos'])

            # setup limits
            hal.Signal('joint-{}-limit-min'.format(i)).set(c['min_limit_rad'])
            hal.Signal('joint-{}-limit-max'.format(i)).set(c['max_limit_rad'])
            hal.Signal('joint-{}-ferror-max'.format(i)).set(c['max_ferror_rad'])

    def _init_stepgens(self):
        for i in range(NUM_JOINTS):
            rt.newinst('dummy_stepgen', 'stepgen-{}'.format(i))

    def _init_encoders(self):
        rt.loadrt('encoder', num_chan=NUM_JOINTS)

    def _setup_brakes(self):
        # pass through SON to brake disable signal
        for i in range(1, NUM_JOINTS + 1):
            or2 = rt.newinst('or2v2', 'or2-brake-release-{}'.format(i))
            hal.addf(or2.name, self.thread.name)
            or2.pin('in0').link('son-{}-out'.format(i))
            or2.pin('out').link('brake-release-{}'.format(i))

    def _setup_estop(self):
        or2 = rt.newinst('or2v2', 'or2-estop-in')
        hal.addf(or2.name, self.thread.name)
        or2.pin('in0').set(True)
        or2.pin('out').link('estop-in')

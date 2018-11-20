# coding=utf-8
from machinekit import hal
from machinekit import rtapi as rt

from utils import PinGroup

NUM_JOINTS = 6


class Hardware(object):
    def __init__(self, thread):
        self.thread = thread

        self._init_stepgens()
        self._init_encoders()

    def read(self):
        hal.addf('stepgen.capture-position', self.thread.name)
        hal.addf('encoder.capture-position', self.thread.name)
        hal.addf('encoder.update-counters', self.thread.name)

    def setup(self):
        self._setup_brakes()
        self._setup_stepgens()

    def write(self):
        hal.addf('stepgen.make-pulses', self.thread.name)
        hal.addf('stepgen.update-freq', self.thread.name)

    def _init_stepgens(self):
        rt.loadrt('stepgen', step_type=','.join(['0'] * NUM_JOINTS))

    def _setup_stepgens(self):
        for i in range(1, NUM_JOINTS + 1):
            nr = 6 - i
            stepgen = PinGroup('stepgen.{}'.format(nr))
            stepgen.pin('enable').link('son-{}'.format(i))

    def _init_encoders(self):
        rt.loadrt('encoder', num_chan=NUM_JOINTS)

    def _setup_brakes(self):
        # pass through SON to brake disable signal
        for i in range(1, NUM_JOINTS + 1):
            or2 = rt.newinst('or2', 'or2-brake-{}'.format(i))
            hal.addf(or2.name, self.thread.name)
            or2.pin('in0').link('son-{}'.format(i))
            or2.pin('out').link('brake-{}'.format(i))

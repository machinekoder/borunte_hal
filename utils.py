# coding=utf-8
from collections import namedtuple
from machinekit import hal


class PinGroup(object):
    def __init__(self, name):
        self.name = name

    def pin(self, name):
        return hal.Pin('{}.{}'.format(self.name, name))


HalThread = namedtuple('Thread', 'name period_ns')
UserComp = namedtuple('UserComp', 'name timeout')

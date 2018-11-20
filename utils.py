# coding=utf-8
from collections import namedtuple
from machinekit import hal


class PinGroup(object):
    def __init__(self, name):
        self.name = name

    def __getitem__(self, item):
        return hal.Pin('{}.{}'.format(self.name, item))


HalThread = namedtuple('Thread', 'name period_ns')

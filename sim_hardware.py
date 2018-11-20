# coding=utf-8
from machinekit import hal
from machinekit import rtapi as rt


class Hardware(object):
    def __init__(self, thread):
        self.thread = thread

    def read(self):
        pass

    def setup(self):
        pass

    def write(self):
        pass

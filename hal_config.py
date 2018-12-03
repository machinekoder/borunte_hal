# coding=utf-8
from machinekit import hal
from borunte_hal.robot import setup_thread, configure_hal

thread = setup_thread()
configure_hal(thread)
hal.loadusr('haltalk', wait=True)

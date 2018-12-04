# -*- coding: utf-8 -*-
import sys
from uuid import uuid4

import pytest
import time

from machinekit import hal
from machinekit import rtapi as rt

setattr(sys, 'testing', True)
from borunte_hal.robot import BorunteConfig

WAIT_TIME_S = 0.1


@pytest.fixture()
def joint_offsets(hal_config):
    nr = str(uuid4())[:8]

    class Offsets(object):
        home_pos = hal.Signal('joint-{}-home-pos'.format(nr), hal.HAL_FLOAT)
        abs_pos = hal.Signal('joint-{}-abs-pos'.format(nr), hal.HAL_FLOAT)
        fb_out_pos = hal.Signal('joint-{}-fb-out-pos'.format(nr), hal.HAL_FLOAT)
        fb_in_pos = hal.Signal('joint-{}-fb-in-pos'.format(nr), hal.HAL_FLOAT)
        cmd_pos = hal.Signal('joint-{}-cmd-pos'.format(nr), hal.HAL_FLOAT)
        cmd_fb_pos = hal.Signal('joint-{}-cmd-fb-pos'.format(nr), hal.HAL_FLOAT)
        cmd_in_pos = hal.Signal('joint-{}-cmd-in-pos'.format(nr), hal.HAL_FLOAT)
        cmd_out_pos = hal.Signal('joint-{}-cmd-out-pos'.format(nr), hal.HAL_FLOAT)
        pos_offset = hal.Signal('joint-{}-pos-offset'.format(nr), hal.HAL_FLOAT)
        limit_min = hal.Signal('joint-{}-limit-min'.format(nr), hal.HAL_FLOAT)
        limit_max = hal.Signal('joint-{}-limit-max'.format(nr), hal.HAL_FLOAT)
        son = hal.Signal('son-{}'.format(nr), hal.HAL_BIT)

        ferror_max = hal.Signal('joint-{}-ferror-max'.format(nr), hal.HAL_FLOAT)
        ferror_active = hal.Signal('joint-{}-ferror-active'.format(nr), hal.HAL_BIT)

        def __init__(self):
            BorunteConfig._setup_joint_offset(nr, hal_config.thread)

            # feed back the output for now
            sum2 = rt.newinst('sum2', 'sum2-{}-test'.format(nr))
            hal.addf(sum2.name, hal_config.thread.name)
            sum2.pin('in0').link(self.cmd_out_pos)
            sum2.pin('out').link(self.fb_in_pos)

    return Offsets()


@pytest.fixture()
def joint_ferror(hal_config):
    nr = str(uuid4())[:8]

    class Ferror(object):
        cmd_fb_pos = hal.Signal('joint-{}-cmd-fb-pos'.format(nr), hal.HAL_FLOAT)
        fb_in_pos = hal.Signal('joint-{}-fb-in-pos'.format(nr), hal.HAL_FLOAT)
        ferror = hal.Signal('joint-{}-ferror'.format(nr), hal.HAL_FLOAT)
        ferror_abs = hal.Signal('joint-{}-ferror-abs'.format(nr), hal.HAL_FLOAT)
        ferror_max = hal.Signal('joint-{}-ferror-max'.format(nr), hal.HAL_FLOAT)
        ferror_active = hal.Signal('joint-{}-ferror-active'.format(nr), hal.HAL_BIT)

        def __init__(self):
            BorunteConfig._setup_joint_ferror(nr, hal_config.thread)

    return Ferror()


def test_cmd_pos_is_reset_after_son(joint_offsets):
    joint_offsets.abs_pos.set(712.70)
    joint_offsets.home_pos.set(799.70)

    time.sleep(WAIT_TIME_S)
    assert joint_offsets.pos_offset.get() == pytest.approx(799.7 - 712.7)
    assert joint_offsets.cmd_pos.get() == pytest.approx(712.7 - 799.7)
    assert joint_offsets.cmd_out_pos.get() == pytest.approx(0.0)
    assert joint_offsets.fb_out_pos.get() == pytest.approx(712.7 - 799.7)
    assert joint_offsets.fb_in_pos.get() == pytest.approx(0.0)

    joint_offsets.son.set(True)
    joint_offsets.cmd_pos.set(0.0)

    time.sleep(WAIT_TIME_S)
    assert joint_offsets.cmd_out_pos.get() == pytest.approx(799.7 - 712.7)


def test_joint_limits_are_enforced(joint_offsets):
    joint_offsets.limit_min.set(-790.81)
    joint_offsets.limit_max.set(399.94)
    joint_offsets.son.set(True)
    joint_offsets.cmd_pos.set(1000.0)

    time.sleep(WAIT_TIME_S)
    assert joint_offsets.cmd_in_pos.get() == pytest.approx(399.94)


def test_ferror_becomes_active_when_ferror_greater_than_max(joint_ferror):
    joint_ferror.ferror_max.set(1.0)
    assert not joint_ferror.ferror_active.get()

    joint_ferror.cmd_fb_pos.set(10.0)
    time.sleep(WAIT_TIME_S)
    assert joint_ferror.ferror_active.get()

    joint_ferror.cmd_fb_pos.set(0.0)
    time.sleep(WAIT_TIME_S)
    assert not joint_ferror.ferror_active.get()

    joint_ferror.cmd_fb_pos.set(-2.4)
    time.sleep(WAIT_TIME_S)
    assert joint_ferror.ferror_active.get()

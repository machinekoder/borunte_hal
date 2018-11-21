# coding=utf-8
import time
import uuid

import pytest

from machinekit import rtapi as rt
from machinekit import hal

WAIT_TIME_S = 0.1


@pytest.fixture()
def abs_joint(hal_config):
    comp = rt.newinst('absolute_joint', str(uuid.uuid4())[:8])
    hal.addf(comp.name, hal_config.thread.name)
    yield comp
    hal.delf(comp.name, hal_config.thread.name)
    rt.delinst(comp.name)


def test_set_home_updates_home_pin(abs_joint):
    abs_joint.pin('abs-pos').set(896.47)

    abs_joint.pin('set-home').set(True)
    time.sleep(WAIT_TIME_S)

    assert abs_joint.pin('home-pos').get() == pytest.approx(896.47)


def test_set_home_updates_offset_pin(abs_joint):
    abs_joint.pin('abs-pos').set(787.87)
    abs_joint.pin('fb-pos').set(33.61)

    abs_joint.pin('set-home').set(True)
    time.sleep(WAIT_TIME_S)

    assert abs_joint.pin('offset').get() == pytest.approx(33.61)


def test_set_abs_updates_offset_pin(abs_joint):
    abs_joint.pin('abs-pos').set(99.38)
    abs_joint.pin('home-pos').set(327.66)
    abs_joint.pin('fb-pos').set(980.36)

    abs_joint.pin('set-abs').set(True)
    time.sleep(WAIT_TIME_S)

    assert abs_joint.pin('home-pos').get() == pytest.approx(327.66)
    assert abs_joint.pin('real-pos').get() == pytest.approx(99.38 - 327.66)
    assert abs_joint.pin('offset').get() == pytest.approx(980.36 - (99.38 - 327.66))


def test_real_pos_is_only_updated_when_set_abs(abs_joint):
    abs_joint.pin('abs-pos').set(319.87)
    abs_joint.pin('home-pos').set(-200.13)

    time.sleep(WAIT_TIME_S)
    assert abs_joint.pin('real-pos').get() == pytest.approx(0.0)

    abs_joint.pin('set-abs').set(True)
    time.sleep(WAIT_TIME_S)

    assert abs_joint.pin('real-pos').get() == pytest.approx(319.87 + 200.13)

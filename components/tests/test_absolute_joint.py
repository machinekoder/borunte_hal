# coding=utf-8
import os
import time
import uuid

import pytest

from machinekit import rtapi as rt
from machinekit import hal

WAIT_TIME_S = 0.1
THREAD_PERIOD_NS = 1e7


@pytest.fixture(scope="module")
def hal_config():
    from machinekit import launcher

    launcher.cleanup_session()
    comp_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), '../absolute_joint.icomp'
    )
    launcher.install_comp(comp_path)
    launcher.start_realtime()
    rt.init_RTAPI()

    thread_name = 'test-thread'
    rt.newthread(thread_name, THREAD_PERIOD_NS, fp=True)
    hal.start_threads()

    yield thread_name

    hal.stop_threads()
    launcher.end_session()


@pytest.fixture()
def abs_joint(hal_config):
    comp = rt.newinst('absolute_joint', str(uuid.uuid4())[:8])
    hal.addf(comp.name, hal_config)
    yield comp
    hal.delf(comp.name, hal_config)
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

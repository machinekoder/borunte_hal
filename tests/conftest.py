# coding=utf-8
import os
from collections import namedtuple
from utils import HalThread

import pytest

from machinekit import rtapi as rt
from machinekit import hal

THREAD = HalThread(name='test-thread', period_ns=1e7)

HalConfig = namedtuple('HalConfig', 'thread')


@pytest.fixture(scope="module")
def hal_config():
    from machinekit import launcher

    launcher.cleanup_session()
    comp_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), '../components/absolute_joint.icomp'
    )
    launcher.install_comp(comp_path)
    launcher.start_realtime()
    rt.init_RTAPI()

    config = HalConfig(thread=THREAD)
    rt.newthread(config.thread.name, config.thread.period_ns, fp=True)
    hal.start_threads()

    yield config

    hal.stop_threads()
    launcher.end_session()

# coding=utf-8
import os

_BASE_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')
CONFIG_PATH = os.path.join(_BASE_PATH, 'config')
COMPONENT_PATH = os.path.join(_BASE_PATH, 'components')
JOINT_CONFIG_FILE = os.path.join(CONFIG_PATH, 'joint_config.yml')
MESA_FIRMWARE_FILE = os.path.join(CONFIG_PATH, 'FPGAFILE.bit')
TIMEOUT_OVERHEAD = 2.5

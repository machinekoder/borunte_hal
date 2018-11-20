#!/usr/bin/python
# coding=utf-8

import sys
import os
import argparse
import time
from machinekit import launcher

from mesahandler import MesaHandler

os.chdir(os.path.dirname(os.path.realpath(__file__)))


def main():
    parser = argparse.ArgumentParser(
        description='This is the anddemo demo run script '
        'it demonstrates how a run script could look like '
        'and of course starts the motorctrl demo'
    )
    parser.add_argument('-d', '--debug', help='Enable debug mode', action='store_true')
    parser.add_argument(
        '-s', '--sim', help='enable simulation mode', action='store_true'
    )

    args = parser.parse_args()

    if args.debug:
        launcher.set_debug_level(5)

    if args.sim:
        os.environ['SIM_MODE'] = '1'

    try:
        launcher.check_installation()
        launcher.cleanup_session()  # kill any running Machinekit instances
        launcher.install_comp('./components/absolute_joint.icomp')
        launcher.start_realtime()  # start Machinekit realtime environment

        launcher.ensure_mklauncher()
        launcher.start_process('configserver -n Borunte-Tester .')

        if not args.sim:
            mesahandler = MesaHandler(
                device='7I80', address='192.168.1.121', firmware='FPGAFILE.BIT'
            )
            mesahandler.load_mesacard()
        launcher.load_hal_file('robot.py')  # load the main HAL file
        # enable on ctrl-C, needs to executed after HAL files
        launcher.register_exit_handler()

        while True:
            launcher.check_processes()
            time.sleep(1)

    except Exception as e:
        sys.stderr.write(
            '\n\n--------- ERROR ---------\n%s\n-------------------------\n\n' % str(e)
        )
        launcher.end_session()
        sys.exit(1)


if __name__ == '__main__':
    main()

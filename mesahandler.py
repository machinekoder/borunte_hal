# coding=utf-8
import time
import os
import sys
import subprocess
from machinekit import launcher


class MesaHandler(object):
    def __init__(self, device, address, firmware):
        self._device = device
        self._address = address
        self._firmware = firmware
        self._timeout = 3.0
        self._mesaflash_cmd = 'mesaflash --device %s --addr %s' % (
            device, address)

        self._check_firmware_exists(firmware)
        # we could check here if the ip address and network config is sane
        # if the ip address specified is not in any networks subnet
        # starting the HAL component will fail

    @staticmethod
    def _check_firmware_exists(firmware):
        if not os.path.exists(firmware):
            raise RuntimeError('Cannot find firmware file %s' % firmware)

    def load_mesacard(self):
        self.check_alive()

        update_to_date = self.verify()
        if not update_to_date:
            self.flash()

        self.reload()

    def flash(self):
        launcher.check_process('%s --write %s --fix-boot-block' %
                               (self._mesaflash_cmd, self._firmware))
        for i in range(5):
            if self.ping():
                time.sleep(1.0)  # additional grace period
                return True
        return False

    def verify(self):
        try:
            launcher.check_process('%s --verify %s' %
                                   (self._mesaflash_cmd, self._firmware))
        except subprocess.CalledProcessError:
            return False
        return True

    def check_alive(self):
        launcher.check_process(self._mesaflash_cmd)

    def reload(self):
        launcher.check_process('%s --reload' % self._mesaflash_cmd)
        time.sleep(self._timeout)

    def ping(self, timeout=1):
        sys.stdout.write('pinging %s\n' % self._address)
        response = os.system(
            "ping -c 1 -W%i %s > /dev/null 2>&1" % (timeout, self._address))
        return response == 0

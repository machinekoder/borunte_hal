#!/usr/bin/env python
# coding=utf-8
import argparse
import hal
import time


class LampControlComponent(object):
    def __init__(self, name, interval_s):
        self.name = name
        self.interval_s = interval_s

        self._comp = hal.component(self.name)
        self._watchdog_pin = self._comp.newpin('watchdog', hal.HAL_BIT, hal.HAL_OUT)
        self._estop_active_pin = self._comp.newpin(
            'estop-active', hal.HAL_BIT, hal.HAL_IN
        )
        self._power_on_pin = self._comp.newpin(
            'power-on', hal.HAL_BIT, hal.HAL_IN
        )
        self._blink_interval_pin = self._comp.newpin(
            'blink-interval', hal.HAL_FLOAT, hal.HAL_IN
        )
        self._lamp_red_pin = self._comp.newpin('lamp-red', hal.HAL_BIT, hal.HAL_OUT)
        self._lamp_green_pin = self._comp.newpin('lamp-green', hal.HAL_BIT, hal.HAL_OUT)
        self._lamp_yellow_pin = self._comp.newpin(
            'lamp-yellow', hal.HAL_BIT, hal.HAL_OUT
        )
        self._signal_pin = self._comp.newpin('signal', hal.HAL_BIT, hal.HAL_OUT)
        self._comp.ready()

        self._estop_active_pin.set(False)
        self._power_on_pin.set(False)
        self._blink_interval_pin.set(0.5)

        self._on_off = True
        self._last_blink = 0.0

    def loop(self):
        while True:
            start_time = time.time()
            if self._estop_active_pin.get():
                blink_interval = self._blink_interval_pin.get()
                if start_time - self._last_blink > blink_interval:
                    self._on_off = not self._on_off
                    self._last_blink = start_time
                self._lamp_green_pin.set(False)
                self._lamp_red_pin.set(self._on_off)
                self._lamp_yellow_pin.set(False)
                self._signal_pin.set(self._on_off)

            elif self._power_on_pin.get():
                self._lamp_green_pin.set(False)
                self._lamp_red_pin.set(False)
                self._lamp_yellow_pin.set(True)
                self._signal_pin.set(False)
                self._on_off = True

            else:
                self._lamp_green_pin.set(True)
                self._lamp_red_pin.set(False)
                self._lamp_yellow_pin.set(False)
                self._signal_pin.set(False)
                self._on_off = True

            self._watchdog_pin.set(not self._watchdog_pin.get())
            time.sleep(self.interval_s - (time.time() - start_time))


def main():
    parser = argparse.ArgumentParser(description='Signal lamp logic HAL component')
    parser.add_argument('-n', '--name', help='HAL component name', required=True)
    parser.add_argument('-i', '--interval', help='Update interval', default=0.10)
    args = parser.parse_args()

    comp = LampControlComponent(name=args.name, interval_s=float(args.interval))

    try:
        comp.loop()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

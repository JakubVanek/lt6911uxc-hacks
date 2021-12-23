#!/usr/bin/env python3
import json
from time import sleep

import RPi.GPIO as GPIO
from smbus import SMBus

BANK_REG = 0xFF

REG_ID_0 = 0x8100
REG_ID_1 = 0x8101
REG_ID_2 = 0x8102

I2C_ENABLE = 0x80EE
DISABLE_WD = 0x8010

LANECNT = 0x86A2
INT_HDMI = 0x86A3
INT_UNK = 0x86A4
INT_AUDIO = 0x86A5
INT_RESPOND = 0x86A6

TMDS_HI = 0x8750
TMDS_MID = 0x8751
TMDS_LO = 0x8752

VPIX_HI = 0x867E
VPIX_LO = 0x867F
HPIX_HI = 0x8680
HPIX_LO = 0x8681

MIPI_TX_CTRL = 0x811D
TX_OFF = 0x00
TX_ON = 0xFB


def main():
    GPIO.setmode(GPIO.BCM)
    chip = LT6911UXC(busNum=10, addr=0x2b, reset=23, irq=24)
    chip.setup_pins()
    chip.reset_device()

    chip.enable_access()
    print("Chip ID:", chip.read_id())
    chip.disable_access()

    # with open("regs-attached-2.json", "w") as fp:
    #     json.dump(chip.dump_all(), fp)

    chip.watch_irqs()


class LT6911UXC:
    def __init__(self, busNum: int, addr: int, reset: int, irq: int):
        self.bus = SMBus(busNum)
        self.addr = addr
        self.reset = reset
        self.irq = irq
        self.current_bank = -1

    def setup_pins(self):
        GPIO.setup(self.reset, GPIO.OUT)
        GPIO.setup(self.irq, GPIO.IN)

    def reset_device(self):
        print("Resetting...")
        sleep(0.1)
        GPIO.output(self.reset, 0)
        sleep(0.1)
        GPIO.output(self.reset, 1)
        sleep(0.1)
        print("Done.")

    def has_irq(self) -> bool:
        return not GPIO.input(self.irq)

    def _set_bank(self, bank_no: int):
        if self.current_bank == bank_no:
            return
        else:
            self.bus.write_byte_data(self.addr, BANK_REG, bank_no)
            self.current_bank = bank_no

    def read_byte(self, reg: int) -> int:
        bank = (reg >> 8) & 0xFF
        offset = reg & 0xFF
        self._set_bank(bank)
        return self.bus.read_byte_data(self.addr, offset)

    def write_byte(self, reg: int, value: int):
        bank = (reg >> 8) & 0xFF
        offset = reg & 0xFF
        self._set_bank(bank)
        self.bus.write_byte_data(self.addr, offset, value)

    def read_id(self) -> str:
        b0 = self.read_byte(REG_ID_0)
        b1 = self.read_byte(REG_ID_1)
        b2 = self.read_byte(REG_ID_2)
        return f"{b0:#04x}-{b1:#04x}-{b2:#04x}"

    def enable_access(self):
        self.write_byte(I2C_ENABLE, 0x01)
        self.write_byte(DISABLE_WD, 0x00)

    def disable_access(self):
        self.write_byte(I2C_ENABLE, 0x00)

    def dump_all(self):
        regmap = {}

        for page in range(0, 256):
            print(f"Dumping #{page:3d}", end="", flush=True)
            self.enable_access()

            page_map = []

            for offset in range(0, 256):
                print(".", end="", flush=True)
                addr = (page << 8) | offset
                value = self.read_byte(addr)
                page_map.append(value)

            if all(x == page_map[0] for x in page_map):
                regmap[f"{page:#04x}"] = f"{page_map[0]:#04x}"
            else:
                regmap[f"{page:#04x}"] = {
                    f"{(page << 8 | idx):#06x}": f"{value:#04x}"
                    for idx, value in enumerate(page_map)
                }

            print()
            self.disable_access()
        return regmap

    def watch_irqs(self):
        counter = 0
        while True:
            got_irq = self.has_irq()
            if got_irq or counter > 40:
                counter = 0
                if got_irq:
                    print(f"GOT IRQ!")
                else:
                    print(f"Periodic report:")
                self.enable_access()
                print(f"MIPI  STATUS: {self.read_byte(LANECNT):#04x}")
                print(f"HDMI  STATUS: {self.read_byte(INT_HDMI):#04x}")
                print(f"UNK   STATUS: {self.read_byte(INT_UNK):#04x}")
                print(f"AUDIO STATUS: {self.read_byte(INT_AUDIO):#04x}")
                print(f"RESP  STATUS: {self.read_byte(INT_RESPOND):#04x}")
                self.print_tmds_state()
                self.print_active_area()
                self.write_byte(INT_RESPOND, 0)
                self.disable_access()
                print()
            else:
                counter += 1
            sleep(0.05)

    def watch_irqs_v2(self):
        while True:
            if self.has_irq():
                print("noice")
            sleep(0.1)

    def print_tmds_state(self):
        b0 = self.read_byte(TMDS_HI)
        b1 = self.read_byte(TMDS_MID)
        b2 = self.read_byte(TMDS_LO)

        link_state = b0 >> 4
        link_freq = ((b0 & 0xF) << 16) | (b1 << 8) | b2

        print(f"TMDS link freq: {link_freq}")
        print(f"TMDS link state: {link_state:#x}")

    def print_active_area(self):
        w = (self.read_byte(HPIX_HI) << 8 | self.read_byte(HPIX_LO)) * 2
        h = self.read_byte(VPIX_HI) << 8 | self.read_byte(VPIX_LO)
        print(f"Active area: {w}x{h}")

    def set_streaming(self, enabled: bool):
        self.write_byte(MIPI_TX_CTRL, TX_ON if enabled else TX_OFF)


if __name__ == '__main__':
    try:
        main()
    finally:
        GPIO.cleanup()

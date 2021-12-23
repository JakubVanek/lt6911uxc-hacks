#!/usr/bin/env python3

from time import sleep
from typing import List
import picamera
import picamera.mmalobj

import pigpio

CMD_END = 0
CMD_TWOBYTE_PREFIX = 1
CMD_ON = 2
CMD_OFF = 3
CMD_ADDR = 4
CMD_FLAGS = 5
CMD_READ = 6
CMD_WRITE = 7

U8 = 1
U16 = 2
U32 = 4

REG_CHIP_ID = 0x0000


def main():

    ## !! NOTE !!
    ## This program is *not* working. It reads the Chip ID as (0, 0), which is wrong.
    ## I gave up on this script and instead reused the existing raspiraw
    ## clone.

    pi = pigpio.pi()
    chip = TC358743(pi, busNum=10, addr=0x0f, reset=23, cableDetect=24)
    chip.setup_pins()
    chip.reset_device()
    try:
        useful_main(chip)
    finally:
        chip.close()


def useful_main(chip: 'TC358743'):
    print("Chip ID:", chip.read_chip_id())


def bytes2int(value: bytearray, little_endian: bool = True) -> int:
    le_bytes = value if little_endian else reversed(value)

    return sum(
        byteval << (8 * byteidx)
        for byteidx, byteval
        in enumerate(le_bytes)
    )


def int2bytes(value: int, bytecnt: int, little_endian: bool = True) -> bytearray:
    data = bytearray(
        (value >> (8 * byteidx)) & 0xFF
        for byteidx
        in range(0, bytecnt)
    )
    if not little_endian:
        data.reverse()
    return data


class TC358743:
    def __init__(self, pi: pigpio.pi, busNum: int, addr: int, reset: int, cableDetect: int):
        self.pi = pi
        self.hnd = pi.i2c_open(busNum, addr)
        self.reset = reset
        self.cable_detect = cableDetect

    def setup_pins(self):
        self.pi.set_mode(self.reset, pigpio.OUTPUT)
        self.pi.set_mode(self.cable_detect, pigpio.INPUT)

    def reset_device(self):
        print("Resetting...")
        sleep(0.1)
        self.pi.write(self.reset, pigpio.LOW)
        sleep(0.1)
        self.pi.write(self.reset, pigpio.HIGH)
        sleep(0.1)
        print("Done.")

    def read_many(self, reg: int, count: int) -> bytearray:
        addr_header = int2bytes(reg, U16, little_endian=False)

        self.pi.i2c_write_device(self.hnd, addr_header)
        return self.pi.i2c_read_device(self.hnd, count)[1]

    def write_many(self, reg: int, data: bytearray):
        all_data = int2bytes(reg, U16, little_endian=False) + data

        self.pi.i2c_write_device(self.hnd, all_data)

    def read_val(self, reg: int, bytecnt: int) -> int:
        if reg % bytecnt != 0:
            raise ValueError("Unaligned reads are not supported.")

        buffer = self.read_many(reg, bytecnt)
        return bytes2int(buffer, little_endian=True)

    def write_val(self, reg: int, bytecnt: int, value: int):
        if reg % bytecnt != 0:
            raise ValueError("Unaligned writes are not supported.")

        buffer = int2bytes(value, bytecnt, little_endian=True)
        self.write_many(reg, buffer)

    def read_chip_id(self):
        val = self.read_val(REG_CHIP_ID, U16)
        return (val >> 8) & 0xFF, val & 0xFF

    def close(self):
        self.pi.i2c_close(self.hnd)

if __name__ == '__main__':
    main()

#!/usr/bin/env python

import argparse
import serial
import sys
from time import sleep
from enum import Enum, auto


class Iface:
    class RxState(Enum):
        RX_START = auto()
        WAIT_FOR_A5 = auto()
        WAIT_FOR_LEN = auto()
        WAIT_FOR_DATA = auto()
        WAIT_FOR_CSUM = auto()
        RX_DONE = auto()

    def __init__(self, uart_path):
        self.uart = serial.Serial(
            port=uart_path,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
        )

        # Flush any partial command that could be in progress
        self.uart.write(b"0xff" * 256)
        sleep(0.1)
        self.uart.read_all()

    def send_frame(self, bytes):

        if len(bytes) == 0:
            raise ValueError("Frame is empty!")

        if len(bytes) > 256:
            raise ValueError(f"Frame length {len(bytes)} is greater than 256")

        frame = bytearray(b"\xa5")
        frame.append(len(bytes) - 1)
        frame += bytes
        frame.append((sum(bytes) & 0xFF) ^ 0xFF)

        self.uart.write(frame)
        self.uart.flush()

    def read_or_die(self):
        b = self.uart.read(1)

        if len(b) != 1:
            raise Exception(f"Timed out while waiting for a frame...")

        return b[0]

    def receive_frame(self, expected_cmd=None):
        extra = bytearray()

        while True:
            b = self.read_or_die()

            if b == 0xA6:
                break

            extra.append(b)
            if b == ord("\n"):
                print(f"Off-band RX data: {extra}")
                extra = bytearray()

        if extra:
            print(f"Off-band RX data: {extra}")

        rxlen = self.read_or_die() + 1

        expected_csum = 0

        res = bytearray()
        for _ in range(0, rxlen):
            b = self.read_or_die()
            expected_csum += b
            res.append(b)

        expected_csum = (expected_csum & 0xFF) ^ 0xFF

        csum = self.read_or_die()

        if csum != expected_csum:
            raise Exception(f"Invalid checksum, expected {expected_csum} got {csum}")

        if expected_cmd:
            if res[0 : len(expected_cmd)] != expected_cmd:
                raise Exception(f"Unexpected frame, wanted {expected_cmd} got {res}")

        return res


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PlayStation Pad/MemoryCard interface")

    parser.add_argument("command")
    parser.add_argument(
        "-u",
        "--uart",
        default="/dev/ttyUSB0",
        help="TTY connected to the interface module",
    )
    args = parser.parse_args()

    iface = Iface(args.uart)

    try:
        iface.send_frame(b"?")
        f = iface.receive_frame(b"?")
    except Exception as e:
        print(f"Couldn't communicate with interface: {e}")
        sys.exit(1)

    iface.send_frame(b"X\x01\x01\x42\x03\x03\x03")
    f = iface.receive_frame(b"X")
    print(f)

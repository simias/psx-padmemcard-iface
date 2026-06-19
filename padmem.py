#!/usr/bin/env python

import argparse
import serial
import sys
import pprint
from string import printable
from time import sleep
from enum import Enum, auto


class Slot(Enum):
    SLOT_1 = 1
    SLOT_2 = 2


class Iface:

    def __init__(self, uart_path):
        self.uart = serial.Serial(
            port=uart_path,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=5,
        )

        # Flush any partial command that could be in progress
        self.uart.write(b"0xff" * 256)
        sleep(0.1)
        self.uart.read_all()

        self.verbose = False

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

            return res[len(expected_cmd) :]

        return res

    def exchange_with_slot(self, slot, bytes):
        cmd = bytearray(b"X")
        cmd.append(slot.value)
        cmd += bytes

        if self.verbose:
            print(red(f"Slot {slot.value} TX:"))
            printbytes(bytes)

        self.send_frame(cmd)
        rx = iface.receive_frame(b"X")

        if self.verbose:
            print(green(f"Slot {slot.value} RX:"))
            printbytes(rx)

        return rx


def col(col, s):
    return f"\033[{col}m{s}\033[00m"


def red(s):
    return col(91, s)


def green(s):
    return col(92, s)


def purple(s):
    return col(95, s)


def printbytes(bytes):
    for c in range(0, len(bytes), 16):
        bb = bytes[c : c + 16]

        buf = f"{c: 4x}: "

        for i, b in enumerate(bb):
            if i != 0:
                buf += " "
            buf += f"{b:02x}"

        print(purple(buf))

        buf = f"      "
        for i, b in enumerate(bb):
            if i != 0:
                buf += " "
            c = chr(b)

            if c in printable:
                buf += c + " "
            else:
                buf += " ."

        print(purple(buf))
    print(purple(f"{len(bytes): 4x}: "))


def do_list(iface, args):
    for s in Slot:
        r = iface.exchange_with_slot(s, b"\x01\x42\x00\x00\x00")
        if len(r) == 5 and r[1] == 0x41 and r[2] == 0x5A:
            print(f"Slot {s.value}: GamePad detected")
        else:
            print(f"Slot {s.value}: No GamePad")

        r = iface.exchange_with_slot(s, b"\x81\x52\x00\x00")
        if len(r) == 4 and r[2] == 0x5A and r[3] == 0x5D:
            print(f"Slot {s.value}: Memory Card detected")
        else:
            print(f"Slot {s.value}: No Memory Card")


def do_mcdump(iface, args):
    slot = Slot(args.slot)

    read_cmd = bytearray(12 + 128)
    read_cmd[0] = 0x81
    read_cmd[1] = 0x52

    f = None

    for page in range(0, 0x400):
        print(f"\r{page + 1} / {0x400}", end="")
        page_hi = page >> 8
        page_lo = page & 0xFF

        read_cmd[4] = page_hi
        read_cmd[5] = page_lo

        r = iface.exchange_with_slot(slot, read_cmd)
        if (
            len(r) != len(read_cmd)
            or r[2] != 0x5A
            or r[3] != 0x5D
            or r[6] != 0x5C
            or r[7] != 0x5D
            or r[8] != page_hi
            or r[9] != page_lo
            or r[139] != 0x47
        ):
            print(f"\nInvalid Memory Card response at page {page}")
            printbytes(r)
            return False

        data = r[10:138]

        expected_xsum = page_hi ^ page_lo
        for b in data:
            expected_xsum ^= b

        csum = r[138]

        if csum != expected_xsum:
            print(f"\nInvalid checksum at page {page}")
            printbytes(r)
            return False

        if f is None:
            f = open(args.output, "bw")

        f.write(data)
        f.flush()


def do_exchange(iface, args):
    slot = Slot(args.slot)
    tx = bytearray(args.rest)

    v = iface.verbose
    iface.verbose = True

    iface.exchange_with_slot(slot, tx)

    iface.verbose = v


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PlayStation Pad/MemoryCard interface")

    parser.add_argument(
        "-u",
        "--uart",
        default="/dev/ttyUSB0",
        help="TTY connected to the interface module",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        help="Display the raw data being exchanged",
        action="store_true",
    )

    subparsers = parser.add_subparsers(required=True)

    parser_list = subparsers.add_parser("list", help="List all connected devices")
    parser_list.set_defaults(cback=do_list)

    parser_mc_dump = subparsers.add_parser(
        "mcdump", help="Dump a Memory Card to a file"
    )
    parser_mc_dump.register("type", "slot", lambda s: Slot(int(s)))
    parser_mc_dump.add_argument(
        "-s",
        "--slot",
        default=1,
        type="slot",
        help="Which slot to dump (default: %(default)s)",
    )
    parser_mc_dump.add_argument(
        "-o",
        "--output",
        help="File where the data should be stored (raw .mcr format)",
        required=True,
    )
    parser_mc_dump.set_defaults(cback=do_mcdump)

    parser_exchange = subparsers.add_parser(
        "exchange", help="Exchange raw data with a slot"
    )
    parser_exchange.register("type", "bint", lambda s: int(s, 0))
    parser_exchange.register("type", "slot", lambda s: Slot(int(s)))
    parser_exchange.add_argument(
        "-s",
        "--slot",
        default=1,
        type="slot",
        help="Which slot to dump (default: %(default)s)",
    )
    parser_exchange.add_argument(
        "rest",
        nargs="+",
        type="bint",
        help="Bytes to send",
    )
    parser_exchange.set_defaults(cback=do_exchange)

    args = parser.parse_args()

    iface = Iface(args.uart)

    iface.verbose = args.verbose

    try:
        iface.send_frame(b"?")
        f = iface.receive_frame(b"?")
    except Exception as e:
        print(f"Couldn't communicate with interface: {e}")
        sys.exit(1)

    if args.cback(iface, args) is not None:
        sys.exit(1)

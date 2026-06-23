#!/usr/bin/env python

import argparse
import serial
import sys
import pprint
from string import printable
from time import sleep
from enum import Enum, auto
from datetime import datetime


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
            timeout=2,
        )

        # Flush any partial command that could be in progress
        self.uart.write(b"0xff" * 256)
        sleep(0.1)
        self.uart.read_all()

        self.verbose = False

    def send_frame(self, bytes):

        if len(bytes) == 0:
            raise ValueError("Frame is empty!")

        if len(bytes) > 255:
            raise ValueError(f"Frame length {len(bytes)} is greater than 255")

        csum = 0

        frame = bytearray(b"\xa5")

        i = 0
        while i < len(bytes):
            b = bytes[i]
            i += 1

            if b == 0:
                # We have an optimized RLE for long runs of 0
                j = i
                while j < len(bytes) and bytes[j] == 0:
                    j += 1

                nzero = min(j - i + 1, 0xF + 3)

                if nzero >= 3:
                    frame.append(0xA7)
                    frame.append(0xB0 + nzero - 3)
                    i += nzero - 1
                else:
                    frame.append(0)

            else:
                csum += b
                frame.append(b)
                if b == 0xA7:
                    # Escape
                    frame.append(b)

        # End marker
        frame.append(0xA7)
        frame.append(len(bytes) & 0x7F)
        frame.append((csum & 0xFF) ^ 0xFF)

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

        expected_csum = 0

        res = bytearray()
        while True:
            b = self.read_or_die()
            if b == 0xA7:
                b = self.read_or_die()
                if b != 0xA7:
                    break

            expected_csum += b
            res.append(b)

        expected_eod = len(res) & 0x7F

        if b != expected_eod:
            raise Exception(f"Invalid end marker, expected {expected_eod} got {b}")

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


def printbytes(bytes, base_addr=0):
    for c in range(0, len(bytes), 16):
        bb = bytes[c : c + 16]

        buf = f"{base_addr + c: 8x}: "

        for i, b in enumerate(bb):
            if i != 0:
                buf += " "
            buf += f"{b:02x}"

        print(purple(buf))

        buf = f"          "
        for i, b in enumerate(bb):
            if i != 0:
                buf += " "
            c = chr(b)

            if c in printable:
                if c == "\n":
                    buf += "\\n"
                elif c == "\r":
                    buf += "\\r"
                elif c == "\t":
                    buf += "\\t"
                elif c == "\f":
                    buf += "\\f"
                elif c == "\v":
                    buf += "\\v"
                else:
                    buf += c + " "
            else:
                buf += " ."

        print(purple(buf))
    print(purple(f"{base_addr + len(bytes): 8x}: "))


def do_list(iface, args):
    slot = Slot(args.slot)

    r = iface.exchange_with_slot(slot, b"\x01\x42\x00\x00\x00")
    if len(r) == 5 and r[1] == 0x41 and r[2] == 0x5A:
        print(f"Slot {slot.value}: GamePad detected")
    else:
        print(f"Slot {slot.value}: No GamePad")

    r = iface.exchange_with_slot(slot, b"\x81\x52\x00\x00")
    if len(r) == 4 and r[2] == 0x5A and r[3] == 0x5D:
        print(f"Slot {slot.value}: Memory Card detected")
    else:
        print(f"Slot {slot.value}: No Memory Card")


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

    print()


def do_mcrestore(iface, args):
    slot = Slot(args.slot)

    with open(args.file, "rb") as f:
        dat = f.read(128 * 1024 + 1)

    if len(dat) != 128 * 1024:
        raise ValueError("MCR file has the wrong size")

    if dat[0] != ord("M") or dat[1] != ord("C"):
        print("Warning: file doesn't appear to be a Memory Card image")

    write_cmd = bytearray(10 + 128)

    write_cmd[0] = 0x81
    write_cmd[1] = 0x57

    for page in range(0, 0x400):
        print(f"\r{page + 1} / {0x400}", end="")

        page_hi = page >> 8
        page_lo = page & 0xFF

        write_cmd[4] = page_hi
        write_cmd[5] = page_lo

        pstart = page * 128
        pdat = dat[pstart : pstart + 128]

        xsum = page_hi ^ page_lo
        for i, b in enumerate(pdat):
            write_cmd[6 + i] = b
            xsum ^= b

        write_cmd[6 + 128] = xsum

        r = iface.exchange_with_slot(slot, write_cmd)

        if (
            len(r) != len(write_cmd)
            or r[2] != 0x5A
            or r[3] != 0x5D
            or r[135] != 0x5C
            or r[136] != 0x5D
            or r[137] != 0x47
        ):
            print(f"\nInvalid Memory Card response at page {page}")
            printbytes(r)
            return False

        # Memory Cards seem to need a cooldown between writes. I get errors with
        # 0.003 (although I do manage to write ~38 pages or so) but 0.004 seems
        # to work. Actually no, it works with the PocketStation but seems to
        # fail with an official Memory Card (after exactly 512 sectors)
        sleep(0.01)

    print()


def do_exchange(iface, args):
    slot = Slot(args.slot)
    tx = bytearray(args.bytes)

    v = iface.verbose
    iface.verbose = True

    iface.exchange_with_slot(slot, tx)

    iface.verbose = v


def pks_memread(slot, addr, mlen, progress=False):
    full_len = mlen
    mem = bytearray()
    read_cmd = bytearray(11 + 0x80)

    read_cmd[0] = 0x81
    read_cmd[1] = 0x5B  # PocketStation command PKSX -> PSX
    read_cmd[2] = 0x01  # Function 1: read memory

    if mlen == 0:
        raise ValueError("Length is 0!")

    if addr + mlen > (2**32):
        raise ValueError("Addr + Len is out of bounds!")

    while mlen > 0:
        dlen = min(mlen, 0x80)

        read_cmd[4] = addr & 0xFF
        read_cmd[5] = (addr >> 8) & 0xFF
        read_cmd[6] = (addr >> 16) & 0xFF
        read_cmd[7] = (addr >> 24) & 0xFF
        read_cmd[8] = dlen

        r = iface.exchange_with_slot(slot, read_cmd[0 : dlen + 11])

        if (
            len(r) != dlen + 11
            or r[3] != 0x5  # Argument count
            or r[9] != dlen
            or r[dlen + 10] != 0xFF
        ):
            print(f"\nInvalid PocketStation response at 0x{addr:x}")
            printbytes(r)
            return None

        mem += r[10 : 10 + dlen]

        if progress:
            print(f"\r{len(mem)} / {full_len}", end="")

        mlen -= dlen
        addr += dlen

    if progress:
        print()

    return mem


def pks_memwrite(slot, addr, data, progress=False):
    write_cmd = bytearray(11 + 0x80)

    write_cmd[0] = 0x81
    write_cmd[1] = 0x5C  # PocketStation command PSX -> PKSX
    write_cmd[2] = 0x01  # Function 1: write memory

    mlen = len(data)

    if mlen == 0:
        raise ValueError("Length is 0!")

    if addr + mlen > (2**32):
        raise ValueError("Addr + Len is out of bounds!")

    di = 0

    while mlen > 0:
        dlen = min(mlen, 0x80)

        write_cmd[4] = addr & 0xFF
        write_cmd[5] = (addr >> 8) & 0xFF
        write_cmd[6] = (addr >> 16) & 0xFF
        write_cmd[7] = (addr >> 24) & 0xFF
        write_cmd[8] = dlen

        for i in range(0, dlen):
            write_cmd[10 + i] = data[di]
            di += 1

        r = iface.exchange_with_slot(slot, write_cmd[0 : dlen + 11])

        if (
            len(r) != dlen + 11
            or r[3] != 0x5  # Argument count
            or r[9] != dlen
            or r[dlen + 10] != 0xFF
        ):
            print(f"\nInvalid PocketStation response at 0x{addr:x}")
            printbytes(r)
            return None

        if progress:
            print(f"\r{di} / {len(data)}", end="")

        mlen -= dlen
        addr += dlen

    if progress:
        print()


def do_pks_memread(iface, args):
    slot = Slot(args.slot)
    addr = args.address
    mlen = args.length

    mem = pks_memread(slot, addr, mlen, progress=True)

    if args.output:
        f = open(args.output, "bw")
        f.write(mem)
        f.flush()
    else:
        printbytes(mem, base_addr=args.address)


def do_pks_memwrite(iface, args):
    slot = Slot(args.slot)
    addr = args.address

    if args.input:
        if args.bytes:
            raise ValueError("Can't both provide bytes and a file!")

        with open(args.input, "rb") as f:
            dat = f.read(128 * 1024 + 1)
    else:
        dat = args.bytes

    if not dat:
        raise ValueError("Nothing to write!")

    if len(dat) > 128 * 1024:
        raise ValueError("Data is too large!")

    pks_memwrite(slot, addr, dat, progress=True)


def do_pks_showdisplay(iface, args):
    slot = Slot(args.slot)

    lcd_bytes = pks_memread(slot, 0xD000100, 0x80)

    if not lcd_bytes:
        print("Couldn't read LCD memory")
        return False

    lcd = []

    for i in range(0, 0x80, 4):
        lcd.append(int.from_bytes(lcd_bytes[i : i + 4], "little"))

    print(" .------------------------------------------------------------------.")
    for line in lcd:
        print(" | ", end="")
        for x in range(0, 32):
            if (line >> x) & 1:
                print("@@", end="")
            else:
                print("  ", end="")
        print(" |")
    print(" '------------------------------------------------------------------'")


def do_pks_rtcread(iface, args):
    slot = Slot(args.slot)
    read_cmd = bytearray(14)

    read_cmd[0] = 0x81
    read_cmd[1] = 0x5B  # PocketStation command PKSX -> PSX
    read_cmd[2] = 0x00  # Function 0: read RTC

    r = iface.exchange_with_slot(slot, read_cmd)

    if (
        len(r) != len(read_cmd)
        or r[3] != 0x0  # Argument count
        or r[4] != 8
        or r[13] != 0xFF
    ):
        print(f"\nInvalid PocketStation response")
        printbytes(r)
        return False

    def from_bcd(b):
        h = (b >> 4) & 0xF
        l = b & 0xF

        if h > 9 or l > 9:
            print(f"\nInvalid BCD value: {b:02x}")

        return h * 10 + l

    mday = from_bcd(r[5])
    month = from_bcd(r[6])
    year = from_bcd(r[7]) + from_bcd(r[8]) * 100
    secs = from_bcd(r[9])
    mins = from_bcd(r[10])
    hours = from_bcd(r[11])
    wday = from_bcd(r[12])

    wday_en = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"][wday - 1]

    print(f"{year:04}-{month:02}-{mday:02} {hours:02}:{mins:02}:{secs:02} - {wday_en}")


def do_pks_rtcsync(iface, args):
    slot = Slot(args.slot)
    cmd = bytearray(14)

    now = datetime.now()

    def to_bcd(b):
        if b > 99:
            print(f"\nInvalid decimal BCD value: {b}")

        h, l = divmod(b, 10)

        h = b // 10
        l = b % 10

        return (h << 4) | l

    cmd[0] = 0x81
    cmd[1] = 0x5C  # PocketStation command PSX -> PKSX
    cmd[2] = 0x00  # Function 0: write RTC
    cmd[5] = to_bcd(now.day)
    cmd[6] = to_bcd(now.month)
    cmd[7] = to_bcd(now.year % 100)
    cmd[8] = to_bcd(now.year // 100)
    cmd[9] = to_bcd(now.second)
    cmd[10] = to_bcd(now.minute)
    cmd[11] = to_bcd(now.hour)
    cmd[12] = to_bcd((now.weekday() + 1) % 7)

    r = iface.exchange_with_slot(slot, cmd)

    if (
        len(r) != len(cmd)
        or r[3] != 0x0  # Argument count
        or r[4] != 8
        or r[13] != 0xFF
    ):
        print(f"\nInvalid PocketStation response")
        printbytes(r)
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PlayStation Pad/MemoryCard interface")
    parser.register("type", "slot", lambda s: Slot(int(s)))

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
    parser.register("type", "slot", lambda s: Slot(int(s)))
    parser.add_argument(
        "-s",
        "--slot",
        default=1,
        type="slot",
        help="Which slot to use (default: %(default)s)",
    )

    subparsers = parser.add_subparsers(required=True)

    parser_list = subparsers.add_parser("list", help="List all connected devices")
    parser_list.set_defaults(cback=do_list)

    parser_mc_dump = subparsers.add_parser(
        "mcdump", help="Dump a Memory Card to a file"
    )
    parser_mc_dump.add_argument(
        "-o",
        "--output",
        help="File where the data should be stored (raw .mcr format)",
        required=True,
    )
    parser_mc_dump.set_defaults(cback=do_mcdump)

    parser_mc_restore = subparsers.add_parser(
        "mcrestore", help="Restore Memory Card contents from a mcr file"
    )
    parser_mc_restore.add_argument(
        "file",
        help="File containing the raw .mcr image",
    )
    parser_mc_restore.set_defaults(cback=do_mcrestore)

    parser_exchange = subparsers.add_parser(
        "exchange", help="Exchange raw data with a slot"
    )
    parser_exchange.register("type", "bint", lambda s: int(s, 0))
    parser_exchange.register("type", "slot", lambda s: Slot(int(s)))
    parser_exchange.add_argument(
        "bytes",
        nargs="+",
        type="bint",
        help="Bytes to send",
    )
    parser_exchange.set_defaults(cback=do_exchange)

    parser_pks_memread = subparsers.add_parser(
        "pks-memread", help="Read the PocketStation memory"
    )
    parser_pks_memread.register("type", "bint", lambda s: int(s, 0))
    parser_pks_memread.add_argument(
        "-o",
        "--output",
        help="File where the binary data should be stored",
    )
    parser_pks_memread.add_argument(
        "address",
        type="bint",
        help="Start address in PocketStation memory",
    )
    parser_pks_memread.add_argument(
        "length",
        type="bint",
        help="How many bytes to read",
    )
    parser_pks_memread.set_defaults(cback=do_pks_memread)

    parser_pks_memwrite = subparsers.add_parser(
        "pks-memwrite", help="Read the PocketStation memory"
    )
    parser_pks_memwrite.register("type", "bint", lambda s: int(s, 0))
    parser_pks_memwrite.add_argument(
        "-i",
        "--input",
        help="File containing the binary data to be sent",
    )
    parser_pks_memwrite.add_argument(
        "address",
        type="bint",
        help="Start address in PocketStation memory",
    )
    parser_pks_memwrite.add_argument(
        "bytes",
        nargs="*",
        type="bint",
        help="Bytes to send",
    )
    parser_pks_memwrite.set_defaults(cback=do_pks_memwrite)

    parser_pks_rtcread = subparsers.add_parser(
        "pks-rtcread", help="Read the PocketStation RTC date"
    )
    parser_pks_rtcread.set_defaults(cback=do_pks_rtcread)

    parser_pks_rtcsync = subparsers.add_parser(
        "pks-rtcsync", help="Set PocketStation RTC date from the computer clock"
    )
    parser_pks_rtcsync.set_defaults(cback=do_pks_rtcsync)

    parser_pks_showdisplay = subparsers.add_parser(
        "pks-showdisplay", help="Dump the current contents of the PocketStation LCD"
    )
    parser_pks_showdisplay.set_defaults(cback=do_pks_showdisplay)

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

#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
"""Terminal for the NuttX Lightweight Link (LWL) console over OpenOCD.

Start OpenOCD for the target first, then run e.g.:

  $ tools/lwl/ocdconsole.py --elf nuttx
  ==Link Activated

  nsh> help

The tool talks to OpenOCD through its TCL RPC server (port 6666) and needs
no extra python modules.  Press Ctrl-] to quit, every other key, including
Ctrl-C, is sent to the target.  See Documentation/guides/drivers/lwl.rst.
"""

import argparse
import os
import socket
import struct
import sys
import time

if os.name == "nt":
    import msvcrt
else:
    import select
    import termios
    import tty

LWL_ACTIVE = 1 << 31
LWL_DNSENSEBIT = 1 << 30
LWL_UPSENSEBIT = 1 << 29
LWL_OCTVALSHIFT = 27
LWL_PORTSHIFT = 24

LWL_SIG = 0x7216A318
LWL_EXT_SIG = 0x4C574C32
LWL_PORT_CONSOLE = 1
LWL_CAP_RING = 1 << 0

# Control block layout, as word offsets from the signature

LWL_DOWNWORD = 1
LWL_UPWORD = 2
LWL_EXTSIG = 3
LWL_DBADDR = 4
LWL_DBVAL = 5
LWL_RINGSIZE = 6
LWL_HOSTCAPS = 16
LWL_UPTAIL = 17
LWL_STATUS = 32
LWL_UPHEAD = 33
LWL_RING = 48

# Symbols holding the control block (current and legacy firmware)

LWL_SYMBOLS = ("g_lwl", "g_d")

# Default RAM ranges searched for the signature when neither --elf nor
# --addr is given: the control block is initialized data, which is usually
# at the start of the RAM: the SRAM of most Cortex-M parts and the AXI SRAM
# of STM32H7.

DEFAULT_SCAN = ("0x20000000:0x10000", "0x24000000:0x10000")

QUIT_KEY = 0x1D  # Ctrl-]
IDLE_POLL = 0.02  # Seconds between polls while the link is idle
RING_RETRY = 0.05  # Seconds before an unacknowledged doorbell is re-rung


class LinkLost(Exception):
    pass


class OpenOCD:
    """Minimal client for the OpenOCD TCL RPC server."""

    TERMINATOR = b"\x1a"

    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.has_read_memory = self.send("info commands read_memory") != ""

    def close(self):
        self.sock.close()

    def send(self, cmd):
        """Run a TCL script and return its result."""
        self.sock.sendall(cmd.encode() + self.TERMINATOR)
        data = b""
        while not data.endswith(self.TERMINATOR):
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionResetError("OpenOCD closed the connection")
            data += chunk
        return data[:-1].decode(errors="replace").strip()

    def read_cmd(self, addr, count):
        if self.has_read_memory:
            return "read_memory 0x%x 32 %d" % (addr, count)
        return "mdw 0x%x %d" % (addr, count)

    def parse_words(self, text):
        try:
            if self.has_read_memory:
                return [int(w, 16) for w in text.split()]

            words = []
            for line in text.splitlines():
                if ":" in line:
                    words += [int(w, 16) for w in line.split(":", 1)[1].split()]
            return words
        except ValueError:
            return []  # OpenOCD reported an error

    def read(self, addr, count=1):
        words = self.parse_words(self.send(self.read_cmd(addr, count)))
        if len(words) != count:
            raise LinkLost("cannot read target memory at 0x%x" % addr)
        return words


def elf_symbol(path, names):
    """Return the address of the first of 'names' found in an ELF32 file."""

    with open(path, "rb") as f:
        elf = f.read()

    if elf[:4] != b"\x7fELF" or elf[4] != 1:
        raise ValueError("%s is not an ELF32 file" % path)

    end = "<" if elf[5] == 1 else ">"
    (shoff,) = struct.unpack_from(end + "I", elf, 0x20)
    shentsize, shnum = struct.unpack_from(end + "HH", elf, 0x2E)

    sections = [
        struct.unpack_from(end + "IIIIIIIIII", elf, shoff + i * shentsize)
        for i in range(shnum)
    ]

    found = {}
    for sh in sections:
        if sh[1] != 2:  # SHT_SYMTAB
            continue

        strtab = sections[sh[6]]
        for off in range(sh[4], sh[4] + sh[5], 16):
            name, value = struct.unpack_from(end + "II", elf, off)
            stroff = strtab[4] + name
            sym = elf[stroff : elf.index(b"\0", stroff)].decode()
            if sym in names:
                found[sym] = value

    for name in names:
        if name in found:
            return found[name]

    raise ValueError("no LWL control block (%s) in %s" % (", ".join(names), path))


def scan(ocd, ranges):
    """Search RAM for the LWL signature."""

    block = 1024
    found = []
    for base, length in ranges:
        for addr in range(base, base + length, block * 4):
            count = min(block, (base + length - addr) // 4)
            try:
                words = ocd.read(addr, count)
            except LinkLost:
                break  # Not readable, go to the next range

            for i, word in enumerate(words):
                if word == LWL_SIG:
                    sig = addr + i * 4
                    ext = ocd.read(sig + LWL_EXTSIG * 4)[0] == LWL_EXT_SIG
                    found.append((sig, ext))

    # RAM keeps stale copies of the control block across resets, prefer the
    # current format but don't guess: writing to the wrong place could
    # corrupt the target.

    extended = [addr for addr, ext in found if ext]
    candidates = extended or [addr for addr, ext in found]
    if len(candidates) == 1:
        return candidates[0]

    if not candidates:
        raise SystemExit("Cannot find the LWL signature, use --elf or --addr")

    raise SystemExit(
        "LWL signature found at %s, use --elf or --addr to select one"
        % ", ".join("0x%x" % addr for addr in candidates)
    )


class Console:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.saved = None

    def __enter__(self):
        if os.name != "nt" and os.isatty(self.fd):
            self.saved = termios.tcgetattr(self.fd)
            tty.setraw(self.fd)
        return self

    def __exit__(self, *args):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def read(self, timeout):
        """Return pending keyboard input, waiting at most 'timeout' s."""

        if os.name == "nt":
            end = time.monotonic() + timeout
            while not msvcrt.kbhit() and time.monotonic() < end:
                time.sleep(0.005)
            data = b""
            while msvcrt.kbhit():
                data += msvcrt.getch()
            return data

        ready, _, _ = select.select([self.fd], [], [], timeout)
        return os.read(self.fd, 64) if ready else b""

    @staticmethod
    def write(data):
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()


class Link:
    def __init__(self, ocd, base, verbose):
        self.ocd = ocd
        self.base = base
        self.down = 0
        self.tail = 0

        words = ocd.read(base, LWL_RINGSIZE + 1)
        if words[0] != LWL_SIG:
            raise LinkLost("no LWL signature at 0x%x" % base)

        # Firmware with the extended control block advertises a doorbell
        # and a ring buffer, and handles the line discipline itself.

        self.legacy = words[LWL_EXTSIG] != LWL_EXT_SIG
        self.ring = 0 if self.legacy else words[LWL_RINGSIZE]
        self.doorbell = ""
        if not self.legacy and words[LWL_DBADDR] != 0:
            self.doorbell = "mww 0x%x 0x%x" % (words[LWL_DBADDR], words[LWL_DBVAL])

        # With the ring the target reports its state in the status word,
        # read together with the ring in a single access.

        if self.ring:
            self.state = (self.addr(LWL_STATUS), LWL_RING - LWL_STATUS + self.ring // 4)
        else:
            self.state = (self.addr(LWL_UPWORD), 1)

        if verbose:
            print(
                "==LWL at 0x%x, %s, %s, %s\r"
                % (
                    base,
                    "legacy firmware" if self.legacy else "extended firmware",
                    "%d bytes ring" % self.ring if self.ring else "word protocol",
                    (
                        "doorbell at 0x%x" % words[LWL_DBADDR]
                        if self.doorbell
                        else "polled"
                    ),
                )
            )

    def addr(self, index):
        return self.base + index * 4

    def write(self, index, value):
        return "mww 0x%x 0x%x" % (self.addr(index), value)

    def update(self, cmds):
        """Run the writes, ring the doorbell and read the target state."""

        if cmds and self.doorbell:
            cmds = cmds + [self.doorbell]
        cmds = cmds + [self.ocd.read_cmd(*self.state)]
        words = self.ocd.parse_words(self.ocd.send("; ".join(cmds)))
        if len(words) != self.state[1]:
            raise LinkLost("cannot access target memory")
        return words

    def wait(self, index, active, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            cmds = [self.write(LWL_DOWNWORD, self.down)]
            if bool(self.update(cmds)[0] & LWL_ACTIVE) == active:
                if bool(self.ocd.read(self.addr(index))[0] & LWL_ACTIVE) == active:
                    return True
            time.sleep(RING_RETRY)
        return False

    def open(self):
        # Close the link first, so that the target forgets any state left
        # over by a previous session, then (re)open it.  Legacy firmware
        # never reports a closed link.

        index = LWL_STATUS if self.ring else LWL_UPWORD
        self.down = 0
        if not self.legacy:
            self.wait(LWL_UPWORD, False, 1.0)
            self.wait(LWL_STATUS, False, 1.0)

        if self.ring:
            self.tail = 0
            self.update(
                [self.write(LWL_HOSTCAPS, LWL_CAP_RING), self.write(LWL_UPTAIL, 0)]
            )

        self.down = LWL_ACTIVE
        while not self.wait(index, True, 1.0):
            print("==Waiting for the target\r")

    def close(self):
        # Legacy firmware reports end of file to the reader when the link
        # goes down, which would terminate the shell, so leave it open.

        if not self.legacy:
            self.down = 0
            self.update([self.write(LWL_DOWNWORD, 0), self.write(LWL_HOSTCAPS, 0)])

    def output(self, console, state):
        """Output the new data, return the writes acknowledging it."""

        if self.ring:
            head = state[LWL_UPHEAD - LWL_STATUS] % self.ring
            if head == self.tail:
                return []

            ring = b"".join(
                w.to_bytes(4, "little") for w in state[LWL_RING - LWL_STATUS :]
            )
            if head > self.tail:
                console.write(ring[self.tail : head])
            else:
                console.write(ring[self.tail :] + ring[:head])

            self.tail = head
            return [self.write(LWL_UPTAIL, head)]

        upword = state[0]
        if not (upword ^ self.down) & LWL_UPSENSEBIT:
            return []

        if (upword >> LWL_PORTSHIFT) & 7 == LWL_PORT_CONSOLE:
            count = (upword >> LWL_OCTVALSHIFT) & 3
            data = (upword & 0xFFFFFF).to_bytes(3, "little")[:count]
            if self.legacy:
                data = data.replace(b"\n", b"\r\n")
            console.write(data)

        self.down ^= LWL_UPSENSEBIT
        return [self.write(LWL_DOWNWORD, self.down)]

    def run(self, console):
        pending = b""
        state = self.update([])
        last_ring = time.monotonic()

        while True:
            ack = state[0]
            if not ack & LWL_ACTIVE:
                raise LinkLost("link deactivated by the target")

            # Data from the target

            cmds = self.output(console, state)

            # Data to the target, once it took the previous word

            acked = not (ack ^ self.down) & LWL_DNSENSEBIT
            idle = not cmds and acked
            pending += console.read(0 if not idle or pending else IDLE_POLL)

            if QUIT_KEY in pending:
                return

            if acked and pending:
                # Legacy firmware only takes the first octet of a word

                size = 1 if self.legacy else 3
                data, pending = pending[:size], pending[size:]
                if self.legacy:
                    data = data.replace(b"\r", b"\n")

                self.down = (
                    (self.down & (LWL_ACTIVE | LWL_UPSENSEBIT))
                    | ((self.down ^ LWL_DNSENSEBIT) & LWL_DNSENSEBIT)
                    | (len(data) << LWL_OCTVALSHIFT)
                    | (LWL_PORT_CONSOLE << LWL_PORTSHIFT)
                    | int.from_bytes(data, "little")
                )
                cmds.append(self.write(LWL_DOWNWORD, self.down))

            # Ring again if the target sits on our data, its RX buffer may
            # have been full when it got the previous doorbell.

            now = time.monotonic()
            if not cmds and not acked and now - last_ring > RING_RETRY:
                cmds.append(self.write(LWL_DOWNWORD, self.down))

            if cmds:
                last_ring = now

            state = self.update(cmds)


def parse_range(text):
    base, length = text.split(":")
    return int(base, 0), int(length, 0)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--host", default="127.0.0.1", help="OpenOCD host")
    parser.add_argument(
        "--port", type=int, default=6666, help="OpenOCD TCL RPC port (6666)"
    )
    where = parser.add_mutually_exclusive_group()
    where.add_argument("--elf", help="NuttX ELF file to take the address from")
    where.add_argument(
        "--addr", type=lambda x: int(x, 0), help="Address of the control block"
    )
    where.add_argument(
        "--scan",
        action="append",
        type=parse_range,
        metavar="BASE:LEN",
        help="RAM range to search for the control block (repeatable)",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Show link details"
    )
    args = parser.parse_args()

    base = args.addr
    if args.elf:
        base = elf_symbol(args.elf, LWL_SYMBOLS)

    with Console() as console:
        while True:
            try:
                ocd = OpenOCD(args.host, args.port)
            except OSError:
                time.sleep(1)
                continue

            try:
                addr = base
                if addr is None:
                    addr = scan(ocd, args.scan or map(parse_range, DEFAULT_SCAN))

                link = Link(ocd, addr, args.verbose)
                link.open()
                print("==Link Activated\r")
                try:
                    link.run(console)
                finally:
                    link.close()
                print("\r\n==Link Closed\r")
                return 0
            except LinkLost as e:
                print("\r\n==Link Lost: %s\r" % e)
                time.sleep(1)
            except OSError:
                print("\r\n==Connection to OpenOCD lost\r")
                time.sleep(1)
            finally:
                ocd.close()


if __name__ == "__main__":
    sys.exit(main())

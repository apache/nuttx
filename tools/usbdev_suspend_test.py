#!/usr/bin/env python3
# tools/usbdev_suspend_test.py
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

"""Check that a NuttX USB CDC/ACM link survives Linux USB runtime suspend.

A USB device driver that reports CLASS_SUSPEND but never CLASS_RESUME leaves
cdcacm_suspend()'s uart_connected(&priv->serdev, false) latched, after which
serial.c refuses every board-side open() and write() on the CDC port with
-ENOTCONN.  The device stays enumerated throughout, so the failure looks like a
random USB wedge rather than a deterministic one.

Linux hosts reach that state on their own: with power/control=auto and the
usual autosuspend_delay_ms=2000, closing the tty is enough.

Each cycle here forces a real runtime suspend, resumes the device by opening
the port, and measures what comes back.  A link that only flushes its stale
CDC TX buffer is caught by the tail measurement: bytes are counted both over
the whole read window and over its last second.

The board must transmit unprompted for this to measure anything -- a console
banner, a telemetry stream, anything periodic.

  ./tools/usbdev_suspend_test.py -d /dev/ttyACM0
  ./tools/usbdev_suspend_test.py -d /dev/ttyACM0 --console /dev/ttyUSB0

Requires a Linux host, pyserial, and sudo for two sysfs power attributes.
"""

import argparse
import errno
import glob
import os
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")


# ---------------------------------------------------------------- discovery


def usb_device_dir(tty_path):
    """Map a tty device node to the sysfs directory of its USB device."""
    tty = os.path.basename(os.path.realpath(tty_path))
    link = "/sys/class/tty/%s/device" % tty

    if not os.path.exists(link):
        raise RuntimeError("%s is not a tty backed by sysfs" % tty_path)

    node = os.path.realpath(link)

    # Walk up from the USB interface to the USB device that owns it.
    while node != "/":
        if os.path.exists(os.path.join(node, "idVendor")):
            return node

        node = os.path.dirname(node)

    raise RuntimeError(
        "%s is not on a USB device (no idVendor in any parent)" % tty_path
    )


def autodetect():
    """Pick the CDC/ACM port when exactly one is present."""
    found = []

    for path in sorted(glob.glob("/dev/serial/by-id/*")):
        if not os.path.basename(os.path.realpath(path)).startswith("ttyACM"):
            continue

        try:
            found.append((path, usb_device_dir(path)))
        except RuntimeError:
            continue

    if not found:
        sys.exit("no CDC/ACM port found, pass --device")

    if len(found) > 1:
        listing = "\n".join(
            "  %s  (%s)" % (path, read_attr(usb, "product")) for path, usb in found
        )
        sys.exit("several CDC/ACM ports present, pass --device:\n" + listing)

    return found[0][0]


# ------------------------------------------------------------------- sysfs


def read_attr(base, name):
    try:
        with open(os.path.join(base, name)) as handle:
            return handle.read().strip()
    except OSError:
        return None


class UsbPower:
    """Runtime PM knobs of one USB device, restored on exit."""

    def __init__(self, usbdir):
        self.dir = os.path.join(usbdir, "power")
        self.saved = {key: self.get(key) for key in ("control", "autosuspend_delay_ms")}

    def get(self, name):
        return read_attr(self.dir, name)

    def set(self, name, value):
        target = os.path.join(self.dir, name)
        proc = subprocess.run(
            ["sudo", "tee", target],
            input=str(value).encode(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )

        if proc.returncode != 0:
            sys.exit("cannot write %s: %s" % (target, proc.stderr.decode().strip()))

    def restore(self):
        for name, value in self.saved.items():
            if value is not None:
                self.set(name, value)

    def arm(self, delay_ms):
        """Re-arm the autosuspend timer.

        Once resumed, a device will not idle out again on its own until
        runtime PM is toggled, so every cycle has to re-allow it.
        """
        self.set("autosuspend_delay_ms", delay_ms)
        self.set("control", "on")
        self.set("control", "auto")

    def wait_suspended(self, timeout):
        deadline = time.time() + timeout

        while time.time() < deadline:
            if self.get("runtime_status") == "suspended":
                return True

            time.sleep(0.1)

        return False


# -------------------------------------------------------------------- test


def port_holders(dev):
    """Processes with the port open, as (pid, command) pairs."""
    real = os.path.realpath(dev)
    holders = []

    for entry in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            if os.readlink(entry) != real:
                continue

            pid = entry.split("/")[2]

            with open("/proc/%s/comm" % pid) as handle:
                holders.append((pid, handle.read().strip()))
        except OSError:
            continue  # process exited, or not ours to inspect

    return sorted(set(holders))


def require_free_port(dev):
    """Abort unless the port is closed.

    An open port pins runtime PM, so the host never suspends and the test
    silently measures nothing.
    """
    try:
        serial.Serial(os.path.realpath(dev), 115200, timeout=0.2).close()
        return
    except OSError as exc:
        if exc.errno != errno.EBUSY:
            sys.exit("cannot open %s: %s" % (dev, exc))

    holders = port_holders(dev)
    who = ", ".join("%s (pid %s)" % (name, pid) for pid, name in holders)
    sys.exit(
        "%s is held open by %s.\nClose it first: an open port keeps the "
        "device active, so it never suspends." % (dev, who or "another process")
    )


def read_stream(dev, seconds, tail_seconds):
    """Read for `seconds`, returning (total bytes, bytes in the last window)."""
    try:
        port = serial.Serial(os.path.realpath(dev), 115200, timeout=0.2)
    except OSError as exc:
        return -1, -1, str(exc)

    start = time.time()
    total = 0
    tail = 0

    try:
        while True:
            now = time.time() - start

            if now >= seconds:
                break

            chunk = port.read(4096)
            total += len(chunk)

            if now >= seconds - tail_seconds:
                tail += len(chunk)
    finally:
        port.close()

    return total, tail, ""


def console_probe(console, baud, cdc_path):
    """Ask the board itself whether its CDC port is writable."""
    try:
        port = serial.Serial(console, baud, timeout=0.2)
    except OSError as exc:
        return "console unavailable (%s)" % exc

    try:
        port.write(b"\n")
        port.flush()
        time.sleep(0.3)
        port.reset_input_buffer()
        port.write(b"echo probe > %s\n" % cdc_path.encode())
        port.flush()

        deadline = time.time() + 2.0
        out = b""

        while time.time() < deadline:
            chunk = port.read(4096)

            if chunk:
                out += chunk
                deadline = time.time() + 0.5
    finally:
        port.close()

    text = out.decode(errors="replace")

    if "not connected" in text or "ENOTCONN" in text:
        return "board-side open failed: -ENOTCONN"

    if "failed" in text:
        return "board-side open failed: %s" % text.strip().splitlines()[-1]

    return "board-side open ok"


def main():
    parser = argparse.ArgumentParser(
        description="Verify a USB CDC/ACM link survives host runtime suspend."
    )
    parser.add_argument(
        "-d", "--device", help="CDC/ACM port of the board (default: autodetect)"
    )
    parser.add_argument(
        "-n", "--cycles", type=int, default=5, help="suspend/resume cycles (default: 5)"
    )
    parser.add_argument(
        "--delay-ms",
        type=int,
        default=1000,
        help="autosuspend delay to force (default: 1000)",
    )
    parser.add_argument(
        "--read-secs",
        type=float,
        default=2.0,
        help="read window per cycle (default: 2.0)",
    )
    parser.add_argument(
        "--tail-secs",
        type=float,
        default=1.0,
        help="trailing part of the window that must carry data",
    )
    parser.add_argument(
        "--min-bytes",
        type=int,
        default=2000,
        help="bytes required in the tail window (default: 2000)",
    )
    parser.add_argument(
        "--console",
        help="board console, to probe the port from the board after resume",
    )
    parser.add_argument(
        "--console-baud",
        type=int,
        default=115200,
        help="baud rate of that console (default: 115200)",
    )
    parser.add_argument(
        "--cdc-path",
        default="/dev/ttyACM0",
        help="CDC path as the board sees it (default: /dev/ttyACM0)",
    )
    args = parser.parse_args()

    dev = args.device or autodetect()
    usbdir = usb_device_dir(dev)
    require_free_port(dev)
    power = UsbPower(usbdir)

    print("device      %s -> %s" % (dev, os.path.realpath(dev)))
    print(
        "usb         %s  %s:%s  %s"
        % (
            os.path.basename(usbdir),
            read_attr(usbdir, "idVendor"),
            read_attr(usbdir, "idProduct"),
            read_attr(usbdir, "product"),
        )
    )
    print(
        "power       control=%s autosuspend_delay_ms=%s (restored on exit)"
        % (power.saved["control"], power.saved["autosuspend_delay_ms"])
    )
    print()

    results = []

    try:
        for i in range(1, args.cycles + 1):
            power.arm(args.delay_ms)
            suspended = power.wait_suspended(args.delay_ms / 1000.0 + 6.0)

            # Opening the port is what resumes the device.
            total, tail, err = read_stream(dev, args.read_secs, args.tail_secs)
            note = ""

            if args.console and suspended:
                # -ENOTCONN *during* suspend is correct on any build; the
                # defect is that it outlives the resume. Pin the device
                # active so the board is asked about the resumed state.
                power.set("control", "on")
                note = console_probe(args.console, args.console_baud, args.cdc_path)

            if err:
                print("[%d] suspended=%-5s  OPEN FAILED: %s" % (i, suspended, err))
            else:
                print(
                    "[%d] suspended=%-5s  read=%-7d tail=%-7d %s"
                    % (i, suspended, total, tail, note)
                )

            results.append((suspended, tail))
    finally:
        power.restore()
        print()
        print(
            "power       restored to control=%s autosuspend_delay_ms=%s"
            % (power.get("control"), power.get("autosuspend_delay_ms"))
        )

    tested = [tail for suspended, tail in results if suspended]
    good = [tail for tail in tested if tail >= args.min_bytes]

    print()

    if not tested:
        print(
            "INCONCLUSIVE: the host never suspended the device, so the bug "
            "was never exercised."
        )
        print(
            "Check %s/power/runtime_usage; something holds a runtime PM "
            "reference." % usbdir
        )
        return 2

    print("cycles with a verified suspend: %d/%d" % (len(tested), len(results)))
    print("of those, still streaming after resume: %d/%d" % (len(good), len(tested)))
    print()

    if len(good) == len(tested):
        print("PASS: the link recovered from every suspend.")
        return 0

    print("FAIL: the link died after suspend and did not come back.")
    print(
        "Expected when the resume event never reaches the class driver: the "
        "first cycle can still flush the stale CDC TX buffer, later cycles "
        "read nothing."
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())

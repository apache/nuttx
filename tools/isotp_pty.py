#!/usr/bin/env python3
# tools/isotp_pty.py

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

"""ISO-TP <-> PTY bridge using the Linux kernel CAN_ISOTP socket.

Creates a pseudo-terminal (PTY) that any tool can connect to:
  minicom:  minicom -D /tmp/isotp-pty
  GDB:      target remote /tmp/isotp-pty

Usage:
  sudo modprobe can_isotp
  python3 tools/isotp_pty.py can0 7D0 7D1
  python3 tools/isotp_pty.py can0 7E0 7E1 --symlink /tmp/isotp-pty-2

  can0  - CAN interface
  7D0   - rx_id: CAN ID NuttX transmits on  (we receive from here)
  7D1   - tx_id: CAN ID NuttX receives on   (we send with this)

The slave PTY is symlinked at /tmp/isotp-pty for easy access.
Run multiple bridge instances as separate processes. Use a unique --symlink
path per instance, and avoid reusing the same (interface, rx_id, tx_id) tuple.
Press Ctrl+C to stop.
"""

import argparse
import errno
import os
import pty
import select
import socket
import struct
import sys
import threading
import time
import tty

CAN_ISOTP = getattr(socket, "CAN_ISOTP", 6)
SOL_CAN_ISOTP = 106  # SOL_CAN_BASE (100) + 6
CAN_ISOTP_RECV_FC = 2
CAN_ISOTP_LL_OPTS = 5
CANFD_MTU = 72
PTY_SYMLINK = "/tmp/isotp-pty"


def isotp_to_pty(sock, master_fd, stop, debug):
    """Forward ISO-TP socket -> PTY master."""
    while not stop.is_set():
        try:
            r, _, _ = select.select([sock], [], [], 0.2)
        except Exception:
            break
        if not r:
            continue
        try:
            data = sock.recv(4096)
        except OSError as e:
            print(f"[isotp->pty] recv error ({e})", flush=True)
            continue
        if data:
            if debug:
                print(f"[isotp->pty] {len(data)} bytes: {data.hex()}", flush=True)
            try:
                os.write(master_fd, data)
            except OSError as e:
                if e.errno == errno.EBADF:
                    break
                print(f"[isotp->pty] write error ({e}), retrying", flush=True)
                time.sleep(0.1)
    stop.set()


def pty_to_isotp(master_fd, sock, stop, debug):
    """Forward PTY master -> ISO-TP socket."""
    while not stop.is_set():
        try:
            r, _, _ = select.select([master_fd], [], [], 0.2)
        except Exception:
            break
        if not r:
            continue
        try:
            data = os.read(master_fd, 4096)
        except OSError as e:
            if e.errno == errno.EBADF:
                break
            print(f"[pty->isotp] read error ({e}), retrying", flush=True)
            time.sleep(0.1)
            continue
        if data:
            if debug:
                print(f"[pty->isotp] {len(data)} bytes: {data.hex()}", flush=True)
            try:
                sock.send(data)
            except OSError as e:
                if e.errno == errno.EBADF:
                    break
                print(f"[pty->isotp] send error ({e}), retrying", flush=True)
                time.sleep(0.1)
    stop.set()


def main():
    parser = argparse.ArgumentParser(
        description=(
            "ISO-TP <-> PTY bridge (kernel socket). " "One bridge per process."
        ),
        epilog=(
            "Multiple bridges can run concurrently as separate processes. "
            "Each instance should use a unique --symlink path and a unique "
            "(interface, rx_id, tx_id) binding."
        ),
    )
    parser.add_argument("interface", help="CAN interface, e.g. can0")
    parser.add_argument("rx_id", help="hex CAN ID NuttX sends on,    e.g. 7D0")
    parser.add_argument("tx_id", help="hex CAN ID NuttX receives on, e.g. 7D1")
    parser.add_argument(
        "--symlink",
        default=PTY_SYMLINK,
        help=(
            f"symlink path for slave PTY (default: {PTY_SYMLINK}); "
            "must be unique when running multiple bridge instances"
        ),
    )
    parser.add_argument(
        "--canfd",
        action="store_true",
        help="use CAN FD frames (64-byte payload, sets CAN_ISOTP_LL_OPTS)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="print hex dumps of data flowing through the bridge",
    )
    parser.add_argument(
        "--fc-bs",
        type=int,
        default=None,
        help="host Flow Control block size (0..255); enables CAN_ISOTP_RECV_FC",
    )
    parser.add_argument(
        "--fc-stmin",
        type=int,
        default=None,
        help="host Flow Control STmin byte (0..255); enables CAN_ISOTP_RECV_FC",
    )
    parser.add_argument(
        "--fc-wftmax",
        type=int,
        default=0,
        help="host Flow Control wftmax byte (0..255), default: 0",
    )
    args = parser.parse_args()

    rx_id = int(args.rx_id, 16)
    tx_id = int(args.tx_id, 16)

    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_DGRAM, CAN_ISOTP)
        if args.canfd:
            # mtu=CANFD_MTU, tx_dl=64, tx_flags=0
            ll_opts = struct.pack("BBB", CANFD_MTU, 64, 0)
            sock.setsockopt(SOL_CAN_ISOTP, CAN_ISOTP_LL_OPTS, ll_opts)
        if args.fc_bs is not None or args.fc_stmin is not None:
            bs = args.fc_bs if args.fc_bs is not None else 0
            stmin = args.fc_stmin if args.fc_stmin is not None else 0
            wftmax = args.fc_wftmax
            if min(bs, stmin, wftmax) < 0 or max(bs, stmin, wftmax) > 255:
                sys.exit("fc bytes must be in range 0..255")
            recv_fc = struct.pack("BBB", bs, stmin, wftmax)
            sock.setsockopt(SOL_CAN_ISOTP, CAN_ISOTP_RECV_FC, recv_fc)
        sock.bind((args.interface, rx_id, tx_id))
    except OSError as e:
        sys.exit(f"Cannot open ISO-TP socket: {e}\n" f"Try: sudo modprobe can_isotp")

    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)

    # Disable canonical mode
    tty.setraw(slave_fd)

    # Create symlink for easy access
    try:
        os.unlink(args.symlink)
    except FileNotFoundError:
        pass
    os.symlink(slave_name, args.symlink)

    print("ISO-TP PTY bridge ready")
    print(f"  interface : {args.interface}")
    print(f"  rx CAN ID : 0x{rx_id:03X}  (NuttX transmits here)")
    print(f"  tx CAN ID : 0x{tx_id:03X}  (NuttX receives here)")
    print(f"  PTY slave : {slave_name}")
    print(f"  symlink   : {args.symlink}")
    print(f"  CAN FD    : {'on' if args.canfd else 'off'}")
    if args.fc_bs is not None or args.fc_stmin is not None:
        print(
            f"  FC opts   : bs={args.fc_bs if args.fc_bs is not None else 0} "
            f"stmin={args.fc_stmin if args.fc_stmin is not None else 0} "
            f"wftmax={args.fc_wftmax}"
        )
    else:
        print("  FC opts   : default kernel")
    print(f"  debug     : {'on' if args.debug else 'off'}")
    print("")
    print("Connect with:")
    print(f"  minicom -D {args.symlink}")
    print(f"  (gdb) target remote {args.symlink}")
    print("")
    print("Press Ctrl+C to stop.")
    sys.stdout.flush()

    stop = threading.Event()
    t1 = threading.Thread(
        target=isotp_to_pty, args=(sock, master_fd, stop, args.debug), daemon=True
    )
    t2 = threading.Thread(
        target=pty_to_isotp, args=(master_fd, sock, stop, args.debug), daemon=True
    )
    t1.start()
    t2.start()

    try:
        stop.wait()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        sock.close()
        os.close(master_fd)
        os.close(slave_fd)
        try:
            os.unlink(args.symlink)
        except FileNotFoundError:
            pass
        print("\nBridge stopped.")


if __name__ == "__main__":
    main()

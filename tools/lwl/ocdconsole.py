#!/usr/bin/env python3
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
#
# Usage
# =====
#
# No special python modules are needed, it should be possible to run the
# application simply as shown below;
#
# ------------------------------------------
# $ ./ocdconsole.py
# ==Link Activated
#
# nsh>
# nsh> help
# help usage:  help [-v] [<cmd>]
#
#  ?        echo     exit     hexdump  ls       mh       sleep    xd
#  cat      exec     help     kill     mb       mw       usleep
# nsh>
# ------------------------------------------
#
# This code is designed to be 'hardy' and will survive a shutdown and
# restart of the openocd process. When your target application
# changes then the location of the upword and downword may change,
# so they are re-searched for again. To speed up the start process
# consider putting those words at fixed locations (e.g. via the
# linker file) and referencing them directly.
#

import os
import socket
import time

if os.name == "nt":
    import msvcrt
else:
    import select
    import sys
    import termios
    import tty

LWL_ACTIVESHIFT = 31
LWL_DNSENSESHIFT = 30
LWL_UPSENSESHIFT = 29
LWL_OCTVALSHIFT = 27
LWL_PORTSHIFT = 24

LWL_PORTMASK = 7 << LWL_PORTSHIFT
LWL_SENSEMASK = 3 << LWL_UPSENSESHIFT
LWL_OCTVALMASK = 3 << LWL_OCTVALSHIFT

LWL_ACTIVE = 1 << LWL_ACTIVESHIFT
LWL_DNSENSEBIT = 1 << LWL_DNSENSESHIFT
LWL_UPSENSEBIT = 1 << LWL_UPSENSESHIFT

LWL_SIG = 0x7216A318

LWL_PORT_CONSOLE = 1

# Memory to scan through looking for signature
baseaddr = 0x20000000
length = 0x8000


def kbhit():
    """Returns True if a keypress is waiting to be read in stdin, False otherwise."""
    if os.name == "nt":
        return msvcrt.kbhit()
    else:
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        return dr != []


def dooutput(x):
    if x & 255 == 10:
        print("\r", flush=True)
    else:
        print(chr(x), end="", flush=True)


###############################################################################
# Code from here to *** below was taken from GPL'ed ocd_rpc_example.py and is
# available in its original form  at contrib/rpc_examples in the openocd tree.
#
# This code was approved for re-release under BSD (licence at the head of this
# file) by the original author (Andreas Ortmann, ortmann@finf.uni-hannover.de)
# via email to Dave Marples on 3rd June 2019.
# email ID: 15e1f0a0-9592-bd07-c996-697f44860877@finf.uni-hannover.de
###############################################################################


def strToHex(data):
    return map(strToHex, data) if isinstance(data, list) else int(data, 16)


class oocd:
    NL = "\x1A"

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.tclRpcIp = "127.0.0.1"
        self.tclRpcPort = 6666
        self.bufferSize = 4096

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __enter__(self):
        self.sock.connect((self.tclRpcIp, self.tclRpcPort))
        return self

    def __exit__(self, type, value, traceback):
        try:
            self.send("exit")
        finally:
            self.sock.close()

    def send(self, cmd):
        """Send a command string to TCL RPC. Return the result that was read."""
        data = (cmd + oocd.NL).encode("utf-8")
        if self.verbose:
            print("<- ", data)

        self.sock.send(data)
        return self._recv()

    def _recv(self):
        """Read from the stream until the NL was received."""
        data = bytes()
        while True:
            chunk = self.sock.recv(self.bufferSize)
            data += chunk
            if bytes(oocd.NL, encoding="utf-8") in chunk:
                break

        data = data.decode("utf-8").strip()
        data = data[:-1]  # strip trailing NL

        return data

    def readVariable(self, address):
        raw = self.send("%s 0x%x" % (self.mdwText, address)).split(": ")
        return None if (len(raw) < 2) else strToHex(raw[1])

    def writeVariable(self, address, value):
        assert value is not None
        self.send("mww 0x%x 0x%x" % (address, value))

    def testInterface(self):
        self.mdwText = "ocd_mdw"
        if self.readVariable(baseaddr) is not None:
            return
        self.mdwText = "mdw"
        if self.readVariable(baseaddr) is not None:
            return
        raise ConnectionRefusedError


# *** Incorporated code ends ######################################################

if __name__ == "__main__":

    def show(*args):
        print(*args, end="\n\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    while True:
        try:
            tty.setraw(fd)
            with oocd() as ocd:
                while True:
                    # Find the location for the communication variables
                    # =================================================
                    try:
                        ocd.testInterface()
                        downwordaddr = 0
                        while downwordaddr < length:
                            if ocd.readVariable(baseaddr + downwordaddr) == LWL_SIG:
                                break
                            downwordaddr = downwordaddr + 4

                        if downwordaddr >= length:
                            print("ERROR: Cannot find signature\r")
                            exit(1)

                        # We have the base address, so get the variables themselves
                        # =========================================================
                        downwordaddr = baseaddr + downwordaddr + 4
                        upwordaddr = downwordaddr + 4
                        downword = LWL_ACTIVE

                        # Now wake up the link...keep on trying if it goes down
                        # =====================================================
                        while True:
                            ocd.writeVariable(downwordaddr, downword)
                            upword = ocd.readVariable(upwordaddr)
                            if upword & LWL_ACTIVE != 0:
                                print("==Link Activated\r")
                                break
                    except (
                        BrokenPipeError,
                        ConnectionRefusedError,
                        ConnectionResetError,
                    ) as e:
                        raise e

                    # Now run the comms loop until something fails
                    # ============================================
                    try:
                        while True:
                            ocd.writeVariable(downwordaddr, downword)
                            upword = ocd.readVariable(upwordaddr)
                            if upword & LWL_ACTIVE == 0:
                                print("\r==Link Deactivated\r")
                                break
                            if kbhit():
                                charin = sys.stdin.read(1)
                                if ord(charin) == 3:
                                    sys.exit(0)
                                if downword & LWL_DNSENSEBIT:
                                    downword = downword & LWL_UPSENSEBIT
                                else:
                                    downword = (
                                        downword & LWL_UPSENSEBIT
                                    ) | LWL_DNSENSEBIT
                                downword |= (
                                    (LWL_PORT_CONSOLE << LWL_PORTSHIFT)
                                    | (1 << LWL_OCTVALSHIFT)
                                    | LWL_ACTIVE
                                    | ord(charin)
                                )

                            if (upword & LWL_UPSENSEBIT) != (downword & LWL_UPSENSEBIT):
                                incomingPort = (upword & LWL_PORTMASK) >> LWL_PORTSHIFT
                                if incomingPort == LWL_PORT_CONSOLE:
                                    incomingBytes = (
                                        upword & LWL_OCTVALMASK
                                    ) >> LWL_OCTVALSHIFT
                                    if incomingBytes >= 1:
                                        dooutput(upword & 255)
                                    if incomingBytes >= 2:
                                        dooutput((upword >> 8) & 255)
                                    if incomingBytes == 3:
                                        dooutput((upword >> 16) & 255)

                                if downword & LWL_UPSENSEBIT:
                                    downword = downword & ~LWL_UPSENSEBIT
                                else:
                                    downword = downword | LWL_UPSENSEBIT
                    except (
                        ConnectionResetError,
                        ConnectionResetError,
                        BrokenPipeError,
                    ) as e:
                        print("\r==Link Lost\r")
                        raise e

        except (BrokenPipeError, ConnectionRefusedError, ConnectionResetError):
            time.sleep(1)
            continue
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

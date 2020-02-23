#!/usr/bin/env python3
# tools/ocdconsole.py
#
#   Copyright (C) 2019 Dave Marples. All rights reserved.
#   Author: Dave Marples <dave@marples.net>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# Console over Lightweight Link
# =============================
#
# LWL is a Lightweight bidirectional communication between target and debug host
# without any need for additional hardware.
#
# It works with openOCD and other debuggers that are capable of reading and
# writing memory while the target is running...it should run with JLink
# for example, if you've got the SDK and modify this file accordingly.
#
# Principle of operation is simple; An 'upword' of 32 bits communicates
# from the target to the host, a 'downword' of the same size runs in the
# opposite direction. These two words can be in any memory that is
# read/write access for both the target and the debug host. A simple ping
# pong handshake protocol over these words allows up/down link communication.
# On the upside no additional integration is needed. On the downside it may be
# necessary to feed lwl with cycles to poll for changes in the downword,
# depending on the use case. For the case of a simple console, that's not
# needed.
#
# For convenience these communication locations are automatically discovered
# from the RAM by searching through it. Just define downwordaddr and
# upwordaddr if you want to work with fixed locations.
#
#
# Bit configuration
# -----------------
#
# Downword (Host to target);
#
# A D U VV XXX O2 O1 O0
#
# A   31    1 - Service Active (Set by host)
# D   30    1 - Downsense (Toggled when there is data)
# U   29    1 - Upsense ack (Toggled to acknowledge receipt of uplink data)
# VV  28-27 2 - Valid Octets (Number of octets valid in the message)
# XXX 26-24 3 - Port in use (Type of the message)
# O2  23-16 8 - Octet 2
# O1  15-08 8 - Octet 1
# O0  07-00 8 - Octet 0
#
# Upword (Target to Host);
#
# A   31    1 - Service Active (Set by device)
# D   30    1 - Downsense ack (Toggled to acknowledge receipt of downlink data)
# U   29    1 - Upsense (Toggled when there is data)
# VV  28-27 2 - Valid upword octets
# XXX 26-24 3 - Port in use (Type of the message)
# O2  23-16 8 - Octet 2
# O1  15-08 8 - Octet 1
# O0  07-00 8 - Octet 0
#
# Port 1 is used for Console. No other ports are currently defined.
#
# Use
# ===
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

LWL_ACTIVESHIFT = 31
LWL_DNSENSESHIFT = 30
LWL_UPSENSESHIFT = 29
LWL_OCTVALSHIFT = 27
LWL_PORTSHIFT = 24

LWL_PORTMASK  = (7<<LWL_PORTSHIFT)
LWL_SENSEMASK  = (3<<LWL_UPSENSESHIFT)
LWL_OCTVALMASK = (3<<LWL_OCTVALSHIFT)

LWL_ACTIVE = (1<<LWL_ACTIVESHIFT)
LWL_DNSENSEBIT = (1<<LWL_DNSENSESHIFT)
LWL_UPSENSEBIT = (1<<LWL_UPSENSESHIFT)

LWL_SIG = 0x7216A318

LWL_PORT_CONSOLE = 1

# Memory to scan through looking for signature
baseaddr = 0x20000000
length = 0x8000

import time
import socket
import os
if os.name == 'nt':
    import msvcrt
else:
    import sys, select, termios, tty

def kbhit():
    ''' Returns True if a keypress is waiting to be read in stdin, False otherwise.
    '''
    if os.name == 'nt':
        return msvcrt.kbhit()
    else:
        dr,dw,de = select.select([sys.stdin], [], [], 0)
        return dr != []

def dooutput(x):
    if (x&255==10):
        print("\r",flush=True)
    else:
        print(chr(x),end="",flush=True)

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
    NL = '\x1A'
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.tclRpcIp       = "127.0.0.1"
        self.tclRpcPort     = 6666
        self.bufferSize     = 4096

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
        data = data[:-1] # strip trailing NL

        return data

    def readVariable(self, address):
        raw = self.send("%s 0x%x" % (self.mdwText,address)).split(": ")
        return None if (len(raw) < 2) else strToHex(raw[1])

    def writeVariable(self, address, value):
        assert value is not None
        self.send("mww 0x%x 0x%x" % (address, value))

    def testInterface(self):
        self.mdwText="ocd_mdw"
        if (self.readVariable(baseaddr)!=None):
            return
        self.mdwText="mdw"
        if (self.readVariable(baseaddr)!=None):
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
                        downwordaddr=0
                        while (downwordaddr<length):
                            if (ocd.readVariable(baseaddr+downwordaddr)==LWL_SIG):
                                break;
                            downwordaddr=downwordaddr+4

                        if (downwordaddr>=length):
                            print("ERROR: Cannot find signature\r")
                            exit(1)

                        # We have the base address, so get the variables themselves
                        # =========================================================
                        downwordaddr=baseaddr+downwordaddr+4
                        upwordaddr=downwordaddr+4
                        downword=LWL_ACTIVE

                        # Now wake up the link...keep on trying if it goes down
                        # =====================================================
                        while True:
                            ocd.writeVariable(downwordaddr, downword)
                            upword = ocd.readVariable(upwordaddr)
                            if (upword&LWL_ACTIVE!=0):
                                print("==Link Activated\r")
                                break
                    except (BrokenPipeError, ConnectionRefusedError, ConnectionResetError) as e:
                        raise e

                    # Now run the comms loop until something fails
                    # ============================================
                    try:
                        while True:
                            ocd.writeVariable(downwordaddr, downword)
                            upword = ocd.readVariable(upwordaddr)
                            if (upword&LWL_ACTIVE==0):
                                print("\r==Link Deactivated\r")
                                break
                            if kbhit():
                                charin = sys.stdin.read(1)
                                if (ord(charin)==3):
                                    sys.exit(0)
                                if (downword&LWL_DNSENSEBIT):
                                    downword=(downword&LWL_UPSENSEBIT)
                                else:
                                    downword=(downword&LWL_UPSENSEBIT)|LWL_DNSENSEBIT
                                downword|=(LWL_PORT_CONSOLE<<LWL_PORTSHIFT)|(1<<LWL_OCTVALSHIFT)|LWL_ACTIVE|ord(charin)

                            if ((upword&LWL_UPSENSEBIT)!=(downword&LWL_UPSENSEBIT)):
                                incomingPort=(upword&LWL_PORTMASK)>>LWL_PORTSHIFT
                                if (incomingPort==LWL_PORT_CONSOLE):
                                    incomingBytes=(upword&LWL_OCTVALMASK)>>LWL_OCTVALSHIFT
                                    if (incomingBytes>=1): dooutput(upword&255);
                                    if (incomingBytes>=2): dooutput((upword>>8)&255);
                                    if (incomingBytes==3): dooutput((upword>>16)&255);

                                if (downword&LWL_UPSENSEBIT):
                                    downword = downword&~LWL_UPSENSEBIT
                                else:
                                    downword = downword|LWL_UPSENSEBIT
                    except (ConnectionResetError, ConnectionResetError, BrokenPipeError) as e:
                        print("\r==Link Lost\r")
                        raise e

        except (BrokenPipeError, ConnectionRefusedError, ConnectionResetError) as e:
            time.sleep(1)
            continue
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

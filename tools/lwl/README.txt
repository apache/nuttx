Console over Lightweight Link
=============================

LWL is a Lightweight bidirectional communication between target and debug host
without any need for additional hardware.

It works with openOCD and other debuggers that are capable of reading and
writing memory while the target is running...it should run with JLink
for example, if you've got the SDK and modify this file accordingly.

Principle of operation is simple; An 'upword' of 32 bits communicates
from the target to the host, a 'downword' of the same size runs in the
opposite direction. These two words can be in any memory that is
read/write access for both the target and the debug host. A simple ping
pong handshake protocol over these words allows up/down link communication.
On the upside no additional integration is needed. On the downside it may be
necessary to feed lwl with cycles to poll for changes in the downword,
depending on the use case. For the case of a simple console, that's not
needed.

For convenience these communication locations are automatically discovered
from the RAM by searching through it. Just define downwordaddr and
upwordaddr if you want to work with fixed locations.

Bit configuration
-----------------

Downword (Host to target);

A D U VV XXX O2 O1 O0

A   31    1 - Service Active (Set by host)
D   30    1 - Downsense (Toggled when there is data)
U   29    1 - Upsense ack (Toggled to acknowledge receipt of uplink data)
VV  28-27 2 - Valid Octets (Number of octets valid in the message)
XXX 26-24 3 - Port in use (Type of the message)
O2  23-16 8 - Octet 2
O1  15-08 8 - Octet 1
O0  07-00 8 - Octet 0

Upword (Target to Host);

A   31    1 - Service Active (Set by device)
D   30    1 - Downsense ack (Toggled to acknowledge receipt of downlink data)
U   29    1 - Upsense (Toggled when there is data)
VV  28-27 2 - Valid upword octets
XXX 26-24 3 - Port in use (Type of the message)
O2  23-16 8 - Octet 2
O1  15-08 8 - Octet 1
O0  07-00 8 - Octet 0

Port 1 is used for Console. No other ports are currently defined.

Usage
=====

No special python modules are needed, it should be possible to run the
application simply as shown below:

In the first terminal execute the openocd command to connect to the board.
Assuming that you already flashed to firmware (nuttx.bin) with the LWL
console support. For stm32f4discovery board I use this command:

------------------------------------------
$ sudo openocd -f board/stm32f4discovery.cfg
Open On-Chip Debugger  v0.10.0-esp32-20200526-6-g4c41a632 (2020-06-23-10:12)
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
srst_only separate srst_nogate srst_open_drain connect_deassert_srst

Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : clock speed 2000 kHz
Info : STLINK V2J17S0 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.216252
Info : stm32f4x.cpu: hardware has 6 breakpoints, 4 watchpoints
Info : Listening on port 3333 for gdb connections
Info : accepting 'tcl' connection on tcp/6666
invalid command name "ocd_mdw"
0x20000000: 000000ff

0x20000000: 000000ff

0x20000004: 7216a318

0x2000000c: 994b5b1b

0x2000000c: 994b5b1b

0x2000000c: 994b5b1b
...

The "0x2000000c:..." will repeat all the time.

Now in another terminal execute:

------------------------------------------
$ ./ocdconsole.py
==Link Activated

nsh>
nsh> help
help usage:  help [-v] [<cmd>]

 ?        echo     exit     hexdump  ls       mh       sleep    xd
 cat      exec     help     kill     mb       mw       usleep
nsh>
------------------------------------------

This code is designed to be 'hardy' and will survive a shutdown and
restart of the openocd process. When your target application
changes then the location of the upword and downword may change,
so they are re-searched for again. To speed up the start process
consider putting those words at fixed locations (e.g. via the
linker file) and referencing them directly.

Future work/Improvements
========================

Currently the lwl driver on NuttX side is doing polling, but for
better performance it could use interrupts to detect when the memory
position was modified to read the data.

It also will avoid using busy waiting inside the driver, look at
nuttx/arch/arm/src/common/arm_lwl_console.c for more information.

===================================
Console over Lightweight Link (LWL)
===================================

The Lightweight Link (LWL) gives a target a NuttShell console, or any
other console traffic, through the debug probe that is already connected
to it.  It needs no UART, no USB and no extra pin: the target and the host
exchange data through two words of target RAM that the debugger reads and
writes while the CPU is running.

It is useful when the board has no spare serial port, during the bring-up
of a new chip, or when the only connection to the board is the SWD/JTAG
probe.  Unlike semihosting it never halts the CPU, so the target keeps its
real-time behaviour, and unlike SWO it works in both directions.

Any debugger that can access memory while the target runs can be used.  The
host side terminal shipped with NuttX, ``tools/lwl/ocdconsole.py``, uses
OpenOCD.

.. contents::
   :local:
   :depth: 2

Quick start
===========

The ``weact-stm32h743:lwl`` configuration is a ready to use example.  Build
and flash it with the ST-Link connected to the SWD pins of the board:

.. code-block:: console

   $ ./tools/configure.sh weact-stm32h743:lwl
   $ make
   $ openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
       -c "program nuttx.bin 0x08000000 verify reset exit"

Leave OpenOCD running:

.. code-block:: console

   $ openocd -f interface/stlink.cfg -f target/stm32h7x.cfg

and start the terminal in another shell:

.. code-block:: console

   $ ./tools/lwl/ocdconsole.py --elf nuttx
   ==Link Activated

   NuttShell (NSH)
   nsh> ps
     TID   PID  PPID PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK            STACK    CPU COMMAND
       0     0     0   0 FIFO     Kthread   - Ready              0000000000000000 0001000 100.0% Idle_Task
       2     2     0 100 RR       Task      - Running            0000000000000000 0003024   0.0% nsh_main
   nsh>

Press ``Ctrl-]`` to leave the terminal.  All the other keys, ``Ctrl-C``
included, are sent to the target.

Enabling LWL on a board
=======================

LWL is a console driver, so it replaces the serial console:

* Disable the serial console of the board (e.g. ``CONFIG_USART1_SERIAL_CONSOLE``).
* Enable ``CONFIG_LWL_CONSOLE``.  It requires an architecture that selects
  ``ARCH_HAVE_RDWR_MEM_CPU_RUN`` (ARM, ARM64 and RISC-V today).
* If the debugger can make an interrupt pending by writing a register of
  the interrupt controller, enable ``CONFIG_LWL_CONSOLE_DOORBELL`` and pick
  an interrupt for it, see `Doorbell interrupt`_.

The driver registers itself as ``/dev/console``.  It is a lower half of the
generic serial driver, so the console behaves like any other NuttX TTY:
buffered I/O, ``termios`` processing (``ECHO``, ``ICRNL``, ``ONLCR``, ...),
``poll()``/``select()`` and the TTY signals all work.  ``up_putc()`` is
provided as well, so ``syslog`` and crash dumps also go to the host.

Configuration options
---------------------

===================================== ========================================
Option                                Description
===================================== ========================================
``CONFIG_LWL_CONSOLE``                Enable the LWL console.
``CONFIG_LWL_CONSOLE_RXBUFSIZE``      Size of the buffer holding the input
                                      until the application reads it (default
                                      64).
``CONFIG_LWL_CONSOLE_TXBUFSIZE``      Size of the buffer holding the output
                                      until the host collects it (default
                                      256).
``CONFIG_LWL_CONSOLE_RINGSIZE``       Size of the ring buffer from which the
                                      host collects the output (default 256, a
                                      multiple of 4).  ``0`` limits the output
                                      to the slow word protocol.
``CONFIG_LWL_CONSOLE_DOORBELL``       Let the host raise an interrupt when it
                                      updates the link.
``CONFIG_LWL_CONSOLE_DOORBELL_IRQ``   The NuttX IRQ number used as doorbell.
``CONFIG_LWL_CONSOLE_DOORBELL_ADDR``  Address of the register the host writes
                                      to make that IRQ pending.
``CONFIG_LWL_CONSOLE_DOORBELL_VALUE`` Value the host writes to that register.
``CONFIG_LWL_CONSOLE_POLL_INTERVAL``  Without the doorbell, interval in
                                      microseconds at which the link is
                                      polled (default 10000).
===================================== ========================================

Doorbell interrupt
------------------

Most interrupt controllers have a register that makes an interrupt
pending when it is written, exactly as if the peripheral had raised it, and
the debugger can write it while the CPU runs.  LWL uses this as a doorbell:
after every update of the link the host rings it, the target handles the
new data in the interrupt handler and goes back to sleep.  There is no
polling and no busy waiting at all, the console costs no CPU time while it
is idle.

The board configuration gives three values:

* ``CONFIG_LWL_CONSOLE_DOORBELL_IRQ``: the NuttX IRQ number of an
  interrupt that is implemented by the chip but not used by any driver of
  the configuration.  A good candidate is the interrupt of a peripheral
  that is not enabled.
* ``CONFIG_LWL_CONSOLE_DOORBELL_ADDR``: the register the host writes to make
  that interrupt pending.
* ``CONFIG_LWL_CONSOLE_DOORBELL_VALUE``: the value the host writes there.

The interrupt handler writes 0 to the register before it services the
link.  This clears level-type software interrupts and has no effect on
set-pending registers, so the register must be one of the two.

The firmware publishes the address and the value in the control block, so
the host does not need to know the chip.  With the doorbell the link is
not polled at all, so the host must ring it: ``ocdconsole.py`` does, older
terminals that don't need a firmware built without the doorbell.

Interrupt controllers that fit:

.. list-table::
   :header-rows: 1

   * - Controller
     - ``DOORBELL_ADDR``
     - ``DOORBELL_VALUE``
   * - ARMv6/7/8-M NVIC
     - Interrupt Set-Pending Register,
       ``0xe000e200 + 4 * ((IRQ - 16) / 32)``
     - ``1 << ((IRQ - 16) % 32)``
   * - ARM GIC
     - ``GICD_ISPENDR<n>`` of a shared peripheral interrupt,
       ``n = INTID / 32``
     - ``1 << (INTID % 32)``
   * - RISC-V CLINT
     - ``msip`` register of the hart (machine software interrupt)
     - ``1``

Only the NVIC has been tested.  For instance ``weact-stm32h743:lwl`` uses
IRQ 160, ``STM32_IRQ_CRS`` (the Clock Recovery System, which NuttX does not
use on STM32H7): external interrupt 144, so the register is ``0xe000e210``
and the value ``0x10000``.

On RISC-V the debugger can only access memory while the hart runs if the
debug module implements System Bus Access, which is optional, and the CLINT
software interrupt must not be used for something else (e.g. SMP
inter-processor interrupts).

Polling
-------

Without the doorbell (no suitable interrupt, a host tool that does not
ring it, a debugger that cannot write the interrupt controller) the driver
polls the link with a watchdog timer every
``CONFIG_LWL_CONSOLE_POLL_INTERVAL`` microseconds.
Each poll refills the output ring buffer, or moves one word with hosts that
don't use it, so the throughput depends on the interval: about 7 KB/s with
the ring and 300 bytes/s without it for the default 10 ms.

Polling runs while the console is open, whether a host is connected or
not, so the CPU wakes up at this rate.  Polling works with any host.

How it works
============

The control block
-----------------

The target and the host share a control block in target RAM, the
``g_lwl`` variable of ``drivers/misc/lwl_console.c``.  The host finds it from
the ELF symbol table or by searching the signature in RAM.

====== ============ =========================================================
Offset Field        Description
====== ============ =========================================================
0x00   ``sig``      ``0x7216a318``, lets the host find the block.
0x04   ``downword`` Host to target data, written by the host.
0x08   ``upword``   Target to host data, written by the target (word
                    protocol only).
0x0c   ``extsig``   ``0x4c574c32`` ("LWL2") if the fields below are valid.
                    Firmware older than this version does not have this
                    field nor the next ones.
0x10   ``dbaddr``   Address the host writes to ring the doorbell, 0 if there
                    is no doorbell.
0x14   ``dbval``    Value the host writes to ``dbaddr``.
0x18   ``ringsize`` Size of the output ring buffer, 0 if there is none.
0x40   ``hostcaps`` Written by the host before it opens the link.  Bit 0:
                    the host reads the ring buffer.
0x44   ``uptail``   Written by the host: offset in ``ring`` of the next octet
                    it will read.
0x80   ``status``   Written by the target: its ``A`` and ``D`` bits (ring
                    protocol only).
0x84   ``uphead``   Written by the target: offset in ``ring`` of the next
                    octet it will write.
0xc0   ``ring``     Output ring buffer of ``ringsize`` octets, written by
                    the target.
====== ============ =========================================================

The debugger accesses the memory behind the data cache of the CPU, so on
targets with a data cache the driver cleans what it writes and invalidates
what the host writes.  The block is aligned to 64 bytes and the fields
written by the host (0x00-0x7f) and by the target (0x80 and above) are in
different 64 byte groups: cleaning a cache line never writes a stale copy
of a host field back to RAM.  The legacy ``upword`` is the exception, so
with a write-back data cache use a host that reads the ring buffer.

Word format
-----------

The downword, the upword and the status word share the same format::

   31  30  29  28-27  26-24  23-16  15-8  7-0
   A   D   U   VV     XXX    O2     O1    O0

===== ===== ================================ ================================
Bits  Name  Downword (host to target)        Upword/status (target to host)
===== ===== ================================ ================================
31    A     Link active, set by the host     Link active, set by the target
30    D     Downsense: toggled when the      Downsense ack: copied from the
            word carries new data            downword once it is consumed
29    U     Upsense ack: copied from the     Upsense: toggled when the word
            upword once it is consumed       carries new data (upword only)
28-27 VV    Number of valid octets, 0 to 3   Number of valid octets, 0 to 3
26-24 XXX   Port, 1 is the console           Port, 1 is the console
23-0  O2-O0 Data octets, O0 in bits 7-0      Data octets, O0 in bits 7-0
===== ===== ================================ ================================

Input: the downword
-------------------

The input always goes through the downword, with a ping-pong handshake on
the ``D`` bit.  The host may write a new downword only when the ``D`` bit
reported by the target equals the ``D`` bit of its last downword, i.e. the
target consumed the previous one.  It then writes up to three octets, their
number and the port with the ``D`` bit toggled, and rings the doorbell.  The
target copies the octets into its RX buffer and acknowledges by copying
``D`` into the status word (or the upword with the word protocol).

Output: the ring buffer
-----------------------

Hosts that set bit 0 of ``hostcaps`` collect the output from the ring
buffer.  The target copies the output to ``ring`` at ``uphead`` and
advances ``uphead``, as long as it does not reach ``uptail`` (one octet
always stays free, ``uphead == uptail`` means empty).  The host reads
``status``, ``uphead`` and ``ring`` in a single memory access, outputs the
octets from ``uptail`` to ``uphead``, writes the new ``uptail`` and rings the
doorbell, so the target refills the ring.  A whole ring of output costs one
round trip instead of one per three octets.

Output: the word protocol
-------------------------

Older hosts collect the output from the upword, with a ping-pong handshake
on the ``U`` bit.  The target writes a new upword with up to three octets
and the ``U`` bit toggled only when the ``U`` bit of the downword equals its
own, i.e. the host collected the previous one.  The host outputs the
octets, acknowledges by copying ``U`` into the downword and rings the
doorbell.  This is also what the target uses when ``ringsize`` is 0.

Opening and closing the link
----------------------------

The host opens the link by setting ``A`` in the downword, after it wrote
``hostcaps`` and ``uptail``.  When the target sees it, it selects the ring
or the word protocol from ``hostcaps``, sets ``A`` in the status word (or
the upword), adopts the host's ``D`` and ``U`` bits and sets ``uphead`` to
``uptail``: there is neither pending input nor stale output whatever the
state of a previous session.

The host closes the link by writing 0 to the downword.  The target answers
with a status word and an upword of 0.
``ocdconsole.py`` always closes the link before opening it, which
resynchronises the target even if the previous host was killed without
closing the link.

Buffering and flow control
--------------------------

* The application writes into the TX buffer and returns; the driver moves
  the data to the ring buffer (or the upword) as the host collects it.
  When the buffer is full the writer blocks until the host collects data,
  as with a UART.
* Received octets are stored in the RX buffer.  When it is full the driver
  leaves the downword unacknowledged, which holds off the host until the
  application reads: no input is lost, even when pasting text into a busy
  shell.  The host re-rings the doorbell while its data is pending.
* While no host is connected, writers never block: the TX buffer keeps the
  most recent output, dropping the oldest data, and it is shown when a host
  connects.  For instance the boot banner appears as soon as the terminal
  is started.
* The driver remembers the line being output (up to 80 octets since the
  last newline, typically the ``nsh>`` prompt and what was typed after it)
  and sends it again to every new host, so the prompt is there without
  pressing Enter even if a previous host already collected it.
* ``up_putc()`` (syslog, crash dumps) writes directly to the link and
  busy-waits for room.  If an open link is not served for a long time
  (the host was killed without closing it) it starts dropping characters
  instead of hanging the system.

The host terminal
=================

``tools/lwl/ocdconsole.py`` connects to the TCL RPC server of OpenOCD
(port 6666) and needs nothing but the Python 3 standard library.  It uses
``read_memory`` when OpenOCD provides it (0.12 and later) and falls back to
``mdw`` otherwise.

.. code-block:: console

   $ ./tools/lwl/ocdconsole.py --help
   usage: ocdconsole.py [-h] [--host HOST] [--port PORT]
                        [--elf ELF | --addr ADDR | --scan BASE:LEN] [-v]

====================== =====================================================
Option                 Description
====================== =====================================================
``--elf nuttx``        Take the address of the control block from the ELF
                       file of the running firmware.  Fastest and most
                       reliable.
``--addr 0x24000040``  Use the given control block address.
``--scan BASE:LEN``    Search the signature in these RAM ranges (can be
                       repeated).  By default the first 64 KiB at
                       ``0x20000000`` and at ``0x24000000`` are searched.
``--host``, ``--port`` OpenOCD TCL RPC server, ``127.0.0.1:6666`` by default.
``-v``                 Print where the control block is, the protocol used
                       and whether the doorbell is used.
====================== =====================================================

The terminal survives restarts of OpenOCD and of the target: it reconnects
and reopens the link.

Compatibility
-------------

The word protocol did not change, so old and new versions interoperate:

* The new terminal with old firmware: the missing ``extsig`` tells the
  terminal to use the word protocol, send one octet per word, translate the
  line endings, skip the doorbell and leave the link open on exit (old
  firmware ends the shell session when the link closes).
* An old terminal with new firmware built without the doorbell: the
  firmware uses the word protocol and polls the link.  The old terminal
  does not wait for the acknowledge before sending the next key, so fast
  typing or pasting may lose characters.  Old terminals don't ring the
  doorbell, so they don't work with firmware built with it.

Writing a host for another debugger
-----------------------------------

Any tool that can read and write target memory while the target runs can
be a host.  The loop of ``ocdconsole.py`` is a good reference:

#. Find the control block and read ``extsig``, ``dbaddr``, ``dbval`` and
   ``ringsize``.
#. Write 0 to the downword, ring the doorbell and wait until both the
   upword and the status word read 0.
#. Write 1 to ``hostcaps`` and 0 to ``uptail``, write ``A`` to the
   downword and ring until the status word has ``A``.
#. Read ``status``, ``uphead`` and ``ring`` (from offset 0x80, in one
   access).  If ``uphead`` differs from ``uptail``, output the octets in
   between and write ``uphead`` to ``uptail``.
#. If the ``D`` bits of the status word and of the downword are equal and
   there is input, put up to three octets in the downword and toggle its
   ``D`` bit.
#. If something was written, ring the doorbell.  Go to 4.
#. On exit write 0 to the downword and to ``hostcaps`` and ring the
   doorbell.

Performance
===========

Measured on the WeAct STM32H743 (480 MHz Cortex-M7) with an ST-Link/V2 and
OpenOCD 0.12, writing to the console with ``dd``:

======================================= ================= ==================
                                        Previous driver   Current driver
======================================= ================= ==================
CPU used by ``nsh`` waiting for input   100 %             0 %
Output throughput                       ~1.8 KB/s         ~28 KB/s
Input buffering                         none              RX buffer, host
                                                          held off when full
Output while no host is connected       writer hangs      last 256 bytes kept
Prompt shown when a host connects       no                yes
Data cache enabled (D-cache on STM32H7) does not work     works
======================================= ================= ==================

For reference a UART at 115200 baud moves about 11 KB/s.  The throughput is
bounded by the round trip time between the terminal, OpenOCD and the probe,
so a larger ring helps a little (about 39 KB/s with
``CONFIG_LWL_CONSOLE_RINGSIZE=1024``).  With the word protocol every three
octets cost a round trip: about 1.3 KB/s with the doorbell and 300 bytes/s
when polled.

Troubleshooting
===============

``==Waiting for the target`` repeats forever
   The firmware does not answer.  Check that it runs (it may have crashed
   before opening the console), that the control block address is right
   (use ``--elf`` with the ELF file that is actually flashed) and, with the
   doorbell, that the terminal rings it (``-v`` shows ``doorbell at ...``).

``Cannot find the LWL signature``
   The control block is outside of the default search ranges.  Use
   ``--elf`` or ``--scan`` with the RAM of your chip.

``LWL signature found at 0x20000af8, 0x24000040, ...``
   RAM keeps its content across resets, so a previous firmware may have
   left a stale copy of the control block.  The terminal does not guess,
   since writing to the wrong place could corrupt the target: use ``--elf``
   or ``--addr``.

Nothing happens on the target when the doorbell is rung
   The doorbell IRQ is not implemented by the chip, or another driver
   attached the same interrupt.  Pick another one.

===================
Raspberry Pi rp2350
===================

.. tags:: chip:rp2350

The rp2350 is a dual core chip produced by Raspberry Pi that
is based on ARM Cortex-M33 or the Hazard3 RISC-V.

ARM Cortex-M33 and Hazard3 RISC-V cores are supported.

This is ARM Cortex-M33 version of the chip configuration.

This port is experimental and still a work in progress. Use with caution.

Peripheral Support
==================

Most drivers were copied from the rp2040 port with some modifications.

The following list indicates peripherals currently supported in NuttX:

==============   ============  =====
Peripheral       Status        Notes
==============   ============  =====
GPIO             Working       See Supported Boards documentation for available pins.
UART             Working       GPIO 0 (UART0 TX) and GPIO 1 (UART0 RX) are used for the console.
I2C              Working
SPI Master       Working
SPI Slave        Untested
DMAC             Working
PWM              Working
USB              Experimental  usbnsh configuration is somewhat working with some data corruption
PIO              Working
IRQs             Working
WDOG             Working
DMA              Working
FPU/DSP          Experimental  The ostest passing the fpu test, dsp needs some real world testing
Clock Output     Untested
Flash ROM Boot   Working       Does not require boot2 from pico-sdk
                               If picotool is available a nuttx.uf2 file will be created
SRAM Boot        Working       Requires external SWD debugger
PSRAM            Working       Three modes of heap allocation described below
TRNG             Working       Hardware RNG at /dev/random and /dev/urandom
Flash MTD        Working       Unused flash tail as an MTD device, answers BIOC_XIPBASE
Timer            Working       /dev/timerN on TIMER0/TIMER1, 1 us resolution
Tickless         Working       Optional, RP2350 TIMER via the alarm/oneshot
RTC              Working       POWMAN always-on timer, backs the system clock
==============   ============  =====

Installation
============

1. Download and build picotool, make it available in the PATH::

    git clone https://github.com/raspberrypi/picotool.git picotool
    cd picotool
    mkdir build
    cd build
    cmake ..
    make
    cp picotool ~/local/bin # somewhere in your PATH

2. Download NuttX and the companion applications.  These must both be
   contained in the same directory::

    git clone https://github.com/apache/nuttx.git nuttx
    git clone https://github.com/apache/nuttx-apps.git apps

Building NuttX
==============

1. Change to NuttX directory::

    cd nuttx

2. Select a configuration. The available configurations
   can be listed with the command::

    ./tools/configure.sh -L

3. Load the selected configuration.::

    make distclean
    ./tools/configure.sh raspberrypi-pico-2:usbnsh

4. Modify the configuration as needed (optional)::

    make menuconfig

5. Build NuttX::

    make

Flash boot
==========

By default, the system is built to build and run from the flash
using XIP. By using the default `BOOT_RUNFROMFLASH` configuration,
the full image is run from the flash making most of the internal
SRAM available for the OS and applications, however the execution
is slower. The cache can speed up, but you might want set your
time critical functions to be placed in the SRAM (copied from
the flash on startup).

It is also possible to execute from SRAM, which reduces the
available SRAM to the OS and applications, however it is very
useful when debugging as erasing and rewriting the flash on
every build is tedious and slow. This option is enabled with
`BOOT_RUNFROMISRAM` and requires `openocd`` and/or `gdb`.

There is a third option which is to write the firmware on the
flash and it gets copied to the SRAM. This is enabled with
`CONFIG_BOOT_COPYTORAM` and might be useful for time critical
applications, on the expense of reduced usable internal SRAM
memory.

PSRAM
=====

Some boards like the `pimoroni-pico-2-plus` have a PSRAM
which greatly increases the available memory for applications.
The PSRAM is very slow compared to the internal SRAM,
so depending on the application, different configuration might
be necessary.

To use the PSRAM, enable the `RP23XX_PSRAM` and select the GPIO
pin used as CS1n with `RP23XX_PSRAM_CS1_GPIO`. See the RP2350
datasheet for more information.

The port offers three options for configuring the heaps to use
the external PSRAM, described below. More custom configurations
can be used with custom board initialization functions.

Use PSRAM and SRAM as a single main heap
----------------------------------------

This option is selected with `RP23XX_PSRAM_HEAP_SINGLE` and
requires `MM_REGIONS > 1`, as the PSRAM memory region will
be added to the heap. It is also necessary to disable
`MM_KERNEL_HEAP`, as there will only be a single heap.

This is the simplest configuration because it will unify the
memories into a single main heap. This way you can see the `free`
command output the total amount of usable RAM in the heap.

However, there are some unpredictable performance issues because
there is no control of where the memory is allocated when issuing
`malloc(3)` and `free(3)`. For this reason, you might want to
consider the other options.

Use PSRAM as user heap, SRAM as kernel heap
-------------------------------------------

This option is selected with `RP23XX_PSRAM_HEAP_USER` and
requires `MM_KERNEL_HEAP` to be set.

The external PSRAM is allocated to the default heap, while
the internal SRAM will be used for the kernel heap. This
configuration is useful because it allows drivers to
use the SRAM and behave much faster than if they used
memory on the PSRAM. While user applications can take
the bull benefit of the larger slower heap on the PSRAM.

Use PSRAM as a separate heap
----------------------------

This option is selected with `RP23XX_PSRAM_HEAP_SEPARATE` and
requires `ARCH_HAVE_EXTRA_HEAPS` to be set.

The internal SRAM is used as the main heap for kernel and
applications, as if there was no PSRAM configured. The
external PSRAM is configured as a separate user heap called
`psram` and can be used through the global variable
`g_psramheap` after including `rp23xx_heaps.h`

Timer
=====

The RP2350 has two system timer blocks, TIMER0 and TIMER1, each a
free-running 64-bit counter incremented once per microsecond.  They are
independent of the ARM SysTick that drives the OS tick, so they are
available for application timers.

Enable the driver with `RP23XX_TIMER` (which selects `TIMER`), then turn on
each block you want: `RP23XX_TIMER0` registers `/dev/timer0` (TIMER0) and
`RP23XX_TIMER1` registers `/dev/timer1` (TIMER1).  Each device implements the
standard NuttX timer lower-half: single-shot or periodic timeouts with 1 us
resolution and a maximum interval of 2^32 - 1 us (about 71.5 minutes), driven
by ALARM0 of the block.

A block claimed by the tickless-OS oneshot
(`RP23XX_SYSTIMER_TICKLESS`) is removed from the choices above, so a
`/dev/timer` device and the tickless OS time source never collide on the same
block -- you can run both at once (e.g. tickless on TIMER0, `/dev/timer1` on
TIMER1).  With tickless disabled, both `/dev/timer0` and
`/dev/timer1` are available.  The `examples/timer` application can exercise a
device; point `CONFIG_EXAMPLES_TIMER_DEVNAME` at the block you enabled.

Tickless OS
===========

By default the OS tick is a periodic ARM SysTick interrupt.  The RP2350 can
instead run tickless, driving the scheduler from a hardware alarm so the CPU
is only interrupted when a timer actually expires.

Enable `RP23XX_SYSTIMER_TICKLESS` together with `SCHED_TICKLESS` and
`SCHED_TICKLESS_ALARM`.  Choose the timer block with the "Tickless timer
block" option (`RP23XX_SYSTIMER_TICKLESS_TIMER0`, the default, or
`RP23XX_SYSTIMER_TICKLESS_TIMER1`).  The system time is then taken from that
block -- a free-running 64-bit microsecond counter -- and its ALARM0 provides
the next-event interrupt through the alarm/oneshot lower-half
(`rp23xx_oneshot.c`).  Because the 64-bit counter is the monotonic time base,
timekeeping is exact to 1 us, and a single alarm can schedule up to ~71
minutes ahead, so long idle periods need no wake-ups.  This is mutually
exclusive with `RP23XX_SYSTIMER_SYSTICK`.

The block chosen here is claimed exclusively by the scheduler and is removed
from the `/dev/timer` driver's choices (see the Timer section), so the tickless
clock and a `/dev/timer` device can run at the same time on different blocks.

RTC
===

The RP2350 has no dedicated RTC block -- the one on the RP2040 was removed --
so NuttX drives the real-time clock from the POWMAN *always-on timer*
instead. This is a 64-bit millisecond counter that can be clocked from the
low-power oscillator (LPOSC, nominally 32.768 kHz), so it keeps counting
across warm resets and through the low-power states managed by POWMAN.

Enable it with `RP23XX_RTC` (which selects `RTC`). The driver
(`rp23xx_rtc.c`) implements the lightweight `up_rtc_initialize()`,
`up_rtc_time()` and `up_rtc_settime()` interface that backs the NuttX system
clock at one-second resolution. `up_rtc_initialize()` sources the timer from
the low-power oscillator at a 1 kHz tick and starts it, but preserves the
current value if the bootrom (or a previous boot) already left it running, so
the wall-clock time survives a warm reset.

Two POWMAN details the driver has to honour:

* Every POWMAN register write must carry the `0x5afe` password in its top 16
  bits or it is silently ignored, so only the low 16 bits of each register
  hold data. The 64-bit time is therefore written and read through four
  16-bit registers, and bit operations use the atomic `SET`/`CLR` register
  aliases.
* The counter must be stopped (`TIMER.RUN` cleared) while a new value is
  loaded and then restarted, otherwise the write is not latched cleanly.

The implementation follows the POWMAN "Always-on Timer" description in the
Power chapter of the `RP2350 datasheet
<https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf>`_ and the
reference `pico-sdk hardware_powman
<https://github.com/raspberrypi/pico-sdk/tree/master/src/rp2_common/hardware_powman>`_
library (`powman_timer_start_lposc()`, `powman_timer_set_ms()` and
`powman_timer_get_ms()`), which the pico-sdk in turn exposes through its
`pico_aon_timer
<https://github.com/raspberrypi/pico-sdk/tree/master/src/rp2_common/pico_aon_timer>`_
wrapper.

Programming
============

Programming using BOOTSEL
-------------------------

Connect  board to USB port while holding BOOTSEL.
The board will be detected as USB Mass Storage Device.
Then copy "nuttx.uf2" into the device.
(Same manner as the standard Pico SDK applications installation.)

Programming with picotool
-------------------------

You can use picotool to load the elf (or the uf2)::

    picotool load nuttx -t elf

Programming using SWD debugger
------------------------------

Most boards provide a serial (SWD) debug port.
The "nuttx" ELF file can be uploaded with an appropriate SDB programmer
module and companion software (openocd and gdb)

Running NuttX
=============

Most builds provide access to the console via UART0.  To access this
GPIO 0 and 1 pins must be connected to the device such as USB-serial converter.

The `usbnsh` configuration provides the console access by USB CDC/ACM serial
device.  The console is available by using a terminal software on the USB host.

TRNG
====

The rp2350 has a hardware true random number generator (TRNG).  Enabling
``RP23XX_RNG`` builds the driver and selects ``ARCH_HAVE_RNG``, which in turn
makes ``DEV_RANDOM`` available.

With ``DEV_RANDOM`` enabled the driver registers ``/dev/random``.  Enabling
``DEV_URANDOM`` additionally registers ``/dev/urandom``; when a hardware RNG is
present the architecture source (``DEV_URANDOM_ARCH``) is selected by default,
so ``/dev/urandom`` is served from the same TRNG rather than a software PRNG.

Each read collects entropy from the TRNG's 192-bit entropy holding register
(EHR): the source is enabled, the driver waits for ``TRNG_VALID``, reads the
six 32-bit EHR words, and repeats until the request is satisfied.  For example::

    nsh> dd if=/dev/random of=/dev/console bs=16 count=1

Flash MTD
=========

The rp2350 executes in place from its external QSPI flash, and a NuttX image
normally leaves most of that flash unused.  Enabling ``RP23XX_FLASH_MTD``
exposes the unused region as an MTD device, registered by the common board
bringup as ``/dev/rpflash``.

The region is described by ``RP23XX_FLASH_MTD_OFFSET`` (byte offset from
``0x10000000``) and ``RP23XX_FLASH_MTD_SIZE``, both of which must be multiples
of the 4096 byte erase sector.  The driver refuses to initialize if the region
would overlap the NuttX image (it checks ``__flash_binary_end``), so a bad
offset fails at boot instead of corrupting the running firmware.

Erase and program go through the bootrom flash routines.  Because those
operations stall instruction fetch from the same flash, the driver runs them
from SRAM with interrupts disabled and, on SMP builds, the other core parked;
expect interrupt latency to suffer for the duration of a write.  Afterwards the
QSPI interface is put back into execute-in-place mode -- by default restoring
the fast read mode the bootrom set up at boot, or, with
``RP23XX_FLASH_MTD_SAFE_XIP``, always through the bootrom
``flash_enter_cmd_xip`` routine, which is slower to execute from but depends
only on the documented bootrom entry point.

The driver answers the ``BIOC_XIPBASE`` ioctl with the memory-mapped address of
the region, so a filesystem that supports execute in place can hand out real
flash pointers instead of copying into RAM.

Any MTD-based filesystem can be layered on the device, for example::

    nsh> mksmartfs /dev/rpflash
    nsh> mount -t smartfs /dev/rpflash /mnt

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

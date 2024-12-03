==================
RaspberryPi rp2350
==================

The rp2350 is a dual core chip produced by the RaspberryPi Foundation that
is based on ARM Cortex-M33 or the Hazard3 RISC-V.

For now, only the ARM Cortex-M33 is supported.

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
I2C              Untested
SPI Master       Working
SPI Slave        Untested
DMAC             Untested
PWM              Untested
USB              Experimental  usbnsh configuration is somewhat working with some data corruption
PIO              Working
IRQs             Working
DMA              Untested
Clock Output     Untested
Flash ROM Boot   Working       Does not require boot2 from pico-sdk
                               If picotool is available a nuttx.uf2 file will be created
SRAM Boot        Working       Requires external SWD debugger
PSRAM            Working       Three modes of heap allocation described below
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

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

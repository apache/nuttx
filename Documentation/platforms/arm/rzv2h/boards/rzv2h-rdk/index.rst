=========
RZV2H-RDK
=========

.. tags:: arch:arm, arch:armv7-r, arch:cortex-r8, chip:rzv2h, vendor:renesas

The RZV2H-RDK is a development board for the Renesas RZ/V2H (R9A09G057)
MPU. NuttX is ported to run on one of the two Cortex-R8 real-time cores
(``CR8_0``).

Board Overview
==============

.. figure:: rzv2h-rdk-overview.png
   :align: center
   :width: 500px

See the `RZ/V2H product page
<https://www.renesas.com/en/design-resources/boards-kits/ws125-v2hrdkrefz>`_
for information about the RZ/V2H MPU.

Board Set-up
============

.. figure:: rzv2h-rdk-board-set-up.png
   :align: center
   :width: 500px

The following steps set up the board to debug/run NuttX over SWD with a
J-Link debugger:

1. Configure the SWD interface to use the J-Link debugger via the DIP
   switches (highlighted in red in the picture above):

   ====  ===
   DSW1  Set
   ====  ===
   1-1   ON
   1-2   OFF
   1-3   ON
   1-4   OFF
   1-5   OFF
   1-6   ON
   ====  ===

2. Connect the J-Link debugger to the board's CN1 debug connector.

3. Plug the CN8 serial port (to check the correct IPL).

4. Connect power to the board to P1 connector. The RDK board is powered by external 12 to 24V DC.
It is recommended to use the 12V/2A power adapter included in the box.

To power off the board, disconnect the J-Link debugger first, then remove
power.

Getting the HAL Renesas
=======================

Clock bring-up (see below) is implemented on top of Renesas' vendored FSP
BSP code, which ships in a separate ``hal_renesas`` repository and is
**not** part of the ``nuttx`` git tree.

``make context`` (run automatically as part of a normal build) clones the
repository at the pinned version into
``arch/arm/src/common/renesas/hal_renesas``, as defined in
``arch/arm/src/common/renesas/Make.defs``. Nothing needs to be cloned
manually. ``make distclean`` removes it.

``HAL_RENESAS_URL`` and ``HAL_RENESAS_VERSION`` can be set in the
environment to use another repository or revision, for example a local
checkout:

.. code-block:: bash

   HAL_RENESAS_URL=/path/to/hal_renesas make

Buttons and LEDs
================

Buttons
-------

The RZ/V2H-RDK has one active-high button on ``P10_0`` exposed through the
generic NuttX GPIO interface as ``/dev/gpio0``.

Hardware connection
~~~~~~~~~~~~~~~~~~~

Connect a normally-open momentary switch between ``P10_0`` and the board's
3.3-V logic supply, with a common ground between the board and the external
circuit.  The board configures ``P10_0`` as an input with its internal
pull-down enabled:

* button released: the pull-down drives the input low; ``/dev/gpio0`` reads
  ``0``;
* button pressed: the switch drives the input high; ``/dev/gpio0`` reads
  ``1``.

The pin definition is ``BOARD_P10_0_BUTTON`` in
``boards/arm/rzv2h/rzv2h-rdk/include/board.h``.  It selects
``BSP_IO_PORT_10_PIN_00`` with input, input-buffer, and pull-down settings.

.. warning::

   ``P10_0`` is a multiplexed pin.  Confirm the RDK schematic and make sure
   that no enabled peripheral claims this pin before connecting the button.

Configuration and registration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use a configuration that enables the generic GPIO test path, for example
``rzv2h-rdk:nsh-leds``.  The required options are::

   CONFIG_DEV_GPIO=y
   CONFIG_RZV2H_GPIO=y
   CONFIG_EXAMPLES_GPIO=y
   CONFIG_BOARD_LATE_INITIALIZE=y

During ``board_late_initialize()``, ``rzv2h_bringup()`` calls
``rzv2h_gpio_initialize()``.  This registers the button input first as
``/dev/gpio0`` and configures the input through the Renesas FSP IOPORT
driver.

NSH usage example
~~~~~~~~~~~~~~~~~

After booting NuttX, first verify that the device was registered::

   nsh> ls /dev
   nsh> gpio /dev/gpio0
   Driver: /dev/gpio0
     Input pin:     Value=0

Press and hold the button, then run the read command again::

   nsh> gpio /dev/gpio0
   Driver: /dev/gpio0
     Input pin:     Value=1

Release the button and repeat the command; the value must return to ``0``.

Current limitations
~~~~~~~~~~~~~~~~~~~

The current GPIO implementation supports synchronous read and write only.
GPIO interrupt attachment and enablement are placeholders, so ``gpio -w``
cannot report button transitions.  Runtime pin-type changes are also not
implemented, so do not use ``gpio -t``.  The board does not debounce the
external switch; use a polling application with software debounce if
continuous monitoring is required.

LEDs
----

Two active-low GPIO outputs are available for LED testing.  They are
registered after the button input, giving the following device mapping:

.. list-table::
   :header-rows: 1

   * - Pin
     - GPIO device
     - Function
   * - ``P11_4``
     - ``/dev/gpio1``
     - LED output 1
   * - ``P11_5``
     - ``/dev/gpio2``
     - LED output 2

Using the NSH GPIO application
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``nsh`` configuration enables ``CONFIG_DEV_GPIO`` and
``CONFIG_EXAMPLES_GPIO``.  The ``gpio`` example is reused from the standard
NuttX application repository at ``apps/examples/gpio``; it is not an
RZ/V2H-specific application.  The RZ/V2H board port only configures the pins,
registers ``/dev/gpio0`` through ``/dev/gpio2``, and enables the example in
the board configuration.

For these LED outputs, only ``-o`` and the device path are needed.  The
``-o`` option writes the logical pin level (zero or one), while the device
path selects the LED.  Because the LEDs are active low, write zero to turn an
LED on and one to turn it off.  For example::

   nsh> gpio -o 0 /dev/gpio1
   nsh> gpio -o 1 /dev/gpio1
   nsh> gpio -o 0 /dev/gpio2
   nsh> gpio -o 1 /dev/gpio2

Using the NuttX user-LED application
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``nsh-leds`` configuration also enables the standard NuttX user-LED
interface and example with these options::

   # CONFIG_ARCH_LEDS is not set
   CONFIG_USERLED=y
   CONFIG_USERLED_LOWER=y
   CONFIG_EXAMPLES_LEDS=y
   CONFIG_BOARD_LATE_INITIALIZE=y

During late initialization, the board registers ``/dev/userleds``.  The
``board_userled_*()`` callbacks configure ``P11_4`` and ``P11_5``, convert the
logical LED state to the active-low pin level, and support LED mask bits
``0x01`` and ``0x02``.

Run the unmodified NuttX example from ``apps/examples/leds`` to cycle the two
LED outputs::

   nsh> ls /dev
   nsh> leds

The device list should contain ``userleds``, and the example should report a
supported LED mask of ``0x03``.

Serial Consoles
===============

The default ``nsh`` configuration uses SCI channel 0 as the serial console
at 115200 baud, 8 data bits, no parity, and one stop bit. The board pin
definitions route SCI0 as follows:

======  ========  =======================  ==========
Signal  Pin       Board definition         J1 header
======  ========  =======================  ==========
TXD0    ``P5_0``  ``BOARD_SCI0_TXD_GPIO``  J1 pin 36
RXD0    ``P5_1``  ``BOARD_SCI0_RXD_GPIO``  J1 pin 22
======  ========  =======================  ==========

Connect a 3.3 V TTL UART adapter to the J1 header pins listed above.
Cross the data lines so that J1 TXD0 connects to the adapter RX and J1
RXD0 connects to the adapter TX. Hardware flow control is not enabled.
Use a 3.3 V TTL UART adapter, not an RS-232 adapter.

The relevant default options are::

   CONFIG_RZV2H_UART_SCI=y
   CONFIG_RZV2H_SCI_B_UART0=y
   CONFIG_SCI0_SERIAL_CONSOLE=y
   CONFIG_SCI0_BAUD=115200
   CONFIG_SCI0_BITS=8
   CONFIG_SCI0_PARITY=0
   CONFIG_SCI0_2STOP=0

After boot, SCI0 is available as both ``/dev/console`` and ``/dev/ttyS0``.
The terminal should display the NuttX banner followed by the ``nsh>`` prompt.
Verify the registration with::

   nsh> ls /dev
   /dev:
    console
    null
    ttyS0
    zero

Additional enabled SCI channels are registered as later ``/dev/ttyS*``
devices in ascending hardware-channel order. Their TX and RX pin choices are
board-specific and are defined by ``BOARD_SCIn_TXD_GPIO`` and
``BOARD_SCIn_RXD_GPIO`` in ``boards/arm/rzv2h/rzv2h-rdk/include/board.h``.

OpenAMP
========

The ``nsh-openamp`` configuration runs NuttX on CR8_0 as the remote side of a
compatible RZ/V Multi-OS v3.3.0 Linux RPMsg sample. It enables the
RPTUN lower half, channel 1, and the ``rpmsg-service-1`` echo
endpoint. The MHU-B notification uses channel 3 (FSP IRQ 314); the
resource-table entry is at ``0x42f00070``, the two VRings are at
``0x43100000`` and ``0x43150000``, and RPMsg buffers start at
``0x43500000``. Only one RPMsg channel is active in NuttX at a time.

The RDK board code inherits this layout from the EVK sample; the board
name alone does not prove Linux-peer compatibility. The RDK Linux peer
must reserve ``0x42f00000--0x42f01fff`` for the resource table/MHU
shared memory and ``0x43000000--0x437fffff`` for RPMsg, with matching
UIO and MHU routing. A mismatch may prevent notifications or corrupt
shared memory.

Configuration and build
-----------------------

From the NuttX source directory::

   $ tools/configure.sh rzv2h-rdk:nsh-openamp
   $ make -j4

The defconfig selects ``CONFIG_RZV2H_RPTUN=y``,
``CONFIG_RZV2H_RPTUN_CHANNEL1=y``,
``CONFIG_RZV2H_RPTUN_ECHO=y``,
``CONFIG_RPMSG_LOCAL_CPUNAME="cr8-0"``, and
``CONFIG_RZV2H_UBOOT_IMAGE=y``. It uses SCI0 for the 115200-8N1
NuttShell console. The build creates ``nuttx_uboot.bin`` in the
NuttX build directory. It contains the initialized SRAM image and starts
with a load copy of the vectors.

.. note::

   SCI peripheral and pin ownership must not conflict between NuttX on
   CR8_0 and Linux on CA55. For the tested RDK pairing, SCI0 (TX on
   ``P5_0``, RX on ``P5_1``) was checked as non-conflicting. This check is
   specific to that Linux image and device tree. If the Linux image, DTB,
   overlays, or console assignment changes, verify that Linux does not
   enable SCI0 or remux these pins; otherwise the NuttX console may fail.

Procedure
---------

Prepare the Linux image
~~~~~~~~~~~~~~~~~~~~~~~

Get the RDK Linux image from the `RDK software page
<https://www.renesas.com/en/design-resources/boards-kits/ws125-v2hrdkrefz>`_
and follow the `RDK quick setup guide
<https://renesas-rdk.github.io/rzv2h_rdk_documentation/latest/chapter-1/quick_setup_guide.html>`_.
The Linux image must provide an RPMsg peer compatible with the Multi-OS
v3.3.0 channel-1 layout, including ``rpmsg_sample_client`` and the
matching UIO reservations.

Prepare the SD card
~~~~~~~~~~~~~~~~~~~

Write the Linux image to a microSD card as described in the quick setup
guide. Then copy ``nuttx_uboot.bin`` from the NuttX build directory to
``boot/`` on the card's second (ext4) partition::

   $ sudo mkdir -p /mnt/sd
   $ sudo mount /dev/sdb2 /mnt/sd
   $ sudo cp nuttx_uboot.bin /mnt/sd/boot
   $ sync
   $ sudo umount /mnt/sd

.. warning::

   Replace ``/dev/sdb`` with the device name of your microSD card. Check
   it with ``lsblk`` before mounting or writing anything.

Insert the card into the RDK's microSD slot. The U-Boot commands below
assume its second partition is ``mmc 0:2``.

Start NuttX and Linux from U-Boot
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Connect a terminal (115200-8N1) to the RDK U-Boot console, power on the
board, and stop U-Boot autoboot. Back up the existing environment.
Enter each ``setenv`` command as **one line**; quoting preserves the
semicolons::

   => setenv cr80start 'dcache off; mw.l 0x10420D24 0x04000000; mw.l 0x10420600 0xE000E000; mw.l 0x10420604 0x00030003; mw.l 0x10420908 0x1FFF0000; mw.l 0x10420C44 0x003F0000; mw.l 0x10420C14 0x00000000; mw.l 0x10420908 0x10001000; mw.l 0x10420C48 0x00000020; mw.l 0x10420908 0x1FFF1FFF; mw.l 0x10420C48 0x00000000; mw.l 0x12040000 0xe51ff004; mw.l 0x12040004 0x08180000; ext4load mmc 0:2 0x08180000 boot/nuttx_uboot.bin; mw.l 0x10420C14 0x00000003; dcache on'
   => run cr80start
   => run envboot
   => setenv mmc_args "setenv bootargs 'rw rootwait earlycon root=/dev/mmcblk0p2 clk_ignore_unused'"
   => run prodsdboot

``run envboot`` imports ``boot/uEnv.txt`` from the SD card, which
defines ``mmc_args`` and ``prodsdboot``. The following ``setenv
mmc_args`` adds ``clk_ignore_unused`` to the kernel arguments for this
boot, and ``run prodsdboot`` loads the kernel, the
``rzv2h-rdk-ver1.dtb`` device tree and its overlays, and boots Linux.

``cr80start`` runs these commands in order. The step numbers refer to
``release_reset_cr8()`` in the Renesas Multi-OS v3.3.0 CM33 sample
(``src/cr8_start.c``).

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Command
     - Purpose
   * - ``dcache off``
     - Turn off the U-Boot data cache so the writes below reach memory directly
   * - ``mw.l 0x10420D24 0x04000000``
     - Release the CR8 bus module stop
   * - ``mw.l 0x10420600 0xE000E000``, ``mw.l 0x10420604 0x00030003``
     - Turn on CR8 clocks: CR8_0_PERIPHCLK, ACLK, ATCLK0 (CLKON_0) and TSCLK, DBGCLK (CLKON_1)
   * - ``mw.l 0x10420908 0x1FFF0000``
     - Assert CR8 resets (CPG_RST_2)
   * - ``mw.l 0x10420C44 0x003F0000``
     - Step 2: set CPG_LP_CR8_CTL3 request bits
   * - ``mw.l 0x10420C14 0x00000000``
     - Step 3: CPG_CR8_CONFIG1 = 0, "Disable fetch" (CPU halted)
   * - ``mw.l 0x10420908 0x10001000``
     - Step 4: release CR8_0_NMISCRESET (RSTB12)
   * - ``mw.l 0x10420C48 0x00000020``
     - Step 6: write RESET_TRIG
   * - ``mw.l 0x10420908 0x1FFF1FFF``
     - Step 8: release all CR8 resets
   * - ``mw.l 0x10420C48 0x00000000``
     - Step 10: clear RESET_TRIG
   * - ``mw.l 0x12040000 0xe51ff004``
     - Load the program into ITCM (``copy_cr8_prog()``; the reset vector at 0x12040000 is 0x42040000 in the CM33 view): ITCM word 0 = ``ldr pc, [pc, #-4]``
   * - ``mw.l 0x12040004 0x08180000``
     - ITCM word 1 = jump target 0x08180000
   * - ``ext4load mmc 0:2 0x08180000 boot/nuttx_uboot.bin``
     - Load the program (``copy_cr8_prog()``): NuttX image into RCPU-SRAM
   * - ``mw.l 0x10420C14 0x00000003``
     - Step 12: nCPUHALT = 3, "Enable fetch"
   * - ``dcache on``
     - Turn the U-Boot data cache back on

``cr80start`` powers up CR8_0 and keeps it halted while its resets are
released. It then writes the two-word reset trampoline into ITCM (see
`Linker script`_), loads ``nuttx_uboot.bin`` into RCPU-SRAM at
0x08180000, and lets CR8_0 start fetching. NuttX is started before Linux; Linux kernel RemoteProc is
**not** used to load NuttX in this flow.

The ``clk_ignore_unused`` argument stops Linux's unused-clock pass from
disabling clocks needed by CR8_0. By default Linux owns all clock
configuration, and peripherals it does not enable itself enter Module
Standby after the kernel boots; see section 3.3 of the
`RZV Multi-OS Package v3.3.0
<https://www.renesas.com/en/document/sws/rzv-multi-os-package-v330?r=25578220>`__.
The argument does not resolve peripheral-ownership conflicts. If the
Linux image uses other boot variables, follow its verified Linux boot
procedure instead.

Linux RPMsg test
----------------

NuttX should register ``/dev/rptun/linux`` after startup. Log in to
the RDK Linux image (the provided Ubuntu image defaults to
``ubuntu``/``ubuntu``). If the compatible sample application is
installed, run::

   $ sudo rpmsg_sample_client

Select option ``4`` (CR8 core0, channel 1). NuttX should create
``rpmsg-service-1`` when the RPMsg device is ready. Success requires
the Linux client to receive and validate echoed payloads; endpoint
creation alone is insufficient. When the client sends its shutdown
word, NuttX logs the shutdown and re-arms RPTUN for another Linux
session.

Linker script
-------------

``boards/arm/rzv2h/rzv2h-rdk/scripts/rzv2h-rdk_cr8_0.ld`` defines the memory
map for CR8_0:

- ``ITCM``  0x00000000, 128 KB
- ``DTCM``  0x00020000, 128 KB
- ``RAM``   0x08180000, 512 KB (RCPU-SRAM; holds .text, .data, .bss,
  heap, stack, and the .vectors load image)

The exception vector table always runs from ITCM at 0x00000000 and always
has a load copy at the start of SRAM, which ``arm_boot()`` copies to ITCM.
Debugger configurations link ``.vectors`` in SRAM. With
``CONFIG_RZV2H_UBOOT_IMAGE``, the table is linked at its ITCM address and
stored in SRAM (``> ITCM AT> RAM``). Because CR8_0 takes its reset vector
from ITCM, ``cr80start`` writes a two-word trampoline there before
releasing CR8_0: ``mw.l 0x12040000 0xe51ff004`` (``ldr pc, [pc, #-4]``)
and ``mw.l 0x12040004 0x08180000`` (its target, the start of
``nuttx_uboot.bin``). The reset slot of the stored table then branches to
``__start``, and ``arm_boot()`` replaces the trampoline with the full
table.
The read-only code region (``__mpu_code_start`` to ``__mpu_code_end``)
is padded to a power of two for its MPU region.

Configurations
==============

nsh
---

NuttShell configuration exercising clock, MPU, interrupt/timer, GPIO, and
SCI-B UART bring-up on CR8_0. SCI0 provides the default 115200-8N1 NSH
console through J1 pin 22 (RXD0) and J1 pin 36 (TXD0) and is registered
as ``/dev/console`` and ``/dev/ttyS0``.

nsh-leds
--------

This configuration enables ``CONFIG_USERLED``, registers ``/dev/userleds``,
and includes the standard NuttX ``leds`` example.  Run ``leds`` to cycle the
active-low ``P11_4`` and ``P11_5`` LED outputs.  The generic ``/dev/gpio0``,
``/dev/gpio1``, and ``/dev/gpio2`` interfaces are also available for
individual read/write testing.

nsh-spi
-------

NuttShell configuration for SPI-B loopback testing on channel 0 only.
SPI-B channel 0 is configured as a master at 1 MHz, mode 0, 8-bit words.
The ``spislv`` and ``spitool`` applications are included for interactive
loopback verification.  Only channel 0 can be tested on this board.

nsh-xspi
--------

Exercises the xSPI controller in QSPI mode (see QSPI Flash above).
Enables ``CONFIG_RZV2H_QSPI`` with an ``N25QXXX`` MTD flash driver,
mounted as a single flat LittleFS volume at ``/mnt/qspi``. Partitioning
(``CONFIG_MTD_PARTITION``) and syslog redirection
(``CONFIG_RZV2H_QSPI_SYSLOG_FILE``) are optional features (see above) not
enabled by this defconfig. As with the other configurations, there is no
working console driver yet, so this is intended for bring-up validation
with a debugger.

pwm
---

PWM configuration enabling GPT6 with the ``pwm`` application for
interactive frequency and duty-cycle testing.  The channel is registered
as ``/dev/pwm6`` at boot.

alarm
-----

NSH + RTC alarm example: enables RTC with alarm support and the
``apps/examples/alarm`` application.

nsh-openamp
-----------

NuttShell configuration for testing RPMsg between Linux on CA55 and NuttX
on CR8_0. It enables channel-1 RPTUN and the ``rpmsg-service-1`` echo
endpoint. This RDK preset generates single uboot images for U-Boot and
uses SCI0 for the console. See the OpenAMP section above for setup, boot,
and test instructions.

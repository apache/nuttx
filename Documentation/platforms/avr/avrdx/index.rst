======================
Microchip DA/DB family
======================

This is an attempt to support AVR DA/DB family of MCUs

Supported chips
===============

The code was developed using AVR128DA28 MCU on a breadboard. Currently,
all testing was done with that chip. The chips in DA family are quite
similar, the main differences are pin count and flash size. (Pin count
also corresponds to supported peripherals, chips with more pins have
more instances of the same peripheral, eg. more USARTs.)

Chips in DB family are fairly similar to DA as well,
some peripherals are missing, some features are added.

This means that support for other chips is fairly easy to add,
all that is needed is to provide interrupt vector definitions
and similar data describing the chip. All architecture code should
otherwise be written in a way to automatically support the chip.

Currently, ``AVR128DA64`` and ``AVR128DB64`` chips are supported
this way (ie. definitions describing the chip are provided
but there is no board.)

Clock Configuration
===================

The chip features internal high frequency oscillator, its frequency
can be set in the Kconfig-based configuration. Clock speed in MHz
is then provided in CONFIG_AVRDX_HFO_CLOCK_FREQ

System Timer
============

System timer is provided by RTC peripheral and takes over it. (In theory,
the Periodic Interrupt - PIT - component of it can be still used
but they share single clock.)

Tickless OS mode is supported in alarm mode. Configure it
in :menuselection:`RTOS Features --> Clocks and Timers --> Support tick-less OS`
:menuselection:`Tickless alarm` must be set, timer mode is not supported.
:menuselection:`System timer tick period (microseconds)` also needs to be
changed to value of at least 300. Higher value is recommended though,
300us is not going to be precise at all.

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======================
Peripheral  Notes
==========  =======================
GPIO        In board code
UART        See below
TWI         Master only, more below
==========  =======================

GPIO
----

For the most part, there is no need for any specific support for GPIO
in architecture code. For example, NuttX button input driver should only
need functions defined in the board code to work.

The architecture code provides means to share single interrupt vector
if the board needs to have multiple peripherals connected to a single
I/O port. See gpio_intr_mux.rst for details and breadxavr board
for example usage.

UART
----

UART is implemented using interrupts. The chip doesn't support DMA.
From some initial testing done using simple echo (read/write) loop
it seems the driver cannot handle sustained data transmission
with baud rate higher than 57600 even with the chip running at its
highest speed. With baud rate 57600, performance was dependent
on read/write buffer size. With buffer size of 8 there were still
some missed bytes.

To be able to use standard serial driver, it is needed to enable
the peripheral in MCU-specific configuration in
:menuselection:`System Type --> AVR DA/DB Peripheral Selections`
UART peripherals are enabled using
:menuselection:`Enable serial driver for USARTn` options. When enabled,
additional option :menuselection:`Activate USART0 alternative pinout`
is provided. See the chip datasheet for pinout information.

Other UART settings are then configured in driver configuration
:menuselection:`Device Drivers --> Serial Driver Support`
For every enabled peripheral, there is a corresponding
:menuselection:`USARTn Configuration` section. RTS/CTS and DMA support
must NOT be enabled here, neither of those is supported.

Each enabled peripheral is automatically registered as ``/dev/ttySn``
device file. File name corresponds to the peripheral related to it.
This for example means that ``USART1`` peripheral will always
be accessed through ``/dev/ttyS1`` regardless of what other ``USART``
peripherals are enabled (if any.)

TWI
---

Currently, only master is supported. Implementation details and quick
usage instructions can be found in :doc:`docs/twi` document.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

Other documents
===============

.. toctree::
   :glob:
   :maxdepth: 1

   docs/*

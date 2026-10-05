=============
Renesas RA8M1
=============

The RA8M1 is a Renesas RA family MCU built around an Arm Cortex-M85 core
(Armv8.1-M, Helium/MVE, running at up to 480 MHz), with TrustZone for
Armv8-M support in hardware.  This NuttX port is a flat, no-TrustZone
image: it always runs in the secure state and uses the secure memory and
register aliases (see the flat/secure model note in the port's board
documentation).

Supported MCUs
==============

The following list includes MCUs from the RA8M1 series and indicates
whether they are supported in NuttX.  Only R7FA8M1AHECBD (BGA224, on the
EK-RA8M1) has been tested; the others are selectable but untested.

=============  ======= ================
MCU            Support Note
=============  ======= ================
R7FA8M1AFECAM  No      Selectable, untested
R7FA8M1AFECBD  No      Selectable, untested
R7FA8M1AFECFB  No      Selectable, untested
R7FA8M1AFECFC  No      Selectable, untested
R7FA8M1AFECFP  No      Selectable, untested
R7FA8M1AHECAM  No      Selectable, untested
R7FA8M1AHECBD  Yes     Tested on the EK-RA8M1
R7FA8M1AHECFB  No      Selectable, untested
R7FA8M1AHECFC  No      Selectable, untested
R7FA8M1AHECFP  No      Selectable, untested
=============  ======= ================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  =====================================
Peripheral  Support  Notes
==========  =======  =====================================
FLASH       No
CLOCK       Yes      MOCO, HOCO, MOSC, PLL1, PLL2, SCICLK
ICU         Yes
KINT        No
ELC         No
DTC         Yes      Used by SCI (see SCI below); normal mode only
DMAC        No
GPT         Yes      Timer only: /dev/timerN (GPT0-7 32-bit, GPT8-13 16-bit)
AGT         No
RTC         No
WDT         No
IWDT        No
SCI         Yes      UART only (SCI_B0-4, SCI_B9); FIFO, DTC, termios
IIC         No
SPI         No
SSIE        No
QSPI        No
SDHI        No
CAN         No
USBFS       No
ADC12       No
DAC12       No
ACMPLP      No
OPAMP       No
CRC         No
GPIO        Yes
==========  =======  =====================================

CLOCK
-----

The board's ``include/board.h`` defines ``BOARD_*`` macros that select the clock
sources, PLL settings and dividers, and ``arch/arm/src/ra8m1/ra_clockconfig.h``
derives every clock frequency from them and checks it against the limits in
the RA8M1 User's Manual (chapter 8) at build time.  See the board's own
documentation for the specific clock tree it configures.

The RA8M1 has two PLLs.  PLL1 can feed the system clock or a peripheral
dedicated clock (SCICLK); PLL2 can only feed a dedicated clock.  SCICLK is
enabled automatically whenever an SCI UART is enabled, since it is the
clock of the SCI baud rate generator.

SCI
---

The Serial Communications Interface, Type B (SCI_B) is configurable to
support several serial communication modes: Asynchronous (UART), Clock
synchronous, Simple SPI, Smart card interface, Simple IIC (master-only).
NuttX driver support covers UART mode.  RA8M1 has SCI_B0-4 and SCI_B9 (no
SCI_B5-8); each channel's baud rate generator runs from SCICLK.  Each
instance is enabled with its own ``CONFIG_RA_SCIn_UART``.

Each instance can independently use its 16-stage hardware FIFO or the
DTC (see `DTC`_ below) to move data, but not both: ``CONFIG_RA_SCIn_FIFO``
and ``CONFIG_RA_SCIn_TXDTC``/``RXDTC`` are mutually exclusive.  Measured on
hardware, a low receive trigger level (``CONFIG_RA_SCIn_FIFO_RXTRG``) gives
better overrun resistance at high baud rates than RX DTC does, since DTC
only protects a byte while its single-activation transfer is actually
armed, not continuously -- see the Kconfig help for each option for the
detail.

``CONFIG_SERIAL_TERMIOS`` enables ``TCGETS``/``TCSETS`` (baud rate, parity,
stop bits only -- word length and the FIFO/DTC choice are fixed at build
time).  ``apps/system/stty`` can change these live, e.g.
``stty -F /dev/ttyS1 speed 3000000`` (FIFO with TTRG=15/RTRG=0 tested
clean up to 3 Mbps).

``CONFIG_SERIAL_TIOCGICOUNT`` exposes frame/overrun/parity counters
through the standard ``TIOCGICOUNT`` ioctl; nothing pushes a
notification, an application has to poll it.

DTC
---

The Data Transfer Controller offloads repetitive, register-triggered
transfers (e.g. UART TX/RX) from the CPU: the ICU event that would
otherwise interrupt the CPU instead activates the DTC for one transfer,
and once its count is exhausted it still interrupts the CPU normally.
Only normal mode (one activation, one transfer) is implemented; repeat
and block modes are not.  Enabled with ``CONFIG_RA_DTC``; peripheral
drivers that support it expose their own per-channel options (e.g.
``CONFIG_RA_SCI9_TXDTC``).

GPIO
----

Pins can be configured/operated using ``ra_gpio_*`` functions.

GPT
---

The General PWM Timer (GPT) has 8 32-bit channels (GPT0-7) and 6 16-bit
channels (GPT8-13).  NuttX uses a channel as a generic timer, registered as
``/dev/timerN`` through the upper-half timer driver (``CONFIG_TIMER``): the
channel counts up in saw-wave PWM mode, with the clock taken from PCLKD
divided by the smallest of 1, 2, 4, 8, 16, 32, 64, 256 or 1024 that fits the
requested timeout, and the overflow interrupt is the timer expiry.  The
PWM, capture, dead time and ELC/hardware trigger functions are not
supported.

Each channel is enabled with ``CONFIG_RA_GPTn_GPT`` (under
``CONFIG_RA_GPT_TIMER``) and registered by the board with
``ra_gpt_timer_initialize()``.  The timeout is set with
``TCIOC_SETTIMEOUT`` in microseconds and can be changed while the timer is
running.  The longest timeout is about 4295 s on a 32-bit channel, and
about 559 ms on a 16-bit channel at a PCLKD of 120 MHz.

ICU
---

The Interrupt Controller Unit routes peripheral events to NVIC vectors:
any event can be routed to any of its IELSR slots.  NuttX gives each event
that an enabled peripheral needs a slot, packed from the first slot in the
order they are listed in ``ra8m1_irq.h``, so only the events of the
peripherals a build enables use a slot: four for each SCI_B UART, and one
(the counter overflow) for each GPT channel.  ``ra_attach_icu()`` calls at
start-up route those events into their slots.

Supported Boards
=================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

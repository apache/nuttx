=============
Nordic nRF54L
=============

The nRF54L series from Nordic Semiconductor is based on an ARM Cortex-M33
application core. NuttX supports nRF54L15 and nRF54LM20A/B as standalone
secure images, with a 128 MHz CPU clock and SysTick scheduling.

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  ======= =====================================
Peripheral  Support Notes
==========  ======= =====================================
GPIO        Yes
GPIOTE      No
GRTC        No
PWM         No
QDEC        No
RADIO       No
RRAMC       No
SAADC       No
SPIM        No
TIMER       No
TWIM        No
UARTE       Yes     No hardware flow control
USBHS       No
WDT         No
==========  ======= =====================================

GPIO
----

Pins can be configured and operated using ``nrf54l_gpio_*`` functions.

UARTE
-----

UART0 through UART4 correspond to UARTE20, UARTE21, UARTE22, UARTE30 and
UARTE00. nRF54LM20 also provides UART5 and UART6 using UARTE23 and UARTE24.
Each enabled UART requires ``BOARD_UARTn_TX_PIN`` and ``BOARD_UARTn_RX_PIN``
definitions. Any enabled UART can be selected as the serial console.
UART4 requires the dedicated UARTE00 TXD and RXD pins listed in the chip's
pin assignment table.

``CONFIG_SERIAL_TERMIOS`` enables runtime baud rate, data bits, parity and
stop bit configuration. A format change returns ``-EBUSY`` while a byte is
being transmitted or DMA error recovery is in progress.

Power Management
================

With SysTick selected, the CPU remains awake so its clock keeps running.
``CONFIG_PM`` initializes the NuttX power management framework. System OFF
is not supported.

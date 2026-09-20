=============
Nordic nRF54L
=============

The nRF54L series from Nordic Semiconductor is based on an ARM Cortex-M33
application core. NuttX supports nRF54L15 and nRF54LM20A/B as standalone
secure images, with a 128 MHz CPU clock and SysTick or GRTC scheduling.

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  ======= =====================================
Peripheral  Support Notes
==========  ======= =====================================
GPIO        Yes
GPIOTE      No
GRTC        Yes     Counter and tickless scheduling
PWM         No
QDEC        No
RADIO       Yes     Bluetooth LE through SDC
RRAMC       Yes     Progmem erase/write interface
SAADC       No
SPIM        No
TIMER       Yes
TWIM        No
UARTE       Yes     No hardware flow control
USBHS       No
WDT         No
==========  ======= =====================================

GPIO
----

Pins can be configured and operated using ``nrf54l_gpio_*`` functions.

GRTC
----

``nrf54l_grtc_init(0)`` provides access to the 52-bit, 1 MHz system counter
and twelve compare channels. ``CONFIG_NRF54L_SYSTIMER_GRTC`` reserves the
instance and compare channel zero for tickless scheduling. The GRTC is timed
by LFCLK, so the LFCLK source selected with ``CONFIG_NRF54L_USE_LFCLK``
determines the long-term accuracy of the system time.
With SDC enabled, channels 7 through 11 are reserved for MPSL.

RADIO
-----

``CONFIG_NRF54L_SOFTDEVICE_CONTROLLER`` enables Nordic's SoftDevice Controller
using nrfxlib 3.4.1. This SDK version no longer accepts raw HCI command
packets, so ``nrf54l_sdc_hci.c`` translates each HCI command into the
corresponding SDC command function and builds the Command Complete or
Command Status event. Events and ACL data pass through unchanged. Supported
commands cover legacy advertising, scanning, central and peripheral
connections, Data Length Extension, and 1M, 2M and Coded PHYs. The driver
provides the NuttX Bluetooth driver interface for the native host or HCI
transport. The build downloads nrfxlib; ``CONFIG_ALLOW_BSDNORDIC_COMPONENTS``
must be enabled.

MPSL and SDC reserve TIMER10, TIMER20, GRTC channels 7 through 11, RADIO,
ECB00, AAR00, CCM00, CLOCK, TEMP and RRAMC, together with their DPPI/PPIB
resources. TIMER0, TIMER6 and progmem are unavailable while SDC is enabled.
The remaining GRTC channels can be used by the tickless scheduler. HFXO
remains running; the LFCLK source is selected with ``CONFIG_NRF54L_USE_LFCLK``.

RRAMC
-----

``CONFIG_NRF54L_PROGMEM`` enables the progmem interface. RRAM supports
overwriting either bit value; erase operations fill emulated 4 KiB blocks
with ``0xff``. Writes require word-aligned addresses and lengths.

TIMER
-----

TIMER0 through TIMER6 correspond to TIMER20, TIMER21, TIMER22, TIMER23,
TIMER24, TIMER00 and TIMER10. The timer lower-half driver uses a 1 MHz
counter clock on each instance.

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

The idle loop uses ``WFI`` when GRTC provides tickless scheduling. With
SysTick selected, the CPU remains awake so its clock keeps running.
``CONFIG_PM`` initializes the NuttX power management framework. System OFF
is not supported.

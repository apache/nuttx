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
GPIOTE      Yes
GRTC        Yes     Counter and tickless scheduling
PWM         Yes
QDEC        No
RADIO       Yes     Bluetooth LE through SDC
RRAMC       Yes     Progmem erase/write interface
SAADC       Yes
SPIM        Yes
TIMER       Yes
TWIM        No
UARTE       Yes     No hardware flow control
USBHS       No
WDT         No
==========  ======= =====================================

GPIO
----

Pins can be configured and operated using ``nrf54l_gpio_*`` functions.
GPIOTE supports channel events and tasks, with optional per-pin callbacks
for PORT events. Channels 0 through 7 belong to GPIOTE20 and serve P1 and
P3. Channels 8 through 11 belong to GPIOTE30 and serve P0. P2 does not
support GPIOTE.

GRTC
----

``nrf54l_grtc_init(0)`` provides access to the 52-bit, 1 MHz system counter
and twelve compare channels. ``CONFIG_NRF54L_SYSTIMER_GRTC`` reserves the
instance and compare channel zero for tickless scheduling. The GRTC is timed
by LFCLK, so the LFCLK source selected with ``CONFIG_NRF54L_USE_LFCLK``
determines the long-term accuracy of the system time.
With SDC enabled, channels 7 through 11 are reserved for MPSL.

PWM
---

PWM0 through PWM2 correspond to PWM20, PWM21 and PWM22. Each instance
supports four output channels. The board must provide an
``NRF54L_PWMn_CHm_PIN`` definition for each enabled channel.

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

SAADC
-----

``nrf54l_adcinitialize()`` provides the ADC lower half, with up to eight
single-ended or differential channels, 8/10/12/14-bit resolution and
optional oversampling. Each ``ANIOC_TRIGGER`` collects one result per
channel. The local timer supports a single channel; scanning with
oversampling requires burst mode on every channel.

The internal reference is 0.9 V. Available gains range from 1/4 to 2.
External analog and reference pins must have their digital input buffers,
outputs and pulls disconnected before opening the device. AIN0 through
AIN7 map to P1.04/05/06/07/11/12/13/14 on L15 and
P1.00/31/30/29/06/05/04/03 on LM20.

SPIM
----

SPI0 through SPI4 correspond to SPIM20, SPIM21, SPIM22, SPIM30 and SPIM00.
nRF54LM20 also provides SPI5 and SPI6 using SPIM23 and SPIM24. Each bus
requires ``BOARD_SPIn_SCK_PIN`` and the applicable ``BOARD_SPIn_MOSI_PIN``
and ``BOARD_SPIn_MISO_PIN`` definitions. Boards provide the select, status
and optional command/data callbacks declared in ``nrf54l_spi.h``.

The driver supports 8-bit transfers, modes 0 through 3 and configurable
bit order. SPI4 supports up to 32 MHz; other instances support up to 8 MHz.
``SPI_SETFREQUENCY()`` returns the clock selected using the hardware divider.
SPI and UART cannot use the same SERIAL instance concurrently.

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

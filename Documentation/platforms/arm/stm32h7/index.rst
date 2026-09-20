==========
ST STM32H7
==========

Supported MCUs
==============

Dual-core lines:

===========  ======= ================
MCU          Support Note
===========  ======= ================
STM32H747    Partial Only STM32H747XI
STM32H757    No
STM32H745    Yes
STM32H755    Partial Only STM32H755II and STM32H755XI
===========  ======= ================

Single-core lines:

===========  ======= ================
MCU          Support Note
===========  ======= ================
STM32H7A3    No
STM32H7B3    Partial Only STM32H7B3LI
STM32H743    Yes
STM32H753    Yes
STM32H742    No
STM32H725    No
STM32H735    No
STM32H723    No
STM32H733    No
STM32H7R3    Yes
STM32H7R7    Yes
STM32H7S3    Yes
STM32H7S7    Yes
===========  ======= ================

Value lines:

===========  ======= ================
MCU          Support Note
===========  ======= ================
STM32H7B0    No
STM32H750    No
STM32H730    No
===========  ======= ================


Peripheral Support
==================


The following list indicates peripherals supported in NuttX:

==========  =======  =====
Peripheral  Support  Notes
==========  =======  =====
RAMECC      No
FLASH       Yes
SMM         No
PM          ?
RCC         Yes
CRS         No
HSEM        Yes
GPIO        Yes
SYSCFG      Yes
MDMA        ?
DMA         Yes
BDMA        Yes
DMA2D       Yes
EXTI        Yes
CRC         Yes
FMC         Yes
QUADSPI     Yes
DLYB        No
ADC         Yes
DAC         Yes
VREFBUF     No
COMP        No
OPAMP       No
DFSDM       No
DCMI        No
LTDC        Yes
JPEG        No
RNG         Yes
CRYP        No
HASH        ?
HRTIM       No
TIM         Yes
LPTIM       No
IWDG        Yes
WWDG        Yes
RTC         Yes
I2C         Yes
USART       Yes
SPI         Yes
I2S         ?
SAI         No
SPIDIFRX    No
SWPMI       No
MDIOS       ?
SDMMC       Yes
FDCAN       Yes
OTG_FS      Yes
OTG_HS      Yes
ETH         Yes
HDMI_CEC    No
==========  =======  =====

STM32H7R/S peripherals
----------------------

The following peripherals are available only on the STM32H7R/S lines:

==========  =======
Peripheral  Support
==========  =======
XSPI        Yes
SBS         Yes
GPDMA       No
HPDMA       No
ADF         No
CORDIC      No
DCMIPP      No
DTS         No
GFXMMU      No
GFXTIM      No
GPU2D       No
I3C         No
MCE         No
PKA         No
PSSI        No
SAES        No
UCPD        No
==========  =======

Dual-core support
=================

Some of the STM32H7 chips have an additional Cortex-M4 core built-in.
The selection of the core for which the image is build is made using options:

  - ``CONFIG_ARCH_CHIP_STM32H7_CORTEXM7`` - selects Cortex-M7 core
  - ``CONFIG_ARCH_CHIP_STM32H7_CORTEXM4`` - selects Cortex-M4 core

Support for the CM7 core is always enabled, support for the CM4 core is controlled
with the ``CONFIG_STM32_CORTEXM4_ENABLED`` option.

Interprocessor communication between cores is realized with the NuttX RPTUN
device based on the OpenAMP framework. ``HSEM`` is used for synchronization and
notification between cores.

32kB of the SRAM3 is reserved for shared memory and this is the only available
option at the moment.

Ethernet
========

The Ethernet MAC of the STM32H7 has a time counter that can stamp the frames
it sends or receives in hardware, at the moment the frame crosses the MAC. A
protocol such as the Precision Time Protocol (IEEE 1588, PTP) uses these
stamps to compare its clock with the clock of another node without the delay
of the software (interrupts and task scheduling) getting into the
measurement. The options below enable that part of the driver.

Precision Time Protocol
-----------------------

The counter has a 32-bit seconds part and a nanoseconds part that counts up to
10^9 (digital rollover), and it is clocked from ``HCLK``. Its rate can be
trimmed by up to +/- 50 %, and its phase can be stepped, which is what a PTP
daemon needs to steer it towards a master clock.

  - ``CONFIG_STM32_ETH_PTP`` - Enables the PTP timer of the MAC. It starts
    at zero when the interface goes up, whether or not the link is up, and it
    is cleared by every reset of the MAC, for example when the interface goes
    down.

  - ``CONFIG_STM32_ETH_PTP_GPIO`` - Enables the pulse-per-second output of the
    MAC on the pin ``GPIO_ETH_PPS_OUT``. The board has to define it in its
    ``board.h`` with one of the pins of the chip (on the STM32H743
    ``GPIO_ETH_PPS_OUT_1`` is PB5 and ``GPIO_ETH_PPS_OUT_2`` is PG8). It is a
    pulse train with a period of one second and a width of half of it, that
    starts at a whole second of the counter. It does not depend on the link
    or on a PTP daemon. The train counts by itself and does not follow a step
    of the time, so the driver starts it again at the next whole second when
    the time is set or stepped.

Clock device
------------

With ``CONFIG_PTP_CLOCK`` the driver registers the counter as a PTP hardware
clock, ``/dev/ptp0`` for the first Ethernet interface (the number of the
device is the number of the interface). It follows the generic framework
described in :doc:`/components/drivers/special/ptp` and offers reading and
setting the time, the resolution (the increment of the counter, 10 ns with
``HCLK`` at 200 MHz), frequency adjustment (``ADJ_FREQUENCY``, up to +/- 50 %)
and phase steps (``ADJ_OFFSET`` and ``ADJ_SETOFFSET``). It does not offer the
cross timestamp of the system and the device clock. The seconds of the time
that is set have to fit in 32 bits.
``CONFIG_CLOCK_ADJTIME`` is needed for ``clock_adjtime()``.

A configuration with the options above, for a board with the Ethernet MAC,
looks like this:

.. code-block:: kconfig

   CONFIG_STM32_ETHMAC=y
   CONFIG_STM32_ETH_PTP=y
   CONFIG_STM32_ETH_PTP_GPIO=y
   CONFIG_PTP_CLOCK=y
   CONFIG_CLOCK_ADJTIME=y

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

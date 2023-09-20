======
SAM V7
======

This page contains information regarding MCUs series SAM E70, SAM S70, SAM V70 and SAMV71 made
by Microchip. The series is based around and ARM Cortex-M7 core running up to 300 MHz.

Microchip distinguishes those MCUs into three series SAM E, SAM S SAM V but NuttX source code uses the
same files and functions for all of those series and therefore they can be merged into one category and
named as SAM V7.

Supported MCUs
==============

The following list includes MCUs from SAM x7 series and indicates whether they are supported in NuttX

=======  =======  ==============  =================
MCU      Support  Core            Frequency
=======  =======  ==============  =================
SAM E70  Yes      Cortex-M7       300 MHz
SAM S70  No       Cortex-M7       300 MHz
SAM V70  No       Cortex-M7       300 MHz
SAM V71  Yes      Cortex-M7       300 MHz
=======  =======  ==============  =================

Data and Instruction Cache
==========================

MCUs in this series have separated caches for instructions and data, both 16 kB. Data cache is
initially set as write-back but sometimes the configuration automatically switch it to write-through
mode. While write-back gives better performance, the reason of the switch are the issues in some
drivers that can arise because this mode is not correctly supported yet. This happens for example
when selecting ``CONFIG_SAMV7_EMAC`` (Ethernet MAC driver).

Data cache can also be switched to write-through mode manually by setting ARMV7M_DCACHE_WRITETHROUGH.

Tickless OS
===========

With Tickless OS, the periodic, timer interrupt is eliminated and replaced with a one-shot,
interval timer, that becomes event driven instead of polled. This allows to run the MCU with
higher resolution without using more of the CPU bandwidth processing useless interrupts.

Current implementation only supports version with two timers: a one-shot that provides the
timed events and a free running timer that provides the current time. Therefore two channels
has to be used for tickless mode. ``CONFIG_USEC_PER_TICK`` option determines the resolution
of time reported by :c:func:`clock_systime_ticks()` and the resolution of times that can be set
for certain delays including watchdog timers and delayed work.

The version (selected by ``CONFIG_SCHED_TICKLESS_ALARM``) with single free running timer that provides
current time and a capture/compare channel for timed events is not currently supported. This would
require just one channel, which can be useful in some MCUs that does not have many channels.

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======
Peripheral  Support
==========  =======
1Wire       Yes
ACC         No
AES         No
AFEC        Yes
DACC        Yes
EEFC        Yes
GMAC        Yes
HSMCI       Yes
I2SC        No
ICM         No
ISI         No
MCAN        Yes
MLB         No
PMC         No
PWM         Yes
QSPI        Yes
SMC         No
SPI         Yes
SSC         Yes
TWIHS       Yes
TC          Yes
USART       Yes
WDT         Yes
XDMAC       Yes
==========  =======

Analog Comparator Controller (ACC)
----------------------------------

Analog Comparator Controller generates an interrupt based on user settings. It can also
generate a compare event which can be used by the PWM driver.

This peripheral is currently not supported.

Advanced Encryption Standard (AES)
----------------------------------

The standard complies with the American FIPS Publication 197 specification. It is not
currently supported.

Analog Front-End Controller (AFEC)
----------------------------------

This controller combines a 12 bit ADC, DAC and two 6 to 1 analog multiplexers. The current
support in NuttX implements the controller as an analog/digital converter that can be trigger
either via software trigger from the application or by timer/counter that runs on a defined
frequency. Triggering by PWM is also supported by the MCUs but not currently implemented
in NuttX. The lower-half of this driver is initialize by calling :c:func:`sam_afec_initialize`

Software trigger can be selected by ``CONFIG_SAMV7_AFECn_SWTRIG``, timer/counter trigger by
``CONFIG_SAMV7_AFECn_TIOATRIG``. It is also necessary to select corresponding timer/counter
(0 for AFEC0 and 1 for AFEC1) and enable corresponding channels. Triggering from PWM driver
is also supported and can be configured by ``CONFIG_SAMV7_AFECn_PWMTRIG``. In this case
subsequental configuration of PWM driver is required (see below in PWM section).

The 12 bit resolution mode can be extended up to a 16 bit resolution by digital averaging.
The averaging can be set by ``CONFIG_SAMV7_AFECn_RES``.

The controller supports data transfer with DMA support which can be enabled by ``CONFIG_SAMV7_AFEC_DMA``.
Option ``CONFIG_SAMV7_AFEC_DMASAMPLES`` then sets the number of samples to be transferred.

Digital/Analog Converter Controller (DACC)
------------------------------------------

Digital/Analog Converter supports 12 bit resolution and can operate in free-running mode, maximum
speed mode, trigger mode from timer/counter and interpolation mode. Trigger mode is set by
enabling ``CONFIG_SAMV7_DAC_TRIGGER`` option.  The lower-half of this driver is initialize by
calling :c:func:`sam_dac_initialize`.

DMA data transfer is supported by the controller but currently not implemented in NuttX.

Ethernet MAC (GMAC)
-------------------

This module implements a 10/100 Mbps Ethernet MAC which is compatible with the IEEE 802.3 standard.
Number of RR and TX buffers can be configured by ``CONFIG_SAMV7_EMAC0_NRXBUFFERS`` and
``CONFIG_SAMV7_EMAC0_NTXBUFFERS`` respectively. Option ``CONFIG_SAMV7_EMAC0_PHYINIT`` may be selected
when board specific initialization (GPIOs configuration, PHY reset etc.) is required prior to
module usage.

High Speed Multimedia Card Interface (HSMCI)
--------------------------------------------

This module supports a high speed connection to MultiMedia Cards (MMC). Support for
the SD slots can be enabled with the following settings:

- System Type -> SAMV7 Peripheral Selection
   - ``CONFIG_SAMV7_HSMCI0=y``                : To enable HSMCI0 support
   - ``CONFIG_SAMV7_XDMAC=y``                  : XDMAC is needed by HSMCI0/1

- System Type
   - ``CONFIG_SAMV7_GPIO_IRQ=y``               : PIO interrupts needed
   - ``CONFIG_SAMV7_GPIOn_IRQ=y``              : Interrupt to corresponding pin gate

- Device Drivers -> MMC/SD Driver Support
   - ``CONFIG_MMCSD=y``                        : Enable MMC/SD support
   - ``CONFIG_MMSCD_NSLOTS=1``                 : One slot per driver instance
   - ``CONFIG_MMCSD_MULTIBLOCK_DISABLE=y``     : (REVISIT)
   - ``CONFIG_MMCSD_HAVE_CARDDETECT=y``        : Supports card-detect PIOs
   - ``CONFIG_MMCSD_MMCSUPPORT=n``             : Interferes with some SD cards
   - ``CONFIG_MMCSD_SPI=n``                    : No SPI-based MMC/SD support
   - ``CONFIG_MMCSD_SDIO=y``                   : SDIO-based MMC/SD support
   - ``CONFIG_SDIO_DMA=y``                     : Use SDIO DMA
   - ``CONFIG_SDIO_BLOCKSETUP=y``              : Needs to know block sizes

- RTOS Features -> Work Queue Support
   - ``CONFIG_SCHED_WORKQUEUE=y``              : Driver needs work queue support

- Application Configuration -> NSH Library
   - ``CONFIG_NSH_ARCHINIT=y``                 : NSH board-initialization, OR
   - ``CONFIG_BOARD_LATE_INITIALIZE=y``

The lower-half of this driver is initialized by calling :c:func:`sdio_initialize`.

Inter-IC Sound Interface (I2CS)
-------------------------------

This controller provides a 5 wire digital audio link to external audio devices. The link
is bidirectional and synchronous. The interface is compliant vit I2C specification.

This peripheral is currently not supported.

Integrity Check Monitor (ICM)
-----------------------------

Integrity Check Monitor is a DMA controller that performs hash calculation over memory
regions.

This peripheral is currently not supported.

Image Sensor Interface (ISI)
----------------------------

This controller connects a CMOS type sensor to the MCU and provides image captures is
selected formatrs.

This peripheral is currently not supported.

Controller Area Network (MCAN)
------------------------------

Provides support for communication according to ISO 11898-1:2015 and to Bosch CAN-FD
specification. It is possible to select CAN FD communication in NuttX configuration.

The lower-half of the peripheral is initialized by function :c:func:`sam_mcan_initialize`

Media Local Bus (MLB)
---------------------

This peripheral maps all the MOST Network data types into a single interface.

This peripheral is currently not supported.

Power Management Controller (PMC)
---------------------------------

Peripheral used to optimize power consumption of MCU.

Not yet supported.

Pulse Width Modulation Controller (PWM)
---------------------------------------

Pulse Width Modulation Controller provides a PWM output on 4 independent channels. Each channel
can control two complementary outputs. PWM can also be used to generate a signal that triggers
ADC conversion. The trigger is generated from configurable comparison units. These units can be
set by config option ``CONFIG_SAMV7_PWMn_TRIGx`` where n is number of PWM instance and x is the number
of comparison unit.

The peripheral has integrated fault protection that drives the output to zero when activated. The
protection activation can be trigger from various peripherals (ADC, PMC) or from GPIO inputs.

Furthermore PWM can implement a dead time delays before the activation of complementary outputs.
These delays are turn on by ``CONFIG_PWM_DEADTIME`` while dead time values are provided from application
level the same way as duty cycle is set.

The lower-half is initialized by function :c:func:`sam_pwminitialize`.

Quad Serial Peripheral Interface (QSPI)
---------------------------------------

This peripheral provides communication with external devices in host mode through synchronous serial
data link. It is possible to use QSPI peripheral in SPI mode if this is supported by MCU (config option
``CONFIG_SAMV7_QSPI_SPI_MODE``).

The lower-half is initialized by function :c:func:`sam_qspi_initialize` in case of QSPI mode and by
:c:func:`sam_qspi_spi_initialize` in case of SPI mode.

Static Memory Controller (SMC)
------------------------------

This peripheral is a part of External Bus Interface (EBI) which is designed to ensure the successful
data transfer between several external devices and the microcontroller.

This controller is currently not supported.

Serial Peripheral Interface (SPI)
---------------------------------

This is a synchronous serial data link that provides communication with external devices in host
or client mode.

The peripheral is initialized by :c:func:`sam_spibus_initialize` function.

Synchronous Serial Controller (SSC)
-----------------------------------

This controller provides a synchronous communication link with external devices.

The controller is initialized by :c:func:`sam_ssc_initialize` function.

Two Wire Interface (TWIHS)
--------------------------

It interconnects components on a two-wire bus. The bus is made up of one clock line
and one data line.

Timer Counter (TC)
------------------

The peripheral implements four timer counter modules, each supporting three independent channels.

Universal Synchronous Asynchronous Receiver Transceiver (USART)
---------------------------------------------------------------

The MCU supports both UART and USART controllers. USART can be also used in RS-485 mode (enabled
by ``CONFIG_SAMV7_USARTx_RS485MODE`` option) or can be used with RX DMA support. For this purpose it
is required to configure idle bus timeout value in ``CONFIG_SAMV7_SERIAL_DMA_TIMEOUT``. This option
ensures data are read from the DMA buffer even if it is not full yet. TX DMA support is not implemented
as well as entire DMA support for UART peripheral.

USART/UART can be also used to emulate 1 wire interface. SAMv7 MCUs do not have build in support for
1 wire therefore external hardware as TX/RX connection or optical isolation might be required. Selecting
``CONFIG_SAMV7_UARTx_1WIREDRIVER`` enables 1 wire driver and sets USART/UART peripheral to this mode.
Output pins are configured as if serial mode was selected plus TX is open drain. SAMv7 part of the driver
is initialized by :c:func:`sam_1wireinitialize` with port number as an argument.

Watchdog Timer (WDT)
---------------------

The timer is used to prevent system lock-up if the software is trapped in a deadlock.

DMA Controller (XDMAC)
----------------------

This peripheral provides a central direct memory access controller which can perform
peripheral to memory or memory to memory transfers.

Supported Boards
================

For board documentation please refer to ``board/arm/samv7`` section to separate README files.

..
   .. toctree::
      :glob:
      :maxdepth: 1

      boards/*/*

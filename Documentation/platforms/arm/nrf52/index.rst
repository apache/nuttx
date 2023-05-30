============
Nordic nRF52
============

The nRF52 series of chips from Nordic Semiconductor are based around an ARM Cortex-M4 core running
at 64 MHz and feature Bluetooth Low Energy (BLE) support.


Memory Map
==========

nRF52832
--------

============ ============= ======
Block Name   Start Address Length
============ ============= ======
FLASH        0x00000000    512K
RAM          0x20000000    64K
============ ============= ======

nRF52840
--------

============ ============= ======
Block Name   Start Address Length
============ ============= ======
FLASH        0x00000000    1024K
RAM          0x20000000    256K
============ ============= ======

Clock Configuration
===================

Clock settings are handled via Kconfig options, which determines whether to start external crystal
for the HFCLK, whether to start the LFCLK and which oscillator to use.

System Timer
============

The clock used for providing system time can be chosen via Kconfig. You can choose to use ARM SysTick
or use RTC in tickless mode.

Regulator Control
=================

DC/DC regulator can be made to be enabled at boot via Kconfig.

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  ======= =====
Peripheral  Support Notes
==========  ======= =====
GPIO        Yes
GPIOTE      Yes
I2S         No
MWU         No
NFCT        No
PDM         No
PPI         Yes
PWM         Yes
QDEC        No
QSPI        Yes
RADIO       Yes     Basic
RNG         Yes
RTC         Yes
SAADC       Yes
SPIM        Yes
SPIS        No
TEMP        No
TIMER       Yes
TWIM        Yes
TWIS        No
UART        Yes
UARTE       No
USBD        Yes
WDT         Yes
==========  ======= =====

Peripherals such as AAR, ACL, CCM, ECB are not directly used by NuttX since they
are part of BLE controller implementation (link).

GPIO/GPIOTE
-----------

Pins can be configured/operated using ``nrf52_gpio_*`` functions. Interrupts are
handled via the GPIOTE peripheral in one of two ways: via a GPIOTE channel or via
PORT events. The former allows for simultaneous rising/falling edge-sensitive interrupts
per-pin. However, as there are a limited number of channels (and sometimes these
are used by some drivers for specific tasks), it may not always be possible to use
this mechanism. The latter approach for pin interrupts is via the PORT event, determined
by pin state on a their corresponding GPIO port. This is related to the SENSE capability
of pins, which can only be set to either rising or falling edge sensing.

Depending on ``CONFIG_NRF52_PER_PIN_INTERRUPTS`` option, you can set a callback for
the PORT event itself or you can set a callback for a given pin. In the latter case
the driver scans for pins with DETECT bit high and calls the configured callback
automatically.

Finally, GPIOTE can also be used to configure a channel in *task mode*, which allows to
control pin state via tasks/events.

ADC
---

The SAADC peripheral is exposed via standard ADC driver. The lower-half of this driver
is initialized by calling :c:func:`nrf52_adcinitialize`.

I2C
---

I2C is supported both in polling and interrupt mode (via EasyDMA).

.. note:: The I2C peripheral does not support sending two transfers without sending
   a START nor RSTART. For this reason, this is supported via an internal buffer where
   messages will be first copied to and sent together.

The lower-half of I2C bus is initialized by :c:func:`nrf52_i2cbus_initialize`.
There's also a software (bitbang) I2C implementation for nRF52. The lower-half is
initialized via :c:func:`nrf52_i2c_bitbang_initialize`.

SPI
---

SPI is supported both in polling and interrupt-based (via EasyDMA) mode. The latter
supports arbitrarily long transfers using Nordic's list-mode EasyDMA (intermediate
transfers are currently still manually started).

It is possible to use SPI without either MOSI/MISO pin defined by simply not providing
the relevant ``BOARD_SPI*_MISO/MOSI_PIN`` definition.

This implementation support power management hooks, which will disable SPI peripheral when
entering either SLEEP or STANDBY modes and reconfigure it when going back to NORMAL mode.

UART
----

UART is implemented using polling. UARTE EasyDMA feature is not yet supported.
This may introduce a large number of interrupts which may be undesirable.

PPI
---

The PPI peripheral is supported via a specific API which lets you control the EVENT
and TASKs to trigger, both for individual and grouped channels.

When using channels, you should consider that some peripherals may use PPI internally
and some may be unavailable for further use. As a helper, if debug assertions are
enabled, calls to PPI API will check for a channel to actually be disabled when being
enabled and viceversa. This may help catch collisions in PPI use.

PWM
---

PWM is supported via standard driver. This means that more advanced features such as
complex sequences or waveform modes are not yet supported.

QSPI
----

QSPI is supported both in interrupt-based (via EasyDMA) mode and is exposed
via standard QSPI interface.

RNG
---

The RNG peripheral will be used to register a random/urandom device automatically, when
enabled.

TIMER
-----

The TIMER peripheral is exposed as standard timer.

RTC
---

The RTC peripheral is exposed as a standard timer, since it is really a low-power
timer, without any date handling capabilities.

USBD
----

The USBD peripheral is exposed via standard USBDEV interface.

WDT
---

The watchdog is supported via low-level API interface and also via standard watchdog
driver. The driver is written so as to handle an already running watchdog, which may
have been set by a bootloader.

BLE Support
===========

BLE is supported in nRF52 using Nordic's `SoftDevice Controller <https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrfxlib/softdevice_controller/README.html>`_, using HCI interface. To enable BLE support
you need to call :c:func:`nrf52_sdc_initialize` on boot, which will initialize the BLE controller.

SDC support involves registering various high-priority zero-latency interrupts and thus requires
enabling BASEPRI and high-priority interrupt support. On supported boards, a sample ``sdc`` configuration
is provided with settings already set.

Note that in this case, some peripherals (mostly those related to BLE) will be unavailable. Some PPI
channels will also be ocuppied (``NRF52_PPI_NUM_CONFIGURABLE_CHANNELS`` will be set accordingly in this case).

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

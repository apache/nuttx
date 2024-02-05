============
Sony CXD56xx
============

(6 x ARM Cortex-M4)

Support for the CXD56\ *xx* was
introduced by Nobuto Kobayashi in NuttX-7.30.

**Sony Spresence**. Spresense is a compact development board based on
Sony’s power-efficient multicore microcontroller CXD5602. Basic support
for the Sony Spresense board was included in the contribution of Nobuto
Kobayashi in NuttX-7.30. *NOTE*: That was an initial, bare bones basic
Spresense port sufficient for running a NuttShell (NSH) and should not
be confused with the full Spresence SDK offered from Sony. Since then
there has been much development of the NuttX CXD56xx port.

**Features:**

-  Integrated GPS: Embedded GNSS with support for GPS, QZSS.
-  Hi-res audio output and multi mic inputs" Advanced 192kHz/24 bit
   audio codec and amplifier for audio output, and support for up to 8
   mic input channels.
-  Multicore microcontroller: Spresense is powered by Sony's CXD5602
   microcontroller (ARM® Cortex®-M4F × 6 cores), with a clock speed of
   156 MHz.

**Driver Status:**

**NuttX-3.31**. In this release, many new architectural features,
peripheral drivers, and board configurations were contributed primarily
through the work of Alin Jerpelea. These new architectural features
include: Inter-core communications, power management, and clock
management. New drivers include: GPIO, PMIC, USB, SDHC, SPI, I2C, DMA,
RTC, PWM, Timers, Watchdog Timer, UID, SCU, ADC, eMMC, Camera CISIF,
GNSS, and others.

**NuttX-8.1**. Alin Jerpelea brought in ten (external) sensor drivers
that integrate through the CXD56xx's SCU.

**NuttX-8.2**. Masayuki Ishikawa implemented SMP operation of the
CX56Dxx parts. Alin Jerpelea: Added support for the Altair LTE modem
support, enabled support for accelerated format converter, rotation and
so on using the CXD5602 image processing accelerator, added ISX012
camera support, added audio and board audio control implementation,
added an audio_tone_generator, added optional initialization of GNSS and
GEOFENCE at boot if the drivers are enabled, added an lcd examples
configuration.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

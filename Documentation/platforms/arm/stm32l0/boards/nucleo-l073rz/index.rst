.. _nucleo-l073rz:

================
ST Nucleo L073RZ
================

.. tags:: chip:stm32, chip:stm32l0, chip:stm32l073

The Nucleo L073RZ is a member of the Nucleo-64 board family.

Serial Console
==============

The ``nsh`` and ``usb-cdc`` configurations use USART2 as the NSH console at
115200 8N1 through the ST-LINK virtual COM port.

USB Device
==========

The STM32L073RZ USB FS device interface uses PA11 for USB_DM and PA12 for
USB_DP.  The ``usb-cdc`` configuration uses the internal HSI48 oscillator as
the 48 MHz USB clock.  The clock recovery system synchronizes HSI48 from USB
start of frame packets, so no external 48 MHz crystal is required.

Configurations
==============

Each configuration is maintained in a sub-directory and selected with::

	tools/configure.sh nucleo-l073rz:<subdir>

nsh:
----
Configures the NuttShell (NSH) on USART2.

usb-cdc:
--------
Configures NSH on USART2 and exposes a separate USB CDC/ACM serial device on
the USB FS connector.

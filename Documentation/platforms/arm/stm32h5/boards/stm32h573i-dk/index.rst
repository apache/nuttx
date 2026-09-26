================
ST STM32H573I-DK
================

.. tags:: chip:stm32, chip:stm32h5, chip:stm32h573

Board Information
=================

The STM32H573I-DK Discovery board features the STM32H573II MCU, a
250 MHz Cortex-M33 with 2 MiB of flash memory and 640 KiB of SRAM.

Serial Console
==============

The console uses USART1 (PA9 TX, PA10 RX) through the ST-LINK virtual
COM port at 115200 baud, 8N1, with no flow control.

Flashing
========

Requires TrustZone disabled (``TZEN=0xC3``) and ``NSBOOTADD=0x80000``::

    STM32_Programmer_CLI -c port=SWD mode=UR -w nuttx.bin 0x08000000 -v -rst

Configurations
==============

nsh
---

Configures the NuttShell (NSH) with a serial console on USART1.

jumbo
-----

Enables additional build and runtime coverage with OS tests, user LEDs,
buttons, C++, libm, tmpfs, work queues and scheduler monitoring.
Run ``ostest``, ``leds`` or ``buttons`` from the NSH prompt.

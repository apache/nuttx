==========
ST STM32U3
==========

The STM32U3 family consists of ultra-low-power microcontrollers based on the
Arm Cortex-M33 core.  NuttX currently supports the STM32U3C5 with TrustZone
disabled.

Supported MCUs
==============

===========  =======  =====================
MCU          Support  Notes
===========  =======  =====================
STM32U3C5    Yes      2 MiB Flash, 640 KiB SRAM
===========  =======  =====================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  =============================================
Peripheral  Support  Notes
==========  =======  =============================================
RCC         Yes      Family-specific clock initialization.
GPIO        Yes      Common STM32U3/STM32U5 Cortex-M33 driver.
SYSCFG      Yes      GPIO EXTI routing.
EXTI        Yes      Common STM32U3/STM32U5 Cortex-M33 driver.
USART       Yes      USART1-3 and UART4-5.
LPUART      Yes      LPUART1.
==========  =======  =============================================

Other STM32U3 peripherals are not yet supported.

References
==========

[RM0487] STMicroelectronics, STM32U3 series Arm-based 32-bit MCUs.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

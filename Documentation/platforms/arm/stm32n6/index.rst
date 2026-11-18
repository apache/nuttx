==========
ST STM32N6
==========

This is a port of NuttX to the STM32N6 family.
The STM32N6 is a chip based on the Arm Cortex-M55.

Supported MCUs
==============

===========  ======= ================
MCU          Support Note
===========  ======= ================
STM32N645     No
STM32N647     No
STM32N655     No
STM32N657     Yes    STM32N657X0 only
===========  ======= ================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  ===============================================
Peripheral  Support  Notes
==========  =======  ===============================================
Cache       Yes      L1 I- and D-cache.
GPIO        Yes
I2C         Yes      I2C1-4 through the common I2C v2 driver.
LTDC        Yes      Common LTDC driver (v2 register layout), RGB565,
                     RGB888 and ARGB8888 layers.  The layer bus masters
                     are made Secure and privileged through the RIF.
EXTI        Yes      GPIO interrupts through the common driver.
ETH         Yes      ETH1 through the common Ethernet v2 driver, MII, RMII
                     or RGMII up to 1000 Mbps.
DMA         Yes      GPDMA1 and HPDMA1 through the common GPDMA driver.
PWR         Yes      Supply configuration, VOS, I/O supply validity.
RCC         Yes      HSI/HSE, PLL1-4, IC1-20, bus prescalers and kernel
                     clock selection from board.h.
RNG         Yes      Through the common driver.
RTC         Yes      Calendar, sub-seconds, Alarm A/B and wakeup timer
                     through the common RTC driver.
SDMMC       Yes      SDMMC1-2 through the common IDMA SDMMC driver.
SPI         Yes      SPI1-6 through the common SPI v5 driver, master
                     mode, optional GPDMA.
USART       Yes      USART1-3, UART4-5, USART6, UART7-9, USART10 and
                     LPUART1 through the common U(S)ART driver.

ADC         No
DCMIPP      No
IWDG        No
LPTIM       No
MPU         No
NPU         No
SAI         No
TIM         No
USB         No
XSPI        No
==========  =======  ===============================================

References
==========

[RM0486] STMicroelectronics, STM32N647/657xx Arm®-based 32-bit MCUs

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

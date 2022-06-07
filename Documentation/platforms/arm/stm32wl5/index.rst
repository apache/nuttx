========
STM32WL5
========

The STM32WL5 is a dual CPU (not core!) chip based on ARM Cortex-M4 and
Cortex-M0 with integrated sub-GHz radio for LoRa (G)FSK, (G)MSK and BPSK
modulations.

Only Cortex-M0 has access to radio peripheral. Pipe between CPUs exists
so that radio packets can be exchanged between CPUs. Chip was designed
so that Cortex-M0 cpu handles radio traffic while Cortex-M4 cpu handles
actions based on traffic received. All other peripherals are shared
(like uart, spi, i2c) and both CPUs can initiate them, but it's required
to be done only by one of them.

Supported MCUs
=============

STM32WL5 has only two chips in family. STM32WL55 and STM32WL54. Only
difference between them is that STM32WL55 has support for LoRa while
Stm32WL54 does not.

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  =====
Peripheral  Support  Notes
==========  =======  =====
IRQs        Yes
GPIO        Yes
EXTI        No
HSE         Yes
PLL         Yes      Tested @ 48MHz
HSI         Yes      Not tested
MSI         Yes      Not tested
LSE         Yes      Not tested
RCC         Yes      All registers defined, not all peripherals enabled
SYSCFG      Yes      All registers defined, remapping not tested
USART       Yes
LPUART      Yes      full speed with HSE works, low power mode with LSE not implemented
DMA         No
SRAM2       No
SPI         No
I2C         No
RTC         No
Timers      No
PM          No
AES         No
RNG         No
CRC         No
WWDG        No
IWDG        No
ADC         No
DAC         No
IPCC        No
Radio@CPU0  No
==========  =======  =====

PLL
---

PLL is a module that allows MCU to generate higher (or lower) clocks than
provided by the source. For example it can be used to drive system clock
with 48MHz when 8MHz HSE crystal is installed.

LSE
---

Low speed external crystal. Can be used to clock RTC and/or independent
watchdog (IWDG). LSE is usually 32768Hz high precision crystal.

HSI
---

High speed internal clock. Can be used as a source for sysclk and
internal buses (APB, AHB). This clock source is not as precise or
as stable as HSE, but it cuts down costs by avoiding external
hardware (crystal and capacitors) and is usually good enough if
operating temperatures are stable. It's clock is fixed at 16MHz.

MSI
---

Adjustable internal clock. Can be adjusted by software, but it's
accuracy and stability is even lower than HSI.

HSE
---

High speed external crystal. Can be used to clock sysclk and internal
buses (APB, AHB). External crystal is more precise and more stable
than HSI.

RCC
---

Reset and clock control. Enables or disables specific peripherals.

SYSCFG
------

System configuration controller. Can be used to remap memory or
manage external interrupts.

GPIO
----

Pins can be configured using :c:func:`stm32wl5_configgpio` function. Writing
to pins is done by :c:func:`stm32wl5_gpiowrite` function and reading is done
by :c:func:`stm32wl5_gpioread`.

UART
----

Universal Asynchronous Receiver/Transmitter module. UART is initialized
automatically during MCU boot.

IPCC
----

Inter-processor communication controller. IPCC is used to exchange data
between Cortex-M4 and Cortex-M0 CPUs.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*


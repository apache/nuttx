===========================
Silicon Labs EFR32 Series 2
===========================

The Silicon Labs EFR32 Series 2 are 2.4 GHz wireless SoCs built around an
Arm Cortex-M33 core. This port covers the EFR32xG24 family (EFR32MG24),
which has:

- Arm Cortex-M33 (ARMv8-M Mainline) at up to 78 MHz, FPU, DSP, TrustZone
- Up to 1536 KB flash and 256 KB RAM
- HFXO (39 MHz crystal), LFXO, HFRCODPLL with a DPLL, FSRCO, LFRCO
- USART0, EUSART0 and EUSART1, I2C0 and I2C1, TIMER0-4, IADC, LDMA
- Peripheral pins routed to any pin of ports A and B through the GPIO block
- 2.4 GHz radio (Bluetooth LE, 802.15.4)

The older EFM32 Series 0/1 parts are in ``arch/arm/src/efm32``. Their clock
controller, GPIO and USART register layouts differ, so the Series 2 parts
have their own directory, ``arch/arm/src/efr32``.

Supported chips
===============

======================== ========= ======== ======= =========
Chip                     Package   Flash    RAM     Radio
======================== ========= ======== ======= =========
EFR32MG24B210F1536IM48   QFN48     1536 KB  256 KB  +10 dBm
======================== ========= ======== ======= =========

Clocks
======

``efr32_clockconfig()`` starts the HFXO, loads the factory calibration of
the 80 MHz HFRCODPLL band and locks the HFRCODPLL to the HFXO with the
DPLL. The board gives the crystal frequency and the DPLL setting in
``board.h``:

============ =========== =================================================
Clock        Frequency   Notes
============ =========== =================================================
SYSCLK, HCLK 78 MHz      HFXO 39 MHz x (N + 1) / (M + 1), N = 3839, M = 1919
PCLK         39 MHz      HCLK / 2 (USART0)
RHCLK        39 MHz      SYSCLK / 2 (radio AHB, at most 40 MHz)
============ =========== =================================================

The flash runs with one wait state above 40 MHz. The HFXO tuning
capacitance comes from the factory value in the user data page when there is
one, otherwise from ``CONFIG_EFR32_HFXO_CTUNE`` (default 140).

NuttX runs in secure state and uses the secure aliases of the peripherals.

Peripheral support
==================

========== ======= ==========================================================
Peripheral Support Notes
========== ======= ==========================================================
GPIO       Yes     Input, output, pin interrupts on pins 0-11 (EXTI)
USART      Yes     USART0, asynchronous mode (console)
EUSART     No
I2C        No
SPI        No
TIMER      No      The system tick is the SysTick
LETIMER    No
SYSRTC     No
IADC       No
LDMA       No
MSC        No      Flash wait states only, no programming
WDOG       No
Radio      No
========== ======= ==========================================================

Supported boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

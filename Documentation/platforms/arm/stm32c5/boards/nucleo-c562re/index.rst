=================
ST NUCLEO-C562RE
=================

.. tags:: chip:stm32, chip:stm32c5, chip:stm32c562

The `NUCLEO-C562RE <https://www.st.com/en/evaluation-tools/nucleo-c562re.html>`_
is an STM32 Nucleo-64 board featuring an STM32C562RE microcontroller with
512 KiB of Flash and 128 KiB of SRAM.  The board includes an ST-LINK-V3EC
debugger and programmer with a virtual COM port.

Clocking
========

NuttX uses the board's 24 MHz HSE oscillator as the PSI reference and runs the
CPU, AHB, and APB buses at 144 MHz.  The board also has a 32.768 kHz LSE
oscillator.

Configurations
==============

nsh
---

Configures a small internal-flash NuttShell image with USART2 as the serial
console and the green status LED enabled.

jumbo
-----

Enables a broader set of NuttX features for additional build and runtime
coverage.

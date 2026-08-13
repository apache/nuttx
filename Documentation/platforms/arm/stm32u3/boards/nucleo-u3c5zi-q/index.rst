======================
ST NUCLEO-U3C5ZI-Q
======================

.. tags:: chip:stm32, chip:stm32u3, chip:stm32u3c5

The `NUCLEO-U3C5ZI-Q <https://www.st.com/en/evaluation-tools/nucleo-u3c5zi-q.html>`_
is an STM32 Nucleo-144 board featuring an STM32U3C5ZI microcontroller with
2 MiB of Flash and 640 KiB of SRAM.  The board includes an ST-LINK debugger
and programmer with a virtual COM port.

Clocking
========

NuttX runs at 96 MHz from the internal MSIRC0 oscillator.  The initial port
runs as a flat, nonsecure image with TrustZone disabled.

Configurations
==============

nsh
---

Configures the NuttShell with USART1 as the serial console.

jumbo
-----

Enables a broad set of NuttX features to provide additional build and runtime
coverage.

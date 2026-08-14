================
ST Nucleo-H503RB
================

.. tags:: chip:stm32, chip:stm32h5, chip:stm32h503

Board Information
=================

This page discusses issues unique to NuttX configurations for the
STMicro NUCLEO-H503RB development board featuring the STM32H503RB
MCU. The STM32H503RB is a 250MHz Cortex-M33 operation with 128KBytes Flash
memory and 32KByte SRAM.

Refer to the http://www.st.com website for further information about this
board (search keyword: NUCLEO-H503RB)

Configurations
==============

nsh
---

Configures the NuttShell (nsh) located at apps/examples/nsh. This
configuration enables a serial console on USART3 (Nucleo Virtual Console).

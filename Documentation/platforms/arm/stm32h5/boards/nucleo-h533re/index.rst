================
ST Nucleo-H533RE
================

.. tags:: arch:arm, chip:stm32h5, part:stm32h533, vendor:st

Board Information
=================

This page discusses issues unique to NuttX configurations for the
STMicro NUCLEO-H533RE development board featuring the STM32H533RE
MCU. The STM32H533RE is a 250MHz Cortex-M33 operation with 512KBytes Flash
memory and 272KByte SRAM.

Refer to the http://www.st.com website for further information about this
board (search keyword: NUCLEO-H533RE)

Configurations
==============

nsh
---

Configures the NuttShell (nsh) located at apps/examples/nsh. This
configuration enables a serial console on USART2 (Nucleo Virtual Console).

iks01a2
-------

NSH with an X-NUCLEO-IKS01A2 sensor shield on the Arduino connector
(I2C1).  The sensors are registered as uORB topics and can be read with
``sensortest``, the bus can be scanned with ``i2c dev``.

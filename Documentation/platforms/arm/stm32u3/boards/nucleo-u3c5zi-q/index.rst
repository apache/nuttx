======================
ST NUCLEO-U3C5ZI-Q
======================

.. tags:: arch:arm, chip:stm32u3, part:stm32u3c5, vendor:st

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

can
---

FDCAN1 as a character driver (``/dev/can0``) at 250 kbit/s on the onboard
CAN transceiver (CN18), with ``examples/can``.

cansock
-------

FDCAN1 as a SocketCAN device (``can0``) at 250 kbit/s, with ``candump``
and ``cansend``.

jumbo
-----

Enables a broad set of NuttX features to provide additional build and runtime
coverage.  DAC1 channel 1 (PA4) is registered as ``/dev/dac0`` and can be
driven with the ``dac`` example.

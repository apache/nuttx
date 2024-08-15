======================
Zynq UltraScale+ MPSoC
======================

The Zynq UltraScale+ MPSoC family consists of a system-on-chip (SoC) style integrated
processing system (PS) and a Programmable Logic (PL) unit, providing an extensible and
flexible SoC solution on a single die.There's 64-bit Quadcore ARM Cortex-A53 Processors
and Dualcore ARM Cortex-R5 Real-Time Processors in the MPSoC, zynq-mpsoc given support 
for Quadcore ARM Cortex-A53 Processors of MPSoC

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  ======= ===============
Peripheral  Support Notes
==========  ======= ===============
MIO         Yes
EMIO        Yes     Depending on PL
I2C         No
CAN         No
NET         No
SPI         No
QSPI        No
TIMER       NO
UART        Yes
WDT         No
DMA         No
SDI         No
ADC         No      Depending on PL
DAC         No      Depending on PL
PCI         NO      Depending on PL
==========  ======= ===============

MIO/EMIO
--------

Key features of the GPIO peripheral are summarized as follows:

- 78 GPIO interfaces to the device pins.
    - Routed through the MIO multiplexer.
    - Programmable I/O drive strength, slew rate, and 3-state control.
- 96 GPIO interfaces to the PL (four allocated by software to reset PL logic).
    - Routed through the EMIO interface.
    - Data inputs.
    - Data outputs.
    - Output enables.
- I/O interface is organized into six banks (3 MIO and 3 EMIO).

Pins can be configured/operated using ``zynq_mio_*`` functions. To handled 96 GPIO in 3
EMIO banks you should map GPIO to chip's pin by HDL design in PL logic.

UART
----

Zynq UltraScale+ MPSoC have two high-speed UARTs (up to 1Mb/s). The UART controller is
a full-duplex asynchronous receiver and transmitter that supports a wide range of
programmable baud rates and I/O signal formats. The controller can accommodate
automatic parity generation and multi-master detection mode this may introduce a large
number of interrupts which may be undesirable.

UART can be configured/operated using ``zynq_uart_*`` functions. Both receive and
transmit can be operated in interrupt mode and polling mode.

Psci and debug
--------------

Default exception level is EL1 for the NuttX OS. However, if we debug NuttX by JTAG
the XSCT of Vivado SDK will set the Zynq MPSoC to EL3. so have to config NuttX to run on
EL3. Other levels are not supported at the moment. And in this operating conditon
we can't use SMC for there's no ATF support.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

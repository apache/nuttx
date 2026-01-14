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
NET         Yes     GEM3
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

ETHERNET
--------

The gigabit Ethernet controller (GEM) implements a 10/100/1000 Mb/s Ethernet MAC that
is compatible with the IEEE Standard for Ethernet (IEEE Std 802.3-2008) and capable of
operating in either half or full-duplex mode in 10/100 mode and full-duplex in 1000 mode.
The processing system (PS) is equipped with four gigabit Ethernet controllers. Each
controller can be configured independently. Each controller uses a reduced gigabit media
independent interface (RGMII) v2.0. Each GEM controller provides management data
input/output (MDIO) interfaces for PHY management. Key features of the NET driver are
summarized as follows:

- Configurable MAC.
    - Configurable DMA receive buffer size.
    - Configurable transmit packet size.
    - Configurable TX and RX buffer number.
    - Preallocate or malloc TX and RX buffer.
- Configurable PHY.
    - MDIO phy read and write interface.
    - Configurable phy address.
    - PHY autonegotiation to determine speed and mode.
    - Board support phy initialize.
- Configurable ethernet 1,2,3,4.
- Configurable ethernet speed 10M,100M,1000M.

Access to the programmable logic (PL) is through the EMIO which provides the gigabit
media independent interface (GMII). Other Ethernet communications interfaces can be
created in the PL using the GMII available on the EMIO interface. GEM supports the serial
gigabit media-independent interface (SGMII, 1000BASE-SX, and 1000BASE-LX) at 1000
Mb/s using the PS-GTR interface.

Psci and debug
--------------

Default exception level is EL1 for the NuttX OS. However, if we debug NuttX by JTAG
the XSCT of Vivado SDK will set the Zynq MPSoC to EL3. so have to config NuttX to run on
EL3. Other levels are not supported at the moment. And in this operating condition
we can't use SMC for there's no ATF support.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

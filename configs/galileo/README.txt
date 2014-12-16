README.txt
==========

  This README.txt file discusses the port of NuttX to the Intel Galileo
  board.

  NOTE: There is no port for the Galileo in place as of this writing.  At this
  point in time, this README file is only a repository for pre-porting
  information.  It is not clear as of this writing whether there ever will
  be a port to the Galileo development board or not.

LEDs and Buttons
================

Serial Console
==============

  Galileo provides two options for a Serial Console:

  1) UART TTL (5V/3.3V) serial communication is available on Arduino
     digital 0 (pin 1, IO0) and digital 1 (pin 2, IO1).  The function of IO0
     and IO1 are controlled via a TS5A23159 2-channel 2:
     multiplexer/demultiplexer

     --- ---- --------- -------------------------------------
     PIN NAME SIGNAL    DESCRIPTION
     --- ---- --------- -------------------------------------
      1  IN1  IO0_MUX   Select NO1 or NC1 as IO on COM1 = IO0
      2  NO1  LVL_RXD   IN=L, IO0=RXD
      3  NC1  IO0_GPIO  IN=H, IO0=GPIO
     --- ---- --------- -------------------------------------
      5  IN2  IO1_MUX   Select NO1 or NC1 as IO on COM2 = IO1
      4  NO2  IO1_GPIO  IN=L, IO1=GPIO
      7  NC2  LVL_TXD   IN=H, IO2=TXD
     --- ---- --------- -------------------------------------

     The LVL_RXD/LVL_TXD driver from UART0_TXD/UART0_RXD via a TXS0108E
     voltage level translator that brings the which brings 1.2-3.6V signals
     to 165-5.5V.

  2) In addition, a second UART provides RS-232 support via a MAX3232 driver
     and is connected via a 3.5mm jack:  Sleeve=GND, RING=SERIAL1_RXD, and
     TIP=SERIAL1_TXD.

  UART1 may be convenient because of its built-in RS232 drivers.  But if you
  have a standard RS-232 shield, then UART0 may be the better choice.

Running from SRAM
=================

  The Host Bridge contains an interface to 512KB of on-chip, low latency,
  embedded SRAM (eSRAM). The eSRAM memory may be used as either 128 x 4KB
  pages, or in block mode as a single contiguous 512KB block page. The
  eSRAM pages may be mapped anywhere in the physical address space as a
  DRAM overlay.

  To map the eSRAM as a single 512KB block page, the register
  ESRAMPGCTRL_BLOCK is used. If any of the 4KB pages are already enabled,
  it is not possible to enable the block page.

  To map and enable the 512KB block page, the following steps should be
  followed:

    - Set ESRAMPGCTRL_BLOCK.BLOCK_PG_SYSTEM_ADDRESS_16MB to the required
      address value
    - Set ESRAMPGCTRL_BLOCK.BLOCK_ENABLE_PG to 1

  Once an eSRAM page is enabled, it is implicitly locked and any further
  configuration change attempts will fail.

Configurations
==============

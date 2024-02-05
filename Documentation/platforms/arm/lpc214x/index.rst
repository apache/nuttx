===========
NXP LPC214x
===========

Support is provided for the NXP LPC214x family of
processors. In particular, support is provided for (1) the mcu123.com
lpc214x evaluation board (LPC2148) and (1) the The0.net ZPA213X/4XPA
development board (with the The0.net UG-2864AMBAG01 OLED) This port also
used the GNU arm-nuttx-elf toolchain\* under Linux or Cygwin.

http://www.nxp.com/pip/LPC2141FBD64.html

The LPC2141/42/44/46/48 microcontrollers are based on a 16-bit/32-bit ARM7TDMI-S
CPU with real-time emulation and embedded trace support, that combine
microcontroller with embedded high-speed flash memory ranging from 32 kB to
512 kB. A 128-bit wide memory interface and a unique accelerator architecture
enable 32-bit code execution at the maximum clock rate. For critical code size
applications, the alternative 16-bit Thumb mode reduces code by more than 30 pct
with minimal performance penalty.

Due to their tiny size and low power consumption, LPC2141/42/44/46/48 are ideal
for applications where miniaturization is a key requirement, such as access
control and point-of-sale. Serial communications interfaces ranging from a USB 2.0
Full-speed device, multiple UARTs, SPI, SSP to I2C-bus and on-chip SRAM of 8 kB
up to 40 kB, make these devices very well suited for communication gateways and
protocol converters, soft modems, voice recognition and low end imaging, providing
both large buffer size and high processing power. Various 32-bit timers, single
or dual 10-bit ADC(s), 10-bit DAC, PWM channels and 45 fast GPIO lines with up
to nine edge or level sensitive external interrupt pins make these microcontrollers
suitable for industrial control and medical systems.

Features
========

* 16-bit/32-bit ARM7TDMI-S microcontroller in a tiny LQFP64 package.
* 8 kB to 40 kB of on-chip static RAM and 32 kB to 512 kB of on-chip flash memory.
  128-bit wide interface/accelerator enables high-speed 60 MHz operation.
* In-System Programming/In-Application Programming (ISP/IAP) via on-chip boot
  loader software. Single flash sector or full chip erase in 400 ms and programming
  of 256 B in 1 ms.
* EmbeddedICE RT and Embedded Trace interfaces offer real-time debugging with the
  on-chip RealMonitor software and high-speed tracing of instruction execution.
* USB 2.0 Full-speed compliant device controller with 2 kB of endpoint RAM. In addition,
  the LPC2146/48 provides 8 kB of on-chip RAM accessible to USB by DMA.
* One or two (LPC2141/42 vs. LPC2144/46/48) 10-bit ADCs provide a total of 6/14 analog
  inputs, with conversion times as low as 2.44 us per channel.
* Single 10-bit DAC provides variable analog output (LPC2142/44/46/48 only).
* Two 32-bit timers/external event counters (with four capture and four compare
  channels each), PWM unit (six outputs) and watchdog.
* Low power Real-Time Clock (RTC) with independent power and 32 kHz clock input.
* Multiple serial interfaces including two UARTs (16C550), two Fast I2C-bus (400
  kbit/s), SPI and SSP with buffering and variable data length capabilities.
* Vectored Interrupt Controller (VIC) with configurable priorities and vector addresses.
* Up to 45 of 5 V tolerant fast general purpose I/O pins in a tiny LQFP64 package.
* Up to 21 external interrupt pins available.
* 60 MHz maximum CPU clock available from programmable on-chip PLL with settling
  time of 100 us.
* On-chip integrated oscillator operates with an external crystal from 1 MHz to 25 MHz.
* Power saving modes include Idle and Power-down.
* Individual enable/disable of peripheral functions as well as peripheral clock scaling
  for additional power optimization.
* Processor wake-up from Power-down mode via external interrupt or BOD.
* Single power supply chip with POR and BOD circuits:
* CPU operating voltage range of 3.0 V to 3.6 V (3.3 V +- 10 pct) with 5 V tolerant
  I/O pads.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

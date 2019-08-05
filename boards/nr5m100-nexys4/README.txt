README
======

This README discusses issues unique to NuttX configurations for the
IQ-Analog NR5M100 FPGA implementation of a RISC-V core on the Digilent
Nexys4 FPGA board.

The port is currently very minimal, though additional support may be
added in the future to address more of the board peripherals supplied
on the FPGA board. Those peripherals include:

  Supported:
  - USB UART (console port)
  - 16 single color LEDs
  - 16 slide switch inputs
  - Two tri-color LEDs
  - 5 Joystick style pushbuttons
  - 16 GPIO pins on 2 of the PMOD expansion connectors

  Not supported:
  - VGA display port
  - 8 digit 7-segement display
  - SD card slot
  - SPI FLASH memory (shared with FPGA configuration data).
  - USB HID (single device) connector serviced by external PIC uC
  - Non-DDR (older version): 16 MB Cellular SRAM
  - DDR (newer version): 128 MB DDR2 SDRAM
  - Microphone
  - 10/100 Ethernet PHY
  - 3-Axis accelerometer
  - Temperature sensor

See http://store.digilentinc.com/nexys-4-ddr-artix-7-fpga-trainer-board-recommended-for-ece-curriculum/
or http://store.digilentinc.com/nexys-4-artix-7-fpga-trainer-board-limited-time-see-nexys4-ddr/
for more information about these boards.

Contents
========

  - NR5M100 Overview
  - Development Environment
  - GNU Toolchain Options
  - Debugger
  - IDEs
  - LEDs
  - PWM
  - UARTs
  - Timer Inputs/Outputs
  - FSMC SRAM
  - SSD1289
  - Mikroe-STM32F4-specific Configuration Options
  - Configurations

Development Environment
=======================
  The NR5M100 RISC-V core was designed as a low gate count / low performance micro controller
  for inclusion in an ASIC.  It is based on a Verilog RISC-V called picorv32, but has many
  additions beyond that baseline.  The design running on the Digilent Nexys4 FPGA is a
  validation platform for the core and is presented as an open source project.

  The reason NR5M100 is "low performance" is that it is a state machine based core (like the
  picorv32) and not a multi-stage pipeline core.  This means that it requires an average of
  4.5 clock cycles to execute each instruction.  On a multi-stage pipeline architecure, this
  average would be closer to 1 clock cycle per instruction (though a bit higher due to
  pipeline branch misses).  The tradeoff for lower performance is a simpler design.  There
  is a single memory bus interface for both instructions and data.  Multi-stage pipeline
  cores require a separate I and D bus with cache SRAM and an external memory cache controller,
  etc.  This in addition to the pipeline registers adds additional gate count.

  The nr5m100-nexys4 core runs at 83.333 Mhz which provides about 18 Mhz effective operating
  speed with the multi-clock per instruction architecture.  If you are looking for a higher
  performance platform, you should check out the PULP Platform ( http://www.pulp-platform.org ).
  That is an FPGA design with a 4-stage pipeline RISC-V core, though not currently supported
  by NuttX.  The NR5M100 project will likely pull in the RISC-V core from that design next,
  though this will probably not be available soon.  With a bit of work, it is possible to
  run the nr5m100-nexys4 core at 170 Mhz with a 6.5 clocks-per-instruction state machine.
  This would give an effective performance of about 26Mhz.

Development Environment
=======================

  Linux is the best choice for development, though Cygwin on Windows may work.
  The source has been built only using the GNU toolchain (see below) under a Linux
  environment.  Other toolchains will likely cause problems or not be available yet.

RISC-V GNU Toolchain
====================

  To compile the code, you must first build a RISC-V GNU Toolchain from the sources at
  https://github.com/riscv/riscv-gnu-toolchain.  I don't know of any sources for pre-compiled
  toolchains (though there may be some out there).

  To build this toolchain, follow these instructions (tested on Ubuntu 12.04):

  1. Create a working directory in your home folder:

     mkdir ~/riscv
     cd ~/riscv

  2. Clone the GNU source tree:

     git clone --recursive https://github.com/riscv/riscv-gnu-toolchain

  3. Ensure the following packages are installed:

     sudo apt-get install texinfo bison flex autoconf automake libgmp-dev libmpfr-dev libmpc-dev

  4. Configure and build the toolchain:

     cd riscv-gnu-toolchain
     ./configure --with-xlen=64 --with-arch=I --disable-float --disable-atomic --enable-multilib --prefix=~/riscv
     make -j4  (or -j8 based on how many cores you have)

  5. Setup your PATH environment variable to include the toolchain (you may want to add this to
     your shell login script, such as .bash_profile, etc.):

     export PATH=~/riscv/bin:$PATH

  Windows based toolchain
  -----------------------
  May be possible to compile the GNU toolchain described above using Cygwin, but havne't tried it.

Debugger
========
  The Debug Module within the NR5M100 RISC-V has been designed to work with the RISC-V gdb
  debugger interfaced with the SiFive implementation of OpenOCD.  The interface has been tested
  with a J-LINK JTAG probe connected to PMOD header B on the FPGA using an adapter board
  that I designed and fabbed at OSHPark.  I will update this README.txt file soon with a link
  to the shared project for anyone who wishes to build one.

  To build OpenOCD, perform the following:

  1. Ensure the proper packages are installed:

     sudo apt-get install autoconf automake libtool libusb-1.0-0-dev

  2. Download the latest OpenOCD sources from the SiFive github repo:

     cd ~/riscv
     git clone --recursive https://github.com/sifive/openocd.git

  3. Configure and build OpenOCD.  The x86_64 GCC compilers will give errors because of
     shadowed variable warnings, so diable the -Werror flag also:

     cd openocd
     sed -i 's/ -Werror//g' configure.ac
     ./bootstrap
     ./configure  --enable-jlink --enable-maintainer-mode --enable-ftdi --prefix=~/riscv CFLAGS=-g

  The configuration scripts for openocd and nr5m100-nexys4 have been provided in the
  nuttx/configs/nr5m100-nexys4/scripts directory.  They are configured to use a J-LINK JTAG
  probe and to search for the IQ-Analog (the company I work for) IDCODE and part number for
  the FPGA board (7a10 for Artix xc7a100 part on the Digilent Nexys4 board).  With FPGA
  source directly from the nr5m100 github site (to be provided), this ID will match the
  hardware.  If changes are made to the JEDEC ID and/or part number, then the nr5m100.cfg
  file will need to be modified with the proper CPUID value.

IDEs
====

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project.  While I haven't tried it as
  I am not an IDE guy, the team at SiFive have reported that they now have
  Eclipse working with the RISC-V gdb debugger.

  NOTE:  The notes below are taken from an ARM build of NuttX, not RISC-V, so
         they may or may not work.  Try it and see I suppose.

  Makefile Build
  --------------
  Under Eclipse, it is pretty easy to set up an "empty makefile project" and
  simply use the NuttX makefile to build the system.  That is almost for free
  under Linux.  Under Windows, you will need to set up the "Cygwin GCC" empty
  makefile project in order to work with Windows (Google for "Eclipse Cygwin" -
  there is a lot of help on the internet).

  Native Build
  ------------
  Here are a few tips before you start that effort:

  1) Select the toolchain that you will be using in your .config file
  2) Start the NuttX build at least one time from the Cygwin command line
     before trying to create your project.  This is necessary to create
     certain auto-generated files and directories that will be needed.
  3) Set up include pathes:  You will need include/, arch/risc-v/src/rv32im,
     arch/risc-v/src/common, arch/risc-v/src/nr5m100, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/risc-v/src/nr5m100/nr5_vectors.S.  With RIDE, I build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by RIDE.

LEDs
====

The Nexys4 board has 16 single-color LEDs onboard, as well as 2 tri-color LEDs.
These are supported using GPIO Ports A (16-single color) and B (tri-color).
Additionally the tri-color LEDs can be driven from the Timer 1 or 2 PWM output
signals.

PWM
===

The nr5m100-nexys4 design has PWM capabilities within the Timer 1 and Timer 2
modules.  These PWM signals can be muxed to the tri-color LEDs or to I/O
pins on one of the PMOD expansion headers.

UARTs
=====

The nr5m100-nexys4 design has an onboard USB-UART providing an RS-232 interface
via the same USB cable that is used to program the FPGA.  The core proivdes a
fixed 8-Data bit, 1 stop bit, no parity UART connected to this intrface.

UART PINS
---------

UART1
  RX      FPGA C4 (USB UART device)
  TX      FPGA D4 (USB UART device)

Default USART/UART Configuration
--------------------------------

UART1 is enabled in all configurations (see */defconfig).

Configurations
==============

Each nr5m100-nexys4 configuration is maintained in a sub-directory and
can be selected as follow:

    tools/configure.sh nr5m100-nexys4/<subdir>

Where <subdir> is one of the following:

  nsh
  ---
  This is an NSH example that uses UART1 as the console.  UART1 is connected
  to the USB UART bridge on the FPGA board.


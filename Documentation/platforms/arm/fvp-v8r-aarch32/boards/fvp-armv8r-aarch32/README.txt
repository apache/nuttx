README.txt
==========

This board configuration will use FVP_BaseR_AEMv8R to emulate
generic ARMv8-R (Cortex-R52) series hardware platform and
provides support for these devices:

 - GICv3 interrupt controllers for ARMv8-r
 - PL011 UART controller(FVP)

Contents
========
  - Getting Started
  - Status
  - Platform Features
  - References

Getting Started
===============

1. Compile Toolchain
   I recommend to use the docker based CI image
   <nuttx>/tools/ci/docker

2. Getting Armv8-R AEM FVP
   The Armv8-R AEM FVP is a free of charge Armv8-R Fixed Virtual Platform.
   It supports the latest Armv8-R feature set. we can get it from:
   https://developer.arm.com/downloads/-/arm-ecosystem-models

   Please select to download Armv8-R AEM FVP product, extract the tool package
   the FVP tool is locate at:
   <path_to>/AEMv8R_FVP/AEMv8R_base_pkg/models/Linux64_GCC-9.3/FVP_BaseR_AEMv8R
   Version 11.20 is tested fine.

3. Configuring and building
  3.1 FVP Overview
     Just like QEMU, Fixed Virtual Platforms (FVP) are complete simulations of an Arm system,
   including processor, memory and peripherals. These are set out in a "programmer's view",
   which gives you a comprehensive model on which to build and test your software.

     The FVP tools simulate 4 serial port and implement them to wait on local socket port:

     $ <path_to>/AEMv8R_FVP/AEMv8R_base_pkg/models/Linux64_GCC-9.3/FVP_BaseR_AEMv8R \
        -f boards/arm/fvp-v8r-aarch32/fvp-armv8r/scripts/fvp_cfg.txt -a ./nuttx
      terminal_0: Listening for serial connection on port 5000
      terminal_1: Listening for serial connection on port 5001
      terminal_2: Listening for serial connection on port 5002
      terminal_3: Listening for serial connection on port 5003

     FVP has four UART port and I choice UART1 as tty, so just telnet to port 5001
     will enter nsh:
     telnet localhost 5001

  3.2 Single Core
   Configuring NuttX and compile:
   $ ./tools/configure.sh -l fvp-armv8r-aarch32:nsh
   $ make

4. Running

  4.1 Single Core

  Step1: Booting NuttX

  $ AEMv8R_FVP/AEMv8R_base_pkg/models/Linux64_GCC-9.3/FVP_BaseR_AEMv8R \
           -f boards/arm/fvp-v8r-aarch32/fvp-armv8r-aarch32/scripts/fvp_cfg.txt \
           -a ./nuttx
    terminal_0: Listening for serial connection on port 5000
    terminal_1: Listening for serial connection on port 5001
    terminal_2: Listening for serial connection on port 5002
    terminal_3: Listening for serial connection on port 5003

  Step2: telnet to UART1
  Starting another terminal and enter:
  $ telnet localhost 5001
    Trying 127.0.0.1...
    Connected to localhost.
    Escape character is '^]'.
    nsh: mkfatfs: command not found
    NuttShell (NSH) NuttX-12.1.0
    nsh>


Status
======

2023-5-31:
1. Initial version for ARMv8-R AARCH32, Single Core, noMPU, noFPU, noCache using GCC Toolchain

Platform Features
=================

The following hardware features are supported:
+--------------+------------+----------------------+
| Interface    | Controller | Driver/Component     |
+==============+============+======================+
| GICv3        | on-chip    | interrupt controller |
+--------------+------------+----------------------+
| PL011 UART   | on-chip    | serial port          |
+--------------+------------+----------------------+
| ARM TIMER    | on-chip    | system clock         |
+--------------+------------+----------------------+


README.txt
==========

This board configuration will use FVP_BaseR_AEMv8R to emulate
generic ARM64v8-R (Cotex-R82) series hardware platform and
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
    The FVP platform using same Compiler like qemu, read the following
  file for How to get the Tool:
  https://github.com/apache/nuttx/tree/master/boards/arm64/qemu/qemu-armv8a

Note:
1. My host environment is Ubuntu 22.04.1 LTS, Ubuntu 18.04 will work too
2. The newest GNU toolchain is 12.2, available from:
   https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads


2. Getting Armv8-R AEM FVP
   The Armv8-R AEM FVP is a free of charge Armv8-R Fixed Virtual Platform.
   It supports the latest Armv8-R feature set. we can get it from:
   https://developer.arm.com/downloads/-/arm-ecosystem-models

   Please select to download Armv8-R AEM FVP product, extract the tool package
   the FVP tool is locate at:
   AEMv8R_FVP/AEMv8R_base_pkg/models/Linux64_GCC-9.3/FVP_BaseR_AEMv8R

3. Configuring and building
  3.1 Single Core
   Configuring NuttX and compile:
   $ ./tools/configure.sh -l fvp-armv8r:nsh
   $ make

  3.2 SMP
   Configuring NuttX and compile:
   $ ./tools/configure.sh -l fvp-armv8r:nsh_smp
   $ make

3. Running
  Single Core
  $ AEMv8R_FVP/AEMv8R_base_pkg/models/Linux64_GCC-9.3/FVP_BaseR_AEMv8R \
           -f boards/arm64/fvp-v8r/fvp-armv8r/scripts/fvp_cfg.txt \
           -a ./nuttx
  SMP
  $ AEMv8R_FVP/AEMv8R_base_pkg/models/Linux64_GCC-9.3/FVP_BaseR_AEMv8R \
           -f boards/arm64/fvp-v8r/fvp-armv8r/scripts/fvp_cfg_smp.txt \
           -a ./nuttx

Status
======

2023-2-18:
1. Release the frist version for ARMv8-R, Single Core and SMP is supported
   OS test is passed.

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

References
===========

1. (ID050815) ARM® Cortex®-A Series - Programmer’s Guide for ARMv8-A
2. (ID020222) Arm® Architecture Reference Manual - for A profile architecture
3. (ARM062-948681440-3280) Armv8-A Instruction Set Architecture
4. AArch64 Exception and Interrupt Handling
5. AArch64 Programmer's Guides Generic Timer
6. Arm Generic Interrupt Controller v3 and v4 Overview
7. Arm® Generic Interrupt Controller Architecture Specification GIC architecture version 3 and version 4
8. (DEN0022D.b) Arm Power State Coordination Interface Platform Design Document
9. Arm® Architecture Reference Manual Supplement, Armv8, for R-profile AArch64 architecture,
   ARM DDI 0600B.a (ID062922)
10.Arm® Cortex®-R82 Processor Technical Reference Manual, Revision: r0p2
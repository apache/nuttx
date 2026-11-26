===========
FVP ARMv8-r
===========

.. tags:: arch:arm64, chip:fvp-v8r, part:virt, vendor:arm

This board configuration will use FVP_BaseR_AEMv8R to emulate generic ARM64v8-R
(Cotex-R82) series hardware platform and provides support for these devices:

* GICv3 interrupt controllers for ARMv8-r
* PL011 UART controller(FVP)

Getting Started
===============

Compile Toolchain
-----------------

The FVP platform using same compiler like QEMU, read the following file for how
to get the tool: :doc:`/platforms/arm64/qemu/boards/qemu-armv8a/index`

.. note::

   1. My host environment is Ubuntu 22.04.1 LTS, Ubuntu 18.04 will work too

   2. The newest GNU toolchain is 12.2, available from:
      https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads


Getting Armv8-R AEM FVP
-----------------------

The Armv8-R AEM FVP is a free of charge Armv8-R Fixed Virtual Platform.
It supports the latest Armv8-R feature set. We can get it from:
https://developer.arm.com/downloads/-/arm-ecosystem-models

Please select to download Armv8-R AEM FVP product, extract the tool package the
FVP tool is located at:
``AEMv8R_FVP/AEMv8R_base_pkg/models/Linux64_GCC-9.3/FVP_BaseR_AEMv8R``

Configuring and building
------------------------

Just like QEMU, Fixed Virtual Platforms (FVP) are complete simulations of an Arm
system, including processor, memory and peripherals. These are set out in a
"programmer's view", which gives you a comprehensive model on which to build and
test your software.

 The FVP tools simulate 4 serial port and implement them to wait on local socket
 port:

 .. code:: console

    $ /home/qinwei/workdir/tools/AEMv8R_FVP/AEMv8R_base_pkg/models/Linux64_GCC-9.3/FVP_BaseR_AEMv8R \
       -f boards/arm64/fvp-v8r/fvp-armv8r/scripts/fvp_cfg.txt -a ./nuttx
     terminal_0: Listening for serial connection on port 5000
     terminal_1: Listening for serial connection on port 5001
     terminal_2: Listening for serial connection on port 5002
     terminal_3: Listening for serial connection on port 5003

 FVP has four UART port and UART1 was chosen as tty, so just telnet to port 5001
 will enter ``nsh``:

 .. code:: console

    $ telnet localhost 5001

-----------
Single Core
-----------

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh -l fvp-armv8r:nsh
   $ make

---
SMP
---

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh -l fvp-armv8r:nsh_smp
   $ make

---
BMP
---

The ``nsh_bmp`` configuration runs one NuttX instance on each of the four
Cortex-R82 cores using Bound Multiprocessing (BMP).  CPU 0 owns the physical
PL011 console and creates an RPTUN/RPMsg link to each of CPUs 1, 2, and 3.
Software-generated interrupts (SGIs) notify the peer cores, and the RPMsg UART
service provides a terminal for each secondary core.

Configuring NuttX and compiling:

.. code:: console

    $ ./tools/configure.sh -l fvp-armv8r:nsh_bmp
    $ make

The configuration enables the RPMsg ping utility and creates the following
CPU 0 devices after the links are ready:

.. list-table:: BMP RPMsg devices on CPU 0
   :header-rows: 1

   * - Remote CPU
     - RPTUN device
     - RPMsg UART device
   * - CPU 1
     - ``/dev/rpmsg/core1``
     - ``/dev/ttyCORE1``
   * - CPU 2
     - ``/dev/rpmsg/core2``
     - ``/dev/ttyCORE2``
   * - CPU 3
     - ``/dev/rpmsg/core3``
     - ``/dev/ttyCORE3``

Running
-------

-------------------
Running Single Core
-------------------

Step 1: Booting NuttX

.. code:: console

   $ AEMv8R_FVP/AEMv8R_base_pkg/models/Linux64_GCC-9.3/FVP_BaseR_AEMv8R \
            -f boards/arm64/fvp-v8r/fvp-armv8r/scripts/fvp_cfg.txt \
            -a ./nuttx
     terminal_0: Listening for serial connection on port 5000
     terminal_1: Listening for serial connection on port 5001
     terminal_2: Listening for serial connection on port 5002
     terminal_3: Listening for serial connection on port 5003
     - Ready to Boot Primary CPU
     - Boot from EL2
     - Boot from EL1
     - Boot to C runtime for OS Initialize
     [ 0] (null): arm64_chip_boot: Main CPU 0x80000000
     [ 0] (null): nx_start: Entry
     [ 0] (null): up_allocate_heap: heap_start=0x0x3c000, heap_size=0x7fc4000
     [ 0] Idle Task: gic_validate_dist_version: GICv3 version detect
     [ 0] Idle Task: gic_validate_dist_version: GICD_TYPER = 0x490067
     [ 0] Idle Task: gic_validate_dist_version: 224 SPIs implemented
     [ 0] Idle Task: gic_validate_dist_version: 0 Extended SPIs implemented
     [ 0] Idle Task: gic_validate_dist_version: Distributor has no Range Selector support
     [ 0] Idle Task: gic_validate_dist_version: MBIs is present, But No support
     [ 0] Idle Task: gic_validate_redist_version: GICR_TYPER = 0x0
     [ 0] Idle Task: gic_validate_redist_version: 16 PPIs implemented
     [ 0] Idle Task: gic_validate_redist_version: no VLPI support, no direct LPI support
     [ 0] Idle Task: up_timer_initialize: up_timer_initialize: cp15 timer(s) running at 100.00MHz, cycle 100000
     [ 0] Idle Task: uart_register: Registering /dev/console
     [ 0] Idle Task: uart_register: Registering /dev/ttyS0
     [ 0] Idle Task: work_start_highpri: Starting high-priority kernel worker thread(s)
     [ 0] Idle Task: nx_start_application: Starting init thread
     [ 0] Idle Task: task_spawn: name=nsh_main entry=0xa590 file_actions=0 attr=0x3bf88 argv=0x3bf80
     nsh: mkfatfs: command not found
     
     NuttShell (NSH) NuttX-12.0.0
     nsh> [ 0] Idle Task: nx_start: CPU0: Beginning Idle Loop

Step 2: telnet to UART1
Starting another terminal and enter:

.. code:: console

   $ telnet localhost 5001
     Trying 127.0.0.1...
     Connected to localhost.
     Escape character is '^]'.
     nsh: mkfatfs: command not found
     NuttShell (NSH) NuttX-12.0.0
     nsh>

-----------
Running SMP
-----------

.. code:: console

   $ AEMv8R_FVP/AEMv8R_base_pkg/models/Linux64_GCC-9.3/FVP_BaseR_AEMv8R \
            -f boards/arm64/fvp-v8r/fvp-armv8r/scripts/fvp_cfg_smp.txt \
            -a ./nuttx
     terminal_0: Listening for serial connection on port 5000
     terminal_1: Listening for serial connection on port 5001
     terminal_2: Listening for serial connection on port 5002
     terminal_3: Listening for serial connection on port 5003
     - Ready to Boot Primary CPU
     - Boot from EL2
     - Boot from EL1
     - Boot to C runtime for OS Initialize
     [CPU0] [ 0] (null): arm64_chip_boot: Main CPU 0x80000000
     [CPU0] [ 0] (null): nx_start: Entry
     [CPU0] [ 0] (null): up_allocate_heap: heap_start=0x0x4a000, heap_size=0x7fb6000
     [CPU0] [ 0] CPU0 IDLE: gic_validate_dist_version: GICv3 version detect
     [CPU0] [ 0] CPU0 IDLE: gic_validate_dist_version: GICD_TYPER = 0x490067
     [CPU0] [ 0] CPU0 IDLE: gic_validate_dist_version: 224 SPIs implemented
     [CPU0] [ 0] CPU0 IDLE: gic_validate_dist_version: 0 Extended SPIs implemented
     [CPU0] [ 0] CPU0 IDLE: gic_validate_dist_version: Distributor has no Range Selector support
     [CPU0] [ 0] CPU0 IDLE: gic_validate_dist_version: MBIs is present, But No support
     [CPU0] [ 0] CPU0 IDLE: gic_validate_redist_version: GICR_TYPER = 0x0
     [CPU0] [ 0] CPU0 IDLE: gic_validate_redist_version: 16 PPIs implemented
     [CPU0] [ 0] CPU0 IDLE: gic_validate_redist_version: no VLPI support, no direct LPI support
     [CPU0] [ 0] CPU0 IDLE: up_timer_initialize: up_timer_initialize: cp15 timer(s) running at 100.00MHz, cycle 100000
     [CPU0] [ 0] CPU0 IDLE: uart_register: Registering /dev/console
     [CPU0] [ 0] CPU0 IDLE: uart_register: Registering /dev/ttyS0
     - Ready to Boot Second CPU
     - Boot from EL2
     - Boot from EL1
     - Boot to C runtime for OS Initialize
     [CPU1] [ 1] CPU1 IDLE: gic_validate_redist_version: GICR_TYPER = 0x100000100
     [CPU1] [ 1] CPU1 IDLE: gic_validate_redist_version: 16 PPIs implemented
     [CPU1] [ 1] CPU1 IDLE: gic_validate_redist_version: no VLPI support, no direct LPI support
     [CPU1] [ 1] CPU1 IDLE: nx_idle_trampoline: CPU1: Beginning Idle Loop
     - Ready to Boot Second CPU
     - Boot from EL2
     - Boot from EL1
     - Boot to C runtime for OS Initialize
     [CPU2] [ 2] CPU2 IDLE: gic_validate_redist_version: GICR_TYPER = 0x200000200
     [CPU2] [ 2] CPU2 IDLE: gic_validate_redist_version: 16 PPIs implemented
     [CPU2] [ 2] CPU2 IDLE: gic_validate_redist_version: no VLPI support, no direct LPI support
     [CPU2] [ 2] CPU2 IDLE: nx_idle_trampoline: CPU2: Beginning Idle Loop
     - Ready to Boot Second CPU
     - Boot from EL2
     - Boot from EL1
     - Boot to C runtime for OS Initialize
     [CPU3] [ 3] CPU3 IDLE: gic_validate_redist_version: GICR_TYPER = 0x300000310
     [CPU3] [ 3] CPU3 IDLE: gic_validate_redist_version: 16 PPIs implemented
     [CPU3] [ 3] CPU3 IDLE: gic_validate_redist_version: no VLPI support, no direct LPI support
     [CPU3] [ 3] CPU3 IDLE: nx_idle_trampoline: CPU3: Beginning Idle Loop
     [CPU0] [ 0] CPU0 IDLE: work_start_highpri: Starting high-priority kernel worker thread(s)
     [CPU0] [ 0] CPU0 IDLE: nx_start_application: Starting init thread
     [CPU0] [ 0] CPU0 IDLE: task_spawn: name=nsh_main entry=0xc41c file_actions=0 attr=0x43f68 argv=0x43f60
     [CPU0] [ 0] CPU0 IDLE: nx_start: CPU0: Beginning Idle Loop
     nsh: mkfatfs: command not found
     
     NuttShell (NSH) NuttX-12.0.0
     nsh>

Step 2: telnet to UART1

Starting another terminal and enter:

.. code:: console

   $ telnet localhost 5001
     Trying 127.0.0.1...
     Connected to localhost.
     Escape character is '^]'.
     nsh: mkfatfs: command not found
     NuttShell (NSH) NuttX-12.0.0
     nsh>

-----------
Running BMP
-----------

Run the BMP image with the four-core FVP configuration:

.. code:: console

   $ AEMv8R_FVP/AEMv8R_base_pkg/models/Linux64_GCC-9.3/FVP_BaseR_AEMv8R \
            -f boards/arm64/fvp-v8r/fvp-armv8r/scripts/fvp_cfg_smp.txt \
            -a ./nuttx

Connect to UART1 for the CPU 0 NSH console:

.. code:: console

   $ telnet localhost 5001

From the CPU 0 NSH console, open the CPU 1 RPMsg UART.  The default device for
the ``cu`` command in this configuration is ``/dev/ttyCORE1``:

.. code:: console

   nsh> cu

Use ``~.`` at the beginning of a line to leave ``cu``.  To connect to another
secondary CPU, specify its RPMsg UART device explicitly:

.. code:: console

   nsh> cu -l /dev/ttyCORE2
   nsh> cu -l /dev/ttyCORE3

The RPMsg link can also be checked from CPU 0 with the ping service.  For
example, the following command sends 100 messages to CPU 1, waits for each
acknowledgment, and uses a 10 ms interval:

.. code:: console

   nsh> rpmsg ping /dev/rpmsg/core1 100 100 1 10

See :doc:`/os/drivers/special/rpmsg/rpmsg_ping` for the complete command syntax
and :doc:`/os/drivers/special/rptun/index` for an overview of RPTUN.

Status
======

**2023-2-18:**

Release the first version for ARMv8-R, Single Core and SMP is supported OS test
is passed.

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
10. Arm® Cortex®-R82 Processor Technical Reference Manual, Revision: r0p2

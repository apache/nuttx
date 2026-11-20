==============
Renesas RZ/V2H
==============

The RZ/V2H is a Renesas AI-accelerated multicore MPU built around a
heterogeneous set of Arm cores: dual Cortex-A55 (application cores) plus
dual Cortex-R8 (real-time cores), alongside a DSP and an AI-MAC (DRP-AI3)
accelerator.

.. image:: RZV2H-block-chart-DRP-AI3.png
   :alt: RZV2H block chart DRP-AI3
   :width: 100%
   :align: center

Supported MCUs
==============

The following list includes MCUs from the RZ/V2H series and indicates
whether they are supported in NuttX:

=============  ======= ================
MCU            Support Note
=============  ======= ================
R9A09G057      Yes     CR8_0 core only
=============  ======= ================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX on the CR8_0
core:

==========  =======  ============================================================
Peripheral  Support  Notes
==========  =======  ============================================================
CLOCK       Yes      BSP clock tree init (PLL), internal use only
GIC         Yes      Interrupt controller init (CPU0 + per-CPU)
MPU         Yes      Static PMSAv7 region table (ITCM/DTCM/SRAM/code)
TIMER       Yes      MPCore Private Timer used as the system tick source
SCI         Yes      Interrupt-driven SCI-B asynchronous UART on channels 0-9
GPIO        Yes      Basic operations (config/read/write), no interrupt support
OpenAMP     Yes      RPTUN/RPMsg transport using shared memory and MHU-B (Mailbox)
==========  =======  ============================================================

CLOCK
-----

``rzv2h_clockconfig()`` calls into ``bsp_clock_init()`` from the vendored
Renesas FSP BSP to program the PLL and clock tree. It is invoked once
during boot and is not user-configurable through Kconfig.

GIC
---

The Generic Interrupt Controller is brought up by ``up_irqinitialize()``,
using NuttX's common ARM GIC support: CPU0-only initialization followed by
per-CPU initialization.

MPU
---

A static PMSAv7 region table protects the CR8_0 memory map (ITCM, DTCM,
SRAM and the executable code window). Regions are set up once at boot and
are not reconfigurable at runtime.

TIMER
-----

The Arm MPCore Private Timer is used as the system tick source, clocked
from the BSP clock tree and reloaded to match ``CLK_TCK``.

SCI
---

The Serial Communication Interface (SCI) provides multiple serial
communication modes. NuttX currently supports interrupt-driven UART operation
on channels 0 through 9 through the SCI-B lower-half driver.
``CONFIG_RZV2H_UART_SCI`` enables the driver and
``CONFIG_RZV2H_SCI_B_UARTn`` selects each required channel. The standard
``CONFIG_SCIn_*`` options configure the channel and select the console.

The driver supports early console output and registers the selected
console as ``/dev/console`` and ``/dev/ttyS0``. Other enabled channels are
registered in ascending channel order.

Runtime baud-rate changes are supported through the standard termios
``TCGETS`` and ``TCSETS`` operations.

Hardware flow control and DMA are not implemented yet.

GPIO
----

The GPIO driver provides fundamental support for pin configuration, read,
and write operations. GPIO interrupt support is currently unavailable.

INTC SEL (Interrupt Select) Lines
---------------------------------

The CR8 core's INTC provides a
pool of 127 programmable "SEL" lines (SEL0-SEL126, GIC IRQ numbers
353-479) whose peripheral event mapping is set through the INTC INTSEL
registers. Peripheral drivers that need one of these programmable lines
request a specific SEL number and event through
``rzv2h_intsel_connect_event()``.

Default SEL IDs
^^^^^^^^^^^^^^^

The 127-line SEL pool (SEL0-SEL126, IRQ 353-479) is conventionally
partitioned by module, allocated from the top of the range downward:

======  ==================  ================================
Module  Count                IRQ range (>= 353)
======  ==================  ================================
TINT    32 (tint0-tint31)    479-448
======  ==================  ================================

OpenAMP
-------

NuttX on CR8_0 can communicate with Linux on CA55 through the generic
RPTUN/OpenAMP RPMsg stack. Linux is the VirtIO master and NuttX is the
device. Shared memory holds the resource table, vrings and message
buffers; the RZ/V2H lower half uses MHU-B (Mailbox) for notifications.

``CONFIG_RZV2H_RPTUN`` enables the transport. The current EVK and RDK
``nsh-openamp`` configurations select channel 1 and provide the
``rpmsg-service-1`` echo endpoint for a compatible Renesas Multi-OS
v3.3.0 Linux peer. NuttX activates one channel at a time, and the
Linux peer must use matching memory reservations and mailbox routing.

Boot and test procedures are board-specific. The EVK and RDK configuration
generates single uboot images for U-Boot. Neither documented flow uses Linux
kernel RemoteProc to load NuttX. See the
:doc:`EVK board guide </platforms/arm/rzv2h/boards/rzv2h-evk/index>`
and :doc:`RDK board guide </platforms/arm/rzv2h/boards/rzv2h-rdk/index>`
for configuration, boot, Linux peer requirements and RPMsg testing.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

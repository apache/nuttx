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
QSPI        Yes      xSPI controller in QSPI mode, manual-command transfers only
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

QSPI
----

``rzv2h_qspi.c`` implements the standard NuttX ``struct qspi_dev_s``
interface (``arch/arm/src/rzv2h/rzv2h_qspi.c``) on top of the Renesas FSP
xSPI-QSPI HAL (``r_xspi_qspi``). It is intended to drive a SPI NOR flash
device wired to the xSPI controller, using the NuttX QSPI-based MTD flash
drivers (``n25qxxx``, ``mx25rxx``, ``w25qxxxjv``, ``gd25``, ``gd55``,
``s25fl1``).

Commands and memory reads/writes are issued through
``R_XSPI_QSPI_DirectTransfer()`` (manual-command mode); each transaction
moves at most 8 bytes, so larger transfers are split into a sequence of
transactions by the driver. There is no XIP (execute-in-place,
memory-mapped) support and no DMA: all data moves through the CPU, so
throughput is limited compared to a memory-mapped or DMA-driven transfer.

Protocol support
^^^^^^^^^^^^^^^^

The ``n25qxxx`` MTD driver always requests Quad I/O Fast Read (``0xEB``).
``rzv2h_qspi_memory()`` sends that read on the wire in whichever protocol
is selected by the ``CONFIG_RZV2H_<BOARD>_QSPI_N25QXXX_PROTOCOL_*`` choice
(``<BOARD>`` is ``EVK`` or ``RDK``).
Program, erase, and status/configuration-register commands always use
``1S-1S-1S``, whatever read protocol is selected.

.. list-table::
   :header-rows: 1

   * - Protocol
     - Kconfig option
     - Read command
     - Status
   * - ``1S-1S-1S``
     - ``CONFIG_RZV2H_<BOARD>_QSPI_N25QXXX_PROTOCOL_1S1S1S``
     - READ (``0x03``)
     - Supported
   * - ``1S-2S-2S``
     - ``CONFIG_RZV2H_<BOARD>_QSPI_N25QXXX_PROTOCOL_1S2S2S``
     - Dual I/O Fast Read (``0xBB``)
     - Supported
   * - ``1S-4S-4S``
     - ``CONFIG_RZV2H_<BOARD>_QSPI_N25QXXX_PROTOCOL_1S4S4S``
     - Quad I/O Fast Read (``0xEB``)
     - Supported (**default**)
   * - ``4S-4S-4S``
     - ``CONFIG_RZV2H_<BOARD>_QSPI_N25QXXX_PROTOCOL_4S4S4S``
     - FAST READ (``0x0B``) in persistent Quad SPI mode (``0x35``/``0xF5``)
     - Supported
   * - ``4S-4D-4D``
     - ``CONFIG_RZV2H_<BOARD>_QSPI_N25QXXX_PROTOCOL_4S4D4D``
     - DTR FAST READ (``0x0D``) in persistent Quad SPI mode
     - Needs a board that connects ``XSPI0_DS`` to the flash (see below)
   * - ``8D-8D-8D``
     - none
     - none
     - Not supported

``2S-2S-2S`` has no Kconfig read option. The driver still sends it for
commands that set the dual-instruction flags (``QSPICMD_IDUAL`` or
``QSPIMEM_IDUAL``).

DDR protocols (``4S-4D-4D`` and ``8D-8D-8D``) sample read data with the
``XSPI0_DS`` strobe. The hardware manual states: "DDR access without
XSPI0_DS is not supported". A board must connect ``XSPI0_DS`` to a flash
that drives it before ``4S-4D-4D`` can work.

``8D-8D-8D`` is not supported for four reasons:

- The FSP xSPI-QSPI HAL rejects it: ``R_XSPI_QSPI_SpiProtocolSet()``
  returns ``FSP_ERR_INVALID_ARGUMENT``. The ``SOPI`` and ``DOPI`` modes
  are rejected the same way.
- It needs ``XSPI0_IO4``-``XSPI0_IO7`` and ``XSPI0_DS`` connected to an
  octal flash.
- The Quad SPI NOR flash on the RZV2H boards has only 4 data lines.
- The generic NuttX QSPI interface (``include/nuttx/spi/qspi.h``) has no
  flags for octal or DDR transfers.

The xSPI serial clock is derived from a dedicated CPG divider that only
feeds this peripheral (four ratios: source clock divided by 2, 4, 8, or
16); ``rzv2h_qspi_setfrequency()`` picks the closest ratio that does not
exceed the requested frequency.

See :doc:`the RZV2H-EVK board documentation <boards/rzv2h-evk/index>` for
board-level bring-up, partitioning, and configuration details.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

==================
NXP FRDM-IMXRT1186
==================

.. tags:: chip:imxrt, chip:imxrt1186, arch:arm, vendor:nxp, experimental

`FRDM-IMXRT1186 <https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-IMXRT1186>`_
is an NXP Freedom development board for the MIMXRT1186 crossover MCU
(Cortex-M33 + Cortex-M7) in the 12 x 12 mm, 196-pin BGA package.

This board is distinct from the MIMXRT1180-EVK, which uses MIMXRT1189 silicon
in a 289-pin package.  Board pinmux definitions are not interchangeable.

This NuttX port targets the Cortex-M33.  Cortex-M7 support is not started.

Features
========

* MIMXRT1186xxxxB (Cortex-M33 + Cortex-M7)
* On-chip ITCM / DTCM / OCRAM
* 16 MiB QSPI NOR on FlexSPI2
* MCU-Link on-board debug probe with virtual COM (LPUART1)
* USB OTG1 Type-C connector **J63**
* Dual YT8531 RGMII PHYs on the RT118x NETC switch
* User RGB LED and SW4 (drivers not yet included)

.. warning::

   This board support is experimental.  Clock bring-up depends on ELE
   transferring AON/MEGA/WAKEUP TRDC ownership before SYS_PLL3 lock.
   LED and button drivers are not included.  Ethernet uses a polling
   data path (MSI-X deferred).

.. note::

   Do not reuse MIMXRT1180-EVK pinmux or boot containers on this Freedom
   board.  The package, flash layout, and PHY wiring differ.

Buttons and LEDs
================

The FRDM schematic maps:

* RGB LED red — ``GPIO2_IO09``
* RGB LED green — ``GPIO2_IO11``
* RGB LED blue — ``GPIO3_IO07``
* User button SW4 — ``GPIO4_IO12``

NuttX does not yet provide LED or button drivers for these pins.

Pin Mapping
===========

Default console and commonly used board pins:

============= ============ =========================================
Pin / pad     Signal       Notes
============= ============ =========================================
GPIO_AON_08   LPUART1_TX   MCU-Link VCOM TX (``nsh`` / ``nsh-xip``)
GPIO_AON_09   LPUART1_RX   MCU-Link VCOM RX
J63           USB OTG1     CDC/ACM console for ``usbnsh``
J23           MCU-Link     CMSIS-DAP + VCOM; not the ``usbnsh`` port
============= ============ =========================================

Ethernet RGMII/MDIO pinmux for the dual YT8531 PHYs is configured by the
``netnsh`` / ``netnsh-xip`` board bring-up path.

Clock Bring-up
==============

Early startup keeps the Cortex-M33 and LPUART1 on the always-available
24 MHz RC oscillator.  NuttX then confirms that ELE responds, transfers
AON/MEGA/WAKEUP TRDC ownership to the Cortex-M33, enables the PLL LDO,
and initializes fixed 480 MHz SYS_PLL3.  The Cortex-M33 runs at 240 MHz
from SYS_PLL3; LPUART1 retains a 24 MHz input.  That SYS_PLL3 path is
sufficient for CM33 NSH and USB CDC console; those configurations do not
require ARM_PLL or SYS_PLL2.  Startup fails closed if ownership or PLL
lock cannot be established.  Failure and completed-stage masks are
available in ``g_imxrt118x_clock_error`` and ``g_imxrt118x_clock_status``.

When ``CONFIG_IMXRT_NETC`` is enabled, bring-up also initializes SYS_PLL1
(the Ethernet PLL) and the NETC/MAC roots.  ARM_PLL (higher or alternate
CPU rates, and eventual CM7), SYS_PLL2 (broader peripheral root options),
and the audio PLL are left for follow-up work when a consumer selects
those roots; enabling them early adds ELE/PLL sequencing surface without
improving NSH or USB.

ROM/LinkServer can leave ARMv8-M ``MSPLIM``/``PSPLIM`` enabled; the
reset path clears both before C startup.

Networking
==========

``netnsh`` and ``netnsh-xip`` enable RT118x NETC Ethernet through ENETC1
and the on-chip switch, with YT8531 PHY discovery and DHCP.  The DHCP
client waits for PHY link before starting.  The data path currently
polls descriptor rings.

USB Console
===========

``usbnsh`` runs the system console and NSH over USB CDC/ACM on J63
(VID:PID ``0525:a4a7``, Linux ``cdc_acm``).  After the host opens the
CDC tty, send three carriage returns to start the NSH session.

Power Supply
============

The board is normally powered from USB through the MCU-Link / hub path
used in the lab.  For true FlexSPI2 XIP power-on reset tests, leave
ISP_CTRL undriven (``wirebootconfig xxxx`` on LinkServer) and cycle the
board supply (for example with ``uhubctl``).  Do not confuse a later
debugger attach (which may enter Boot ROM / SDP) with POR proof; the
serial ``nsh>`` prompt after a hub cycle is the POR evidence.

Installation
============

* Arm GNU Toolchain (lab builds used Arm GNU 13.2.1)
* NXP LinkServer / MCU-Link tools for CMSIS-DAP load and FlexSPI program
* Optional: ``uhubctl`` for controlled USB hub power cycles during XIP POR

Building NuttX
==============

Configure from the NuttX top-level tree:

.. code:: console

   $ ./tools/configure.sh frdm-imxrt1186:nsh
   $ make -j$(nproc)

CMake is also supported.  Replace ``nsh`` with ``nsh-xip``, ``usbnsh``,
``netnsh``, or ``netnsh-xip`` as needed.

Flashing
========

``nsh`` / ``netnsh`` (debugger RAM-load)
----------------------------------------

1. Build the configuration.
2. Load the ELF/HEX with LinkServer into the ITCM/OCRAM layout
   (executable sections at the ITCM alias ``0x0ffe0000``; writable
   sections, stack, and heap at the OCRAM alias ``0x20480000``).
3. Attach a serial terminal to the MCU-Link VCOM at 115200 8N1.

``nsh-xip`` / ``netnsh-xip`` (FlexSPI2 XIP)
-------------------------------------------

1. Build the XIP configuration (image in the secure AHB window at
   ``0x14000000``, including FCB, XMCD, Boot ROM container, then text).
2. Program the Intel HEX/binary into FlexSPI2 with LinkServer.
3. Leave ISP_CTRL undriven (``wirebootconfig xxxx``).
4. Power-cycle the board without attaching gdb on the boot path.
5. Attach to MCU-Link VCOM at 115200 8N1 and confirm ``nsh>``.

``usbnsh``
----------

1. Build and RAM-load (or flash, if using an XIP USB image later).
2. Connect a host cable to **J63**.
3. Open the new CDC ACM tty and send three carriage returns.

Configurations
==============

Board name for ``tools/configure.sh`` is ``frdm-imxrt1186``.

.. code:: console

   $ ./tools/configure.sh frdm-imxrt1186:<config>

nsh
---

Minimal NuttShell (``examples/nsh``) on the Cortex-M33 using the
ITCM/OCRAM debugger-load layout.  Console is LPUART1 via MCU-Link VCOM
at 115200 8N1.  Networking is not enabled.

nsh-xip
-------

Same NSH feature set as ``nsh``, linked for FlexSPI2 XIP at
``0x14000000``.  Console remains LPUART1 via MCU-Link VCOM.

usbnsh
------

System console and NSH over USB CDC/ACM on J63.  Networking is not
enabled.

netnsh
------

NSH with RT118x NETC Ethernet (ENETC1 + switch path, YT8531 PHYs) using
the ITCM/OCRAM debugger-load layout.  DHCP waits for PHY link.

netnsh-xip
----------

Same networking feature set as ``netnsh``, linked for FlexSPI2 XIP at
``0x14000000``.

Testing
========

Hardware validation results, test methods, limitations, and retained serial
logs are recorded in :doc:`testing`.

.. toctree::
   :hidden:

   testing

Implementation Status
=====================

The table distinguishes code included by this board port from hardware
verification completed on the FRDM-IMXRT1186.

.. list-table::
   :header-rows: 1
   :widths: 18 30 25 27

   * - Area
     - Implemented
     - Tested on FRDM-IMXRT1186
     - Remaining
   * - CM33 bring-up / NSH
     - ``ARCH_CHIP_MIMXRT1186``, ARMv8-M, stack-limit reset, LPUART1
     - LinkServer RAM load, ``nsh>``, ``help``, and ``uname -a``
     - None
   * - Clock tree / CCM
     - SYS_PLL3 at 480 MHz, M33 at 240 MHz, LPUART at 24 MHz, ELE fail-closed; SYS_PLL1 under ``CONFIG_IMXRT_NETC``
     - Clock status ``0x7f`` with error ``0``; NSH/USB on SYS_PLL3 only
     - ARM_PLL, SYS_PLL2, and audio PLL deferred until a root consumer needs them
   * - ELE / TRDC
     - AON, MEGA, and WAKEUP ownership transfer
     - Covered by clock smoke test
     - Full TRDC policy and ELE active-timer ping
   * - SysTick
     - Processor-clock SysTick
     - ``uptime`` advances at approximately one second per second
     - GPT and LPIT exercise
   * - Memory map
     - ITCM ``0x0ffe0000`` and OCRAM ``0x20480000`` RAM image
     - Passed debugger RAM-load test
     - DTCM is unused
   * - FlexSPI2 / XIP
     - ``nsh-xip`` and ``netnsh-xip`` at ``0x14000000``
     - Passed true power-on-reset boot
     - ISP must be left undriven for power-on reset
   * - USB device
     - OTG1 CDC/ACM console on J63
     - ``0525:a4a7``, ``nsh>``, and ``uname -a``
     - None
   * - Ethernet NETC
     - ENETC1, switch/IERB/VLAN, YT8531 MDIO, polling data path, DHCP
     - PHY discovery, 1-Gbit link, and DHCP lease
     - MSI-X and final bidirectional ping verification
   * - LEDs / button
     - Pin mapping only
     - Not tested by this port
     - Drivers and polarity verification
   * - Dual-core CM7
     - Not implemented
     - Not tested
     - Deferred

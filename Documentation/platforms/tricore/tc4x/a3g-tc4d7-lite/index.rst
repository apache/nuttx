================================
KIT_A3G_TC4D7_LITE (AURIX TC4D7)
================================

Infineon's AURIX TC4D7 Lite Kit (KIT_A3G_TC4D7_LITE) is the
introductory evaluation platform for the TriCore TC4X (TC1.8P) family.

Features
========

- Infineon AURIX TC4D7 - 4 TriCore CPUs, 10 MiB PFLASH, 5 MiB LMU.
- ASCLIN0 routed to the on-board USB-serial bridge (115200-8-N-1) for
  the NSH console.
- iSYSTEM DAP-Miniwiggler header for flash and debug.
- Single USB-C port for power, console and debug.

Default configuration
=====================

::

  ./tools/configure.sh -E a3g-tc4d7-lite:nsh
  make -j$(nproc)

The default toolchain is ``tricore-elf-gcc``; the HighTec LLVM
toolchain can be selected via ``CONFIG_TRICORE_TOOLCHAIN_LLVM=y`` and
requires a valid HighTec licence reachable through ``RLM_LICENSE``.

Console
=======

The NSH prompt is exposed on UART0 (ASCLIN0) at 115200-8-N-1 over the
USB virtual COM port.

Flashing
========

The board's ``scripts/Make.defs`` provides a ``flash`` target that
drives OpenOCD with the Infineon TAS server.  The prerequisites are
documented in :doc:`../../index` (Flashing and debugging).

.. code-block:: console

   # Once-per-session: start the TAS server (listens on 24817)
   cd <path-to-DAS>/bin && LD_LIBRARY_PATH=. ./tas_server &

   # Flash the kit
   make flash

Override knobs (all set with ``?=`` in ``Make.defs``):

============================== =======================================
``OPENOCD``                    Path to the patched openocd binary.
``OPENOCD_SCRIPTS``            Directory holding the openocd scripts.
``OPENOCD_BOARD``              Per-board cfg used by openocd.
``OPENOCD_SERIAL``             Adapter serial reported by the Lite Kit.
``FLASH_IMAGE``                The .hex image to program.
============================== =======================================

Known limitations
=================

- Only the ``:nsh`` configuration is validated.  No kernel-mode build,
  no networking, no file system.
- Single CPU (CPU0) only.  AMP / multi-core bring-up is intentionally
  not part of this port.
- ``ostest`` has not been run against this board.  The port has been
  validated through the NSH UART + STM clock-tick + synchronous-trap
  test battery only.

=====================================
KIT_A2G_TC397XA_3V3_TFT (AURIX TC397)
=====================================

Infineon's AURIX TC397 Application Kit (KIT_A2G_TC397XA_3V3_TFT) is an
evaluation board for the TriCore TC3X (TC1.6.2P) family with the TC397XA
SoC.

Features
========

- Infineon AURIX TC39x (TC397XA) - 6 TriCore CPUs, 16 MiB PFLASH.
- ASCLIN0 on P14.0/P14.1 routed to the on-board USB-serial bridge
  (115200-8-N-1) for the NSH console.

Default configuration
=====================

::

  ./tools/configure.sh -E a2g-tc397xa-3v3-tft:nsh
  make -j$(nproc)

The toolchain is selected the same way as for the TC375 Lite Kit, see
:doc:`../a2g-tc375-lite/index`.

Known limitations
=================

- Only CPU0 is used, with the same memory layout as the TC375 Lite Kit.
- The ``:nsh`` configuration is build tested only.

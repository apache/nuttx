=========
RH850/U2C
=========

.. tags:: arch:rh850, chip:u2c, vendor:renesas

**RH850/U2C** (R7F702600, G4KH core) is supported in a flat build with these
features:

- Runs in supervisor mode with the FPU enabled (``-msoft-float`` builds with
  ``CONFIG_ARCH_FPU=n``).
- Interrupts are managed by INTC1 and INTC2 with the direct vector method and
  16 priority levels.  Interrupts of a higher priority nest; ISPR holds
  requests of equal and lower priority until the handler returns.
- OSTM0 in free-run compare mode is the system timer through the oneshot
  lower half (``arch_alarm``), ticked or tickless.
- RLIN311 in UART mode is the console, with receive and transmit interrupts.
- CPU exceptions (EI and FE level) print the cause and the registers, then
  panic.

This port has been tested on QEMU only:

- ``qemu-rh850-u2c``: the QEMU ``rh850-u2c`` machine.

Peripheral Support
==================

==========  =======  ===============================
Peripheral  Support  Notes
==========  =======  ===============================
INTC1/2     Yes      Priorities 0-15, nesting
OSTM0       Yes      System timer
RLIN3       Yes      RLIN311 UART console
Others      No
==========  =======  ===============================

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

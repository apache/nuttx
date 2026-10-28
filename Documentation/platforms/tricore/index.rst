=======
TriCore
=======

NuttX runs on Infineon AURIX TriCore microcontrollers in flat 32-bit address
mode.  The port is implemented entirely with direct SFR access; there is no
dependency on Infineon's iLLD SDK, low-level startup is provided by NuttX.

The port covers two AURIX SoC families:

================ ================ =====================
 Family           TriCore ISA      ARCH_FAMILY
================ ================ =====================
 TC3X (TC1.6.2)   TC162            ``tc1v6``
 TC4X (TC1.8)     TC18             ``tc1v8``
================ ================ =====================

Common features:

- Runs in Supervisor Mode on CPU0.
- IRQs routed by the Interrupt Router (IR), per-Service-Request-Node SRC
  registers; SRPN equals the IRQ number.
- System Timer (STM) drives the NuttX systick (oneshot lower-half).
- ASCLIN-based UART for the NSH console.
- GPIO pinmuxing for both families through a shared aurix layer.
- HighTec LLVM 10.2.0 (clang -target tricore) and tricore-elf-gcc 11.3.1
  toolchains are both supported.

Source layout
=============

::

  arch/tricore/
    include/{tc3x,tc4x}/    chip-public headers
    src/
      common/               core arch (boot, traps, IRQs, scheduling)
      aurix/                shared peripherals (ASCLIN UART, GPIO)
      tc3x/                 TC3X-specific (clock tree, ENDINIT, GPIO)
      tc4x/                 TC4X-specific (clock tree, GPIO)

Supported boards
================

.. toctree::
   :maxdepth: 1
   :glob:

   tc3x/*
   tc4x/*

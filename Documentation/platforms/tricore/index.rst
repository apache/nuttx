=======
TriCore
=======

NuttX runs on Infineon AURIX TriCore microcontrollers in flat 32-bit
supervisor mode on CPU0.  The port is implemented entirely with direct
SFR access; there is no dependency on Infineon's iLLD SDK, and the
low-level startup, trap handling and interrupt routing are provided by
NuttX itself.

The port covers two AURIX SoC families:

- `AURIX TC3X <https://www.infineon.com/products/microcontroller/32-bit-tricore/aurix-tc3xx>`_
  using the TC1.6.2P ISA
- `AURIX TC4X <https://www.infineon.com/products/microcontroller/32-bit-tricore/aurix-tc4x>`_
  using the TC1.8P ISA

Key supported features
======================

The table below summarises the state of the kernel-relevant features
across both AURIX SoC families.

================================ ======== ========
Feature                          TC3X     TC4X
================================ ======== ========
ISA version                      TC1.6.2P TC1.8P
Single-CPU NSH kernel            Yes      Yes
SMP / AMP                        No       No
Hardware MPU (DPR/CPR)           No       No
MPU stack guard                  No       No
System Timer (STM) tick          Yes      Yes
Single-precision FPU             Yes      Yes
Double-precision FPU             No       Yes
ASCLIN UART (NSH console)        Yes      Yes
GPIO pinmuxing                   Yes      Yes
Instruction/data cache control   Yes      No
================================ ======== ========

Architecture
============

Registers
---------

The TriCore architecture exposes 32 General Purpose Registers (GPRs)
split into sixteen 32-bit data registers (D[0] through D[15]) and
sixteen 32-bit address registers (A[0] through A[15]).  The Program
Counter (PC), Program Status Word (PSW) and Previous Context Information
register (PCXI) form the core system state.

Several address and data registers have dedicated roles:

- A[10]: Stack Pointer (SP)
- A[11]: Return Address (RA)
- A[0], A[1], A[8], A[9]: Global address registers
- D[15]: Implicit data register
- A[15]: Implicit base address register

TC1.8P additionally exposes the PPRS (Previous Protection Register Set)
and FCX (Free CSA List Head Pointer) as architecturally visible system
registers.

Context Save Areas (CSA)
------------------------

A defining feature of the TriCore architecture is its hardware-managed
context save/restore mechanism using Context Save Areas (CSAs).  Each
CSA is a 64-byte memory block aligned on a 16-word boundary.  CSAs are
linked together to form two lists:

- **Free Context List (FCX)**: pool of available CSAs.
- **Previous Context List (PCX)**: chain of saved contexts for the
  currently executing thread.

The register set is split into an upper and a lower context:

- **Upper context** (automatically saved on interrupts, traps and
  calls): A[10]-A[15], D[8]-D[15], PSW, PCXI and the return address
  A[11].
- **Lower context** (saved explicitly via SVLCX or BISR): A[2]-A[7],
  D[0]-D[7], and A[11].

This hardware mechanism eliminates the need for software to manually
push and pop registers on context switches, interrupt entry or function
calls.

The NuttX port allocates the CSA pool as a fixed array sized by
``CONFIG_TRICORE_CSA_COUNT``.  The boot stub in ``tricore_head.S``
links every CSA into the free list and programs FCX and LCX before
calling ``nx_start()``.

Interrupt handling
------------------

Each interrupt source is assigned a Service Request Priority Number
(SRPN) by writing the per-source SRC register inside the Interrupt
Router (IR).  The Interrupt Control Register (ICR) holds the Current
CPU Priority Number (CCPN) and the Pending Interrupt Priority Number
(PIPN).

An interrupt is taken when ``ICR.IE == 1`` AND ``PIPN > CCPN``.  On
entry the hardware:

- saves the upper context to a CSA;
- sets ``PCXI.PIE = ICR.IE`` and ``PCXI.PCPN = ICR.CCPN``;
- raises ``ICR.CCPN = ICR.PIPN`` and clears ``ICR.IE``;
- loads A[10] from the Interrupt Stack Pointer (ISP) if not already on
  the interrupt stack;
- jumps to the vector entry computed from BIV and PIPN.

NuttX configures BIV for a single-entry dispatch where every interrupt
priority arrives at the common ``tricore_doirq`` stub, which then
dispatches through the NuttX IRQ table.  ``RFE`` (Return From
Exception) restores the upper context and ICR from PCXI.

SRPN equals the NuttX IRQ number on TC4X.  On TC3X the SRPN to IRQ
mapping is dense and identical except for the Time-Out-of-Service
field shift (TOS), which is rebuilt by NuttX at IRQ-attach time.

Trap handling
-------------

The TriCore architecture defines 8 trap classes addressed via the BTV
(Base Trap Vector) register with 32-byte vector spacing:

- Class 0: MMU traps (virtual address fill / protection)
- Class 1: Internal protection (MPR/MPW/MPX/MPP)
- Class 2: Instruction errors (illegal opcode, privilege violation,
  alignment)
- Class 3: Context management (CSA depletion, call depth overflow)
- Class 4: System bus and peripheral errors
- Class 5: Assertion (arithmetic overflow)
- Class 6: System call (``SYSCALL`` instruction)
- Class 7: Non-Maskable Interrupt

NuttX routes every trap class through ``tricore_trap_handler`` in
``arch/tricore/src/common/tricore_trapcall.c``.  The handler decodes
the class/TIN pair from the BTV stub, looks up the cause string and
emits a full register dump via ``up_dump_register``.  Class 6 traps
are forwarded to ``tricore_svcall`` for syscall dispatch; all other
classes log + return so the system can keep running where the trap is
recoverable.  Class 7 (NMI) panics the system.

Toolchain
=========

Two toolchains are supported and both must build the tree warning-free:

- **AURIX GCC** (default).  Available from
  `Infineon Software Tools <https://softwaretools.infineon.com/assets/com.ifx.tb.tool.aurixgcc>`_.
  The current supported version is GCC 11.3.1
  (``tricore-elf-gcc 11.3.1``).
- **HighTec LLVM** (commercial; enables ``CONFIG_TRICORE_TOOLCHAIN_LLVM``).
  Requires a HighTec Development Platform licence reachable through the
  ``RLM_LICENSE`` environment variable at build time.

Configure the chosen toolchain by placing its ``bin`` directory on the
build environment's ``PATH``:

.. code-block:: console

   export PATH=/opt/tooling/tricore-gcc/bin:$PATH

NuttX picks up the AURIX target tuple (``tricore-elf-``) and the right
``-mcpu=`` flag automatically based on the selected SoC family.

Source layout
=============

::

  arch/tricore/
    include/                 chip-public headers (arch.h, irq.h, ...)
    src/
      common/                core arch (boot, traps, IRQs, scheduling)
      aurix/                 shared peripherals (ASCLIN UART, GPIO)
      tc3x/                  TC3X-specific (clock tree, ENDINIT)
      tc4x/                  TC4X-specific (clock tree)

  boards/tricore/
    tc3x/<board>/            TC3X boards
    tc4x/<board>/            TC4X boards

Drivers
=======

================================== ======== ========
Driver                             TC3X     TC4X
================================== ======== ========
ASCLIN UART                        Yes      Yes
GPIO pinmuxing                     Yes      Yes
System Timer (STM)                 Yes      Yes
Clock control                      Yes      Yes
Interrupt Router (IR)              Yes      Yes
================================== ======== ========

Flashing and debugging
======================

The recommended flash path on Linux is OpenOCD with the TriCore/AURIX
extensions and Infineon's Device Access Server (DAS) talking to the
on-board DAP-Miniwiggler.

Patched OpenOCD with AURIX support is maintained at
`linumiz/openocd-aurix <https://gitlab.com/linumiz/infineon/release/openocd-aurix>`_.

Two Infineon vendor packages are required at runtime:

- `DAS (Device Access Server) <https://softwaretools.infineon.com/>`_,
  which provides the ``tas_server`` daemon used by OpenOCD to talk to
  the on-board debugger.
- `libftd2xx <https://ftdichip.com/drivers/d2xx-drivers/>`_ from FTDI,
  which must be placed alongside ``tas_server``.

Build OpenOCD:

.. code-block:: console

   sudo apt install build-essential git autoconf libtool \
                    pkg-config libusb-1.0-0-dev

   git clone https://gitlab.com/linumiz/infineon/release/openocd-aurix.git
   cd openocd-aurix
   ./bootstrap
   mkdir build && cd build
   ../configure --prefix=/opt/tooling/tricore-openocd \
                --enable-tas-client --enable-ftdi \
                --enable-jlink --enable-stlink
   make -j$(nproc) && sudo make install

Install DAS following the package instructions, then copy ``libftd2xx``
into the ``tas_server`` directory.

Add a udev rule so the debugger is accessible without root:

.. code-block:: console

   sudo tee /etc/udev/rules.d/99-infineon-aurix.rules >/dev/null <<'EOF'
   SUBSYSTEM=="usb", ATTRS{idVendor}=="058b", MODE="0666", GROUP="plugdev"
   SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", MODE="0666", GROUP="plugdev"
   EOF
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   sudo usermod -aG plugdev $USER

Log out and back in to pick up the group membership.

Start the TAS server once per boot.  It listens on TCP port 24817:

.. code-block:: console

   cd <path-to-tas_server>
   LD_LIBRARY_PATH=. ./tas_server &

With OpenOCD installed and ``tas_server`` running, flash from the NuttX
tree with:

.. code-block:: console

   make flash

Each AURIX board's ``Make.defs`` invokes OpenOCD with the correct board
configuration and adapter serial automatically.

Known limitation: TC37x debug (OCDS5)
-------------------------------------

After a synchronous trap on TC37x, the on-chip OCDS5 debug unit can
refuse a clean ``reset run`` until the next power cycle, producing
``[tc37x.cpu0] Failed to enable debug`` from OpenOCD.  Recover by
power-cycling the kit (USB unplug for 10+ seconds).  TC4Dx does not
exhibit this behaviour.

Alternative tools
-----------------

`iSYSTEM winIDEA <https://www.isystem.com/products/software/winidea.html>`_
together with the
`AURIX Development Studio <https://softwaretools.infineon.com/assets/com.ifx.tb.tool.aurixide>`_
is a commercial alternative supported on Windows.  A full list of
supported flash programming tools is available at
`Infineon AURIX Flash Tools <https://www.infineon.com/design-resources/platforms/aurix-software-tools/aurix-tools/flash>`_.

Architecture references
=======================

- `TC1.8 Architecture Manual Volume 1 <https://www.infineon.com/assets/row/public/documents/10/44/infineon-infineon-tricore-tc1.8-architecture-usermanual-en.pdf>`_
- `TC1.8 Architecture Manual Volume 2 <https://www.infineon.com/assets/row/public/documents/10/44/infineon-tricore-tc1.8-architecture-volume2-usermanual-en.pdf>`_
- `TC1.6 Architecture Manual Volume 1 <https://www.infineon.com/assets/row/public/documents/10/44/infineon-aurix-architecture-vol1-usermanual-en.pdf>`_
- `TC1.6 Architecture Manual Volume 2 <https://www.infineon.com/assets/row/public/documents/10/44/infineon-aurix-architecture-vol2-usermanual-en.pdf>`_

Maintainers
===========

- Parthiban Nallathambi (`Linumiz <https://linumiz.com>`_)
- Saravanan Sekar (`Linumiz <https://linumiz.com>`_)

Supported SoC families
======================

.. toctree::
   :maxdepth: 1
   :glob:

   */*

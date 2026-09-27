===================
Device Tree support
===================

A device tree describes the hardware of a board as data -- what peripherals
exist, at which addresses, on which interrupts -- instead of as code.  The
same kernel image can then boot on several boards, reading the differences
at runtime rather than being compiled for one of them.

Enabled with ``CONFIG_DEVICE_TREE``, which selects ``CONFIG_LIBC_FDT``.  The
code is in ``drivers/devicetree/``.

Where NuttX uses it
===================

NuttX is not a device-tree-first system the way Linux is.  Most ports
describe their hardware in the board directory and in ``Kconfig``, which is
smaller and needs no parsing at boot.  Device tree is used where the
hardware genuinely is not known until run time:

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Source file
     - What it reads out of the tree
   * - ``fdt.c``
     - The tree itself: finding nodes and reading properties.
   * - ``fdt_pci.c``
     - PCI host bridges and their address windows.
   * - ``fdt_virtio_mmio.c``
     - Virtio devices, which is how a virtual machine tells the guest what
       it has.
   * - ``fdt_cfi.c``
     - CFI flash, whose geometry the tree can describe.

The pattern is the same in each: a platform where the *set* of devices is
decided by something outside the firmware -- a hypervisor, a bootloader, a
board with sockets -- and where compiling the answer in would be wrong.

Getting one
===========

Whatever produces the tree, a port hands its address to
``fdt_register()`` during start-up, and code that needs it later calls
``fdt_get()``.  How the address is arrived at differs by port, and in the
tree today it is usually *not* the bootloader:

* ``qemu-rv`` takes the pointer the RISC-V boot protocol hands it, which is
  the case the word "bootloader" describes.
* ``litex`` uses a pointer if one was passed and otherwise falls back to
  ``CONFIG_LITEX_FDT_MEMORY_ADDRESS``, a compile-time constant.
* the ARM and ARM64 ``qemu`` and ``goldfish`` ports register a hard-coded
  ``0x40000000``, which is simply where the emulator puts the tree.

On QEMU the tree itself is generated for you from the command line options.
:doc:`/guides/drivers/devicetree` covers using one in practice.

Nearly every configuration that turns this on is an emulator: of the 25
defconfigs with ``CONFIG_DEVICE_TREE=y``, 24 are QEMU and one is the
Allwinner A527.

=======
Intel64
=======

Architecture-Specific Directories
=================================

Architecture-specific directories hold common source files shared for by
implementations of specific x86_64 architectures.

``intel64``
  This directory holds logic appropriate for any instantiation of the 64-bit
  intel64 architecture.

Chip-Specific directories
=========================

The same x86 architecture may be realized in different chip implementations.
For SoC chips, in particular, on-chip devices and differing interrupt
structures may require special, chip-specific definitions in these chip-
specific directories.

``qemu``
  This is the implementation of NuttX on the QEMU x86_64. It's empty for now.

Features
========

Hardware acceleration
---------------------

Configurable hardware acceleration: SSE, AVX2, AVX512 support.

FMA, AVX and AVX512 support requires ``XSAVE`` instructions support which is
controled with ``CONFIG_ARCH_X86_64_HAVE_XSAVE`` option.

IRQs
----

IRQs are managed by LAPIC(X2APIC) and IOAPIC.

Clock source
------------

TSC DEADLINE timer, APIC timer or HPET can be used as system clock.

SMP
---

SMP is supported up to 4 cores now (BSP + 3 AP), but can be easly extended.

HPET
----

High Precision Event Timer (HPET) is supported as oneshot timer.

RDRAND
------

The ``/dev/random`` device support with ``RDRAND`` instruction is enabled with
``CONFIG_ARCH_INTEL64_HAVE_RDRAND=y``

PCI bus
-------

PCI bus is supported with legacy interrupts, MSI and MSI-X.

Multiboot Framebufer
--------------------

Multiboot2 framebuffer is supported with ``CONFIG_MULTBOOT2_FB_TERM=y``.

It is very possible that the framebuffer is mapped in a memory region above 4GB,
so you may also need to set ``CONFIG_MM_PGALLOC=y``.

To enable framebuffer support in QEMU, ommit the ``-nographic`` argument
and use ``-cdrom boot.iso`` (multiboot2 framebuffer doesn't work with
``-kernel`` option).

Also, your GRUB configuration (``grub.cfg``) should insert the appropriate video
module, in many cases ``insmod all_video`` should be enough.

Kernel build
------------

Kernel build is supported.

.. warning:: 
   IMPORTANT: the current implementation doesn't include any protection against
   speculative execution vulnerabilities (Spectre, Meltdown and others) !

Creating a bootable disk
========================

This build supports multiboot2, which means that usual multiboot2 bootlaoders,
e.g. grub can be used. To create a bootable disk with grub2, create a directory
named ``iso`` with grub configuration file and the compiled ``nuttx.elf``.

Directory and file hierarchy::

 - iso/
   - boot/
     - grub/
       - grub.cfg
     - nuttx.elf

The grub.cfg should contain the boot entry of NuttX::

  set timeout=0
  set default=0
  menuentry "kernel" {
    multiboot2 /boot/nuttx.elf
  }

Making the disk
---------------

Use the following command to create the disk.
P.S. In some distros, ``grub-mkrescue`` is called ``grub2-mkrescue``::

  grub-mkrescue -o boot.iso iso

Grub with UEFI
--------------

This flow is very similar except you need to have the BOOTX64.EFI file.
You can find this in most Linux distributions::

  iso/
  └── boot
      ├── efi
      │   └── EFI
      │       └── BOOT
      │           └── BOOTX64.EFI
      ├── grub
      │   └── grub.cfg
      └── nuttx.elf

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

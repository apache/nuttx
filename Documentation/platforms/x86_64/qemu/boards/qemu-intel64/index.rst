============
qemu-intel64
============

This page file describes the contents of the build configurations available
for the NuttX QEMU x86_64 port.

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

QEMU/KVM
========

QEMU is a generic and open source machine emulator and virtual machine.  Here are
some links (which will probably be mostly outdated by the time your read this):

* Home Page: http://wiki.qemu.org/Main_Page
* Downloads: http://wiki.qemu.org/Download
* Documentation: http://wiki.qemu.org/Manual

KVM is the Linux kernel hypervisor.
It supports creations of virtual machines in Linux systems.
It is usually coupled with Qemu as its I/O supporting layer.

The qemu can be build from source or downloaded from distro repositories.
However, a modern CPU and KVM support are mandatory because the X2APIC is not
available in pure emulator mode.
This mean using this build with qemu in windows or old x86 machine can be
frustrating. In such case, looks the next section and use bochs emulator instead.

Running QEMU
------------

In the top-level NuttX directory::

    qemu-system-x86_64 -cpu host -enable-kvm -m 2G -cdrom boot.iso -nographic -serial mon:stdio

This multiplex the qemu console and COM1 to your console.

Use control-a 1 and 2 to switch between.
Use control-a x to terminate the emulation.

P.S. Make sure that you CPU supports the mandatory features. Look at Real machine
section for more information.

Bochs
=====

Bochs is also a generic and open source machine emulator and virtualizer.
It does very comprehensive emulation of x86 platform, even the state-of-art processors.
Here are some links (which will probably be mostly outdated by the time your read this):

* Home Page: http://bochs.sourceforge.net

The bochs can be build from source.
Unlike qemu, it does not rely on KVM to support modern hardware features,
therefore it can also be used under Windows.
When building bochs, remember to enable x86-64 support with ``--enable-x86-64``.
If you also want support for SIMD instructions, enable them with ``--enable-avx --enable-evex``.

Running Bochs
-------------

First edit/check the ``.bochsrc``
You can create one in the top-level NuttX directory or bochs will use the one in your $HOME.
Remember to change the CPU model to one with mandatory features and enable the COM port.

* Find and edit (You might adjust the IPS as you machine perform)::

    cpu: model=broadwell_ult, count=1, ips=50000000, reset_on_triple_fault=0, ignore_bad_msrs=0, msrs="msrs.def"
    ata0-master: type=cdrom, path="<PATH TO boot.iso>", status=inserted

* Add::

    com1: enabled=1, mode=file, dev=com1.out

* In the top-level NuttX directory::

    bochs

The emulator will drop into debugger mode.
Enter ``c`` to start the emulation.
COM port output will be in the com1.out file.

Real machine
============

This port should work on real x86-64 machine with a proper CPU.
The mandatory CPU features are:

* TSC DEADLINE or APIC timer
* PCID
* X2APIC

WARNING: IF you use TSC DEADLINE, make sure that your CPU's TSC DEADLINE timer
is not buggy!

Toolchains
==========

Currently, only the Linux GCC toolchain is tested.
While building on a modern x86_64 PC, the default system GCC can be used.

Configurations
==============

Common Configuration Notes
--------------------------

1. Each Qemu-intel64 configuration is maintained in a sub-directory
   and can be selected as follow::

     tools/configure.sh qemu-intel64:<subdir>

   Where ``<subdir>`` is one of the configuration sub-directories described in
   the following paragraph.

2. These configurations use the mconf-based configuration tool.  To
   change a configurations using that tool, you should:

   a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
      see additional README.txt files in the NuttX tools repository.

   b. Execute ``make menuconfig`` in nuttx/ in order to start the
      reconfiguration process.

3. By default, all configurations assume the Linux.  This is easily
   reconfigured::

     CONFIG_HOST_LINUX=y

Configuration Sub-Directories
-----------------------------

ostest
------

The "standard" NuttX examples/ostest configuration.

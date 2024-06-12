=======
rv-virt
=======

RISC-V Toolchain
================

Any generic RISC-V toolchain can be used. It's recommended to use the same toolchain used by NuttX CI.

Please refer to the `Docker container <https://github.com/apache/nuttx/tree/master/tools/ci/docker/linux/Dockerfile>`_ and
check for the current compiler version being used. For instance:

.. code-block::

    ###############################################################################
    # Build image for tool required by RISCV builds
    ###############################################################################
    FROM nuttx-toolchain-base AS nuttx-toolchain-riscv
    # Download the latest RISCV GCC toolchain prebuilt by xPack
    RUN mkdir riscv-none-elf-gcc && \
    curl -s -L "https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v13.2.0-2/xpack-riscv-none-elf-gcc-13.2.0-2-linux-x64.tar.gz" \
    | tar -C riscv-none-elf-gcc --strip-components 1 -xz

It uses the xPack's prebuilt toolchain based on GCC 13.2.0-2.

RISC-V QEMU
===========

Build and install ``qemu``::

  $ git clone https://github.com/qemu/qemu
  $ cd qemu
  $ ./configure --target-list=riscv32-softmmu,riscv64-softmmu
  $ make
  $ sudo make install

QEMU 7.2.9 or later and OpenSBI v1.1 or later (usually shipped with QEMU) is required, to support RISC-V "Sstc" Extension. It is also recommended to use the latest QEMU and OpenSBI.

For users who wish to use their own OpenSBI, please refer to `OpenSBI repository <https://github.com/riscv-software-src/opensbi>`_.

Configurations
==============

All of the configurations presented below can be tested by running the following commands::

   $ ./tools/configure.sh rv-virt:<config_name>

Where <config_name> is the name of the configuration you want to use, i.e.: nsh, knsh32, knsh64...

To build it, run the following command::

   $ make -j$(nproc)

or, with more verbosity::

   $ make V=1 -j$(nproc)

.. warning::
    Some configurations require additional steps to be built. Please refer to the specific
    configurations to check it out

Finally, to run it, use the following command:

For 32-bit configurations::

    $ qemu-system-riscv32 -semihosting -M virt,aclint=on -cpu rv32 -smp <cpu number> -bios none -kernel nuttx -nographic

And, for 64-bit configurations::

    $ qemu-system-riscv64 -semihosting -M virt,aclint=on -cpu rv64 -smp <cpu number> -bios none -kernel nuttx -nographic

``-smp`` option can be only used in smp build, and the ``cpu number`` needs
to be set to the same value as ``CONFIG_SMP_NCPUS`` in the build config file.

If testing with S-mode build, remove the ``-bios none`` option. S-mode build
requires SBI to function properly.

citest
------

This configuration is the default configuration intended to be used by the automated
testing on CI of 32-bit RISC-V using QEMU.

To run it with QEMU, use the following command::

    $ qemu-system-riscv32 -semihosting -M virt -cpu rv32 \
      -drive index=0,id=userdata,if=none,format=raw,file=./fatfs.img \
      -device virtio-blk-device,bus=virtio-mmio-bus.0,drive=userdata \
      -bios none -kernel nuttx -nographic

To run the CI scripts, use the following command::

    $ ./nuttx/boards/risc-v/qemu-rv/rv-virt/configs/citest/run

citest64
--------

Identical to the `citest`_ configuration, but for 64-bit RISC-V.

fb
--

Uses the VirtIO GPU driver to run the `fb` demo application on 32-bit RISC-V.

To run it with QEMU, use the following command::

    $ qemu-system-riscv32 -semihosting -M virt -cpu rv32 -smp 8 \
      -chardev stdio,id=con,mux=on \
      -serial chardev:con \
      -device virtio-gpu-device,xres=640,yres=480,bus=virtio-mmio-bus.0 \
      -mon chardev=con,mode=readline \
      -bios none -kernel nuttx

fb64
----

Identical to the `fb`_ configuration, but for 64-bit RISC-V.

To run it with QEMU, use the following command::

    $ qemu-system-riscv64 -semihosting -M virt -cpu rv64 -smp 8 \
      -chardev stdio,id=con,mux=on \
      -serial chardev:con \
      -device virtio-gpu-device,xres=640,yres=480,bus=virtio-mmio-bus.0 \
      -mon chardev=con,mode=readline \
      -bios none -kernel nuttx

knetnsh64
---------

Similar to the `knsh32`_ configuration, but with networking support and 64-bit RISC-V.

To run it with QEMU, use the following command::

    $ dd if=/dev/zero of=./mydisk-1gb.img bs=1M count=1024

    $ qemu-system-riscv64 -semihosting -M virt,aclint=on -cpu rv64 -smp 8 \
      -global virtio-mmio.force-legacy=false \
      -device virtio-serial-device,bus=virtio-mmio-bus.0 \
      -chardev socket,telnet=on,host=127.0.0.1,port=3450,server=on,wait=off,id=foo \
      -device virtconsole,chardev=foo \
      -device virtio-rng-device,bus=virtio-mmio-bus.1 \
      -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
      -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.2 \
      -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd \
      -device virtio-blk-device,bus=virtio-mmio-bus.3,drive=hd \
      -kernel ./nuttx/nuttx -nographic

knetnsh64_smp
-------------

Similar to the `knetnsh64`_ configuration, but with SMP support for 64-bit RISC-V.

knsh32
------

This is similar to the `nsh`_ configuration except that NuttX
is built as a kernel-mode, monolithic module, and the user applications
are built separately. It uses `hostfs` and QEMU in semi-hosting mode to
load the user-space applications. This is intended to 32-bit RISC-V.

To build it, use the following command::

    $ make V=1 -j$(nproc)
    $ make export V=1 -j$(nproc)
    $ pushd ../apps
    $ ./tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz
    $ make import V=1 -j$(nproc)
    $ popd

Run it with QEMU using the default command for 32-bit RISC-V.

In `nsh`, applications can be run from the `/system/bin` directory::

    nsh> /system/bin/hello

.. _knsh32_paging:

knsh32_paging
-------------

Similar to ``knsh32_romfs``, but enabling on-demand paging: this
configuration simulates a 4MiB device (using QEMU), but sets the number of
heap pages equal to ``CONFIG_ARCH_HEAP_NPAGES=2048``. This means that each
process's heap is 8MiB, whereas ``CONFIG_POSIX_SPAWN_DEFAULT_STACKSIZE`` is
``1048576`` (1MiB) represents the stack size of the processes (which is
allocated from the process's heap). This configuration is used for 32-bit
RISC-V which implements the Sv32 MMU specification and enables processes
to have their own address space larger than the available physical memory.
This is particularly useful for implementing a set of programming language
interpreters.

knsh32_romfs
------------

Similar to the `knsh32`_ configuration, but uses ROMFS instead of `hostfs`.
A ROMFS image is generated and linked to the kernel. This requires re-running ``make``::

    $ make V=1 -j$(nproc)
    $ make export V=1 -j$(nproc)
    $ pushd ../apps
    $ ./tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz
    $ make import V=1 -j$(nproc)
    $ ./tools/mkromfsimg.sh ../nuttx/arch/risc-v/src/board/romfs_boot.c
    $ popd
    $ make V=1 -j$(nproc)

To run it, use the following command::

    $ qemu-system-riscv32 -M virt,aclint=on -cpu rv32 -kernel nuttx -nographic

In `nsh`, applications can be run from the `/system/bin` directory::

    nsh> /system/bin/hello

knsh64
------

Similar to the `knsh32`_ configuration, but for 64-bit RISC-V.

Run it with QEMU using the default command for 64-bit RISC-V.

In `nsh`, applications can be run from the `/system/bin` directory::

    nsh> /system/bin/hello

ksmp64
------

Identical to the `knsh64`_ configuration but with SMP support.

netnsh
------

Similar to the `nsh`_ configuration, but with networking support for 32-bit RISC-V.

To run it with QEMU, use the following command::

    $ dd if=/dev/zero of=./mydisk-1gb.img bs=1M count=1024

    $ qemu-system-riscv32 -semihosting -M virt,aclint=on -cpu rv32 -smp 8 \
      -global virtio-mmio.force-legacy=false \
      -device virtio-serial-device,bus=virtio-mmio-bus.0 \
      -chardev socket,telnet=on,host=127.0.0.1,port=3450,server=on,wait=off,id=foo \
      -device virtconsole,chardev=foo \
      -device virtio-rng-device,bus=virtio-mmio-bus.1 \
      -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
      -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.2 \
      -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd \
      -device virtio-blk-device,bus=virtio-mmio-bus.3,drive=hd \
      -bios none -kernel ./nuttx/nuttx -nographic

netnsh64
--------

Similar to the `netnsh`_ configuration, but for 64-bit RISC-V.

To run it with QEMU, use the following command::

    $ dd if=/dev/zero of=./mydisk-1gb.img bs=1M count=1024

    $ qemu-system-riscv64 -semihosting -M virt,aclint=on -cpu rv64 -smp 8 \
      -global virtio-mmio.force-legacy=false \
      -device virtio-serial-device,bus=virtio-mmio-bus.0 \
      -chardev socket,telnet=on,host=127.0.0.1,port=3450,server=on,wait=off,id=foo \
      -device virtconsole,chardev=foo \
      -device virtio-rng-device,bus=virtio-mmio-bus.1 \
      -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
      -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.2 \
      -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd \
      -device virtio-blk-device,bus=virtio-mmio-bus.3,drive=hd \
      -bios none -kernel ./nuttx/nuttx -nographic

netnsh64_smp
------------

Similar to the `netnsh64`_ configuration, but with SMP support for 64-bit RISC-V.

To run it with QEMU, use the following command::

    $ dd if=/dev/zero of=./mydisk-1gb.img bs=1M count=1024

    $ qemu-system-riscv64 -semihosting -M virt,aclint=on -cpu rv64 -smp 8 \
      -global virtio-mmio.force-legacy=false \
      -device virtio-serial-device,bus=virtio-mmio-bus.0 \
      -chardev socket,telnet=on,host=127.0.0.1,port=3450,server=on,wait=off,id=foo \
      -device virtconsole,chardev=foo \
      -device virtio-rng-device,bus=virtio-mmio-bus.1 \
      -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
      -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.2 \
      -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd \
      -device virtio-blk-device,bus=virtio-mmio-bus.3,drive=hd \
      -bios none -kernel ./nuttx/nuttx -nographic

netnsh_smp
----------

Similar to the `netnsh`_ configuration, but with SMP support for 32-bit RISC-V.

To run it with QEMU, use the following command::

    $ dd if=/dev/zero of=./mydisk-1gb.img bs=1M count=1024

    $ qemu-system-riscv32 -semihosting -M virt,aclint=on -cpu rv32 -smp 8 \
      -global virtio-mmio.force-legacy=false \
      -device virtio-serial-device,bus=virtio-mmio-bus.0 \
      -chardev socket,telnet=on,host=127.0.0.1,port=3450,server=on,wait=off,id=foo \
      -device virtconsole,chardev=foo \
      -device virtio-rng-device,bus=virtio-mmio-bus.1 \
      -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
      -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.2 \
      -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd \
      -device virtio-blk-device,bus=virtio-mmio-bus.3,drive=hd \
      -bios none -kernel ./nuttx/nuttx -nographic

nsh
---

Configures the NuttShell (nsh) located at examples/nsh.  This NSH
configuration is focused on low-level, command-line driver testing.
This configuration is used for 32-bit RISC-V

nsh64
-----

Identical to the `nsh`_ configuration, but for 64-bit RISC-V.

smp
---

Similar to the `nsh`_ configuration, but with SMP support.
This configuration is used for 32-bit RISC-V

smp64
-----

Similar to the `nsh`_ configuration, but with SMP support
This configuration is used for 64-bit RISC-V

flats
-------

Similar to the `nsh`_ configuration, but running in S-mode.
This configuration is used for 32-bit RISC-V

flats64
-------

Similar to the `nsh`_ configuration, but running in S-mode.
This configuration is used for 64-bit RISC-V

RISC-V GDB Debugging
====================

First of all, make sure to select ``CONFIG_DEBUG_SYMBOLS=y`` in `menuconfig`.

After building the kernel (and the applications, in kernel mode), use the toolchain's GDB
to debug RISC-V applications. For instance, if you are using the xPack's prebuilt toolchain,
you can use the following command to start GDB::

    $ riscv-none-elf-gdb-py3 -ix tools/gdb/__init__.py --tui nuttx

To use QEMU for debugging, one should add the parameters ``-s -S`` to the QEMU command line.

For instance::

    $ qemu-system-riscv32 -semihosting -M virt,aclint=on -cpu rv32 -smp 8 -bios none -kernel nuttx -nographic -s -S

Then, in GDB, use the following command to connect to QEMU::

    $ target extended-remote localhost:1234

Debugging Applications in Kernel Mode
-------------------------------------

In kernel mode, only the kernel symbols are loaded by default.

If needed, one should also load the application symbols using the following command::

    $ add-symbol-file <file> <address>

``address`` refers to the ``.text`` section of the application and can be retrieved from the ELF file using the following command::

    $ riscv-none-elf-readelf -WS <file> | grep .text

For instance, to check the ``.text`` section address of the ``hello`` application, use the following command::

    $ riscv-none-elf-readelf -WS ../apps/bin/hello | grep .text
    [ 1] .text             PROGBITS        c0000000 001000 0009e0 00  AX  0   0  2

.. note:: Pay attention that ``riscv-none-elf-readelf`` refers to your toolchain's readelf utility. Adjust accordingly if you are
    using a different toolchain.

Then, look for the ``.text`` section address and use the ``c0000000`` as the address to load the symbols.

For instance, if you want to load the ``hello`` application, you can use the following command in GDB::

    $ add-symbol-file ../apps/bin/hello 0xc0000000

Then, you can set breakpoints, step through the code, and inspect the memory and registers of the applications too.

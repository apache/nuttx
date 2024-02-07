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

    $ qemu-system-riscv32 -semihosting -M virt,aclint=on -cpu rv32 -smp 8 -bios none -kernel nuttx -nographic

And, for 64-bit configurations::

    $ qemu-system-riscv64 -semihosting -M virt,aclint=on -cpu rv64 -smp 8 -bios none -kernel nuttx -nographic


citest
------

This configuration is the default configuration intended to be used by the automated
testing on CI of 32-bit RISC-V using QEMU.

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
      -bios none -kernel ./nuttx/nuttx -nographic

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

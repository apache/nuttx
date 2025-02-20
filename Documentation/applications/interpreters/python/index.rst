=============================
``Python`` Python interpreter
=============================

This guide explains how to run Python on NuttX.

*Yes, you heard it right*. **Python on NuttX**. This is a port of the `CPython <https://github.com/python/cpython>`_ repository for NuttX.
The `CPython` repository is the reference implementation of the Python programming language.
It is written in C and is the most widely used Python interpreter.

.. warning::
   Python for NuttX is still in the experimental stage (thus, it requires ``CONFIG_EXPERIMENTAL`` to be enabled)
   It is not fully functional for all the architectures and configurations.
   Please check this `issue <https://github.com/apache/nuttx-apps/issues/2884>`_ in the `nuttx-apps <https://github.com/apache/nuttx-apps/>`_ repository to know the current status.

How Does it Work?
=================

1. Python for NuttX runs on RISC-V QEMU (emulated) and on Espressif's ESP32-S3.
2. Python modules are stored in `pyc <https://docs.python.org/3/glossary.html#term-bytecode>`_ (byte-code format) and are loaded from a ROMFS image at startup.
3. The Python wrapper application on NuttX mounts the ROMFS partition which contains the Python modules and sets the required environment variables (``PYTHONHOME`` and ``PYTHON_BASIC_REPL``) automatically.

Building and Running Python on NuttX
====================================

``rv-virt`` (RISC-V QEMU)
----------------------------

Use the ``rv-virt:python`` config to build Python for NuttX. Note that the CMake scripts don't work for this configuration. For now, please use the makefile build instead:

.. code:: console

   $ cd nuttx
   $ make distclean
   $ ./tools/configure.sh rv-virt:python
   $ make -j$(nproc)
   $ ls -l nuttx

This will generate a ``nuttx`` binary. This file can be run using the RISC-V QEMU.

Before running the RISC-V QEMU, create a raw disk image with the following command:

.. code:: console

   $ dd if=/dev/zero of=./mydisk-1gb.img bs=1M count=1024

Then, run RISC-V QEMU with the following command:

.. code:: console

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
      -bios none -kernel ./nuttx -nographic

   ABC[    0.062131] board_userled: LED 1 set to 0
   [    0.063269] board_userled: LED 2 set to 0
   [    0.063367] board_userled: LED 3 set to 0
   telnetd [4:100]

   NuttShell (NSH) NuttX-10.4.0
   nsh> python
   Python 3.13.0 (main, Dec  4 2024, 17:00:42) [GCC 13.2.0] on nuttx
   Type "help", "copyright", "credits" or "license" for more information.
   >>>

Demo
^^^^

Check the following `asciinema <https://asciinema.org/>`_ demo to see how to run Python on NuttX. You can copy and paste the commands from the demo to try it yourself.

.. image:: https://asciinema.org/a/bYYy1fyIOQ3hOY4lJ7L3WFcNb.svg
   :target: https://asciinema.org/a/bYYy1fyIOQ3hOY4lJ7L3WFcNb

ESP32-S3
--------

For this example, we will use the ESP32-S3 board. Use the ``esp32s3-devkit:python`` config to build
Python for NuttX. Note that this defconfig uses a board with the ESP32-S3-WROOM-2 module with 32MiB
of flash and 8MiB of PSRAM.

Build and flash the ESP32-S3 board with the following commands:

.. code:: console

   $ cd nuttx
   $ make distclean
   $ ./tools/configure.sh esp32s3-devkit:python
   $ make flash ESPTOOL_BINDIR=./ ESPTOOL_PORT=/dev/ttyUSB0 -s -j$(nproc)

To run Python on ESP32-S3, just open a serial terminal and connect to the ESP32-S3 board.
You will see the NuttShell (NSH) prompt. Run the following commands:

.. code:: console

   nsh> python
   Python 3.13.0 (main, Dec  4 2024, 17:00:42) [GCC 13.2.0] on nuttx
   Type "help", "copyright", "credits" or "license" for more information.
   >>>

==============
qemu-rh850-u2c
==============

.. tags:: arch:rh850, chip:u2c, vendor:renesas

This board runs NuttX on the QEMU ``rh850-u2c`` machine, which models the
RH850/U2C8 (R7F702600) CPU0 with code flash at 0x00000000, local RAM at
0xFDC00000, INTC1/INTC2, OSTM0 and RLIN311 (serial0).

Toolchain
=========

The ``v850-elf`` GCC toolchain (GCC 14, ``-mv850e3v5 -mrh850-abi``) is used.  A
prebuilt Linux x86_64 toolchain is available as a release asset::

  $ curl -L -O https://github.com/linumiz/rh850-gcc-toolchain/releases/download/v1.0.0/v850-elf-toolchain-linux-x86_64.tar.gz
  $ mkdir -p v850-elf-gcc
  $ tar -C v850-elf-gcc -xzf v850-elf-toolchain-linux-x86_64.tar.gz
  $ export PATH=$PWD/v850-elf-gcc/bin:$PATH

The toolchain may also come with the Zephyr SDK later.

QEMU
====

``qemu-system-rh850`` is not in upstream QEMU yet.  A prebuilt Linux x86_64
binary is available as a release asset::

  $ curl -L -O https://github.com/linumiz/qemu-rh850/releases/download/v1.0.0/qemu-rh850-linux-x86_64.tar.gz
  $ mkdir -p qemu-rh850
  $ tar -C qemu-rh850 -xzf qemu-rh850-linux-x86_64.tar.gz
  $ export PATH=$PWD/qemu-rh850/bin:$PATH

Both release binaries run on x86_64 Linux with glibc 2.34 or later (Ubuntu
22.04 and 24.04) and need these runtime packages: ``libexpat1`` (GDB) and
``libglib2.0-0`` on Ubuntu 22.04 or ``libglib2.0-0t64`` on Ubuntu 24.04 (QEMU).

Configurations
==============

Each configuration is selected with::

  $ ./tools/configure.sh qemu-rh850-u2c:<subdir>

nsh
---

NuttShell on the RLIN311 console, ticked system timer.

citest
------

The CI runtime test.  ``ostest`` is the init entry; ``run.sh`` runs QEMU with
the options below and returns the QEMU exit code: 0 when ostest passes, the
ostest result when it fails, 1 on a PANIC and 124 when the 900 s timeout
expires.

Building
========

Make::

  $ ./tools/configure.sh qemu-rh850-u2c:nsh
  $ make -j$(nproc)

CMake::

  $ cmake -B build -DBOARD_CONFIG=qemu-rh850-u2c:nsh -GNinja
  $ cmake --build build

Running
=======

::

  $ qemu-system-rh850 -M rh850-u2c,test-exit=on -icount shift=4,sleep=on \
      -nographic -kernel nuttx

- ``-icount shift=4,sleep=on`` runs 16 ns per instruction with virtual time
  following host time while the CPU is in HALT.  ``CONFIG_BOARD_LOOPSPERMSEC``
  (8906) is calibrated for this setting.
- ``test-exit=on`` maps the test-exit device at 0x40000000.  ``board_power_off``
  writes 0x5555 (exit code 0) or ``(status << 16) | 0x3333`` (exit code
  ``status``).  With ``CONFIG_QEMU_RH850_U2C_TEST_EXIT`` (on in the citest
  configuration) ``board_reset`` with a non-zero status, the PANIC path with
  ``CONFIG_BOARD_RESET_ON_ASSERT``, ends QEMU with that status.
- ``board_reset`` otherwise, and ``reboot`` in NSH, start a software system
  reset through SWSRESA: the machine restarts from the reset vector.
- ``-M rh850-u2c,clk-hsb=<Hz>`` changes the OSTM clock (default 20 MHz); set
  ``CONFIG_RH850_CLK_HSB_FREQUENCY`` to the same value.

Limitations
===========

- Tested on QEMU only; silicon is untested.
- Local RAM keeps its content over a reset in QEMU; on silicon its value
  after a reset is not guaranteed.
- No MPU, no protected or kernel build, no user mode.
- CPU exceptions always panic.

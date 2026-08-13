==========
hpm6360evk
==========

1. Download and install toolchain::

    $ wget https://github.com/hpmicro/riscv-gnu-toolchain/releases/download/2022.05.15/riscv32-unknown-elf-newlib-multilib_2022.05.15_linux.tar.gz

    $ tar xf riscv32-unknown-elf-newlib-multilib_2022.05.15_linux.tar.gz

    $ export PATH=$PATH:/<path to compiler>/riscv32-unknown-elf-newlib-multilib/bin/

   NOTE: Make sure HPMicro GNU riscv32 toolchain have been installed and be found in PATH.::

    $ riscv32-unknown-elf-gcc -v
    gcc version 11.1.0 (g5964b5cd727)

2. Configure and build NuttX::

    $ mkdir ./nuttxspace
    $ cd ./nuttxspace
    $ git clone https://github.com/apache/nuttx.git nuttx
    $ git clone https://github.com/apache/nuttx-apps.git apps
    $ cd nuttx
    $ make distclean
    $ ./tools/configure.sh hpm6750evk2:nsh
    $ make menuconfig
    $ make V=1

3. Download and install openocd::

    Download hpmicro sdk_env, openocd in the path: sdk_env/tools/openocd


4. Debug the NuttX with openocd::

    picocom -b 115200 /dev/ttyACM0

  When using fireDAP, command as follows. Those cfg files in the path: ``sdk_env/hpm_sdk/boards/openocd``::

    $ openocd -f probes/cmsis_dap.cfg -f soc/hpm6750-single-core.cfg -f boards/hpm6750evk2.cfg

    $ riscv32-unknown-elf-gdb ./nuttx
    (gdb) target extended-remote [ip_addr]:3333
    (gdb) load
    (gdb) c

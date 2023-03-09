1. Download and install toolchain

  $ curl https://github.com/hpmicro/riscv-gnu-toolchain/releases/tag/2022.05.15

2. Download and install openocd

  Download hpmicro sdk_env, openocd in the path: sdk_env/tools/openocd

3. Configure and build NuttX

  $ mkdir ./nuttxspace
  $ cd ./nuttxspace
  $ git clone https://github.com/apache/nuttx.git nuttx
  $ git clone https://github.com/apache/nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh hpm6750evk2:nsh
  $ make menuconfig
  $ make V=1

  Note: make menuconfig to config toolchain
  ==================
    To switch GNU riscv64 toolchain to GNU riscv32 toolchain, the following option must be selected:

    System Type  --->
        Toolchain Selection   --->
            [ ] Generic GNU RV64 toolchain
            [x] Generic GNU RV32 toolchain

    Make sure HPMicro GNU riscv32 toolchain have been installed and be found in PATH.

4. Debug the nuttx with openocd and run

  $ picocom -b 115200 /dev/ttyACM0

  When using fireDAP, command as follows. Those cfg files in the path: sdk_env/hpm_sdk/boards/openocd.
  $ openocd -f probes/cmsis_dap.cfg -f soc/hpm6750-single-core.cfg -f boards/hpm6750evk2.cfg

  $ riscv32-unknown-elf-gdb ./nuttx
  (gdb) target extended-remote [ip_addr]:3333
  (gdb) load
  (gdb) c

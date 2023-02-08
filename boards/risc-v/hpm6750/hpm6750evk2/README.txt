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
  $ make V=1

4. Debug the nuttx with openocd and run

  $ picocom -b 115200 /dev/ttyACM0

  When using fireDAP, command as follows. Those cfg files in the path: sdk_env/hpm_sdk/boards/openocd.
  $ openocd -f probes/cmsis_dap.cfg -f soc/hpm6750-single-core.cfg -f boards/hpm6750evk2.cfg

  $ riscv64-unknown-elf-gdb ./nuttx
  (gdb) target extended-remote [ip_addr]:3333
  (gdb) load
  (gdb) c

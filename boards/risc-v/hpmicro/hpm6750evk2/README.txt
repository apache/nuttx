1. Verification environment

  Ubuntu/Bash 22.04 LTS shell under Windows 11.

2. Download and install toolchain

  $ curl https://github.com/hpmicro/riscv-gnu-toolchain/releases/tag/2022.05.15

3. Download and install openocd

  Download hpmicro sdk_env, openocd in the path: sdk_env/tools/openocd

4. Configure and build NuttX

  $ cd nuttxspace/nuttx
  $ make distclean
  $ ./tools/configure.sh hpm6750evk2:nsh
  $ make V=1

  note:
  [1] Default linker file is flash_xip.ld, you can config to ram.ld or flash_sdram_xip.ld.

5. Flash the nuttx with openocd and run

  $ picocom -b 115200 /dev/ttyACM0

  When using fireDAP, command as follows. Those cfg files in the path: sdk_env/hpm_sdk/boards/openocd.
  $ openocd -f probes/cmsis-dap.cfg -f soc/hpm6750-single-core.cfg -f boards/hpm6750evk2.cfg

  $ riscv32-unknown-elf-gdb ./nuttx
  (gdb) target remote [ip_addr]:3333
  (gdb) load nuttx
  (gdb) c

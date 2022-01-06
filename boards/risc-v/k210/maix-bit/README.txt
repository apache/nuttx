1. Download and install toolchain and openocd-k210

  $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz
  $ export PATH=$PATH:/$TOOL_CHAIN_PATH/bin

2. Build openocd-k210

  $ git clone https://github.com/kendryte/openocd-kendryte
  $ cd openocd-kendryte
  $ ./bootstrap & ./configure & make

3. Configure and build NuttX

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://github.com/apache/incubator-nuttx.git nuttx
  $ git clone https://github.com/apache/incubator-nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh maix-bit:nsh
  $ make V=1

4. Download and run the nuttx from SRAM (not SPI-Flash)

  $ picocom -b 115200 /dev/ttyUSB0
  $ sudo ./src/openocd -s ./tcl -f ./tcl/kendryte.cfg -m 0
  $ riscv64-unknown-elf-gdb ./nuttx
  (gdb) target extended-remote :3333
  (gdb) load nuttx
  (gdb) c

5. Write nuttx.bin to SPI-Flash

  $ pip3 install kflash
  $ kflash -p /dev/ttyUSB0 -b 1500000 ./nuttx/nuttx.bin

  NOTE: The kflash_gui is not recommended because it's unstable

6. TODO

  Support peripherals such as GPIO/SPI/I2C/...
  Support FPU
  Support RISC-V U-mode including memory protection

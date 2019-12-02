1. Download and install toolchain

  $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz

2. Build and install qemu

  $ git clone https://github.com/qemu/qemu
  $ cd qemu
  $ ./configure --target-list=riscv32-softmmu
  $ make
  $ sudo make install

3. Configure and build NuttX

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://bitbucket.org/nuttx/nuttx.git
  $ git clone https://bitbucket.org/nuttx/apps.git
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh hifive1-revb:nsh
  $ make V=1

4. Run the nuttx with qemu

  $ qemu-system-riscv32 -nographic -machine sifive_e -kernel ./nuttx

5. TODO

  Configure PLL and UART divisor
  Run nuttx on HiFive1-Rev.B board
  Support GPIO/SPI/I2C/RTC/WDT/PWM
  Support RISC-V User mode

1. Download and install toolchain

  $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz

2. Download and install J-Link Software Documentation Pack

  https://www.segger.com/downloads/jlink/

  $ sudo apt install JLink_Linux_V656b_x86_64.deb

3. Configure and build NuttX

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://github.com/apache/nuttx.git nuttx
  $ git clone https://github.com/apache/nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh hifive1-revb:nsh
  $ make V=1

4. Flash the nuttx with J-Link and run

  $ picocom -b 115200 /dev/ttyACM0

  $ /opt/SEGGER/JLink_V656b/JLinkGDBServer -device FE310

  $ riscv64-unknown-elf-gdb ./nuttx
  (gdb) target extended-remote:2331
  (gdb) load nuttx
  (gdb) c

5. TODO

  Support GPIO/SPI/I2C/RTC/WDT/PWM
  Support RISC-V User mode

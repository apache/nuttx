1. Download and install toolchain

  $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz

2. Follow instruction on https://github.com/icebreaker-fpga/icebreaker-litex-examples to build the vexriscv softcore fpga gateware
   and flash to icebreaker

3. Flash the gateware with ./icebreaker.py --flash

3. Configure and build NuttX

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://github.com/apache/incubator-nuttx.git nuttx
  $ git clone https://github.com/apache/incubator-nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh icebreaker:nsh
  $ make V=1

4. Program the flash with Nuttx $ iceprog -o 0x00040000 nuttx.bin

5. Run $ screen /dev/ttyUSB0 115200

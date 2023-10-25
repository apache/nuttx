1. Download and install toolchain

  $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz

2. Follow instruction on https://github.com/enjoy-digital/litex to build the vexriscv softcore fpga gateware
   and flash to arty_a7 board

  $ cd litex-boards/litex_boards/targets
  $ ./digilent_arty.py --with-ethernet --with-sdcard --uart-baudrate 1000000 --cpu-type=vexriscv --cpu-variant=secure --build --load --flash

3. Configure and build NuttX

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://github.com/apache/nuttx.git nuttx
  $ git clone https://github.com/apache/nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh arty_a7:nsh
  $ make V=1

4. Setup tftp server on your laptop, copy nuttx.bin to your tftpboot directory and change its name to boot.bin

5. Setup the wire connection(uart and tftp) between your board and laptop

6. Run  $ minicom -b 1000000 /dev/ttyUSB1 (the default baudrate on litex vexriscv is 1e6)
   when you see the bios prompt "litex>", type "netboot" and enter soon comes the nsh prompt

7. TODO

  Support GPIO/SPI/I2C/RTC/WDT/PWM
  Support RISC-V User mode

1. Download and install toolchain

  $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz

2. Download Bouffalo lab flash tools

  $ git clone https://github.com/bouffalo/flash_tools.git

3. Configure and build NuttX

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://github.com/apache/incubator-nuttx.git nuttx
  $ git clone https://github.com/apache/incubator-nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh bl602evb:nsh
  $ make -j

4. Connect bl602 and computer via USB

5. Run flash tools, select the nuttx.bin generated in the previous step in the
   Firmware bin field, and refer to the document for the settings of the remaining fields.
   Click the download button to download bin to bl602.

6. Run  $ picocom -b 115200 /dev/ttyUSB0 (If you see garbled characters at the begining, it is because 
   boot2 outputs some log at 2M baud rate), then you will see "nuttx>" prompt.

How to download to bl602 can refer to the docs in the https://github.com/bouffalolab/bl_iot_sdk repository.

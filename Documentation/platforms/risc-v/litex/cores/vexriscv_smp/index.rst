==================
VexRISCV_SMP Core
==================

The vexrisc_smp core supports a two-pass build, producing the kernel (nuttx.bin), and a number of applications, 
compiled into the apps/bin directory. In the standard configuration, the applications are loaded to the FPGA in a RAMdisk. 
Although, for custom boards this could be extended to loading from SDCards, flash, or other mediums.

Building
--------

Nuttx uses openSBI to configure and prepare the vexriscv_smp core. With this configuration, 
the Nuttx kernel is a binary payload for OpenSBI. The configuration used is
identical to that used for Linux on Litex project (https://github.com/litex-hub/linux-on-litex-vexriscv).

To build OpenSBI::

   $ git clone https://github.com/litex-hub/opensbi --branch 0.8-linux-on-litex-vexriscv
   $ cd opensbi
   $ make CROSS_COMPILE=riscv64-unknown-elf- PLATFORM=litex/vexriscv
   $ cp build/platform/litex/vexriscv/firmware/fw_jump.bin ../opensbi.bin"

Build the Nuttx kernel::

   $ ./tools/configure.sh arty_a7:knsh
   $ make

Build the loadable applications::

   $ make export -j16
   $ cd ../apps
   $ make ./tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz
   $ make import

Generate a romfs to be loaded to the FPGA as a ramdisk::

   $ cd nuttx
   $ genromfs -f romfs.img -d ../apps/bin -V "NuttXBootVol"

Booting
--------

Create a file, 'boot.json' in the Nuttx root directory, with the following content::

  {
    "romfs.img":   "0x40C00000",
    "nuttx.bin":   "0x40000000",
    "board.dtb":   "0x41ec0000",
    "opensbi.bin": "0x40f00000"
  }

Load the application over serial with::

  litex_term --images=boot.json --speed=1e6 /dev/ttyUSB0

Update the baud rate and serial port to suit your configuration.
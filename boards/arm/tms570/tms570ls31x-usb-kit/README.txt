README
======

  This README provides some information about the port of NuttX to the TI
  Hercules TMS570LS31x USB Kit featuring the Hercules TMS570LS3137ZWT chip.

Contents
========

  - Status
  - Toolchain
  - LEDs and Buttons
  - Serial Console
  - Debugging
  - Configurations

Status
======

  2017-10-18:
  The basic port to the TMS570 is complete. The NSH with basic commands is
  up and running. There is support for SCI communication and RTI.

Toolchain
=========

  Build Platform
  --------------
  All of these configurations are set up to build with Ubuntu.

  Endian-ness Issues
  ------------------
  I used a version of the NuttX buildroot toolchain that can be built like
  this:

    cd buildroot/
    cp boards/cortexr4f-eabi-defconfig-4.8.5 .config
    make oldconfig
    make

  Before building the compiler I installed the following packages which
  are needed for the compiler build.

  # install or update all apt-get dependencies
  sudo apt-get update
  sudo apt-get upgrade
  sudo apt-get install gcc                   # not cross
  sudo apt-get install g++
  sudo apt-get install make
  sudo apt-get install bison
  sudo apt-get install flex
  sudo apt-get install gawk
  sudo apt-get install libgmp3-dev
  sudo apt-get install libmpfr-dev libmpfr-doc libmpfr4 libmpfr4-dbg
  sudo apt-get install mpc
  sudo apt-get install texinfo               # optional
  sudo apt-get install libcloog-isl-dev      # optional
  sudo apt-get install build-essential
  sudo apt-get install glibc-devel
  sudo apt-get -y install gcc-multilib libc6-i386

Serial Console
==============

  This TMS570ls3137 has a single SCI and one combined SCI/LIN interface.
  The SCI_RX and TX pins are connected to the FTDI chip which provides a
  virtual COM port for the usb kit.

Debugging
=========

  I used the On Board Debugger.

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each TMS570LS31X Usb Kit configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh tms570ls31x-usb-kit:<subdir>

  Before building, make sure the PATH environment variable includes the
  correct path to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make oldconfig
    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

Configuration sub-directories
-----------------------------

  nsh:

    Configures the NuttShell (nsh) located at examples/nsh.

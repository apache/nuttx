README
======

  This README file discusses the port of NuttX to the Raspberry Pi Zero.
  The Raspberry Pi Zero is based on the Broadcom BCM2835 CPU.  This is the
  same Broadcom chip used in the Raspberry Pi Model A, B, B+, the Compute
  Module.

STATUS
======

  2017-10-09:  This is a work in progress.  The port is not complete and
    will not build to completion.  Missing logic includes:

    - GPIO support,
    - System timer initialization, and
    - Serial driver.

  2017-10-19:  Much of that logic is in place but there are a few things
    still missing.

    - Mini-UART Baud divisor calculation,
    - Start-up logic.  My understand from what I have read on the internet
      is the PiZero OS starts up in hypervisor mode with the MMU and I- and
      D-Caches enabled.  This probaby means that the standard, classic ARM
      startup logic at at arch/arm/src/arm/up_head.S will need to be replaced
      with some custom logic.

    No testing has yet been performed.

Basic Setup
===========

    +------------+
    |            |
    |         +--+
    |     USB |  | <----------------------------> USB Power Source
    |         +--+           +------------+
    |            |           |         +--+
    |         +--+           |         |  | <---> Keyboard
    |     USB |  | <-------->|   USB   +--+
    |         +--+           |   HUB      |
    |            |           |         +--+
    |            |           |         |  | <---> Mouse
    |            |           |         +--+
    |            |           +------------+
    |    Mini +--+
    |    HDMI |  | <----------------------------> Monitor/TV
    |         +--+
    |            |
    +------------+

  You might be able to use the hub to power the Pi Zero, but I was not
  able to do that; my hub would not switch on power until it was enumerated
  by the host (the Raspberry Pi Zero), but then I could not power the Pi
  with the hub because it was not providing power.  Chicken'n'Egg.

NuttX Boot Sequence
===================

  To be provided.

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each Raspberry Pi Zero configuration is maintained in a sub-directory
  and can be selected as follow:

    tools/configure.sh [OPTIONS] pizero/<subdir>

  Where [OPTIONS] include -l to configure for a Linux host platform and
  -c means to configure for a Windows Cygwin host platform.  -h will give
  you the list of all options.

  Before building, make sure the PATH environment variable includes the
  correct path to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

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

  Configuration Sub-directories
  -----------------------------

  nsh:

    This simple configuration directory provide the NuttShell (NSH).

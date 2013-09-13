README
======

This README discusses issues unique to NuttX configurations for the
maple board from LeafLabs (http://leaflabs.com).

Microprocessor: 32-bit ARM Cortex M3 at 72MHz STM32F103RBT6 (STM32F103CBT6 for mini version)
Memory:         120 KB Flash and 20 KB SRAM
I/O Pins Out:   43 (34 for mini version)
ADCs:           9 (at 12-bit resolution)
Peripherals:    4 timers, 2 I2Cs, 2 SPI ports, 3 USARTs
Other:          Sleep, stop, and standby modes; serial wire debug and JTAG interfaces

Please see below link for a list of maple devices and documentations.

    http://leaflabs.com/devices
    http://leaflabs.com/docs

This config supports Maple and Maple Mini.

Contents
========

  - Development Environment
  - DFU

Development Environment
=======================

  Either Linux (recommended), Mac or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment because the Raisonance R-Link emulatator and some RIDE7 development tools
  were used and those tools works only under Windows.

DFU
===

  The linker files in these projects can be configured to indicate that you
  will be loading code using STMicro built-in USB Device Firmware Upgrade (DFU)
  loader or via some JTAG emulator.  You can specify the DFU bootloader by
  adding the following line:

    CONFIG_STM32_DFU=y

  to your .config file. Most of the configurations in this directory are set
  up to use the DFU loader.

  If CONFIG_STM32_DFU is defined, the code will not be positioned at the beginning
  of FLASH (0x08000000) but will be offset to 0x08005000.  This offset is needed
  to make space for the DFU loader and 0x08005000 is where the DFU loader expects
  to find new applications at boot time.  If you need to change that origin for some
  other bootloader, you will need to edit the file(s) ld.script.dfu for each
  configuration. In LeafLabs case, we are using maple bootloader:

      http://leaflabs.com/docs/bootloader.html

  For Linux or Mac:
  ----------------

  While on Linux or Mac, we can use dfu-util to upload nuttx binary.

  1. Make sure we have installed dfu-util. (From yum, apt-get or build from source.)
  2. Start the DFU loader (bootloader) on the maple board. You do this by
     resetting the board while holding the "Key" button. Windows should
     recognize that the DFU loader has been installed.
  3. Flash the nuttx.bin to the board use dfu-util. Here's an example:

      $ dfu-util -a1 -d 1eaf:0003 -D nuttx.bin -R

  For anything not clear, we can refer to LeafLabs official document:

    http://leaflabs.com/docs/unix-toolchain.html

  For Windows:
  -----------

  The DFU SE PC-based software is available from the STMicro website,
  http://www.st.com.  General usage instructions:

  1. Connect the maple board to your computer using a USB
     cable.
  2. Start the DFU loader on the maple board. You do this by
     resetting the board while holding the "Key" button. Windows should
     recognize that the DFU loader has been installed.
  3. Run the DFU SE program to load nuttx.bin into FLASH.

  What if the DFU loader is not in FLASH? The loader code is available
  inside of the Demo dirctory of the USBLib ZIP file that can be downloaded
  from the STMicro Website. You can build it using RIDE (or other toolchains);
  you will need a JTAG emulator to burn it into FLASH the first time.

  In order to use STMicro's built-in DFU loader, you will have to get
  the NuttX binary into a special format with a .dfu extension. The
  DFU SE PC_based software installation includes a file "DFU File Manager"
  conversion program that a file in Intel Hex format to the special DFU
  format. When you successfully build NuttX, you will find a file called
  nutt.hex in the top-level directory. That is the file that you should
  provide to the DFU File Manager. You will end up with a file called
  nuttx.dfu that you can use with the STMicro DFU SE program.

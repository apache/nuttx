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
  - Configurations

Development Environment
=======================

  Either Linux (recommended), Mac or Cygwin on Windows can be used for the development
  environment.  The source has been built only using the GNU toolchain (see below).
  Other toolchains will likely cause problems.

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
  inside of the Demo directory of the USBLib ZIP file that can be downloaded
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

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each Maple configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh maple:<subdir>

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

    This configuration directory provide the basic NuttShell (NSH).
    A serial console is provided on USART1.

    NOTES:
    1. Currently configured for the STM32F103CB.  But this is easily
       reconfigured:

       CONFIG_ARCH_CHIP_STM32F103RB=n
       CONFIG_ARCH_CHIP_STM32F103CB=y

    2. Support for the I2C tool has been disabled, but can be restored
       with following configure options:

       System Type -> Peripherals
         CONFIG_STM32_I2C1=y
         CONFIG_STM32_I2C2=y
         CONFIG_STM32_I2CTIMEOSEC=1
         CONFIG_STM32_I2CTIMEOMS=500
         CONFIG_STM32_I2CTIMEOTICKS=500

       Drivers
        CONFIG_I2C=y

       Applications -> System Add-Ons
         CONFIG_SYSTEM_I2CTOOL=y
         CONFIG_I2CTOOL_MINBUS=1
         CONFIG_I2CTOOL_MAXBUS=2
         CONFIG_I2CTOOL_MINADDR=0x0
         CONFIG_I2CTOOL_MAXADDR=0xf0
         CONFIG_I2CTOOL_MAXREGADDR=0xff
         CONFIG_I2CTOOL_DEFFREQ=100000

  nx:

    This configuration has been used to bring up the  Sharp Memory LCD
    on a custom board.  This NX configuration was used for testing that
    LCD.  Debug output will appear on USART1.

    NOTES:
    1. Currently configured for the STM32F103CB.  But this is easily
       reconfigured:

       CONFIG_ARCH_CHIP_STM32F103RB=n
       CONFIG_ARCH_CHIP_STM32F103CB=y

    2. You won't be able to buy a Sharp Memory LCD to use with your
       Maple.  If you want one, you will have to make one yourself.

  usbnsh:

    This is an alternative NuttShell (NSH) configuration that uses a USB
    serial console for interaction.

    NOTES:
    1. Currently configured for the STM32F103CB.  But this is easily
       reconfigured:

       CONFIG_ARCH_CHIP_STM32F103RB=n
       CONFIG_ARCH_CHIP_STM32F103CB=y

    2. Support for the I2C tool has been disabled, but can be restored
       with following configure options:

       System Type -> Peripherals
         CONFIG_STM32_I2C1=y
         CONFIG_STM32_I2C2=y
         CONFIG_STM32_I2CTIMEOSEC=1
         CONFIG_STM32_I2CTIMEOMS=500
         CONFIG_STM32_I2CTIMEOTICKS=500

       Drivers
        CONFIG_I2C=y

       Applications -> System Add-Ons
         CONFIG_SYSTEM_I2CTOOL=y
         CONFIG_I2CTOOL_MINBUS=1
         CONFIG_I2CTOOL_MAXBUS=2
         CONFIG_I2CTOOL_MINADDR=0x0
         CONFIG_I2CTOOL_MAXADDR=0xf0
         CONFIG_I2CTOOL_MAXREGADDR=0xff
         CONFIG_I2CTOOL_DEFFREQ=100000

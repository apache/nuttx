README
======

  This README file describes the port of NuttX to the SAMA5D3-Xplained
  development board. This board features the Atmel SAMA5D36 microprocessor.
  See http://www.atmel.com/devices/sama5d36.aspx for further information.

    PARAMETER                 SAMA5D36
    ------------------------- -------------
    Pin Count                 324
    Max. Operating Frequency  536 MHz
    CPU                       Cortex-A5
    Max I/O Pins              160
    Ext Interrupts            160
    USB Transceiver           3
    USB Speed                 Hi-Speed
    USB Interface             Host, Device
    SPI                       6
    TWI (I2C)                 3
    UART                      7
    CAN                       2
    LIN                       4
    SSC                       2
    Ethernet                  2
    SD / eMMC                 3
    Graphic LCD               Yes
    Camera Interface          Yes
    ADC channels              12
    ADC Resolution (bits)     12
    ADC Speed (ksps)          1000
    Resistive Touch Screen    Yes
    Crypto Engine             AES/DES/
                              SHA/TRNG
    SRAM (Kbytes)             128
    External Bus Interface    1
    DRAM Memory               DDR2/LPDDR,
                              SDRAM/LPSDR
    NAND Interface            Yes
    Temp. Range (deg C)       -40 to 105
    I/O Supply Class          1.8/3.3
    Operating Voltage (Vcc)   1.08 to 1.32
    FPU                       Yes
    MPU / MMU                 No/Yes
    Timers                    6
    Output Compare channels   6
    Input Capture Channels    6
    PWM Channels              4
    32kHz RTC                 Yes
    Packages                  LFBGA324_A

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - Loading Code into SRAM with J-Link
  - Writing to FLASH using SAM-BA
  - Running NuttX from SDRAM
  - Buttons and LEDs
  - Serial Console
  - Networking
  - AT25 Serial FLASH
  - HSMCI Card Slots
  - Auto-Mounter
  - USB Ports
  - USB High-Speed Device
  - USB High-Speed Host
  - SDRAM Support
  - NAND Support
  - I2C Tool
  - CAN Usage
  - SAMA5 ADC Support
  - SAMA5 PWM Support
  - RTC
  - Watchdog Timer
  - TRNG and /dev/random
  - Tickless OS
  - I2S Audio Support
  - Shields
  - SAMA5D3-Xplained Configuration Options
  - Configurations
  - To-Do List

Development Environment
=======================

  Several possible development environments may be used:

  - Linux or macOS native
  - Cygwin under Windows
  - MinGW + MSYS under Windows
  - Windows native (with GNUMake from GNUWin32).

  All testing has been performed using Cygwin under Windows.

  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems.

GNU Toolchain Options
=====================

  The NuttX make system will support the several different toolchain options.

  All testing has been conducted using the CodeSourcery GCC toolchain.  To use
  a different toolchain, you simply need to add change to one of the following
  configuration options to your .config (or defconfig) file:

    CONFIG_ARM_TOOLCHAIN_BUILDROOT=y  : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_ARM_TOOLCHAIN_GNU_EABI=y   : Generic GCC ARM EABI toolchain

  NOTE about Windows native toolchains
  ------------------------------------

  There are several limitations to using a Windows based toolchain in a
  Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath'
     utility but you might easily find some new path problems.  If so, check
     out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic
     links are used in NuttX (e.g., include/arch).  The make system works
     around these problems for the Windows tools by copying directories
     instead of linking them.  But this can also cause some confusion for
     you:  For example, you may edit a file in a "linked" directory and find
     that your changes had no effect.  That is because you are building the
     copy of the file in the "fake" symbolic directory.  If you use a\
     Windows toolchain, you should get in the habit of making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

IDEs
====

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project.

  Makefile Build
  --------------
  Under Eclipse, it is pretty easy to set up an "empty makefile project" and
  simply use the NuttX makefile to build the system.  That is almost for free
  under Linux.  Under Windows, you will need to set up the "Cygwin GCC" empty
  makefile project in order to work with Windows (Google for "Eclipse Cygwin" -
  there is a lot of help on the internet).

  Native Build
  ------------
  Here are a few tips before you start that effort:

  1) Select the toolchain that you will be using in your .config file
  2) Start the NuttX build at least one time from the Cygwin command line
     before trying to create your project.  This is necessary to create
     certain auto-generated files and directories that will be needed.
  3) Set up include paths:  You will need include/, arch/arm/src/sam34,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/sam34/sam_vectors.S.  You may need to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by an IDE.

NuttX EABI "buildroot" Toolchain
================================

  A GNU GCC-based toolchain is assumed.  The PATH environment variable should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  Bitbucket download site (https://bitbucket.org/nuttx/buildroot/downloads/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1.  You must have already configured NuttX in <some-dir>/nuttx.

      tools/configure.sh sama5d3-xplained:<sub-dir>

  2.  Download the latest buildroot package into <some-dir>

  3.  unpack the buildroot tarball.  The resulting directory may
      have versioning information on it like buildroot-x.y.z.  If so,
      rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4.  cd <some-dir>/buildroot

  5.  Copy the configuration file from the boards/ sub-directory to the
      top-level build directory:

      cp boards/cortexa8-eabi-defconfig-4.8.2 .config

  6a. You may wish to modify the configuration before you build it.  For
      example, it is recommended that you build the kconfig-frontends tools,
      generomfs, and the NXFLAT tools as well.  You may also want to change
      the selected toolchain.  These reconfigurations can all be done with

      make menuconfig

  6b. If you chose to make the configuration with no changes, then you
      should still do the following to make certain that the build
      configuration is up-to-date:

      make oldconfig

  7. make

  8. Make sure that the PATH variable includes the path to the newly built
     binaries.

  See the file boards/README.txt in the buildroot source tree.  That has more
  details PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

NXFLAT Toolchain
================

  If you are *not* using the NuttX buildroot toolchain and you want to use
  the NXFLAT tools, then you will still have to build a portion of the buildroot
  tools -- just the NXFLAT tools.  The buildroot with the NXFLAT tools can
  be downloaded from the NuttX Bitbucket download site
  (https://bitbucket.org/nuttx/nuttx/downloads/).

  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured NuttX in <some-dir>/nuttx.

     tools/configure.sh sama5d3-xplained:<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp boards/cortexm3-defconfig-nxflat .config

  6. make oldconfig

  7. make

  8. Make sure that the PATH variable includes the path to the newly built
     NXFLAT binaries.

  NOTE:  There are some known incompatibilities with 4.6.3 EABI toolchain
  and the NXFLAT tools.  See the top-level TODO file (under "Binary
  loaders") for more information about this problem. If you plan to use
  NXFLAT, please do not use the GCC 4.6.3 EABI toochain.

Loading Code into SRAM with J-Link
==================================

  Loading code with the Segger tools and GDB
  ------------------------------------------

    1) Change directories into the directory where you built NuttX.
    2) Start the GDB server and wait until it is ready to accept GDB
       connections.
    3) Then run GDB like this:

         $ arm-none-eabi-gdb
         (gdb) target remote localhost:2331
         (gdb) mon reset
         (gdb) load nuttx
         (gdb) breakpoint nsh_main
         (gdb) continue
         Continuing.

         Breakpoint 1, nsh_main (argc=1, argv=0x2007757c) at nsh_main.c:218
         218	  sched_getparam(0, &param);
         (gdb) continue
         (gdb) ... debugging ...

  Loading code using J-Link Commander
  ----------------------------------

    J-Link> r
    J-Link> loadbin <file> <address>
    J-Link> setpc <address of __start>
    J-Link> ... start debugging ...

Writing to FLASH using SAM-BA
=============================

  Assumed starting configuration:

    1. You have installed the J-Link CDC USB driver (Windows only, there is
        no need to install a driver on any regular Linux distribution),
    2. You have the USB connected to DBGU port (J23)
    3. Terminal configuration:  115200 8N1

  Using SAM-BA to write to FLASH:

    1. Exit the terminal emulation program and remove the USB cable from
       the DBGU port (J23)
    2. Connect the USB cable to the device USB port (J6)
    3. JP9 must open (BMS == 1) to boot from on-chip Boot ROM.
    4. Press and maintain PB4 CS_BOOT button and power up the board.  PB4
       CS_BOOT button prevents booting from Nand or serial Flash by
       disabling Flash Chip Selects after having powered the board, you can
       release the PB4 BS_BOOT button.
    5. On Windows you may need to wait for a device driver to be installed.
    6. Start the SAM-BA application, selecting (1) the correct USB serial
       port, and (2) board = at91sama5d3-xplained.
    7. The SAM-BA menu should appear.
    8. Select the FLASH bank that you want to use and the address to write
       to and "Execute"
    9. When you are finished writing to FLASH, remove the USB cable from J6
       and re-connect the serial link on USB CDC / DBGU connector (J23) and
       re-open the terminal emulator program.
    10. Power cycle the board.

Running NuttX from SDRAM
========================

  NuttX may be executed from SDRAM.  But this case means that the NuttX
  binary must reside on some other media (typically NAND FLASH, Serial
  FLASH, or, perhaps even a TFTP server).  In these cases, an intermediate
  bootloader such as U-Boot or Barebox must be used to configure the
  SAMA5D3 clocks and SDRAM and then to copy the NuttX binary into SDRAM.

    - NuttX Configuration
    - Boot sequence
    - NAND FLASH Memory Map
    - Programming the AT91Boostrap Binary
    - Programming U-Boot
    - Load NuttX with U-Boot on AT91 boards

  TODO:  Some drivers may require some adjustments to run from SDRAM.  That
  is because in this case macros like BOARD_MCK_FREQUENCY are not constants
  but are instead function calls:  The MCK clock frequency is not known in
  advance but instead has to be calculated from the bootloader PLL configuration.
  See the TODO list at the end of this file for further information.

NuttX Configuration
-------------------

  In order to run from SDRAM, NuttX must be built at origin 0x20008000 in
  SDRAM (skipping over SDRAM memory used by the bootloader).  The following
  configuration option is required:

    CONFIG_SAMA5_BOOT_SDRAM=y
    CONFIG_BOOT_RUNFROMSDRAM=y

  These options tell the NuttX code that it will be booting and running from
  SDRAM.  In this case, the start-logic will do to things:  (1) it will not
  configure the SAMA5D3 clocking.  Rather, it will use the clock configuration
  as set up by the bootloader.  And (2) it will not attempt to configure the
  SDRAM.  Since NuttX is already running from SDRAM, it must accept the SDRAM
  configuration as set up by the bootloader.

Boot sequence
-------------

  Reference: http://www.at91.com/linux4sam/bin/view/Linux4SAM/GettingStarted

  Several pieces of software are involved to boot a Nutt5X into SDRAM.  First
  is the primary bootloader in ROM which is in charge to check if a valid
  application is present on supported media (NOR FLASH, Serial DataFlash,
  NAND FLASH, SD card).

  The boot sequence of linux4SAM is done in several steps :

  1. The ROM bootloader checks if a valid application is present in FLASH
     and if it is the case downloads it into internal SRAM.  This program
     is usually a second level bootloader called AT91BootStrap.

  2. AT91Bootstrap is the second level bootloader. It is in charge of the
     hardware configuration.  It downloads U-Boot / Barebox binary from
     FLASH to SDRAM / DDRAM and starts the third level bootloader
     (U-Boot / Barebox)

    (see http://www.at91.com/linux4sam/bin/view/Linux4SAM/AT91Bootstrap).

  3. The third level bootloader is either U-Boot or Barebox.  The third
     level bootloader is in charge of downloading NuttX binary from FLASH,
     network, SD card, etc.  It then starts NuttX.

   4. Then NuttX runs from SDRAM

NAND FLASH Memory Map
---------------------

  Reference: http://www.at91.com/linux4sam/bin/view/Linux4SAM/GettingStarted

  0x0000:0000 - 0x0003:ffff: AT91BootStrap
  0x0004:0000 - 0x000b:ffff: U-Boot
  0x000c:0000 - 0x000f:ffff: U-Boot environment
  0x0010:0000 - 0x0017:ffff: U-Boot environment redundant
  0x0018:0000 - 0x001f:ffff: Device tree (DTB)
  0x0020:0000 - 0x007f:ffff: NuttX
  0x0080:0000 - end:         Available for use as a NAND file system

Programming the AT91Boostrap Binary
-----------------------------------

  Reference: http://www.at91.com/linux4sam/bin/view/Linux4SAM/AT91Bootstrap

  This section describes how to program AT91Bootstrap binary into the boot
  media with SAM-BA tool using NandFlash as boot media.

  1. Get AT91BootStrap binaries.  Build instructions are available here:

       http://www.at91.com/linux4sam/bin/view/Linux4SAM/AT91Bootstrap#Build_AT91Bootstrap_from_sources

     A pre-built AT91BootStrap binary is available here:

      ftp://www.at91.com/pub/at91bootstrap/AT91Bootstrap3.6.1/sama5d3_xplained-nandflashboot-uboot-3.6.1.bin

  2. Start the SAM-BA GUI Application:

     - Connect the USB Device interface to your host machine using the USB
       Device Cable.
     - Make sure that the chip can execute the SAM-BA Monitor.
     - Start SAM-BA GUI application.
     - Select the board in the drop-down menu and choose the USB connection.

  3. In the SAM-BA GUI Application:

     - Choose the "NandFlash" tab in the SAM-BA GUI interface.
     - Initialize the NandFlash by choosing the "Enable NandFlash" action in
       the Scripts rolling menu, then press "Execute" button.
     - Erase the NandFlash device by choosing the "Erase All" action, then
       press "Execute" button.
     - Enable the PMECC by choosing the "Enable OS PMECC parameters" action,
       then press "Execute" button.

         PMECC
         Number of sectors per page: 4
         Spare Size: 64
         Number of ECC bits required: 4
         Size of the ECC sector: 512
         ECC offset: 36

   - Choose "Send Boot File" action, then press Execute button to select the
     at91bootstrap binary file and to program the binary to the NandFlash.
   - Close SAM-BA, remove the USB Device cable.

Programming U-Boot
-------------------

  Reference http://www.at91.com/linux4sam/bin/view/Linux4SAM/U-Boot

  1. Get U-Boot Binaries.  Build instructions are available here:

     http://www.at91.com/linux4sam/bin/view/Linux4SAM/U-Boot#Build_U_Boot_from_sources

     A pre-Built binary image is available here:

     ftp://www.at91.com/pub/uboot/u-boot-v2013.07/u-boot-sama5d3_xplained-v2013.07-at91-r1.bin

  2. Start the SAM-BA GUI Application:

     - Connect the USB Device interface to your host machine using the USB
       Device Cable.
     - Make sure that the chip can execute the SAM-BA Monitor.
     - Start SAM-BA GUI application.
     - Select the board in the drop-down menu and choose the USB connection.

  3. In the SAM-BA GUI Application:

     - Choose the NandFlash tab in the SAM-BA GUI interface.
     - Initialize the NandFlash by choosing the "Enable NandFlash" action in
       the Scripts rolling menu, then press Execute button.
     - Enable the PMECC by choosing the "Enable OS PMECC parameters" action,
       then press Execute button.

         PMECC
         Number of sectors per page: 4
         Spare Size: 64
         Number of ECC bits required: 4
         Size of the ECC sector: 512
         ECC offset: 36

     - Press the "Send File Name" Browse button
     - Choose u-boot.bin binary file and press Open
     - Enter the proper address on media in the Address text field:
       0x00040000
     - Press the "Send File" button
     - Close SAM-BA, remove the USB Device cable.

  You should now be able to interrupt with U-Boot via the DBGU interface.

Load NuttX with U-Boot on AT91 boards
-------------------------------------

  Reference http://www.at91.com/linux4sam/bin/view/Linux4SAM/U-Boot

  Preparing NuttX image

    U-Boot does not support normal binary images.  Instead you have to
    create an uImage file with the mkimage tool which encapsulates kernel
    image with header information, CRC32 checksum, etc.

    mkimage comes in source code with U-Boot distribution and it is built
    during U-Boot compilation (u-boot-source-dir/tools/mkimage).  There
    are also sites where you can download pre-built mkimage binaries.  For
    example: http://www.trimslice.com/wiki/index.php/U-Boot_images

    See the U-Boot README file for more information.  More information is
    also available in the mkimage man page (for example,
    http://linux.die.net/man/1/mkimage).

    Command to generate an uncompressed uImage file (4) :

      mkimage -A arm -O linux -C none -T kernel -a 20008000 -e 20008000 \
        -n nuttx -d nuttx.bin uImage

    Where:

      -A arm: Set architecture to ARM
      -O linux: Select operating system. bootm command of u-boot changes
         boot method by os type.
      -T kernel: Set image type.
      -C none: Set compression type.
      -a 20008000:  Set load address.
      -e 20008000: Set entry point.
      -n nuttx: Set image name.
      -d nuttx.bin: Use image data from nuttx.bin.

    This will generate a binary called uImage.  If you have the path to
    mkimage in your PATH variable, then you can automatically build the
    uImage file by adding the following to your .config file:

      CONFIG_RAW_BINARY=y
      CONFIG_UBOOT_UIMAGE=y
      CONFIG_UIMAGE_LOAD_ADDRESS=0x20008000
      CONFIG_UIMAGE_ENTRY_POINT=0x20008040

    The uImage file can them be loaded into memory from a variety of sources
    (serial, SD card, JFFS2 on NAND, TFTP).

    STATUS:
      2014-4-1:  So far, I am unable to get U-Boot to execute the uImage
                 file.  I get the following error messages (in this case
                 trying to load from an SD card):

        U-Boot> fatload mmc 0 0x22000000 uimage
        reading uimage
        97744 bytes read in 21 ms (4.4 MiB/s)

        U-Boot> bootm 0x22000000
        ## Booting kernel from Legacy Image at 0x22000000 ...
           Image Name:   nuttx
           Image Type:   ARM Linux Kernel Image (uncompressed)
           Data Size:    97680 Bytes = 95.4 KiB
           Load Address: 20008000
           Entry Point:  20008040
           Verifying Checksum ... OK
           XIP Kernel Image ... OK
        FDT and ATAGS support not compiled in - hanging
        ### ERROR ### Please RESET the board ###

      This, however, appears to be a usable workaround:

        U-Boot> fatload mmc 0 0x20008000 nuttx.bin
        mci: setting clock 257812 Hz, block size 512
        mci: setting clock 257812 Hz, block size 512
        mci: setting clock 257812 Hz, block size 512
        gen_atmel_mci: CMDR 00001048 ( 8) ARGR 000001aa (SR: 0c100025) Command Time Out
        mci: setting clock 257812 Hz, block size 512
        mci: setting clock 22000000 Hz, block size 512
        reading nuttx.bin
        108076 bytes read in 23 ms (4.5 MiB/s)

        U-Boot> go 0x20008040
        ## Starting application at 0x20008040 ...

        NuttShell (NSH) NuttX-7.2
        nsh>

      It is possible to autoboot from the SD Card:

        1. Format an SD Card as FAT.
        2. Copy the file nuttx/boards/arm/sama5/sama5d3-xplained/boot/uImage file to the SD Card.
        3. Copy the file nuttx.bin you just compiled to the SD Card.
        4. Attach a 3.3V USB-serial adapter to the DEBUG console port.
        5. Open a serial terminal to the debug console. In Linux, do this:

          picocom -b 115200 /dev/ttyUSB0

        6. Press the RESET button. You should see a U-Boot prompt. Press a key to stop the booting process.
        7. Issue the following commands to U-Boot:

          U-Boot> setenv load_nuttx 'fatload mmc 0 0x20008000 nuttx.bin'
          U-Boot> setenv run_nuttx 'go 0x20008040'
          U-Boot> setenv boot_nuttx 'run load_nuttx; run run_nuttx'
          U-Boot> setenv bootcmd 'boot_nuttx'
          U-Boot> saveenv
          U-Boot> reset

        8. The board should now always boot to NuttX if you have the SD Card inserted.

  Loading through network

    On a development system, it is useful to get the kernel and root file
    system through the network. U-Boot provides support for loading
    binaries from a remote host on the network using the TFTP protocol.

    To manage to use TFTP with U-Boot, you will have to configure a TFTP
    server on your host machine. Check your distribution manual or Internet
    resources to configure a Linux or Windows TFTP server on your host:

      - U-Boot documentation on a Linux host:
        http://www.denx.de/wiki/view/DULG/SystemSetup#Section_4.6.

      - Another TFTP configuration reference:
        http://www.linuxhomenetworking.com/wiki/index.php/Quick_HOWTO_:_Ch16_:_Telnet%2C_TFTP%2C_and_xinetd#TFTP

    On the U-Boot side, you will have to setup the networking parameters:

     1. Setup an Ethernet address (MAC address)
        Check this U-Boot network BuildRootFAQ entry to choose a proper MAC
        address: http://www.denx.de/wiki/DULG/EthernetDoesNotWork

          setenv ethaddr 00:e0:de:ad:be:ef

     2. Setup IP parameters:
        The board ip address

          setenv ipaddr 10.0.0.2

        The server ip address where the TFTP server is running

          setenv serverip 10.0.0.1

     3. saving Environment to flash

          saveenv

     4. If Ethernet Phy has not been detected during former bootup, reset
        the board to reload U-Boot : the Ethernet address and Phy
        initialization shall be ok, now

     5. Download the NuttX uImage and the root file system to a ram location
       using the U-Boot tftp command (Cf. U-Boot script capability chapter).

     6. Launch NuttX issuing a bootm or boot command.

    If the board has both emac and gmac, you can use following to choose
    which one to use:

       setenv ethact macb0,gmacb0
       setenv ethprime gmacb0

  STATUS:
    2014-3-30:  These instructions were adapted from the Linux4SAM website
                but have not yet been used.

  Using JTAG
  ----------

  This description assumes that you have a JTAG debugger such as Segger
  J-Link connected to the SAMA5D3-Xplained.

  1. Start the GDB server
  2. Start GDB
  3. Use the 'target remote localhost:xxxx' command to attach to the GDG
     server
  4. Do 'mon reset' then 'mon go' to start the internal boot loader (maybe
     U-Boot).
  5. Let the boot loader run until it completes SDRAM initialization, then
     do 'mon halt'.
  6. Now you have SDRAM initialized and you use 'load nuttx' to load the
     ELF file into SDRAM.
  7. Use 'file nuttx' to load symbols
  8. Set the PC to the NuttX entry point 'mon pc 0x20008040' and start
     nuttx using 'mon go'.

Buttons and LEDs
================

  Buttons
  -------

  The following push buttons switches are available:

    1. One board reset button (BP2). When pressed and released, this push
       button causes a power-on reset of the whole board.

    2. One wakeup pushbutton that brings the processor out of Low-power mode
       (BP1)

    3. One user pushbutton (BP3)

  Only the user push button (BP3) is controllable by software:

    - PE29.  Pressing the switch connect PE29 to ground.  Therefore, PE29
      must be pulled high internally.  When the button is pressed the SAMA5
      will sense "0" is on PE29.

  LEDs
  ----
  There are two LEDs on the SAMA5D3 series-CM board that can be controlled
  by software.  A  blue LED is controlled via PIO pins.  A red LED normally
  provides an indication that power is supplied to the board but can also
  be controlled via software.

    PE23.  This blue LED is pulled high and is illuminated by pulling PE23
    low.

    PE24.  The red LED is also pulled high but is driven by a transistor so
    that it is illuminated when power is applied even if PE24 is not
    configured as an output.  If PE24 is configured as an output, then the
    LED is illuminated by a high output.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
  events as follows:

    SYMBOL                Meaning                     LED state
                                                    Blue     Red
    -------------------  -----------------------  -------- --------
    LED_STARTED          NuttX has been started     OFF      OFF
    LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
    LED_IRQSENABLED      Interrupts enabled         OFF      OFF
    LED_STACKCREATED     Idle stack created         ON       OFF
    LED_INIRQ            In an interrupt              No change
    LED_SIGNAL           In a signal handler          No change
    LED_ASSERTION        An assertion failed          No change
    LED_PANIC            The system has crashed     OFF      Blinking
    LED_IDLE             MCU is is sleep mode         Not used

  Thus if the blue LED is statically on, NuttX has successfully booted and
  is, apparently, running normally.  If the red LED is flashing at
  approximately 2Hz, then a fatal error has been detected and the system
  has halted.

Serial Console
==============

  UARTS/USARTS
  ------------

    CONN  LABEL   PIO   UART/USART  FUNCTION
    ----- ------- ----- ----------- ---------------
    J18   SCL0    PC30  UART0       UTXD0
    J18   SDA0    PC29  UART0       URXD0
    J15   1       PA31  UART1       UTXD1
    J15   0       PA30  UART1       URXD1
    J20   TXD3 14 PC26  UART1       URXD1
    J20   RXD3 15 PC27  UART1       UTXD1
    J20   TXD1 16 PD18  USART0      TXD0
    J20   RXD1 17 PD17  USART0      RXD0
    J20   TXD0 18 PB29  USART1      TXD1
    J20   RXD0 19 PB28  USART1      RXD1
    J20   SDA  20 PE19  USART3      TXD3
    J20   SCL  21 PE18  USART3      RXD3

  DBGU Interface
  --------------

  The SAMA5D3 Xplained board has a dedicated serial port for debugging,
  which is accessible through the 6-pin male header J23.

  PIN PIO  Usage
  --- ---- -----------------------------------------
   1  PE13 (available)
   2  PB31 DBGU DTXD
   3  PB30 DBGU DRXD
   4  N/C  (may be used by debug interface tool)
   5  PE14 (available)
   6  GND

  By default the DBUG port is used as the NuttX serial console in all
  configurations (unless otherwise noted).  The DBGU is available at
  logic levels at pins RXD and TXD of the DEBUG connector (J23).  GND
  is available at J23 and +3.3V is available from J14

Networking
==========

  Networking support via the can be added to NSH by selecting the following
  configuration options.  The SAMA5D36 supports two different Ethernet MAC
  peripherals:  (1) The 10/100Base-T EMAC peripheral and (2) the
  10/100/1000Base-T GMAC peripheral. Ethernet over USB using the
  CDC ECM driver is also supported, and should work on Linux, macOS, and
  Windows.

  Selecting the EMAC peripheral
  -----------------------------

  System Type -> SAMA5 Peripheral Support
    CONFIG_SAMA5_EMACA=y                 : Enable the EMAC A peripheral

  System Type -> EMAC device driver options
    CONFIG_SAMA5_EMAC_NRXBUFFERS=16      : Set aside some RS and TX buffers
    CONFIG_SAMA5_EMAC_NTXBUFFERS=4
    CONFIG_SAMA5_EMAC_PHYADDR=1          : KSZ9031 PHY is at address 1
    CONFIG_SAMA5_EMAC_AUTONEG=y          : Use autonegotiation
    CONFIG_SAMA5_EMAC_RMII=y             : Either MII or RMII interface should work
    CONFIG_SAMA5_EMAC_PHYSR=30           : Address of PHY status register on KSZ9031
    CONFIG_SAMA5_EMAC_PHYSR_ALTCONFIG=y  : Needed for KSZ9031
    CONFIG_SAMA5_EMAC_PHYSR_ALTMODE=0x7  : "    " " " "     "
    CONFIG_SAMA5_EMAC_PHYSR_10HD=0x1     : "    " " " "     "
    CONFIG_SAMA5_EMAC_PHYSR_100HD=0x2    : "    " " " "     "
    CONFIG_SAMA5_EMAC_PHYSR_10FD=0x5     : "    " " " "     "
    CONFIG_SAMA5_EMAC_PHYSR_100FD=0x6    : "    " " " "     "

  PHY selection.  Later in the configuration steps, you will need to select
  the KSZ9031 PHY for EMAC (See below)

  Selecting the GMAC peripheral
  -----------------------------

  System Type -> SAMA5 Peripheral Support
    CONFIG_SAMA5_GMAC=y                  : Enable the GMAC peripheral

  System Type -> GMAC device driver options
    CONFIG_SAMA5_GMAC_NRXBUFFERS=16      : Set aside some RS and TX buffers
    CONFIG_SAMA5_GMAC_NTXBUFFERS=4
    CONFIG_SAMA5_GMAC_PHYADDR=1          : KSZ8081 PHY is at address 1
    CONFIG_SAMA5_GMAC_AUTONEG=y          : Use autonegotiation

  If both EMAC and GMAC are selected, you will also need:

    CONFIG_SAMA5_GMAC_ISETH0=y           : GMAC is "eth0"; EMAC is "eth1"

  PHY selection.  Later in the configuration steps, you will need to select
  the  KSZ9081 PHY for GMAC (See below)

  Selecting Ethernet over USB (CDC ECM driver)
  --------------------------------------------

  This uses the USB 2.0 connector labeled USB-A. On the host computer you will
  need to configure the CDC ECM Ethernet over USB driver (see below for Linux
  configuration script).

    CONFIG_USBDEV=y
    CONFIG_USBDEV_DMA=y
    CONFIG_USBDEV_DUALSPEED=y
    CONFIG_NET_CDCECM=y
    CONFIG_NET_ETH_PKTSIZE=1514

  You can also use the defconfig file in `boards/arm/sama5/sama5d3-xplained/configs/ethernet-over-usb-2-high-speed`.

  Common configuration settings
  -----------------------------

  Networking Support
    CONFIG_NET=y                         : Enable Neworking
    CONFIG_NET_SOCKOPTS=y                : Enable socket operations
    CONFIG_NET_ETH_PKTSIZE=562           : Maximum packet size 1518 is more standard
    CONFIG_NET_TCP=y                     : Enable TCP/IP networking
    CONFIG_NET_TCPBACKLOG=y              : Support TCP/IP backlog
    CONFIG_NET_UDP=y                     : Enable UDP networking
    CONFIG_NET_ICMP=y                    : Enable ICMP networking
    CONFIG_NET_ICMP_SOCKET=y             : Needed for NSH ping command
                                         : Defaults should be okay for other options
  Device drivers -> Network Device/PHY Support
    CONFIG_NETDEVICES=y                  : Enabled PHY selection
    CONFIG_ETH0_PHY_KSZ8081=y            : Select the KSZ8081 PHY (for EMAC), OR
    CONFIG_ETH0_PHY_KSZ90x1=y            : Select the KSZ9031 PHY (for GMAC)

  Application Configuration -> Network Utilities
    CONFIG_NETDB_DNSCLIENT=y             : Enable host address resolution
    CONFIG_NETUTILS_TELNETD=y            : Enable the Telnet daemon
    CONFIG_NETUTILS_TFTPC=y              : Enable TFTP data file transfers for get and put commands
    CONFIG_NETUTILS_NETLIB=y             : Network library support is needed
    CONFIG_NETUTILS_WEBCLIENT=y          : Needed for wget support
                                         : Defaults should be okay for other options
  Application Configuration -> NSH Library
    CONFIG_NSH_TELNET=y                  : Enable NSH session via Telnet
    CONFIG_NSH_IPADDR=0x0a000002         : Select an IP address
    CONFIG_NSH_DRIPADDR=0x0a000001       : IP address of gateway/host PC
    CONFIG_NSH_NETMASK=0xffffff00        : Netmask
    CONFIG_NSH_NOMAC=y                   : Need to make up a bogus MAC address
                                         : Defaults should be okay for other options

  Using the network with NSH
  --------------------------

  So what can you do with this networking support?  First you see that
  NSH has several new network related commands:

    ifconfig, ifdown, ifup:  Commands to help manage your network
    get and put:             TFTP file transfers
    wget:                    HTML file transfers
    ping:                    Check for access to peers on the network
    Telnet console:          You can access the NSH remotely via telnet.

  You can also enable other add on features like full FTP or a Web
  Server or XML RPC and others.  There are also other features that
  you can enable like DHCP client (or server) or network name
  resolution.

  By default, the IP address of the SAMA5D3-Xplained will be 10.0.0.2 and
  it will assume that your host is the gateway and has the IP address
  10.0.0.1.

    nsh> ifconfig
    eth0    HWaddr 00:e0:de:ad:be:ef at UP
            IPaddr:10.0.0.2 DRaddr:10.0.0.1 Mask:255.255.255.0

  You can use ping to test for connectivity to the host (Careful,
  Window firewalls usually block ping-related ICMP traffic).  On the
  target side, you can:

    nsh> ping 10.0.0.1
    PING 10.0.0.1 56 bytes of data
    56 bytes from 10.0.0.1: icmp_seq=1 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=2 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=3 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=4 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=5 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=6 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=7 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=8 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=9 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=10 time=0 ms
    10 packets transmitted, 10 received, 0% packet loss, time 10100 ms

  NOTE: In this configuration is is normal to have packet loss > 0%
  the first time you ping due to the default handling of the ARP
  table.

  On the host side, you should also be able to ping the SAMA5D3-Xplained:

    $ ping 10.0.0.2

  You can also log into the NSH from the host PC like this:

    $ telnet 10.0.0.2
    Trying 10.0.0.2...
    Connected to 10.0.0.2.
    Escape character is '^]'.
    sh_telnetmain: Session [3] Started

    NuttShell (NSH) NuttX-6.31
    nsh> help
    help usage:  help [-v] [<cmd>]

      [           echo        ifconfig    mkdir       mw          sleep
      ?           exec        ifdown      mkfatfs     ping        test
      cat         exit        ifup        mkfifo      ps          umount
      cp          free        kill        mkrd        put         usleep
      cmp         get         losetup     mh          rm          wget
      dd          help        ls          mount       rmdir       xd
      df          hexdump     mb          mv          source

    Builtin Apps:
    nsh>

  NOTE:  If you enable this feature, you experience a delay on booting.
  That is because the start-up logic waits for the network connection
  to be established before starting NuttX.  In a real application, you
  would probably want to do the network bringup on a separate thread
  so that access to the NSH prompt is not delayed.

  This delay will be especially long if the board is not connected to
  a network.  On the order of a minute!  You will probably think that
  NuttX has crashed!  And then, when it finally does come up, the
  network will not be available.

  Network Initialization Thread
  -----------------------------
  There is a configuration option enabled by CONFIG_NSH_NETINIT_THREAD
  that will do the NSH network bring-up asynchronously in parallel on
  a separate thread.  This eliminates the (visible) networking delay
  altogether.  This networking initialization feature by itself has
  some limitations:

    - If no network is connected, the network bring-up will fail and
      the network initialization thread will simply exit.  There are no
      retries and no mechanism to know if the network initialization was
      successful.

    - Furthermore, there is no support for detecting loss of the network
      connection and recovery of networking when the connection is restored.

  Both of these shortcomings can be eliminated by enabling the network
  monitor:

  Network Monitor
  ---------------
  By default the network initialization thread will bring-up the network
  then exit, freeing all of the resources that it required.  This is a
  good behavior for systems with limited memory.

  If the CONFIG_NSH_NETINIT_MONITOR option is selected, however, then the
  network initialization thread will persist forever; it will monitor the
  network status.  In the event that the network goes down (for example, if
  a cable is removed), then the thread will monitor the link status and
  attempt to bring the network back up.  In this case the resources
  required for network initialization are never released.

  Pre-requisites:

    - CONFIG_NSH_NETINIT_THREAD as described above.

    - CONFIG_NETDEV_PHY_IOCTL. Enable PHY IOCTL commands in the Ethernet
      device driver. Special IOCTL commands must be provided by the Ethernet
      driver to support certain PHY operations that will be needed for link
      management. There operations are not complex and are implemented for
      the Atmel SAMA5 family.

    - CONFIG_ARCH_PHY_INTERRUPT. This is not a user selectable option.
      Rather, it is set when you select a board that supports PHY interrupts.
      In most architectures, the PHY interrupt is not associated with the
      Ethernet driver at all. Rather, the PHY interrupt is provided via some
      board-specific GPIO and the board-specific logic must provide support
      for that GPIO interrupt. To do this, the board logic must do two things:
      (1) It must provide the function arch_phy_irq() as described and
      prototyped in the nuttx/include/nuttx/arch.h, and (2) it must select
      CONFIG_ARCH_PHY_INTERRUPT in the board configuration file to advertise
      that it supports arch_phy_irq().  This logic can be found at
      nuttx/boards/arm/sama5/sama5d3-xplained/src/sam_ethernet.c.

    - One other thing: UDP support is required (CONFIG_NET_UDP).

  Given those prerequisites, the network monitor can be selected with these additional settings.

    Networking Support -> Networking Device Support
      CONFIG_NETDEV_PHY_IOCTL=y             : Enable PHY ioctl support

    Application Configuration -> NSH Library -> Networking Configuration
      CONFIG_NSH_NETINIT_THREAD             : Enable the network initialization thread
      CONFIG_NSH_NETINIT_MONITOR=y          : Enable the network monitor
      CONFIG_NSH_NETINIT_RETRYMSEC=2000     : Configure the network monitor as you like

  Ethernet Over USB Configuration Script
  --------------------------------------

  There is a configuration script for Linux that will configure the USB Ethernet interface,
  it is in `tools/netusb.sh`. You can use it as follows:

  Once you boot a NuttX system with the CDC ECM Ethernet over USB device, the Linux network interface
  will be added to your system. You should see something like the following messages in
  /var/log/kern.log:

   [302074.552879] usb 1-2: new high-speed USB device number 107 using ehci-pci
   [302074.718264] usb 1-2: New USB device found, idVendor=0525, idProduct=a4a2, bcdDevice= 1.00
   [302074.718267] usb 1-2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
   [302074.718269] usb 1-2: Product: CDC/ECM Ethernet
   [302074.718271] usb 1-2: Manufacturer: NuttX
   [302074.718272] usb 1-2: SerialNumber: 0
   [302074.760638] cdc_ether 1-2:1.0 usb0: register 'cdc_ether' at usb-0000:02:03.0-2, CDC Ethernet Device, 02:00:00:11:22:33
   [302074.796215] cdc_ether 1-2:1.0 ens160u4u2: renamed from usb0

  If you execute the command 'ifconfig -a' you should see a new interface:

  $ ifconfig -a

  ens33: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
          inet 192.168.46.156  netmask 255.255.255.0  broadcast 192.168.46.255
          inet6 fe80::20c:29ff:fe57:d0f8  prefixlen 64  scopeid 0x20<link>
          ether 00:0c:29:57:d0:f8  txqueuelen 1000  (Ethernet)
          RX packets 7628014  bytes 2002078802 (2.0 GB)
          RX errors 0  dropped 0  overruns 0  frame 0
          TX packets 6040388  bytes 5327276865 (5.3 GB)
          TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

  ens160u4u2: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
          inet6 fe80::ff:fe11:2233  prefixlen 64  scopeid 0x20<link>
          ether 02:00:00:11:22:33  txqueuelen 1000  (Ethernet)
          RX packets 36798  bytes 51705300 (51.7 MB)
          RX errors 0  dropped 0  overruns 0  frame 0
          TX packets 24196  bytes 1312512 (1.3 MB)
          TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

  ens33 is the host Ethernet or wireless LAN interface. ens160u4u2 is the USB Ethernet
  interface.

  The script will bring up the interface, configure it, and set up routes and IP Tables rules so the
  nuttx system can access the internet:

  $ sudo ./tools/netusb.sh ens33 ens160u4u2 on

  This will bring down the interface, configure it, and delete routes and IP Tables rules:

  $ sudo ./tools/netusb.sh ens33 ens160u4u2 off

  Now that the new interface has an IP address, you can ping the NuttX box at 10.0.0.2
  (or whatever IP address you configured it to have). If you configured the telnet daemon
  and started it, you should be able to telnet to the board using:

  $ telnet 10.0.0.2

  The helper script also sets up Network Address Translation (NAT) so the NuttX system
  can access the Internet. If that is not what you want, you can remove the iptables

AT25 Serial FLASH
=================

  Connections
  -----------

  The SAMA5D3-Xplained board supports an options Serial DataFlash connected
  at MN8.  The SPI connection is as follows:

         MN8       SAMA5
    ------------- -----------------------------------------------
    PIN  FUNCTION  PIO   FUNCTION
    --- --------- ----- -----------------------------------------
     5    SI       PD11  SPI0_MOSI
     2    SO       PD10  SPI0_MIS0
     6    SCK      PD12  SPI0_SPCK
     1    /CS      PD13  if jumper JP6 is closed.

  NOTE:  The MN8 is not populated on my SAMAD3 Xplained board.  So, as a
  result, these instructions would only apply if you were to have an AT25
  Serial DataFlash installed in MN8.

  Configuration
  -------------

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_SPI0=y                   : Enable SPI0
      CONFIG_SAMA5_DMAC0=y                  : Enable DMA controller 0

    System Type -> SPI device driver options
      CONFIG_SAMA5_SPI_DMA=y                : Use DMA for SPI transfers
      CONFIG_SAMA5_SPI_DMATHRESHOLD=4       : Don't DMA for small transfers

    Device Drivers -> SPI Driver Support
      CONFIG_SPI=y                          : Enable SPI support
      CONFIG_SPI_EXCHANGE=y                 : Support the exchange method

    Device Drivers -> Memory Technology Device (MTD) Support
      CONFIG_MTD=y                          : Enable MTD support
      CONFIG_MTD_AT25=y                     : Enable the AT25 driver
      CONFIG_AT25_SPIMODE=0                 : Use SPI mode 0
      CONFIG_AT25_SPIFREQUENCY=10000000     : Use SPI frequency 10MHz

  The AT25 is capable of higher SPI rates than this.  I have not experimented
  a lot, but at 20MHz, the behavior is not the same with all CM modules.  This
  lower rate gives more predictable performance.

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

    Board Selection
      CONFIG_SAMA5D3XPLAINED_AT25_AUTOMOUNT=y         : Mounts AT25 for NSH
      CONFIG_SAMA5D3XPLAINED_AT25_FTL=y               : Create block driver for FAT

  NOTE: that you must close JP6 in order to enable the AT25 FLASH chip select.

  You can then format the AT25 FLASH for a FAT file system and mount the
  file system at /mnt/at25 using these NSH commands:

    nsh> mkfatfs /dev/mtdblock0
    nsh> mount -t vfat /dev/mtdblock0 /mnt/at25

  Then you an use the FLASH as a normal FAT file system:

    nsh> echo "This is a test" >/mnt/at25/atest.txt
    nsh> ls -l /mnt/at25
    /mnt/at25:
     -rw-rw-rw-      16 atest.txt
    nsh> cat /mnt/at25/atest.txt
    This is a test

HSMCI Card Slots
================

  Physical Slots
  --------------

  The SAMA5D3-Xplained provides a two SD memory card slots:  (1) a full size SD
  card slot (J10), and (2) a microSD memory card slot (J11).

  The full size SD card slot connects via HSMCI0.  The card detect discrete
  is available on PD17 (pulled high).  The write protect discrete is tied to
  ground and not i savailable to software.  The slot supports 8-bit wide
  transfer mode, but the NuttX driver currently uses only the 4-bit wide
  transfer mode

    PD17 MCI0_CD
    PD1  MCI0_DA0
    PD2  MCI0_DA1
    PD3  MCI0_DA2
    PD4  MCI0_DA3
    PD5  MCI0_DA4
    PD6  MCI0_DA5
    PD7  MCI0_DA6
    PD8  MCI0_DA7
    PD9  MCI0_CK
    PD0  MCI0_CDA

    PE2  PWR_MCI0

  The microSD connects vi HSMCI1.  The card detect discrete is available on
  PD18 (pulled high):

    PD18  MCI1_CD
    PB20  MCI1_DA0
    PB21  MCI1_DA1
    PB22  MCI1_DA2
    PB23  MCI1_DA3
    PB24  MCI1_CK
    PB19  MCI1_CDA

  Configuration Settings
  ----------------------

  Enabling HSMCI support. The SAMA5D3-Xplained provides a two SD memory card
  slots:  (1) a full size SD card slot (J10), and (2) a microSD memory card
  slot (J11).  The full size SD card slot connects via HSMCI0; the microSD
  connects via HSMCI1.  Support for both SD slots can be enabled with the
  following settings:

    System Type->ATSAMA5 Peripheral Support
      CONFIG_SAMA5_HSMCI0=y                 : Enable HSMCI0 support
      CONFIG_SAMA5_HSMCI1=y                 : Enable HSMCI1 support
      CONFIG_SAMA5_DMAC0=y                  : DMAC0 is needed by HSMCI0
      CONFIG_SAMA5_DMAC1=y                  : DMAC1 is needed by HSMCI1

    System Type
      CONFIG_SAMA5_PIO_IRQ=y                : PIO interrupts needed
      CONFIG_SAMA5_PIOD_IRQ=y               : Card detect pins are on PIOD

    Device Drivers -> MMC/SD Driver Support
      CONFIG_MMCSD=y                        : Enable MMC/SD support
      CONFIG_MMSCD_NSLOTS=1                 : One slot per driver instance
      CONFIG_MMCSD_MULTIBLOCK_LIMIT=1       : (REVISIT)
      CONFIG_MMCSD_HAVE_CARDDETECT=y        : Supports card-detect PIOs
      CONFIG_MMCSD_MMCSUPPORT=n             : Interferes with some SD cards
      CONFIG_MMCSD_SPI=n                    : No SPI-based MMC/SD support
      CONFIG_MMCSD_SDIO=y                   : SDIO-based MMC/SD support
      CONFIG_SDIO_DMA=y                     : Use SDIO DMA
      CONFIG_SDIO_BLOCKSETUP=y              : Needs to know block sizes

    Library Routines
      CONFIG_SCHED_WORKQUEUE=y              : Driver needs work queue support

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

    Using the SD card
    -----------------

    1) After booting, the HSCMI devices will appear as /dev/mmcsd0
       and /dev/mmcsd1.

    2) If you try mounting an SD card with nothing in the slot, the
       mount will fail:

         nsh> mount -t vfat /dev/mmcsd1 /mnt/sd1
         nsh: mount: mount failed: 19

       NSH can be configured to provide errors as strings instead of
       numbers.  But in this case, only the error number is reported.  The
       error numbers can be found in nuttx/include/errno.h:

         #define ENODEV              19
         #define ENODEV_STR          "No such device"

       So the mount command is saying that there is no device or, more
       correctly, that there is no card in the SD card slot.

    3) Inserted the SD card.  Then the mount should succeed.

        nsh> mount -t vfat /dev/mmcsd1 /mnt/sd1
        nsh> ls /mnt/sd1
        /mnt/sd1:
         atest.txt
        nsh> cat /mnt/sd1/atest.txt
        This is a test

       NOTE:  See the next section entitled "Auto-Mounter" for another way
       to mount your SD card.

    4) Before removing the card, you must umount the file system.  This is
       equivalent to "ejecting" or "safely removing" the card on Windows:  It
       flushes any cached data to the card and makes the SD card unavailable
       to the applications.

         nsh> umount -t /mnt/sd1

       It is now safe to remove the card.  NuttX provides into callbacks
       that can be used by an application to automatically unmount the
       volume when it is removed.  But those callbacks are not used in
       these configurations.

Auto-Mounter
============

  NuttX implements an auto-mounter than can make working with SD cards
  easier.  With the auto-mounter, the file system will be automatically
  mounted when the SD card is inserted into the HSMCI slot and automatically
  unmounted when the SD card is removed.

  The auto-mounter is enable with:

      CONFIG_FS_AUTOMOUNTER=y

  However, to use the automounter you will to provide some additional
  board-level support.
  See boards/arm/sama5/sama5d4-xplaned for and example of how
  you might do this.

  WARNING:  SD cards should never be removed without first unmounting
  them.  This is to avoid data and possible corruption of the file
  system.  Certainly this is the case if you are writing to the SD card
  at the time of the removal.  If you use the SD card for read-only access,
  however, then I cannot think of any reason why removing the card without
  mounting would be harmful.

USB Ports
=========

  The SAMA5D3-Xplained features three USB communication ports:

    * Port A Host High Speed (EHCI) and Full Speed (OHCI) multiplexed with
      USB Device High Speed Micro AB connector, J6

    * Port B Host High Speed (EHCI) and Full Speed (OHCI) standard type A
      connector, J7 upper port

    * Port C Host Full Speed (OHCI) only standard type A connector, J7
      lower port

  The two USB host ports (only) are equipped with 500-mA high-side power
  switch for self-powered and bus-powered applications.

  The USB device port A (J6) features a VBUS insert detection function.

  Port A
  ------

    PIO  Signal Name Function
    ---- ----------- -------------------------------------------------------
    PE9  VBUS_SENSE VBus detection

    Note: No VBus power switch enable on port A.  I think that this limits
    this port to a device port or as a host port for self-powered devices
    only.

  Port B
  ------

    PIO  Signal Name Function
    ---- ----------- -------------------------------------------------------
    PE4  EN5V_USBB   VBus power enable (via MN3 power switch).  To the A1
                     pin of J7 Dual USB A connector

  Port C
  ------

    PIO  Signal Name Function
    ---- ----------- -------------------------------------------------------
    PE3  EN5V_USBC   VBus power enable (via MN3 power switch).  To the B1
                     pin of J7 Dual USB A connector

  Both Ports B and C
  ------------------

    PIO  Signal Name Function
    ---- ----------- -------------------------------------------------------
    PE5  OVCUR_USB   Combined over-current indication from port A and B

USB High-Speed Device
=====================

  Basic USB High-Speed Device Configuration
  -----------------------------------------

  Support the USB high-speed device (UDPHS) driver can be enabled with these
  NuttX configuration settings.

    Device Drivers -> USB Device Driver Support
      CONFIG_USBDEV=y                       : Enable USB device support
      CONFIG_USBDEV_DUALSPEED=y             : Device support High and Full Speed
      CONFIG_USBDEV_DMA=y                   : Device uses DMA

    System Type -> ATSAMA5 Peripheral Support
      CONFIG_SAMA5_UDPHS=y                  : Enable UDPHS High Speed USB device

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

  Mass Storage Class
  ------------------

  The Mass Storage Class (MSC) class driver is selected for use with
  UDPHS:

    Device Drivers -> USB Device Driver Support
      CONFIG_USBMSC=y                       : Enable the USB MSC class driver
      CONFIG_USBMSC_EPBULKOUT=1             : Use EP1 for the BULK OUT endpoint
      CONFIG_USBMSC_EPBULKIN=2              : Use EP2 for the BULK IN endpoint

  The following setting enables an add-on that can can be used to control
  the USB MSC device.  It will add two new NSH commands:

    a. msconn will connect the USB serial device and export the AT25
       to the host, and
    b. msdis which will disconnect the USB serial device.

    Application Configuration -> System Add-Ons:
      CONFIG_SYSTEM_USBMSC=y                : Enable the USBMSC add-on
      CONFIG_SYSTEM_USBMSC_NLUNS=1          : One LUN
      CONFIG_SYSTEM_USBMSC_DEVMINOR1=0      : Minor device zero
      CONFIG_SYSTEM_USBMSC_DEVPATH1="/dev/mtdblock0"
                                            : Use a single, LUN:  The AT25
                                            : block driver.

    NOTES:

    a. To prevent file system corruption, make sure that the AT25 is un-
       mounted *before* exporting the mass storage device to the host:

         nsh> umount /mnt/at25
         nsh> mscon

       The AT25 can be re-mounted after the mass storage class is disconnected:

        nsh> msdis
        nsh> mount -t vfat /dev/mtdblock0 /mnt/at25

    b. If you change the value CONFIG_SYSTEM_USBMSC_DEVPATH1, then you
       can export other file systems:

        "/dev/mmcsd1" will export the HSMCI1 microSD
        "/dev/mmcsd0" will export the HSMCI0 full-size SD slot
        "/dev/ram0" could even be used to export a RAM disk.  But you would
          first have to use mkrd to create the RAM disk and mkfatfs to put
          a FAT file system on it.

  CDC/ACM Serial Device Class
  ---------------------------

  This will select the CDC/ACM serial device.  Defaults for the other
  options should be okay.

    Device Drivers -> USB Device Driver Support
      CONFIG_CDCACM=y                       : Enable the CDC/ACM device
      CONFIG_CDCACM_BULKIN_REQLEN=768       : Default too small for high-speed

  The following setting enables an example that can can be used to control
  the CDC/ACM device.  It will add two new NSH commands:

    a. sercon will connect the USB serial device (creating /dev/ttyACM0), and
    b. serdis which will disconnect the USB serial device (destroying
        /dev/ttyACM0).

    Application Configuration -> Examples:
      CONFIG_SYSTEM_CDCACM=y              : Enable an CDC/ACM example

  CDC/ECM Ethernet Over USB
  -------------------------

  This allows networking to the host system via Ethernet over USB. See the
  Networking section for configuration. On USB 2.0 High Speed, the CDC ECM
  driver uses DMA and can transfer 4.4 MBytes/sec (34 Mbits/sec).

  Debugging USB Device
  --------------------

  There is normal console debug output available that can be enabled with
  CONFIG_DEBUG_FEATURES + CONFIG_DEBUG_USB.  However, USB device operation is very
  time critical and enabling this debug output WILL interfere with the
  operation of the UDPHS.  USB device tracing is a less invasive way to get
  debug information:  If tracing is enabled, the USB device will save
  encoded trace output in in-memory buffer; if the USB monitor is also
  enabled, that trace buffer will be periodically emptied and dumped to the
  system logging device (the serial console in this configuration):

    Device Drivers -> "USB Device Driver Support:
      CONFIG_USBDEV_TRACE=y                   : Enable USB trace feature
      CONFIG_USBDEV_TRACE_NRECORDS=256        : Buffer 256 records in memory
      CONFIG_USBDEV_TRACE_STRINGS=y           : (optional)

    Application Configuration -> NSH LIbrary:
      CONFIG_NSH_USBDEV_TRACE=n               : No builtin tracing from NSH
      CONFIG_NSH_ARCHINIT=y                   : Automatically start the USB monitor

    Application Configuration -> System NSH Add-Ons:
      CONFIG_USBMONITOR=y              : Enable the USB monitor daemon
      CONFIG_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
      CONFIG_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
      CONFIG_USBMONITOR_INTERVAL=1     : Dump trace data every second
      CONFIG_USBMONITOR_TRACEINIT=y    : Enable TRACE output
      CONFIG_USBMONITOR_TRACECLASS=y
      CONFIG_USBMONITOR_TRACETRANSFERS=y
      CONFIG_USBMONITOR_TRACECONTROLLER=y
      CONFIG_USBMONITOR_TRACEINTERRUPTS=y

  NOTE: If USB debug output is also enabled, both outputs will appear on the
  serial console.  However, the debug output will be asynchronous with the
  trace output and, hence, difficult to interpret.

USB High-Speed Host
===================

  OHCI Only
  ---------

  Support the USB low/full-speed OHCI host driver can be enabled by changing
  the NuttX configuration file as follows:

    System Type -> ATSAMA5 Peripheral Support
      CONFIG_SAMA5_UHPHS=y                 : USB Host High Speed

    System Type -> USB High Speed Host driver options
      CONFIG_SAMA5_OHCI=y                  : Low/full-speed OHCI support
                                           : Defaults for values probably OK
    Device Drivers
      CONFIG_USBHOST=y                     : Enable USB host support

    Device Drivers -> USB Host Driver Support
      CONFIG_USBHOST_ISOC_DISABLE=y        : Isochronous endpoints not used
      CONFIG_USBHOST_MSC=y                 : Enable the mass storage class driver
      CONFIG_USBHOST_HIDKBD=y              : Enable the HID keyboard class driver

    RTOS Features -> Work Queue Support
      CONFIG_SCHED_WORKQUEUE=y             : High priority worker thread support is required
      CONFIG_SCHED_HPWORK=y                :

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                : NSH board-initialization
file1: CONFIG_USBHOST_ISOC_DISABLE=y

  NOTE:  When OHCI is selected, the SAMA5 will operate at 384MHz instead of
  396MHz.  This is so that the PLL generates a frequency which is a multiple
  of the 48MHz needed for OHCI.  The delay loop calibration values that are
  used will be off slightly because of this.

  EHCI
  ----

  Support the USB high-speed EHCI host driver can be enabled by changing the
  NuttX configuration file as follows.  If EHCI is enabled by itself, then
  only high-speed devices can be supported.  If OHCI is also enabled, then
  all low-, full-, and high speed devices will work.

    System Type -> ATSAMA5 Peripheral Support
      CONFIG_SAMA5_UHPHS=y                 : USB Host High Speed

    System Type -> USB High Speed Host driver options
      CONFIG_SAMA5_EHCI=y                  : High-speed EHCI support
      CONFIG_SAMA5_OHCI=y                  : Low/full-speed OHCI support
                                           : Defaults for values probably OK for both
    Device Drivers
      CONFIG_USBHOST=y                     : Enable USB host support
      CONFIG_USBHOST_INT_DISABLE=y         : Interrupt endpoints not needed
      CONFIG_USBHOST_ISOC_DISABLE=y        : Isochronous endpoints not needed

    Device Drivers -> USB Host Driver Support
      CONFIG_USBHOST_ISOC_DISABLE=y        : Isochronous endpoints not used
      CONFIG_USBHOST_MSC=y                 : Enable the mass storage class driver
      CONFIG_USBHOST_HIDKBD=y              : Enable the HID keyboard class driver

    RTOS Features -> Work Queue Support
      CONFIG_SCHED_WORKQUEUE=y             : High priority worker thread support is required
      CONFIG_SCHED_HPWORK=y                :

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                : NSH board-initialization

  USB Hub Support
  ----------------

  USB hub support can be included by adding the following changes to the configuration (in addition to those listed above):

    Drivers -> USB Host Driver Support
      CONFIG_USBHOST_HUB=y                 : Enable the hub class
      CONFIG_USBHOST_ASYNCH=y              : Asynchronous I/O supported needed for hubs

    System Type -> USB High Speed Host driver options
      CONFIG_SAMA5_OHCI_NEDS=12            : You will probably want more pipes
      CONFIG_SAMA5_OHCI_NTDS=18
      CONFIG_SAMA5_OHCI_TDBUFFERS=12
      CONFIG_SAMA5_OHCI_TDBUFSIZE=128

    Board Selection ->
      CONFIG_SAMA5D3XPLAINED_USBHOST_STACKSIZE=2048 (bigger than it needs to be)

    RTOS Features -> Work Queue Support
      CONFIG_SCHED_LPWORK=y                 : Low priority queue support is needed
      CONFIG_SCHED_LPNTHREADS=1
      CONFIG_SCHED_LPWORKSTACKSIZE=1024

    NOTES:

    1. It is necessary to perform work on the low-priority work queue
       (vs. the high priority work queue) because deferred hub-related
       work requires some delays and waiting that is not appropriate on
       the high priority work queue.

    2. Stack usage make increase when USB hub support is enabled because
       the nesting depth of certain USB host class logic can increase.

    STATUS:
    2015-05-01:
      This USB host function does not work on the SAMA5D3-Xplained board.
      Those same drivers work on the other SAMA5Dx boards and so I believe
      that there is some issue with either clocking to USB or to powering
      of the USB host ports.

  Mass Storage Device Usage
  -------------------------

  Example Usage:

    NuttShell (NSH) NuttX-6.29
    nsh> ls /dev
     /dev:
     console
     mtdblock0
     null
     ttyS0

  Here a USB FLASH stick is inserted.  Nothing visible happens in the
  shell.  But a new device will appear:

    nsh> ls /dev
    /dev:
     console
     mtdblock0
     null
     sda
     ttyS0
    nsh> mount -t vfat /dev/sda /mnt/sda
    nsh> ls -l /mnt/sda
    /mnt/sda:
     -rw-rw-rw-    8788 viminfo
     drw-rw-rw-       0 .Trash-1000/
     -rw-rw-rw-    3378 zmodem.patch
     -rw-rw-rw-    1503 sz-1.log
     -rw-rw-rw-     613 .bashrc

  HID Keyboard Usage
  ------------------

  If a (supported) USB keyboard is connected, a /dev/kbda device will appear:

    nsh> ls /dev
    /dev:
     console
     kbda
     mtdblock0
     null
     ttyS0

  /dev/kbda is a read-only serial device.  Reading from /dev/kbda will get
  keyboard input as ASCII data (other encodings are possible):

    nsh> cat /dev/kbda

  Debugging USB Host
  ------------------

  There is normal console debug output available that can be enabled with
  CONFIG_DEBUG_FEATURES + CONFIG_DEBUG_USB.  However, USB host operation is very time
  critical and enabling this debug output might interfere with the operation
  of the UDPHS.  USB host tracing is a less invasive way to get debug
  information:  If tracing is enabled, the USB host will save encoded trace
  output in in-memory buffer; if the USB monitor is also enabled, that trace
  buffer will be periodically emptied and dumped to the system logging device
  (the serial console in this configuration):

    Device Drivers -> "USB Host Driver Support:
      CONFIG_USBHOST_TRACE=y                   : Enable USB host trace feature
      CONFIG_USBHOST_TRACE_NRECORDS=256        : Buffer 256 records in memory
      CONFIG_USBHOST_TRACE_VERBOSE=y           : Buffer everything

    Application Configuration -> NSH LIbrary:
      CONFIG_NSH_USBDEV_TRACE=n               : No builtin tracing from NSH
      CONFIG_NSH_ARCHINIT=y                   : Automatically start the USB monitor

    Application Configuration -> System NSH Add-Ons:
      CONFIG_USBMONITOR=y              : Enable the USB monitor daemon
      CONFIG_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
      CONFIG_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
      CONFIG_USBMONITOR_INTERVAL=1     : Dump trace data every second

  NOTE: If USB debug output is also enabled, both outpus will appear on the
  serial console.  However, the debug output will be asynchronous with the
  trace output and, hence, difficult to interpret.

SDRAM Support
=============

  SRAM Heap Configuration
  -----------------------

  In these configurations, .data and .bss are retained in ISRAM.  SDRAM can
  be initialized and included in the heap.  Relevant configuration settings:

    System Type->ATSAMA5 Peripheral Support
      CONFIG_SAMA5_MPDDRC=y                 : Enable the DDR controller

    System Type->External Memory Configuration
      CONFIG_SAMA5_DDRCS=y                  : Tell the system that DRAM is at the DDR CS
      CONFIG_SAMA5_DDRCS_SIZE=268435456     : 2Gb DRAM -> 256MB
      CONFIG_SAMA5_DDRCS_LPDDR2=y           : Its DDR2
      CONFIG_SAMA5D3XPLAINED_MT47H128M16RT=y          : This is the type of DDR2

    System Type->Heap Configuration
      CONFIG_SAMA5_DDRCS_HEAP=y             : Add the SDRAM to the heap
      CONFIG_SAMA5_DDRCS_HEAP_OFFSET=0
      CONFIG_SAMA5_DDRCS_HEAP_SIZE=268435456

    Memory Management
      CONFIG_MM_REGIONS=2                   : Two heap memory regions:  ISRAM and SDRAM

  RAM Test
  --------

  Another thing you could do is to enable the RAM test built-in application.
  You can enable the NuttX RAM test that may be used to verify the external
  SDRAM.  To do this, keep the SDRAM out of the heap so that it can be tested
  without crashing programs using the memory:

    System Type->Heap Configuration
      CONFIG_SAMA5_DDRCS_HEAP=n             : Don't add the SDRAM to the heap

    Memory Management
      CONFIG_MM_REGIONS=1                   : One memory regions:  ISRAM

  Then enable the RAM test built-in application:

    Application Configuration->System NSH Add-Ons->Ram Test
      CONFIG_SYSTEM_RAMTEST=y

  In this configuration, the SDRAM is not added to heap and so is not
  accessible to the applications.  So the RAM test can be freely executed
  against the SRAM memory beginning at address 0x2000:0000 (DDR CS):

    nsh> ramtest -h
    Usage: <noname> [-w|h|b] <hex-address> <decimal-size>

    Where:
      <hex-address> starting address of the test.
      <decimal-size> number of memory locations (in bytes).
      -w Sets the width of a memory location to 32-bits.
      -h Sets the width of a memory location to 16-bits (default).
      -b Sets the width of a memory location to 8-bits.

    To test the entire external 256MB SRAM:

    nsh> ramtest -w 20000000 268435456
    RAMTest: Marching ones: 20000000 268435456
    RAMTest: Marching zeroes: 20000000 268435456
    RAMTest: Pattern test: 20000000 268435456 55555555 aaaaaaaa
    RAMTest: Pattern test: 20000000 268435456 66666666 99999999
    RAMTest: Pattern test: 20000000 268435456 33333333 cccccccc
    RAMTest: Address-in-address test: 20000000 268435456

  SDRAM Data Configuration
  ------------------------

  In these configurations, .data and .bss are retained in ISRAM by default.
  .data and .bss can also be retained in SDRAM using these slightly
  different configuration settings.  In this configuration, ISRAM is
  used only for the Cortex-A5 page table for the IDLE thread stack.

    System Type->ATSAMA5 Peripheral Support
      CONFIG_SAMA5_MPDDRC=y                 : Enable the DDR controller

    System Type->External Memory Configuration
      CONFIG_SAMA5_DDRCS=y                  : Tell the system that DRAM is at the DDR CS
      CONFIG_SAMA5_DDRCS_SIZE=268435456     : 2Gb DRAM -> 256GB
      CONFIG_SAMA5_DDRCS_LPDDR2=y           : Its DDR2
      CONFIG_SAMA5D3XPLAINED_MT47H128M16RT=y          : This is the type of DDR2

    System Type->Heap Configuration
      CONFIG_SAMA5_ISRAM_HEAP=n              : These do not apply in this case
      CONFIG_SAMA5_DDRCS_HEAP=n

    System Type->Boot Memory Configuration
      CONFIG_RAM_START=0x20000000           : Physical address of SDRAM
      CONFIG_RAM_VSTART=0x20000000          : Virtual address of SDRAM
      CONFIG_RAM_SIZE=268435456             : Size of SDRAM
      CONFIG_BOOT_SDRAM_DATA=y              : Data is in SDRAM

      Care must be used applied these RAM locations; graphics
      configurations may use SDRAM in an incompatible way to set aside
      LCD framebuffers.

    Memory Management
      CONFIG_MM_REGIONS=1                   : One heap memory region:  ISDRAM

NAND Support
============

  NAND support is only partial in that there is no file system that works
  with it properly.  Lower-level NAND support has been developed and
  verified, but there is no way to use it in the current NuttX architecture
  other than through the raw MTD interface.

  NAND should still be considered a work in progress.  You will not want to
  use NAND unless you are interested in investing a little effort,
  particularly in infrastructure. See the "STATUS SUMMARY" section below.

  NAND Support
  ------------

  NAND Support can be added to the NSH configuration by modifying the
  NuttX configuration file as follows:

    Build Setup
      CONFIG_EXPERIMENTAL=y             : NXFFS implementation is incomplete and
                                        : not yet fully functional.

    System Type -> SAMA5 Peripheral support
      CONFIG_SAMA5_HSMC=y               : Make sure that the SMC is enabled

    Drivers -> Memory Technology Device (MTD) Support
      CONFIG_MTD=y                      : Enable MTD support
      CONFIG_MTD_NAND=y                 : Enable NAND support
      CONFIG_MTD_NAND_BLOCKCHECK=n      : Interferes with NXFFS bad block checking
      CONFIG_MTD_NAND_SWECC=y           : Use S/W ECC calculation

      Defaults for all other NAND settings should be okay

    System Type -> External Memory Configuration
      CONFIG_SAMA5_EBICS3=y             : Enable External CS3 memory
      CONFIG_SAMA5_EBICS3_NAND=y        : Select NAND memory type
      CONFIG_SAMA5_EBICS3_SIZE=8388608  : Use this size
      CONFIG_SAMA5_EBICS3_SWECC=y       : Use S/W ECC calculation

      Defaults for ROM page table addresses should be okay

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y             : Use architecture-specific initialization

    NOTES:

    1. WARNING:  This will wipe out everything that you may have on the NAND
       FLASH!  I have found that using the JTAG with no valid image on NAND
       or Serial FLASH is a problem:  In that case, the code always ends up
       in the SAM-BA bootloader.

       My understanding is that you can enable JTAG in this case by simply
       entering any data on the DBG serial port.  I have not tried this.
       Instead, I just changed to boot from Serial Flash:

    2. Unfortunately, there are no appropriate NAND file system in NuttX as
       of this writing.  The following sections discussion issues/problems
       with using NXFFS and FAT.

    PMECC
    -----

    Hardware ECC calculation using the SAMA5D3's PMECC can be enable as
    follows:

    Drivers -> Memory Technology Device (MTD) Support
      CONFIG_MTD_NAND_SWECC=y           : Don't use S/W ECC calculation
      CONFIG_MTD_NAND_HWECC=y           : Use H/W ECC instead

    System Type -> External Memory Configuration
      CONFIG_SAMA5_EBICS3_SWECC=n       : Don't use S/W ECC calculation
      CONFIG_SAMA5_HAVE_PMECC=n         : Use H/W ECC instead

    Other PMECC-related default settings should be okay.

    STATUS:  As of the writing, NAND transfers using PMECC appear to
    work correctly.  However, the PMECC based systems do not work as
    as well with FAT or NXFFS.  My belief that that the FAT/NXFFS layers
    are inappropriate for NAND and, as a result, happen not to work with
    the PMECC ECC calculation.  See also the "STATUS SUMMARY" section below.

    DMA Support
    -----------

    DMA support can be enabled as follows:

    System Type -> SAMA5 Peripheral support
      CONFIG_SAMA5_DMAC0=y              : Use DMAC0 for memory-to-memory DMA

    System Type -> External Memory Configuration
      CONFIG_SAMA5_NAND_DMA=y           : Use DMAC0 for NAND data transfers

    STATUS:  DMA appears to be functional, but probably has not been
    exercised enough to claim that with any certainty.  See also the "STATUS
    SUMMARY" section below.

    NXFFS
    -----

    The NuttX FLASH File System (NXFFS) works well with NOR-like FLASH
    but does not work well with NAND (See comments below under STATUS)

    File Systems:
      CONFIG_FS_NXFFS=y                 : Enable the NXFFS file system

      Defaults for all other NXFFS settings should be okay.

      NOTE:  NXFFS will require some significant buffering because of
      the large size of the NAND flash blocks.  You will also need
      to enable SDRAM as described above.

    Board Selection
      CONFIG_SAMA5D3XPLAINED_NAND_BLOCKMOUNT=y : Enable FS support on NAND
      CONFIG_SAMA5D3XPLAINED_NAND_NXFFS=y      : Use the NXFFS file system

      Other file systems are not recommended because only NXFFS can handle
      bad blocks and only NXFFS performs wear-levelling.

    FAT
    ---

    Another option is FAT.  FAT, however, is not appropriate for use with
    NAND: FAT will not handle bad blocks, does not perform any wear
    levelling, and may not conform to writing ordering requirements of NAND.
    Also, there appear to be issues with FAT when PMECC is enabled (see
    "STATUS SUMMARY" below).

    File Systems:
      CONFIG_FS_FAT=y                   : Enable the FAT FS
      CONFIG_FAT_LCNAMES=y              : With lower case name support
      CONFIG_FAT_LFN=y                  : And (patented) FAT long file name support
      CONFIG_FS_NXFFS=n                 : Don't need NXFFS

      Defaults for all other NXFFS settings should be okay.

    Board Selection
      CONFIG_SAMA5D3XPLAINED_NAND_BLOCKMOUNT=y : Enable FS support on NAND
      CONFIG_SAMA5D3XPLAINED_NAND_FTL=y        : Use an flash translation layer

      NOTE:  FTL will require some significant buffering because of
      the large size of the NAND flash blocks.  You will also need
      to enable SDRAM as described above.

    SMART FS
    --------

    Another option is Smart FS.  Smart FS is another small file system
    designed to work with FLASH.  Properties:  It does support some wear-
    leveling like NXFFS, but like FAT, cannot handle bad blocks and like
    NXFFS, it will try to re-write erased bits.

    Using NAND with NXFFS
    ---------------------

    With the options CONFIG_SAMA5D3XPLAINED_NAND_BLOCKMOUNT=y and
    CONFIG_SAMA5D3XPLAINED_NAND_NXFFS=y, the NAND FLASH will be mounted in the NSH
    start-up logic before the NSH prompt appears.  There is no feedback as
    to whether or not the mount was successful.  You can, however, see the
    mounted file systems using the nsh 'mount' command:

      nsh> mount
      /mnt/nand type nxffs

    Then NAND can be used like any other file system:

      nsh> echo "This is a test" >/mnt/nand/atest.txt
      nsh> ls -l /mnt/nand
      /mnt/nand:
       ---x--x--x      16 atest.txt
      nsh> cat /mnt/nand/atest.txt
      This is a test

    The NAND volume can be un-mounted with this comment:

      nsh> umount /mnt/nand
      nsh> mount

    And re-mounted with this command:

      nsh> mount -t nxffs /mnt/mystuff
      nsh> mount
        /mnt/mystuff type nxffs

    NOTES:
      1. NXFFS can be very slow.  The first time that you start the system,
         be prepared for a wait; NXFFS will need to format the NAND volume.
         I have lots of debug on so I don't yet know what the optimized wait
         will be.  But with debug ON, software ECC, and no DMA the wait is
         in many tens of minutes (and substantially  longer if many debug
         options are enabled.

         [I don't yet have data for the more optimal cases. It will be
          significantly less, but still not fast.]

      2. On subsequent boots, after the NXFFS file system has been created
         the delay will be less.  When the new file system is empty, it will
         be very fast.  But the NAND-related boot time can become substantial
         when there has been a lot of usage of the NAND.  This is because
         NXFFS needs to scan the NAND device and build the in-memory dataset
         needed to access NAND and there is more that must be scanned after
         the device has been used.  You may want to create a separate thread at
         boot time to bring up NXFFS so that you don't delay the boot-to-prompt
         time excessively in these longer delay cases.

      3. There is another NXFFS related performance issue:  When the FLASH
         is fully used, NXFFS will restructure the entire FLASH, the delay
         to restructure the entire FLASH will probably be even larger.  This
         solution in this case is to implement an NXFSS clean-up daemon that
         does the job a little-at-a-time so that there is no massive clean-up
         when the FLASH becomes full.

      4. Bad NXFFS behavior with NAND:  If you restart NuttX, the files that
         you wrote to NAND will be gone.  Why?  Because the multiple writes
         have corrupted the NAND ECC bits.  See STATUS below.  NXFFS would
         require a major overhaul to be usable with NAND.

    Using NAND with FAT
    -------------------

    If configured for FAT, the system will create block driver at
    /dev/mtdblock0:

      NuttShell (NSH)
      nsh> ls /dev
      /dev:
       console
       mtdblock0
       null
       ttyS0

    You will not that the system comes up immediately because there is not
    need to scan the volume in this case..

    The NSH 'mkfatfs' command can be used to format a FAT file system on
    NAND.

      nsh> mkfatfs /dev/mtdblock0

    This step, on the other hand, requires quite a bit of time.

    And the FAT file system can be mounted like:

      nsh> mount -t vfat /dev/mtdblock0 /mnt/nand
      nsh> ls /mnt/nand
      /mnt/nand:

      nsh> echo "This is a test" > /mnt/nand/atest.txt

        NOTE:  This will take a long time because it will require reading,
        modifying, and re-writing the 128KB erase page!

      nsh> ls -l /mnt/nand
      /mnt/nand:
       -rw-rw-rw-      16 atest.txt

      nsh> cat /mnt/fat/atest.txt
      This is a test

    NOTES:

    1. Unlike NXFFS, FAT can work with NAND (at least with PMECC disabled).
       But there are some significant issues.

    2. First, each NAND write access will cause a 256KB data transfer:  It
       will read the entire 128KB erase block, modify it and write it back
       to memory.  There is some caching logic so that this cached erase
       block can be re-used if possible and writes will be deferred as long
       as possible.

    3. If you hit a bad block, then FAT is finished.  There is no mechanism
       in place in FAT not to mark and skip over bad blocks.

    What is Needed
    --------------

    What is needed to work with FAT properly would be another MTD layer
    between the FTL layer and the NAND FLASH layer.  That layer would
    perform bad block detection and sparing so that FAT works transparently
    on top of the NAND.

    Another, less general, option would be support bad blocks within FAT.

  STATUS SUMMARY
  --------------

  1. PMECC appears to be working in that I can write a NAND block with its
     ECC and read the block back and verify that that is are no bit
     failures.  However, when attempting to work with FAT, it does not
     work correctly:  The MBR is written and read back correctly, but gets
     corrupted later for unknown reasons.

  2. DMA works (at least with software ECC), but I have seen occasional
     failures.  I recommend enabling DMA with caution.

     In NuttX, DMA will also cost two context switches (and, hence, four
     register state transfers).  With smaller NAND page sizes (say 2KiB and
     below), I would expect little or no performance improvement with DMA
     for this reason.

  3. NXFFS does not work with NAND. NAND differs from other other FLASH
     types several ways.  For one thing, NAND requires error correction
     (ECC) bytes that must be set in order to work around bit failures.
     This affects NXFFS in two ways:

     a. First, write failures are not fatal. Rather, they should be tried by
        bad blocks and simply ignored.  This is because unrecoverable bit
        failures will cause read failures when reading from NAND.  Setting
        the CONFIG_EXPERIMENTAL+CONFIG_NXFFS_NANDs option will enable this
        behavior.

     b. Secondly, NXFFS will write a block many times.  It tries to keep
        bits in the erased state and assumes that it can overwrite those bits
        to change them from the erased to the non-erased state.  This works
        will with NOR-like FLASH.  NAND behaves this way too.  But the
        problem with NAND is that the ECC bits cannot be re-written in this
        way.  So once a block has been written, it cannot be modified.  This
        behavior has NOT been fixed in NXFFS.  Currently, NXFFS will attempt
        to re-write the ECC bits causing the ECC to become corrupted because
        the ECC bits cannot be overwritten without erasing the entire block.

     This may prohibit NXFFS from ever being used with NAND.

  4. As mentioned above, FAT does work but (1) has some performance issues on
     writes and (2) cannot handle bad blocks.

  5. There was a major reorganization of the SAMA5 code after NuttX-7.11 to
     add support for the SAMA5D2.  Only the SAMA5D4-EK nsh configuration was
     re-verified on 2015-09-29.  But as of this writing, none of the SAMA5D3-
     Xplained configurations a been re-verified.  Some regression testing is
     needed.

I2C Tool
========

  I2C Tool. NuttX supports an I2C tool at apps/system/i2c that can be used
  to peek and poke I2C devices.  That tool can be enabled by setting the
  following:

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_TWI0=y                   : Enable TWI0
      CONFIG_SAMA5_TWI1=y                   : Enable TWI1
      CONFIG_SAMA5_TWI2=y                   : Enable TWI2

    System Type -> TWI device driver options
      SAMA5_TWI0_FREQUENCY=100000           : Select a TWI0 frequency
      SAMA5_TWI1_FREQUENCY=100000           : Select a TWI1 frequency
      SAMA5_TWI2_FREQUENCY=100000           : Select a TWI2 frequency

    Device Drivers -> I2C Driver Support
      CONFIG_I2C=y                          : Enable I2C support

    Application Configuration -> NSH Library
      CONFIG_SYSTEM_I2CTOOL=y               : Enable the I2C tool
      CONFIG_I2CTOOL_MINBUS=0               : TWI0 has the minimum bus number 0
      CONFIG_I2CTOOL_MAXBUS=2               : TWI2 has the maximum bus number 2
      CONFIG_I2CTOOL_DEFFREQ=100000         : Pick a consistent frequency

    The I2C tool has extensive help that can be accessed as follows:

    nsh> i2c help
    Usage: i2c <cmd> [arguments]
    Where <cmd> is one of:

      Show help     : ?
      List buses    : bus
      List devices  : dev [OPTIONS] <first> <last>
      Read register : get [OPTIONS] [<repetitions>]
      Show help     : help
      Write register: set [OPTIONS] <value> [<repetitions>]
      Verify access : verf [OPTIONS] [<value>] [<repetitions>]

    Where common "sticky" OPTIONS include:
      [-a addr] is the I2C device address (hex).  Default: 03 Current: 03
      [-b bus] is the I2C bus number (decimal).  Default: 0 Current: 0
      [-r regaddr] is the I2C device register address (hex).  Default: 00 Current: 00
      [-w width] is the data width (8 or 16 decimal).  Default: 8 Current: 8
      [-s|n], send/don't send start between command and data.  Default: -n Current: -n
      [-i|j], Auto increment|don't increment regaddr on repetitions.  Default: NO Current: NO
      [-f freq] I2C frequency.  Default: 100000 Current: 100000

    NOTES:
    o Arguments are "sticky".  For example, once the I2C address is
      specified, that address will be re-used until it is changed.

    WARNING:
    o The I2C dev command may have bad side effects on your I2C devices.
      Use only at your own risk.

    As an example, the I2C dev command can be used to list all devices
    responding on TWI0 (the default) like this:

      nsh> i2c dev 0x03 0x77
          0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
      00:          -- -- -- -- -- -- -- -- -- -- -- -- --
      10: -- -- -- -- -- -- -- -- -- -- 1a -- -- -- -- --
      20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      30: -- -- -- -- -- -- -- -- -- 39 -- -- -- 3d -- --
      40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      60: 60 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      70: -- -- -- -- -- -- -- --
      nsh>

     Address 0x1a is the WM8904.  Address 0x39 is the SIL9022A. I am not sure
     what is at address 0x3d and 0x60

CAN Usage
=========
  I planned to verify CAN using the IXXAT USB-to-CAN Compact.  This section
  provides miscellaneous CAN-related notes, mostly to myself but perhaps of
  interest to others.

  [Unfortunately, as of this writing, I still do not have a proper CAN test
   bed to verify the CAN driver.]

  CAN Configuration
  -----------------

  The following steps illustrate how to enable CAN0 and/or CAN1 in the NuttX
  configuration:

    System Type -> SAMA5 Peripheral Support
       CONFIG_SAMA5_CAN0=y            : Select CAN0 and/or CAN1
       CONFIG_SAMA5_CAN1=y

    Device Drivers -> CAN Driver Support
       CONFIG_CAN=y                   : (Automatically selected)
       CONFIG_CAN_EXTID=y             : For extended, 29-bit CAN IDs

    System Type -> CAN Drive Support
       CONFIG_SAMA5_CAN0_BAUD=250000  : Select some BAUD for CAN0 (if enabled)
       CONFIG_SAMA5_CAN0_NRECVMB=1    : Select number of receive mailboxes (see below)
       CONFIG_SAMA5_CAN1_BAUD=250000  : Select some BAUD for CAN1 (if enabled)
       CONFIG_SAMA5_CAN1_NRECVMB=1    : Select number of receive mailboxes (see below)

  Receive Mailboxes and Address Filtering
  ---------------------------------------

  The SAMA5 CAN0 peripheral supports 8 mailboxes that can be used for sending
  and receiving messages.  Note that the number of dedicated receive mailboxes
  (CONFIG_SAMA5_CANn_NRECVMB) was set to one in the above configuration.  This
  could be set to any value from 1 to 3 (the upper limit of 3 is purely
  arbrary and can be increased with some minor code enhancement).  The
  remainder can be configured dynamically to send CAN messages.

  Why would you want to use more than one receive mailbox?  There are two
  reasons. Multiple receive mailboxes might needed to either (1) receive
  bursts of messages, or (2) to support multiple groups of messages filtered
  on message ID.

  You must also specify the address filtering for each dedicated receive mailbox:

    System Type -> CAN Drive Support
       CONFIG_SAMA5_CAN0_ADDR0 and CONFIG_SAMA5_CAN0_MASK0 : If CONFIG_SAMA5_CAN0_NRECVMB >= 1
       CONFIG_SAMA5_CAN0_ADDR1 and CONFIG_SAMA5_CAN0_MASK1 : If CONFIG_SAMA5_CAN0_NRECVMB >= 2
       CONFIG_SAMA5_CAN0_ADDR2 and CONFIG_SAMA5_CAN0_MASK2 : If CONFIG_SAMA5_CAN0_NRECVMB >= 3
       CONFIG_SAMA5_CAN1_ADDR0 and CONFIG_SAMA5_CAN1_MASK0 : If CONFIG_SAMA5_CAN1_NRECVMB >= 1
       CONFIG_SAMA5_CAN1_ADDR1 and CONFIG_SAMA5_CAN1_MASK1 : If CONFIG_SAMA5_CAN1_NRECVMB >= 2
       CONFIG_SAMA5_CAN1_ADDR2 and CONFIG_SAMA5_CAN1_MASK2 : If CONFIG_SAMA5_CAN1_NRECVMB >= 3

  Only messages that have IDs that match the CONFIG_SAMA5_CANn_ADDRn when both
  the received and the configured address are masked by CONFIG_SAMA5_CANn_MASKn
  will be accepted.  For example, if the mask is all ones, then only messages
  with exact address matches will be accepted; if the mask is all zeroes than
  any address will be accepted.

  CAN connectors
  --------------

  CAN1 and CAN2 are available via RJ-11 connectors on the SAMA5D3-Xplained.  Each
  is wired as follows.  Also shown below is the matching pins if you want connect
  the CAN to a device that uses an DB-9 connector (Such as the IXXAT USB-to-CAN
  Compact).  Both connector types (as well as RJ-45) are common.

                    +----------+     RJ-11       DB-9
                    |    O     |     ----------- --------------
  +------------+    |          |     Pin 1 3v3   Pin 1 N/C
  |    +--+    |    |  o5      |     Pin 2 5v    Pin 2 CANL
  |    |  |    |    |       o9 |     Pin 3 N/C   Pin 3 GND
  |  +-+  +-+  |    |  o4      |     Pin 4 CANL  Pin 4 N/C
  |  |      |  |    |       o8 |     Pin 5 CANH  Pin 5 N/C
  |  |654321|  |    |  o3      |     Pin 6 N/C   Pin 6 N/C
  |  |oooooo|  |    |       o7 |                 Pin 7 CANH
  |  +------+  |    |  o2      |                 Pin 8 N/C
  +------------+    |       o6 |                 Pin 9 CANV+ (N/C on IXXAT)   RJ-11 Female     |  x1      |
                    |          |
                    |    O     |
                    +----------+
                      DB-9 Male

SAMA5 ADC Support
=================

  Basic driver configuration
  --------------------------
  ADC support can be added to the NSH configuration.  However, there are no
  ADC input pins available to the user for ADC testing (the touchscreen ADC
  inputs are intended for other functionality).  Because of this, there is
  not much motivation to enable ADC support on the SAMA5D3-Xplained.  This
  paragraph is included here, however, for people using a custom SAMA5D3x
  board that requires ADC support.

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_ADC=y               : Enable ADC driver support
      CONFIG_SAMA5_TC0=y               : Enable the Timer/counter library need for periodic sampling

    Drivers
      CONFIG_ANALOG=y                  : Should be automatically selected
      CONFIG_ADC=y                     : Should be automatically selected

    System Type -> ADC Configuration
      CONFIG_SAMA5_ADC_CHAN0=y         : These settings enable the sequencer to collect
      CONFIG_SAMA5_ADC_CHAN1=y         : Samples from ADC channels 0-3 on each trigger
      CONFIG_SAMA5_ADC_CHAN2=y
      CONFIG_SAMA5_ADC_CHAN3=y
      CONFIG_SAMA5_ADC_SEQUENCER=y

      CONFIG_SAMA5_ADC_TIOA0TRIG=y     : Trigger on the TC0, channel 0 output A
      CONFIG_SAMA5_ADC_TIOAFREQ=2      : At a frequency of 2Hz
      CONFIG_SAMA5_ADC_TIOA_RISING=y   : Trigger on the rising edge

    Default ADC settings (like gain and offset) may also be set if desired.

    System Type -> Timer/counter Configuration
      CONFIG_SAMA5_TC0_TIOA0=y         : Should be automatically selected

  Work queue supported is also needed:

    Library routines
      CONFIG_SCHED_WORKQUEUE=y

  ADC Test Example
  ----------------
  For testing purposes, there is an ADC program at apps/examples/adc that
  will collect a specified number of samples.  This test program can be
  enabled as follows:

    Application Configuration -> Examples -> ADC example
      CONFIG_EXAMPLES_ADC=y            : Enables the example code
      CONFIG_EXAMPLES_ADC_DEVPATH="/dev/adc0"

    Other default settings for the ADC example should be okay.

  ADC DMA Support
  ---------------
  At 2Hz, DMA is not necessary nor desire-able.  The ADC driver has support
  for DMA transfers of converted data (although that support has not been
  tested as of this writing).  DMA support can be added by include the
  following in the configuration.

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_DMAC1=y             : Enable DMAC1 support

    System Type -> ADC Configuration
      CONFIG_SAMA5_ADC_DMA=y           : Enable ADC DMA transfers
      CONFIG_SAMA5_ADC_DMASAMPLES=2    : Collect two sets of samples per DMA

    Drivers -> Analog device (ADC/DAC) support
      CONFIG_ADC_FIFOSIZE=16           : Driver may need a large ring buffer

    Application Configuration -> Examples -> ADC example
      CONFIG_EXAMPLES_ADC_GROUPSIZE=16 : Larger buffers in the test

SAMA5 PWM Support
=================

  Basic driver configuration
  --------------------------
  PWM support can be added to the NSH configuration.  However, there are no
  PWM output pins available to the user for PWM testing.  Because of this,
  there is not much motivation to enable PWM support on the SAMA5D3-Xplained.  This
  paragraph is included here, however, for people using a custom SAMA5D3x
  board that requires PWM support.

  Basic driver configuration:

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_PWM=y               : Enable PWM driver support

    Drivers
      CONFIG_PWM=y                     : Should be automatically selected

    PWM Channel/Output Selection
    ----------------------------
    In order to use the PWM, you must enable one or more PWM Channels:

    System Type -> PWM Configuration
      CONFIG_SAMA5_PWM_CHAN0=y         : Enable one or more of channels 0-3
      CONFIG_SAMA5_PWM_CHAN1=y
      CONFIG_SAMA5_PWM_CHAN2=y
      CONFIG_SAMA5_PWM_CHAN3=y

    For each channel that is enabled, you must also specify the output pins
    to be enabled and the clocking supplied to the PWM channel.

      CONFIG_SAMA5_PWM_CHANx_FAULTINPUT=n : (not used currently)
      CONFIG_SAMA5_PWM_CHANx_OUTPUTH=y  : Enable One of both of the H and L output pins
      CONFIG_SAMA5_PWM_CHANx_OUTPUTL=y

    Where x=0..3.

    Care must be taken because all PWM output pins conflict with some other
    usage of the pin by other devices.  Furthermore, many of these pins have
    not been brought out to an external connector:

      -----+---+---+----+------+----------------
       PWM  PIN PER PIO   I/O   CONFLICTS
      -----+---+---+----+------+----------------
       PWM0 FI   B  PC28 J2.30  SPI1, ISI
            H    B  PB0   ---   GMAC
                 B  PA20 J1.14  LCDC, ISI
            L    B  PB1   ---   GMAC
                 B  PA21 J1.16  LCDC, ISI
      -----+---+---+----+------+----------------
       PWM1 FI   B  PC31 J2.36  HDMI
            H    B  PB4   ---   GMAC
                 B  PA22 J1.18  LCDC, ISI
            L    B  PB5   ---   GMAC
                 B  PE31 J3.20  ISI, HDMI
                 B  PA23 J1.20  LCDC, ISI
      -----+---+---+----+------+----------------
       PWM2 FI   B  PC29 J2.29  UART0, ISI, HDMI
            H    C  PD5   ---   HSMCI0
                 B  PB8   ---   GMAC
            L    C  PD6   ---   HSMCI0
                 B  PB9   ---   GMAC
      -----+---+---+----+------+----------------
       PWM3 FI   C  PD16  ---  SPI0, Audio
            H    C  PD7   ---  HSMCI0
                 B  PB12 J3.7  GMAC
            L    C  PD8   ---  HSMCI0
                 B  PB13  ---  GMAC
      -----+---+---+----+--------------------

    See boards/arm/sama5/sama5d3-xplained/include/board.h for all of the default PWM
    pin selections.  I used PWM channel 0, pins PA20 and PA21 for testing.

    Clocking is addressed in the next paragraph.

    PWM Clock Configuration
    -----------------------
    PWM Channels can be clocked from either a coarsely divided divided down
    MCK or from a custom frequency from PWM CLKA and/or CLKB.  If you want
    to use CLKA or CLKB, you must enable and configure them.

    System Type -> PWM Configuration
      CONFIG_SAMA5_PWM_CLKA=y
      CONFIG_SAMA5_PWM_CLKA_FREQUENCY=3300
      CONFIG_SAMA5_PWM_CLKB=y
      CONFIG_SAMA5_PWM_CLKB_FREQUENCY=3300

    Then for each of the enabled, channels you must select the input clock
    for that channel:

    System Type -> PWM Configuration
      CONFIG_SAMA5_PWM_CHANx_CLKA=y     : Pick one of MCK, CLKA, or CLKB (only)
      CONFIG_SAMA5_PWM_CHANx_CLKB=y
      CONFIG_SAMA5_PWM_CHANx_MCK=y
      CONFIG_SAMA5_PWM_CHANx_MCKDIV=128 : If MCK is selected, then the MCK divider must
                                        : also be provided (1,2,4,8,16,32,64,128,256,512, or 1024).

  PWM Test Example
  ----------------
  For testing purposes, there is an PWM program at apps/examples/pwm that
  will collect a specified number of samples.  This test program can be
  enabled as follows:

    Application Configuration -> Examples -> PWM example
      CONFIG_EXAMPLES_PWM=y            : Enables the example code

    Other default settings for the PWM example should be okay.

      CONFIG_EXAMPLES_PWM_DEVPATH="/dev/pwm0"
      CONFIG_EXAMPLES_PWM_FREQUENCY=100

  Usage of the example is straightforward:

    nsh> pwm -h
    Usage: pwm [OPTIONS]

    Arguments are "sticky".  For example, once the PWM frequency is
    specified, that frequency will be re-used until it is changed.

    "sticky" OPTIONS include:
      [-p devpath] selects the PWM device.  Default: /dev/pwm0 Current: /dev/pwm0
      [-f frequency] selects the pulse frequency.  Default: 100 Hz Current: 100 Hz
      [-d duty] selects the pulse duty as a percentage.  Default: 50 % Current: 50 %
      [-t duration] is the duration of the pulse train in seconds.  Default: 5 Current: 5
      [-h] shows this message and exits

RTC
===

  The Real Time Clock/Calendar RTC) may be enabled with these settings:

    System Type:
      CONFIG_SAMA5_RTC=y                   : Enable the RTC driver

    Drivers (these values will be selected automatically):
      CONFIG_RTC=y                         : Use the RTC for system time
      CONFIG_RTC_DATETIME=y                : RTC supports data/time

  You can set the RTC using the NSH date command:

    NuttShell (NSH) NuttX-7.3
    nsh> help date
    date usage:  date [-s "MMM DD HH:MM:SS YYYY"]
    nsh> date
    Jan 01 00:34:45 2012
    nsh> date -s "JUN 29 7:30:00 2014"
    nsh> date
    Jun 29 07:30:01 2014

  After a power cycle and reboot:

    NuttShell (NSH) NuttX-7.3
    nsh> date
    Jun 29 07:30:55 2014
    nsh>

  The RTC also supports an alarm that may be enable with the following
  settings.  However, there is nothing in the system that currently makes
  use of this alarm.

    Drivers:
      CONFIG_RTC_ALARM=y                   : Enable the RTC alarm

    Library Routines
      CONFIG_SCHED_WORKQUEUE=y              : Alarm needs work queue support

Watchdog Timer
==============

  NSH can be configured to exercise the watchdog timer test
  (apps/examples/watchdog).  This can be selected with the following
  settings in the NuttX configuration file:

    System Type:
      CONFIG_SAMA5_WDT=y                  : Enable the WDT peripheral
                                          : Defaults in "RTC Configuration" should be OK

    Drivers (this will automatically be selected):
      CONFIG_WATCHDOG=y                   : Enables watchdog timer driver support

    Application Configuration -> Examples
      CONFIG_EXAMPLES_WATCHDOG=y          : Enable apps/examples/watchdog

  The WDT timer is driven off the slow, 32768Hz clock divided by 128. As a
  result, the watchdog a maximum timeout value of 16 seconds.  The SAMA5 WDT
  may also only be programmed one time; the processor must be reset before
  the WDT can be reprogrammed.

  The SAMA5 always boots with the watchdog timer enabled at its maximum
  timeout (16 seconds).  In the normal case where no watchdog timer driver
  has been configured, the watchdog timer is disabled as part of the start
  up logic.  But, since we are permitted only one opportunity to program
  the WDT, we cannot disable the watchdog time if CONFIG_SAMA5_WDT=y.  So,
  be forewarned:  You have only 16 seconds to run your watchdog timer test!

TRNG and /dev/random
====================

  NSH can be configured to enable the SAMA5 TRNG peripheral so that it
  provides /dev/random.  The following configuration will enable the TRNG,
  and support for /dev/random:

    System Type:
      CONFIG_SAMA5_TRNG=y                 : Enable the TRNG peripheral

    Drivers:
      CONFIG_DEV_RANDOM=y                 : Enable /dev/random

  A simple test of /dev/random is available at apps/examples/random and
  can be enabled as a NSH application via the following additional
  configuration settings:

    Applications -> Examples
      CONFIG_EXAMPLES_RANDOM=y            : Enable apps/examples/random
      CONFIG_EXAMPLES_MAXSAMPLES=64       : Default settings are probably OK
      CONFIG_EXAMPLES_NSAMPLES=8

Tickless OS
===========

  Background
  ----------
  By default, a NuttX configuration uses a periodic timer interrupt that
  drives all system timing. The timer is provided by architecture-specifi
  code that calls into NuttX at a rate controlled by CONFIG_USEC_PER_TICK.
  The default value of CONFIG_USEC_PER_TICK is 10000 microseconds which
  corresponds to a timer interrupt rate of 100 Hz.

  An option is to configure NuttX to operation in a "tickless" mode. Some
  limitations of default system timer are, in increasing order of
  importance:

  - Overhead: Although the CPU usage of the system timer interrupt at 100Hz
    is really very low, it is still mostly wasted processing time. One most
    timer interrupts, there is really nothing that needs be done other than
    incrementing the counter.
  - Resolution: Resolution of all system timing is also determined by
    CONFIG_USEC_PER_TICK. So nothing that be time with resolution finer than
    10 milliseconds be default. To increase this resolution,
    CONFIG_USEC_PER_TICK an be reduced. However, then the system timer
    interrupts use more of the CPU bandwidth processing useless interrupts.
  - Power Usage: But the biggest issue is power usage. When the system is
    IDLE, it enters a light, low-power mode (for ARMs, this mode is entered
    with the wfi or wfe instructions for example). But each interrupt
    awakens the system from this low power mode. Therefore, higher rates
    of interrupts cause greater power consumption.

  The so-called Tickless OS provides one solution to issue. The basic
  concept here is that the periodic, timer interrupt is eliminated and
  replaced with a one-shot, interval timer. It becomes event driven
  instead of polled: The default system timer is a polled design. On
  each interrupt, the NuttX logic checks if it needs to do anything
  and, if so, it does it.

  Using an interval timer, one can anticipate when the next interesting
  OS event will occur, program the interval time and wait for it to fire.
  When the interval time fires, then the scheduled activity is performed.

  Configuration
  -------------
  The following configuration options will enable support for the Tickless
  OS for the SAMA5D platforms using TC0 channels 0-3 (other timers or
  timer channels could be used making the obvious substitutions):

    RTOS Features -> Clocks and Timers
      CONFIG_SCHED_TICKLESS=y          : Configures the RTOS in tickless mode
      CONFIG_SCHED_TICKLESS_ALARM=n    : (option not implemented)

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_TC0=y               : Enable TC0 (TC channels 0-3

    System Type -> Timer/counter Configuration
      CONFIG_SAMA5_ONESHOT=y           : Enables one-shot timer wrapper
      CONFIG_SAMA5_FREERUN=y           : Enabled free-running timer wrapper
      CONFIG_SAMA5_TICKLESS_ONESHOT=0  : Selects TC0 channel 0 for the one-shot
      CONFIG_SAMA5_TICKLESS_FREERUN=1  : Selects TC0 channel 1 for the free-
                                       : running timer

  The resolution of the clock is provided by the CONFIG_USEC_PER_TICK
  setting in the configuration file.

  NOTE: In most cases, the slow clock will be used as the timer/counter
  input.  You should enable the 32.768KHz crystal for the slow clock by
  calling sam_sckc_enable().  Otherwise, you will be doing all system
  timing using the RC clock!  UPDATE: This will now be selected by default
  when you configure for TICKLESS support.

  The slow clock has a resolution of about 30.518 microseconds.  Ideally,
  the value of CONFIG_USEC_PER_TICK should be the exact clock resolution.
  Otherwise there will be cumulative timing inaccuracies.  But a choice
  choice of:

    CONFIG_USEC_PER_TICK=31

  will have an error of 0.6%  and will have inaccuracies that will
  effect the time due to long term error build-up.

  UPDATE: As of this writing (2015-12-03), the Tickless support is
  functional.  However, there are inaccuracies  in delays.  For example,

    nsh> sleep 10

  results in a delay of maybe 5.4 seconds.  But the timing accuracy is
  correct if all competing uses of the interval timer are disabled (mostly
  from the high priority work queue).  Therefore, I conclude that this
  inaccuracy is due to the inaccuracies in the representation of the clock
  rate.  30.518 usec cannot be represented accurately.   Each timing
  calculation results in a small error.  When the interval timer is very
  busy, long delays will be divided into many small pieces and each small
  piece has a large error in the calculation.  The cumulative error is the
  cause of the problem.

  SAMA5 Timer Usage
  -----------------
  This current implementation uses two timers:  A one-shot timer to
  provide the timed events and a free running timer to provide the current
  time.  Since timers are a limited resource, that could be an issue on
  some systems.

  We could do the job with a single timer if we were to keep the single
  timer in a free-running at all times.  The SAMA5 timer/counters have
  32-bit counters with the capability to generate a compare interrupt when
  the timer matches a compare value but also to continue counting without
  stopping (giving another, different interrupt when the timer rolls over
  from 0xffffffff to zero).  So we could potentially just set the compare
  at the number of ticks you want PLUS the current value of timer.  Then
  you could have both with a single timer:  An interval timer and a free-
  running counter with the same timer!  In this case, you would want to
  to set CONFIG_SCHED_TICKLESS_ALARM in the NuttX configuration.

  Patches are welcome!

I2S Audio Support
=================

  The SAMA5D3-Xplained has two devices on-board that can be used for verification
  of I2S functionality:  HDMI and a WM8904 audio CODEC.  As of this writing,
  the I2S driver is present, but there are not drivers for either the HDMI
  or the WM8904.

  WM8904 Audio CODEC Interface
  ----------------------------

    ------------- ---------------- -----------------
    WM8904        SAMA5D3          NuttX Pin Name
    ------------- ---------------- -----------------
     3 SDA        PA30 TWD0        PIO_TWI0_D
     2 SCLK       PA31 TWCK0       PIO_TWI0_CK
    28 MCLK       PD30 PCK0        PIO_PMC_PCK0
    29 BCLK/GPIO4 PC16 TK          PIO_SSC0_TK
    "" "        " PC19 RK          PIO_SSC0_RK
    30 LRCLK      PC17 TF          PIO_SSC0_TF
    "" "   "      PC20 RF          PIO_SSC0_RF
    31 ADCDAT     PC21 RD          PIO_SSC0_RD
    32 DACDAT     PC18 TD          PIO_SSC0_TD
     1 IRQ/GPIO1  PD16 INT_AUDIO   N/A
    ------------- ---------------- -----------------

  I2S Loopback Test
  -----------------

  The I2S driver was verified using a special I2C character driver (at
  nuttx/drivers/audio/i2schar.c) and a test driver at apps/examples/i2schar.
  The I2S driver was verified in loopback mode with no audio device.

  [NOTE: The above statement is anticipatory:  As of this writing I2S driver
   verification is underway and still not complete].

  This section describes the modifications to the NSH configuration that were
  used to perform the I2S testing:

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_SSCO=y              : Enable SSC0 driver support
      CONFIG_SAMA5_DMAC0=y             : DMAC0 required by SSC0

    Alternatively, SSC1 could have be used:

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_SSC1=y              : Enable SSC0 driver support
      CONFIG_SAMA5_DMAC1=y             : DMAC0 required by SSC0

    System Type -> SSC Configuration
      CONFIG_SAMA5_SSC_MAXINFLIGHT=16  : Up to 16 pending DMA transfers
      CONFIG_SAMA5_SSC0_MASTER=y       : Master mode
      CONFIG_SAMA5_SSC0_DATALEN=16     : 16-bit data
      CONFIG_SAMA5_SSC0_RX=y           : Support a receiver
      CONFIG_SAMA5_SSC0_RX_RKINPUT=y   : Receiver gets clock from RK input
      CONFIG_SAMA5_SSC0_TX=y           : Support a transmitter
      CONFIG_SAMA5_SSC0_TX_MCKDIV=y    : Transmitter gets clock from MCK/2
      CONFIG_SAMA5_SSC0_MCKDIV_SAMPLERATE=48000 : Sampling at 48K samples/sec
      CONFIG_SAMA5_SSC0_TX_TKOUTPUT_XFR=y  : Outputs clock on TK when transferring data
      CONFIG_SAMA5_SSC0_LOOPBACK=y     : Loopmode mode connects RD/TD and RK/TK

    Audio
      CONFIG_AUDIO=y                   : Audio support needed
                                       : Defaults should be okay

    Drivers -> Audio
      CONFIG_I2S=y                     : General I2S support
      CONFIG_DRIVERS_AUDIO=y           : Audio device support
      CONFIG_AUDIO_I2SCHAR=y           : Build I2S character driver

    The following describes how I have the test application at
    apps/examples/i2schar configured:

      CONFIG_EXAMPLES_I2SCHAR=y
      CONFIG_EXAMPLES_I2SCHAR_DEVPATH="/dev/i2schar0"
      CONFIG_EXAMPLES_I2SCHAR_TX=y
      CONFIG_EXAMPLES_I2SCHAR_TXBUFFERS=4
      CONFIG_EXAMPLES_I2SCHAR_TXSTACKSIZE=1536
      CONFIG_EXAMPLES_I2SCHAR_RX=y
      CONFIG_EXAMPLES_I2SCHAR_RXBUFFERS=4
      CONFIG_EXAMPLES_I2SCHAR_RXSTACKSIZE=1536
      CONFIG_EXAMPLES_I2SCHAR_BUFSIZE=256
      CONFIG_EXAMPLES_I2SCHAR_DEVINIT=y

    Board Selection
      CONFIG_SAMA5D3XPLAINED_I2SCHAR_MINOR=0
      CONFIG_SAMA5D3XPLAINED_SSC_PORT=0     : 0 or SSC0, 1 for SSC1

    Library Routines
      CONFIG_SCHED_WORKQUEUE=y          : Driver needs work queue support

Shields
=======

  Support is built in for the following shields:

  Itead Joystick Shield
  ---------------------
  See http://imall.iteadstudio.com/im120417014.html for more information
  about this joystick.

  Itead Joystick Connection:

    --------- ----------------- ---------------------------------
    ARDUINO   ITEAD             SAMA5D3 XPLAINED
    PIN NAME  SIGNAL            CONNECTOR  SIGNAL
    --------- ----------------- ---------- ----------------------
     D3       Button E Output   J18 pin 4  PC8
     D4       Button D Output   J18 pin 5  PC28
     D5       Button C Output   J18 pin 6  PC7
     D6       Button B Output   J18 pin 7  PC6
     D7       Button A Output   J18 pin 8  PC5
     D8       Button F Output   J15 pin 1  PC4
     D9       Button G Output   J15 pin 2  PC3
     A0       Joystick Y Output J17 pin 1  PC18  AD0 (function 4)
     A1       Joystick X Output J17 pin 2  PD21  AD1 (function 1)
    --------- ----------------- ---------- ----------------------

    All buttons are pulled on the shield.  A sensed low value indicates
    when the button is pressed.

  Possible conflicts:

    ---- ----- --------------------------------------------------
    ARDU SAMA5 SAMA5D3 XPLAINED
    PIN  GPIO  SIGNAL            FUNCTION
    ---- ----- ----------------- --------------------------------
     D3  PC8   EMDC              10/100Mbit Ethernet MAC
     D4  PC28  SPI1_NPCS3/ISI_D9 SPI1/ISI
     D5  PC7   EREFCK            10/100Mbit Ethernet MAC
     D6  PC6   ECRSDV            10/100Mbit Ethernet MAC
     D7  PC5   ECRSDV            10/100Mbit Ethernet MAC
     D8  PC4   ETXEN             10/100Mbit Ethernet MAC
     D9  PC3   ERX1              10/100Mbit Ethernet MAC
     A0  PC18  RK0               SSC/Audio
     A1  PC21  RD0               SSC/Audio
    ---- ----- ----------------- --------------------------------

  Itead Joystick Signal interpretation:

    --------- ----------------------- ---------------------------
    BUTTON     TYPE                    NUTTX ALIAS
    --------- ----------------------- ---------------------------
    Button A  Large button A          JUMP/BUTTON 3
    Button B  Large button B          FIRE/BUTTON 2
    Button C  Joystick select button  SELECT/BUTTON 1
    Button D  Tiny Button D           BUTTON 6
    Button E  Tiny Button E           BUTTON 7
    Button F  Large Button F          BUTTON 4
    Button G  Large Button G          BUTTON 5
    --------- ----------------------- ---------------------------

  Itead Joystick configuration settings:

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_ADC=y               : Enable ADC driver support
      CONFIG_SAMA5_TC0=y               : Enable the Timer/counter library need for periodic sampling
      CONFIG_SAMA5_EMACA=n             : 10/100Mbit Ethernet MAC conflicts
      CONFIG_SAMA5_SSC0=n              : SSC0 Audio conflicts
      CONFIG_SAMA5_SPI1=?              : SPI1 might conflict if PCS3 is used
      CONFIG_SAMA5_ISI=?               : ISIS conflics if bit 9 is used

    System Type -> PIO Interrupts
      CONFIG_SAMA5_PIO_IRQ=y           : PIO interrupt support is required
      CONFIG_SAMA5_PIOC_IRQ=y          : PIOC interrupt support is required

    Drivers
      CONFIG_ANALOG=y                  : Should be automatically selected
      CONFIG_ADC=y                     : Should be automatically selected
      CONFIG_INPUT=y                   : Select input device support
      CONFIG_INPUT_AJOYSTICK=y         : Select analog joystick support

    System Type -> ADC Configuration
      CONFIG_SAMA5_ADC_CHAN0=y         : These settings enable the sequencer to collect
      CONFIG_SAMA5_ADC_CHAN1=y         : Samples from ADC channels 0-1 on each trigger
      CONFIG_SAMA5_ADC_SEQUENCER=y
      CONFIG_SAMA5_ADC_TIOA0TRIG=y     : Trigger on the TC0, channel 0 output A
      CONFIG_SAMA5_ADC_TIOAFREQ=10     : At a frequency of 10Hz
      CONFIG_SAMA5_ADC_TIOA_RISING=y   : Trigger on the rising edge

    Default ADC settings (like gain and offset) may also be set if desired.

    System Type -> Timer/counter Configuration
      CONFIG_SAMA5_TC0_TIOA0=y         : Should be automatically selected

    Library routines
      CONFIG_SCHED_WORKQUEUE=y         : Work queue support is needed

  There is nothing in the configuration that currently uses the joystick.
  For testing, you can add the following configuration options to enable the
  analog joystick example at apps/examples/ajoystick:

    CONFIG_NSH_ARCHINIT=y
    CONFIG_EXAMPLES_AJOYSTICK=y
    CONFIG_EXAMPLES_AJOYSTICK_DEVNAME="/dev/ajoy0"

  STATUS:
  2014-12-03:  As nearly I can tell, the Itead Joystick shield cannot be
    used with the SAMA5D3-Xplained.  I believe that the EMAC PHY chip is
    enableed and since it shares pins with the Joystick, it interferes with
    the Joystick inputs.  There is probably more wrong than this; perhaps I
    am not setting up the pins correctly.  But having seen the states of the
    button output pins change when powering up the board, I have lost hope
    of getting the shield to work on this board.  I leave the
    implementation in place only for reference.

SAMA5D3-Xplained Configuration Options
=================================

  CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
  be set to:

    CONFIG_ARCH="arm"

  CONFIG_ARCH_family - For use in C code:

    CONFIG_ARCH_ARM=y

  CONFIG_ARCH_architecture - For use in C code:

    CONFIG_ARCH_CORTEXA5=y

  CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

    CONFIG_ARCH_CHIP="sama5"

  CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
  chip:

    CONFIG_ARCH_CHIP_SAMA5=y

  and one of:

   CONFIG_ARCH_CHIP_ATSAMA5D31=y
   CONFIG_ARCH_CHIP_ATSAMA5D33=y
   CONFIG_ARCH_CHIP_ATSAMA5D34=y
   CONFIG_ARCH_CHIP_ATSAMA5D35=y

  CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
  hence, the board that supports the particular chip or SoC.

    CONFIG_ARCH_BOARD="sama5d3-xplained" (for the SAMA5D3-Xplained development board)

  CONFIG_ARCH_BOARD_name - For use in C code

    CONFIG_ARCH_BOARD_SAMA5D3_XPLAINED=y

  CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
  of delay loops

  CONFIG_ENDIAN_BIG - define if big endian (default is little
  endian)

  CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

    CONFIG_RAM_SIZE=0x0002000 (128Kb)

  CONFIG_RAM_START - The physical start address of installed DRAM

    CONFIG_RAM_START=0x20000000

  CONFIG_RAM_VSTART - The virtual start address of installed DRAM

    CONFIG_RAM_VSTART=0x20000000

  CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
  have LEDs

  CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
  stack. If defined, this symbol is the size of the interrupt
  stack in bytes.  If not defined, the user task stacks will be
  used during interrupt handling.

  CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

  CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

  Individual subsystems can be enabled:

    CONFIG_SAMA5_DBGU        - Debug Unit
    CONFIG_SAMA5_PIT         - Periodic Interval Timer
    CONFIG_SAMA5_WDT         - Watchdog timer
    CONFIG_SAMA5_HSMC        - Multi-bit ECC
    CONFIG_SAMA5_SMD         - SMD Soft Modem
    CONFIG_SAMA5_USART0      - USART 0
    CONFIG_SAMA5_USART1      - USART 1
    CONFIG_SAMA5_USART2      - USART 2
    CONFIG_SAMA5_USART3      - USART 3
    CONFIG_SAMA5_UART0       - UART 0
    CONFIG_SAMA5_UART1       - UART 1
    CONFIG_SAMA5_TWI0        - Two-Wire Interface 0
    CONFIG_SAMA5_TWI1        - Two-Wire Interface 1
    CONFIG_SAMA5_TWI2        - Two-Wire Interface 2
    CONFIG_SAMA5_HSMCI0      - High Speed Multimedia Card Interface 0
    CONFIG_SAMA5_HSMCI1      - High Speed Multimedia Card Interface 1
    CONFIG_SAMA5_HSMCI2      - High Speed Multimedia Card Interface 2
    CONFIG_SAMA5_SPI0        - Serial Peripheral Interface 0
    CONFIG_SAMA5_SPI1        - Serial Peripheral Interface 1
    CONFIG_SAMA5_TC0         - Timer Counter 0 (ch. 0, 1, 2)
    CONFIG_SAMA5_TC1         - Timer Counter 1 (ch. 3, 4, 5)
    CONFIG_SAMA5_PWM         - Pulse Width Modulation Controller
    CONFIG_SAMA5_ADC         - Touch Screen ADC Controller
    CONFIG_SAMA5_DMAC0       - DMA Controller 0
    CONFIG_SAMA5_DMAC1       - DMA Controller 1
    CONFIG_SAMA5_UHPHS       - USB Host High Speed
    CONFIG_SAMA5_UDPHS       - USB Device High Speed
    CONFIG_SAMA5_GMAC        - Gigabit Ethernet MAC
    CONFIG_SAMA5_EMACA       - Ethernet MAC (type A)
    CONFIG_SAMA5_LCDC        - LCD Controller
    CONFIG_SAMA5_ISI         - Image Sensor Interface
    CONFIG_SAMA5_SSC0        - Synchronous Serial Controller 0
    CONFIG_SAMA5_SSC1        - Synchronous Serial Controller 1
    CONFIG_SAMA5_CAN0        - CAN controller 0
    CONFIG_SAMA5_CAN1        - CAN controller 1
    CONFIG_SAMA5_SHA         - Secure Hash Algorithm
    CONFIG_SAMA5_AES         - Advanced Encryption Standard
    CONFIG_SAMA5_TDES        - Triple Data Encryption Standard
    CONFIG_SAMA5_TRNG        - True Random Number Generator
    CONFIG_SAMA5_ARM         - Performance Monitor Unit
    CONFIG_SAMA5_FUSE        - Fuse Controller
    CONFIG_SAMA5_MPDDRC      - MPDDR controller

  Some subsystems can be configured to operate in different ways. The drivers
  need to know how to configure the subsystem.

    CONFIG_SAMA5_PIOA_IRQ    - Support PIOA interrupts
    CONFIG_SAMA5_PIOB_IRQ    - Support PIOB interrupts
    CONFIG_SAMA5_PIOC_IRQ    - Support PIOD interrupts
    CONFIG_SAMA5_PIOD_IRQ    - Support PIOD interrupts
    CONFIG_SAMA5_PIOE_IRQ    - Support PIOE interrupts

    CONFIG_USART0_SERIALDRIVER - USART0 is configured as a UART
    CONFIG_USART1_SERIALDRIVER - USART1 is configured as a UART
    CONFIG_USART2_SERIALDRIVER - USART2 is configured as a UART
    CONFIG_USART3_SERIALDRIVER - USART3 is configured as a UART

  AT91SAMA5 specific device driver settings

    CONFIG_SAMA5_DBGU_SERIAL_CONSOLE - selects the DBGU
      for the console and ttyDBGU
    CONFIG_SAMA5_DBGU_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_SAMA5_DBGU_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_SAMA5_DBGU_BAUD - The configure BAUD of the DBGU.
    CONFIG_SAMA5_DBGU_PARITY - 0=no parity, 1=odd parity, 2=even parity

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=0,1,2,3) or UART
           m (m=4,5) for the console and ttys0 (default is the DBGU).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARITY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

  AT91SAMA5 USB Host Configuration
  Pre-requisites

    CONFIG_USBDEV          - Enable USB device support
    CONFIG_USBHOST         - Enable USB host support
    CONFIG_SAMA5_UHPHS     - Needed
    CONFIG_SAMA5_OHCI      - Enable the STM32 USB OTG FS block
    CONFIG_SCHED_WORKQUEUE - Worker thread support is required

  Options:

    CONFIG_SAMA5_OHCI_NEDS
      Number of endpoint descriptors
    CONFIG_SAMA5_OHCI_NTDS
      Number of transfer descriptors
    CONFIG_SAMA5_OHCI_TDBUFFERS
      Number of transfer descriptor buffers
    CONFIG_SAMA5_OHCI_TDBUFSIZE
      Size of one transfer descriptor buffer
    CONFIG_USBHOST_INT_DISABLE
      Disable interrupt endpoint support
    CONFIG_USBHOST_ISOC_DISABLE
      Disable isochronous endpoint support
    CONFIG_USBHOST_BULK_DISABLE
      Disable bulk endpoint support

config SAMA5_OHCI_REGDEBUG

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each SAMA5D3-Xplained configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh sama5d3-xplained:<subdir>

  Before building, make sure that the PATH environment variable include the
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

  2. Unless stated otherwise, all configurations generate console
     output on the DBGU (J23).

  3. All of these configurations use the Code Sourcery for Windows toolchain
     (unless stated otherwise in the description of the configuration).  That
     toolchain selection can easily be reconfigured using 'make menuconfig'.
     Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Microsoft Windows
       CONFIG_WINDOWS_CYGWIN=y             : Using Cygwin or other POSIX environment

     System Type -> Toolchain:
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y  : GNU EABI toolchain

  4. The SAMA5Dx is running at 396MHz by default in these configurations.
     This is because the original timing for the PLLs, NOR FLASH, and SDRAM
     came from the Atmel NoOS sample code which runs at that rate.

     The SAMA5Dx is capable of running at 528MHz, however, and is easily
     re-configured:

       Board Selection -> CPU Frequency
         CONFIG_SAMA5D3XEK_396MHZ=n     # Disable 396MHz operation
         CONFIG_SAMA5D3XEK_528MHZ=y     # Enable 528MHz operation

     If you switch to 528MHz, you should also check the loop calibration
     value in your .config file.  Of course, it would be best to re-calibrate
     the timing loop, but these values should get you in the ballpark:

       CONFIG_BOARD_LOOPSPERMSEC=49341  # Calibrated on SAMA5D3-EK at 396MHz
                                        # running from ISRAM
       CONFIG_BOARD_LOOPSPERMSEC=65775  # Calibrated on SAMA4D3-Xplained at
                                        # 528MHz running from SDRAM

     Operation at 528MHz has been verified but is not the default in these
     configurations because most testing was done at 396MHz.  NAND has not
     been verified at these rates.

  Configuration Sub-directories
  -----------------------------
  Summary:  Some of the descriptions below are long and wordy. Here is the
  concise summary of the available SAMA5D3-Xplained configurations:

    bridge:  This is a simple testing that exercises EMAC and GMAC for
      a simple UDP relay bridge test.
    nsh:  This is another NSH configuration, not too different from the
      demo configuration.  The nsh configuration is, however, bare bones.
      It is the simplest possible NSH configuration and is useful as a
      platform for debugging and integrating new features in isolation.

  There may be issues with some of these configurations.  See the details
  before of the status of individual configurations.

  Now for the gory details:

  bridge:

    This is a simple testing that exercises EMAC and GEMAC for a simple
    UDP relay bridge test using apps/examples/bridge.  See
    apps/examples/README.txt for more information about this test.

    NOTES:

    1. This configuration uses the default DBGU serial console.  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    2. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain.  Both the build environment
       and the toolchain selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under Windows
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain

       If you are running on Linux, make *certain* that you have
       CONFIG_HOST_LINUX=y *before* the first make or you will create a
       corrupt configuration that may not be easy to recover from. See
       the warning in the section "Information Common to All Configurations"
       for further information.

    3. This configuration executes out of SDRAM flash and is loaded into
       SDRAM from NAND, Serial DataFlash, SD card or from a TFTPC sever via
       U-Boot or BareBox.  Data also is positioned in SDRAM.

       I did most testing with nuttx.bin on an SD card.  These are the
       commands that I used to boot NuttX from the SD card:

         U-Boot> fatload mmc 0 0x20008000 nuttx.bin
         U-Boot> go 0x20008040

    4. You will almost certainly need to adapt this configuration to
       work in your network environment.  I did all testing with a
       single 10.0.0.xx network and a 4+1 port switch:

       - Host PC IP 10.0.0.1
       - Target GMAC IP: 10.0.0.2
       - Target EMAC IP: 10.0.0.3

       Host PC, EMAC, and GMAC were all connected using an Ethernet
       switch to the same 255.255.255.0 network.

    STATUS:

      2014-11-20:  Configuration created.  Partially verified.  Both the
        EMAC and GMAC appear to be function; both respond to pings from
        the host PC.  But I cannot perform the full bridge test yet
        because there still is no host-side test driver in apps/examples/bridge.
      2014-11-21:  Added the host-side test driver and correct a number
        of errors in the test logic.  The testing is working (according
        to WireShark), but I an having some procedural issues related to
        the Windows firewall.

  nsh:

    This configuration directory provide the NuttShell (NSH).  There are
    two NSH configurations:  nsh and demo.  The difference is that nsh is
    intended to be a very simple NSH configuration upon which you can build
    further functionality.  The demo configuration, on the other hand, is
    intended to be a rich configuration that shows many features all working
    together.

    NOTES:
    1. This configuration uses the default DBGU serial console.  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    2. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain.  Both the build environment
       and the toolchain selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under windows
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain

       If you are running on Linux, make *certain* that you have
       CONFIG_HOST_LINUX=y *before* the first make or you will create a
       corrupt configuration that may not be easy to recover from. See
       the warning in the section "Information Common to All Configurations"
       for further information.

    3. This configuration executes out of SDRAM flash and is loaded into
       SDRAM from NAND, Serial DataFlash, SD card or from a TFTPC sever via
       U-Boot or BareBox.  Data also is positioned in SDRAM.

       I did most testing with nuttx.bin on an SD card.  These are the
       commands that I used to boot NuttX from the SD card:

         U-Boot> fatload mmc 0 0x20008000 nuttx.bin
         U-Boot> go 0x20008040

    4. This configuration has support for NSH built-in applications enabled.
       However, no built-in applications are selected in the base configuration.

    5. This configuration has support for the FAT file system built in.  However,
       by default, there are no block drivers initialized.  The FAT file system can
       still be used to create RAM disks.

    6. The SAMA5D3 Xplained board includes an option serial DataFlash.  Support
       for that serial FLASH can be enabled by modifying the NuttX configuration
       as described above in the paragraph entitled "AT25 Serial FLASH".

    7. Enabling HSMCI support. The SAMA5D3-Xplained provides a two SD memory
       card slots:  (1) a full size SD card slot (J10), and (2) a microSD
       memory card slot (J11).  The full size SD card slot connects via HSMCI0;
       the microSD connects vi HSMCI1.  Support for both SD slots can be enabled
       with the settings provided in the paragraph entitled "HSMCI Card Slots"
       above.

    8. Support the USB low-, high- and full-speed OHCI host driver can be enabled
       by changing the NuttX configuration file as described in the section
       entitled "USB High-Speed Host" above.

    9. Support the USB high-speed USB device driver (UDPHS) can be enabled
       by changing the NuttX configuration file as described above in the
       section entitled "USB High-Speed Device."

    10. I2C Tool. NuttX supports an I2C tool at apps/system/i2c that can be
        used to peek and poke I2C devices.  See the discussion above under
        "I2C Tool" for detailed configuration settings.

    11. Networking support via the can be added to NSH by modifying the
        configuration.  See the "Networking" section above for detailed
        configuration settings.

    12. The Real Time Clock/Calendar (RTC) may be enabled by reconfiguring NuttX.
        See the section entitled "RTC" above for detailed configuration settings.

    13. This example can be configured to exercise the watchdog timer test
        (apps/examples/watchdog).  See the detailed configuration settings in
        the section entitled "Watchdog Timer" above.

    14. This example can be configured to enable the SAMA5 TRNG peripheral so
        that it provides /dev/random.  See the section entitled "TRNG and
        /dev/random" above for detailed configuration information.

    16. See also the sections above for additional configuration options:
        "CAN Usage", "SAMA5 ADC Support", "SAMA5 PWM Support", "I2S Audio
        Support"

    STATUS:
       See the To-Do list below

       2014-4-3:  Delay loop calibrated: CONFIG_BOARD_LOOPSPERMSEC=65775

To-Do List
==========

1) Neither USB OHCI nor EHCI support Isochronous endpoints.  Interrupt
   endpoint support in the EHCI driver is untested (but works in similar
   EHCI drivers).

2) HSCMI. CONFIG_MMCSD_MULTIBLOCK_LIMIT=1 is set to disable multi-block
   transfers because of some issues that I saw during testing.  The is very
   low priority to me but might be important to you if you are need very
   high performance SD card accesses.

   HSMCI TX DMA is currently disabled for the SAMA5D3.  There is some
   issue with the TX DMA setup (HSMCI TX DMA the same driver works with
   the SAMA5D4 which has a different DMA subsystem).  This is a bug that
   needs to be resolved.

   UPDATE:  This problem may be fixed with a bug correct on 2015-03-15).
   Need to retest.  That change is necessary, but may not be sufficient to
   solve the problem.

3) GMAC has only been tested on a 10/100Base-T network.  I don't have a
   1000Base-T network to support additional testing.

4) Some drivers may require some adjustments if you intend to run from SDRAM.
   That is because in this case macros like BOARD_MCK_FREQUENCY are not constants
   but are instead function calls:  The MCK clock frequency is not known in
   advance but instead has to be calculated from the bootloader PLL configuration.

   As of this writing, all drivers have been converted to run from SDRAM except
   for the PWM and the Timer/Counter drivers.  These drivers use the
   BOARD_MCK_FREQUENCY definition in more complex ways and will require some
   minor redesign and re-testing before they can be available.

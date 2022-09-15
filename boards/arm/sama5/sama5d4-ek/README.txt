README
======

  This README file describes the port of NuttX to the SAMA4D4-EK
  development board. This board features the Atmel SAMA5D44 microprocessor.
  See http://www.atmel.com for further information.

  This port was actually performed on a board designated SAMA5D4-MB.  This
  board should be equivalent to the SAMA5D4-EK.  However, care should be
  taken when I refer to PIO, Connector, or Jumper Usage in this document.
  Please consult the schematic for your actual board-in-hand to verify that
  information.

  SAMA5D44
  --------

    ---------------------------- -------------
    PARAMETER                    SAMA5D44
    ---------------------------- -------------
    CPU                          Cortex-A5
    ARM TrustZone                Yes
    NEON Multimedia Architecture Yes
    Pin Count                    361
    Data Cache                   32KiB
    Instruction Cache            32KiB
    L2 Cache                     128KiB
    Max. Operating Frequency     533MHz
    SRAM                         128KiB
    Max I/O Pins                 138
    USB Transceiver              3
    USB Speed                    Hi-Speed
    USB Interface                Host, Device
    SPI                          3
    TWI (I2C)                    4
    UART                         7
    LIN                          4
    SSC                          2
    Ethernet                     2 10/100Mbps
    SD / eMMC                    2
    Graphic LCD                  Yes
    Camera Interface             Yes
    Video Decoder                Yes
    Soft Modem                   Yes
    ADC channels                 5
    Resistive Touch Screen       Yes
    Capacitive Touch Module      Yes
    Crypto Engine                SHA/AES/TDES
    TRNG                         Yes
    External Bus Interface       1
    DRAM Memory                  DDR2/LPDDR,
                                 SDRAM/LPSDR,
                                 32-bit
    NAND Interface               Yes
    FPU                          Yes
    MPU / MMU                    No/Yes
    Timers                       9
    Output Compare channels      9
    Input Capture Channels       9
    PWM Channels                 4
    32kHz RTC                    Yes
    Package                      BGA361
    ---------------------------- -------------

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - Loading Code into SRAM with J-Link
  - Writing to FLASH using SAM-BA
  - Creating and Using DRAMBOOT
  - Creating and Using AT25BOOT
  - Running NuttX from SDRAM
  - SAMA4D44-MB RevC PIO Usage
  - Board Revisions
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
  - SAMA5 ADC Support
  - SAMA5 PWM Support
  - RTC
  - Watchdog Timer
  - TRNG and /dev/random
  - Audio Support
  - TM7000 LCD/Touchscreen
  - Tickless OS
  - SAMA4D4-EK Configuration Options
  - Configurations
  - To-Do List

Development Environment
=======================

  Several possible development environments may be used:

  - Linux or macOS native
  - Cygwin unders Windows
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

      tools/configure.sh sama5d4-ek:<sub-dir>

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

     tools/configure.sh sama5d4-ek:<sub-dir>

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
         (gdb) ... start debugging ...

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
       port, and (2) board = at91sama5d4-ek.
    7. The SAM-BA menu should appear.
    8. Select the FLASH bank that you want to use and the address to write
       to and "Execute"
    9. When you are finished writing to FLASH, remove the USB cable from J6
       and re-connect the serial link on USB CDC / DBGU connector (J23) and
       re-open the terminal emulator program.
    10. Power cycle the board.

Creating and Using DRAMBOOT
===========================

  In order to have more control of debugging code that runs out of DARM,
  I created the sama5d4-ek/dramboot configuration.  That configuration is
  described below under "Configurations."

  Here are some general instructions on how to build an use dramboot:

  Building:
  1. Remove any old configurations (if applicable).

       cd <nuttx>
       make distclean

  2. Install and build the dramboot configuration.  This steps will establish
     the dramboot configuration and setup the PATH variable in order to do
     the build:

       tools/configure.sh sama5d4-ek:dramboot

     Before building, make sure that the PATH environment variable includes
     the correct path  to the directory than holds your toolchain binaries.

     NOTE:  Be aware that the default dramboot also disables the watchdog.
     Since you will not be able to re-enable the watchdog later, you may
     need to set CONFIG_SAMA5_WDT=y in the NuttX configuration file.

     Then make dramboot:

       make

     This will result in an ELF binary called 'nuttx' and also HEX and
     binary versions called 'nuttx.hex' and 'nuttx.bin'.

  3. Rename the binaries.  Since you will need two versions of NuttX:  this
     dramboot version that runs in internal SRAM and another under test in
     NOR FLASH, I rename the resulting binary files so that they can be
     distinguished:

       mv nuttx dramboot
       mv nuttx.hex dramboot.hex
       mv nuttx.bin dramboot.bin

   4. Build the "real" DRAM configuration.  This will create the nuttx.hex
      that you will load using dramboot.  Note that you must select
      CONFIG_SAMA5D4EK_DRAM_BOOT=y.  This controls the origin at which the
      code is linked and positions it correctly for the DRAMBOOT program.

   5. Restart the system holding DIS_BOOT.  You should see the RomBOOT
      prompt on the 115200 8N1 serial console (and nothing) more.  Hit
      the ENTER key with the focus on your terminal window a few time.
      This will enable JTAG.

   6. Then start the J-Link GDB server and GDB.  In GDB, I do the following:

       (gdb) mon heal                 # Halt the CPU
       (gdb) load dramboot            # Load dramboot into internal SRAM
       (gdb) mon go                   # Start dramboot

      You should see this message:

        Send Intel HEX file now

      Load your program by sending the nuttx.hex via the terminal program.
      Then:

       (gdb) mon halt                 # Break in
       (gdb) mon reg pc = 0x20000040  # Set the PC to DRAM entry point
       (gdb) mon go                   # And jump into DRAM

      The dramboot program can also be configured to jump directly into
      DRAM without requiring the final halt and go by setting
      CONFIG_SAMA5D4EK_DRAM_START=y in the NuttX configuration.  However,
      since I have been debugging the early boot sequence, the above
      sequence has been most convenient for me since it allows me to
      step into the program in SDRAM.

   7. An option is to use the SAM-BA tool to write the DRAMBOOT image into
      Serial FLASH.  Then, the system will boot from Serial FLASH by
      copying the DRAMBOOT image in SRAM which will run, download the nuttx.hex
      file, and then start the image loaded into DRAM automatically.  This is
      a very convenient usage!

      NOTES: (1) There is that must be closed to enable use of the AT25
      Serial Flash.  (2) If using SAM-BA, make sure that you load the DRAM
      boot program into the boot area via the pull-down menu.  (3) If
      you don't have SAM-BA, an alternative is to use the AT25BOOT program
      described in the next section.

   STATUS:  I don't have a working SAM-BA at the moment and there are issues
   with my AT25BOOT (see below).  I currently work around these issues by
   putting DRAMBOOT on a microSD card (as boot.bin).  The RomBOOT loader does
   boot that image without issue.

Creating and Using AT25BOOT
===========================

  To work around some SAM-BA availability issues that I had at one time,
  I created the AT25BOOT program. AT25BOOT is a tiny program that runs in
  ISRAM.  AT25BOOT will enable SDRAM and configure the AT25 Serial FLASH.
  It will prompt and then load an Intel HEX program into SDRAM over the
  serial console. If the program is successfully loaded in SDRAM, AT25BOOT
  will copy the program at the beginning of the AT26 Serial FLASH.
  If the jumpering is set correctly, the SAMA5D4 RomBOOT loader will
  then boot the program from the serial FLASH the next time that it
  reset.

  The AT25BOOT configuration is described below under "Configurations."

  Here are some general instructions on how to build an use AT25BOOT:

  Building:
  1. Remove any old configurations (if applicable).

       cd <nuttx>
       make distclean

  2. Install and build the AT25BOOT configuration.  This steps will establish
     the AT25BOOT configuration and setup the PATH variable in order to do
     the build:

       tools/configure.sh sama5d4-ek:at25boot

     Before building, make sure that the PATH environment variable includes
     the correct path  to the directory than holds your toolchain binaries.

     Then make AT25BOOT:

       make

     This will result in an ELF binary called 'nuttx' and also HEX and
     binary versions called 'nuttx.hex' and 'nuttx.bin'.

  3. Rename the binaries.  If you want to save this version of AT25BOOT so
     that it does not get clobbered later, you may want to rename the
     binaries:

       mv nuttx at25boot
       mv nuttx.hex at25boot.hex
       mv nuttx.bin at25boot.bin

   4. Build the "real" DRAMBOOT configuration.  This will create the
      dramboot.hex that you will write to the AT25 FLASH using AT25BOOT. See
      the section above entitled "Creating and Using AT25BOOT" for more
      information.

   5. Restart the system holding DIS_BOOT.  You should see the RomBOOT
      prompt on the 115200 8N1 serial console (and nothing) more.  Hit
      the ENTER key with the focus on your terminal window a few time.
      This will enable JTAG.

   6. Then start the J-Link GDB server and GDB.  In GDB, I do the following:

       (gdb) mon heal                 # Halt the CPU
       (gdb) load at25boot            # Load AT25BOOT into internal SRAM
       (gdb) mon go                   # Start AT25BOOT

      You should see this message:

        Send Intel HEX file now

      Load DRAMBOOT by sending the dramboot.hex via the terminal program.
      At this point you will get messages indicated whether or not the write
      to the AT25 FLASH was successful or not.  When you reset the board,
      it should then boot from the AT25 Serial FLASH and you should again
      get the prompt:

        Send Intel HEX file now

      But now you are being prompted to load the DRAM program under test
      (See the section above entitled "Creating and Using AT25BOOT").

   7. An better option, if available, is to use the SAM-BA tool to write the
      DRAMBOOT image into Serial FLASH.

   NOTES: (1) There is that must be closed to enable use of the AT25
   Serial Flash.  (2) If using SAM-BA, make sure that you load the DRAM
   boot program into the boot area via the pull-down menu.

   STATUS:  While this program works great and appears to correctly write
   the binary image onto the AT25 Serial FLASH, the RomBOOT loader will
   not boot it!  I believe that is because the secure boot loader has some
   undocumented requirements that I am unaware of. (2014-6-28)

Running NuttX from SDRAM
========================

  NuttX may be executed from SDRAM.  But this case means that the NuttX
  binary must reside on some other media (typically NAND FLASH, Serial
  FLASH) or transferred over some interface (perhaps a UARt or even a
  TFTP server).  In these cases, an intermediate bootloader such as U-Boot
  or Barebox must be used to configure the SAMA5D4 clocks and SDRAM and
  then to copy the NuttX binary into SDRAM.

  The SRAMBOOT program is another option (see above). But this section
  will focus on U-Boot.

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
  configure the SAMA5D4 clocking.  Rather, it will use the clock configuration
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

SAMA4D44-MB RevC PIO Usage
==========================

  Rev. B. 0111A
  ------------------------------ ------------------- -------------------------
  SAMA5D4 PIO                    SIGNAL              USAGE
  ------------------------------ ------------------- -------------------------
  PA0/LCDDAT0/TMS                PA0                 LCDDAT0, TMS
  PA1/LCDDAT1                    PA1                 LCDDAT1
  PA2/LCDDAT2/G1_TXCK            PA2                 LCDDAT2, G1_TXCK
  PA3/LCDDAT3/G1_RXCK            PA3                 LCDDAT3
  PA4/LCDDAT4/G1_TXEN            PA4                 LCDDAT4, G1_TXEN
  PA5/LCDDAT5/G1_TXER            PA5                 LCDDAT5
  PA6/LCDDAT6/G1_CRS             PA6                 LCDDAT6
  PA7/LCDDAT7                    PA7                 LCDDAT7
  PA8/LCDDAT8/TCK                PA8                 LCDDAT8, TCK
  PA9/LCDDAT9/G1_COL             PA9                 LCDDAT9
  PA10/LCDDAT10/G1_RXDV          PA10                LCDDAT10, G1_RXDV
  PA11/LCDDAT11/G1_RXER          PA11                LCDDAT11, G1_RXER
  PA12/LCDDAT12/G1_RX0           PA12                LCDDAT12, G1_RX0
  PA13/LCDDAT13/G1_RX1           PA13                LCDDAT13, G1_RX1
  PA14/LCDDAT14/G1_TX0           PA14                LCDDAT14, G1_TX0
  PA15/LCDDAT15/G1_TX1           PA15                LCDDAT15, G1_TX1
  PA16/LCDDAT16/NTRST            PA16                LCDDAT16, NTRST
  PA17/LCDDAT17                  PA17                LCDDAT17
  PA18/LCDDAT18/G1_RX2           PA18                LCDDAT18
  PA19/LCDDAT19/G1_RX3           PA19                LCDDAT19
  PA20/LCDDAT20/G1_TX2           PA20                LCDDAT20
  PA21/LCDDAT21/G1_TX3           PA21                LCDDAT21
  PA22/LCDDAT22/G1_MDC           PA22                LCDDAT22, G1_MDC
  PA23/LCDDAT23/G1_MDIO          PA23                LCDDAT23, G1_MDIO
  PA24/LCDPWM/PCK0               PA24                LCDPWM, EXP
  PA25/LCDDISP/TD0               PA25                LCDDISP, EXP
  PA26/LCDVSYNC/PWMH0/SPI1_NPCS1 PA26                LCDVSYNC
  PA27/LCDHSYNC/PWML0/SPI1_NPCS2 PA27                LCDHSYNC
  PA28/LCDPCK/PWMH1/SPI1_NPCS3   PA28                LCDPCK
  PA29/LCDDEN/PWML1              PA29                LCDDEN
  PA30/TWD0                      PA30                TWD0
  PA31/TWCK0                     PA31                TWCK0
  ------------------------------ ------------------- -------------------------
  PB0/G0_TXCK                    PB0                 G0_TXCK, EXP
  PB1/G0_RXCK/SCK2/ISI_PCK       ISI_PCK_PB1         ISI_PCK
  PB2/G0_TXEN                    PB2                 G0_TXEN,EXP
  PB3/G0_TXER/CTS2/ISI_VSYNC     ISI_VSYNC_PB3       ISI_VSYNC
  PB4/G0_CRS/RXD2/ISI_HSYNC      ISI_HSYNC_PB4       ISI_HSYNC
  PB5/G0_COL/TXD2/PCK2           ISI_PWD_PB5         ISI_PWD
  PB6/G0_RXDV                    PB6                 G0_RXDV, EXP
  PB7/G0_RXER                    PB7                 G0_RXER, EXP
  PB8/G0_RX0                     PB8                 G0_RX0, EXP
  PB9/G0_RX1                     PB9                 G0_RX1, EXP
  PB10/G0_RX2/PCK2/PWML1         PB10                AUDIO_PCK2, EXP
  PB11/G0_RX3/RTS2/PWMH1         ISI_RST_PB11        ISI_RST
  PB12/G0_TX0                    PB12                G0_TX0, EXP
  PB13/G0_TX1                    PB13                G0_TX1, EXP
  PB14/G0_TX2/SPI2_NPCS1/PWMH0   ZIG_SPI2_NPCS1      ZIG_SPI2_NPCS1
  PB15/G0_TX3/SPI2_NPCS2/PWML0   HDMI_RST_PB15       HDMI_RST
  PB16/G0_MDC                    PB16                G0_MDC, EXP
  PB17/G0_MDIO                   PB17                G0_MDIO, EXP
  PB18/SPI1_MISO/D8              LCD_SPI1_SO         LCD_SPI1_SO
  PB19/SPI1_MOSI/D9              LCD_SPI1_SI         LCD_SPI1_SI
  PB20/SPI1_SPCK/D10             LCD_SPI1_CLK        LCD_SPI1_CLK
  PB21/SPI1_NPCS0/D11            EXP_PB21            EXP
  PB22/SPI1_NPCS1/D12            EXP_PB22            EXP
  PB23/SPI1_NPCS2/D13            LCD_SPI1_CS2        LCD_SPI1_NPCS2
  PB24/DRXD/D14/TDI              PB24                TDI, EXP
  PB25/DTXD/D15/TDO              PB25                TDO, EXP
  PB26/PCK0/RK0/PWMH0            PB26                AUDIO_RK0
  PB27/SPI1_NPCS3/TK0/PWML0      PB27                AUDIO, HDMI_TK0, EXP
  PB28/SPI2_NPCS3/TD0/PWMH1      PB28                AUDIO, HDMI_TD0, EXP
  PB29/TWD2/RD0/PWML1            PB29                AUDIO_RD0, ZIG_TWD2
  PB30/TWCK2/RF0                 PB30                AUDIO_RF, ZIG_TWCK2
  PB31/TF0                       PB31                AUDIO, HDMI_TF0, EXP
  ------------------------------ ------------------- -------------------------
  PC0/SPI0_MISO/PWMH2/ISI_D8     PC0                 AT25_SPI0_SO, ISI_D8
  PC1/SPI0_MOSI/PWML2/ISI_D9     PC1                 AT25_SPI0_SI, ISI_D9
  PC2/SPI0_SPCK/PWMH3/ISI_D10    PC2                 AT25_SPI0_SPCK, ISI_D10,
                                                     ZIG_PWMH3_PC2
  PC3/SPI0_NPCS0/PWML3/ISI_D11   PC3                 AT25_SPI0_NCPS0, ISI_D11,
                                                     ZIG_PWML3_PC3 (See JP6)
  PC4/SPI0_NPCS1/MCI0_CK/PCK1    PC4                 MCI0_CK, ISI_MCK, EXP
  PC5/D0/MCI0_CDA                PC5                 MCI0_CDA, NAND_IO0
  PC6/D1/MCI0_DA0                PC6                 MCI0_DA0, NAND_IO1
  PC7/D2/MCI0_DA1                PC7                 MCI0_DA1, NAND_IO2
  PC8/D3/MCI0_DA2                PC8                 MCI0_DA2, NAND_IO3
  PC9/D4/MCI0_DA3                PC9                 MCI0_DA3, NAND_IO4
  PC10/D5/MCI0_DA4               PC10                MCI0_DA4, NAND_IO5
  PC11/D6/MCI0_DA5               PC11                MCI0_DA5, NAND_IO6
  PC12/D7/MCI0_DA6               PC12                MCI0_DA6, NAND_IO7
  PC13/NRD/NANDOE/MCI0_DA7       PC13                MCI0_DA7, NAND_RE
  PC14/NWE/NANDWE                NAND_WEn            NWE, NANDWE
  PC15/NCS3                      NAND_NCS3           NAND_NCS3
  PC16/NANDRDY                   NAND_RDY            NANDRDY
  PC17/A21/NANDALE               NAND_ALE            NAND_ALE
  PC18/A22/NANDCLE               NAND_CLE            NAND_CLE
  PC19/ISI_D0/TK1                PC19                ISI_D0
  PC20/ISI_D1/TF1                PC20                ISI_D1
  PC21/ISI_D2/TD1                PC21                ISI_D2
  PC22/ISI_D3/RF1                PC22                ISI_D3
  PC23/ISI_D4/RD1                PC23                ISI_D4
  PC24/ISI_D5/RK1/PCK1           PC24                ISI_D5
  PC25/ISI_D6/TWD3/URXD1         PC25                AUDIO_TWD3, ISI_D6
  PC26/ISI_D7/TWCK3/UTXD1        PC26                AUDIO_TWCK3, ISI_D7
  PC27/AD0/SPI0_NPCS1/PWML0      AD0_XP              AD0_XP
  PC28/AD1/SPI0_NPCS2/PWML1      AD1_XM              AD1_XM
  PC29/AD2/SPI0_NPCS3/PWMFI0     AD2_YP              AD2_YP
  PC30/AD3/PWMH0                 AD3_YM              AD3_YM
  PC31/AD4/PWMH1                 AD4_LR              AD4_LR, ADC_INPUT
  ------------------------------ ------------------- -------------------------
  PD8/PCK0                       PD8                 EXP_PCK0
  PD9/FIQ                        USB_OVCUR_PD9       USB_OVCUR_PD9
  PD10/CTS0/CDETA                ZIG_CTS0_PD10       ZIG_CTS0
  PD11/RTS0/SPI2_MISO            ZIG_SPI2_MISO_RTS0  ZIG_SPI2_MISO_RTS0
  PD12/RXD0/DCENA                ZIG_RXD0_PD12       ZIG_RXD0
  PD13/TXD0/SPI2_MOSI            ZIG_SPI2_MOSI_TXD0  ZIG_SPI2_MOSI_TXD0
  PD14/CTS1/CDETB                ZIG_CTS1_PD14       ZIG_CTS1
  PD15/RTS1/SPI2_SPCK            ZIG_SPI2_SPCK_RTS1  ZIG_SPI2_SPCK_RTS
  PD16/RXD1/DCENB                ZIG_RXD1_PD16       ZIG_RXD1_PD16
  PD17/TXD1/SPI2_NPCS0           ZIG_SPI2_NPCS0_TXD1 ZIG_SPI2_NPCS0_TXD
  PD18/SENSE0                    SENSE0_PD18         SENSE0
  PD19/SENSE1                    SENSE1_PD19         SENSE1
  PD20/SENSE2                    SENSE2_PD20         SENSE2
  PD21/SENSE3                    SENSE3_PD21         SENSE3
  PD22/SENSE4                    SENSE4_PD22         SENSE4
  PD23/SENSE5                    N/C                 N/C
  PD24/SENSE6                    N/C                 N/C
  PD25/SENSE7                    N/C                 N/C
  PD26/SENSE8                    N/C                 N/C
  PD27/SENSE9                    N/C                 N/C
  PD28/SCK0                      N/C                 PD28
  PD29/SCK1                      SENSE_DISCH_PD29    SENSE_DISCH
  PD30                           EXP_PD30            EXP
  PD31/SPI0_NPCS2/PCK1           EXP_PD31            EXP
  ------------------------------ ------------------- -------------------------
  PE0/A0/NBS0/MCI0_CDB/CTS4      PMIC_IRQ_PE0        PMIC_IRQ
  PE1/A1/MCI0_DB0                G0_IRQ_PE1          G0_IRQ
  PE2/A2/MCI0_DB1                G1_IRQ_PE2          G1_IRQ
  PE3/A3/MCI0_DB2                HDMI_IRQ_PE3        HDMI_IRQ
  PE4/A4/MCI0_DB3                AUDIO_IRQ_PE4       AUDIO_IRQ
  PE5/A5/CTS3                    MCI0_CD_PE5         MCI0_CD
  PE6/A6/TIOA3                   MCI1_CD_PE6         MCI1_CD
  PE7/A7/TIOB3/PWMFI1            EXP_PE7             EXP
  PE8/A8/TCLK3/PWML3             LED_USER_PE8        LED_USER (D10)
  PE9/A9/TIOA2                   LED_POWER_PE9       LED_POWER (D9, Red)
  PE10/A10/TIOB2                 USBA_EN5V_PE10      EN5V_USBA
  PE11/A11/TCLK2                 USBB_EN5V_PE11      EN5V_USBB
  PE12/A12/TIOA1/PWMH2           USBC_EN5V_PE12      EN5V_USBC
  PE13/A13/TIOB1/PWML2           PB_USER1_PE13       PB_USER1
  PE14/A14/TCLK1/PWMH3           MCI1_CD_PE14        MCI1_CD              ???
  PE15/A15/SCK3/TIOA0            MCI1_PWR_PE15       MCI1_PWR
  PE16/A16/RXD3/TIOB0            DBGU_RXD3_PE16      DBGU_RXD3 (See JP19)
  PE17/A17/TXD3/TCLK0            DBGU_TXD3_PE17      DBGU_TXD3 (See JP20)
  PE18/A18/TIOA5/MCI1_CK         PE18                MCI1_CK, EXP
  PE19/A19/TIOB5/MCI1_CDA        PE19                MCI1_CDA, EXP
  PE20/A20/TCLK5/MCI1_DA0        PE20                MCI1_DA0, EXP
  PE21/A23/TIOA4/MCI1_DA1        PE21                MCI1_DA1, EXP
  PE22/A24/TIOB4/MCI1_DA2        PE22                MCI1_DA2, EXP
  PE23/A25/TCLK4/MCI1_DA3        PE23                MCI1_DA3, EXP
  PE24/NCS0/RTS3                 LCD_PE24            LCD_PE24
  PE25/NCS1/SCK4/IRQ             LCD_PE25            LCD_PE25
  PE26/NCS2/RXD4/A18             RXD4_PE26           RXD4
  PE27/NWR1/NBS1/TXD4            TXD4_PE27           TXD4
  PE28/NWAIT/RTS4/A19            1Wire_PE28          1-WIRE ROM, LCD, D8 (green)
  PE29/DIBP/URXD0/TWD1           SMD_DIBP_PE29       DIBP
  PE30/DIBN/UTXD0/TWCK1          SMD_DIBN_PE30       DIBP
  PE31/ADTRG                     USBA_VBUS_PE31      USBA_VBUS_PE31
  ------------------------------ ------------------- -------------------------

Board Revisions
===============

  PIO Usage
  ---------
  Most of this work was developed on a SAMA5D4-MB Rev C. board.  Here is a
  pin-for-pin comparison between the Rev C and Rev E boards.  This is a
  comparison of signal naming only; some differences are simply due to
  differences in naming and any functional differences with no signal name
  change would no be noted.

    ---------- --------------------- ---------------------
    PINS       SAMA5D44-MB Rev C.    SAMA5D44-MB Rev E.
    ---------- --------------------- ---------------------
    PA0-PA31:  Identical
    ---------- --------------------- ---------------------
    PB0-PB13:  Identical
    PB14       ZIG_SPI2_NPCS1        XPRO_SPI2_NPCS1
    PB15-PB31: Identical
    ---------- --------------------- ---------------------
    PC0-PC1:   Identical
    PC2        A-SPCK/ISI_D10/PWMH3  SPCK/ISI_D10/PWMH3
    PC3        A-NCPS0/ISI_D11/PWML3 NCPS0/ISI_D11/PWML3
    PC4-PC31:  Identical
    ---------- --------------------- ---------------------
    PD0-PD9:   Identical
    PD10       ZIG_CTS0              XPRO_CTS0
    PD11       ZIG_SPI2_MISO_RTS0    XPRO_MISO_RTS0
    PD12       ZIG_RXD0              XPRO_RXD0
    PD13       ZIG_SPI2_MOSI_TXD0    XPRO_MOSI_TXD0
    PD14       ZIG_CTS1              XPRO_CTS1
    PD15       ZIG_SPI2_SPCK_RTS1    XPRO_SPCK_RTS1
    PD16       ZIG_RXD1_PD16         XPRO_RXD1_PD16
    PD17       ZIG_SPI2_NPCS0_TXD    XPRO_NPCS0_TXD1
    PD18       SENSE0                NC
    PD19       SENSE1                NC
    PD20       SENSE2                NC
    PD21       SENSE3                NC
    PD22       NSENSE4C              NC
    PD23-PD27: Identical
    PD28       PD28                  SCK0
    PD29       SENSE_DISCH           SCK1
    PD30-PD31: Identical
    ---------- --------------------- ---------------------
    PE0-PE13:  Identical
    PE14       MCI1_CD               EXP
    PE15-PE30: Identical
    PE31       USBA_VBUS_PE31        USBA_VBUS/ADTRG
    ---------- --------------------- ---------------------

  Jumpers
  -------
    ---------- --------------------- ---------------------
    Jumpers    SAMA5D44-MB Rev C.    SAMA5D44-MB Rev E.
    ---------- --------------------- ---------------------
    JP2-J3     Identical function
    JP4        Force power on        Not present on Rev E.
               function selection
    JP5-J22    Identical function
    JP23       AUDIO_TWD0_PA30       Not present on Rev E.
    JP24       Not present on Rev. C For CTS,RTS usage

Buttons and LEDs
================

  Buttons
  -------
  A single button, PB_USER1 (PB2), is available on the SAMA5D4-EK:

  ------------------------------ ------------------- -------------------------
  SAMA5D4 PIO                    SIGNAL              USAGE
  ------------------------------ ------------------- -------------------------
  PE13/A13/TIOB1/PWML2           PB_USER1_PE13       PB_USER1
  ------------------------------ ------------------- -------------------------

  Closing JP2 will bring PE13 to ground so 1) PE13 should have a weak pull-up,
  and 2) when PB2 is pressed, a low value will be senses.

  Support for pollable buttons is enabled with:

    CONFIG_ARCH_BUTTONS=y

  For interrupt driven buttons, add:

    CONFIG_ARCH_IRQBUTTONS=y

  Program interfaces for button access are described in nuttx/include/nuttx/arch.h

  There is an example that can be enabled to test button interrupts.  That
  example is enabled like:

    CONFIG_EXAMPLES_BUTTONS=y
    CONFIG_EXAMPLES_BUTTONS_MAX=0
    CONFIG_EXAMPLES_BUTTONS_MIN=0
    CONFIG_EXAMPLES_BUTTONS_NAME0="PB_USER"
    CONFIG_EXAMPLES_IRQBUTTONS_MAX=0
    CONFIG_EXAMPLES_IRQBUTTONS_MIN=0

  LEDs
  ----
  There are 3 LEDs on the SAMA5D4-EK:

  ------------------------------ ------------------- -------------------------
  SAMA5D4 PIO                    SIGNAL              USAGE
  ------------------------------ ------------------- -------------------------
  PE28/NWAIT/RTS4/A19            1Wire_PE28          1-WIRE ROM, LCD, D8 (green)
  PE8/A8/TCLK3/PWML3             LED_USER_PE8        LED_USER (D10)
  PE9/A9/TIOA2                   LED_POWER_PE9       LED_POWER (D9, Red)
  ------------------------------ ------------------- -------------------------

  - D8: D8 is shared with other functions and cannot be used if the 1-Wire ROM
    is used.  I am not sure of the LCD function, but the LED may not be available
    if the LCD is used either.  We will avoid using D8 just for simplicity.
  - D10:  Nothing special here.  A low output illuminates.
  - D9: The Power ON LED.  Connects to the via an IRLML2502 MOSFET.  This LED will
    be on when power is applied but otherwise a low output value will turn it
    off.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
  events as follows:

    SYMBOL                Meaning                     LED state
                                                  USER D10 POWER D9
    -------------------  -----------------------  -------- --------
    LED_STARTED          NuttX has been started     OFF      ON
    LED_HEAPALLOCATE     Heap has been allocated    OFF      ON
    LED_IRQSENABLED      Interrupts enabled         OFF      ON
    LED_STACKCREATED     Idle stack created         ON       ON
    LED_INIRQ            In an interrupt              No change
    LED_SIGNAL           In a signal handler          No change
    LED_ASSERTION        An assertion failed          No change
    LED_PANIC            The system has crashed     OFF      Blinking
    LED_IDLE             MCU is is sleep mode         Not used

  Thus if the D0 and D9 are statically on, NuttX has successfully booted and
  is, apparently, running normally.  If the red D9 LED is flashing at
  approximately 2Hz, then a fatal error has been detected and the system
  has halted.

Serial Console
==============

  Two UART ports are available:

  Virtual COM / DBGU Port (J24).  Either may be driven by USART3, depending
  upon the setting of JP19 and JP20:

  ------------------------------ ------------------- -------------------------
  SAMA5D4 PIO                    SIGNAL              USAGE
  ------------------------------ ------------------- -------------------------
  PE16/A16/RXD3/TIOB0            DBGU_RXD3_PE16      DBGU_RXD3 (See JP19)
  PE17/A17/TXD3/TCLK0            DBGU_TXD3_PE17      DBGU_TXD3 (See JP20)
  ------------------------------ ------------------- -------------------------

  In one jumper position UART3 connects to the SAM3U which will, in turn,
  provide the serial output over a USB virtual COM port.  In other other
  jumper position, UART3 will connect the RS-232 port labelled DBGU (J24).

  I personally prefer the RS-232 port because my terminal software does not
  lose the USB Virtual COM every time I reset or power-cycle the board.

  USART4 TTL-Level
  ------------------------------ ------------------- -------------------------
  SAMA5D4 PIO                    SIGNAL              USAGE
  ------------------------------ ------------------- -------------------------
  PE26/NCS2/RXD4/A18             RXD4_PE26           RXD4
  PE27/NWR1/NBS1/TXD4            TXD4_PE27           TXD4
  ------------------------------ ------------------- -------------------------

  A TTL-to-RS232 converter is required to use this USART for a serial console.

  - RXD4/PE26 is available at Expansion Interface, J19C pin 59
  - TXD4/PE27 is available at Expansion Interface, J19C pin 60
  - VCC_3V3 is also available at Expansion Interface, J19B pins 21 and 22
  - GND is available J19A pin 11, J19B pin 31, and J19C pin 51

  By default the RS-232 DBGU port on USART3 is used as the NuttX serial
  console in all configurations (unless otherwise noted).  USART4, however,
  is the also available.

Networking
==========

  Networking support via the can be added to NSH by selecting the following
  configuration options.  The SAMA5D44 supports two different 10/100Base-T
  Ethernet MAC peripherals.

    NOTE:  See the "kludge" for EMAC that is documented in the To-Do
    list at the end of this README file.

  ------------------------------ ------------------- -------------------------
  SAMA5D4 PIO                    SAMA5D4-MB          KSZ8081RNB
  ------------------------------ ------------------- -------------------------
  PB0/G0_TXCK                    G0_TXCK_PB0         RXF_CLK/B-CAST_OFF
  PB1/G0_RXCK/SCK2/ISI_PCK       (RMII, not used)    (RMII, not used)
  PB2/G0_TXEN                    G0_TXEN_PB2         TXEN
  PB3/G0_TXER/CTS2/ISI_VSYNC     (RMII, not used)    (RMII, not used)
  PB4/G0_CRS/RXD2/ISI_HSYNC      (RMII, not used)    (RMII, not used)
  PB5/G0_COL/TXD2/PCK2           (RMII, not used)    (RMII, not used)
  PB6/G0_RXDV                    G0_RXDV_PB6         CRS_DV/CONFIG2
  PB7/G0_RXER                    G0_RXER_PB7         RXER/ISO
  PB8/G0_RX0                     G0_RX0_PB8          RXD0/DUPLEX
  PB9/G0_RX1                     G0_RX1_PB9          RXD1/PHYAD2
  PB10/G0_RX2/PCK2/PWML1         (RMII, not used)    (RMII, not used)
  PB11/G0_RX3/RTS2/PWMH1         (RMII, not used)    (RMII, not used)
  PB12/G0_TX0                    G0_TX0_PB12         TXD0
  PB13/G0_TX1                    G0_TX1_PB13         TXD1
  PB14/G0_TX2/SPI2_NPCS1/PWMH0   (RMII, not used)    (RMII, not used)
  PB15/G0_TX3/SPI2_NPCS2/PWML0   (RMII, not used)    (RMII, not used)
  PB16/G0_MDC                    G0_MDC_PB16         MDC
  PB17/G0_MDIO                   G0_MDIO_PB17        MDIO
  PE1/A1/MCI0_DB0                G0_IRQ_PE1          nINTRP/NAND_TREE
  ------------------------------ ------------------- -------------------------
  PA2/LCDDAT2/G1_TXCK            G1_TXCK_PA2         RXF_CLK/B-CAST_OFF
  PA3/LCDDAT3/G1_RXCK            (RMII, not used)    (RMII, not used)
  PA4/LCDDAT4/G1_TXEN            G1_TXEN_PA4         TXEN
  PA5/LCDDAT5/G1_TXER            (RMII, not used)    (RMII, not used)
  PA6/LCDDAT6/G1_CRS             (RMII, not used)    (RMII, not used)
  PA9/LCDDAT9/G1_COL             (RMII, not used)    (RMII, not used)
  PA10/LCDDAT10/G1_RXDV          G1_RXDV_PA10        CRS_DV/CONFIG2
  PA11/LCDDAT11/G1_RXER          G1_RXER_PA11        RXER/ISO
  PA12/LCDDAT12/G1_RX0           G1_RX0_PA12         RXD0/DUPLEX
  PA13/LCDDAT13/G1_RX1           G1_RX1_PA13         RXD1/PHYAD2
  PA18/LCDDAT18/G1_RX2           (RMII, not used)    (RMII, not used)
  PA19/LCDDAT19/G1_RX3           (RMII, not used)    (RMII, not used)
  PA14/LCDDAT14/G1_TX0           G1_TX0_PA14         TXD0
  PA15/LCDDAT15/G1_TX1           G1_TX1_PA15         TXD1
  PA20/LCDDAT20/G1_TX2           (RMII, not used)    (RMII, not used)
  PA21/LCDDAT21/G1_TX3           (RMII, not used)    (RMII, not used)
  PA22/LCDDAT22/G1_MDC           G1_MDC_PA22         MDC
  PA23/LCDDAT23/G1_MDIO          G1_MDIO_PA23        MDIO
  PE2/A2/MCI0_DB1                G1_IRQ_PE2          nINTRP/NAND_TREE
  ------------------------------ ------------------- -------------------------

  EMAC2 connects (directly) to a KSZ8081RNB PHY (U10) and is available at
  the ETH0 connector.

  EMAC1 connects (indirectly) to another KSZ8081RNB PHY (U7) and is available
  at the ETH1 connector.

  The ETH1 signals go through line drivers that are enabled via the board
  LCD_ETH1_CONFIG signal.  Jumper JP2 selects either the EMAC1 or the LCD by
  controlling the LCD_ETH1_CONFIG signal on the board.

    - JP2 open, LCD_ETH1_CONFIG pulled high:

      LCD_ETH1_CONFIG=1: LCD 5v enable(LCD_DETECT#=0); ETH1 disable

    - JP2 closed, LCD_ETH1_CONFIG grounded:

      LCD_ETH1_CONFIG=0: LCD 5v disable; ETH1 enable

  Selecting the EMAC0 peripheral
  -----------------------------

  System Type -> SAMA5 Peripheral Support
    CONFIG_SAMA5_EMAC0=y                 : Enable the EMAC peripheral

  System Type -> EMAC device driver options
    CONFIG_SAMA5_EMAC0_NRXBUFFERS=16     : Set aside some RS and TX buffers
    CONFIG_SAMA5_EMAC0_NTXBUFFERS=8
    CONFIG_SAMA5_EMAC0_PHYADDR=1         : KSZ8081 PHY is at address 1
    CONFIG_SAMA5_EMAC0_AUTONEG=y         : Use autonegotiation
    CONFIG_SAMA5_EMAC0_RMII=y            : The RMII interfaces is used on the board
    CONFIG_SAMA5_EMAC0_PHYSR=30          : Address of PHY status register on KSZ8081
    CONFIG_SAMA5_EMAC0_PHYSR_ALTCONFIG=y : Needed for KSZ8081
    CONFIG_SAMA5_EMAC0_PHYSR_ALTMODE=0x7 : "    " " " "     "
    CONFIG_SAMA5_EMAC0_PHYSR_10HD=0x1    : "    " " " "     "
    CONFIG_SAMA5_EMAC0_PHYSR_100HD=0x2   : "    " " " "     "
    CONFIG_SAMA5_EMAC0_PHYSR_10FD=0x5    : "    " " " "     "
    CONFIG_SAMA5_EMAC0_PHYSR_100FD=0x6   : "    " " " "     "

  PHY selection.  Later in the configuration steps, you will need to select
  the KSZ8081 PHY for EMAC (See below)

  Selecting the EMAC1 peripheral
  -----------------------------

  System Type -> SAMA5 Peripheral Support
    CONFIG_SAMA5_EMAC1=y                 : Enable the EMAC peripheral

  System Type -> EMAC device driver options
    CONFIG_SAMA5_EMAC1_NRXBUFFERS=16     : Set aside some RS and TX buffers
    CONFIG_SAMA5_EMAC1_NTXBUFFERS=8
    CONFIG_SAMA5_EMAC1_PHYADDR=1         : KSZ8081 PHY is at address 1
    CONFIG_SAMA5_EMAC1_AUTONEG=y         : Use autonegotiation
    CONFIG_SAMA5_EMAC1_RMII=y            : The RMII interfaces is used on the board
    CONFIG_SAMA5_EMAC1_PHYSR=30          : Address of PHY status register on KSZ8081
    CONFIG_SAMA5_EMAC1_PHYSR_ALTCONFIG=y : Needed for KSZ8081
    CONFIG_SAMA5_EMAC1_PHYSR_ALTMODE=0x7 : "    " " " "     "
    CONFIG_SAMA5_EMAC1_PHYSR_10HD=0x1    : "    " " " "     "
    CONFIG_SAMA5_EMAC1_PHYSR_100HD=0x2   : "    " " " "     "
    CONFIG_SAMA5_EMAC1_PHYSR_10FD=0x5    : "    " " " "     "
    CONFIG_SAMA5_EMAC1_PHYSR_100FD=0x6   : "    " " " "     "

  PHY selection.  Later in the configuration steps, you will need to select
  the KSZ8081 PHY for EMAC (See below)

  If both EMAC0 and EMAC1 are selected, you will also need:

    CONFIG_SAMA5_EMAC0_ISETH0=y          : EMAC0 is "eth0"; EMAC1 is "eth1"

  PHY selection.  Later in the configuration steps, you will need to select
  the  KSZ9081 PHY for GMAC (See below)

  Common configuration settings
  -----------------------------

  Networking Support
    CONFIG_NET=y                         : Enable Neworking
    CONFIG_NET_SOCKOPTS=y                : Enable socket operations
    CONFIG_NET_ETH_PKTSIZE=562           : Maximum packet size 1518 is more standard
    CONFIG_NET_ARP=y                     : ARP support should be enabled
    CONFIG_NET_ARP_IPIN=y                : IP address harvesting (optional)
    CONFIG_NET_TCP=y                     : Enable TCP/IP networking
    CONFIG_NET_TCPBACKLOG=y              : Support TCP/IP backlog
    CONFIG_NET_TCP_WRITE_BUFFERS=y       : Enable TCP write buffering
    CONFIG_NET_UDP=y                     : Enable UDP networking
    CONFIG_NET_BROADCAST=y               : Support UDP broadcast packets
    CONFIG_NET_ICMP=y                    : Enable ICMP networking
    CONFIG_NET_ICMP_SOCKET=y             : Needed for NSH ping command
                                         : Defaults should be okay for other options
  Device drivers -> Network Device/PHY Support
    CONFIG_NETDEVICES=y                  : Enabled PHY selection
    CONFIG_ETH0_PHY_KSZ8081=y            : Select the KSZ8081 PHY used with EMAC0 and 1

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

  By default, the IP address of the SAMA4D4-EK will be 10.0.0.2 and
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

  On the host side, you should also be able to ping the SAMA4D4-EK:

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
      nuttx/boards/arm/sama5/sama5d4-ek/src/sam_ethernet.c.

    - One other thing: UDP support is required (CONFIG_NET_UDP).

  Given those prerequisites, the network monitor can be selected with these
  additional settings.

    Networking Support -> Networking Device Support
      CONFIG_NETDEV_PHY_IOCTL=y             : Enable PHY ioctl support

    Application Configuration -> NSH Library -> Networking Configuration
      CONFIG_NSH_NETINIT_THREAD             : Enable the network initialization thread
      CONFIG_NSH_NETINIT_MONITOR=y          : Enable the network monitor
      CONFIG_NSH_NETINIT_RETRYMSEC=2000     : Configure the network monitor as you like
      CONFIG_NSH_NETINIT_SIGNO=18

AT25 Serial FLASH
=================

  Connections
  -----------

  The SAMA4D4-EK board supports an options Serial DataFlash connected
  at MN8.  The SPI connection is as follows:

  ------------------------------ ------------------- -------------------------
  SAMA5D4 PIO                    SIGNAL              USAGE
  ------------------------------ ------------------- -------------------------
  PC0/SPI0_MISO/PWMH2/ISI_D8     PC0                 AT25_SPI0_SO, ISI_D8
  PC1/SPI0_MOSI/PWML2/ISI_D9     PC1                 AT25_SPI0_SI, ISI_D9
  PC2/SPI0_SPCK/PWMH3/ISI_D10    PC2                 AT25_SPI0_SPCK, ISI_D10,
                                                     ZIG_PWMH3_PC2
  PC3/SPI0_NPCS0/PWML3/ISI_D11   PC3                 AT25_SPI0_NCPS0, ISI_D11,
                                                     ZIG_PWML3_PC3 (See JP6)
  ------------------------------ ------------------- -------------------------

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
      CONFIG_SAMA5D4EK_AT25_BLOCKMOUNT=y    : Mounts AT25 for NSH
      CONFIG_SAMA5D4EK_AT25_FTL=y           : Create block driver for FAT

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

  The SAMA4D4-EK provides a two SD memory card slots:  (1) a full size SD
  card slot (J10), and (2) a microSD memory card slot (J11).

  HSMCI0
  ------
  The full size SD card slot connects via HSMCI0.  The card detect discrete
  is available on PE5 (pulled high).  The write protect discrete is tied to
  ground and is not available to software.  The slot supports 8-bit wide
  transfer mode, but the NuttX driver currently uses only the 4-bit wide
  transfer mode

  ------------------------------ ------------------- -------------------------
  SAMA5D4 PIO                    SIGNAL              USAGE
  ------------------------------ ------------------- -------------------------
  PC4/SPI0_NPCS1/MCI0_CK/PCK1    PC4                 MCI0_CK, ISI_MCK, EXP
  PC5/D0/MCI0_CDA                PC5                 MCI0_CDA, NAND_IO0
  PC6/D1/MCI0_DA0                PC6                 MCI0_DA0, NAND_IO1
  PC7/D2/MCI0_DA1                PC7                 MCI0_DA1, NAND_IO2
  PC8/D3/MCI0_DA2                PC8                 MCI0_DA2, NAND_IO3
  PC9/D4/MCI0_DA3                PC9                 MCI0_DA3, NAND_IO4
  PC10/D5/MCI0_DA4               PC10                MCI0_DA4, NAND_IO5
  PC11/D6/MCI0_DA5               PC11                MCI0_DA5, NAND_IO6
  PC12/D7/MCI0_DA6               PC12                MCI0_DA6, NAND_IO7
  PC13/NRD/NANDOE/MCI0_DA7       PC13                MCI0_DA7, NAND_RE
  PE5/A5/CTS3                    MCI0_CD_PE5         MCI0_CD
  ------------------------------ ------------------- -------------------------

  HSMCI1
  ------
  The microSD connects vi HSMCI1.  The card detect discrete is available on
  PE6 (pulled high).  NOTE that PE15 must be controlled to provide power
  to the HSMCI1 slot (the HSMCI0 slot is always powered).

  ------------------------------ ------------------- -------------------------
  SAMA5D4 PIO                    SIGNAL              USAGE
  ------------------------------ ------------------- -------------------------
  PE14/A14/TCLK1/PWMH3           MCI1_CD_PE14        MCI1_CD        Rev C. ???
  PE15/A15/SCK3/TIOA0            MCI1_PWR_PE15       MCI1_PWR
  PE18/A18/TIOA5/MCI1_CK         PE18                MCI1_CK, EXP
  PE19/A19/TIOB5/MCI1_CDA        PE19                MCI1_CDA, EXP
  PE20/A20/TCLK5/MCI1_DA0        PE20                MCI1_DA0, EXP
  PE21/A23/TIOA4/MCI1_DA1        PE21                MCI1_DA1, EXP
  PE22/A24/TIOB4/MCI1_DA2        PE22                MCI1_DA2, EXP
  PE23/A25/TCLK4/MCI1_DA3        PE23                MCI1_DA3, EXP
  PE6/A6/TIOA3                   MCI1_CD_PE6         MCI1_CD
  ------------------------------ ------------------- -------------------------

  Configuration Settings
  ----------------------

  Enabling HSMCI support. The SAMA4D4-EK provides a two SD memory card
  slots:  (1) a full size SD card slot (J10), and (2) a microSD memory card
  slot (J11).  The full size SD card slot connects via HSMCI0; the microSD
  connects via HSMCI1.  Support for both SD slots can be enabled with the
  following settings:

    System Type->ATSAMA5 Peripheral Support
      CONFIG_SAMA5_HSMCI0=y                 : To enable HSMCI0 support
      CONFIG_SAMA5_HSMCI1=y                 : To enable HSMCI1 support
      CONFIG_SAMA5_XDMAC0=y                 : XDMAC0 is needed by HSMCI0/1
                                            : (HSMCI0 seemds to be secure by default)
    System Type
      CONFIG_SAMA5_PIO_IRQ=y                : PIO interrupts needed
      CONFIG_SAMA5_PIOE_IRQ=y               : Card detect pins are on PE5 and PE6

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
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization, OR
      CONFIG_BOARD_LATE_INITIALIZE=y

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
       flushes any cached data to an SD card and makes the SD card unavailable
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

  Here is a sample configuration for the auto-mounter:

    File System Configuration
      CONFIG_FS_AUTOMOUNTER=y

    Board-Specific Options
      CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT=y
      CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_FSTYPE="vfat"
      CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_BLKDEV="/dev/mmcsd0"
      CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_MOUNTPOINT="/mnt/sdcard"
      CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_DDELAY=1000
      CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_UDELAY=2000

  WARNING:  SD cards should never be removed without first unmounting
  them.  This is to avoid data and possible corruption of the file
  system.  Certainly this is the case if you are writing to the SD card
  at the time of the removal.  If you use the SD card for read-only access,
  however, then I cannot think of any reason why removing the card without
  mounting would be harmful.

USB Ports
=========

  The SAMA4D4-EK features three USB communication ports:

    * Port A Host High Speed (EHCI) and Full Speed (OHCI) multiplexed with
      USB Device High Speed Micro AB connector, J1

    * Port B Host High Speed (EHCI) and Full Speed (OHCI) standard type A
      connector, J5 upper port

    * Port C Host Full Speed (OHCI)  and Full Speed (OHCI) standard type A
      connector, J5 lower port

  The three  USB host ports are equipped with 500-mA high-side power
  switch for self-powered and bus-powered applications.

  The USB device port A (J5) features a VBUS insert detection function.

  Port A
  ------

    PIO  Signal Name    Function
    ---- -------------- -------------------------------------------------------
    PE10 USBA_EN5V_PE10 VBus power enable (via MN2 power switch) to VBus pin of
                        the OTG connector (host)
    PE31 USBA_VBUS_PE31 VBus sensing from the VBus pin of the OTG connector (device)

  Port B
  ------

    PIO  Signal Name    Function
    ---- -------------- -------------------------------------------------------
    PE11 USBB_EN5V_PE11 VBus power enable (via MN4 power switch).  To the A1
                        pin of J5 Dual USB A connector

  Port C
  ------

    PIO  Signal Name    Function
    ---- -------------- -------------------------------------------------------
    PE12 USB_OVCUR_PD9  VBus power enable (via MN4 power switch).  To the B1
                        pin of J5 Dual USB A connector

  Both Ports B and C
  ------------------

    PIO  Signal Name   Function
    ---- ------------- -------------------------------------------------------
    PD9  USB_OVCUR_PD9 Combined over-current indication from port A and B

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
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

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
      CONFIG_SAMA5_UHPHS_RHPORT1=n         : (Reserved for use by USB device)
      CONFIG_SAMA5_UHPHS_RHPORT2=y         : Enable port B
      CONFIG_SAMA5_UHPHS_RHPORT3=y         : Enable port C

    Device Drivers
      CONFIG_USBHOST=y                     : Enable USB host support
      CONFIG_USBHOST_ISOC_DISABLE=y        : Isochronous endpoints not needed

    Device Drivers -> USB Host Driver Support
      CONFIG_USBHOST_ISOC_DISABLE=y        : Isochronous endpoints not used
      CONFIG_USBHOST_MSC=y                 : Enable the mass storage class driver
      CONFIG_USBHOST_HIDKBD=y              : Enable the HID keyboard class driver

    RTOS Features -> Work Queue Support
      CONFIG_SCHED_WORKQUEUE=y             : High priority worker thread support is required
      CONFIG_SCHED_HPWORK=y                :

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

  USB Hub Support
  ----------------

  USB hub support can be included by adding the following changes to the configuration (in addition to those listed above):

    Drivers -> USB Host Driver Support
      CONFIG_USBHOST_HUB=y                 : Enable the hub class
      CONFIG_USBHOST_ASYNCH=y              : Asynchronous I/O supported needed for hubs

    System Type -> USB High Speed Host driver options
      CONFIG_SAMA5_OHCI_NEDS=12            : You will probably want more OHCI pipes
      CONFIG_SAMA5_OHCI_NTDS=18            : You will probably want more OHCI pipes
      CONFIG_SAMA5_OHCI_TDBUFFERS=12
      CONFIG_SAMA5_OHCI_TDBUFSIZE=128

      CONFIG_SAMA5_EHCI_NQHS=12            : You will probably want more OHCI pipes
      CONFIG_SAMA5_EHCI_NQTDS=16           : You will probably want more OHCI pipes
      CONFIG_SAMA5_EHCI_BUFSIZE=128

    Board Selection ->
      CONFIG_SAMA5D4EK_USBHOST_STACKSIZE=2048 (bigger than it needs to be)

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
        Verified that normal, non-hub OHCI still works.

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
      CONFIG_SAMA5D4EK_MT47H128M16RT=y          : This is the type of DDR2

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
      CONFIG_SAMA5D4EK_MT47H128M16RT=y          : This is the type of DDR2

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

    Hardware ECC calculation using the SAMA5D4's PMECC can be enable as
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
      CONFIG_SAMA5D4EK_NAND_BLOCKMOUNT=y : Enable FS support on NAND
      CONFIG_SAMA5D4EK_NAND_NXFFS=y      : Use the NXFFS file system

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
      CONFIG_SAMA5D4EK_NAND_BLOCKOMOUNT=y : Enable FS support on NAND
      CONFIG_SAMA5D4EK_NAND_FTL=y         : Use an flash translation layer

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

    With the options CONFIG_SAMA5D4EK_NAND_BLOCKMOUNT=y and
    CONFIG_SAMA5D4EK_NAND_NXFFS=y, the NAND FLASH will be mounted in the NSH
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
     re-verified on 2015-09-29.

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
      SAMA5_TWI0_FREQUENCY=100000           : Select a TWI0 frequency (default)
      SAMA5_TWI1_FREQUENCY=100000           : Select a TWI1 frequency (default)
      SAMA5_TWI2_FREQUENCY=100000           : Select a TWI2 frequency (default)

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

    As an example, the I2C dev comman can be used to list all devices
    responding on TWI0 (the default) like this:

      nsh> i2c dev 0x03 0x77
           0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
      00:          -- -- -- -- -- -- -- -- -- -- -- -- --
      10: -- -- -- -- -- -- -- -- -- -- -- 1b -- -- -- --
      20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      40: -- -- -- -- -- -- -- -- -- -- -- -- 4c -- -- --
      50: 50 -- -- -- -- -- -- -- -- -- -- 5b -- -- -- --
      60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      70: -- -- -- -- -- -- -- --
      nsh>

     Addresses 0x1b, 0x4c, and 0x50 are devices on the TM7000 module.
     0x5b is the address of the on-board PMIC chip.

SAMA5 ADC Support
=================

  Basic driver configuration
  --------------------------
  ADC support can be added to the NSH configuration.  However, there are no
  ADC input pins available to the user for ADC testing (the touchscreen ADC
  inputs are intended for other functionality).  Because of this, there is
  not much motivation to enable ADC support on the SAMA4D4-EK.  This
  paragraph is included here, however, for people using a custom SAMA5D4x
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
  there is not much motivation to enable PWM support on the SAMA4D4-EK.  This
  paragraph is included here, however, for people using a custom SAMA5D4x
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

    See boards/arm/sama5/sama5d4-ek/include/board.h for all of the default PWM
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

  NOTE:  If you want the RTC to preserve time over power cycles, you will
  need to install a battery in the battery holder (J12) and close the jumper,
  JP13.

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
      CONFIG_SCHED_WORKQUEUE=y             : Alarm needs work queue support

Watchdog Timer
==============

  NSH can be configured to exercise the watchdog timer test
  (apps/examples/watchdog).  This can be selected with the following
  settings in the NuttX configuration file:

    System Type:
      CONFIG_SAMA5_WDT=y                  : Enable the WDT peripheral
                                          : Defaults values for others settings
                                            should be OK

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

  NOTE:  If you are using the dramboot program to run from DRAM as I did,
  beware that the default version also disables the watchdog.  You will
  need a special version of dramboot with CONFIG_SAMA5_WDT=y.

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

Audio Support
==============

  WM8904 CODEC
  ------------
  The SAMA4D4-EK has two devices on-board that can be used for verification
  of I2S functionality:  HDMI and a WM8904 audio CODEC.  As of this writing,
  the I2S driver is present, but there are not drivers for either the HDMI
  or the WM8904.

  WM8904 Audio CODEC Interface:
  ---- ------------------ ---------------- ------------- ---------------------------------------
  PIO  USAGE              BOARD SIGNAL     WM8904 PIN    NOTE
  ---- ------------------ ---------------- ------------- ---------------------------------------
  PA30 TWD0               AUDIO_TWD0_PA30  3  SDA        Pulled up, See JP23 note below
  PA31 TWCK0              AUDIO_TWCK0_PA31 2  SCLK       Pulled up
  PB10 AUDIO_PCK2/EXP     AUDIO_PCK2_PB10  28 MCLK
  PB27 AUDIO/HDMI_TK0/EXP AUDIO_TK0_PB27   29 BCLK/GPIO4 Note TK0 and RK0 are mutually exclusive
  PB26 AUDIO_RK0          AUDIO_RK0_PB26   29 "  "/"   " "  " " " " " " " " " "      " "       "
  PB30 AUDIO_RF/ZIG_TWCK2 AUDIO_RF0_PB30   30 LRCLK      Note TF0 and RF0 are mutually exclusive
  PB31 AUDIO/HDMI_TF0/EXP AUDIO_TF0_PB31   30 "   "      "  " " " " " " " " " "      " "       "
  PB29 AUDIO_RD0/ZIG_TWD2 AUDIO_RD0_PB29   31 ADCDAT
  PB28 AUDIO/HDMI_TD0/EXP AUDIO_TD0_PB28   32 ACDAT
  PE4  AUDIO_IRQ          AUDIO_IRQ_PE4    1  IRQ/GPIO1  Audio interrupt
  ---- ------------------ ---------------- ------------- ---------------------------------------
  Note that jumper JP23 must be closed to connect AUDIO_TWD0_PA30 (Rev C. only)

  WM8904 Configuration
  --------------------
    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_XDMAC0=y                 : XDMAC0 required by SSC0
      CONFIG_SAMA5_TWI0=y                   : Enable TWI0 driver support
      CONFIG_SAMA5_SSCO=y                   : Enable SSC0 driver support

    System Type -> SSC0 Configuration
      CONFIG_SAMA5D4_MB_REVE=y              : No WM8904 for Rev C version of the board
      CONFIG_SAMA5_SSC_MAXINFLIGHT=16
      CONFIG_SAMA5_SSC0_DATALEN=16

    Device Drivers -> I2C Driver Support
      CONFIG_I2C=y                          : Enable I2C support
      CONFIG_I2C_RESET=n                    : (Maybe y, if you have bus problems)

    System Type -> SSC Configuration
      CONFIG_SAMA5_SSC_MAXINFLIGHT=16       : Up to 16 pending DMA transfers
      CONFIG_SAMA5_SSC0_DATALEN=16          : 16-bit data
      CONFIG_SAMA5_SSC0_RX=y                : Support a receiver (although it is not used!)
      CONFIG_SAMA5_SSC0_RX_RKINPUT=y        : Receiver gets clock the RK0 input
      CONFIG_SAMA5_SSC0_RX_FSLEN=1          : Minimal frame sync length
      CONFIG_SAMA5_SSC0_RX_STTDLY=1         : Start delay
      CONFIG_SAMA5_SSC0_TX=y                : Support a transmitter
      CONFIG_SAMA5_SSC0_TX_RXCLK=y          : Transmitter gets clock the RXCLCK
      CONFIG_SAMA5_SSC0_TX_FSLEN=0          : Disable frame synch generation
      CONFIG_SAMA5_SSC0_TX_STTDLY=1         : Start delay
      CONFIG_SAMA5_SSC0_TX_TKOUTPUT_NONE=y  : No output

    Audio Support
      CONFIG_AUDIO=y                        : Audio support needed
      CONFIG_AUDIO_FORMAT_PCM=y             : Only PCM files are supported
      CONFIG_AUDIO_NUM_BUFFERS=8            : Number of audio buffers
      CONFIG_AUDIO_BUFFER_NUMBYTES=8192     : Audio buffer size

    Drivers -> Audio
      CONFIG_I2S=y                          : General I2S support
      CONFIG_DRIVERS_AUDIO=y                : Audio device support
      CONFIG_AUDIO_WM8904=y                 : Build WM8904 driver character driver

    Board Selection
      CONFIG_SAMA5D4EK_WM8904_I2CFREQUENCY=400000
      CONFIG_SAMA5D4EK_WM8904_SRCMAIN=y    : WM8904 MCLK is the SAMA5D Main Clock

    Library Routines
      CONFIG_SCHED_WORKQUEUE=y              : MW8904 driver needs work queue support

  I2S Loopback Test
  -----------------

  The I2S driver was verified using a special I2C character driver (at
  nuttx/drivers/audio/i2schar.c) and a test driver at apps/examples/i2schar.
  The I2S driver was verified in loopback mode with no audio device.  That
  test case has never been exercised on the SAMA454-EK.  See the README.txt
  file at SAMA5D4-EK for information about how you might implement this test
  for the SAMA5D4-EK.

  The NxPlayer
  ------------

  The NxPlayer is a audio library and command line application for playing
  audio file.  The NxPlayer can be found at apps/system/nxplayer.  If you
  would like to add the NxPlayer, here are some recommended configuration
  settings.

  First of all, the NxPlayer depends on the NuttX audio subsystem.  See the
  "WM8904 Configuration" above for an example of how the audio subsystem is
  configured to use the WM8904 CODED with PCM decoding.  Or, for testing
  purposes, here is how might want to configure NULL, do-nothing audio
  device:

  Audio Support ->
    CONFIG_AUDIO=y
    CONFIG_AUDIO_NUM_BUFFERS=4
    CONFIG_AUDIO_BUFFER_NUMBYTES=8192
    CONFIG_AUDIO_FORMAT_PCM=y

    CONFIG_AUDIO_NULL=y
    CONFIG_AUDIO_NULL_BUFFER_SIZE=8192
    CONFIG_AUDIO_NULL_MSG_PRIO=1
    CONFIG_AUDIO_NULL_WORKER_STACKSIZE=768

  Then the NxPlayer can be enabled as follows:

  System Libraries and NSH Add-Ons -> NxPlayer media player / command line ->
    CONFIG_SYSTEM_NXPLAYER=y                     : Build the NxPlayer library
    CONFIG_NXPLAYER_PLAYTHREAD_STACKSIZE=1500    : Size of the audio player stack
    CONFIG_NXPLAYER_COMMAND_LINE=y               : Build command line application
    CONFIG_NXPLAYER_INCLUDE_HELP=y               : Includes a help command
    CONFIG_NXPLAYER_INCLUDE_DEVICE_SEARCH=n      : (Since there is only one audio device)
    CONFIG_NXPLAYER_INCLUDE_PREFERRED_DEVICE=y   : Only one audio device is supported
    CONFIG_NXPLAYER_FMT_FROM_EXT=y               : (Since only PCM is supported)
    CONFIG_NXPLAYER_FMT_FROM_HEADER=n            : (Since only PCM is supported)
    CONFIG_NXPLAYER_INCLUDE_MEDIADIR=y           : Specify a media directory
    CONFIG_NXPLAYER_DEFAULT_MEDIADIR="/mnt/sdcard"  : See below
    CONFIG_NXPLAYER_RECURSIVE_MEDIA_SEARCH=y     : Search all sub-directories
    CONFIG_NXPLAYER_INCLUDE_SYSTEM_RESET=y       : Add support for reset command

  You must include the full path to the location where NxPlayer can find the
  media files.  That path is given by CONFIG_NXPLAYER_DEFAULT_MEDIADIR.
  Here I use the example "/mnt/scard".  That is a location where you could,
  for example, mount an MMC/SD card driver.

TM7000 LCD/Touchscreen
======================

  The TM7000 LCD is available for the SAMA5D4-EK.  See documentation
  available on the Precision Design Associates website:
  http://www.pdaatl.com/doc/tm7000.pdf

  The TM7000 features:

    - 7 inch LCD at 800x480 18-bit RGB resolution and white backlight
    - Projected Capacitive Multi-Touch Controller based on the Atmel
      MXT768E maXTouch IC
    - 4 Capacitive Navigation Keys available via an Atmel AT42QT1070
      QTouch Button Sensor IC
    - 200 bytes of non-volatile serial EEPROM

  NOTE: It appears that my TM7000 differs slightly from the version
  described in the tm7000.pdf file:  That document claims that the
  hardware interface to the LCD is 18-bit RGB666; but the one that
  I have is certainly 24-bit RGB888.  If you have LCD issues, you may
  need to tweak some of the settings in boards/arm/sama5/sama5d4-ek/include/board.h.

  Jumper JP2 selects either the EMAC1 or the LCD by controlling the
  the LCD_ETH1_CONFIG signal on the board.

    - JP2 open, LCD_ETH1_CONFIG pulled high:

      LCD_ETH1_CONFIG=1: LCD 5v enable(LCD_DETECT#=0); ETH1 disable

    - JP2 closed, LCD_ETH1_CONFIG grounded:

      LCD_ETH1_CONFIG=0: LCD 5v disable; ETH1 enable

  LCD Connector
  -------------

  ------------------------- ----------------------- --------
  SAMA5D4-EK                TM7000                  FUNCTION
  ------------------------- ----------------------- --------
  LCD_PE24       J9 pin 5   ~MXT_CHG      J4 pin 5  MXT
  LCD_PE25       J9 pin 6   ~QT_CHG       J4 pin 6  QT
  LCD_TWCK0_PA31 J9 pin 7   I2C SCL       J4 pin 7  MXT,QT
  LCD_TWD0_PA30  J9 pin 8   I2C SDA       J4 pin 8  MXT,QT
  LCD_DAT0_PA0   J9 pin 18  LCD_DATA_0    J4 pin 18 LCD
  LCD_DAT1_PA1   J9 pin 19  LCD_DATA_1    J4 pin 19 LCD
  LCD_DAT2_PA2   J9 pin 20  LCD_DATA_2    J4 pin 20 LCD
  LCD_DAT3_PA3   J9 pin 21  LCD_DATA_3    J4 pin 21 LCD
  LCD_DAT4_PA4   J9 pin 22  LCD_DATA_4    J4 pin 22 LCD
  LCD_DAT3_PA5   J9 pin 23  LCD_DATA_5    J4 pin 23 LCD
  LCD_DAT6_PA6   J9 pin 24  LCD_DATA_6    J4 pin 24 LCD
  LCD_DAT7_PA7   J9 pin 25  LCD_DATA_7    J4 pin 25 LCD
  LCD_DAT8_PA8   J9 pin 26  LCD_DATA_8    J4 pin 26 LCD
  LCD_DAT9_PA9   J9 pin 27  LCD_DATA_9    J4 pin 27 LCD
  LCD_DAT10_PA10 J9 pin 28  LCD_DATA_10   J4 pin 28 LCD
  LCD_DAT11_PA11 J9 pin 29  LCD_DATA_11   J4 pin 29 LCD
  LCD_DAT12_PA12 J9 pin 16  LCD_DATA_12   J4 pin 16 LCD
  LCD_DAT13_PA13 J9 pin 12  LCD_DATA_13   J4 pin 12 LCD
  LCD_DAT14_PA14 J9 pin 14  LCD_DATA_14   J4 pin 14 LCD
  LCD_DAT15_PA15 J9 pin 10  LCD_DATA_15   J4 pin 10 LCD
  ------------------------- ----------------------- --------
  LCD_DAT16_PA16 J10 pin 5  LCD_DATA_16   J5 pin 5  LCD
  LCD_DAT17_PA17 J10 pin 6  LCD_DATA_17   J5 pin 6  LCD
  LCD_DAT18_PA18 J10 pin 7  LCD_DATA_18   J5 pin 7  LCD
  LCD_DAT19_PA19 J10 pin 8  LCD_DATA_19   J5 pin 8  LCD
  LCD_DAT20_PA20 J10 pin 9  LCD_DATA_20   J5 pin 9  LCD
  LCD_DAT21_PA21 J10 pin 10 LCD_DATA_21   J5 pin 10 LCD
  LCD_DAT22_PA22 J10 pin 11 LCD_DATA_22   J5 pin 11 LCD
  LCD_DAT23_PA23 J10 pin 12 LCD_DATA_23   J5 pin 12 LCD
  LCD_DISP_PA25  J10 pin 15 DISP          J5 pin 15 LCD (Display Enable)
  LCD_PWM_PA24   J10 pin 16 Backlight PWM J5 pin 16 LCD
  LCD_VSYNC_PA26 J10 pin 17 VSYNC         J5 pin 17 LCD
  LCD_HSYNC_PA27 J10 pin 18 HSYNC         J5 pin 18 LCD
  LCD_DEN_PA29   J10 pin 19 DE            J5 pin 19 LCD
  LCD_PCK_PA28   J10 pin 20 PCLK          J5 pin 20 LCD
  AD0_XP         J10 pin 23 N/C           J5 pin 23 N/A
  AD1_XM         J10 pin 24 N/C           J5 pin 24 N/A
  AD2_YP         J10 pin 25 N/C           J5 pin 25 N/A
  AD3_YM         J10 pin 26 N/C           J5 pin 26 N/A
  AD4_LR         J10 pin 27 N/C           J5 pin 27 N/A
  1Wire_PE28     J10 pin 28 1-Wire        J5 pin 28 EE
  LCD_SPI1_SO    J10 pin 31 N/C           J5 pin 31 N/A
  LCD_SPI1_SI    J10 pin 32 N/C           J5 pin 32 N/A
  LCD_SPI1_CLK   J10 pin 33 N/C           J5 pin 33 N/A
  LCD_SPI1_CS2   J10 pin 34 N/C           J5 pin 34 N/A
  EN_PWRLCD      J10 pin 35 N/C           J5 pin 35 N/A
  LCD_DETECT#    J10 pin 36 LCD Presence  J5 pin 36 All
  RXD4_PE26      J10 pin 37 N/C           J5 pin 37 N/A
  XD4_PE27       J10 pin 38 N/C           J5 pin 38 N/A
  ------------------------- ----------------------- --------

  LCD Configuration
  -----------------

  Here is a configuration that enables the LCD with backlight in RGB565
  color mode.  Notice that this configuration sets up an LCD framebuffer of
  size 6,291,456 (0x0060:0000, 6MiB) at the end of DRAM. DRAM begins at
  address 0x2000:0000 and has size 268,435,456 (0x1000:0000); The
  framebuffer the begins at 0x2000:0000 + 0x1000:0000 - 0x0060:0000 =
  0x2fa0:0000.

    System Type -> SAMA5 Peripheral Support ->
    CONFIG_SAMA5_LCDC=y                    : Enable LCDC

    System Type -> LCDC Configuration ->
    CONFIG_SAMA5_LCDC_BACKLIGHT=y          : With backlight
    CONFIG_SAMA5_LCDC_DEFBACKLIGHT=0xc8
    CONFIG_SAMA5_LCDC_BACKCOLOR=0x7b5d     : Color to use when clearing the display
    CONFIG_SAMA5_LCDC_FB_VBASE=0x2fa00000  : Set aside the framebuffer
    CONFIG_SAMA5_LCDC_FB_PBASE=0x2fa00000
    CONFIG_SAMA5_LCDC_FB_SIZE=6291456
    CONFIG_SAMA5_LCDC_BASE_ROT0=y          : No rotation
    CONFIG_SAMA5_LCDC_BASE_RGB565=y        : RGB565 color format

  This framebuffer size must then be subtracted from the memory available in the
  heap (0x3000:0000 - 0x0058:0000 = 0x2fa8:0000):

    System Type -> Heap Configuration ->
    CONFIG_SAMA5_DDRCS_RESERVE=y           : Reserve DRAM for the framebuffer
    CONFIG_SAMA5_DDRCS_HEAP_END=0x2fa00000 : End of DRAM heap (excludes framebuffer)

  There are several simple graphics examples under apps/examples/ that can
  be use to verify the LCD: nx, nxhello, nximage, nxlines, nxtext.  See
  apps/examples/README.txt for information about configuring these examples.

  For example, these settings will enable the apps/examples/nx example.  The
  NX example is a simple test using the NuttX graphics system (NX).  This
  test case focuses on general window controls, movement, mouse and keyboard
  input.  It requires no user interaction.

  First you need to enable NuttX graphics support:

    Graphics Support ->
    CONFIG_NX=y                            : Enable NX graphics
    CONFIG_NX_NPLANES=1                    : 1 color plane
    CONFIG_NX_PACKEDMSFIRST=y

    Graphics Support -> Supported Pixel Depths ->
    CONFIG_NX_DISABLE_1BPP=y               : Disable all resolutions except 16 bpp
    CONFIG_NX_DISABLE_2BPP=y
    CONFIG_NX_DISABLE_4BPP=y
    CONFIG_NX_DISABLE_8BPP=y
    CONFIG_NX_DISABLE_24BPP=y
    CONFIG_NX_DISABLE_32BPP=y

    Graphics Support -> Input Devices ->
    CONFIG_NX_XYINPUT=y                    : Build in mouse/touchscreen support (not used)
    CONFIG_NX_KBD=y                        : Build in keyboard support (not used)

    Graphics Support -> Framed Window Borders ->
    CONFIG_NXTK_BORDERWIDTH=4              : Framed window configuration
    CONFIG_NXTK_DEFAULT_BORDERCOLORS=y

    Graphics Support -> Font Selections ->
    CONFIG_NXFONTS_CHARBITS=7              : Font configuration
    CONFIG_NXFONT_SERIF22X28B=y

  Then you can enable the NX example:

    Application Configuration -> Examples -> NX graphics example
    CONFIG_EXAMPLES_NX=y                   : Enable the NX example
    CONFIG_EXAMPLES_NX_VPLANE=0            : Use color plane 0
    CONFIG_EXAMPLES_NX_DEVNO=0             : Use device zero
    CONFIG_EXAMPLES_NX_DEFAULT_COLORS=y    : Use default colors
    CONFIG_EXAMPLES_NX_DEFAULT_FONT=y      : Use default fonts
    CONFIG_EXAMPLES_NX_BPP=16              : Use 16 bpp
    CONFIG_EXAMPLES_NX_TOOLBAR_HEIGHT=16   : Configure toolbar

  maXTouch
  --------
  Both the MXT768E and the AT42QT1070 are I2C devices with interrupting
  PIO pins:

  ------------------------ -----------------
  SAMA5D4-EK               TM7000
  ------------------------ -----------------
  J9 pin 5 LCD_PE24        J4 pin 5 ~CHG_mxt
  J9 pin 6 LCD_PE25        J4 pin 6 ~CHG_QT
  J9 pin 7 LCD_TWCK0_PA31  J4 pin 7 SCL_0
  J9 pin 8 LCD_TWD0_PA30   J4 pin 8 SDA_0
  ------------------------ -----------------

  The schematic indicates the MXT468E address is 0x4c/0x4d.

  Here are the configuration settings the configuration settings that will
  enable the maXTouch touchscreen controller:

  System Type
    CONFIG_SAMA5_TWI0=y     : Enable the TWI0 peripheral
    CONFIG_SAMA5_PIO_IRQ=y  : Support for PIOE interrupts
    CONFIG_SAMA5_PIOE_IRQ=y

  Device Drivers
    CONFIG_INPUT=y          : Input device support
    CONFIG_INPUT_MXT=y      : Enable maXTouch input device

    Optionally, use CONFIG_ARCH_HAVE_I2CRESET=y if you have issues
    with other I2C devices on board locking up the I2C bus.

  Board Configuration
    CONFIG_SAMA5D4EK_MXT_DEVMINOR=0
    CONFIG_SAMA5D4EK_MXT_I2CFREQUENCY=100000

  There is a test at apps/examples/touchscreen that can be enabled to
  build in a touchscreen test:

    CONFIG_EXAMPLES_TOUCHSCREEN=y
    CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH="/dev/input0"
    CONFIG_EXAMPLES_TOUCHSCREEN_MINOR=0

  Usage is like:

    nsh> tc [<number-of-touches>]

  QTouch Button Sensor
  --------------------
  To be provided.

  LCD
  ---
  To be provided.

Tickless OS
===========

  Background
  ----------
  By default, a NuttX configuration uses a periodic timer interrupt that
  drives all system timing. The timer is provided by architecture-specific
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

SAMA4D4-EK Configuration Options
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
    CONFIG_ARCH_CHIP_ATSAMA5D44=y

  CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
  hence, the board that supports the particular chip or SoC.

    CONFIG_ARCH_BOARD="sama5d4-ek" (for the SAMA4D4-EK development board)

  CONFIG_ARCH_BOARD_name - For use in C code

    CONFIG_ARCH_BOARD_SAMA5D4_EK=y

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
    CONFIG_SAMA5_SPI0        - Serial Peripheral Interface 0
    CONFIG_SAMA5_SPI1        - Serial Peripheral Interface 1
    CONFIG_SAMA5_TC0         - Timer Counter 0 (ch. 0, 1, 2)
    CONFIG_SAMA5_TC1         - Timer Counter 1 (ch. 3, 4, 5)
    CONFIG_SAMA5_PWM         - Pulse Width Modulation Controller
    CONFIG_SAMA5_ADC         - Touch Screen ADC Controller
    CONFIG_SAMA5_XDMAC0      - XDMA Controller 0
    CONFIG_SAMA5_XDMAC1      - XDMA Controller 1
    CONFIG_SAMA5_UHPHS       - USB Host High Speed
    CONFIG_SAMA5_UDPHS       - USB Device High Speed
    CONFIG_SAMA5_EMAC0       - Ethernet MAC 0 (GMAC0)
    CONFIG_SAMA5_EMAC1       - Ethernet MAC 1 (GMAC1)
    CONFIG_SAMA5_LCDC        - LCD Controller
    CONFIG_SAMA5_ISI         - Image Sensor Interface
    CONFIG_SAMA5_SSC0        - Synchronous Serial Controller 0
    CONFIG_SAMA5_SSC1        - Synchronous Serial Controller 1
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
  Each SAMA4D4-EK configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh sama5d4-ek:<subdir>

  Before building, make sure that the PATH environment variable includes
  the correct path  to the directory than holds your toolchain binaries.

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

  4. The SAMA5Dx is running at 528MHz by default in these configurations.

       Board Selection -> CPU Frequency
         CONFIG_SAMA5D4EK_528MHZ=y       : Enable 528MHz operation
         CONFIG_BOARD_LOOPSPERMSEC=65775 : Calibrated on SAMA5D3-Xplained at
                                         : 528MHz running from SDRAM

  Configuration Sub-directories
  -----------------------------
  Summary:  Some of the descriptions below are long and wordy. Here is the
  concise summary of the available SAMA4D4-EK configurations:

    at25boot: This is a little program to write a boot loader into the
      AT25 serial FLASH (in particular, dramboot).  See the description
      below and the section above entitled "Creating and Using AT25BOOT"
      for more information
    bridge:  This is a simple testing that exercises EMAC0 and EMAC1 for
      a simple UDP relay bridge test.
    dramboot: This is a little program to help debug of code in DRAM.  See
      the description below and the section above entitled "Creating and
      Using DRAMBOOT" for more information
    elf:  Demonstrates execution of ELF file from a file system.
    ipv6: This is another version of the NuttShell configuration.  It is
      very similar to the nsh configuration except that it has IPv6 enabled
      and IPv4 disabled.
    knsh: An NSH configuration used to test the SAMA5D kernel build
      configuration.  Uses a tiny NSH configuration that runs at
      start time from a mounted file system.
    nsh:  This is an NuttShell (NSH) configuration that supports extensive
      functionality as possible (unlike the minimal ramtest configuration).
      See the detailed description below for a summary of the feature
      set supported by this configuration.  You may want to disable some
      of these features if you plan to use the NSH as a platform for
      debugging and integrating new features.
    nxwm: This is a special configuration setup for the NxWM window manager
      UnitTest.  It integrates support for both the SAMA5 LCDC and the
      SAMA5 ADC touchscreen controller and provides a more advance
      graphics demo. It provides an interactive windowing experience.
    ramtest: This is a stripped down version of NSH that runs out of
      internal SRAM.  It configures SDRAM and supports only the RAM test
      at apps/examples/ramtest.  This configuration is useful for
      bringing up SDRAM.

  There may be issues with some of these configurations.  See the details
  before of the status of individual configurations.

  Now for the gory details:

  at25boot:

    To work around some SAM-BA availability issues that I had at one time,
    I created the at25boot program. at25boot is a tiny program that runs in
    ISRAM.  at25boot will enable SDRAM and configure the AT25 Serial FLASH.
    It will prompt and then load an Intel HEX program into SDRAM over the
    serial console. If the program is successfully loaded in SDRAM, at25boot
    will copy the program at the beginning of the AT26 Serial FLASH.
    If the jumpering is set correctly, the SAMA5D4 RomBOOT loader will
    then boot the program from the serial FLASH the next time that it
    reset.

    The usage is different, otherwise I believe the notes for the dramboot
    configuration should all apply.

    STATUS:  While this program works great and appears to correctly write
    the binary image onto the AT25 Serial FLASH, the RomBOOT loader will
    not boot it!  I believe that is because the secure boot loader has some
    undocumented requirements that I am unaware of. (2014-6-28)

  bridge:

    This is a simple testing that exercises EMAC0 and EMAC1 for a simple
    UDP relay bridge test using apps/examples/bridge.  See
    apps/examples/README.txt for more information about this test.

    NOTES:

    1. This configuration uses the USART3 for the serial console
       which is available at the "DBGU" RS-232 connector (J24).  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    2. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain (such as the CodeSourcery
       toolchain).  Both the build environment and the toolchain
       selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under Windows
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain

       If you are running on Linux, make *certain* that you have
       CONFIG_HOST_LINUX=y *before* the first make or you will create a
       corrupt configuration that may not be easy to recover from. See
       the warning in the section "Information Common to All Configurations"
       for further information.

     3. EMAC0 and EMAC1 connect KSZ8081RNB PHYs and are available at the
        ETH0 and ETH1 connector, respectively.

        The ETH1 signals go through line drivers that are enabled via the
        board LCD_ETH1_CONFIG signal.  Jumper JP2 selects either the EMAC1
        or the LCD by controlling the LCD_ETH1_CONFIG signal on the
        board.

        - JP2 open, LCD_ETH1_CONFIG pulled high:

          LCD_ETH1_CONFIG=1: LCD 5v enable(LCD_DETECT#=0); ETH1 disable

        - JP2 closed, LCD_ETH1_CONFIG grounded:

          LCD_ETH1_CONFIG=0: LCD 5v disable; ETH1 enable

    STATUS:

      2014-11-17:  Configuration created.  Only partially verified.  EMAC0
        seems functional, but EMAC1 does not respond to pings.  Cannot perform
        the full bridge test yet anyway because there still is no host-side
        test driver in apps/examples/bridge.
      2014-11-18:  Continued working with EMAC1:  It does not work.  No
        errors are reported, link auto-negotiation works without error, but I
        cannot send or receive anything on EMAC1:  TX transfers all timeout
        with no interrupts and nothing appearing on the line; RX transfers
        are not received... no RX interrupts and no RX status gets set.  This
        appears to be some very low-level issue, perhaps a pin configuration
        problem.  But I am not seeing it yet. No interrupts are ever received.

  dramboot:

    This is a little program to help debug of code in DRAM.  It does the
    following:

    - Sets the clocking so that the SAMA5 is running at 528MHz.
    - Configures DRAM,
    - Loads and Intel HEX file into DRAM over the terminal port,
    - Waits for you to break in with GDB (or optionally starts the
      newly loaded program).

    At that point, you can set the PC and begin executing from SDRAM under
    debug control.  See the section entitled "Creating and Using
    DRAMBOOT" above.

    NOTES:

    1. This configuration uses the USART3 for the serial console
       which is available at the "DBGU" RS-232 connector (J24).  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    2. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain (such as the CodeSourcery
       toolchain).  Both the build environment and the toolchain
       selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under Windows
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain

       If you are running on Linux, make *certain* that you have
       CONFIG_HOST_LINUX=y *before* the first make or you will create a
       corrupt configuration that may not be easy to recover from. See
       the warning in the section "Information Common to All Configurations"
       for further information.

    3. This configuration executes out of internal SRAM flash and is
       loaded into SRAM by the boot RomBoot from NAND, Serial
       DataFlash, SD card or from a TFTPC sever via the Boot ROM.
       Data also is positioned in SRAM.

    2. The default dramboot program initializes the DRAM memory,
       displays a message, loads an Intel HEX program into DRAM over the
       serial console and halts.  The dramboot program can also be
       configured to jump directly into DRAM without requiring the
       final halt and go by setting CONFIG_SAMA5D4EK_DRAM_START=y in the
       NuttX configuration.

    3. Be aware that the default dramboot also disables the watchdog.
       Since you will not be able to re-enable the watchdog later, you may
       need to set CONFIG_SAMA5_WDT=y in the NuttX configuration file.

    4. If you put dramboot on the Serial FLASH, you can automatically
       boot to SDRAM on reset.  See the section "Creating and Using DRAMBOOT"
       above.

    5. Here are the steps that I use to execute this program in SRAM
       using only the ROM Bootloader:

       a) Hold the DIS_BOOT button and

       b) With the DIS_BOOT button pressed, power cycle the board. A
          reset does not seem to be sufficient.

       c) The serial should show RomBOOT in a terminal window (at 115200
          8N1) and nothing more.

       d) Press ENTER in the terminal window a few times to enable JTAG.

       e) Start the Segger GDB server.  It should successfully connect to
          the board via JTAG (if JTAG was correctly enabled in step d)).

       f) Start GDB, connect, to the GDB server, load NuttX, and debug.

          gdb> target remote localhost:2331
          gdb> mon halt (don't do mon reset)
          gdb> load nuttx
          gdb> mon reg pc (make sure that the PC is 0x200040
          gdb> ... and debug ...

    STATUS:  I don't have a working SAM-BA at the moment and there are issues
    with my AT25BOOT (see above).  I currently work around these issues by
    putting DRAMBOOT on a microSD card (as boot.bin).  The RomBOOT loader does
    boot that image without issue.

  elf:

    Demonstrates execution of ELF file from a file system using
    apps/examples/elf.  This is a very simple configuration so there is not
    really much that needs to be said.

    NOTES:

    1. This configuration uses the USART3 for the serial console
       which is available at the "DBGU" RS-232 connector (J24).  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    2. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain (such as the CodeSourcery
       toolchain).  Both the build environment and the toolchain
       selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under windows
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y     : GNU EABI toolchain

       If you are running on Linux, make *certain* that you have
       CONFIG_HOST_LINUX=y *before* the first make or you will create a
       corrupt configuration that may not be easy to recover from. See
       the warning in the section "Information Common to All Configurations"
       for further information.

    3. This configuration currently has Cortex-A address environments selected.
       With this option, the MMU is used to create a custom address environment
       for each ELF program (effectively making them processes).  This option
       can be disabled in which case the ELF programs will simply execute out
       normal memory allocated from the heap.  To disable this feature:

      System Type -> Architecture Options
        CONFIG_ARCH_ADDRENV=n                  : Disable address environment support

      System Type -> Heap Configuration
        CONFIG_SAMA5_DDRCS_RESERVE=n           : Don't reserve any page cache memory
        CONFIG_SAMA5_DDRCS_PGHEAP=n            : Don't try to set up the page allocator

      Memory Management
        CONFIG_GRAN=n                          : Disable the granule allocator
        CONFIG_MM_PGALLOC=n                    : Disable the page allocator

    4. A system call interface is enabled and the ELF test programs interface
       with the base RTOS code system calls.  This eliminates the need for symbol
       tables to link with the base RTOS (symbol tables are still used, however,
       to interface with the common C library instantiation).  Relevant
       configuration settings:

      RTOS Features -> System call support
        CONFIG_LIB_SYSCALL=y                   : Enable system call support
        CONFIG_SYS_NNEST=2                     : Max number of nested system calls

      Application Configurations -> Examples -> ELF Loader Example
        CONFIG_EXAMPLES_ELF_SYSCALL=y          : Link apps with the SYStem call library

    5. By default, this configuration uses the ROMFS file system.  It can also
       be modified to use the compressed CROMFS:

       -CONFIG_PATH_INITIAL="/mnt/romfs"
       +CONFIG_PATH_INITIAL="/mnt/cromfs"

       -CONFIG_FS_ROMFS=y
       +CONFIG_FS_CROMFS=y

       -CONFIG_EXAMPLES_ELF_ROMFS=y
       +CONFIG_EXAMPLES_ELF_CROMFS=y

    STATUS:
      2014-8-24: This configuration works with the address environment
                 and system call options disabled.
      2014-8-28: Now this option works well well with address environments
                 enabled.  There is a potential issue with the use of
                 task_create() as it is used in the ELF test, but the code
                 seems to survive it. See:

                 https://cwiki.apache.org/confluence/display/NUTTX/Memory+Configurations

      2014-8-29: System call interface verified.
      2014-9-16: Reverified after fixing changes for the knsh configuration
                 that broke this one.  All seems to be well now.

  ipv6:
  ----
    This is another version of the NuttShell configuration.  It is very
    similar to the nsh configuration except that it has IPv6 enabled and
    IPv4 disabled.  Several network utilities that are not yet available
    under IPv6 are disabled.

    NOTES:

    1. As of 2015-02-09, this configuration was identical to the nsh
       configuration other than using IPv6.  So all of the notes below
       regarding the nsh configuration apply.

       Telnet does work with IPv6 but is not enabled in this
       configuration (but could be).

    2. This configuration can be modified to that both IPv4 and IPv6
       are support.  Here is a summary of the additional configuration
       settings requird to support both IPv4 and IPv6:

         CONFIG_NET_IPv4=y
         CONFIG_NET_ARP=y
         CONFIG_NET_ARP_SEND=y (optional)
         CONFIG_NET_ICMP=y
         CONFIG_NET_ICMP_SOCKET=y

         CONFIG_NETDB_DNSCLIENT=y
         CONFIG_NETUTILS_TELNETD=y

         CONFIG_NSH_IPADDR=0x0a000002
         CONFIG_NSH_DRIPADDR=0x0a000001
         CONFIG_NSH_NETMASK=0xffffff00
         CONFIG_NSH_TELNET=y

       Then from NSH, you have both ping and ping6 commands:

         nsh> ping 10.0.0.1
         nsh> ping6 fc00::1

       And from the host you can do similar:

         ping 10.0.0.2
         ping6 fc00::2   (Linux)
         ping -6 fc00::2 (Windows cmd)

       and Telnet is now enabled and works from the host... but only using
       IPv6 addressing:

         telnet fc00::2

       That is because the Telnet daemon will default to IPv6 and there is
       no Telnet option to let you select which if both IPv4 and IPv6 are
       enabled.

    3. You can enable IPv6 autonomous address configuration with the
       following changes to the configuration:

       + CONFIG_NET_ICMPv6_AUTOCONF=y
       + CONFIG_ICMPv6_AUTOCONF_DELAYMSEC=100
       + CONFIG_ICMPv6_AUTOCONF_MAXTRIES=5

       - CONFIG_NSH_DRIPv6ADDR_1=0xfc00
       - CONFIG_NSH_DRIPv6ADDR_2=0x0000
       - CONFIG_NSH_DRIPv6ADDR_3=0x0000
       - CONFIG_NSH_DRIPv6ADDR_4=0x0000
       - CONFIG_NSH_DRIPv6ADDR_5=0x0000
       - CONFIG_NSH_DRIPv6ADDR_6=0x0000
       - CONFIG_NSH_DRIPv6ADDR_7=0x0000
       - CONFIG_NSH_DRIPv6ADDR_8=0x0001

       - CONFIG_NSH_IPv6ADDR_1=0xfc00
       - CONFIG_NSH_IPv6ADDR_2=0x0000
       - CONFIG_NSH_IPv6ADDR_3=0x0000
       - CONFIG_NSH_IPv6ADDR_4=0x0000
       - CONFIG_NSH_IPv6ADDR_5=0x0000
       - CONFIG_NSH_IPv6ADDR_6=0x0000
       - CONFIG_NSH_IPv6ADDR_7=0x0000
       - CONFIG_NSH_IPv6ADDR_8=0x0002
       - CONFIG_NSH_IPv6NETMASK_1=0xffff
       - CONFIG_NSH_IPv6NETMASK_2=0xffff
       - CONFIG_NSH_IPv6NETMASK_3=0xffff
       - CONFIG_NSH_IPv6NETMASK_4=0xffff
       - CONFIG_NSH_IPv6NETMASK_5=0xffff
       - CONFIG_NSH_IPv6NETMASK_6=0xffff
       - CONFIG_NSH_IPv6NETMASK_7=0xffff
       - CONFIG_NSH_IPv6NETMASK_8=0xff80

  knsh:
    An NSH configuration used to test the SAMA5D kernel build configuration.

    NOTES:

    1. This configuration uses the USART3 for the serial console
       which is available at the "DBGU" RS-232 connector (J24).  That
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

    3. Some key setup configuration values for this configuration:

       Build Setup -> Build Configuration -> Memory Organization
         CONFIG_BUILD_KERNEL=y                  : Kernel build enabled

       RTOS Features -> Tasks and Scheduling
         CONFIG_INIT_FILE=y                     : Start-up is via an ELF file
         CONFIG_INIT_FILEPATH="/bin/init"       : The location of the startup
         CONFIG_SCHED_HAVE_PARENT=y             : Needed to handle task exit

       RTOS Features -> RTOS hooks
         CONFIG_SCHED_ONEXIT=y                 : Needed to handle task exit
         CONFIG_SCHED_ONEXIT_MAX=2

       Memory Management
        CONFIG_MM_KERNEL_HEAP=y                : Enable a kernel heap
        CONFIG_MM_KERNEL_HEAPSIZE=8192         : (temporary.. will change)

    4. By default, this configuration is setup to boot from an SD card.
       Unfortunately, there some issues when using the SD card that prevent
       this from working properly (see STATUS below).  And alternative is to
       use a built-in ROMFS file system that does not suffer from the
       (assumed) HSMCI bug.

       So why isn't this the default configuration?  Because it does not
       build out-of-the-box.  You have to take special steps in the build
       process as described below.

       Assuming that you will want to reconfigure to use the ROMFS (rather
       than debugging HSCMI), you will need to disable all of these settings:

       System Type->ATSAMA5 Peripheral Support
         CONFIG_SAMA5_HSMCI0=n           : Disable HSMCI0 support
         CONFIG_SAMA5_XDMAC0=n           : XDMAC0 is no longer needed

       System Type
         CONFIG_SAMA5_PIO_IRQ=n          : PIO interrupts are no longer needed

       Device Drivers -> MMC/SD Driver Support
         CONFIG_MMCSD=n                  : Disable MMC/SD support

       File System
         CONFIG_FS_FAT=n                 : FAT file system no longer needed

       Board Selection
         CONFIG_SAMA5D4EK_HSMCI0_MOUNT=y : Don't mount HSMCI0 at boot

       And then enable these features in order to use the ROMFS boot file
       system:

       File System
         CONFIG_FS_ROMFS=y               : Enable the ROMFS file system

       Board Selection
         CONFIG_SAMA5D4EK_ROMFS_MOUNT=y  : Mount the ROMFS file system at boot
         CONFIG_SAMA5D4EK_ROMFS_MOUNT_MOUNTPOINT="/bin"
         CONFIG_SAMA5D4EK_ROMFS_ROMDISK_DEVNAME="/dev/ram0"
         CONFIG_SAMA5D4EK_ROMFS_ROMDISK_MINOR=0
         CONFIG_SAMA5D4EK_ROMFS_ROMDISK_SECTSIZE=512

       Then you will need to follow some special build instructions below
       in order to build and install the ROMFS file system image.

    5. Board initialization is performed before the application is started:

       RTOS Features -> RTOS Hooks
         CONFIG_BOARD_INITITIALIZE=y

       In the special ROMFS boot configuration, you need to do nothing
       additional: The board initialization will mount the ROMFS file
       system at boot time.

       In the default configuration, however, the board initialization
       will instead mount the FAT filesystem on an SD card inserted in
       the HSMCI0 slot (full size).  The SAMA4D4-EK provides two SD
       memory card slots:  (1) a full size SD card slot (J10), and (2) a
       microSD memory card slot (J11).  The full size SD card slot connects
       via HSMCI0; the microSD connects vi HSMCI1.  See the relevant
       configuration settings above in the paragraph entitled "HSMCI Card
       Slots" above.

       The SD card is mounted at /bin by this board initialization logic.
       NuttX will boot from the SD card so there are some special operational
       requirements to use this configuration:

       a. The SD card must contain a NuttX executable called 'init'
       b. The SD card must be in the HSCMCI slot when NuttX boots and must
          not be removed while NuttX is running.

       The NuttX automounter is *not* enabled.  It cannot be used it would
       mount the boot file system with a delay.  In this configuration.  The
       file system must be mounted immediately at boot up.  To accomplish
       this, the board logic supports these special configurations:

       Board Selection ->
         CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT=y
         CONFIG_SAMA5D4EK_HSMCI0_MOUNT_BLKDEV="/dev/mmcsd0"
         CONFIG_SAMA5D4EK_HSMCI0_MOUNT_FSTYPE="vfat"
         CONFIG_SAMA5D4EK_HSMCI0_MOUNT_MOUNTPOINT="/bin"

    6a. General build directions (boot from SD card):

        A. Build with no symbol table

        $ make menuconfig

          Disable ROMFS support in the .config file; Enable FAT file system
          support in the .config file.  Enable "HSMCIO boot mount" support in
          the board

        $ cd nuttx                          : Go to the NuttX build directory
        $ tools/configure.sh sama5d4-ek:knsh  : Establish this configuration
        $ export PATH=???:$PATH             : Set up the PATH variable
        $ make                              : Build the kerne with a dummy ROMFS image
                                            : This should create the nuttx ELF

        B. Create the export package

        $ make export                       : Create the kernel export package
                                            : You should have a file like
                                            : nuttx-export-*.zip

        C. Build the file system image at apps/bin

        $ cd apps/                          : Go to the apps/ directory
        $ tools/mkimport.sh -z -x <tgz-file>: Use the full path to nuttx-export-*.tar.gz
        $ make import                       : This will build the file system.

      You will then need to copy the files from apps/bin to an SD card or USB
      FLASH drive to create the bootable SD card.

      But how does the SD card/USB FLASH drive get mounted?  This must be
      done in board-specific logic before the 'init' program is started.
      That logic is not yet implemented for the case of SD card or USB FLASH
      driver

    6b. General build directions (boot from ROMFS image):

        A. Build with dummy ROMFS file system image and no symbol table

        $ make menuconfig

          Enable the ROMFS file system and board-specific "ROMFS boot mount"
          support to auto-mount the ROMFS file system on bootup.

        $ tools/configure.sh sama5d4-ek:knsh  : Establish this configuration
        $ export PATH=???:$PATH             : Set up the PATH variable
        $ touch boards/arm/sama5/sama5d4-ek/include/boot_romfsimg.h
        $ make                              : Build the kernel with a dummy ROMFS image
                                            : This should create the nuttx ELF

        B. Create the export package

        $ make export                       : Create the kernel export package
                                            : You should have a file like
                                            : nuttx-export-*.zip

        C. Build the file system image at apps/bin

        $ cd apps/                          : Go to the apps/ directory
        $ tools/mkimport.sh -z -x <tgz-file>: Use the full path to nuttx-export-*.tar.gz
        $ make import                       : This will build the file system

        D. Create the ROMFS file system image

        $ tools/mkromfsimg.sh               : Create the real ROMFS image
        $ mv boot_romfsimg.h ../nuttx/boards/arm/sama5/sama5d4-ek/include/boot_romfsimg.h

        E. Rebuild NuttX with the new file system image

        $ cd nuttx/                         : Rebuild the system with the correct
        $ make clean clean_context all      : ROMFS file system and symbol table

      But how does the ROMFS file system get mounted?  This is done in board-
      specific logic before the 'init' program is started.

    STATUS:

    2014-9-4: The kernel works up to the point where the nsh 'init'
       is started from the file system then fails.  This is good,
       however, because I do not yet have the file system in place yet.

    2014-9-8: I am seeing HSMCI read() failures while loading the ELF image
       from the SD card.  This seems odd since I have never seen other read()
       failures with HSMCI (and, hence, this may be some issue unique to this
       configuration).  In any a event, this has stopped testing for the
       moment.

       Also, the mount() in boards/arm/sama5/sama5d4x-ek/src/sam_bringup.c will fail
       unless you add a delay between the HSMCI initialization and the mount.
       No idea why (and there they is now delay in the baseline code... one
       has to be added).

       Update: I don't believe that this HSMCI error occurs if file system
       debug output is enabled.

    2014-9-11: Everything seems to be working quite nicely with the ROMFS
       file system.  A considerable amount of testing has been done and
       there are no known defects as of this writing.

    2014-9-16: After some substantial effort, I think I may have resolved
       the last of the mainstream bugs that prevented from executing other
       user processes from a user processes.  Long story but I am glad to
       have that done.

    2018-07-15:  Revisited.  It is not clear to me how, back in 2014, the
       symbol table was created.  I have added logic to created the symbol
       table.  After some additional fixes, the full build is again
       successful.

  nsh:

    This configuration directory provide the NuttShell (NSH).  This is a
    very simple NSH configuration upon which you can build further
    functionality.

    NOTES:

    1. This configuration uses the USART3 for the serial console
       which is available at the "DBGU" RS-232 connector (J24).  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    2. This configuration was verified using the SAMA5D4-MB, Rev C. board.
       There may be some differences in the released SAMA5D4-EK board.  Also,
       this configuration assumes that you have the TM7000 LCD/Touchscreen
       attached.  If you do not, you should disable the LCD and touchscreen
       drivers as described above under "TM7000 LCD/Touchscreen" and also
       below.

    3. By default, this configuration is set up to build on Windows
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

    4. This configuration supports logging of debug output to a circular
       buffer in RAM.  This feature is discussed fully in this Wiki page:
       https://cwiki.apache.org/confluence/display/NUTTX/SYSLOG . Relevant
       configuration settings are summarized below:

       Device Drivers:
       CONFIG_RAMLOG=y             : Enable the RAM-based logging feature.
       CONFIG_RAMLOG_SYSLOG=y      : This enables the RAM-based logger as the
                                     system logger.
       CONFIG_RAMLOG_NONBLOCKING=y : Needs to be non-blocking for dmesg
       CONFIG_RAMLOG_BUFSIZE=16384 : Buffer size is 16KiB

       NOTE: This RAMLOG feature is really only of value if debug output
       is enabled.  But, by default, no debug output is disabled in this
       configuration.  Therefore, there is no logic that will add anything
       to the RAM buffer.  This feature is configured and in place only
       to support any future debugging needs that you may have.

       If you don't plan on using the debug features, then by all means
       disable this feature and save 16KiB of RAM!

       NOTE: There is an issue with capturing data in the RAMLOG:  If
       the system crashes, all of the crash dump information will into
       the RAMLOG and you will be unable to access it!  You can tell that
       the system has crashed because (a) it will be unresponsive and (b)
       the RED LED will be blinking at about 2Hz.

       That is another good reason to disable the RAMLOG!

    5. This configuration executes out of SDRAM flash and is loaded into
       SDRAM from NAND, Serial DataFlash, SD card or from a TFTPC sever via
       U-Boot, BareBox, or the DRAMBOOT configuration described above.  Data
       also is positioned in SDRAM.

       The load address is different for the DRAMBOOT program and the Linux
       bootloaders.  This can easily be reconfigured, however:

         CONFIG_SAMA5D4EK_DRAM_BOOT=y

       See the section above entitled "Creating and Using DRAMBOOT" above
       for more information.  Here is a summary of the steps that I used
       to boot the NSH configuration:

         a. Create the DRAMBOOT program as described above.  It should be
            configured with CONFIG_SAMA5D4EK_DRAM_START=y so that DRAMBOOT
            will immediately start the program.  You may not want to do
            this is your prefer to break in with GDB.

         b. Write the DRAMBOOT program binary (nuttx.bin) to a microSD
            card as "boot.bin".  Insert the microSD card into the boar;
            The ROM Booloader should now boot DRAMBOOT on reset and you
            should see this message:

              Send Intel HEX file now

         c. Build the NSH version of NuttX.  Send the Intel HEX of NSH
            at the prompt.  After the file is received, NSH should start
            automatically.

       At times the past, have have tested with nuttx.bin on an SD card and
       booting with U-Boot.  These are the commands that I used to boot NuttX
       from the SD card:

         U-Boot> fatload mmc 0 0x20008000 nuttx.bin
         U-Boot> go 0x20008040

    6. Board LEDs and buttons are supported as described under "Buttons and
       LEDs".  The interrupt button test is also enabled as an NSH built-in
       commands.  To run this test, you simply inter the command:

          nsh>buttons [npresses]

       The interrupt button test will log button press information to the
       syslog.  Since the RAMLOG is enabled, the SYSLOG output will be
       captured to a circular buffer in ram and may be examined using the
       NSH dmesg command:

       nsh> buttons 2
       nsh> dmesg
       maxbuttons: 2
       Attached handler at 200106f0 to button 0 [PB_USER], oldhandler:0
       IRQ:81 Button 0:PB_USER SET:01:
         PB_USER depressed
       IRQ:81 Button 0:PB_USER SET:00:
         PB_USER released
       IRQ:81 Button 0:PB_USER SET:01:
         PB_USER depressed
       IRQ:81 Button 0:PB_USER SET:00:
         PB_USER released

    7. This configuration supports /dev/null, /dev/zero, and /dev/random.

         CONFIG_DEV_NULL=y    : Enables /dev/null
         CONFIG_DEV_ZERO=y    : Enabled /dev/zero

       Support for /dev/random is implemented using the SAMA5D4's True
       Random Number Generator (TRNG).  See the section above entitled
       "TRNG and /dev/random" for information about configuring /dev/random.

        CONFIG_SAMA5_TRNG=y   : Enables the TRNG peripheral
        CONFIG_DEV_RANDOM=y   : Enables /dev/random

    8. This configuration has support for NSH built-in applications enabled.
       Two built-in applications are included by default:

       a. The I2C Tool.  See the section above entitled "I2C Tool" and the
          note with regard to I2C below.
       b. The interrupting button test as described above in these notes.
       c. The touchscreen test program as described above under "TM7000
          LCD/Touchscreen" and also below in this notes.
       d. An LCD/graphics test program.  See the section above entitle
          "TM7000 LCD/Touchscreen" and also below in this notes.
       e. The NxPlayer command line media player.  This is a work in
          progress see the "Audio Support" section above and additional
          notes below.

    9. This configuration has support for the FAT, ROMFS, and PROCFS file
       systems built in.

       The FAT file system includes long file name support.  Please be aware
       that Microsoft claims patents against the long file name support (see
       more discussion in the top-level NOTICE file).

         CONFIG_FS_FAT=y        : Enables the FAT file system
         CONFIG_FAT_LCNAMES=y   : Enable lower case 8.3 file names
         CONFIG_FAT_LFN=y       : Enables long file name support
         CONFIG_FAT_MAXFNAME=32 : Arbitrarily limits the size of a path
                                  segment name to 32 bytes

       The ROMFS file system is enabled simply with:

         CONFIG_FS_ROMFS=y      : Enable ROMFS file system

       The ROMFS file system is enabled simply with:

         CONFIG_FS_PROCFS=y     : Enable PROCFS file system

   10. An NSH start-up script is provided by the ROMFS file system.  The ROMFS
       file system is mounted at /etc and provides:

         |- dev/
         |   |- ...
         |   `- ram0                     : ROMFS block driver
         `- etc/
             `- init.d/
                 `- rcS                  : Start-up script

       (There will, of course, be other devices under /dev including /dev/console,
       /dev/null, /dev/zero, /dev/random, etc.).

       Relevant configuration options include:

         CONFIG_NSH_ROMFSETC=y           : Enable mounting at of startup file system
         CONFIG_NSH_ROMFSMOUNTPT="/etc"  : Mount at /etc
         CONFIG_NSH_ROMFSDEVNO=0         : Device is /dev/ram0
         CONFIG_NSH_ARCHROMFS=y          : ROMFS image is at
                                           boards/arm/sama5/sama5d4-ek/include/nsh_romfsimg.h
       The content of /etc/init.d/rcS can be see in the file rcS.template that
       can be found at: boards/arm/sama5/sama5d4-ek/include/rcS.template:

         # Mount the procfs file system at /proc

         mount -f procfs /proc
         echo "rcS: Mounted /proc"

         # Create a RAMDISK at /dev/ram1, size 0.5MiB, format it with a FAT
         # file system and mount it at /tmp

         mkrd -m 1 -s 512 1024
         mkfatfs /dev/ram1
         mount -t vfat /dev/ram1 /tmp
         echo "rcS: Mounted /tmp"

       The above commands will mount the procfs file system at /proc and a
       RAM disk at /tmp.

       The second group of commands will: (1) Create a RAM disk block device
       at /dev/ram1 (mkrd).  The RAM disk will take 0.4MiB of memory (512 x
       1024).  Then it will then: (2) create a FAT file system on the ram
       disk (mkfatfs) and (3) mount it at /tmp (mount).

       So after NSH starts and runs the rcS script, we will have:

         |- dev/
         |   |- ...
         |   `- ram0                     : ROMFS block driver
         |   `- ram1                     : RAM disk block driver
         |- etc/
         |   `- init.d/
         |       `- rcS                  : Start-up script
         |- proc/
         |   |- 0/                       : Information about Task ID 0
         |   |  |- cmdline               : Command line used to start the task
         |   |  |- stack                 : Stack allocation
         |   |  |- status                : Current task status
         |   |  `- group/                : Information about the task group
         |   |      |- fd                : File descriptors open in the group
         |   |      `- status            : Status of the group
         |   |- 1/                       : Information about Task ID 1
         |   |  `- ...                   : Same pseudo-directories as for Task ID 0
         |   |- ...                      : ...
         |   |- n/                       : Information about Task ID n
         |   |  `- ...                   : Same pseudo-directories as for Task ID 0
         |   |- uptime                   : Processor uptime
         `- tmp/

       The /tmp directory can them be used for and scratch purpose.  The
       pseudo-files in the proc/ directory can be used to query properties
       of NuttX.  As examples:

         nsh> cat /proc/1/stack
         StackBase:  0x2003b1e8
         StackSize:  2044

         nsh> cat /proc/uptime
              31.89

         nsh> cat /proc/1/status
         Name:       work
         Type:       Kernel thread
         State:      Signal wait
         Priority:   192
         Scheduler:  SCHED_FIFO
         SigMask:    00000000

         nsh> cat /proc/1/cmdline
         work

         nsh> cat /proc/1/group/status
         Flags:      0x00
         Members:    1

         nsh> cat /proc/1/group/fd

         FD  POS      OFLAGS
          0        0 0003
          1        0 0003
          2        0 0003

         SD  RF TYP FLAGS

   11. The Real Time Clock/Calendar (RTC) is enabled in this configuration.
       See the section entitled "RTC" above for detailed configuration
       settings.

       The RTC alarm is not enabled by default since there is nothing in
       this configuration that uses it.  The alarm can easily be enabled,
       however, as described in the "RTC" section.

       The time value from the RTC will be used as the NuttX system time
       in all timestamp operations.  You may use the NSH 'date' command
       to set or view the RTC as described above in the "RTC" section.

       NOTE:  If you want the RTC to preserve time over power cycles, you
       will need to install a battery in the battery holder (J12) and close
       the jumper, JP13.

   12. Support for HSMCI0 is built-in by default. The SAMA4D4-EK provides
       two SD memory card slots:  (1) a full size SD card slot (J10), and
       (2) a microSD memory card slot (J11).  The full size SD card slot
       connects via HSMCI0; the microSD connects vi HSMCI1.  Support for
       the microSD slot could also be enabled with the settings provided
       in the paragraph entitled "HSMCI Card Slots" above.

       NOTE:  For now I am boot off the microSD slot so, unless are booting
       in a different manner, this HSMCI1 slot may not be useful to you
       anyway.

       The auto-mounter is also enabled.  See the section above entitled
       "Auto-Mounter".

   13. Networking is supported via EMAC0.  See the "Networking" section
       above for detailed configuration settings.  DHCP is not used in
       this configuration; rather, a hard-coded IP address of 10.0.0.2 is
       used with a netmask of 255.255.255.0.  The host is assumed to be
       10.0.0.1 in places.  You can reconfigure to enabled DHCPC or to
       change these addresses as you see fit.

       See also the "kludge" for EMAC that is documented in the To-Do list
       at the end of this README file.

       The configuration option CONFIG_NSH_NETINIT_THREAD is enabled so
       that NSH network bring-up asynchronously and in parallel on a
       separate thread.  This eliminates the (visible) networking bring-up
       delay.    This networking initialization feature by itself has
       some limitations:

         - If no network is connected, the network bring-up will fail and
           the network initialization thread will simply exit.  There are no
           retries and no mechanism to know if the network initialization was
           successful.

         - Furthermore, there is no support for detecting loss of the network
           connection and recovery of networking when the connection is restored.

       Both of these shortcomings can be eliminated by enabling the network
       monitor as described above in the "Network Monitor" paragraph.

   14. I2C Tool. This configuration enables TWI0 (only) as an I2C master
       device.  This configuration also supports the I2C tool at
       apps/system/i2c that can be used to peek and poke I2C devices on the
       TIW0 bus.  See the discussion above under "I2C Tool" for detailed
       configuration settings.

   15. Support the USB low-, high- and full-speed OHCI host driver is enabled
       enabled with the NuttX configuration file as described in the section
       above entitled "USB High-Speed Host".  Only port B and port C, the
       larger "Type A" connectors, are enabled; port A (the smaller OTG
       connector) is reserved for future use with USB device (but could also
       be configured as a USB host port if desired).

       Support for Mass Storage Class and USB (Boot) Keyboard class is also
       enabled.  The keyboard class was useful for verifying that low-speed
       devices can connect successfully, but is otherwise not used by this
       configuration.  Feel free to disable it if you like:

         CONFIG_USBHOST_HIDKBD=n

       You could also replace the NSH stdin device to take input from a USB
       keyboard with:

         CONFIG_NSH_USBKBD=y
         CONFIG_NSH_USBKBD_DEVNAME="/dev/kbda"

       The keyboard is currently configured to poll at 80 MSec intervals.
       This is controlled by:

          CONFIG_HIDKBD_POLLUSEC=80000

       which can be reduced if better keyboard response is required.

       NOTE: You will not have access to the RAMLOG via the NSH dmseg command
       if the USB keyboard is selected.  You can still access NSH via Telnet
       or you may want to disable the RAMLOG so that debug information comes
       out on the console.

   16. Support the USB high-speed USB device driver (UDPHS) is not enabled by
       default but could be enabled by changing the NuttX configuration file as
       described above in the section entitled "USB High-Speed Device."

   17. Support for the maXTouch MXT768E touchscreen driver on the TM7000
       LCD/Touchscreen module is enabled by default.  See the section above
       entitled "TM7000 LCD/Touchscreen" for detailed configuration information.
       You will probably want to disable this option if you are not using the
       TM7000 LCD/Touchscreen.

       The Touchscreen test program is also built in.  This test program can
       be found in the source tree at apps/examples/touchscreen.  Usage is
       like:

         nsh> tc [<number-of-touches>]

   18. Support for the TM7000 LCD is enabled by default.  See the section above
       entitled "TM7000 LCD/Touchscreen" for detailed configuration information.
       You will probably want to disable this option if you are not using the
       TM7000 LCD.

       There are several LCD test programs available.  One is built into this
       configuration:  apps/examples/nx.  The NX example is a simple test
       using the NuttX graphics system (NX).  This test case focuses on general
       window controls, movement, mouse and keyboard input.  It requires no
       user interaction.

       The test is executed by simply typing:

         nsh> nx

       There are several simple graphics examples under apps/examples/ that
       could be configured to verify LCD/graphics operation:

         a. nxhello.  Just displays "Hello, World!" at the center of the
            display.
         b. nximage.  Displays the NuttX logo in the center of the display.
         c. nxlines.  Shows many fat lines.  This generally looks like a
            "clock" with a cicle and a rotating line in the center.
         d. nxtext.  This demonstrates scrolling text with pop-up windows on
            top of the test.  The pop-up windows come and go without
            corrupting the scrolling text.

       See apps/examples/README.txt for information about configuring these
       examples.

   19. NxPlayer

       This configuration has the command line NxPlayer enabled.  Support
       for the WM8904 CODEC is built in.

         NOTE:  The WM8904 driver should not be included in the
         configuration if you are using the Rev C version of the board
         (there were some I2C communication issues for the WM8904 interface
         on Rev C of the board):

           CONFIG_SYSTEM_NXPLAYER=n

       This configuration depends on media files in the default mountpoint
       at /mnt/sdard.  You will need to mount the media before running
       NxPlayer,  Here are the general steps to play a file:

         a. You will need an (full size) SD card containing the .WAV files
            that you want to play (.WAV is only format supported as of this
            writing).  That SD card should be inserted in the HSMCI0 media
            slot A (best done before powering up).

         b. If the NuttX auto-mounter is enabled and properly configured,
            then the FAT file system appear at /mnt/sdcard.  If the auto-
            mounter is not enabled, then here are the steps to manually
            mount the FAT file system:

             Then from NSH prompt, you need to mount the media volume like:

              nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard

            NOTE:  The auto-mounter is enabled by default in this
            configuration.

         c. Then you can run the media player like:

              nsh> nxplayer
              nxplayer> device pcm0
              nxplayer> play <filename>

       STATUS:  Not yet functional.  See the To-Do list at the bottom of this
       README file.

   20. The SAMA5D4-EK includes for an AT25 serial DataFlash.  That support is
       NOT enabled in this configuration.  Support for that serial FLASH could
       be enabled by modifying the NuttX configuration as described above in
       the paragraph entitled "AT25 Serial FLASH".

   21. This example can be configured to exercise the watchdog timer test
       (apps/examples/watchdog).  See the detailed configuration settings in
       the section entitled "Watchdog Timer" above.

   STATUS:
       See the To-Do list below

   2014-8-30: Retesting today I am seeing a strange behavior:  Serial
       output is coming out in chunks with delays between the chunks.  It
       appears that something is not good in the serial port configuration.
       I see no such chunky behavior in, for example, graphics output.
   2014-9-17: I do not see that chunked output behavior.  However, I do
       notice that the serial output is sluggish if there is not network
       cable connected.  When the network connected the serial output is
       responsive:  Something must be keeping the system too busy when
       there is not network (probably the network monitor).

  nxwm:

    This is a special configuration setup for the NxWM window manager
    UnitTest.  It integrates support for both the SAMA5 LCDC and the
    SAMA5 ADC touchscreen controller and provides a more advance
    graphics demo. It provides an interactive windowing experience.

    NOTES:

    1. The NxWM window manager is a tiny window manager tailored for use
       with smaller LCDs but which is show here on the larger, SAMA5D4-EK
       TM7000 LCD.  It supports a toolchain, a start window, and
       multiple application windows.  However, to make the best use of
       the visible LCD space, only one application window is visible at
       at time.

        The NxWM window manager can be found here:

          nuttx-git/NxWidgets/nxwm

        The NxWM unit test can be found at:

          nuttx-git/NxWidgets/UnitTests/nxwm

    2. This configuration is set up generally like the nsh configuration
       except that:

       - It boots into a graphic, window manage environment instead of
         the serial console command line.
       - The console command line is still available within NxTerm
         windows.
       - Obviously, the nx and touchscreen built in applications cannot
         be supported.

       Refer to the NOTES for the nsh configuration.  Those also apply
       for the nxwm configuration (other than the differences noted
       above).

    3. NSH Console Access.

       This configuration boots directly into a graphic, window manage
       environment.  There is no serial console.  Some initial stdout
       information will go to the USART3 serial output, but otherwise
       the serial port will be silent.

       Access to the NSH console is available in two ways:

       a. The NxWM provides a graphics-based terminals (called NxTerms);
          The console command line is still available within NxTerm
          windows once NxWM is up and running.  The console input (stdin) is
          provided via a USB HID keyboard, but console output will go to the
          NxTerm terminal.  See below for more information about the USB
          HID keyboard input,

|      b. Telnet NSH sessions are still supported and this is, in general,
          the convenient way to access the shell (and RAMLOG).

       As with the NSH configuration, debug output will still go to the
       circular RAMLOG buffer but cannot be accessed from a serial console.
       Instead, you will need use the dmesg command from an NxTerm or
       from a Telnet session to see the debug output

    4. USB HID Keyboard Input

       USB keyboard support is enabled in the default configuration, but
       can be disabled:

         CONFIG_USBHOST_HIDKBD=y

       Not all keyboards may be supported; only "boot" keyboards will be
       recognized.

       The USB keyboard is configured to replace the NSH stdin device some
       that NSH will take input from the USB keyboard.  This has to be
       done a little differently for the case of NxWM::CNxTerms than
       in the standard NSH configuration.  Here the relevant configuration
       options are:

         CONFIG_NXWM_KEYBOARD_USBHOST=y
         CONFIG_NXWM_KEYBOARD_DEVPATH="/dev/kbda"

       NSH will then automatically start when the NxTerm is started:

         NuttShell (NSH) NuttX-7.3
         nsh>

       When the NxTerm comes up, it will attempt to use /dev/kbda device
       for input.  Obviously, you cannot enter text if there is no keyboard
       but otherwise you will not see any indication whether a keyboard is
       connected or not.

       If the keyboard is detached, you not be able to enter text until the
       keyboard is reconnected.  Again, there is no other special indication
       of the keyboard state.

       The keyboard is currently configured to poll at 80 MSec intervals.
       That might not be fast enough for you if you are a fast typist.  This
       polling rate is controlled by:

          CONFIG_HIDKBD_POLLUSEC=80000

       which can be reduced if better keyboard response is required.

    5. Media Player

       This configuration has the media player application enabled. Support
       for the WM8904 CODEC is built in.

         NOTE:  The WM8904 driver should not be included in the
         configuration if you are using the Rev C version of the board
         (there were some I2C communication issues for the WM8904 interface
         on Rev C of the board).  You may either (1) Disable audio support
         and disable the Media Player GUI, or (2) configure the "NULL" audio
         device so that the GUI will function correctly (with no sound,
         of course).

       This configuration depends on media files in the default mountpoint
       at /mnt/sdard (configurable).  If you see the message "Media volume
       not mounted" in the media player text box, then you will need to
       mount the media volume:

         a. You will need an (full size) SD card containing the .WAV files
            that you want to play (.WAV is only format supported as of this
            writing).  That SD card should be inserted in the HSMCI0 media
            slot A (best done before powering up).

         b. If the NuttX auto-mounter is enabled and properly configured,
            then the FAT file system appear at /mnt/sdcard.  If the auto-
            mounter is not enabled, then you need to perform the following
            steps to manually mount the FAT file system:

             Then from NSH prompt, you need to mount the media volume like:

              nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard

            I usually do this via Telnet from the host PC.  Here is a
            complete host Telnet session:

              $ telnet 10.0.0.2
              Trying 10.0.0.2...
              Connected to 10.0.0.2.
              Escape character is '^]'.

              NuttShell (NSH) NuttX-7.3
              nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard
              nsh> exit
              Connection closed by foreign host.

            NOTE:  The auto-mounter is enabled by default in this
            configuration.

         c. Then if you close the old media player window and bring up a
            new one, you should see the .WAV files on the SD card in the lis
            box.

         STATUS:  Despite the comments above, WM8904 support has *NOT* yet
         been enabled in this configuration.  This is because it is not yet
         working in the nxwm configuration.  See the To-Do list at the
         bottom of this README file.  The current nxwm configuration is still
         set up for the Rev C board using the "NULL" audio device.

       Things still to do:

         a. Currently the list box is not scrollable.  So you will be
            limited to the number .WAV files that will fit in the existing
            list box (a scrollable list box class exists, but has not been
            integrated into the media play demo).

         b. Although the lower level NxPlayer does support them, there are
            no controls at the GUI for balance or tone/equalization.

         c. There is no visual indication of play status or end of playing.

   STATUS:
       See the To-Do list below

  ramtest:

    This is a stripped down version of NSH that runs out of
    internal SRAM.  It configures SDRAM and supports only the RAM test
    at apps/examples/ramtest.  This configuration is useful for
    bringing up SDRAM.

    NOTES:

    1. This configuration uses the USART3 for the serial console
       which is available at the "DBGU" RS-232 connector (J24).  That
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

    3. This configuration executes out of internal SRAM flash and is
       loaded into SRAM by the boot ROM SDRAM from NAND, Serial
       DataFlash, SD card or from a TFTPC sever via the Boot ROM.
       Data also is positioned in SRAM.

       Here are the steps that I use to execute this program in SRAM
       using only the ROM Bootloader:

       a) Hold the DIS_BOOT button and

       b) With the DIS_BOOT button pressed, power cycle the board. A
          reset does not seem to be sufficient.

       c) The serial should show RomBOOT in a terminal window (at 115200
          8N1) and nothing more.

       d) Press ENTER in the terminal window a few times to enable JTAG.

       e) Start the Segger GDB server.  It should successfully connect to
          the board via JTAG (if JTAG was correctly enabled in step d)).

       f) Start GDB, connect, to the GDB server, load NuttX, and debug.

          gdb> target remote localhost:2331
          gdb> mon halt (don't do mon reset)
          gdb> load nuttx
          gdb> mon reg pc (make sure that the PC is 0x200040
          gdb> ... and debug ...

To-Do List
==========

1) Neither USB OHCI nor EHCI support Isochronous endpoints.  Interrupt
   endpoint support in the EHCI driver is untested (but works in similar
   EHCI drivers).

2) HSCMI. CONFIG_MMCSD_MULTIBLOCK_LIMIT=1 is set to disable multi-block
   transfers because of some issues that I saw during testing.  The is very
   low priority to me but might be important to you if you are need very
   high performance SD card accesses.

3) There is a kludge in place in the Ethernet code to work around a problem
   that I see.  The problem that I see is as follows:

     a. To send packets, the software keeps a queue of TX descriptors in
        memory.

     b. When a packet is ready to be sent, the software clears bit 31 of a
        status word in the descriptor meaning that the descriptor now
        "belongs" to the hardware.

     c. The hardware sets bit 31 in memory when the transfer completes.

   The problem that I see is that:

     d. Occasionally bit 31 of the status word is not cleared even though
        the Ethernet packet was successfully sent.

   Since the software does not see bit 31 set, it seems like the transfer
   did not complete and the Ethernet locks up.

   The workaround/kludge that is in place makes this assumption:  If an
   Ethernet transfer complete interrupt is received, then at least one
   packet must have completed.  In this case, the software ignores
   checking the USED bit for one packet.

   With this kludge in place, the driver appears to work fine.  However,
   there is a danger to what I have done:  If a spurious interrupt
   occurs, than the USED bit would not be set and the transfer would be
   lost.

4) Some drivers may require some adjustments if you intend to run from SDRAM.
   That is because in this case macros like BOARD_MCK_FREQUENCY are not constants
   but are instead function calls:  The MCK clock frequency is not known in
   advance but instead has to be calculated from the bootloader PLL configuration.

   As of this writing, all drivers have been converted to run from SDRAM except
   for the PWM and the Timer/Counter drivers.  These drivers use the
   BOARD_MCK_FREQUENCY definition in more complex ways and will require some
   minor redesign and re-testing before they can be available.

5) The WM8904 is not usable on the Rev C version of the board due to some I2C
   related issues.  These issues seem to be resolved on the Rev E version of
   the board.  However, the WM8904 is still not function:

   a) With a logic analyzer I can see that the I2C writes to the WM8904
      device look good.  This is the same setup that was used in the working
      SAMA5D3x-EK nxplayer configuration and so should be correct (you
      cannot even get this far on the Rev C board).
   b) I2C readback of the WM8904 registers (via CONFIG_WM8904_REGDUMP) does
      not, however, show proper registers contents.  Groups of extra bits
      (apparently 0x01fd) appear to be set in many registers on reading.
      This is assumed to be some interference from some other device on the
      I2C bus rather that errors in writing.  This assumption is credible
      since the bad bits appear immediately after resetting the WM8904 and
      before anything has been written to it.
   c) Also with the logic analyzer, I can that the 12MHz MCLK input is
      being provided to the WM8904.
   d) However, no bit clock (BLCK) is being generated by the WM8904.  This
      should appear on both AUDIO_TK0_PB27 and AUDIO_RK0_PB28, but I do not
      see a clock on these pins.
   e) With no BCLK, I would expect the SSC0 DMA transfers to hang... they do
      not.  No errors of any kind are detected by the firmware; it believes
      that it is successfully playing .WAV files.  This leads to believe
      that there may be some schematic error.
   e) There is, of course, no audio output.

   You can replace the WM8904 with the "NULL" audio driver by:

      CONFIG_AUDIO_WM8904=n                         : Disable the WM8904
      CONFIG_SAMA5_SSC0=n                           : Disable SSC0

      CONFIG_AUDIO_NULL=y                           : Enable the NULL audio device
      CONFIG_AUDIO_NULL_BUFFER_SIZE=8192
      CONFIG_AUDIO_NULL_MSG_PRIO=1
      CONFIG_AUDIO_NULL_NUM_BUFFERS=4
      CONFIG_AUDIO_NULL_WORKER_STACKSIZE=768
      CONFIG_AUDIO_NUM_BUFFERS=2

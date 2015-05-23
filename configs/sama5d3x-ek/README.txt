README
======

  This README file describes the port of NuttX to the SAMA5D3x-EK
  development boards. These boards feature the Atmel SAMA5D3
  microprocessors.  Four different SAMA5D3x-EK kits are available

    - SAMA5D31-EK with the ATSAMA5D31 (http://www.atmel.com/devices/sama5d31.aspx)
    - SAMA5D33-EK with the ATSAMA5D33 (http://www.atmel.com/devices/sama5d33.aspx)
    - SAMA5D34-EK with the ATSAMA5D34 (http://www.atmel.com/devices/sama5d34.aspx)
    - SAMA5D35-EK with the ATSAMA5D35 (http://www.atmel.com/devices/sama5d35.aspx)

  The each consist of an identical base board with different plug-in
  modules for each CPU.  I also have a 5 inch LCD for my SAMA5D3x-EK,
  Atmel 5.0_WVGA_R_AEA_DM, Rev. B, but this LCD is not yet generally available
  as of this writing.

  I have seen SAMA5D3x-EK boards with different LCDs attached.  One of the more
  common LCDS is the 7 inch TM0000 TFT LCD controller with MaXTouch multi-touch,
  capacitive touchscreen (http://www.pdaatl.com/doc/tm7000.pdf).  That is NOT the
  LCD supported by this configuration; the LCD used here is smaller and has a
  resistive touchscreen that interfaces via the SAMA5D3 ADC interface.  Refer to
  the SAMA5D4-EK for TM0000 support that can be back-ported to the SAMA5D3x-EK.

    SAMA5D3 Family

                              ATSAMA5D31    ATSAMA5D33    ATSAMA5D34    ATSAMA5D35
    ------------------------- ------------- ------------- ------------- -------------
    Pin Count                 324           324           324           324
    Max. Operating Frequency  536           536           536           536
    CPU                       Cortex-A5     Cortex-A5     Cortex-A5     Cortex-A5
    Max I/O Pins              160           160           160           160
    Ext Interrupts            160           160           160           160
    USB Transceiver           3             3             3             3
    USB Speed                 Hi-Speed      Hi-Speed      Hi-Speed      Hi-Speed
    USB Interface             Host, Device  Host, Device  Host, Device  Host, Device
    SPI                       6             6             6             6
    TWI (I2C)                 3             3             3             3
    UART                      7             5             5             7
    CAN                       -             -             2             2
    LIN                       4             4             4             4
    SSC                       2             2             2             2
    Ethernet                  1             1             1             2
    SD / eMMC                 3             2             3             3
    Graphic LCD               Yes           Yes           Yes           -
    Camera Interface          Yes           Yes           Yes           Yes
    ADC channels              12            12            12            12
    ADC Resolution (bits)     12            12            12            12
    ADC Speed (ksps)          440           440           440           440
    Resistive Touch Screen    Yes           Yes           Yes           Yes
    Crypto Engine             AES/DES/      AES/DES/      AES/DES/      AES/DES/
                              SHA/TRNG      SHA/TRNG      SHA/TRNG      SHA/TRNG
    SRAM (Kbytes)             128           128           128           128
    External Bus Interface    1             1             1             1
    DRAM Memory               DDR2/LPDDR,   DDR2/LPDDR,   DDR2/LPDDR,   DDR2/LPDDR,
                              SDRAM/LPSDR   SDRAM/LPSDR   DDR2/LPDDR,   DDR2/LPDDR,
    NAND Interface            Yes           Yes           Yes           Yes
    Temp. Range (deg C)       -40 to 85     -40 to 85     -40 to 85     -40 to 85
    I/O Supply Class          1.8/3.3       1.8/3.3       1.8/3.3       1.8/3.3
    Operating Voltage (Vcc)   1.08 to 1.32  1.08 to 1.32  1.08 to 1.32  1.08 to 1.32
    FPU                       Yes           Yes           Yes           Yes
    MPU / MMU                 No/Yes        No/Yes        No/Yes        No/Yes
    Timers                    5             5             5             6
    Output Compare channels   6             6             6             6
    Input Capture Channels    6             6             6             6
    PWM Channels              4             4             4             4
    32kHz RTC                 Yes           Yes           Yes           Yes
    Packages                  LFBGA324_A    LFBGA324_A    LFBGA324_A    LFBGA324_A

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - Loading Code into SRAM with J-Link
  - Writing to FLASH using SAM-BA
  - Creating and Using NORBOOT
  - Running NuttX from SDRAM
  - Buttons and LEDs
  - Serial Consoles
  - Networking
  - AT25 Serial FLASH
  - HSMCI Card Slots
  - Auto-Mounter
  - USB Ports
  - USB High-Speed Device
  - USB High-Speed Host
  - SDRAM Support
  - NAND Support
  - AT24 Serial EEPROM
  - I2C Tool
  - CAN Usage
  - SAMA5 ADC Support
  - SAMA5 PWM Support
  - RTC
  - Watchdog Timer
  - TRNG and /dev/random
  - Touchscreen Testing
  - Tickless OS
  - OV2640 Camera Interface
  - I2S Audio Support
  - SAMA5D3x-EK Configuration Options
  - Configurations
  - To-Do List

Development Environment
=======================

  Several possible development environments may be used:

  - Linux or OSX native
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

    CONFIG_ARMV7A_TOOLCHAIN_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_ARMV7A_TOOLCHAIN_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_ARMV7A_TOOLCHAIN_ATOLLIC=y        : Atollic toolchain for Windos
    CONFIG_ARMV7A_TOOLCHAIN_DEVKITARM=y      : devkitARM under Windows
    CONFIG_ARMV7A_TOOLCHAIN_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_ARMV7A_TOOLCHAIN_GNU_EABIL=y      : Generic GCC ARM EABI toolchain for Linux
    CONFIG_ARMV7A_TOOLCHAIN_GNU_EABIW=y      : Generic GCC ARM EABI toolchain for Windows

  The CodeSourcery GCC toolchain is selected with
  CONFIG_ARMV7A_TOOLCHAIN_CODESOURCERYW=y and setting the PATH variable
  appropriately.

  NOTE about Windows native toolchains
  ------------------------------------

  There are several limitations to using a Windows based toolchain in a
  Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath'
     utility but you might easily find some new path problems.  If so, check
     out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic
     links are used in Nuttx (e.g., include/arch).  The make system works
     around these problems for the Windows tools by copying directories
     instead of linking them.  But this can also cause some confusion for
     you:  For example, you may edit a file in a "linked" directory and find
     that your changes had no effect.  That is because you are building the
     copy of the file in the "fake" symbolic directory.  If you use a\
     Windows toolchain, you should get in the habit of making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows paths which do not
     work with the Cygwin make.

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

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
  3) Set up include pathes:  You will need include/, arch/arm/src/sam34,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/sam34/sam_vectors.S.  You may need to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by an IDE.

NuttX EABI "buildroot" Toolchain
================================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh sama5d3x-ek/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5.  Copy the configuration file from the configs/ sub-directory to the
      top-level build directory:

      cp configs/cortexa8-eabi-defconfig-4.8.2 .config

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

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  details PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

NXFLAT Toolchain
================

  If you are *not* using the NuttX buildroot toolchain and you want to use
  the NXFLAT tools, then you will still have to build a portion of the buildroot
  tools -- just the NXFLAT tools.  The buildroot with the NXFLAT tools can
  be downloaded from the NuttX SourceForge download site
  (https://sourceforge.net/projects/nuttx/files/).

  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh sama5d3x-ek/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-nxflat .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built NXFLAT binaries.

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
    2. You have the USB connected to DBGU port (J14)
    3. Terminal configuration:  115200 8N1

  Using SAM-BA to write to FLASH:

    1. Exit the terminal emulation program and remove the USB cable from
       the DBGU port (J14)
    2. Connect the USB cable to the device USB port (J20)
    3. JP9 must open (BMS == 1) to boot from on-chip Boot ROM.
    4. Press and maintain PB4 CS_BOOT button and power up the board.  PB4
       CS_BOOT button prevents booting from Nand or serial Flash by
       disabling Flash Chip Selects after having powered the board, you can
       release the PB4 BS_BOOT button.
    5. On Windows you may need to wait for a device driver to be installed.
    6. Start the SAM-BA application, selecting (1) the correct USB serial
       port, and (2) board = at91sama5d3x-ek.
    7. The SAM-BA menu should appear.
    8. Select the FLASH bank that you want to use and the address to write
       to and "Execute"
    9. When you are finished writing to FLASH, remove the USB cable from J20
       and re-connect the serial link on USB CDC / DBGU connector (J14) and
       re-open the terminal emulator program.
    10. If you loaded code in NOR flash (CS0), then you will need to close
        JP9 (BMS == 0) to force booting out of NOR flash (see NOTE).
    11. Power cycle the board.

  NOTES:  By closing JP9 (BMS == 0), you can force the board to boot
  directly to NOR FLASH.  Executing from other memories will require that
  you provide a special code header so that you code can be recognized as a
  boot-able image by the ROM bootloader.

Creating and Using NORBOOT
==========================

  In order to have more control of debugging code that runs out of NOR FLASH,
  I created the sama5d3x-ek/norboot configuration.  That configuration is
  described below under "Configurations."

  Here are some general instructions on how to build an use norboot:

  Building:
  1. Remove any old configurations (if applicable).

       cd <nuttx>
       make distclean

  2. Install and build the norboot configuration.  This steps will establish
     the norboot configuration and setup the PATH variable in order to do
     the build:

       cd tools
       ./configure.sh sama5d3x-ek/<subdir>
       cd -
       . ./setenv.sh

     Before sourcing the setenv.sh file above, you should examine it and
     perform edits as necessary so that TOOLCHAIN_BIN is the correct path
     to the directory than holds your toolchain binaries.

     NOTE:  Be aware that the default norboot also disables the watchdog.
     Since you will not be able to re-enable the watchdog later, you may
     need to set CONFIG_SAMA5_WDT=y in the NuttX configuration file.

     Then make norboot:

       make

     This will result in an ELF binary called 'nuttx' and also HEX and
     binary versions called 'nuttx.hex' and 'nuttx.bin'.

  3. Rename the binaries.  Since you will need two versions of NuttX:  this
     norboot version that runs in internal SRAM and another under test in
     NOR FLASH, I rename the resulting binary files so that they can be
     distinguished:

       mv nuttx norboot
       mv nuttx.hex norboot.hex
       mv nuttx.bin norboot.bin

  4. Build your NOR configuration and write this into NOR FLASH.  Here, for
     example, is how you would create the NSH NOR configuration:

       cd <nuttx>
       make distclean                 # Remove the norboot configuration
       cd tools
       ./configure.sh sama5d3x-ek/nsh # Establish the NSH configuration
       cd -
       make                           # Build the NSH configuration

     Then use SAM-BA to write the nuttx.bin binary into NOR FLASH.  This
     will involve holding the CS_BOOT button and power cycling to start
     the ROM loader.  The SAM-BA serial connection will be on the device
     USB port, not the debug USB port.  Follow the SAM-BA instruction to
     write the nuttx.bin binary to NOR FLASH.

   5. Restart the system without holding CS_BOOT to get back to the normal
      debug setup.

   6. Then start the J-Link GDB server and GDB.  In GDB, I do the following:

       (gdb) mon reset                # Reset and halt the CPU
       (gdb) load norboot             # Load norboot into internal SRAM
       (gdb) mon go                   # Start norboot
       (gdb) mon halt                 # Break in
       (gdb) mon reg pc = 0x10000040  # Set the PC to NOR flash entry point
       (gdb) mon go                   # And jump into NOR flash

      The norboot program can also be configured to jump directly into
      NOR FLASH without requiring the final halt and go by setting
      CONFIG_SAMA5D3xEK_NOR_START=y in the NuttX configuration.  However,
      since I have been debugging the early boot sequence, the above
      sequence has been most convenient for me since it allows me to
      step into the program in NOR.

   7. An option is to use the SAM-BA tool to write the NORBOOT image into
      Serial FLASH.  Then, the system will boot from Serial FLASH by
      copying the NORBOOT image in SRAM which will run and then start the
      image in NOR FLASH automatically.  This is a very convenient usage!

      NOTES: (1) There is jumper on the CM module that must be closed to
      enable use of the AT25 Serial Flash.  (2) If using SAM-BA, make sure
      that you load the NOR boot program into the boot area via the pull-
      down menu.

    STATUS:
      2013-7-30:  I have been unable to execute these configurations from NOR
        FLASH by closing the BMS jumper (J9).  As far as I can tell, this
        jumper does nothing on my board???  So I have been using the norboot
        configuration exclusively to start the program-under-test in NOR FLASH.

Running NuttX from SDRAM
========================

  Executing from SDRAM
  --------------------

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

DRAMBOOT
--------

  See also configs/sama5d4-ek/README.txt for a description of the DRAMBOOT
  program.  This is a tiny version of NuttX that can run out of internal
  SRAM.  If you put this program on the HSMCI1 microSD card as boot.bin, then
  it will boot on power up and you can download NuttX directly into DRAM by
  sending the nuttx.hex file over the serial connection.

NAND FLASH Memory Map
---------------------

  Reference: http://www.at91.com/linux4sam/bin/view/Linux4SAM/GettingStarted

  0x0000:0000 - 0x0003:ffff: AT91BootStrap
  0x0004:0000 - 0x000b:ffff: U-Boot
  0x000c:0000 - 0x000f:ffff: U-Boot environment
  0x0010:0000 - 0x0017:ffff: U-Boot environement redundant
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

      ftp://www.at91.com/pub/at91bootstrap/AT91Bootstrap3.6.0/sama5d3xek-nandflashboot-uboot-3.6.0.bin

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

     A pre-built binary is available here:

     ftp://www.at91.com/pub/uboot/u-boot-v2012.10/u-boot-sama5d3xek_nandflash_linux4sam_4.2.bin

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

  You should now be able to interrupt with U-Boot vie the DBGU interface.

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

Buttons and LEDs
================

  Buttons
  -------
  There are five push button switches on the SAMA5D3X-EK base board:

    1. One Reset, board reset (BP1)
    2. One Wake up, push button to bring the processor out of low power mode
      (BP2)
    3. One User momentary Push Button
    4. One Disable CS Push Button

  Only the momentary push button is controllable by software (labeled
  "PB_USER1" on the board):

    - PE27.  Pressing the switch connects PE27 to grounded.  Therefore, PE27
      must be pulled high internally.  When the button is pressed the SAMA5
      will sense "0" is on PE27.

  LEDs
  ----
  There are two LEDs on the SAMA5D3 series-CM board that can be controlled
  by software.  A  blue LED is controlled via PIO pins.  A red LED normally
  provides an indication that power is supplied to the board but can also
  be controlled via software.

    PE25.  This blue LED is pulled high and is illuminated by pulling PE25
    low.

    PE24.  The red LED is also pulled high but is driven by a transistor so
    that it is illuminated when power is applied even if PE24 is not
    configured as an output.  If PE24 is configured as an output, then the
    LED is illuminated by a high output.

      N.B. PE24 Drives the RED Led on the CM (SODIMM200), but unfortunately
      it is also connected to ISI_RST on the MB (Main Board) and controlling
      it will reset a Camera connected to the ISI

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
  events as follows when the red LED (PE24) is available:

    SYMBOL                Meaning                     LED state
                                                    Blue     Red
    -------------------  -----------------------  -------- --------
    LED_STARTED          NuttX has been started     OFF      OFF
    LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
    LED_IRQSENABLED      Interrupts enabled         OFF      OFF
    LED_STACKCREATED     Idle stack created         ON       OFF
    LED_INIRQ            In an interrupt            -- No change --
    LED_SIGNAL           In a signal handler        -- No change --
    LED_ASSERTION        An assertion failed        -- No change --
    LED_PANIC            The system has crashed     OFF      Blinking
    LED_IDLE             MCU is is sleep mode       -- Not used  --

  If CONFIG_SAMA5D3xEK_NOREDLED=y, then the red LED is not used by the
  system and the controls are as follows:

    SYMBOL                Meaning                     LED state
                                                    Blue        Red
    -------------------  -----------------------  ----------- -----------
    LED_STARTED          NuttX has been started     OFF       Not used
    LED_HEAPALLOCATE     Heap has been allocated    OFF       " " "  "
    LED_IRQSENABLED      Interrupts enabled         OFF       " " "  "
    LED_STACKCREATED     Idle stack created         ON        " " "  "
    LED_INIRQ            In an interrupt            No change " " "  "
    LED_SIGNAL           In a signal handler        No change " " "  "
    LED_ASSERTION        An assertion failed        No change " " "  "
    LED_PANIC            The system has crashed     Blinking  " " "  "
    LED_IDLE             MCU is is sleep mode       Not used  " " "  "

  Thus if the blue LED is statically on, NuttX has successfully booted and
  is, apparently, running normally.  If the red (or blue) LED is flashing
  at approximately 2Hz, then a fatal error has been detected and the system
  has halted.

Serial Consoles
===============

  USART1
  ------
  By default USART1 is used as the NuttX serial console in all
  configurations (unless otherwise noted).  USART1 is buffered with an
  RS-232 Transceiver (Analog Devices ADM3312EARU) and connected to the DB-9
  male socket (J8).

    USART1 Connector J8
    -------------------------------
    SAMA5 FUNCTION  NUTTX PIO
    PIO   NAME      CONFIGURATION
    ---- ---------- ---------------
    PB27 RTS1       PIO_USART1_RTS
    PB29 TXD1       PIO_USART1_TXD
    PB28 RXD1       PIO_USART1_RXD
    PB26 CTS1       PIO_USART1_CTS

    NOTE: Debug TX (DTXD) and RX (DRXD) pins also are routed to the
    ADM3312EARU via non populated 0 Ohm resistors. Thus allowing one
    skilled with a soldering iron to choose which UART is level
    translated by the ADM3312EARU

    -------------------------------
    SAMA5 FUNCTION  NUTTX PIO
    PIO   NAME      CONFIGURATION
    ---- ---------- ---------------
    PB31 DTXD       PIO_DBGU_DTXD
    PB30 DRXD       PIO_DBGU_DRXD

  Hardware UART via CDC
  ---------------------
  "J-Link-OB-ATSAM3U4C comes with an additional hardware UART that is
   accessible from a host via CDC which allows terminal communication with
   the target device. This feature is enabled only if a certain port (CDC
   disabled, PA25, pin 24 on J-Link-OB-ATSAM3U4C) is NOT connected to ground
   (open).

    - Jumper JP16 not fitted: CDC is enabled
    - Jumper JP16 fitted : CDC is disabled"

Networking
==========

  Networking support via the can be added to NSH by selecting the following
  configuration options.  The SAMA5D3x supports two different Ethernet MAC
  peripherals:  (1) The 10/100Base-T EMAC peripheral and (2) the
  10/100/1000Base-T GMAC peripheral.  Only the SAMA5D31 and SAMAD35 support
  the EMAC peripheral; Only the SAMA5D33, SAMA5D34, and SAMA5D35 support
  the GMAC perpheral!  NOTE that the SAMA5D35 supports both!

  Selecting the EMAC peripheral
  -----------------------------

  System Type
    CONFIG_ARCH_CHIP_ATSAMA5D31=y        : SAMA5D31 or SAMAD35 support EMAC
    CONFIG_ARCH_CHIP_ATSAMA5D35=y        : (others do not)

  System Type -> SAMA5 Peripheral Support
    CONFIG_SAMA5_EMACA=y                  : Enable the EMAC (type A) peripheral

  System Type -> EMAC device driver options
    CONFIG_SAMA5_EMAC_NRXBUFFERS=16      : Set aside some RS and TX buffers
    CONFIG_SAMA5_EMAC_NTXBUFFERS=4
    CONFIG_SAMA5_EMAC_PHYADDR=1          : KSZ8021/31 PHY is at address 1
    CONFIG_SAMA5_EMAC_AUTONEG=y          : Use autonegotiation
    CONFIG_SAMA5_EMAC_RMII=y             : Either MII or RMII interface should work
    CONFIG_SAMA5_EMAC_PHYSR=30           : Address of PHY status register on KSZ8021/31
    CONFIG_SAMA5_EMAC_PHYSR_ALTCONFIG=y  : Needed for KSZ8021/31
    CONFIG_SAMA5_EMAC_PHYSR_ALTMODE=0x7  : "    " " " "     "
    CONFIG_SAMA5_EMAC_PHYSR_10HD=0x1     : "    " " " "     "
    CONFIG_SAMA5_EMAC_PHYSR_100HD=0x2    : "    " " " "     "
    CONFIG_SAMA5_EMAC_PHYSR_10FD=0x5     : "    " " " "     "
    CONFIG_SAMA5_EMAC_PHYSR_100FD=0x6    : "    " " " "     "

  PHY selection.  Later in the configuration steps, you will need to select
  the KSZ8021/31 PHY for EMAC (See below)

  Selecting the GMAC peripheral
  -----------------------------

  System Type
    CONFIG_ARCH_CHIP_ATSAMA5D33=y        : SAMA5D31, SAMA5D33 and SAMAD35
    CONFIG_ARCH_CHIP_ATSAMA5D34=y        : support GMAC (others do not)
    CONFIG_ARCH_CHIP_ATSAMA5D35=y        :

  System Type -> SAMA5 Peripheral Support
    CONFIG_SAMA5_GMAC=y                  : Enable the GMAC peripheral

  System Type -> GMAC device driver options
    CONFIG_SAMA5_GMAC_NRXBUFFERS=16      : Set aside some RS and TX buffers
    CONFIG_SAMA5_GMAC_NTXBUFFERS=4
    CONFIG_SAMA5_GMAC_PHYADDR=1          : KSZ8051 PHY is at address 1
    CONFIG_SAMA5_GMAC_AUTONEG=y          : Use autonegotiation

  If both EMAC and GMAC are selected, you will also need:

    CONFIG_SAMA5_GMAC_ISETH0=y           : GMAC is "eth0"; EMAC is "eth1"

  PHY selection.  Later in the configuration steps, you will need to select
  the  KSZ9021/31 PHY for GMAC (See below)

  Common configuration settings
  -----------------------------

  Networking Support
    CONFIG_NET=y                         : Enable Networking
    CONFIG_NET_SOCKOPTS=y                : Enable socket operations
    CONFIG_NET_ETH_MTU=562               : Maximum packet size (MTU) 1518 is more standard
    CONFIG_NET_ETH_TCP_RECVWNDO=562      : Should be the same as CONFIG_NET_ETH_MTU
    CONFIG_NET_TCP=y                     : Enable TCP/IP networking
    CONFIG_NET_TCPBACKLOG=y              : Support TCP/IP backlog
    CONFIG_NET_TCP_READAHEAD_BUFSIZE=562 : Read-ahead buffer size
    CONFIG_NET_UDP=y                     : Enable UDP networking
    CONFIG_NET_ICMP=y                    : Enable ICMP networking
    CONFIG_NET_ICMP_PING=y               : Needed for NSH ping command
                                         : Defaults should be okay for other options
  Device drivers -> Network Device/PHY Support
    CONFIG_NETDEVICES=y                  : Enabled PHY selection
    CONFIG_ETH0_PHY_KSZ8051=y            : Select the KSZ8051 PHY (for EMAC), OR
    CONFIG_ETH0_PHY_KSZ90x1=y            : Select the KSZ9021/31 PHY (for GMAC)

  Application Configuration -> Network Utilities
    CONFIG_NETUTILS_DNSCLIENT=y          : Enable host address resolution
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

  By default, the IP address of the SAMA5D3x-EK will be 10.0.0.2 and
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

  On the host side, you should also be able to ping the SAMA5D3x-EK:

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
      df          hexdump     mb          mv          sh

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
      nuttx/configs/sama5d3x-ek/src/sam_ethernet.c.

    - And a few other things: UDP support is required (CONFIG_NET_UDP) and
      signals must not be disabled (CONFIG_DISABLE_SIGNALS).

  Given those prerequisites, the newtork monitor can be selected with these additional settings.

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

  Both the Ronetix and Embest versions of the SAMAD3x CPU modules include an
  Atmel AT25DF321A, 32-megabit, 2.7-volt SPI serial flash.  The SPI
  connection is as follows:

    AT25DF321A      SAMA5
    --------------- -----------------------------------------------
    SI              PD11 SPI0_MOSI
    SO              PD10 SPI0_MIS0
    SCK             PD12 SPI0_SPCK
    /CS             PD13 via NL17SZ126 if JP1 is closed (See below)

  JP1 and JP2 seem to related to /CS on the Ronetix board, but the usage is
  less clear.  For the Embest module, JP1 must be closed to connect /CS to
  PD13; on the Ronetix schematic, JP11 seems only to bypass a resistor (may
  not be populated?).  I think closing JP1 is correct in either case.

  Configuration
  -------------

  The Embest or Ronetix CPU module includes an Atmel AT25DF321A, 32-megabit,
  2.7-volt SPI serial flash.  Support for that serial FLASH can be enabled
  in these configurations.  These are the relevant  configuration settings:

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
      CONFIG_SAMA5D3xEK_AT25_BLOCKMOUNT=y   : Mounts AT25 for NSH
      CONFIG_SAMA5D3xEK_AT25_FTL=y          : Create block driver for FAT

  NOTE that you must close JP1 on the Embest/Ronetix board in order to
  enable the AT25 FLASH chip select.

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

  NOTE:  It appears that if Linux runs out of NAND, it will destroy the
  contents of the AT25.

HSMCI Card Slots
================

  Physical Slots
  --------------

  The SAMA5D3x-EK provides a two SD memory card slots:  (1) a full size SD
  card slot (J7 labelled MCI0), and (2) a microSD memory card slot (J6
  labelled MCI1).

  The full size SD card slot connects via HSMCI0.  The card detect discrete
  is available on PD17 (pulled high).  The write protect discrete is tied to
  ground (via PP6) and not available to software.  The slot supports 8-bit
  wide transfer mode, but the NuttX driver currently uses only the 4-bit
  wide transfer mode

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

  Enabling HSMCI support. The SAMA5D3x-EK provides a two SD memory card
  slots:  (1) a full size SD card slot (J7 labelled MCI0), and (2) a
  microSD memory card slot (J6 labelled MCI1).  The full size SD card slot
  connects via HSMCI0; the microSD connects vi HSMCI1.  Support for both SD
  slots can be enabled with the following settings:

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
      CONFIG_MMCSD_MULTIBLOCK_DISABLE=y     : (REVISIT)
      CONFIG_MMCSD_HAVECARDDETECT=y         : Supports card-detect PIOs
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
  board-level support.  See configs/sama5d4-ek for and example of how
  you might do this.

  WARNING:  SD cards should never be removed without first unmounting
  them.  This is to avoid data and possible corruption of the file
  system.  Certainly this is the case if you are writing to the SD card
  at the time of the removal.  If you use the SD card for read-only access,
  however, then I cannot think of any reason why removing the card without
  mounting would be harmful.

USB Ports
=========

  The SAMA5D3 series-MB features three USB communication ports:

    * Port A Host High Speed (EHCI) and Full Speed (OHCI) multiplexed with
      USB Device High Speed Micro AB connector, J20

    * Port B Host High Speed (EHCI) and Full Speed (OHCI) standard type A
      connector, J19 upper port

    * Port C Host Full Speed (OHCI) only standard type A connector, J19
      lower port

  All three USB host ports are equipped with 500 mA high-side power switch
  for self-powered and bus powered applications. The USB device port feature
  VBUS inserts detection function.

  Port A
  ------

    PIO  Signal Name Function
    ---- ----------- -------------------------------------------------------
    PD29  VBUS_SENSE VBus detection
    PD25  EN5V_USBA  VBus power enable (via MN15 AIC1526 Dual USB High-Side
                     Power Switch.  The other channel of the switch is for
                     the LCD)

  Port B
  ------

    PIO  Signal Name Function
    ---- ----------- -------------------------------------------------------
    PD26 EN5V_USBB   VBus power enable (via MN14 AIC1526 Dual USB High-Side
                     Power Switch).  To the A1 pin of J19 Dual USB A
                     connector

  Port C
  ------

    PIO  Signal Name Function
    ---- ----------- -------------------------------------------------------
    PD27 EN5V_USBC   VBus power enable (via MN14 AIC1526 Dual USB High-Side
                     Power Switch).  To the B1 pin of J19 Dual USB A
                     connector

  Both Ports B and C
  ------------------

    PIO  Signal Name Function
    ---- ----------- -------------------------------------------------------
    PD28 OVCUR_USB   Combined overrcurrent indication from port A and B

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
  CONFIG_DEBUG + CONFIG_DEBUG_USB.  However, USB device operation is very
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
      CONFIG_SYSTEM_USBMONITOR=y              : Enable the USB monitor daemon
      CONFIG_SYSTEM_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
      CONFIG_SYSTEM_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
      CONFIG_SYSTEM_USBMONITOR_INTERVAL=1     : Dump trace data every second
      CONFIG_SYSTEM_USBMONITOR_TRACEINIT=y    : Enable TRACE output
      CONFIG_SYSTEM_USBMONITOR_TRACECLASS=y
      CONFIG_SYSTEM_USBMONITOR_TRACETRANSFERS=y
      CONFIG_SYSTEM_USBMONITOR_TRACECONTROLLER=y
      CONFIG_SYSTEM_USBMONITOR_TRACEINTERRUPTS=y

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
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

  USB Hub Support
  ----------------

  USB hub support can be included by adding the following changes to the configuration (in addition to those listed above):

    Drivers -> USB Host Driver Support
      CONFIG_USBHOST_HUB=y                 : Enable the hub class
      CONFIG_USBHOST_ASYNCH=y              : Asynchonous I/O supported needed for hubs

    System Type -> USB High Speed Host driver options
      CONFIG_SAMA5_OHCI_NEDS=12            : You will probably want more pipes
      CONFIG_SAMA5_OHCI_NTDS=18
      CONFIG_SAMA5_OHCI_TDBUFFERS=12
      CONFIG_SAMA5_OHCI_TDBUFSIZE=128

    Board Selection ->
      CONFIG_SAMA5D3XEK_USBHOST_STACKSIZE=2048 (bigger than it needs to be)

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
      Hub support has not been verified on this board yet.

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
  CONFIG_DEBUG + CONFIG_DEBUG_USB.  However, USB host operation is very time
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
      CONFIG_SYSTEM_USBMONITOR=y              : Enable the USB monitor daemon
      CONFIG_SYSTEM_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
      CONFIG_SYSTEM_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
      CONFIG_SYSTEM_USBMONITOR_INTERVAL=1     : Dump trace data every second

  NOTE: If USB debug output is also enabled, both outpus will appear on the
  serial console.  However, the debug output will be asynchronous with the
  trace output and, hence, difficult to interpret.

NOR FLASH Support
=================

  Most of these configurations execute out of CS0 NOR flash and can only be
  loaded via SAM-BA.  These are the relevant configuration options the
  define the NOR FLASH configuration:

    CONFIG_SAMA5_BOOT_CS0FLASH=y            : Boot from FLASH on CS0
    CONFIG_BOOT_RUNFROMFLASH=y              : Run in place on FLASH (vs copying to RAM)

    CONFIG_SAMA5_EBICS0=y                   : Enable CS0 external memory
    CONFIG_SAMA5_EBICS0_SIZE=134217728      : Memory size is 128KB
    CONFIG_SAMA5_EBICS0_NOR=y               : Memory type is NOR FLASH

    CONFIG_FLASH_START=0x10000000           : Physical FLASH start address
    CONFIG_FLASH_VSTART=0x10000000          : Virtual FLASH start address
    CONFIG_FLASH_SIZE=134217728             : FLASH size (again)

    CONFIG_RAM_START=0x00300400             : Data stored after page table
    CONFIG_RAM_VSTART=0x00300400
    CONFIG_RAM_SIZE=114688                  : Available size of 128KB - 16KB for page table

  NOTE:  In order to boot in this configuration, you need to close the BMS
  jumper.

  STATUS:  I have been unable to execute these configurations from NOR FLASH
  by closing the BMS jumper (J9).  As far as I can tell, this jumper does
  nothing on my board???  So I have been using the norboot configuration
  exclusively to start the program-under-test in NOR FLASH (see the section
  entitled "Creating and Using NORBOOT" above.)

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

    Board Selection
      CONFIG_SAMA5D3xEK_MT47H128M16RT=y     : This is the type of DDR2

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

    Board Selection
      CONFIG_SAMA5D3xEK_MT47H128M16RT=y     : This is the type of DDR2

    System Type->Heap Configuration
      CONFIG_SAMA5_ISRAM_HEAP=n             : These do not apply in this case
      CONFIG_SAMA5_DDRCS_HEAP=n

    System Type->Boot Memory Configuration
      CONFIG_RAM_START=0x20000000           : Physical address of SDRAM
      CONFIG_RAM_VSTART=0x20000000          : Virtual address of SDRAM
      CONFIG_RAM_SIZE=268435456             : Size of SDRAM
      CONFIG_BOOT_SDRAM_DATA=y              : Data is in SDRAM

      Care must be used applied these RAM locations; the graphics
      configurations use SDRAM in an incompatible way to set aside
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

    2. Booting from Serial Flash. The work around for this case is to put
       the NORBOOT image into Serial FLASH.  Then, the system will boot from
       Serial FLASH by copying the NORBOOT image in SRAM which will run and
       then start the image in NOR FLASH.  See the discussion of the NORBOOT
       configuration in the "Creating and Using NORBOOT" section above.

       NOTE that there is jumper on the CM module that must be closed to enable
       use of the AT25 Serial Flash.  Also, if you are using using SAM-BA,
       make sure that you load the NOR boot program into the boot area via
       the pull-down menu.

    3. Unfortunately, there are no appropriate NAND file system in NuttX as
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
      CONFIG_SAMA5D3XEK_NAND_BLOCKMOUNT=y : Enable FS support on NAND
      CONFIG_SAMA5D3xEK_NAND_NXFFS=y      : Use the NXFFS file system

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
      CONFIG_SAMA5D3XEK_NAND_BLOCKMOUNT=y : Enable FS support on NAND
      CONFIG_SAMA5D3xEK_NAND_FTL=y        : Use an flash translation layer

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

    With the options CONFIG_SAMA5D3XEK_NAND_BLOCKMOUNT=y and
    CONFIG_SAMA5D3xEK_NAND_NXFFS=y, the NAND FLASH will be mounted in the NSH
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

AT24 Serial EEPROM
==================

  AT24 Connections
  ----------------

  A AT24C512 Serial EEPPROM was used for tested I2C.  There are other I2C/TWI
  devices on-board, but the serial EEPROM is the simplest test.

  There is, however, no AT24 EEPROM on board the SAMA5D3x-EK:  The Serial
  EEPROM was mounted on an external adaptor board and connected to the
  SAMA5D3x-EK thusly:

    - VCC -- VCC
    - GND -- GND
    - TWCK0(PA31) -- SCL
    - TWD0(PA30)  -- SDA

  By default, PA30 and PA31 are SWJ-DP pins, it can be used as a pin for TWI
  peripheral in the end application.

  Configuration Settings
  ----------------------

  The following configuration settings were used:

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_TWI0=y                   : Enable TWI0

    System Type -> TWI device driver options
      SAMA5_TWI0_FREQUENCY=100000           : Select a TWI frequency

    Device Drivers -> I2C Driver Support
      CONFIG_I2C=y                          : Enable I2C support
      CONFIG_I2C_TRANSFER=y                 : Driver supports the transfer() method
      CONFIG_I2C_WRITEREAD=y                : Driver supports the writeread() method

    Device Drivers -> Memory Technology Device (MTD) Support
      CONFIG_MTD=y                          : Enable MTD support
      CONFIG_MTD_AT24XX=y                   : Enable the AT24 driver
      CONFIG_AT24XX_SIZE=512                : Specifies the AT 24C512 part
      CONFIG_AT24XX_ADDR=0x53               : AT24 I2C address

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

    File systems
      CONFIG_NXFFS=y                        : Enables the NXFFS file system
      CONFIG_NXFFS_PREALLOCATED=y           : Required
                                            : Other defaults are probably OK

    Board Selection
      CONFIG_SAMA5D3xEK_AT24_BLOCKMOUNT=y   : Mounts AT24 for NSH
      CONFIG_SAMA5D3xEK_AT24_NXFFS=y        : Mount the AT24 using NXFFS

  You can then format the AT24 EEPROM for a FAT file system and mount the
  file system at /mnt/at24 using these NSH commands:

    nsh> mkfatfs /dev/mtdblock0
    nsh> mount -t vfat /dev/mtdblock0 /mnt/at24

  Then you an use the FLASH as a normal FAT file system:

    nsh> echo "This is a test" >/mnt/at24/atest.txt
    nsh> ls -l /mnt/at24
    /mnt/at24:
     -rw-rw-rw-      16 atest.txt
    nsh> cat /mnt/at24/atest.txt
    This is a test

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
      CONFIG_I2C_TRANSFER=y                 : Driver supports the transfer() method
      CONFIG_I2C_WRITEREAD=y                : Driver supports the writeread() method

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
      List busses   : bus
      List devices  : dev [OPTIONS] <first> <last>
      Read register : get [OPTIONS] [<repititions>]
      Show help     : help
      Write register: set [OPTIONS] <value> [<repititions>]
      Verify access : verf [OPTIONS] [<value>] [<repititions>]

    Where common "sticky" OPTIONS include:
      [-a addr] is the I2C device address (hex).  Default: 03 Current: 03
      [-b bus] is the I2C bus number (decimal).  Default: 0 Current: 0
      [-r regaddr] is the I2C device register address (hex).  Default: 00 Current: 00
      [-w width] is the data width (8 or 16 decimal).  Default: 8 Current: 8
      [-s|n], send/don't send start between command and data.  Default: -n Current: -n
      [-i|j], Auto increment|don't increment regaddr on repititions.  Default: NO Current: NO
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
  will be accepted.  For example, if the mask is all ones, then only messasges
  with exact address matches will be accepted; if the mask is all zeroes than
  any address will be accepted.

  CAN connectors
  --------------

  CAN1 and CAN2 are available via RJ-11 connectors on the SAMA5D3x-EK.  Each
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
  not much motivation to enable ADC support on the SAMA5D3x-EK.  This
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
  there is not much motivation to enable PWM support on the SAMA5D3x-EK.  This
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

    See configs/sama5d3x-ek/include/board.h for all of the default PWM
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

  NOTE:  If you are using the norboot program to run from FLASH as I did,
  beware that the default version also disables the watchdog.  You will
  need a special version of norboot with CONFIG_SAMA5_WDT=y.

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

Touchscreen Testing
===================

  You can enable the touchscreen by modifying the configuration in the
  following ways:

    System Type:
      CONFIG_SAMA5_ADC=y                  : ADC support is required
      CONFIG_SAMA5_TSD=y                  : Enabled touchcreen device support
      SAMA5_TSD_4WIRE=y                   : 4-Wire interface with pressure

  You might want to tinker with the SWAPXY and THRESHX and THRESHY settings
  to get the result that you want.

    Drivers:
      CONFIG_INPUT=y                      : (automatically selected)

    Board Selection:
       CONFIG_SAMA5D3xEK_TSD_DEVMINOR=0   : Register as /dev/input0

    Library Support:
      CONFIG_SCHED_WORKQUEUE=y            : Work queue support required

  These options may also be applied to enable a built-in touchscreen test
  application:

    Application Configuration:
      CONFIG_EXAMPLES_TOUCHSCREEN=y       : Enable the touchscreen built-int test
      CONFIG_EXAMPLES_TOUCHSCREEN_MINOR=0 : To match the board selection
      CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH="/dev/input0"

  Defaults should be okay for all related settings.

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

  NOTE: In most cases, the slow clock will be used as the timer/counter
  input.  You should enable the 32.768KHz crystal for the slow clock by
  calling sam_sckc_enable().  Otherwise, you will be doing all system
  timing using the RC clock!  UPDATE: This will now be selected by default
  when you configure for TICKLESS support.

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
  from 0xffffffff to zero).  So we could potentially used the just set the compare
  at the number of ticks you want PLUS the current value of timer.  Then
  you could have both with a single timer:  An interval timer and a free-
  running counter with the same timer!  In this case, you would want to
  to set CONFIG_SCHED_TICKLESS_ALARM in the NuttX configuration.

  Patches are welcome!

OV2640 Camera Interface
=======================

    SAMA5D3x PIN             SAMA5D3x-EK    OV2640
    PIO  PER SIGNAL        ISI Socket J11
    ---- --- ------------- --- ------------ ----------------------------------------
    ---                     1  VDDISI       ---
    ---                     2  GND          ---
    ---                     3  VDDISI       ---
    ---                     4  GND          ---
    PE28  ?  ?              5  ZB_SLPTR     ???
    PE29  ?  ?              6  ZB_RST       C6 RESETB Reset mode (?)
    PC27  B  TWI1_CK        7  TWCK1        C2 SIO_C SCCB serial interface clock input
    PC26  B  TWI1_D         8  TWD1         C1 SIO_D SCCB serial interface data I/O
    ---                     9  GND          ---
    PD31  B  PCK1 (ISI_MCK) 10 ISI_MCK      C4 XVCLK System clock input (?)
    ---                     11 GND          ---
    PA30  C  ISI_VSYNC      12 ISI_VSYNC    D2 VSYNC Vertical synchronization
    ---                     13 GND          ---
    PA31  C  ISI_HSYNC      14 ISI_HSYNC    C3 HREF Horizontal reference output (?)
    ---                     15 GND          ---
    PC30  C  ISI_PCK        16 ISI_PCK      E3 PCLK Pixel clock output
    ---                     17 GND          ---
    PA16  C  ISI_D0         18 ISI_D0       E2 Y0 Video port output bit[0]
    PA17  C  ISI_D1         19 ISI_D1       E1 Y1 Video port output bit[1]
    PA18  C  ISI_D2         20 ISI_D2       F3 Y2 Video port output bit[2]
    PA19  C  ISI_D3         21 ISI_D3       G3 Y3 Video port output bit[3]
    PA20  C  ISI_D4         22 ISI_D4       F4 Y4 Video port output bit[4]
    PA21  C  ISI_D5         23 ISI_D5       G4 Y5 Video port output bit[5]
    PA22  C  ISI_D6         24 ISI_D6       E5 Y6 Video port output bit[6]
    PA23  C  ISI_D7         25 ISI_D7       G5 Y7 Video port output bit[7]
    PC29  C  ISI_D8         26 ISI_D8       F5 Y8 Video port output bit[8]
    PC28  C  ISI_D9         27 ISI_D9       G6 Y9 Video port output bit[9]
    PC27  C  ISI_D10        28 ISI_D10      ---
    PC26  C  ISI_D11        29 ISI_D11      ---
    ---                     30 GND          ---

    ???                     ??              A2 EXPST_B Snapshot exposure start trigger
    ???                     ??              A6 STROBE  Flash control output
    ???                     ??              B2 FREX    Snapshot trigger
    ???                     ??              B6 PWDN    Power-down mode enable

I2S Audio Support
=================

  The SAMA5D3x-EK has two devices on-board that can be used for verification
  of I2S functionality:  HDMI and a WM8904 audio CODEC.  As of this writing,
  the I2S driver is present, but there are not drivers for either the HDMI
  or the WM8904.

  WM8904 Audio CODEC Interface
  ----------------------------

    ------------- ---------------- ----------------- ----------------------
    WM8904        SAMA5D3          NuttX Pin Name    External Access
    ------------- ---------------- ----------------- ----------------------
     3 SDA        PA30 TWD0        PIO_TWI0_D        J1 Pin 34
     2 SCLK       PA31 TWCK0       PIO_TWI0_CK       J1 Pin 36
    28 MCLK       PD30 PCK0        PIO_PMC_PCK0      (Not available)
    29 BCLK/GPIO4 PC16 TK          PIO_SSC0_TK       J2 Pin 6
    "" "        " PC19 RK          PIO_SSC0_RK       J2 Pin 12
    30 LRCLK      PC17 TF          PIO_SSC0_TF       J2 Pin 8
    "" "   "      PC20 RF          PIO_SSC0_RF       J2 Pin 14
    31 ADCDAT     PC21 RD          PIO_SSC0_RD       J2 Pin 16
    32 DACDAT     PC18 TD          PIO_SSC0_TD       J2 Pin 10
     1 IRQ/GPIO1  PD16 INT_AUDIO   N/A               (Not available)
    ------------- ---------------- ----------------- ----------------------
                                                     Ground at Pins 3,4,37,38

  WM8904 Configuration
  --------------------

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_DMAC0=y                  : DMAC0 required by SSC0
      CONFIG_SAMA5_TWI0=y                   : Enable TWI0 driver support
      CONFIG_SAMA5_SSCO=y                   : Enable SSC0 driver support

    System Type -> SSC0 Configuration
      CONFIG_SAMA5_SSC_MAXINFLIGHT=16
      CONFIG_SAMA5_SSC0_DATALEN=16

    Device Drivers -> I2C Driver Support
      CONFIG_I2C=y                          : Enable I2C support
      CONFIG_I2C_EXCHANGE=y                 : Support the exchange method
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

    Audio
      CONFIG_AUDIO=y                        : Audio support needed
      CONFIG_AUDIO_FORMAT_PCM=y             : Only PCM files are supported
      CONFIG_AUDIO_NUM_BUFFERS=8            : Number of audio buffers
      CONFIG_AUDIO_BUFFER_NUMBYTES=8192     : Audio buffer size

    Drivers -> Audio
      CONFIG_I2S=y                          : General I2S support
      CONFIG_AUDIO_DEVICES=y                : Audio device support
      CONFIG_AUDIO_WM8904=y                 : Build WM8904 driver character driver

    Board Selection
      CONFIG_SAMA5D3xEK_WM8904_I2CFREQUENCY=400000
      CONFIG_SAMA5D3xEK_WM8904_SRCMAIN=y    : WM8904 MCLK is the SAMA5D Main Clock

    Library Routines
      CONFIG_SCHED_WORKQUEUE=y              : MW8904 driver needs work queue support

  The NxPlayer
  ------------

  The NxPlayer is a audio library and command line application for playing
  audio file.  The NxPlayer can be found at apps/system/nxplayer.  If you
  would like to add the NxPlayer, here are some recommended configuration
  settings.

  First of all, the NxPlayer depends on the NuttX audio subsystem.  See the
  "WM8904 Configuration" above for an example of how the audio subsystem is
  configured to use the WM8904 CODED with PCM decoding.

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
    CONFIG_NXPLAYER_DEFAULT_MEDIADIR="/music"    : See nxplayer configuration
    CONFIG_NXPLAYER_RECURSIVE_MEDIA_SEARCH=y     : Search all sub-directories
    CONFIG_NXPLAYER_INCLUDE_SYSTEM_RESET=y       : Add support for reset command

  You must include the full path to the location where NxPlayer can find the
  media files.  That path is given by CONFIG_NXPLAYER_DEFAULT_MEDIADIR.
  Here I use the example "/mnt/scard".  That is a location where you could,
  for example, mount an MMC/SD card driver.

  I2S Loopback Test
  -----------------

  The I2S driver was verified using a special I2C character driver (at
  nuttx/drivers/audio/i2schar.c) and a test driver at apps/examples/i2schar.
  The I2S driver was verified in loopback mode with no audio device.

  This section describes the modifications to the NSH configuration that were
  used to perform the I2S testing:

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_SSCO=y              : Enable SSC0 driver support
      CONFIG_SAMA5_DMAC0=y             : DMAC0 required by SSC0

    Alternatively, SSC1 could have be used:

    System Type -> SAMA5 Peripheral Support
      CONFIG_SAMA5_SSC1=y              : Enable SSC0 driver support
      CONFIG_SAMA5_DMAC0=y             : DMAC0 required by SSC0

    System Type -> SSC Configuration
      CONFIG_SAMA5_SSC_MAXINFLIGHT=16  : Up to 16 pending DMA transfers
      CONFIG_SAMA5_SSC0_MASTER=y       : Master mode
      CONFIG_SAMA5_SSC0_DATALEN=16     : 16-bit data
      CONFIG_SAMA5_SSC0_RX=y           : Support a receiver
      CONFIG_SAMA5_SSC0_RX_RKINPUT=y   : Receiver gets clock from RK input
      CONFIG_SAMA5_SSC0_RX_FSLEN=2     : Pick some matching frame synch length
      CONFIG_SAMA5_SSC0_TX=y           : Support a transmitter
      CONFIG_SAMA5_SSC0_TX_MCKDIV=y    : Transmitter gets clock from MCK/2
      CONFIG_SAMA5_SSC0_TX_FSLEN=2     : Pick some matching frame synch length
      CONFIG_SAMA5_SSC0_MCKDIV_SAMPLERATE=48000 : Sampling at 48K samples/sec
      CONFIG_SAMA5_SSC0_TX_TKOUTPUT_XFR=y  : Outputs clock on TK when transferring data
      CONFIG_SAMA5_SSC0_LOOPBACK=y     : Loopmode mode connects RD/TD and RK/TK

    Audio
      CONFIG_AUDIO=y                   : Audio support needed
                                       : Defaults should be okay

    Drivers -> Audio
      CONFIG_I2S=y                     : General I2S support
      CONFIG_AUDIO_DEVICES=y           : Audio device support
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
      CONFIG_SAMA5D3xEK_I2SCHAR_MINOR=0
      CONFIG_SAMA5D3xEK_SSC_PORT=0      : 0 or SSC0, 1 for SSC1

    Library Routines
      CONFIG_SCHED_WORKQUEUE=y          : Driver needs work queue support

SAMA5D3x-EK Configuration Options
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

  CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
  hence, the board that supports the particular chip or SoC.

    CONFIG_ARCH_BOARD="sama5d3x-ek" (for the SAMA5D3x-EK development board)

  CONFIG_ARCH_BOARD_name - For use in C code

    CONFIG_ARCH_BOARD_SAMA5D3X_EK=y

  CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
  of delay loops

  CONFIG_ENDIAN_BIG - define if big endian (default is little
  endian)

  CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

    CONFIG_RAM_SIZE=0x0002000 (128Kb)

  CONFIG_RAM_START - The physical start address of installed DRAM

    CONFIG_RAM_START=0x20000000

  CONFIG_RAM_VSTART - The virutal start address of installed DRAM

    CONFIG_RAM_VSTART=0x20000000

  CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
  have LEDs

  CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
  stack. If defined, this symbol is the size of the interrupt
  stack in bytes.  If not defined, the user task stacks will be
  used during interrupt handling.

  CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

  CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

  CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
  cause a 100 second delay during boot-up.  This 100 second delay
  serves no purpose other than it allows you to calibrate
  CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
  the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
  the delay actually is 100 seconds.

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
    CONFIG_SAMA5_EMACA       - Ethernet MAC (Type A)
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

    CONFIG_USART0_ISUART     - USART0 is configured as a UART
    CONFIG_USART1_ISUART     - USART1 is configured as a UART
    CONFIG_USART2_ISUART     - USART2 is configured as a UART
    CONFIG_USART3_ISUART     - USART3 is configured as a UART

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
           m (m=4,5) for the console and ttys0 (default is the USART1).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.
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
  Each SAMA5D3x-EK configuration is maintained in a sub-directory and
  can be selected as follow:

    cd tools
    ./configure.sh sama5d3x-ek/<subdir>
    cd -
    . ./setenv.sh

  Before sourcing the setenv.sh file above, you should examine it and perform
  edits as necessary so that TOOLCHAIN_BIN is the correct path to the directory
  than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       and misc/tools/

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on USART1 (J8).

  3. All of these configurations use the Code Sourcery for Windows toolchain
     (unless stated otherwise in the description of the configuration).  That
     toolchain selection can easily be reconfigured using 'make menuconfig'.
     Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Microsoft Windows
       CONFIG_WINDOWS_CYGWIN=y             : Using Cygwin or other POSIX environment

     System Type -> Toolchain:
       CONFIG_ARMV7A_TOOLCHAIN_GNU_EABIW=y : GNU EABI toolchain for windows

     That same configuration will work with Atmel GCC toolchain.  The only
     change required to use the Atmel GCC toolchain is to change the PATH
     variable so that those tools are selected instead of the CodeSourcery
     tools.  Try 'which arm-none-eabi-gcc' to make sure that you are
     selecting the right tool.

     The setenv.sh file is available for you to use to set the PATH
     variable.  The path in the that file may not, however, be correct
     for your installation.

     See also the "NOTE about Windows native toolchains" in the section call
     "GNU Toolchain Options" above.

     !!!WARNING!!! The first time that you type 'make', the system will
     configure itself based on the settings in the .config file.  One of
     these settings can cause a lot of confusion if you configure the build
     in the wrong state:  If you are running on Linux, make *certain* that
     you have CONFIG_HOST_LINUX=y *before* the first make or you will
     create a very corrupt configuration that may not be easy to recover
     from.

  4. The SAMA5Dx is running at 396MHz by default in these configurations.
     This is because the original timing for the PLLs, NOR FLASH, and SDRAM
     came from the Atmel NoOS sample code which runs at that rate.

     The SAMA5Dx is capable of running at 528MHz, however, and is easily
     re-configured:

       Board Selection -> CPU Frequency
         CONFIG_SAMA5D3xEK_396MHZ=n     # Disable 396MHz operation
         CONFIG_SAMA5D3xEK_528MHZ=y     # Enable 528MHz operation

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

  5. By default, all of these configurations run from ISRAM or NOR FLASH
     (as indicated below in each description of the configuration).
     Operation from SDRAM is also an option as described in the paragraph
     entitled, "Running NuttX from SDRAM."

  Configuration Sub-directories
  -----------------------------
  Summary:  Some of the descriptions below are long and wordy. Here is the
  concise summary of the available SAMA5D3x-EK configurations:

    demo: This is an NSH configuration that supports as much functionality
      as possible.  That is why it gets its name:  It attempts to show as
      much as possible
    hello:  The tiniest configuration possible (almost).  It just says
      "Hello, World!"  On the serial console.  It is so tiny that it is
      able to run entirely out of internal SRAM (all of the other
      configurations except norboot use NOR FLASH for .text and internal
      SRAM for .data and .bass).  This configuration is only useful for
      bring-up.
    norboot:
      This is a little program to help debug of code in NOR flash.  I wrote
      it because I don't yet understand how to get the SAMA5 to boot from
      NOR FLASH.  See the description below and the section above entitled
      "Creating and Using NORBOOT" for more information
    nsh:  This is another NSH configuration, not too different from the
      demo configuration.  The nsh configuration is, however, bare bones.
      It is the simplest possible NSH configuration and is useful as a
      platform for debugging and integrating new features in isolation.
    nx: A simple test using the NuttX graphics system (NX) that has been
      used to verify the SAMA5D3x-EK TFT LCD.  This test case focuses on
      general window controls, movement, mouse and keyboard input.  It
      requires no user interaction.
    nxplayer:  A command line media player using the on-board WM8904 audio
      CODEC.
    nxwm: This is a special configuration setup for the NxWM window manager
      UnitTest.  It integrates support for both the SAMA5 LCDC and the
      SAMA5 ADC touchscreen controller and provides a more advance
      graphics demo. It provides an interactive windowing experience.
    ov2640:  A test of the SAMA5 ISI using an OV2640 camera.  INCOMPLETE!

  There may be issues with some of these configurations.  See the details
  before of the status of individual configurations.

  Now for the gory details:

  demo:

    This configuration directory provide the NuttShell (NSH).  There are
    two NSH configurations:  nsh and demo.  The difference is that nsh is
    intended to be a very simple NSH configuration upon which you can build
    further functionality.  The demo configuration, on the other hand, is
    intended to be a rich configuration that shows many features all working
    together.

    See also the NOTES associated with the nsh configuration for other hints
    about features that can be included with this configuration.

    NOTES:
    1. This configuration uses the default USART1 serial console.  That
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
       CONFIG_ARMV7A_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

       If you are running on Linux, make *certain* that you have
       CONFIG_HOST_LINUX=y *before* the first make or you will create a
       corrupt configuration that may not be easy to recover from. See
       the warning in the section "Information Common to All Configurations"
       for further information.

    3. This configuration executes out of CS0 NOR flash and can only
       be loaded via SAM-BA.  The are the relevant configuration options
       are provided above in the section entitled "NOR FLASH Support".

    4. Data resides in ISRAM, but can be moved to SDRAM as described above
       under "SDRAM Data Configuration."

    The following features are pre-enabled in the demo configuration, but not
    in the nsh configuration:

    5. SDRAM is supported.  .data and .bss is still retained in ISRAM, but
       SDRAM is initializeed and the SDRAM memory is included in the heap.
       Relevant configuration settings are provided in the paragraph entitled
       "SDRAM Support" above.

    6. The Real Time Clock/Calendar (RTC) is enabled.  See the section entitled
       "RTC" above.

    7. The Embest or Ronetix CPU module includes an Atmel AT25DF321A,
       32-megabit, 2.7-volt SPI serial flash.  Support for that serial
       FLASH can is enabled in this configuration.  See the paragraph
       entitle "AT25 Serial FLASH" for detailed configuration settings.

    8. Support for HSMCI car slots. The SAMA5D3x-EK provides a two SD memory
       card slots:  (1) a full size SD card slot (J7 labelled MCI0), and (2)
       a microSD memory card slot (J6 labelled MCI1).  The full size SD card
       slot connects via HSMCI0; the microSD connects vi HSMCI1.  Relevant
       configuration settings can be found in the section entitled "HSMCI
       Card Slots" above.

    9. Support the USB high-speed device (UDPHS) driver is enabled.  See the
       section above entitled "USB High-Speed Device" for relevant NuttX
       configuration settings.

    10. The USB high-speed EHCI and the low-/full- OHCI host drivers are supported
        in this configuration.  See the section above entitle "USB High-Speed Host"
        for relevant configuration information.

    11. Support SAMA5D3 TRNG peripheral is enabled so that it provides
        /dev/random.  See the section entitled "TRNG and /dev/random"
        above for detailed configuration information.

    STATUS:
       See the To-Do list below

       2014-3-30: I some casual retesting, I am seeing some slow boot-
                  up times and possible microSD card issues.  I will
                  need to revisit this.
  hello:

    This configuration directory, performs the (almost) simplest of all
    possible examples:  examples/hello.  This just comes up, says hello
    on the serial console and terminates.  This configuration is of
    value during bring-up because it is small and can run entirely out
    of internal SRAM.

    NOTES:
    1. This configuration uses the default USART1 serial console.  That
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
       CONFIG_ARMV7A_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

       If you are running on Linux, make *certain* that you have
       CONFIG_HOST_LINUX=y *before* the first make or you will create a
       corrupt configuration that may not be easy to recover from. See
       the warning in the section "Information Common to All Configurations"
       for further information.

    3. This configuration executes out of internal SRAM and can only
       be loaded via JTAG.

       CONFIG_SAMA5_BOOT_ISRAM=y               : Boot into internal SRAM
       CONFIG_BOOT_RUNFROMISRAM=y              : Run from internal SRAM

    STATUS:
       See the To-Do list below

  norboot:
    This is a little program to help debug of code in NOR flash.  It
    does the following:

    - It enables and configures NOR FLASH, then
    - Waits for you to break in with GDB.

    At that point, you can set the PC and begin executing from NOR FLASH
    under debug control.  See the section entitled "Creating and Using
    NORBOOT" above.

    NOTES:

    1. This program derives from the hello configuration.  All of the
       notes there apply to this configuration as well.

    2. The default norboot program initializes the NOR memory,
       displays a message and halts.  The norboot program can also be
       configured to jump directly into NOR FLASH without requiring the
       final halt and go by setting CONFIG_SAMA5D3xEK_NOR_START=y in the
       NuttX configuration.

    3. Be aware that the default norboot also disables the watchdog.
       Since you will not be able to re-enable the watchdog later, you may
       need to set CONFIG_SAMA5_WDT=y in the NuttX configuration file.

    4. If you put norboot on the Serial FLASH, you can automatically
       boot to NOR on reset.  See the section "Creating and Using NORBOOT"
       above.

    STATUS:
       See the To-Do list below

  nsh:

    This configuration directory provide the NuttShell (NSH).  There are
    two NSH configurations:  nsh and demo.  The difference is that nsh is
    intended to be a very simple NSH configuration upon which you can build
    further functionality.  The demo configuration, on the other hand, is
    intended to be a rich configuration that shows many features all working
    together.

    NOTES:
    1. This configuration uses the default USART1 serial console.  That
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
       CONFIG_ARMV7A_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

       If you are running on Linux, make *certain* that you have
       CONFIG_HOST_LINUX=y *before* the first make or you will create a
       corrupt configuration that may not be easy to recover from. See
       the warning in the section "Information Common to All Configurations"
       for further information.

    3. This configuration executes out of CS0 NOR flash and can only
       be loaded via SAM-BA.  The are the relevant configuration options
       are provided above in the section entitled "NOR FLASH Support".

    4. This configuration has support for NSH built-in applications enabled.
       However, no built-in applications are selected in the base configuration.

    5. Data resides in ISRAM, but can be moved to SDRAM as described above
       under "SDRAM Data Configuration."

    6. This configuration has support for the FAT file system built in.  However,
       by default, there are no block drivers initialized.  The FAT file system can
       still be used to create RAM disks.

    7. SDRAM support can be enabled by modifying your NuttX configuration as
       described above in the paragraph entitle "SDRAM Support"

    8. The Embest or Ronetix CPU module includes an Atmel AT25DF321A,
       32-megabit, 2.7-volt SPI serial flash.  Support for that serial
       FLASH can be enabled by modifying the NuttX configuration as
       described above in the paragraph entitled "AT25 Serial FLASH".

    9. Enabling HSMCI support. The SAMA5D3x-EK provides a two SD memory card
       slots:  (1) a full size SD card slot (J7 labeled MCI0), and (2) a
       microSD memory card slot (J6 labeled MCI1).  The full size SD card
       slot connects via HSMCI0; the microSD connects vi HSMCI1.  Support
       for both SD slots can be enabled with the settings provided in the
       paragraph entitled "HSMCI Card Slots" above.

    10. Support the USB low-, high- and full-speed OHCI host driver can be enabled
        by changing the NuttX configuration file as described in the section
        entitled "USB High-Speed Host" above.

    11. Support the USB high-speed USB device driver (UDPHS) can be enabled
        by changing the NuttX configuration file as described above in the
        section entitled "USB High-Speed Device."

    12. AT24 Serial EEPROM. A AT24C512 Serial EEPPROM was used for tested
        I2C.  There is, however, no AT24 EEPROM on board the SAMA5D3x-EK:
        The  serial EEPROM was mounted on an external adaptor board and
        connected to the SAMA5D3x-EK thusly.  See the section above entitle
        "AT24 Serial EEPROM" for further information.

    13. I2C Tool. NuttX supports an I2C tool at apps/system/i2c that can be
        used to peek and poke I2C devices.  See the discussion above under
        "I2C Tool" for detailed configuration settings.

    14. Networking support via the can be added to NSH by modifying the
        configuration.  See the "Networking" section above for detailed
        configuration settings.

    15. You can enable the touchscreen and a touchscreen by following the
        configuration instrcutions in the section entitled "Touchscreen
        Testing" above.

    16. The Real Time Clock/Calendar (RTC) may be enabled by reconfiguring NuttX.
        See the section entitled "RTC" above for detailed configuration settings.

    17. This example can be configured to exercise the watchdog timer test
        (apps/examples/watchdog).  See the detailed configuration settings in
        the section entitled "Watchdog Timer" above.

    18. This example can be configured to enable the SAMA5 TRNG peripheral so
        that it provides /dev/random.  See the section entitled "TRNG and
        /dev/random" above for detailed configuration information.

    19. See also the sections above for additional configuration options:
        "AT24 Serial EEPROM", "CAN Usage", "SAMA5 ADC Support", "SAMA5 PWM
        Support", "OV2640 Camera Interface", "I2S Audio Support"

    STATUS:
       See the To-Do list below

      I2C
      2013-9-12:  I have been unsuccessful getting the external serial
        AT24 EEPROM to work.  I am pretty sure that this is a problem with
        my external AT24 board (the TWI0 bus hangs when the AT24 is plugged
        in).  I will skip the AT24 integration since it is not on the critical
        path at the moment.
      2013-9-12:  The I2C tool, however, seems to work well.  It succesfully
        enumerates the devices on the bus and successfully exchanges a few
        commands.  The real test of the come later when a real I2C device is
        integrated.

  nx:

    A simple test using the NuttX graphics system (NX) that has been used to
    verify the SAMA5D3x-EK TFT LCD.  This test case focuses on general
    window controls, movement, mouse and keyboard input.  It requires no
    user interaction.

  nxplayer

    A command line media player using the on-board WM8904 audio CODEC.
    This configuration is based on the nsh configuration above with the
    following extensions:

      a. It runs at 528MHz
      b. It includes SDRAM support
      c. Support for the WM8904 audio CODEC is enabled along with
         support for TWI0, SSC0, and DMAC0 needed by the SM8904.
      d. Support for the full size SD card slot (HSMCI0) is enable
      e. The NxPlayer command line media player is built in.

    NOTES:

    1. See the NOTEs for the nsh configuration.  Since this configuration
       derives from that configuration, all notes apply.

    2. Using NxPlayer

       This configuration depends on media files in the default mountpoint
       at /music.  You will need to mount the media before running
       NxPlayer,  Here are the general steps to play a file:

         a. You will need an (full size) SD card containing the .WAV files
            that you want to play (.WAV is only format supported as of this
            writing).  That SD card should be inserted in the HSMCI0 media
            slot A (best done before powering up).

         b. If the NuttX auto-mounter is enabled and properly configured,
            then the FAT file system appear at /music.  If the auto-
            mounter is not enabled, then here are the steps to manually
            mount the FAT file system:

             Then from NSH prompt, you need to mount the media volume like:

              nsh> mount -t vfat /dev/mmcsd0 /music

            NOTE:  There is an auto-mounter that could be used to eliminate
            this step.  The auto mounter is not enabled or integrated into
            in this configuration, however.  See the section entitle
            "Auto-Mounter " above.

         c. You can then see the available .wav files like:

               nsh>ls /music

         d. Then you can run the media player like:

               nsh> nxplayer
               nxplayer> device pcm0
               nxplayer> play <filename>

            where <filename> is name or path of the .WAV file to be playerd.

  nxwm:

    This is a special configuration setup for the NxWM window manager
    UnitTest.  It integrates support for both the SAMA5 LCDC and the
    SAMA5 ADC touchscreen controller and provides a more advance
    graphics demo. It provides an interactive windowing experience.

    The NxWM window manager is a tiny window manager tailored for use
    with smaller LCDs.  It supports a taskbar, a start window, and
    multiple application windows with toolbars.  However, to make the
    best use of the visible LCD space, only one application window is
    visible at at time.

    The NxWM window manager can be found here:

      nuttx-git/NxWidgets/nxwm

    The NxWM unit test can be found at:

      nuttx-git/NxWidgets/UnitTests/nxwm

    Documentation for installing the NxWM unit test can be found here:

      nuttx-git/NxWidgets/UnitTests/README.txt

    Here is the quick summary of the build steps.  These steps assume that
    you have the entire NuttX GIT in some directory ~/nuttx-git.  You may
    have these components installed elsewhere.  In that case, you will need
    to adjust all of the paths in the following accordingly:

    1. Install the nxwm configuration

       $ cd ~/nuttx-git/nuttx/tools
       $ ./configure.sh sama5d3x-ek/nxwm

    2. Make the build context (only)

       $ cd ..
       $ . ./setenv.sh
       $ make context
       ...

       NOTE: the use of the setenv.sh file is optional.  All that it will
       do is to adjust your PATH variable so that the build system can find
       your tools.  If you use it, you will most likely need to modify the
       script so that it has the correct path to your tool binaries
       directory.

    3. Install the nxwm unit test

       $ cd ~/nuttx-git/NxWidgets
       $ tools/install.sh ~/nuttx-git/apps nxwm
       Creating symbolic link
        - To ~/nuttx-git/NxWidgets/UnitTests/nxwm
        - At ~/nuttx-git/apps/external

    4. Build the NxWidgets library

       $ cd ~/nuttx-git/NxWidgets/libnxwidgets
       $ make TOPDIR=~/nuttx-git/nuttx
       ...

    5. Build the NxWM library

       $ cd ~/nuttx-git/NxWidgets/nxwm
       $ make TOPDIR=~/nuttx-git/nuttx
       ...

    6. Built NuttX with the installed unit test as the application

       $ cd ~/nuttx-git/nuttx
       $ make

    STATUS:
    See the To-Do list below

    2013-10-18.  This example kind of works, but there are still far too
    many outstanding issues:

    a) It runs of the SAMA5D31 and SAMA5D34, but not on the SAMA5D33.  This
       board is from a different manufacturer and there may be some SDRAM-
       related issues?
    b) There may be an SDRAM noise issue on the SAMA5D31 and SAMA5D34.
       I suspect that the SDRAM setup is non-optimal.  The symptom is that
       writing into frame buffer (in SDRAM) occasionally corrupts the DMA
       descriptors (also in SDRAM)  When the bad DMA descriptors are
       fetched, the channel shuts down and the display goes black.  This
       problem could also be cause by a bad write outside of the framebuffer
       and, in fact, putting a guard band around the framebuffers seems to
       eliminate the problem.
    c) There are some occasional start up issues.  It appears that the LCDC
       is programmed incorrectly and groups of pixels in the images are
       reversed (producing an odd serrated look to the images).
       Update:  I corrected a similar problem on the SAMA5D4-EK by
       increasing the SCLK from MCK to 2*MCK.  That eliminated all start up
       problems with the SAMA5D4-EK and needs to be tried on the SAMA5D3e-EK
       as well.  This is controlled by an LCD setting in include/board.h.
    d) I think that there may be more issues if GRAPHICS and INPUT debug is
       off.  I have not tested with DEBUG off.
    e) The biggest problem is the touchscreen accuracy.  The touchscreen
       seems stable during calibration, but the first thing that this
       example requires is a touch in the far, far, upper left corner of
       the display.  In that region, I cannot get reliable touch measurements
       and so I cannot get past the opening display.

    Bottom line:  Not ready for prime time.

    2014-9-20:  Trying to verify the build today, there are now compilation
    errors in the ADC/Touchscreen driver.  See STATUS at the end of this file

  ov2640:

    A test of the SAMA5 ISI using an OV2640 camera.

To-Do List
==========

1) Most of these configurations execute from NOR FLASH. I have been unable
   to execute these configurations from NOR FLASH by closing the BMS jumper
   (J9).  As far as I can tell, this jumper does nothing on my board???  I
   have been using the norboot configuration to start the program in NOR
   FLASH (see just above).  See "Creating and Using NORBOOT" above.

   UPDATE: It has been confirmed at that there is an issue with the BMS
   jumper on my board. However, other NuttX users have confirmed operation
   booting directly into NOR FLASH.  So although I cannot confirm this
   behavior, this appears to be no longer an issue.

2) Neither USB OHCI nor EHCI support Isochronous endpoints.  Interrupt
   endpoint support in the EHCI driver is untested (but works in similar
   EHCI drivers).

3) HSCMI. CONFIG_MMCSD_MULTIBLOCK_DISABLE=y is set to disable multi-block
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

4) I believe that there is an issue when the internal AT25 FLASH is
   formatted by NuttX.  That format works fine with Linux, but does not
   appear to work with Windows.  Reformatting on Windows can resolve this.
   NOTE:  This is not a SAMA5Dx issue.

   UPDATE: Two important bugs were recently fixed in the NuttX FAT
   formatting function (mkfatfs).  It is likely that these fixes will
   eliminate this issue, but that has not yet been verified.

5) CAN testing has not yet been performed due to issues with cabling.  I
   just do not have a good test bed (or sufficient CAN knowledge) for
   good CAN testing.

6) There are lots of LCDC hardware features that are not tested with NuttX.
   The simple NuttX graphics system does not have support for all of the
   layers and other features of the LCDC.

7) I have a Camera, but there is still no ISI driver.  I am not sure what to
   do with the camera.  NuttX needs something like V4L to provide the
   definition for what a camera driver is supposed to do.

   I will probably develop a test harness for ISI, but it is of only
   minimal value with no OS infrastructure to deal with images and video.

8) GMAC has only been tested on a 10/100Base-T network.  I don't have a
   1000Base-T network to support additional testing.

9) Some drivers may require some adjustments if you intend to run from SDRAM.
   That is because in this case macros like BOARD_MCK_FREQUENCY are not constants
   but are instead function calls:  The MCK clock frequency is not known in
   advance but instead has to be calculated from the bootloader PLL configuration.

   As of this writing, all drivers have been converted to run from SDRAM except
   for the PWM and the Timer/Counter drivers.  These drivers use the
   BOARD_MCK_FREQUENCY definition in more complex ways and will require some
   minor redesign and re-testing before they can be available.

10) 2014-9-20: Failed to build the NxWM configuration:

   CC:  chip/sam_adc.c
   chip/sam_adc.c: In function 'sam_adc_interrupt':
   chip/sam_adc.c:886:21: warning: unused variable 'priv' [-Wunused-variable]
      struct sam_adc_s *priv = &g_adcpriv;
                        ^
   chip/sam_adc.c: In function 'sam_adc_initialize':
   chip/sam_adc.c:1977:7: error: 'g_adcdev' undeclared (first use in this function)
          g_adcdev.ad_ops  = &g_adcops;
          ^
   chip/sam_adc.c:1977:7: note: each undeclared identifier is reported only once for each function it appears in
   chip/sam_adc.c:1977:27: error: 'g_adcops' undeclared (first use in this function)
          g_adcdev.ad_ops  = &g_adcops;
                              ^
   chip/sam_adc.c:1983:11: error: 'struct sam_adc_s' has no member named 'dev'
          priv->dev = &g_adcdev;
              ^
   chip/sam_adc.c:2090:1: warning: control reaches end of non-void function [-Wreturn-type]
    }
    ^

   The failure occurs because there are no ADC channels configured (as there
   should not be) so SAMA5_ADC_HAVE_CHANNELS is not defined (as it should
   not be).  However, if there are no configured ADC channel, then
   sam_adc_initialize() does not compile correctly -- and it should not
   given nature the logic that is in place there now.

   A quick glance at the history of these files does not reveal what the
   obvious solution is so I will need to come back and revisit this in the
   future.

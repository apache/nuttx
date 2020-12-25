README
======

  This README file discusses the port of NuttX to the Olimex LPC-H3131 board.

  NOTE:  This is a minimal port to the Olimex LPC-H3131.  According to Olimex
  documentation, the LPC-H3131 is similar in design to the Embedded Artists
  EA3131.  As a consequence, it should be possible to leverage additional
  functionality from boards/arm/lpc31xx/ea3131 without too much difficulty.

Contents
========

  o Development Environment
  o GNU Toolchain Options
  o IDEs
  o NuttX buildroot Toolchain
  o Boot Sequence
  o Buttons and LEDs
  o Image Format
  o Image Download to ISRAM
  o Using OpenOCD and GDB
  o ARM/LPC-H3131-specific Configuration Options
  o Configurations

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems.

GNU Toolchain Options
=====================

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The NuttX buildroot Toolchain (see below), or
  2. Any generic arm-none-eabi GNU toolchain.

  All testing has been conducted using the NuttX buildroot toolchain.  To use
  a different toolchain, you simply need to modify the configuration.  As an
  example:

    CONFIG_ARM_TOOLCHAIN_GNU_EABIL : Generic arm-none-eabi toolchain

  Generic arm-none-eabi GNU Toolchain
  -----------------------------------
  There are a number of toolchain projects providing support for ARMv4/v5
  class processors, including:

    GCC ARM Embedded
      https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

  Others exist for various Linux distributions, MacPorts, etc.  Any version
  based on GCC 4.6.3 or later should work.

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
  3) Set up include paths:  You will need include/, arch/arm/src/lpc31xx,
     arch/arm/src/common, arch/arm/src/arm, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/lpc31xx/lpc31_vectors.S.  You may have to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by an IDE.

NuttX buildroot Toolchain
=========================

  A GNU GCC-based toolchain is assumed.  The PATH environment variable should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  Bitbucket download site (https://bitbucket.org/nuttx/buildroot/downloads/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured NuttX in <some-dir>/nuttx.

     tools/configure.sh olimex-lpc-h3131:<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp boards/arm926t-defconfig-4.2.4 .config

  6. make oldconfig

  7. make

  8. Make sure that the PATH variable includes the path to the newly built
     binaries.

  See the file boards/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

Boot Sequence
=============

  LPC313x has on chip bootrom which loads properly formatted images from multiple
  sources into SRAM.  These sources include including SPI Flash, NOR Flash, UART,
  USB, SD Card, and NAND Flash.

  In all configurations, NuttX is loaded directly into ISRAM.  NuttX is linked
  to execute from ISRAM, regardless of the boot source.

Buttons and LEDs
================

  Buttons
  -------

  There are no user buttons on the H3131

  LEDs
  ----

  There are two LEDs on the LPC-H3131 that can be controlled by software:

      LED              GPIO
      ---------------- -----
      LED1 Yellow      GPIO17 High output illuminates
      LED2 Green       GPIO18 High output illuminates

  Both can be illuminated by driving the GPIO output to high.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/lpc31_leds.c. The LEDs are used to encode
  OS-related events as follows:

    SYMBOL                Meaning                     LED state
                                                    LED2     LED1
    -------------------  -----------------------  -------- --------
    LED_STARTED          NuttX has been started     OFF      OFF
    LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
    LED_IRQSENABLED      Interrupts enabled         OFF      OFF
    LED_STACKCREATED     Idle stack created         ON       OFF
    LED_INIRQ            In an interrupt            N/C      N/C
    LED_SIGNAL           In a signal handler        N/C      N/C
    LED_ASSERTION        An assertion failed        N/C      N/C
    LED_PANIC            The system has crashed     N/C      Blinking
    LED_IDLE             MCU is is sleep mode         Not used

  Thus if LED2 is statically on, NuttX has successfully booted and is,
  apparently, running normmally.  If LED1 is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

  NOTE: That LED2 is not used after completion of booting and may
  be used by other board-specific logic.

Image Format
============

  In order to use the bootrom bootloader, a special header must be added to
  the beginning of the binary image that includes information about the
  binary (things like the entry point, the size, and CRC's to verify the image.

  NXP provides a Windows program to append such a header to the binary
  image.  However, (1) that program won't run under Linux, and (2) when I
  try it under WinXP, Symantec immediately claims that the program is
  misbehaving and deletes it!

  To work around both of these issues, I have created a small program under
  boards/olimex-lpc-h3131/tools to add the header.  This program can be
  built under either Linux or Cygwin (and probably other tool environments
  as well).  That tool can be built as follows:

  - cd boards/olimex-lpc-h3131/tools
  - make

  Then, to build the NuttX binary ready to load with the bootloader, just
  following these steps:

  - tools/configure.sh olimex-lpc-h3131:ostest  # (using the ostest configuration for this example)
  - cd ..                         # Set up environment
  - make                          # Make NuttX.  This will produce nuttx.bin
  - mklpc.sh                      # Make the bootloader binary (nuttx.lpc)

  NOTES:

    1. Make sure to set your PATH variable appropriately or use the full path
       to mklpc.sh in the final step.
    2. You can instruct Symantec to ignore the errors and it will stop
       quarantining the NXP program.
    3. The CRC32 logic in boards/olimex-lpc-h3131/tools doesn't seem to
       work.  As a result, the CRC is currently disabled in the header:

       RCS file: /cvsroot/nuttx/nuttx/boards/olimex-lpc-h3131/tools/lpchdr.c,v
       retrieving revision 1.2
       diff -r1.2 lpchdr.c
       264c264
       <   g_hdr.imageType       = 0x0000000b;
       ---
       >   g_hdr.imageType       = 0x0000000a;

Image Download to ISRAM
=======================

  Assuming that you already have the FTDI driver installed*, then here is the
  are the steps that I use for loading new code into the LPC-H3131:

  1. Create the bootloader binary, nuttx.lpc, as described above.

  2. With the power off, set the boot jumpers to enable booting from UART.
     The boot jumpers are the block of three jumper just in-board from the
     JTAG connector; Jumper pair 1-2 is the pair furthest from the JTAG
     connector:

       1-2: Closed
       3-4: Closed
       5-6: Open

  3. Connected the LPC-H3131 using the FTDI USB port (not the lpc3131 USB port)
     This will power up the LPC-H3131 and start the bootloader.

  4. Start a terminal emulator (such as TeraTerm) at 115200 8NI.

  5. Reset the LPC-H3131 and you should see:

       LPC31xx READY FOR PLAIN IMAGE>

  6. Send the nuttx.lpc file and you should see:

       Download finished

  That will load the NuttX binary into ISRAM and attempt to execute it.

  *See the LPC313x documentation if you do not have the FTDI driver installed.

  TeraTerm Note:  This is how to send a file from TeraTerm.  It is essentially
  step 6 exploded in more detail for the case of TeraTerm:

  1. Start the ROM bootloader as described above.

  2. At the "LPC31xx READY FOR PLAIN IMAGE>" prompt, open the File menu and
     select the "Send File..." option.

  3. Select the file to send,

  4. Before "Open" -ing the file MAKE SURE TO CHECK THE "Binary" BOX!  This
     has cost me a few hours a few times because I forget to do this.  The
     program will NOT RUN is sent non-binary.

     [NO, I am not SHOUTING.  I am just making sure that I never forget to
      do this again].

  5. "Open"-ing the file will send it to the ROM bootloader.

  6. You should see "Download finished" from the bootloader followed
     immediately by any serial console output from your program.

Using OpenOCD and GDB
=====================
  [NOTE:  As of this writing, my OpenOCD script does NOT work.  It fails
   because it is unable to halt the LPC3131.  So, unfortunately, OpenOCD
   is not a option right now.]

  I have been using the Olimex ARM-USB-OCD JTAG debugger with the LPC-H3131
  (http://www.olimex.com).  The OpenOCD configuration file is here:
  tools/armusbocb.cfg.  There is also a script on the tools directory that
  I used to start the OpenOCD daemon on my system called oocd.sh.  That
  script would probably require some modifications to work in another
  environment:

    - possibly the value of OPENOCD_PATH
    - If you are working under Linux you will need to change any
      occurrences of `cygpath -w blablabla` to just blablabla

  Then you should be able to start the OpenOCD daemon like:

    boards/olimex-lpc-h3131/tools/oocd.sh $PWD

  Where it is assumed that you are executing oocd.sh from the top level
  directory where NuttX is installed.

  Once the OpenOCD daemon has been started, you can connect to it via
  GDB using the following GDB command:

   arm-nuttx-elf-gdb
   (gdb) target remote localhost:3333

  And you can load the NuttX ELF file:

   (gdb) symbol-file nuttx
   (gdb) load nuttx

ARM/LPC-H3131-specific Configuration Options
============================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_ARM926EJS=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc313x

    CONFIG_ARCH_CHIP_name - For use in C code

       CONFIG_ARCH_CHIP_LPC3131

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD="olimex-lpc-h3131"

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_OLIMEX_LPC_H3131

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - For most ARM9 architectures, this describes the
      size of installed DRAM.  For the LPC313X, it is used only to
      deterimine how to map the executable regions.  It is SDRAM size
      only if you are executing out of the external SDRAM; or it could
      be NOR FLASH size, external SRAM size, or internal SRAM size.

    CONFIG_RAM_START - The start address of installed DRAM (physical)

    CONFIG_RAM_VSTART - The startaddress of DRAM (virtual)

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
       stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    CONFIG_ARCH_BUTTONS -  Enable support for buttons. Unique to board architecture.

    CONFIG_ARCH_DMA - Support DMA initialization

    CONFIG_ARCH_LOWVECTORS - define if vectors reside at address 0x0000:00000
      Undefine if vectors reside at address 0xffff:0000

    CONFIG_ARCH_ROMPGTABLE - A pre-initialized, read-only page table is available.
      If defined, then board-specific logic must also define PGTABLE_BASE_PADDR,
      PGTABLE_BASE_VADDR, and all memory section mapping in a file named
      board_memorymap.h.

  Individual subsystems can be enabled:

    CONFIG_LPC31_MCI, CONFIG_LPC31_SPI, CONFIG_LPC31_UART

  External memory available on the board (see also CONFIG_MM_REGIONS)

    CONFIG_LPC31_EXTSRAM0 - Select if external SRAM0 is present
    CONFIG_LPC31_EXTSRAM0HEAP - Select if external SRAM0 should be
      configured as part of the NuttX heap.
    CONFIG_LPC31_EXTSRAM0SIZE - Size (in bytes) of the installed
      external SRAM0 memory
    CONFIG_LPC31_EXTSRAM1 - Select if external SRAM1 is present
    CONFIG_LPC31_EXTSRAM1HEAP - Select if external SRAM1 should be
      configured as part of the NuttX heap.
    CONFIG_LPC31_EXTSRAM1SIZE - Size (in bytes) of the installed
      external SRAM1 memory
    CONFIG_LPC31_EXTDRAM - Select if external SDRAM is present
    CONFIG_LPC31_EXTDRAMHEAP - Select if external SDRAM should be
      configured as part of the NuttX heap.
    CONFIG_LPC31_EXTDRAMSIZE - Size (in bytes) of the installed
      external SDRAM memory
    CONFIG_LPC31_EXTNAND - Select if external NAND is present
    CONFIG_LPC31_EXTNANDSIZE - Size (in bytes) of the installed
      external NAND memory

  LPC313X specific device driver settings

    CONFIG_UART_SERIAL_CONSOLE - selects the UART for the
      console and ttys0
    CONFIG_UART_RXBUFSIZE - Characters are buffered as received.
      This specific the size of the receive buffer
    CONFIG_UART_TXBUFSIZE - Characters are buffered before
      being sent.  This specific the size of the transmit buffer
    CONFIG_UART_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_UART_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_UART_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_UART_2STOP - Two stop bits

Configurations
==============

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each LPC-H3131 configuration is maintained in a sub-directory and can be
  selected as follow:

    tools/configure.sh olimex-lpc-h3131:<subdir>

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

  2. Unless stated otherwise, all configurations generate console
     output on the UART0 associated with the FT232RL USB-to UART
     converter.

  3. Unless otherwise stated, the configurations are setup for
     Windows undery Cygwin.  This can, however, be easilty reconfigured.

  4. All of these configurations use the Code Sourcery for Windows toolchain
     (unless stated otherwise in the description of the configuration).  That
     toolchain selection can easily be reconfigured using 'make menuconfig'.
     Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Microsoft Windows
       CONFIG_WINDOWS_CYGWIN=y             : Using Cygwin or other POSIX environment

     System Type -> Toolchain:
       CONFIG_ARM_TOOLCHAIN_GNU_EABIW=y : GNU EABI toolchain for windows

  Configuration sub-directories
  -----------------------------

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables only the serial NSH interface.

    General Configuration.  These are easily change by modifying the NuttX
    configuration:

      - Console on UART -> UART-to-USB converter
      - Platform: Windows with Cygwin
      - Toolchain:  ARM EABI GCC for Windows

    NOTES:
    1. Built-in applications are not supported by default.  To enable NSH
       built-in applications:

       Binary
         CONFIG_BUILTIN=y                      : Support built-in applications

       Application Configuration -> NSH Library
         CONFIG_NSH_BUILTIN_APPS=y             : Enable built-in applications

    2. SDRAM support is not enabled by default.  SDRAM support can be enabled
       by adding the following to your NuttX configuration file:

       [NOTE: There is still something wrong with the SDRAM setup.  At present
        it hangs on the first access from SDRAM during configuration.]

       System Type->LPC31xx Peripheral Support
         CONFIG_LPC31_EXTDRAM=y                : Enable external DRAM support
         CONFIG_LPC31_EXTDRAMSIZE=33554432     : 256Mbit -> 32Mbyte
         CONFIG_LPC31_SDRAM_16BIT=y            : Organized 16Mbit x 16 bits wide

       Now that you have SDRAM enabled, what are you going to do with it?  One
       thing you can is add it to the heap

       System Type->Heap Configuration
         CONFIG_LPC31_EXTDRAMHEAP=y            : Add the SDRAM to the heap

       Memory Management
         CONFIG_MM_REGIONS=2                   : Two memory regions: ISRAM and SDRAM

       Another thing you could do is to enable the RAM test built-in
       application:

    3. You can enable the NuttX RAM test that may be used to verify the
       external SDAM.  To do this, keep the SDRAM out of the heap so that
       it can be tested without crashing programs using the memory.

       First enable built-in applications as described above, then make
       the following additional modifications to the NuttX configuration:

       System Type->Heap Configuration
         CONFIG_LPC31_EXTDRAMHEAP=n            : Don't add the SDRAM to the heap

       Memory Management
         CONFIG_MM_REGIONS=1                   : One memory regions:  ISRAM

       Then enable the RAM test built-in application:

       Application Configuration->System NSH Add-Ons->Ram Test
         CONFIG_SYSTEM_RAMTEST=y

       In this configuration, the SDRAM is not added to heap and so is not
       excessible to the applications.  So the RAM test can be freely
       executed against the SRAM memory beginning at address 0x2000:0000
       (DDR CS):

       nsh> ramtest -h
       Usage: ramtest [-w|h|b] <hex-address> <decimal-size>

       Where:
         <hex-address> starting address of the test.
         <decimal-size> number of memory locations (in bytes).
         -w Sets the width of a memory location to 32-bits.
         -h Sets the width of a memory location to 16-bits (default).
         -b Sets the width of a memory location to 8-bits.

       To test the entire external 256MB SRAM:

       nsh> ramtest -w 30000000 33554432
       RAMTest: Marching ones: 30000000 33554432
       RAMTest: Marching zeroes: 30000000 33554432
       RAMTest: Pattern test: 30000000 33554432 55555555 aaaaaaaa
       RAMTest: Pattern test: 30000000 33554432 66666666 99999999
       RAMTest: Pattern test: 30000000 33554432 33333333 cccccccc
       RAMTest: Address-in-address test: 30000000 33554432

    4. This configuration has been used to test USB host functionality.  USB
       host is *not* enabled by default.  If you will to enable USB host
       support in the NSH configuration, please modify the NuttX
       configuration as follows:

       a) Basic USB Host support

          Drivers -> USB Host Driver Support
            CONFIG_USBHOST=y              : General USB host support
            CONFIG_USBHOST_INT_DISABLE=n  : Interrupt EPs need with hub, HID keyboard, and HID mouse
            CONFIG_USBHOST_ISOC_DISABLE=y : Not needed (or supported)

          System Type -> Peripherals
            CONFIG_LPC31_USBOTG=y         : Enable the USB OTG peripheral

          System Type -> USB host configuration
            CONFIG_LPC31_EHCI_BUFSIZE=128
            CONFIG_LPC31_EHCI_PREALLOCATE=y

          RTOS Features -> Work Queue Support
            CONFIG_SCHED_WORKQUEUE=y      : High priority queue support is needed
            CONFIG_SCHED_HPWORK=y
            CONFIG_SCHED_HPWORKSTACKSIZE=1536 (1024 seems to work okay too)

       b. Hub Support.

          Drivers -> USB Host Driver Support
            CONFIG_USBHOST_INT_DISABLE=n  : Interrupt endpoint support needed
            CONFIG_USBHOST_HUB=y          : Enable the hub class
            CONFIG_USBHOST_ASYNCH=y       : Asynchronous I/O supported needed for hubs

          RTOS Features -> Work Queue Support
            CONFIG_SCHED_LPWORK=y         : Low priority queue support is needed
            CONFIG_SCHED_LPNTHREADS=1
            CONFIG_SCHED_LPWORKSTACKSIZE=1024

          NOTES:

          1. It is necessary to perform work on the low-priority work queue
             (vs. the high priority work queue) because:

             a. Deferred work requires some delays and waiting, and
             b. There are dependencies between the waiting and driver
                interrupt related work.  Since that interrupt related work
                will performed on the high priority work queue, there would
                be the likelihood of deadlocks if you wait for events on the
                high priority work thread that can only occur if the high
                priority work thread is available to post those events.

          2. Logic nesting becomes deeper with a hub and it may also be
             necessary to increase some stack sizes.

       c. USB Mass Storage Class.  With this class enabled, you can support
          connection of USB FLASH storage drives.  Support for the USB
          mass storage class is enabled like this:

          Drivers -> USB Host Driver Support
            CONFIG_USBHOST_MSC=y          : Mass storage class support

          The MSC class will work like this.  When you first start NSH, you
          can look at the available devices like this:

            NuttShell (NSH) NuttX-6.31
            nsh> ls -l /dev
            /dev:
             crw-rw-rw-       0 console
             crw-rw-rw-       0 null
             crw-rw-rw-       0 ttyS0

          The crw-rw-rw- indicates a readable, write-able character device.

            nsh> ls -l /dev
            /dev:
             crw-rw-rw-       0 console
             crw-rw-rw-       0 null
             brw-rw-rw-       0 sda
             crw-rw-rw-       0 ttyS0

          The brw-rw-rw- indicates a readable, write-able block device.
          This block device can then be mounted like this:

            nsh> mount -t vfat /dev/sda /mnt/flash

          The USB FLASH drive contents are then visible under /mnt/flash and
          can be operated on with normal file system commands like:

            nsh> mount -t vfat /dev/sda /mnt/flash
            nsh> cat /mnt/flash/filec.c
            etc.

          It is recommended that the drive by unmounted BEFORE it is
          removed.  That is not always possible so if the USB FLASH is
          removed BEFORE the drive is unmounted, the device at /dev/sda will
          persist in an unusable stack until it is unmounted with the
          following command (NOTE:  If the FLASH drive is re-inserted in
          this state, it will appear as /dev/sdb):

            nsh> umount /mnt/flash

       d. HID Keyboard support.  The following support will enable support
          for certain keyboard devices (only the so-called "boot" keyboard
          class is supported):

          Drivers -> USB Host Driver Support
            CONFIG_USBHOST_HIDKBD=y       : HID keyboard class support

          Drivers -> USB Host Driver Support
            CONFIG_USBHOST_INT_DISABLE=n  : Interrupt endpoint support needed

          In this case, when the HID keyboard is installed, you see a new
          character device called /dev/kbda.

          There is a HID keyboard test example that can be enabled with the
          following settings.  NOTE:  In this case, NSH is disabled because
          the HID keyboard test is a standalone test.

          This selects the HIDKBD example:

          Application Configuration -> Examples
            CONFIG_EXAMPLES_HIDKBD=y
            CONFIG_EXAMPLES_HIDKBD_DEVNAME="/dev/kbda"

          RTOS Features
            CONFIG_USER_ENTRYPOINT="hidkbd_main"

          These settings disable NSH:

          Application Configuration -> Examples
            CONFIG_SYSTEM_NSH=n

          Application Configuration -> NSH Library
            CONFIG_NSH_LIBRARY=y

          Using the HID Keyboard example:  Anything typed on the keyboard
          should be echoed on the serial console.  Here is some sample of
          a session:

          Initialization

            hidkbd_main: Register class drivers
            hidkbd_main: Initialize USB host keyboard driver
            hidkbd_main: Start hidkbd_waiter
            hidkbd_waiter: Running

          The test example will periodically attempt to open /dev/kbda

            Opening device /dev/kbda
            Failed: 2
            Opening device /dev/kbda
            Failed: 2
            etc.

          The open will fail each time because there is no keyboard
          attached.  When a USB keyboard is attached, the open of /dev/kbda
          will succeed and the test will begin echoing data to the serial
          console:

            hidkbd_waiter: connected
            Opening device /dev/kbda
            Device /dev/kbda opened

          For example, this text was entered from the keyboard:

            Now is the time for all good men to come to the aid of their party.

          Then when the device is removed, the test will resume attempting
          to open the driver until the next time it is connected

            Closing device /dev/kbda: -1
            Opening device /dev/kbda
            Failed: 19
            hidkbd_waiter: disconnected

            Opening device /dev/kbda
            Failed: 2
            etc.

       d. The USB monitor can also be enabled:

         Drivers -> USB Host Driver Support
           CONFIG_USBHOST_TRACE=y
           CONFIG_USBHOST_TRACE_NRECORDS=128
           CONFIG_USBHOST_TRACE_VERBOSE=y

         Application Configuration -> System Add-Ons
           CONFIG_USBMONITOR=y
           CONFIG_USBMONITOR_INTERVAL=1

       NOTE:  I have found that if you enable USB DEBUG and/or USB tracing,
       the resulting image requires to much memory to execute out of
       internal SRAM.  I was able to get the configurations to run out of
       SRAM with debug/tracing enabled by carefully going through the
       configuration and reducing stack sizes, disabling unused OS features,
       disabling un-necessary NSH commands, etc.

    5. Making the Configuration Smaller.  This configuration runs out of
       internal SRAM.  If you enable many features, then your code image
       may outgrow the available SRAM; even if the code can be loaded into
       SRAM, it may still fail at runtime due to insufficient memory.

       Since SDRAM is not currently working (see above) and NAND support
       has not be integrated, the only really option is to put NSH "on a
       diet" to reduct the size so that it will fit into memory.

       Here are a few things you can do:

       1. Try using smaller stack sizes,

       2. Disable operating system features.  Here some that can go:

          CONFIG_DISABLE_ENVIRON=y
          CONFIG_DISABLE_MQUEUE=y
          CONFIG_DISABLE_POSIX_TIMERS=y
          CONFIG_DISABLE_PTHREAD=y
          CONFIG_MQ_MAXMSGSIZE=0
          CONFIG_TLS_NELEM=0
          CONFIG_NUNGET_CHARS=0
          CONFIG_PREALLOC_MQ_MSGS=0

       3. Disable NSH commands.  I can life fine without these:

          CONFIG_NSH_DISABLE_ADDROUTE=y
          CONFIG_NSH_DISABLE_CD=y
          CONFIG_NSH_DISABLE_CMP=y
          CONFIG_NSH_DISABLE_CP=y
          CONFIG_NSH_DISABLE_DD=y
          CONFIG_NSH_DISABLE_DELROUTE=y
          CONFIG_NSH_DISABLE_EXEC=y
          CONFIG_NSH_DISABLE_EXIT=y
          CONFIG_NSH_DISABLE_GET=y
          CONFIG_NSH_DISABLE_HEXDUMP=y
          CONFIG_NSH_DISABLE_IFCONFIG=y
          CONFIG_NSH_DISABLE_LOSETUP=y
          CONFIG_NSH_DISABLE_MB=y
          CONFIG_NSH_DISABLE_MH=y
          CONFIG_NSH_DISABLE_MKFIFO=y
          CONFIG_NSH_DISABLE_MKRD=y
          CONFIG_NSH_DISABLE_NSFMOUNT=y
          CONFIG_NSH_DISABLE_PING=y
          CONFIG_NSH_DISABLE_PUT=y
          CONFIG_NSH_DISABLE_PWD=y
          CONFIG_NSH_DISABLE_RM=y
          CONFIG_NSH_DISABLE_RMDIR=y
          CONFIG_NSH_DISABLE_SET=y
          CONFIG_NSH_DISABLE_SOURCE=y
          CONFIG_NSH_DISABLE_SLEEP=y
          CONFIG_NSH_DISABLE_TEST=y
          CONFIG_NSH_DISABLE_UNSET=y
          CONFIG_NSH_DISABLE_USLEEP=y
          CONFIG_NSH_DISABLE_WGET=y
          CONFIG_NSH_DISABLE_XD=y

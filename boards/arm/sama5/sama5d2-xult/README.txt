README
======

  This README file describes the port of NuttX to the SAMA5D2 Xplained Ulta
  development board. This board features the Atmel SAMA5D27 microprocessor.
  See http://www.atmel.com for further information.

Contents
========

  - STATUS
  - Loading Code into SRAM with J-Link
  - DRAMBOOT, AT25BOOT, SRAMBOOT
  - Running NuttX from SDRAM
  - Buttons and LEDs
  - Serial Console
  - SAMA5D2-XULT Configuration Options
  - Configurations

STATUS
======

1. Most of this document is a partially corrected clone of the SAMA5D4-EK
   README.txt and still contains errors and inconsistencies.

2. Coding is complete for the basic SAMA5D2-XULT NSH configuration, but
   is completely untested as of this writing (2015-09-15).  The primary
   issue is  that I have not yet determine how to load and test code.

Loading Code into SRAM with J-Link
==================================
REVISIT: Unverified, cloned text from the SAMA5D4-EK README.txt

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

DRAMBOOT, AT25BOOT, SRAMBOOT
----------------------------

  See also boards/arm/sama5/sama5d4-xult/README.txt for a description of the DRAMBOOT
  program.  This is a tiny version of NuttX that can run out of internal
  SRAM.  If you put this program on the HSMCI1 microSD card as boot.bin, then
  it will boot on power up and you can download NuttX directly into DRAM by
  sending the nuttx.hex file over the serial connection.

  The boards/arm/sama5/sama5d4-xult/README.txt also describes variants AT25BOOT and
  SRAMBOOT.  This have not yet been ported to the SAMA5D2-XULT, but are
  available if they are useful too you.

Running NuttX from SDRAM
========================
REVISIT: Unverified, cloned text from the SAMA5D4-EK README.txt

  NuttX may be executed from SDRAM.  But this case means that the NuttX
  binary must reside on some other media (typically NAND FLASH, Serial
  FLASH) or transferred over some interface (perhaps a UARt or even a
  TFTP server).  In these cases, an intermediate bootloader such as U-Boot
  or Barebox must be used to configure the SAMA5D2 clocks and SDRAM and
  then to copy the NuttX binary into SDRAM.

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
  configure the SAMA5D2 clocking.  Rather, it will use the clock configuration
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

      mkimage -A arm -O linux -C none -T kernel -a 20008000 -e 20008E20 \
        -n nuttx -d nuttx.bin uImage

    Where:

      -A arm: Set architecture to ARM
      -O linux: Select operating system. bootm command of u-boot changes
         boot method by os type.
      -T kernel: Set image type.
      -C none: Set compression type.
      -a 20008000:  Set load address.
      -e 20008E20: Set entry point.
      -n nuttx: Set image name.
      -d nuttx.bin: Use image data from nuttx.bin.

    This will generate a binary called uImage.  If you have the path to
    mkimage in your PATH variable, then you can automatically build the
    uImage file by adding the following to your .config file:

      CONFIG_RAW_BINARY=y
      CONFIG_UBOOT_UIMAGE=y
      CONFIG_UIMAGE_LOAD_ADDRESS=0x20008000
      CONFIG_UIMAGE_ENTRY_POINT=0x20008E20

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
           Entry Point:  20008E20
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

        U-Boot> go 0x20008E20
        ## Starting application at 0x20008E20 ...

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
  8. Set the PC to the NuttX entry point 'mon pc 0x20008E20' and start
     nuttx using 'mon go'.

Buttons and LEDs
================

  Buttons
  -------
  A single button, PB_USER1 (PB6), is available on the SAMA5D2-XULT

    ------------------------------ ------------------- -------------------------
    SAMA5D2 PIO                    SIGNAL              USAGE
    ------------------------------ ------------------- -------------------------
    PB6                            USER_PB_PB6         PB_USER push button
    ------------------------------ ------------------- -------------------------

  Closing PB_USER will bring PB6 to ground so 1) PB6 should have a weak pull-up,
  and 2) when PB_USER is pressed, a low value will be senses.

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

  There is an RGB LED on board the SAMA5D2-XULT.  The RED component is driven by
  the SDHC_CD pin (PA13) and so will not be used.  The LEDs are provided VDD_LED
  and so bringing the LED low will will illuminated the LED.

    ------------------------------ ------------------- -------------------------
    SAMA5D2 PIO                    SIGNAL              USAGE
    ------------------------------ ------------------- -------------------------
    PA13                           SDHC_CD_PA13        Red LED
    PB5                            LED_GREEN_PB5       Green LED
    PB0                            LED_BLUE_PB0        Blue LED
    ------------------------------ ------------------- -------------------------

  When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
  control the Green LED (only)as follows:

    SYMBOL              Meaning                 Green LED
    ------------------- ----------------------- ---------
    LED_STARTED         NuttX has been started  OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF
    LED_IRQSENABLED     Interrupts enabled      OFF
    LED_STACKCREATED    Idle stack created      ON
    LED_INIRQ           In an interrupt         N/C
    LED_SIGNAL          In a signal handler     N/C
    LED_ASSERTION       An assertion failed     N/C
    LED_PANIC           The system has crashed  FLASH

  Thus if the Green LED is statically on, NuttX has successfully  booted and
  is, apparently, running normally.  If LED is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

Serial Console
==============

  DEBUG / DBGU Port (J1).  There is a TTL serial connection available on
  pins 2 and 3 of the DEBUG connector.  This may be driven by UART1,
  depending upon the setting of JP2 (DBGU_PE on the schematic, DEBUG_DIS
  on the board):

    ---- ------------------------ -------------
    J1   SCHEMATIC                   SAMA5D2
    PIN  NAME(s)                  PIO  FUNCTION
    ---- ------------------------ -------------
     2   DBGU_TXD  DBGU_UTXD1_PD3 PD3  UTXD1
     3   DBGU_RXD  DBGU_URXD1_PD2 PD2  URXD1
    ---- ------------------------ -------------

  Standard UART on Arduino connector (J21) is FLEXCOM4.
  Terminology: FLEXCOM is the same as USART in previous SAMA5D versions.

    ---- ------- -------------
    J21  BOARD      SAMA5D2
    PIN  NAME    PIO  FUNCTION
    ---- ------- -------------
     7   F4_TXD  PD12 FLEXCOM4
     8   F4_RXD  PD13 FLEXCOM4
    ---- ------- -------------

  Other USARTs are available on J22:

    ---- ------- -------------
    J22  BOARD      SAMA5D2
    PIN  NAME    PIO  FUNCTION
    ---- ------- -------------
     3   F0_TXD  PB28 FLEXCOM0
     4   F0_RXD  PB29 FLEXCOM0
     5   F3_TXD  PB23 FLEXCOM3
     6   F3_RXD  PB22 FLEXCOM3
    ---- ------- -------------

  UARTs available of EXT1

    ---- ------- -------------
    EXT1 BOARD      SAMA5D2
    PIN  NAME    PIO  FUNCTION
    ---- ------- -------------
     13  UART_RX PA23 FLEXCOM1
     14  UART_TX PA24 FLEXCOM1
    ---- ------- ---- --------

  UARTs available of EXT2

    ---- ------- -------------
    EXT2 BOARD      SAMA5D2
    PIN  NAME    PIO  FUNCTION
    ---- ------- -------------
     13  UART_RX PB29 FLEXCOM0
     14  UART_TX PB28 FLEXCOM0
    ---- ------- ---- --------

  By default, the standard UART on Arduino connector (J21, FLEXCOM4) is
  enabled in all of these configurations unless otherwise noted.

REVISIT: UART1 on the DBGU connect might be a better choice for the
default serial console

SAMA5D2-XULT Configuration Options
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
    CONFIG_ARCH_CHIP_ATSAMA5D27=y

  CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
  hence, the board that supports the particular chip or SoC.

    CONFIG_ARCH_BOARD="sama5d2-xult" (for the SAMA5D2-XULT development board)

  CONFIG_ARCH_BOARD_name - For use in C code

    CONFIG_ARCH_BOARD_SAMA5D2_XULT=y

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
REVISIT: Unverified, cloned text from the SAMA5D4-EK README.txt

    CONFIG_SAMA5_DBGU        - Debug Unit
    CONFIG_SAMA5_PIT         - Periodic Interval Timer
    CONFIG_SAMA5_WDT         - Watchdog timer
    CONFIG_SAMA5_HSMC        - Multi-bit ECC
    CONFIG_SAMA5_SMD         - SMD Soft Modem
    CONFIG_SAMA5_FLEXCOM0    - Flexcom 0
    CONFIG_SAMA5_FLEXCOM1    - Flexcom 0
    CONFIG_SAMA5_FLEXCOM2    - Flexcom 0
    CONFIG_SAMA5_FLEXCOM3    - Flexcom 0
    CONFIG_SAMA5_FLEXCOM4    - Flexcom 0
    CONFIG_SAMA5_UART0       - UART 0
    CONFIG_SAMA5_UART1       - UART 1
    CONFIG_SAMA5_UART2       - UART 2
    CONFIG_SAMA5_UART3       - UART 3
    CONFIG_SAMA5_UART4       - UART 4
    CONFIG_SAMA5_TWI0        - Two-Wire Interface 0
    CONFIG_SAMA5_TWI1        - Two-Wire Interface 1
    CONFIG_SAMA5_SDMMC0      - SD MMC card interface 0
    CONFIG_SAMA5_SDMMC1      - SD MMC card interface 1
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

    CONFIG_USART0_SERIALDRIVER - Flexcom0 is configured as a UART
    CONFIG_USART1_SERIALDRIVER - Flexcom1 is configured as a UART
    CONFIG_USART2_SERIALDRIVER - Flexcom2 is configured as a UART
    CONFIG_USART3_SERIALDRIVER - Flexcom3 is configured as a UART
    CONFIG_USART4_SERIALDRIVER - Flexcom4 is configured as a UART

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
  Each SAMA5D2-XULT configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh sama5d2-xult:<subdir>

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
         CONFIG_SAMA5D2XULT_528MHZ=y       : Enable 528MHz operation
         CONFIG_BOARD_LOOPSPERMSEC=65775 : Calibrated on SAMA5D3-Xplained at
                                         : 528MHz running from SDRAM

  Configuration Sub-directories
  -----------------------------
  Summary:  Some of the descriptions below are long and wordy. Here is the
  concise summary of the available SAMA5D2-XULT configurations:

    nsh:  This is a basic NuttShell (NSH) configuration.

  There may be issues with some of these configurations.  See the details
  for status of individual configurations.

  Now for the gory details:

  netnsh:

    This is a network enabled configuration based on the NuttShell (NSH).
    The CDC-ECM driver is enabled, so you can plug a USB cable into the
    USB-Micro port (USB-A) and the board will appear as an CDC-ECM
    ethernet adapter.

  nsh:

    This configuration directory provide the NuttShell (NSH).  This is a
    very simple NSH configuration upon which you can build further
    functionality.

    NOTES:

    1. This configuration uses the UART1 (PD2 and PD3) for the serial
       console.  USART1 is available at the "DBGU" RS-232 connector (J24).
       This is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as the
       console device.

    2. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain (such as the ARM supported
       toolchain).  Both the build environment and the toolchain
       selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y           : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y         : POSIX environment under windows
       CONFIG_ARMV7A_TOOLCHAIN_EABIW=y : Generic GCC EABI toolchain for Windows

       If you are running on Linux, make *certain* that you have
       CONFIG_HOST_LINUX=y *before* the first make or you will create a
       corrupt configuration that may not be easy to recover from. See
       the warning in the section "Information Common to All Configurations"
       for further information.

    4. This configuration supports logging of debug output to a circular
       buffer in RAM.  This feature is discussed fully in this Wiki page:
       https://cwiki.apache.org/confluence/display/NUTTX/SYSLOG . Relevant
       configuration settings are summarized below:

       File System:

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

         CONFIG_SAMA5D2XULT_DRAM_BOOT=y

       See the section above entitled "Creating and Using DRAMBOOT" above
       for more information.  Here is a summary of the steps that I used
       to boot the NSH configuration:

         a. Create the DRAMBOOT program as described above.  It should be
            configured with CONFIG_SAMA5D2XULT_DRAM_START=y so that DRAMBOOT
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
         U-Boot> go 0x20008E20 

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

       Support for /dev/random is implemented using the SAMA5D2's True
       Random Number Generator (TRNG).  See the section above entitled
       "TRNG and /dev/random" for information about configuring /dev/random.

        CONFIG_SAMA5_TRNG=y   : Enables the TRNG peripheral
        CONFIG_DEV_RANDOM=y   : Enables /dev/random

    8. This configuration has support for NSH built-in applications enabled.
       No built-in applications are enabled, however.

    9. This configuration has support for the FAT and PROCFS file
       systems built in.

       The FAT file system includes long file name support.  Please be aware
       that Microsoft claims patents against the long file name support (see
       more discussion in the top-level NOTICE file).

         CONFIG_FS_FAT=y        : Enables the FAT file system
         CONFIG_FAT_LCNAMES=y   : Enable lower case 8.3 file names
         CONFIG_FAT_LFN=y       : Enables long file name support
         CONFIG_FAT_MAXFNAME=32 : Arbitrarily limits the size of a path
                                  segment name to 32 bytes

       The PROCFS file system is enabled simply with:

         CONFIG_FS_PROCFS=y     : Enable PROCFS file system

   10. The Real Time Clock/Calendar (RTC) is enabled in this configuration.
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

  sdmmcnsh:

    This is a configuration based on the NuttShell (NSH). The SDMMC
    peripheral is enabled, and can read and write to a VFAT filesystem
    on the SD Card.


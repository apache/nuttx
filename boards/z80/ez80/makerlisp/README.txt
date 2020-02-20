README.txt
==========

The MakerLisp machine is a portable, modular computer system, designed to
recapture the feel of classic computing, with modern hardware.

The machine centers on a 2" x 3.5" business card-sized CPU, which can be used
stand-alone, or plugged in to a 2" x 8" main board, for expansion into a full
computer system.  A laser-cut wood enclosure holds a small keyboard, an LCD
monitor, the circuit boards, and a prototyping area with a breadboard for
electronics experimentation and development.

The CPU is a Zilog eZ80 running at 50 MHz, with up to 16 Mb of zero-wait state
RAM. A VGA display adapter provides an IBM PC-like color text-mode display. A
USB Host Controller supports a USB keyboard and other USB communications.
Data storage and interchange is accomplished by a micro-SD card supporting the
FAT file system. All four of these circuit boards (shown on the web site's cover
page) are new MakerLisp products, and will be available as part of the first
product offering

Contents
========

  o ZDS-II Compiler Versions
  o Serial Console
    - UARTs
    - Serial Keyboard and VGA Display
  o LEDs and Buttons
    - LEDs
    - Buttons
  o Configurations
    - Common Configuration Notes
    - Configuration Subdirectories

ZDS-II Compiler Versions
========================

Version 5.3.3

  As of this writing, this is the latest version available.  This is the
  default configured for all ez80 boards.

Version 5.3.0

  I verified compilation using 5.3.0 on June 2, 2019.  To use this version,
  I had to make spurious modification to the implementation of gmtimer() to
  work around an internal compiler error.  I have still not verified that
  are no errors in the compiled code.

Other Versions
  If you use any version of ZDS-II other than 5.3.0 or if you install ZDS-II
  at any location other than the default location, you will have to modify
  three files:  (1) arch/arm/z80/src/ez80/Kconfig, (2)
  boards/z80/ez80/makerlisp/scripts/Make.defs and, perhaps, (3)
  arch/z80/src/ez80/Toolchain.defs.

Serial Console
==============

  There are two options for a serial console:  (1) A UART connected to a
  terminal program or (2) the MakerLisp Serial Keyboard and VGA display.

UARTs
-----

  The eZ80 has two UART peripherals:

  UART 0:  All of Port D pins can support UART0 functions when configured
  for the alternate function 7.  For typical configurations only RXD and TXD
  need be configured.

    eZ80 PIN        BOARD SIGNAL CN1 ACCESS
    =======================================
    PD0/TXD0/IR_IXD CN1_TX0      Pin 61
    PD1/RXD0/IR_RXD CN1_RX0      Pin 59
    PD2/RTS0        CN1_RTS0     Pin 63
    PD3/CTS0        CN1_CTS0     Pin 65
    PD4/DTR0        CN1_DTR0     Pin 67
    PD5/DSR0        CN1_DSR0     Pin 69
    PD6/DCD0        CN1_DCD0     Pin 71
    PD7/RIO0        CN1_RI0      Pin 73

  UART0 (as well as I2C) is also available via a USB using the on-board
  MCP2221A USB adapter.  CN1_USBUART_TX_EN and CN1_USBUART_RX_EN are pulled
  low poll on the CPU board in order to connect CN1_RX0 and CN1_TX0 to
  MCP_RX and MCP_TX.

  When the I/O expander board is connected, jumpers J1 and J2 control this
  functionality.  These can pull the CN1_USBUART_TX_EN and CN1_USBUART_RX_EN
  pins high and so that UART0 can be used for other purposes.

  UART 1:  All of Port C pins can support UART1 functions when configured
  for the alternate function 7.  For typical configurations only RXD and TXD
  need be configured.

    eZ80 PIN        BOARD SIGNAL CN1 ACCESS
    =======================================
    PC0/TXD1        CN1_TX1      Pin 62
    PC1/RXD1        CN1_RX1      Pin 60
    PC2/RTS1        CN1_RTS1     Pin 64
    PC3/CTS1        CN1_CTS1     Pin 66
    PC4/DTR1        CN1_DTR1     Pin 68
    PC5/DSR1        CN1_DSR1     Pin 70
    PC6/DCD1        CN1_DCD1     Pin 72
    PC7/RIO1        CN1_RI1      Pin 74

  With the I/O exanpander board (and J1 and J2 open), these UARTs can be
  used with a host terminal emulation, by connecting either a TTL-to-RS232
  or a TTL-to-USB Serial adapter to CN1 pins 59 and 61, and 60 and 62,
  depending on the selected UART.

Serial Keyboard and VGA Display
-------------------------------

  The serial console can also be implemented using the MakerLisp USB
  Keyboard Controller Board and VGA Display Controller.  These are accessed
  via the one UART port, UART0.

  In the default MakerLisp configuration.  These boards are connected as
  follows:

  1. VGA display controller connections (UART0 TX)

     Board interface header
     5  – 5V regulated power input
     RX – VGA Display Controller serial input
     C  – VGA Display Controller ready output
     TX – VGA Display Controller serial output
     G  – GND

     Connections:

     a. 5V '5' pin on VGA board to expansion board power distribution 5V.
     b. Ground 'G' pin on VGA board to expansion board power distribution
        ground.
     c. Receive 'RX' pin on VGA board to expansion board GPIO PD0 (TXD0).
     d. Communication, terminal ready indicator 'C' pin on VGA board to
        expansion board GPIO PB1.
     e. Transmit 'TX' pin on VGA board to USB keyboard controller 'R'

     To use the VGA display controller with stdout and stderr, you also
     need to selection CONFIG_MAKERLISP_VGA=y in your configuration.  This
     enables a required VGA initialization sequence.

  2. USB keyboard controller (UART0 RX)

     Board interface header

     5 – 5V regulated power input
     R – USB Keyboard Controller serial input
     T – USB Keyboard Controller serial output
     G – GND

     Connections:

     a. 5V '5' pin on USB board to (other) expansion board power
        distribution 5V.
     b. Ground 'G' pin on USB board to (other) expansion board power
        distribution ground.
     c. Receive 'R' pin on USB board to VGA board 'TX' (see above).
     d. Transmit 'T' pin on USB board to expansion board GPIO PD1 (RXD0).

     If your keyboard does not seem to be doing anything, check the 'RX'
     jumper on the expansion board. For input from a USB keyboard, and NOT
     the USB/UART connection, you want this jumper REMOVED, not bridging the
     two header pins front to back.

  The PC terminal software should be configured as described in the MakerLisp
  Putty HOWTO document:  115200N1 BAUD.

Default Serial Console
----------------------

  UART0 is the default serial console in all configurations unless
  otherwise noted in the description of the configuration.

LEDs and Buttons
================

LEDs
----

  Three LEDs are available on the CPU Card, but none are available for
  general use by applications:

  D2 RED:    CPU Card power.  Not under eZ80 control
  D3 GREEN:  Driven by CPU GPI/O pin.  However, it has some additional
             properties:

             1. On input, it will be '1' if the I/O expansion board is
                present.
             2. Setting it to an output of '0' will generate a system reset.
             3. Setting it to an output of '1' will not only illuminate the
                LED take the card out of reset and enable power to the SD
                card slot.

             As a consequence, the GREEN LED will not be illuminated if
             SD card support or SPI is disabled.  The only effect of
             CONFIG_ARCH_LEDS is that the GREEN LED will turned off in
             the event of a crash.

  D1 AMBER:  Controlled by the on-board MCP2221A USB bridge and provides USB
             enumeration status.  Not under eZ80 control.

Buttons
-------

  The MakerLisp CPU board has no on-board buttons that can be sensed by the
  eZ80.

Configurations
==============

Common Configuration Notes
--------------------------

  1. src/ and include/

     These directories contain common logic for all MakerLisp
     configurations.

  2. Variations on the basic MakerLisp configuration are maintained
     in subdirectories.  To configure any specific configuration, do the
     following steps:

       tools/configure.sh [OPTIONS] makerlisp:<sub-directory>
       make

     Where <sub-directory> is the specific board configuration that you
     wish to build.  Use 'tools/configure.sh -h' to see the possible
     options.  Typical options are:

       -l Configure for a Linux host
       -c Configure for a Windows Cygwin host
       -g Configure for a Windows MYS2 host

     Use configure.bat instead of configure.sh if you are building in a
     native Windows environment.

     The available board-specific configurations are  summarized in the
     following paragraphs.

     When the build completes successfully, you will find this files in
     the top level nuttx directory:

     a. nuttx.hex - A loadable file in Intel HEX format
     b. nuttx.lod - A loadable file in ZDS-II binary format
     c. nuttx.map - A linker map file

  3. ZDS-II make be used to write the nuttx.lod file to FLASH.  General
     instructions:

     a. Start ZDS-II
     b. Open the project, for example, nsh/nsh.zdsproj
     c. Select Debug->Connect To Target
     d. Select Debug->Download code

     There are projects for the ZiLOG Smart Flash Programmer as well but
     these are not functional as of this writing.

  4. This configuration uses the mconf-based configuration tool.  To
     change this configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

Configuration Subdirectories
----------------------------

  nsh_flash, nsh_ram:

    These configuration build the NuttShell (NSH).  That code can be
    found in apps/system/nsh and apps/system/nshlib..  For more
    information see:  apps/system/nsh/README.txt and
    Documentation/NuttShell.html.

    NOTES:

    1. The two configurations different only in that one builds for
       execution entirely from FLASH and the other for execution entirely
       from RAM.  A bootloader of some kind is required to support such
       execution from RAM!  This difference is reflected in a single
       configuration setting:

         CONFIG_BOOT_RUNFROMFLASH=y    # Execute from flash (default)
         CONFIG_BOOT_RUNFROMEXTSRAM=y  # Execute from external SRAM

       A third configuration is possible but not formalized with its own
       defconfig file:  You can also configure the code to boot from FLASH,
       copy the code to external SRAM, and then execute from RAM.  Such a
       configuration needs the following settings in the .config file:

         CONFIG_BOOT_RUNFROMEXTSRAM=y  # Execute from external SRAM
         CONFIG_MAKERLISP_COPYTORAM=y  # Boot from FLASH but copy to SRAM

       Why execute from SRAM at all?  Because you will get MUCH better
       performance because of the zero wait state SRAM implementation.

    2. A serial console is provided on UART0.  This configuration should work
       with or without the the VGA and Keyboard adapter boards.  Normal
       connectivity is via host serial console connected through the USB
       serial console.

       With the I/O expansion board, the serial console can also be used with
       either a TTL-to-RS232 or a TTL-to-USB Serial adapter connected by CN1
       pins 59 and 61.

       The default baud setting is 115200N1.

       To use the VGA display controller with stdin, stdout and stderr, you
       also need to selection CONFIG_MAKERLISP_VGA=y in your configuration.
       This enables a required VGA initialization sequence.

       The PC terminal software should be configured as described in the
       MakerLisp Putty HOWTO document:  115200N1 BAUD.

    3. The eZ80 RTC, the procFS file system, and SD card support in included.
       The procFS file system will be auto-mounted at /proc when the board
       boots.

       The RTC can be read and set from the NSH date command.

         nsh> date
         Thu, Dec 19 20:53:29 2086
         nsh> help date
         date usage:  date [-s "MMM DD HH:MM:SS YYYY"]
         nsh> date -s "Jun 16 15:09:00 2019"
         nsh> date
         Sun, Jun 16 15:09:01 2019

       When the system boots, it will probe the SD card and create a
       block driver called mmcsd0:

         nsh> ls /dev
         /dev:
          console
          mmcsd0
          null
          ttyS0
         nsh> mount
           /proc type procfs

       The SD card can be mounted with the following NSH mount command:

         nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard
         nsh> ls /mnt
         /mnt:
          sdcard/
         nsh> mount
           /mnt/sdcard type vfat
           /proc type procfs
         nsh> ls -lR /mnt/sdcard
         /mnt/sdcard:
          drw-rw-rw-       0 System Volume Information/
         /mnt/sdcard/System Volume Information:
          -rw-rw-rw-      76 IndexerVolumeGuid
          -rw-rw-rw-      12 WPSettings.dat

       You can they use the SD card as any other file system.

         nsh> ls /mnt/sdcard
         /mnt/sdcard:
          System Volume Information/
         nsh> echo "This is a test" >/mnt/sdcard/atest.txt
         nsh> ls /mnt/sdcard
         /mnt/sdcard:
          System Volume Information/
          atest.txt
         nsh> cat /mnt/sdcard/atest.txt
         This is a test

       Don't forget to un-mount the volume before power cycling:

         nsh> mount
           /mnt/sdcard type vfat
           /proc type procfs
         nsh> umount /mnt/sdcard
         nsh> mount
           /proc type procfs

       NOTE:  The is no card detect signal so the microSD card must be
       placed in the card slot before the system is started.

    4. Debugging the RAM version

       You can debug the all RAM version using ZDS-II as follows:

       a. Connect to the debugger,
       b. Reset, Go, and Break.  This will initialize the external RAM
       c. Break and Load the nuttx.lod file
       c. Set the PC to 0x040000
       d. Single step a few times to make sure things look good, then
       e. Go

    5. Optimizations:

       - The stack sizes have not been tuned and, hence, are probably too
         large.

    STATUS:
      2019-06-16:  The basic NSH configuration appears to be fully functional
        using only the CPU and I/O expansion card.  Console is provided over
        USB.

        Added support for SPI-based SD cards, the RTC and procFS.  There are
        still a few issues at the end-of-the-day:  (1) the SD card initialization
        hangs and prevents booting, and (2) RTC does not preserve time across a
        power cycle.

      2019-06-17:  The SD initialization was due to some error in the SPI driver:
        It waits for a byte transfer to complete but it never receives the
        indication that the transfer completed.  That SPI problem has been
        fixed and now the SD card is functional.

      2019-06-18:  The RTC now appears to be fully functional.

      2019-06-26:  Renamed nsh configuration to nsh_flash.  Added nsh_ram
        configuration.  Not yet verified.

      2019-07-09:  The RAM version does not run!  I can single step through
        the initialization and all looks well, but when I "Go", the system
        crashes.  The PC is sitting at a crazy address when I break in.  I
        have not yet debugged this.

        The identical FLASH version, differing only in the selected linker
        script, works just fine.  This implies some issue with the
        configuration of SRAM for execution.

  sdboot

    This configuration implements a very simple boot loader.  In runs from
    FLASH and simply initializes the external SRAM, mounts the FAT file
    system on the SD card, and checks to see if there is a file called
    nuttx.hex on the SD card.  If so, it will load the Intel HEX file into
    memory and jump to address 0x040000.  This, of course, assumes that
    the application's reset vector resides at address 0x040000 in external
    SRAM.

    The boot loader source is located at boards/makerlisp/src/sd_main.c.

    STATUS:
      2019-06-26:  Configuration added.  Not yet verified.

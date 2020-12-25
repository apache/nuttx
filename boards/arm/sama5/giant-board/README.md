# README

  This README file describes the port of NuttX to the Groboards Giant Board
  development board. This board features the Atmel SAMA5D27 microprocessor
  as a SIP with 128KB on-chip DDR2 RAM (part number `ATSAMA5D27C-D1G`).
  See https://groboards.com/giant-board/ for further information.

  This was copied from the SAMA5D2-XULT README, and needs updating.

## Contents

  - STATUS
  - Loading Code into SRAM with J-Link
  - DRAMBOOT, AT25BOOT, SRAMBOOT
  - Running NuttX from SDRAM
  - Buttons and LEDs
  - Serial Console
  - Giant Board Configuration Options
  - Configurations

## Status

1. Most of this document is a partially corrected clone of the SAMA5D2-XULT
   README.txt and still contains errors and inconsistencies.

## Loading Code into SRAM from SD Card

There is no JTAG connector on the Giant Board. There are pads to wire up an SWD
adapter, but this has not been tested.

The way to run NuttX is to boot from an SD Card. You can download an SD Card image
or a zip file of the required files from this page:

https://www.starcat.io/starcat-nuttx/

The SD Card has to be FAT formatted, have an AT91Bootstrap binary called boot.bin,
a U-Boot binary called u-boot.bin as well as a compiled device tree for the
SAMA5D27C-D1G called `at91-sama5d27_giantboard.dtb` in the `dtbs/` folder. You can
build these yourself using the tools at

https://github.com/Groboards/giantboard-tools

The layout should look like this:

    BOOT.BIN
    uboot.env
    nuttx.bin
    u-boot.bin
    dtbs/
      at91-sama5d27_giantboard.dtb

You only need uboot.env if you want to boot automatically. See the U-Boot
documentation for instructions on how to create this file.

## Running NuttX from SDRAM

NuttX will be executed from SDRAM, and NuttX binary must reside on SD Card media.

### NuttX Configuration

In order to run from SDRAM, NuttX must be built at origin 0x20008000 in
SDRAM (skipping over SDRAM memory used by the bootloader). The following
configuration option is required:

    CONFIG_SAMA5_BOOT_SDRAM=y
    CONFIG_BOOT_RUNFROMSDRAM=y

These options tell the NuttX code that it will be booting and running from
SDRAM. In this case, the start-logic will do to things:  (1) it will not
configure the SAMA5D2 clocking. Rather, it will use the clock configuration
as set up by the bootloader. And (2) it will not attempt to configure the
SDRAM. Since NuttX is already running from SDRAM, it must accept the SDRAM
configuration as set up by the bootloader.

### Boot sequence

Reference: http://www.at91.com/linux4sam/bin/view/Linux4SAM/GettingStarted

Several pieces of software are involved to boot a Nutt5X into SDRAM. First
is the primary bootloader in ROM which is in charge to check if a valid
application is present on supported media (NOR FLASH, Serial DataFlash,
NAND FLASH, SD card).

The boot sequence of linux4SAM is done in several steps :

1. The ROM bootloader checks if a valid application is present in FLASH
   and if it is the case downloads it into internal SRAM. This program
   is usually a second level bootloader called AT91BootStrap.

2. AT91Bootstrap is the second level bootloader. It is in charge of the
   hardware configuration. It downloads U-Boot / Barebox binary from
   FLASH to SDRAM / DDRAM and starts the third level bootloader
   (U-Boot / Barebox)

  (see http://www.at91.com/linux4sam/bin/view/Linux4SAM/AT91Bootstrap).

3. The third level bootloader is either U-Boot or Barebox. The third
   level bootloader is in charge of downloading NuttX binary from FLASH,
   network, SD card, etc. It then starts NuttX.

 4. Then NuttX runs from SDRAM

### NAND FLASH Memory Map

Reference: http://www.at91.com/linux4sam/bin/view/Linux4SAM/GettingStarted

    0x0000:0000 - 0x0003:ffff: AT91BootStrap
    0x0004:0000 - 0x000b:ffff: U-Boot
    0x000c:0000 - 0x000f:ffff: U-Boot environment
    0x0010:0000 - 0x0017:ffff: U-Boot environment redundant
    0x0018:0000 - 0x001f:ffff: Device tree (DTB)
    0x0020:0000 - 0x007f:ffff: NuttX
    0x0080:0000 - end:         Available for use as a NAND file system

### Load NuttX with U-Boot on AT91 boards

Reference http://www.at91.com/linux4sam/bin/view/Linux4SAM/U-Boot

#### Preparing NuttX image

U-Boot does not support normal binary images. Instead you have to
create an nuttx.bin file. The NuttX build generates this file
automatically. Copy it to the root of the SD Card that you made,
and boot the card. The SD Card image above will automatically boot
using the nuttx.bin file. If you are using another image (the
Giant Board linux image for instance), you can hit space to enter
U-Boot, and then from the U-Boot prompt do the following:

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

## Buttons and LEDs

### Buttons

A single button, PB1, is available on the Giant Board. This is connected to the
Power Management Integrated Circuit (PMIC). It is not available to the user.

This appears to have no affect under NuttX.

You can add your own buttons, support for pollable buttons is enabled with:

    CONFIG_ARCH_BUTTONS=y

For interrupt driven buttons, add:

    CONFIG_ARCH_IRQBUTTONS=y

Program interfaces for button access are described in nuttx/include/nuttx/arch.h

There is an example that can be enabled to test button interrupts. That
example is enabled like:

    CONFIG_EXAMPLES_BUTTONS=y
    CONFIG_EXAMPLES_BUTTONS_MAX=0
    CONFIG_EXAMPLES_BUTTONS_MIN=0
    CONFIG_EXAMPLES_BUTTONS_NAME0="PB_USER"
    CONFIG_EXAMPLES_IRQBUTTONS_MAX=0
    CONFIG_EXAMPLES_IRQBUTTONS_MIN=0

### LEDs

There is an Orange LED on the Giant Board, driven by pin (PA6) labeled STATUS.
Bringing the pin high will illuminate the LED.

    ------------------------------ ------------------- -------------------------
    SAMA5D2 PIO                    SIGNAL              USAGE
    ------------------------------ ------------------- -------------------------
    PA6                            STATUS_LED          Orange LED
    ------------------------------ ------------------- -------------------------

When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
control the Orange LED as follows:

    SYMBOL              Meaning                 Orange LED
    ------------------- ----------------------- ---------
    LED_STARTED         NuttX has been started  OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF
    LED_IRQSENABLED     Interrupts enabled      OFF
    LED_STACKCREATED    Idle stack created      ON
    LED_INIRQ           In an interrupt         N/C
    LED_SIGNAL          In a signal handler     N/C
    LED_ASSERTION       An assertion failed     N/C
    LED_PANIC           The system has crashed  FLASH

Thus if the Orange LED is statically on, NuttX has successfully  booted and
is, apparently, running normally. If LED is flashing at approximately
2Hz, then a fatal error has been detected and the system has halted.

## Serial Console

The default serial console is UART1 (TX and RX on the pin connectors).
There is a TTL serial connection available on pins 14 and 15 of the J1
connector.

    ---- ------------------------ -------------
    J1   SCHEMATIC                   SAMA5D2
    PIN  NAME(s)                  PIO  FUNCTION
    ---- ------------------------ -------------
    15   UART1_RX  DBGU_UTXD1_PD3 PD3  UTXD1
    14   UART1_TX  DBGU_URXD1_PD2 PD2  URXD1
    ---- ------------------------ -------------

The other UART on the connectors (J1 and J1) is FLEXCOM4.
Terminology: FLEXCOM is the same as USART in previous SAMA5D versions.

    ---- ----------- -------------
           BOARD        SAMA5D2
    PIN    NAME       PIO  FUNCTION
    ---- ------------ -------------
    J2 4  FLEXCOM_IO1 PD21 FLEXCOM4
    J1 6  AD2         PD13 FLEXCOM4
    ---- ------------ -------------

By default, the standard UART on the connectors (FLEXCOM4) is
enabled in all of these configurations unless otherwise noted.

REVISIT: UART1 on the DBGU connect might be a better choice for the
default serial console

## Giant Board Configuration Options

CONFIG_ARCH - Identifies the arch/ subdirectory. This should
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

    CONFIG_ARCH_BOARD="giant-board" (for the Groboards Giant Board)

CONFIG_ARCH_BOARD_name - For use in C code

    CONFIG_ARCH_BOARD_GIANT_BOARD=y

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
stack in bytes. If not defined, the user task stacks will be
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
    CONFIG_SAMA5_UART0       - UART 0 (not available on the pins)
    CONFIG_SAMA5_UART1       - UART 1
    CONFIG_SAMA5_UART2       - UART 2 (not available on the pins)
    CONFIG_SAMA5_UART3       - UART 3 (not available on the pins)
    CONFIG_SAMA5_UART4       - UART 4 (not available on the pins)
    CONFIG_SAMA5_TWI0        - Two-Wire Interface 0
    CONFIG_SAMA5_TWI1        - Two-Wire Interface 1
    CONFIG_SAMA5_SDMMC0      - SD MMC card interface 0 (not available on the pins)
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
    CONFIG_SAMA5_EMAC0       - Ethernet MAC 0 (GMAC0) (not available on the pins)
    CONFIG_SAMA5_EMAC1       - Ethernet MAC 1 (GMAC1) (not available on the pins)
    CONFIG_SAMA5_LCDC        - LCD Controller (not available on the pins)
    CONFIG_SAMA5_ISI         - Image Sensor Interface (not available on the pins)
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

#### AT91SAMA5 specific device driver settings

    CONFIG_SAMA5_DBGU_SERIAL_CONSOLE - selects the DBGU
      for the console and ttyDBGU
    CONFIG_SAMA5_DBGU_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_SAMA5_DBGU_TXBUFSIZE - Characters are buffered before
       being sent. This specific the size of the transmit buffer
    CONFIG_SAMA5_DBGU_BAUD - The configure BAUD of the DBGU.
    CONFIG_SAMA5_DBGU_PARITY - 0=no parity, 1=odd parity, 2=even parity

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=0,1,2,3) or UART
           m (m=4,5) for the console and ttys0 (default is the DBGU).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent. This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART. Must be
    CONFIG_U[S]ARTn_BITS - The number of bits. Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARITY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

#### AT91SAMA5 USB Host Configuration

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

## Configurations

### Information Common to All Configurations

Each Giant Board configuration is maintained in a sub-directory and
can be selected as follow:

    tools/configure.sh giant-board:<subdir>

Before building, make sure the PATH environment variable includes the
correct path to the directory than holds your toolchain binaries.

And then build NuttX by simply typing the following. At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

The <subdir> that is provided above as an argument to the tools/configure.sh
must be is one of the following.

NOTES:

  1. These configurations use the mconf-based configuration tool. To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool. See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on the DBGU (J23).

  3. All of these configurations use the Code Sourcery for Windows toolchain
     (unless stated otherwise in the description of the configuration). That
     toolchain selection can easily be reconfigured using 'make menuconfig'.
     Here are the relevant current settings:

     Build Setup:

    CONFIG_HOST_WINDOWS=y               : Microsoft Windows
    CONFIG_WINDOWS_CYGWIN=y             : Using Cygwin or other POSIX environment

System Type -> Toolchain:

    CONFIG_ARMV7A_TOOLCHAIN_GNU_EABIW=y : GNU EABI toolchain for windows

4. The SAMA5Dx is running at 528MHz by default in these configurations.

Board Selection -> CPU Frequency

    CONFIG_SAMA5D2XULT_528MHZ=y       : Enable 528MHz operation
    CONFIG_BOARD_LOOPSPERMSEC=65775   : Calibrated on SAMA5D3-Xplained at 528MHz running from SDRAM

### Configuration Sub-directories

Summary:  Some of the descriptions below are long and wordy. Here is the
concise summary of the available Giant Board configurations:

- nsh:

    This is a basic NuttShell (NSH) configuration.

    There may be issues with some of these configurations. See the details
    for status of individual configurations.

#### Now for the gory details:

- netnsh:

  This is a network enabled configuration based on the NuttShell (NSH).
  The CDC-ECM driver is enabled, so you can plug a USB cable into the
  USB-Micro port (USB-A) and the board will appear as an CDC-ECM
  ethernet adapter.

- nsh:

  This configuration directory provide the NuttShell (NSH). This is a
  very simple NSH configuration upon which you can build further
  functionality.

    NOTES:

    1. This configuration uses the UART1 (PD2 and PD3) for the serial
       console. USART1 is available at the "DBGU" RS-232 connector (J24).
       This is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as the
       console device.

    2. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain (such as the ARM supported
       toolchain). Both the build environment and the toolchain
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
       buffer in RAM. This feature is discussed fully in this Wiki page:
       http://nuttx.org/doku.php?id=wiki:howtos:syslog . Relevant
       configuration settings are summarized below:

       File System:

       Device Drivers:
       CONFIG_RAMLOG=y             : Enable the RAM-based logging feature.
       CONFIG_RAMLOG_SYSLOG=y      : This enables the RAM-based logger as the
                                     system logger.
       CONFIG_RAMLOG_NONBLOCKING=y : Needs to be non-blocking for dmesg
       CONFIG_RAMLOG_BUFSIZE=16384 : Buffer size is 16KiB

       NOTE: This RAMLOG feature is really only of value if debug output
       is enabled. But, by default, no debug output is disabled in this
       configuration. Therefore, there is no logic that will add anything
       to the RAM buffer. This feature is configured and in place only
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
       SDRAM from SD card U-Boot. Data also is positioned in SDRAM.

       Booting with U-Boot from nuttx.bin on an SD card is the only boot
       method that has been tested. These are the commands that I used to boot NuttX
       from the SD card:

         U-Boot> fatload mmc 0 0x20008000 nuttx.bin
         U-Boot> go 0x20008040

    6. This configuration supports /dev/null, /dev/zero, and /dev/random.

         CONFIG_DEV_NULL=y    : Enables /dev/null
         CONFIG_DEV_ZERO=y    : Enabled /dev/zero

       Support for /dev/random is implemented using the SAMA5D2's True
       Random Number Generator (TRNG). See the section above entitled
       "TRNG and /dev/random" for information about configuring /dev/random.

        CONFIG_SAMA5_TRNG=y   : Enables the TRNG peripheral
        CONFIG_DEV_RANDOM=y   : Enables /dev/random

    7. This configuration has support for NSH built-in applications enabled.
       No built-in applications are enabled, however.

    8. This configuration has support for the FAT and PROCFS file
       systems built in.

       The FAT file system includes long file name support. Please be aware
       that Microsoft claims patents against the long file name support (see
       more discussion in the top-level COPYING file).

         CONFIG_FS_FAT=y        : Enables the FAT file system
         CONFIG_FAT_LCNAMES=y   : Enable lower case 8.3 file names
         CONFIG_FAT_LFN=y       : Enables long file name support
         CONFIG_FAT_MAXFNAME=32 : Arbitrarily limits the size of a path
                                  segment name to 32 bytes

       The PROCFS file system is enabled simply with:

         CONFIG_FS_PROCFS=y     : Enable PROCFS file system

    9. The Real Time Clock/Calendar (RTC) is enabled in this configuration.
       See the section entitled "RTC" above for detailed configuration
       settings.

       The RTC alarm is not enabled by default since there is nothing in
       this configuration that uses it. The alarm can easily be enabled,
       however, as described in the "RTC" section.

       The time value from the RTC will be used as the NuttX system time
       in all timestamp operations. You may use the NSH 'date' command
       to set or view the RTC as described above in the "RTC" section.

       NOTE:  If you want the RTC to preserve time over power cycles, you
       will need to install a battery in the battery holder (J12) and close
       the jumper, JP13.

- sdmmcnsh:

    This is a configuration based on the NuttShell (NSH). The SDMMC
    peripheral is enabled, and can read and write to a VFAT filesystem
    on the SD Card.

    NuttX will mount the SD Card at `/mnt/mmcsd1`.

- sdmmc-net-nsh:

    This is a combination of the netnsh and sdmmcnsh configurations.

README
^^^^^^

README for NuttX port to the Stellaris LMS38962 Evaluation Kit

Contents
^^^^^^^^

  Stellaris LMS38962 Evaluation Kit
  OLED
  USB Device Controller Functions
  Stellaris LM3S8962 Evaluation Kit Configuration Options
  Configurations

Stellaris LMS38962 Evaluation Kit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Stellaris LM3S8962 Evaluation Board includes the following features:

 o  Stellaris LM3S8962 microcontroller with fully-integrated 10/100 embedded
    Ethernet controller
 o  Simple setup; USB cable provides serial communication, debugging, and
    power
 o  OLED graphics display with 128 x 96 pixel resolution
 o  User LED, navigation switches, and select pushbuttons
 o  Magnetic speaker
 o  LM3S8962 I/O available on labeled break-out pads
 o  Standard ARM® 20-pin JTAG debug connector with input and output modes
 o  USB interface for debugging and power supply
 o  MicroSD card slot

Features of the LM3S8962 Microcontroller

 o  32-bit RISC performance using ARM® Cortex™-M3 v7M architecture
    – 50-MHz operation
    – Hardware-division and single-cycle-multiplication
    – Integrated Nested Vectored Interrupt Controller (NVIC)
    – 42 interrupt channels with eight priority levels
 o  256 KB single-cycle flash
 o  64 KB single-cycle SRAM
 o  Four general-purpose 32-bit timers
 o  Integrated Ethernet MAC and PHY
 o  Three fully programmable 16C550-type UARTs
 o  Four 10-bit channels (inputs) when used as single-ended inputs
 o  Two independent integrated analog comparators
 o  Two I2C modules
 o  Three PWM generator blocks
    – One 16-bit counter
    – Two comparators
    – Produces two independent PWM signals
    – One dead-band generator
 o  Two QEI modules with position integrator for tracking encoder position
 o  0 to 42 GPIOs, depending on user configuration
 o  On-chip low drop-out (LDO) voltage regulator

GPIO Usage

PIN SIGNAL      EVB Function
--- ----------- ---------------------------------------
 26 PA0/U0RX    Virtual COM port receive
 27 PA1/U0TX    Virtual COM port transmit
 10 PD0/IDX0    SD card chip select
 11 PD1/PWM1    Sound
 30 PA4/SSI0RX  SD card data out
 31 PA5/SSI0TX  SD card and OLED display data in
 28 PA2/SSI0CLK SD card and OLED display clock
 22 PC7/PHB0    OLED display data/control select
 29 PA3/SSI0FSS OLED display chip select
 73 PE1/PWM5    Down switch
 74 PE2/PHB1    Left switch
 72 PE0/PWM4    Up switch
 75 PE3/PHA1    Right switch
 61 PF1/IDX1    Select switch
 47 PF0/PWM0    User LED
 23 PC6/CCP3    Enable +15 V

OLED
^^^^

  The Evaluation Kit includes an OLED graphics display. Features:

  - RiT P14201 series display
  - 128 columns by 96 rows
  - 4-bit, 16-level gray scale.
  - High-contrast (typ. 500:1)
  - Excellent brightness (120 cd/m2)
  - Fast 10 us response.

  The OLED display has a built-in controller IC with synchronous serial and
  parallel interfaces (SSD1329). Synchronous serial (SSI) is used on the EVB.
  The SSI port is shared with the microSD card slot.

  - PC7: OLED display data/control select (D/Cn)
  - PA3: OLED display chip select (CSn)

  NOTE:  Newer versions of the LM3S8962 Evaluation Kit has an OSAM 128x64x4 OLED
  display.  Some tweaks to drivers/lcd/p14201.c would be required to support that
  LCD.

USB Device Controller Functions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  Device Overview

    An FT2232 device from Future Technology Devices International Ltd manages
    USB-to-serial conversion. The FT2232 is factory configured by Luminary
    Micro to implement a JTAG/SWD port (synchronous serial) on channel A and
    a Virtual COM Port (VCP) on channel B. This feature allows two simultaneous
    communications links between the host computer and the target device using
    a single USB cable. Separate Windows drivers for each function are provided
    on the Documentation and Software CD.

  Debugging with JTAG/SWD

    The FT2232 USB device performs JTAG/SWD serial operations under the control
    of the debugger or the Luminary Flash Programmer.  It also operate as an
    In-Circuit Debugger Interface (ICDI), allowing debugging of any external
    target board.  Debugging modes:

    MODE DEBUG FUNCTION       USE                         SELECTED BY
    1    Internal ICDI        Debug on-board LM3S8962     Default Mode
                              microcontroller over USB
                              interface.
    2    ICDI out to JTAG/SWD The EVB is used as a USB    Connecting to an external
         header               to SWD/JTAG interface to    target and starting debug
                              an external target.         software. The red Debug Out
                                                          LED will be ON.
    3    In from JTAG/SWD     For users who prefer an     Connecting an external
         header               external debug interface    debugger to the JTAG/SWD
                              (ULINK, JLINK, etc.) with   header.
                              the EVB.

  Virtual COM Port

    The Virtual COM Port (VCP) allows Windows applications (such as HyperTerminal)
    to communicate with UART0 on the LM3S8962 over USB. Once the FT2232 VCP
    driver is installed, Windows assigns a COM port number to the VCP channel.

Stellaris LM3S8962 Evaluation Kit Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lm

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LM3S8962

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=lm3s8962-ek (for the Stellaris LM3S8962 Evaluation Kit)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_LM3S8962EK

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x00010000 (64Kb)

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x20000000

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
        stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

  There are configurations for disabling support for interrupts GPIO ports.
  GPIOJ must be disabled because it does not exist on the LM3S8962.
  Additional interrupt support can be disabled if desired to reduce memory
  footprint.

    CONFIG_TIVA_GPIOA_IRQS=y
    CONFIG_TIVA_GPIOB_IRQS=y
    CONFIG_TIVA_GPIOC_IRQS=y
    CONFIG_TIVA_GPIOD_IRQS=y
    CONFIG_TIVA_GPIOE_IRQS=y
    CONFIG_TIVA_GPIOF_IRQS=y
    CONFIG_TIVA_GPIOG_IRQS=y
    CONFIG_TIVA_GPIOH_IRQS=y
    CONFIG_TIVA_GPIOJ_IRQS=n << Always

  LM3S8962 specific device driver settings

    CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn for the
       console and ttys0 (default is the UART0).
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_UARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_UARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_UARTn_2STOP - Two stop bits

    CONFIG_TIVA_SSI0 - Select to enable support for SSI0
    CONFIG_TIVA_SSI1 - Select to enable support for SSI1
    CONFIG_SSI_POLLWAIT - Select to disable interrupt driven SSI support.
      Poll-waiting is recommended if the interrupt rate would be to
      high in the interrupt driven case.
    CONFIG_SSI_TXLIMIT - Write this many words to the Tx FIFO before
      emptying the Rx FIFO.  If the SPI frequency is high and this
      value is large, then larger values of this setting may cause
      Rx FIFO overrun errors.  Default: half of the Tx FIFO size (4).

    CONFIG_TIVA_ETHERNET - This must be set (along with CONFIG_NET)
      to build the Stellaris Ethernet driver
    CONFIG_TIVA_ETHLEDS - Enable to use Ethernet LEDs on the board.
    CONFIG_TIVA_BOARDMAC - If the board-specific logic can provide
      a MAC address (via tiva_ethernetmac()), then this should be selected.
    CONFIG_TIVA_ETHHDUPLEX - Set to force half duplex operation
    CONFIG_TIVA_ETHNOAUTOCRC - Set to suppress auto-CRC generation
    CONFIG_TIVA_ETHNOPAD - Set to suppress Tx padding
    CONFIG_TIVA_MULTICAST - Set to enable multicast frames
    CONFIG_TIVA_PROMISCUOUS - Set to enable promiscuous mode
    CONFIG_TIVA_BADCRC - Set to enable bad CRC rejection.
    CONFIG_TIVA_DUMPPACKET - Dump each packet received/sent to the console.

Configurations
^^^^^^^^^^^^^^

Each Stellaris LM3S8962 Evaluation Kit configuration is maintained in a
sub-directory and can be selected as follow:

    tools/configure.sh lm3s8962-ek:<subdir>

Where <subdir> is one of the following:

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. This configuration uses Cygwin under Windows with the NuttX buildroot
       tool chain.  This is eaily change by modifying the following:

         CONFIG_HOST_WINDOWS=y
         CONFIG_WINDOWS_CYGWIN=y
         CONFIG_ARM_TOOLCHAIN_BUILDROOT=y

    3. The configuration enables both the serial and telnetd NSH interfaces.

    4. As it is configured now, you MUST have a network connected. Otherwise,
       the NSH prompt will not come up because the Ethernet driver is waiting
       for the network to come up.  That is probably a bug in the Ethernet
       driver behavior!

  nx
    An example using the NuttX graphics system (NX).  This example
    focuses on general window controls, movement, mouse and keyboard
    input.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. This configuration uses Cygwin under Windows with the NuttX buildroot
       tool chain.  This is eaily change by modifying the following:

         CONFIG_HOST_WINDOWS=y
         CONFIG_WINDOWS_CYGWIN=y
         CONFIG_ARM_TOOLCHAIN_BUILDROOT=y

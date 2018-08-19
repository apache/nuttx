README
^^^^^^

README for NuttX port to the Stellaris EKK-LM3S9B96 Evaluation Kit

Contents
^^^^^^^^

  Stellaris EKK-LM3S9B96 Evaluation Kit
  Stellaris EKK-LM3S9B96 Evaluation Kit Configuration Options
  Configurations

Stellaris EKK-LM3S9B96 Evaluation Kit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The EKK-LM3S9B96 evaluation kit provides the following features:

 o LM3S9B96 high-performance Stellaris microcontroller and large memory
    – 32-bit ARM® Cortex™-M3 core
    – 256 KB single-cycle Flash memory, 96 KB single-cycle SRAM, 23.7 KB single-cycle ROM
 o Ethernet 10/100 port with two LED indicators
 o USB 2.0 Full-Speed OTG port
 o SAFERTOS™ operating system in microcontroller ROM
 o Virtual serial communications port capability
 o Oversized board pads for GPIO access
 o User pushbutton and LED
 o Detachable ICDI board can be used for debugging other Luminary Micro boards
 o Easy to customize

Features of the LM3S9B96 Microcontroller

 o  ARM® Cortex™-M3  architecture
    – 80-MHz operation
    – ARM Cortex SysTick Timer
    – Integrated Nested Vectored Interrupt Controller (NVIC)
 o  External Peripheral Interface (EPI)
 o  256 KB single-cycle flash
 o  96 KB single-cycle SRAM
 o  Four general-purpose 32-bit timers
 o  Integrated Ethernet MAC and PHY
 o  Three fully programmable 16C550-type UARTs
 o  Two 10-bit channels (inputs) when used as single-ended inputs
 o  Three independent integrated analog comparators
 o  Two CAN modules
 o  Two I2C modules
 o  Two SSI modules
 o  Two Watchdog Timers (32-bit)
 o  Three PWM generator blocks
    – One 16-bit counter
    – Two comparators
    – Produces eight independent PWM signals
    – One dead-band generator
 o  Two QEI modules with position integrator for tracking encoder position
 o  Up to 65 GPIOs, depending on user configuration
 o  On-chip low drop-out (LDO) voltage regulator

GPIO Usage

PIN SIGNAL      EVB Function
--- ----------- ---------------------------------------
 26 PA0/U0RX      Virtual COM port receive
 27 PA1/U0TX      Virtual COM port transmit
 66 PB0/USB0ID    USBID signal from the USB-On-the-Go
 67 PB1/USB0VBUS  USB VBUS input signal from USB-OTG
 92 PB4/GPIO      User pushbutton SW2.
 80 PC0/TCK/SWCLK JTAG or SWD clock input
 79 PC1/TMS/SWDIO JTAG TMS input or SWD bidirectional signal SWDIO
 78 PC2/TDI       JTAG TDI signal input
 77 PC3/TDO/SWO   JTAG TDO output or SWD trace signal SWO output.
 10 PD0/GPIO      User LED
 60 PF2/LED1      Ethernet LED1 (yellow)
 59 PF3/LED0      Ethernet LED0 (green)
 83 PH3/USB0EPEN  USB-OTG power switch
 76 PH4/USB0PFLT  Overcurrent input status from USB-OTG power switch

Stellaris EKK-LM3S9B96 Evaluation Kit Configuration Options
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

       CONFIG_ARCH_CHIP_LM3S9B96

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=ekk-lm3s9b96 (for the Stellaris EKK-LM3S9b96 Evaluation Kit)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_EKKLM3S9B96

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x00018000 (96Kb)

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
  GPIOJ must be disabled because it does not exist on the LM3S9B96.
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

  LM3S9B96 specific device driver settings

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

Each Stellaris EKK-LM3S9b96 Evaluation Kit configuration is maintained in a
sub-directory and can be selected as follow:

    tools/configure.sh ekk-lm3s9b96/<subdir>

Where <subdir> is one of the following:

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables both the serial and telnetd NSH interfaces.

    NOTE: As it is configured now, you MUST have a network connected.
    Otherwise, the NSH prompt will not come up because the Ethernet
    driver is waiting for the network to come up.  That is probably
    a bug in the Ethernet driver behavior!


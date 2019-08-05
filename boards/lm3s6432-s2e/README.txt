README
^^^^^^

README for NuttX port to the Stellaris RDK-S2E Reference Design Kit and
the MDL-S2E Ethernet to Serial module.

Contents
^^^^^^^^

  Stellaris RDK-S2E Reference Design Kit
  Stellaris MDL-S2E Reference Design Configuration Options
  Configurations

Stellaris RDK-S2E Reference Design Kit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Stellaris RDK-S2E Reference Design Kit includes the following features:

 o  MDL-S2E Ethernet to serial module
 o  LM3S6432 in a 10 x 10 mm BGA package for reduced board size
 o  10/100 Mbit Ethernet port
 o  Auto MDI/MDIX cross-over correction
 o  Traffic and link indicators Serial ports
 o  UART ports include RTS/CTS for flow control
 o  UART0 has RS232 levels, transceiver runs at up to 230.4 Kbaud
 o  UART1 has CMOS/TTL levels, can run at 1.0 Mbaud

Features of the LM3S6432 Microcontroller

 o  32-bit RISC performance using ARM® Cortex™-M3 v7M architecture
    - 50-MHz operation
    - Hardware-division and single-cycle-multiplication
    - Integrated Nested Vectored Interrupt Controller (NVIC)
    - 42 interrupt channels with eight priority levels
 o  96 KB single-cycle flash
 o  32 KB single-cycle SRAM
 o  Three general-purpose 32-bit timers
 o  Integrated Ethernet MAC and PHY
 o  Two fully programmable 16C550-type UARTs
 o  Three 10-bit channels (inputs) when used as single-ended inputs
 o  Two independent integrated analog comparators
 o  One I2C module
 o  One PWM generator block
    – One 16-bit counter
    – Two comparators
    – Produces two independent PWM signals
    – One dead-band generator
 o  0 to 43 GPIOs, depending on user configuration
 o  On-chip low drop-out (LDO) voltage regulator

GPIO Usage

PIN SIGNAL            Function
--- ----------------- ---------------------------------------
 L3 PA0/U0RX          UART0 receive
 M3 PA1/U0TX          UART0 transmit
E12 PB0/U0CTS         UART0 CTS
D12 PB1/U0RTS         UART0 RTS
 L5 PA4/SPIRX         SPI receive (pin hardwired to U1RX)
 M5 PA5/SPITX         SPI transmit (pin hardwired to U1TX)
 H2 PD2/U1RX          UART1 receive
 H1 PD3/U1TX          UART1 transmit
 L4 PA3/U1CTS/SPICLK  UART1 CTS or SPI clock
 M4 PA2/U1RTS/SPISEL  UART1 RTS or SPI slave select
J11 PF0/LED1          Ethernet LED1 (green)
J12 PF1/LED0          Ethernet LED0 (yellow)
C11 PB2               Transciever #INVALID
C12 PB3               Transciever #ENABLE
 A6 PB4               Transciever ON
 B7 PB5               Transciever #OFF

Stellaris MDL-S2E Reference Design Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

       CONFIG_ARCH_CHIP_LM3S6432

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=lm3s6432-s2e (for the Stellaris MDL-S2E Reference Design)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_LM3S6432S2E

    CONFIG_ARCH_LOOPSPERMSEC - As supplied, calibrated for correct operation
       of delay loops assuming 50MHz CPU frequency.

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
  GPIOH and GPIOJ must be disabled because they do not exist on the LM3S6432.
  Additional interrupt support can be disabled if desired to reduce memory
  footprint - GPIOs C-G are not pinned out on the MDL-S2E board.

    CONFIG_TIVA_GPIOA_IRQS=y
    CONFIG_TIVA_GPIOB_IRQS=y
    CONFIG_TIVA_GPIOC_IRQS=n << Always
    CONFIG_TIVA_GPIOD_IRQS=n << Always
    CONFIG_TIVA_GPIOE_IRQS=n << Always
    CONFIG_TIVA_GPIOF_IRQS=n << Always
    CONFIG_TIVA_GPIOG_IRQS=n << Always
    CONFIG_TIVA_GPIOH_IRQS=n << Always
    CONFIG_TIVA_GPIOJ_IRQS=n << Always

  LM3S6432 specific device driver settings

    CONFIG_UARTn_DISABLE
       The TX and RX pins for UART1 share I/O pins with the TX and RX pins
       for SSI0.  To avoid conflicts, only one of SSI0 and UART1 should
       be enabled in a configuration.
    CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn for the
       console and ttys0 (default is UART1).
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_UARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_UARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_UARTn_2STOP - Two stop bits

    CONFIG_TIVA_SSI0 - Select to enable support for SSI0
      The TX and RX pins for SSI0 share I/O pins with the TX and RX pins
      for UART1.  To avoid conflicts, only one of SSI0 and UART1 should
      be enabled in a configuration.
    CONFIG_TIVA_SSI1 - Select to enable support for SSI1
      Note that the LM3S6432 only has one SSI, so SSI1 should always be
      disabled.
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
    CONFIG_TIVA_BOARDMAC - This should be set in order to use the
      MAC address configured in the flash USER registers.
    CONFIG_TIVA_ETHHDUPLEX - Set to force half duplex operation
    CONFIG_TIVA_ETHNOAUTOCRC - Set to suppress auto-CRC generation
    CONFIG_TIVA_ETHNOPAD - Set to suppress Tx padding
    CONFIG_TIVA_MULTICAST - Set to enable multicast frames
    CONFIG_TIVA_PROMISCUOUS - Set to enable promiscuous mode
    CONFIG_TIVA_BADCRC - Set to enable bad CRC rejection.
    CONFIG_TIVA_DUMPPACKET - Dump each packet received/sent to the console.

Configurations
^^^^^^^^^^^^^^

Each Stellaris MDL-S2E Reference Design configuration is maintained in a
sub-directory and can be selected as follow:

    tools/configure.sh lm3s6432-s2e/<subdir>

Where <subdir> is one of the following:

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables both the serial and telnetd NSH interfaces.

    NOTE: As it is configured now, you MUST have a network connected.
    Otherwise, the NSH prompt will not come up because the Ethernet
    driver is waiting for the network to come up.  That is probably
    a bug in the Ethernet driver behavior!

README
^^^^^^

README file for the Microment Eagle100 NuttX port.

References:
^^^^^^^^^^

  Micromint: http://www.micromint.com/
  Luminary:  http://www.luminarymicro.com/

Ethernet-Bootloader
^^^^^^^^^^^^^^^^^^^

  Here are some notes about using the Luminary Ethernet boot-loader built
  into the Eagle-100 board.

  Built-In Application:

  - The board has no fixed IP address but uses DHCP to get an address.
    I used a D-link router; I can use a web browser to surf to the D-link
    web page to get the address assigned by

  - Then you can use this IP address in your browser to surf to the Eagle-100
    board.  It presents several interesting pages -- the most important is
    the page called "Firmware Update".  That page includes instructions on
    how to download code to the Eagle-100.

  - After you burn the first program, you lose this application.  Then you
    will probably be better off connected directly to the Eagle-100 board
    or through a switch (The router caused problems for me during downloads).

  Using the Ethernet Bootloader:

  - You will need the "LM Flash Programmer application".  You can get that
    program from the Luminary web site.  There is a link on the LM3S6918 page.

  - Is there any documentation for using the bootloader?  Yes and No:  There
    is an application note covering the bootloader on the Luminary site, but
    it is not very informative.  The Eagle100 User's Manual has the best
    information.

  - Are there any special things I have to do in my code, other than setting
    the origin to 0x0000:2000 (APP_START_ADDRESS)?  No.  The bootloader assumes
    that you have a vector table at that address .  The bootloader does the
    following each time it boots (after you have downloaded the first valid
    application):

    o The bootloader sets the vector table register to the APP_START_ADDRESS,
    o It sets the stack pointer to the address at APP_START_ADDRESS, and then
    o Jumps to the address at APP_START_ADDRESS+4.

  - You can force the bootloader to skip starting the application and stay
    in the update mode.  You will need to do this in order to download a new
    application.  You force the update mode by holding the user button on the
    Eagle-100 board while resetting the board.  The user button is GPIOA, pin 6
    (call FORCED_UPDATE_PIN in the bootloader code).

  - Note 1:  I had to remove my D-Link router from the configuration in order
    to use the LM Flash Programmer (the Bootloader issues BOOTP requests to
    communicate with the LM Flash Programmer, my router was responding to
    these BOOTP requests and hosing the download).  It is safer to connect
    via a switch or via an Ethernet switch.

  - Note 2:  You don't need the router's DHCPD server in the download
    configuration; the Luminary Flash Programmer has the capability of
    temporarily assigning the IP address to the Eagle-100 via BOOTP.

Eagle100-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

       CONFIG_ARCH_CHIP_LM3S6918

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=eagle100 (for the MicroMint Eagle-100 development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_EAGLE100

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
  GPIOH and GPIOJ must be disabled because they do not exist on the LM3S6918.
  Additional interrupt support can be disabled if desired to reduce memory
  footprint.

    CONFIG_TIVA_GPIOA_IRQS=y
    CONFIG_TIVA_GPIOB_IRQS=y
    CONFIG_TIVA_GPIOC_IRQS=y
    CONFIG_TIVA_GPIOD_IRQS=y
    CONFIG_TIVA_GPIOE_IRQS=y
    CONFIG_TIVA_GPIOF_IRQS=y
    CONFIG_TIVA_GPIOG_IRQS=y
    CONFIG_TIVA_GPIOH_IRQS=n << Always
    CONFIG_TIVA_GPIOJ_IRQS=n << Always

  LM3S6918 specific device driver settings

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

    CONFIG_TIVA_SSI0 - Select to ensable support for SSI0
    CONFIG_TIVA_SSI1 - Select to ensable support for SSI1
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

Common Configuration Notes
--------------------------

  1. Each Eagle-100 configuration is maintained in a sub-directory and
     can be selected as follow:

       tools/configure.sh eagle100:<subdir>

     Where <subdir> is one of the configuration sub-directories described in
     the following paragraph.

  2. These configurations use the mconf-based configuration tool.  To
     change a configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

Configuration Sub-Directories
-----------------------------

  nettest:
    This configuration directory may be used to enable networking using the
    LM3S6918's Ethernet controller. It uses examples/nettest to exercise the
    TCP/IP network.

    NOTES:

    1. This configuration is set to use Cygwin under Windows and the ARM
       EABI GCC toolchain.  That, however, is easily reconfigurable:

         CONFIG_HOST_WINDOWS=y
         CONFIG_WINDOWS_CYGWIN=y
         CONFIG_ARM_TOOLCHAIN_GNU_EABI=y

  httpd:
    This builds the uIP web server example using the examples/webserver application
    (for execution from FLASH).

    NOTES:

    1. This configuration is set to use Cygwin under Windows and the ARM
       EABI GCC toolchain.  That, however, is easily reconfigurable:

         CONFIG_HOST_WINDOWS=y
         CONFIG_WINDOWS_CYGWIN=y
         CONFIG_ARM_TOOLCHAIN_GNU_EABI=y

       This example can only be built using the buildroot toolchain
       with NXFLAT support

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables only the serial NSH interfaces (the telnet
    interface should also be functional, but is not enabled in this
    configuration).

    NOTES:

    1. This configuration is set to use Cygwin under Windows and the
       devkitARM toolchain.  That, however, is easily reconfigurable:

       CONFIG_HOST_WINDOWS=y
       CONFIG_WINDOWS_CYGWIN=y
       CONFIG_ARM7M_TOOLCHAIN_DEVKITARM=y

  nxflat:
    This builds the NXFLAT example at apps/examples/nxfalt.

    NOTES:

    1. This example can only be built using the NuttX buildroot
       toolchain with the NXFLAT tools.

    2. This configuration is set to use Cygwin under Windows and the
       devkitARM toolchain.  That, however, is easily reconfigurable:

       CONFIG_HOST_WINDOWS=y
       CONFIG_WINDOWS_CYGWIN=y
       CONFIG_ARM7M_TOOLCHAIN_DEVKITARM=y

  thttpd:
    This builds the THTTPD web server example using the THTTPD and
    the apps/examples/thttpd application.

    NOTES:

    1. This configuration is set to use Linux and the buildroot toolchain.
       That, however, is easily reconfigurable:

         CONFIG_HOST_LINUX=y
         CONFIG_ARM7M_ARM_TOOLCHAIN_BUILDROOT=y

       This example can only be built using the buildroot toolchain
       with NXFLAT support

  By default, all of these examples are built to be used with the Luminary
  Ethernet Bootloader (you can change the ld.script file in any of these
  sub-directories to change that configuration).

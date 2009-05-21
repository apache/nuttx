README
^^^^^^

Toolchain
^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/project/showfiles.php?group_id=189573).

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh eagle100/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-4.3.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

Ethernet-Bootloader
^^^^^^^^^^^^^^^^^^^

  Here are some notes about using the Luminary Ethernet boot-loader built
  into the Eagle-100 board.

  - The board has no fixed IP address but uses DHCP to get an address.
    I use a D-link router; I can use a web browser to surf to the D-link
    web page to get the address assigned by 

  - Then you can use this IP address in your browser to surf to the Eagle-100
    board.  It presents several interesting pages -- the most important is
    the page called "Firmware Update".  That page includes instructions on
    how to download code to the Eagle-100.

  - You will need the "LM Flash Programmer application".  You can get that
    program from the Luminary web site.  There is a link on the LM3S6918 page.

  - Is there any documentation for using the bootloader?  Yes and No:  There
    is an application note covering the bootloader on the Luminary site, but
    it is not very informative.

  - Are there any special things I have to do in my code, other than setting 
    the origin to 0x0000:2000 (APP_START_ADDRESS)?  No.  The bootload assumes
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

	   CONFIG_ARCH_CHIP=lm3s

	CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
	   chip:

	   CONFIG_ARCH_CHIP_LM3S6918

	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=eagle100 (for the MicroMint Eagle-100 development board)

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_EAGLE100

	CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
	   of delay loops

	CONFIG_ENDIAN_BIG - define if big endian (default is little
	   endian)

	CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

	   CONFIG_DRAM_SIZE=0x00010000 (64Kb)

	CONFIG_DRAM_START - The start address of installed DRAM

	   CONFIG_DRAM_START=0x20000000

	CONFIG_DRAM_END - Last address+1 of installed RAM

	   CONFIG_DRAM_END=(CONFIG_DRAM_START+CONFIG_DRAM_SIZE)

	CONFIG_ARCH_IRQPRIO - The LM3S6918 supports interrupt prioritization

	   CONFIG_ARCH_IRQPRIO=y

	CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
	   have LEDs

	CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
	   stack. If defined, this symbol is the size of the interrupt
	    stack in bytes.  If not defined, the user task stacks will be
	  used during interrupt handling.

	CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

	CONFIG_ARCH_BOOTLOADER - Configure to use the MicroMint Eagle-100
	   Ethernet bootloader.

	CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

	CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
	   cause a 100 second delay during boot-up.  This 100 second delay
	   serves no purpose other than it allows you to calibratre
	   CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
	   the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
	   the delay actually is 100 seconds.

  LM3S6818 specific device driver settings

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

	CONFIG_LM3S_ETHERNET - This must be set (along with CONFIG_NET)
	  to build the LM3S Ethernet driver
	CONFIG_LM3S_ETHLEDS - Enable to use Ethernet LEDs on the board.
	CONFIG_LM3S_BOARDMAC - If the board-specific logic can provide
	  a MAC address (via lm3s_ethernetmac()), then this should be selected.

Configurations
^^^^^^^^^^^^^^

Each Eagle-100 configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh eagle100/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

ostest
^^^^^^

This configuration directory, performs a simple OS test using
examples/ostest.

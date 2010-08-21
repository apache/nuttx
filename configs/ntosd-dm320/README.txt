README
^^^^^^

  This is the README file for the port of NuttX to the Neuros OSD
  v1.0 Dev Board.  References:

    http://www.neurostechnology.com/neuros-developer-community
    http://wiki.neurostechnology.com/index.php/OSD_1.0_Developer_Home
    http://wiki.neurostechnology.com/index.php/DM320_Platform_development

  There are some differences between the Dev Board and the currently
  available commercial v1.0 Boards, most notably in the amount of memory:
  8Mb FLASH and 32Mb RAM vs. 16Mb and 64Mb as in the production board.
  See the following for more information:
    
     http://wiki.neurostechnology.com/index.php/OSD_Developer_Board_v1

  NuttX operates on the ARM9EJS of this dual core processor.  The DSP
  is available and unused.

  STATUS: This port is code complete, verified, and included in the
  NuttX 0.2.1 release.

Toolchain
^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the ARM926 GCC toolchain (if
  different from the default).

  If you have no ARM toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/project/showfiles.php?group_id=189573).

  1. You must have already configured Nuttx in <some-dir>nuttx.

     cd tools
     ./configure.sh ntosd-dm320/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack

  4. cd <some-dir>/buildroot

  5. cp configs/arm-defconfig .config OR
     cp configs/arm926t_defconfig-4.2.4 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h so that the PATH variable includes the path to the
     newly built binaries.

ARM/DM320-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
	   be set to:

	   CONFIG_ARCH=arm

	CONFIG_ARCH_family - For use in C code:

	   CONFIG_ARCH_ARM=y

	CONFIG_ARCH_architecture - For use in C code:

	   CONFIG_ARCH_ARM926EJS=y

	CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

	   CONFIG_ARCH_CHIP=dm320

	CONFIG_ARCH_CHIP_name - For use in C code

	   CONFIG_ARCH_CHIP_DM320

	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=ntosd-dm320

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_NTOSD_DM320 (for the Spectrum Digital C5471 EVM)

	CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
	   of delay loops

	CONFIG_ENDIAN_BIG - define if big endian (default is little
	   endian)

	CONFIG_DRAM_SIZE - Describes the installed DRAM.

	CONFIG_DRAM_START - The start address of installed DRAM

	CONFIG_DRAM_VSTART - The startaddress of DRAM (virtual)

	CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
	   have LEDs

	CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
	   stack. If defined, this symbol is the size of the interrupt
	   stack in bytes.  If not defined, the user task stacks will be
	  used during interrupt handling.

	CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

	CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
	   cause a 100 second delay during boot-up.  This 100 second delay
	   serves no purpose other than it allows you to calibratre
	   CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
	   the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
	   the delay actually is 100 seconds.

  DM320 specific device driver settings

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

  DM320 USB Configuration

	CONFIG_DM320_GIO_USBATTACH
	   GIO that detects USB attach/detach events
	CONFIG_DM320_GIO_USBDPPULLUP
	   GIO 
	CONFIG_DMA320_USBDEV_DMA
	   Enable DM320-specific DMA support
	CONFIG_DM320_GIO_USBATTACH=6

Configurations
^^^^^^^^^^^^^^

Each Neuros OSD configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh ntosd-dm320/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

nettest
^^^^^^^

This alternative configuration directory may be used to
enable networking using the OSDs DM9000A Ethernet interface.
It uses examples/nettest to excercise the TCP/IP network.

nsh
^^^

Configures the NuttShell (nsh) located at examples/nsh.  The
Configuration enables both the serial and telnetd NSH interfaces.

ostest
^^^^^^

This configuration directory, performs a simple OS test using
examples/ostest.

poll
^^^^

This configuration exercises the poll()/select() text at
examples/poll

udp
^^^

This alternative configuration directory is similar to nettest
except that is use examples/upd to exercise UDP.

uip
^^^

This configuration file demonstrates the tiny webserver
at examples/uip.

Configuration Options
^^^^^^^^^^^^^^^^^^^^^

In additional to the common configuration options listed in the
file configs/README.txt, there are other configuration options
specific to the DM320:

 CONFIG_ARCH - identifies the arch subdirectory and, hence, the
   processor architecture.
 CONFIG_ARCH_name - for use in C code.  This identifies the
   particular chip or SoC that the architecture is implemented
   in.
 CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory
 CONFIG_ARCH_CHIP_name - For use in C code
 CONFIG_ARCH_BOARD - identifies the configs subdirectory and, hence,
   the board that supports the particular chip or SoC.
 CONFIG_ENDIAN_BIG - define if big endian (default is little endian)
 CONFIG_ARCH_BOARD_name - for use in C code
 CONFIG_BOARD_LOOPSPERMSEC - for delay loops
 CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to lpc2148.
 CONFIG_DRAM_SIZE - Describes the internal DRAM.
 CONFIG_DRAM_START - The start address of internal DRAM
 CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

DM320 specific device driver settings

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

DM320 USB Configuration

 CONFIG_DM320_GIO_USBATTACH
   GIO that detects USB attach/detach events
 CONFIG_DM320_GIO_USBDPPULLUP
   GIO connected to D+.  Support software connect/disconnect.
 CONFIG_DMA320_USBDEV_DMA
   Enable DM320-specific DMA support

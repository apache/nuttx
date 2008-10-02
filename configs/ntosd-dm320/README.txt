README
^^^^^^

Each Neuros OSD configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh ntosd-dm320/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

ostest
^^^^^^

This configuration directory, performs a simple OS test using
examples/ostest.

nsh
^^^

Configures the NuttShell (nsh) located at examples/nsh.  The
Configuration enables both the serial and telnetd NSH interfaces.

nettest
^^^^^^^

This alternative configuration directory may be used to
enable networking using the OSDs DM9000A Ethernet interface.
It uses examples/nettest to excercise the TCP/IP network.

uip
^^^

This configuration file demonstrates the tiny webserver
at examples/uip.

udp
^^^

This alternative configuration directory is similar to nettest
except that is use examples/upd to exercise UDP.

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

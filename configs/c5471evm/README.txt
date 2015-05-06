README
^^^^^^

Toolchain
^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the SH toolchain (if
  different from the default).

  If you have no ARM toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot).

  1. You must have already configured Nuttx in <some-dir>nuttx.

     cd tools
     ./configure.sh c5471evm/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack

  4. cd <some-dir>/buildroot

  5. cp configs/arm-defconfig .config

  6. make oldconfig

  7. make

  8. Edit setenv.h so that the PATH variable includes the path to the
     newly built binaries.

Issues
^^^^^^

  Title:       UART RECONFIGURATION
  Description: UART re-configuration is untested and conditionally compiled out.
  Status:      Open
  Priority:    Medium.  ttyS1 is not configured, but not used; ttyS0 is configured
               by the bootloader

ARM/C5471-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
	   be set to:

	   CONFIG_ARCH=arm

	CONFIG_ARCH_family - For use in C code:

	   CONFIG_ARCH_ARM=y

	CONFIG_ARCH_architecture - For use in C code:

	   CONFIG_ARCH_ARM7TDMI=y

	CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

	   CONFIG_ARCH_CHIP=c5471

	CONFIG_ARCH_CHIP_name - For use in C code

	   CONFIG_ARCH_CHIP_C5471

	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=c5471evm (for the Spectrum Digital C5471 EVM)

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_C5471EVM (for the Spectrum Digital C5471 EVM)

	CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
	   of delay loops

	CONFIG_ENDIAN_BIG - define if big endian (default is little
	   endian)

	CONFIG_ROM_VECTORS - should be defined for the C5471 because the
	   interrupt vectors are in ROM

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

  C5471 specific device driver settings

	CONFIG_SERIAL_IRDA_CONSOLE - selects the IRDA UART for the
	   console ant ttys0 (default is the modem UART).
	CONFIG_UART_*_HWFLOWCONTROL - enables hardware flow control
	CONFIG_UART_*_RXBUFSIZE - Characters are buffered as received.
	   This specific the size of the receive buffer
	CONFIG_UART_*_TXBUFSIZE - Characters are buffered before
	   being sent.  This specific the size of the transmit buffer
	CONFIG_UART_*_BAUD - The configure BAUD of the UART.  Must be
	CONFIG_UART_*_BITS - The number of bits.  Must be either 7 or 8.
	CONFIG_UART_*_PARTIY - 0=no parity, 1=odd parity, 2=even parity
	CONFIG_UART_*_2STOP - Two stop bits

  C5471 Ethernet Driver settings

	CONFIG_C5471_NET_STATS
	CONFIG_C5471_PHY_AC101L or C5471_PHY_LU3X31T_T64
	CONFIG_C5471_AUTONEGOTIATION
	CONFIG_C5471_BASET100
	CONFIG_C5471_BASET10

Configurations
^^^^^^^^^^^^^^

Common Configuration Notes
--------------------------

  1. Each C5471 configuration is maintained in a sub-directory and
     can be selected as follow:

       cd tools
       ./configure.sh c5471evm/<subdir>
       cd -
       . ./setenv.sh

     Where <subdir> is one of the configuration sub-directories described in
     the following paragraph.

  2. These configurations use the mconf-based configuration tool.  To
     change a configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        and misc/tools/

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

  3. By default, all configurations assume the NuttX Buildroot toolchain
     under Linux (should work under Windows with Cygwin as well).  This
     is easily reconfigured:

        CONFIG_HOST_LINUX=y
        CONFIG_ARM_TOOLCHAIN_BUILDROOT=y

Configuration Sub-Directories
-----------------------------

  nettest

    This configuration enables networking using the c5471's built-in Ethernet
    interface.  It uses examples/nettest to exercise the TCP/IP network.

  nsh

    This configuration file builds NSH (examples/nsh) using the TELNET server
    front end

  httpd

    This configuration uses the tiny webserver for uIP.

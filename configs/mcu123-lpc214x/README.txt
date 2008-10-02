README
^^^^^^

Tools
^^^^^

I use the lpc21isp tool to load NuttX into FLASH.  That tool is available
in the files section at http://tech.groups.yahoo.com/group/lpc21isp/.  In
order version 1.60 of lpc21isp for Linux, I had to make several changes.
This changes are shown in lpc21ips-1.60.diff.

I use the script lpc21isp.sh to perform the actual download.  You will
probably have to make some changes to this script in order to use it.
For example, the path to the built lpc21isp binary will most likely
have to have change.  Then move this script to the top level NuttX
directory and simply execute it to load NuttX onto the board.

Configurations
^^^^^^^^^^^^^^

Each NXP LPC214x configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh mcu123-lpc214x/<subdir>
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
Configuration enables only the serial NSH interfaces.

Configuration Options
^^^^^^^^^^^^^^^^^^^^^

In additional to the common configuration options listed in the
file configs/README.txt, there are other configuration options
specific to the LPC214x:

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

LPC2148 specific chip initialization

 CONFIG_EXTMEM_MODE, CONFIG_RAM_MODE. CONFIG_CODE_BASE, CONFIG_PLL_SETUP,
 CONFIG_MAM_SETUP, CONFIG_APBDIV_SETUP, CONFIG_EMC_SETUP,. CONFIG_BCFG0_SETUP,
 CONFIG_BCFG1_SETUP, CONFIG_BCFG2_SETUP, CONFIG_BCFG3_SETUP, CONFIG_ADC_SETUP

LPC214X UART device driver settings

 CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn for the
   console and ttys0 (default is the UART0).
 CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
   This specific the size of the receive buffer
 CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
   being sent.  This specific the size of the transmit buffer
 CONFIG_UARTn_BAUD - The configure BAUD of the UART.  Must be
 CONFIG_UARTn_BITS - The number of bits.  Must be either 7 or 8.
 CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity, 3=mark 1, 4=space 0
 CONFIG_UARTn_2STOP - Two stop bits

LPC214X USB Configuration

 CONFIG_LPC214X_USBDEV_FRAME_INTERRUPT
   Handle USB Start-Of-Frame events.
   Enable reading SOF from interrupt handler vs. simply reading on demand.
   Probably a bad idea... Unless there is some issue with sampling the SOF
   from hardware asynchronously.
 CONFIG_LPC214X_USBDEV_EPFAST_INTERRUPT
   Enable high priority interrupts.  I have no idea why you might want to
   do that
 CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS
   Number of DMA descriptors to allocate in the 8Kb USB RAM.  This is a
   tradeoff between the number of DMA channels that can be supported vs
   the size of the DMA buffers available.
 CONFIG_LPC214X_USBDEV_DMA
   Enable lpc214x-specific DMA support

Olimex STR-P711
^^^^^^^^^^^^^^^
 Features:

 - MCU: STR711FR2T6 16/32 bit ARM7TDMI™ with 256K Bytes Program Flash,
   64K Bytes RAM, USB 2.0, RTC, 12 bit ADC, 4x UARTs, 2x I2C,2x SPI,
   5x 32bit TIMERS, 2x PWM, 2x CCR, WDT, up to 50MHz operation
 - Standard JTAG connector with ARM 2x10 pin layout for programming/debugging
   with ARM-JTAG
 - USB connector
 - Two channel RS232 interface and drivers
 - SD/MMC card connector
 - Two buttons
 - Trimpot connected to ADC
 - Two status LEDs
 - Buzzer
 - UEXT - 10 pin extension connector for Olimex addon peripherials like MP3,
   RF2.4Ghz, RFID etc. modules
 - 2x SPI connectors
 - I2C connector
 - On board voltage regulator 3.3V with up to 800mA current
 - Single power supply: 6V AC or DC required, USB port can power the board
 - Power supply LED
 - Power supply filtering capacitor
 - RESET circuit
 - RESET button
 - 4 Mhz crystal oscillator
 - 32768 Hz crystal and RTC

 Power Supply

   6V AC or DC (or powered from USB port)

 GIO with on-board connections (others available for prototyping):

   SIGNAL  DESCRIPTION           PIN
   ------- --------------------- -----
   MISO1   BSPI0 to MMC/SD       P0.4
   MOSI1   "   " "" "    "       P0.5
   SCLK1   "   " "" "    "       P0.6
   SS1     "   " "" "    "       P0.7
   U0RX    UART 0                P0.8
   U0TX    "  " "                P0.9
   U1RX    UART 1                P0.10
   U1TX    "  " "                P0.11
   BUZZ    Buzzer                P0.13
   WAKE-UP Button                P0.15
   AIN0    Potentiometer (AN_TR) P1.3
   LED1    LED 1                 P1.8
   LED2    LED 2                 P1.9
   WP      MMC/SD write protect  P1.10
   USBOP   USB                   P1.11
   USBON   " "                   P1.12
   BUT     Button                P1.13
   CP      MMC/SD card present   P1.15

 Jumpers
   STNBY   Will pull pin 23 /STDBY low

Toolchain
^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the SH toolchain (if
  different from the default).

  If you have no SH toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/project/showfiles.php?group_id=189573).

  1. You must have already configured Nuttx in <some-dir>nuttx.

     cd tools
     ./configure.sh olimex-strp711/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack

  4. cd <some-dir>/buildroot

  5. cp configs/arm-defconfig .config
     or
     cp configs/arm7tdmi-defconfig-4.3.3 .config  (Last tested with this toolchain)

  6. make oldconfig

  7. make

  8. Edit setenv.h so that the PATH variable includes the path to the
     newly built binaries.

OpenOCD
^^^^^^^

For a debug environment, I am using OpenOCD with a Wiggler-clone JTAG interface.  The
following steps worked for me with a 20081028 OpenOCD snapshot.

GENERAL STEPS:

1. Check out OpenOCD

    svn checkout svn://svn.berlios.de/openocd/trunk openocd

2. Build OpenOCD

  Read the INSTALL file from the files you just downloaded.  You probably just need
  to run:

    ./bootstrap

  Then configure OpenOCD using the configure script created by ./bootstrap.

    ./configure --enable-parport

  Build OpenOCD with:

    make

  Install OpenOCD.  Since we used the default configuration the code will be
  installed at /usr/local/bin/openocd. Other files will be installed at
  /usr/local/lib/openocd (configuration files, scripts, etc.) and /usr/local/share/info
  (online documentation accessable via 'info openocd').  You need root priviledges
  to do the following:

    make install.

3. Setup

  OpenOCD reads its configuration from the file openocd.cfg in the current directory
  when started.  You have two different options:

  * Create a symbolic link named openocd.cfg to one of the configuration files in
    /usr/local/lib/openocd, or

  * Use a custom configuration file specified with the ‘-f <conf.file>’ command line
    switch opeion when starting OpenOCD.

  For the STR-P711, I have included bash scripts in the scripts sub-directory.

4. Running OpenOCD

  Make sure the ARM7TDMI board is powered and the JTAG cable is connected

  Run 'src/openocd -d' (might be required to be root) and check for any errors
  reported. The '-d' option enables debugging info.

5. Telnet interface

  telnet into port 4444 to get a command interface: 'telnet localhost 4444'

6. GDB

  start arm-elf-gdb
  type 'file <executable.elf>' to load the executable
  type 'set debug remote 1' to enable tracing of gdb protocol (if required)
  type 'target remote localhost:3333' to connect to the target
  The same commands from the telnet interface can now be accessed through the
  'monitor' command, e.g. 'monitor help'

Configurations:
---------------

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables both the serial and telnetd NSH interfaces.

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest.

STR71x-Specific Configuration Settings
--------------------------------------

  CONFIG_STR71X_I2C0, CONFIG_STR71X_I2C1, CONFIG_STR71X_UART0, CONFIG_STR71X_UART1,
  CONFIG_STR71X_UART2, CONFIG_STR71X_UART3, CONFIG_STR71X_USB, CONFIG_STR71X_CAN,
  CONFIG_STR71X_BSPI0, CONFIG_STR71X_BSPI1, CONFIG_STR71X_HDLC, CONFIG_STR71X_XTI, 
  CONFIG_STR71X_GPIO0, CONFIG_STR71X_GPIO1, CONFIG_STR71X_GPIO2, CONFIG_STR71X_ADC12, 
  CONFIG_STR71X_CKOUT, CONFIG_STR71X_TIM1, CONFIG_STR71X_TIM2, CONFIG_STR71X_TIM3, and
  CONFIG_STR71X_RTC
    Select peripherals to initialize (Timer0 and EIC are always initialized)
  CONFIG_STR71X_BANK0, CONFIG_STR71X_BANK1, CONFIG_STR71X_BANK2, and CONFIG_STR71X_BANK3
    Enable initialize of external memory banks 0-3.
  CONFIG_STR71X_BANK0_SIZE, CONFIG_STR71X_BANK1_SIZE, CONFIG_STR71X_BANK2_SIZE, and
  CONFIG_STR71X_BANK3_SIZE
    If a particular external memory bank is configured, then its width must be provided.
    8 and 16 (bits) are the only valid options.
  CONFIG_STR71X_BANK0_WAITSTATES, CONFIG_STR71X_BANK1_WAITSTATES,
  CONFIG_STR71X_BANK2_WAITSTATES, and CONFIG_STR71X_BANK3_WAITSTATES
    If a particular external memory bank is configured, then the number of waistates
    for the bank must also be provided.  Valid options are {0, .., 15}
  CONFIG_STR71X_BIGEXTMEM
    The default is to provide 20 bits of address for all external memory regions.  If
    any memory region is larger than 1Mb, then this option should be selected.  In this
    case, 24 bits of addressing will be used

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


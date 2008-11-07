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


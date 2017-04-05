README
======

  This is the README file for the port of NuttX to the Mikroe Clicker2 STM32
  board based on the STMicro STM32F407VGT6 MCU.

  Reference: https://shop.mikroe.com/development-boards/starter/clicker-2/stm32f4

Contents
========

  o Serial Console
  o LEDs
  o Buttons
  o Using JTAG
  o Configurations

Serial Console
==============

  The are no RS-232 drivers on-board.  An RS-232 Click board is available:
  https://shop.mikroe.com/click/interface/rs232 or you can cannot an off-
  board TTL-to-RS-232 converter as follows:

    USART2:  mikroBUS1 PD6/RX and PD5/TX
    USART3:  mikroBUS2 PD9/RX and PD8TX

  GND, 3.3V, and 5V.  Are also available

  By default, USART3 on mikroBUS2 is used as the serial console in each
  configuration unless stated otherwise in the description of the
  configuration.

LEDs
====

  The Mikroe Clicker2 STM32 has two user controllable LEDs:

     LD1/PE12, Active high output illuminates
     LD2/PE15, Active high output illuminates

  If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
  way.  If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs on
  board the Clicker2 for STM32.  The following definitions describe how NuttX
  controls the LEDs:

    SYMBOL               Meaning                      LED state
                                                    LD1      LD2
    -------------------  -----------------------  -------- --------
    LED_STARTED          NuttX has been started     OFF      OFF
    LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
    LED_IRQSENABLED      Interrupts enabled         OFF      OFF
    LED_STACKCREATED     Idle stack created         ON       OFF
    LED_INIRQ            In an interrupt            N/C      ON
    LED_SIGNAL           In a signal handler          No change
    LED_ASSERTION        An assertion failed          No change
    LED_PANIC            The system has crashed     OFF      Blinking
    LED_IDLE             STM32 is is sleep mode       Not used

  Thus is LD1 is illuminated, the Clicker2 has completed boot-up.  IF LD2
  is glowly softly, then interrupts are being taken; the level of illumination
  depends amount of time processing interupts.  If LD1 is off and LD2 is
  blinking at about 2Hz, then the system has crashed.

Buttons
=======

  The Mikroe Clicker2 STM32 has two buttons available to software:

    T2/E0, Low sensed when pressed
    T3/PA10, Low sensed when pressed

Using JTAG
==========

  The Clicker2 comes with the mikroBootloader installed.  That bootloader
  has not been used and is possibly incompatible with the Clicker2-STM32
  linker script at configs/clicker2-stm32/scripts/flash.ld.  Often code must
  be built to execute at an offset in to FLASH when a bootloader is used.
  Certainly that is the case for the ST-Micro DFU bootloader but I am not
  aware of the requirements for use with the mikroBootloader.

  JTAG has been used in the development of this board support.  The
  Clicker2-STM32 board offers a 2x5 JTAG connector.  You may use Dupont
  jumpers to connect this port to JTAG as described here:

    https://www.mikroe.com/how-to-use-st-link-v2-with-clicker-2-for-stm32-a-detailed-walkthrough/
    http://www.playembedded.org/blog/en/2016/02/06/mikroe-clicker-2-for-stm32-and-stlink-v2/

  NOTE that the FLASH probably has read protection enabled locked.  You may
  need to follow the instructions at the second link to unlock it.  You can
  also use the STM32 ST-Link CLI tool on Windows to remove the read protection
  using the -OB command:

    $ ./ST-LINK_CLI.exe -c SN=53FF6F064966545035320387 SWD LPM
    STM32 ST-LINK CLI v2.3.0
    STM32 ST-LINK Command Line Interface

    ST-LINK SN : 53FF6F064966545035320387
    ST-LINK Firmware version : V2J24S4
    Connected via SWD.
    SWD Frequency = 4000K.
    Target voltage = 3.2 V.
    Connection mode : Normal.
    Debug in Low Power mode enabled.
    Device ID:0x413
    Device family :STM32F40xx/F41xx

    $ ./ST-LINK_CLI.exe -OB RDP=0
    STM32 ST-LINK CLI v2.3.0
    STM32 ST-LINK Command Line Interface

    ST-LINK SN : 53FF6F064966545035320387
    ST-LINK Firmware version : V2J24S4
    Connected via SWD.
    SWD Frequency = 4000K.
    Target voltage = 3.2 V.
    Connection mode : Normal.
    Device ID:0x413
    Device family :STM32F40xx/F41xx
    Updating option bytes...
    Option bytes updated successfully.

  NOTE:
  1. You can get the ST-Link Utilies here:
     http://www.st.com/en/embedded-software/stsw-link004.html
  2. The ST-LINK Utility command line interface is located at:
     [Install_Directory]\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe
  3. You can get a summary of all of the command options by running
     ST-LINK_CLI.exe with no arguments.
  4. You can get the serial number of the ST-Link when from the information
     window if you connect via the ST-Link Utility:

       11:04:28 : ST-LINK SN : 53FF6F064966545035320387
       11:04:28 : ST-LINK Firmware version : V2J24S4
       11:04:28 : Connected via SWD.
       11:04:28 : SWD Frequency = 100 KHz.
       11:04:28 : Connection mode : Normal.
       11:04:28 : Debug in Low Power mode enabled.
       11:04:30 : Device ID:0x413
       11:04:30 : Device family :STM32F40xx/F41xx
       11:04:30 : Can not read memory!
                  Disable Read Out Protection and retry.

  You can avoid the mess of jumpers using the mikroProg to ST-Link v2 adapter
  along with a 2x5, 10-wire ribbon cable connector:

    https://shop.mikroe.com/add-on-boards/adapter/mikroprog-st-link-v2-adapter

  Then you can use the ST-Link Utility or other debugger software to write
  the NuttX binary to FLASH.  OpenOCD can be used with the ST-Link to provide
  a debug environment.  The debug adaptor is NOT compatible with other JTAG
  debuggers such as the Segger J-Link.

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each Clicker2 configuration is maintained in a sub-directory and can be
  selected as follow:

    cd tools
    ./configure.sh clicker2-stm32/<subdir>
    cd -
    . ./setenv.sh

  Before sourcing the setenv.sh file above, you should examine it and
  perform edits as necessary so that TOOLCHAIN_BIN is the correct path
  to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make oldconfig
    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTES:

    1. These configurations use the mconf-based configuration tool.  To
      change any of these configurations using that tool, you should:

      a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
         see additional README.txt files in the NuttX tools repository.

      b. Execute 'make menuconfig' in nuttx/ in order to start the
         reconfiguration process.

    2. Unless stated otherwise, all configurations generate console
       output on USART3, channel 0) as described above under "Serial
       Console".  The relevant configuration settings are listed below:

         CONFIG_STM32_USART3=y
         CONFIG_STM32_USART3_SERIALDRIVER=y
         CONFIG_STM32_USART=y

         CONFIG_USART3_SERIALDRIVER=y
         CONFIG_USART3_SERIAL_CONSOLE=y

         CONFIG_USART3_RXBUFSIZE=256
         CONFIG_USART3_TXBUFSIZE=256
         CONFIG_USART3_BAUD=115200
         CONFIG_USART3_BITS=8
         CONFIG_USART3_PARITY=0
         CONFIG_USART3_2STOP=0


  3. All of these configurations are set up to build under Linux using the
     "GNU Tools for ARM Embedded Processors" that is maintained by ARM
     (unless stated otherwise in the description of the configuration).

       https://launchpad.net/gcc-arm-embedded

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_LINUX  =y               : Linux environment

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL=y : GNU ARM EABI toolchain

  Configuration sub-directories
  -----------------------------

  nsh:

    Configures the NuttShell (nsh) located at examples/nsh.  This
    configuration is focused on low level, command-line driver testing.  It
    has no network.

    NOTES:

    1. Support for NSH built-in applications is provided:

       Binary Formats:
         CONFIG_BUILTIN=y           : Enable support for built-in programs

       Application Configuration:
         CONFIG_NSH_BUILTIN_APPS=y  : Enable starting apps from NSH command line

       No built applications are enabled in the base configuration, however.

    2. C++ support for applications is enabled:

      CONFIG_HAVE_CXX=y
      CONFIG_HAVE_CXXINITIALIZE=y
      CONFIG_EXAMPLES_NSH_CXXINITIALIZE=y

  usbnsh:
  -------

    This is another NSH example.  If differs from other 'nsh' configurations
    in that this configurations uses a USB serial device for console I/O.
    Such a configuration is useful on the Clicker2 STM32 which has no
    builtin RS-232 drivers.

    NOTES:

    1. This configuration does have USART3 output enabled and set up as
       the system logging device:

       CONFIG_SYSLOG_CHAR=y               : Use a character device for system logging
       CONFIG_SYSLOG_DEVPATH="/dev/ttyS0" : USART3 will be /dev/ttyS0

       However, there is nothing to generate SYLOG output in the default
       configuration so nothing should appear on USART3 unless you enable
       some debug output or enable the USB monitor.

    2. Enabling USB monitor SYSLOG output.  If tracing is enabled, the USB
       device will save encoded trace output in in-memory buffer; if the
       USB monitor is enabled, that trace buffer will be periodically
       emptied and dumped to the system logging device (USART3 in this
       configuration):

       CONFIG_USBDEV_TRACE=y                   : Enable USB trace feature
       CONFIG_USBDEV_TRACE_NRECORDS=128        : Buffer 128 records in memory
       CONFIG_NSH_USBDEV_TRACE=n               : No builtin tracing from NSH
       CONFIG_NSH_ARCHINIT=y                   : Automatically start the USB monitor
       CONFIG_USBMONITOR=y              : Enable the USB monitor daemon
       CONFIG_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
       CONFIG_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
       CONFIG_USBMONITOR_INTERVAL=2     : Dump trace data every 2 seconds

       CONFIG_USBMONITOR_TRACEINIT=y    : Enable TRACE output
       CONFIG_USBMONITOR_TRACECLASS=y
       CONFIG_USBMONITOR_TRACETRANSFERS=y
       CONFIG_USBMONITOR_TRACECONTROLLER=y
       CONFIG_USBMONITOR_TRACEINTERRUPTS=y

    Using the Prolifics PL2303 Emulation
    ------------------------------------
    You could also use the non-standard PL2303 serial device instead of
    the standard CDC/ACM serial device by changing:

      CONFIG_CDCACM=n               : Disable the CDC/ACM serial device class
      CONFIG_CDCACM_CONSOLE=n       : The CDC/ACM serial device is NOT the console
      CONFIG_PL2303=y               : The Prolifics PL2303 emulation is enabled
      CONFIG_PL2303_CONSOLE=y       : The PL2303 serial device is the console


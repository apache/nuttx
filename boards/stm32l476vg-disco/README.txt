XXX all this needs review and update
README
======

This README discusses issues unique to NuttX configurations for the ST
STM32L476VG Discovery board from ST Micro.  See

  http://www.st.com/stm32l476g-disco

STM32L476VG:

  Microprocessor: 32-bit ARM Cortex M4 at 80MHz STM32L476VGT6
  Memory:         1024 KB Flash and 96+32 KB SRAM
  ADC:            3x12-bit, 2.4 MSPS A/D converter: up to 24 channels
  DMA:            16-stream DMA controllers with FIFOs and burst support
  Timers:         Up to 11 timers: up to eight 16-bit, two 32-bit timers, two
                  watchdog timers, and a SysTick timer
  GPIO:           Up to 51 I/O ports with interrupt capability
  I2C:            Up to 3 x I2C interfaces
  USARTs:         Up to 3 USARTs, 2 UARTs, 1 LPUART
  SPIs:           Up to 3 SPIs
  SAIs:           Up to 2 dual-channel audio interfaces
  CAN interface
  SDIO interface
  QSPI interface
  USB:            USB 2.0 full-speed device/host/OTG controller with on-chip PHY
  CRC calculation unit
  RTC

Board features:

  Peripherals:    2 led, 1 d-pad joystick, 2 x LED, LCD, USC OTG FS, SAI stereo
                  Digital Microphone, MEMS Accelerometer, Magnetometer,
                  Gyroscope, 128 Mbit QSPI Flash, current ammeter
  Debug:          Serial wire debug and JTAG interfaces

  Uses a STM32F103 to provide a ST-Link for programming, debug similar to the
  OpenOcd FTDI function - USB to JTAG front-end.

  See http://mbed.org/platforms/ST-Nucleo-L476RG for more
  information about these boards.

Contents
========

  - mbed
  - Hardware
    - Button
    - LED
    - USARTs and Serial Consoles
  - LQFP64
  - mbed
  - Shields
  - Configurations

mbed
====

  The Nucleo-F401RE includes boot loader from mbed:

    https://mbed.org/platforms/ST-Nucleo-F401RE/
    https://mbed.org/handbook/Homepage

  Using the mbed loader:

  1. Connect the Nucleo-F4x1RE to the host PC using the USB connector.
  2. A new file system will appear called NUCLEO; open it with Windows
     Explorer (assuming that you are using Windows).
  3. Drag and drop nuttx.bin into the MBED window.  This will load the
     nuttx.bin binary into the Nucleo-F4x1RE.  The NUCLEO window will
     close then re-open and the Nucleo-F4x1RE will be running the new code.

Hardware
========

  GPIO
  ----
  SERIAL_TX=PA_2    USER_BUTTON=PC_13
  SERIAL_RX=PA_3    LED1       =PA_5

  A0=PA_0  USART2RX D0=PA_3            D8 =PA_9
  A1=PA_1  USART2TX D1=PA_2            D9 =PC_7
  A2=PA_4           D2=PA_10   WIFI_CS=D10=PB_6 SPI_CS
  A3=PB_0  WIFI_INT=D3=PB_3            D11=PA_7 SPI_MOSI
  A4=PC_1      SDCS=D4=PB_5            D12=PA_6 SPI_MISO
  A5=PC_0   WIFI_EN=D5=PB_4       LED1=D13=PA_5 SPI_SCK
               LED2=D6=PB_10  I2C1_SDA=D14=PB_9 Probe
                    D7=PA_8   I2C1_SCL=D15=PB_8 Probe

  From: https://mbed.org/platforms/ST-Nucleo-F401RE/

  Buttons
  -------
  B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
  microcontroller.

  LEDs
  ----
  The Nucleo F401RE and Nucleo F411RE provide a single user LED, LD2.  LD2
  is the green LED connected to Arduino signal D13 corresponding to MCU I/O
  PA5 (pin 21) or PB13 (pin 34) depending on the STM32target.

    - When the I/O is HIGH value, the LED is on.
    - When the I/O is LOW, the LED is off.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
  events as follows when the red LED (PE24) is available:

    SYMBOL                Meaning                   LD2
    -------------------  -----------------------  -----------
    LED_STARTED          NuttX has been started     OFF
    LED_HEAPALLOCATE     Heap has been allocated    OFF
    LED_IRQSENABLED      Interrupts enabled         OFF
    LED_STACKCREATED     Idle stack created         ON
    LED_INIRQ            In an interrupt            No change
    LED_SIGNAL           In a signal handler        No change
    LED_ASSERTION        An assertion failed        No change
    LED_PANIC            The system has crashed     Blinking
    LED_IDLE             MCU is is sleep mode       Not used

  Thus if LD2, NuttX has successfully booted and is, apparently, running
  normally.  If LD2 is flashing at approximately 2Hz, then a fatal error
  has been detected and the system has halted.

Serial Consoles
===============

  USART1
  ------
  Pins and Connectors:

    RXD: PA11  CN10 pin 14
         PB7   CN7 pin 21
    TXD: PA10  CN9 pin 3, CN10 pin 33
         PB6   CN5 pin 3, CN10 pin 17

  NOTE:  You may need to edit the include/board.h to select different USART1
  pin selections.

  TTL to RS-232 converter connection:

    Nucleo CN10 STM32F4x1RE
    ----------- ------------
    Pin 21 PA9  USART1_RX   *Warning you make need to reverse RX/TX on
    Pin 33 PA10 USART1_TX    some RS-232 converters
    Pin 20 GND
    Pin 8  U5V

  To configure USART1 as the console:

    CONFIG_STM32_USART1=y
    CONFIG_USART1_SERIALDRIVER=y
    CONFIG_USART1_SERIAL_CONSOLE=y
    CONFIG_USART1_RXBUFSIZE=256
    CONFIG_USART1_TXBUFSIZE=256
    CONFIG_USART1_BAUD=115200
    CONFIG_USART1_BITS=8
    CONFIG_USART1_PARITY=0
    CONFIG_USART1_2STOP=0

  USART2
  -----
  Pins and Connectors:

    RXD: PA3   CN9 pin 1 (See SB13, 14, 62, 63). CN10 pin 37
         PD6
    TXD: PA2   CN9 pin 2(See SB13, 14, 62, 63). CN10 pin 35
         PD5

  UART2 is the default in all of these configurations.

  TTL to RS-232 converter connection:

    Nucleo CN9  STM32F4x1RE
    ----------- ------------
    Pin 1  PA3  USART2_RX   *Warning you make need to reverse RX/TX on
    Pin 2  PA2  USART2_TX    some RS-232 converters

  Solder Bridges.  This configuration requires:

  - SB62 and SB63 Closed: PA2 and PA3 on STM32 MCU are connected to D1 and D0
    (pin 7 and pin 8) on Arduino connector CN9 and ST Morpho connector CN10
    as USART signals.  Thus SB13 and SB14 should be OFF.

  - SB13 and SB14 Open:  PA2 and PA3 on STM32F103C8T6 (ST-LINK MCU) are
    disconnected to PA3 and PA2 on STM32 MCU.

  To configure USART2 as the console:

    CONFIG_STM32_USART2=y
    CONFIG_USART2_SERIALDRIVER=y
    CONFIG_USART2_SERIAL_CONSOLE=y
    CONFIG_USART2_RXBUFSIZE=256
    CONFIG_USART2_TXBUFSIZE=256
    CONFIG_USART2_BAUD=115200
    CONFIG_USART2_BITS=8
    CONFIG_USART2_PARITY=0
    CONFIG_USART2_2STOP=0

  USART6
  ------
  Pins and Connectors:

    RXD: PC7    CN5 pin2, CN10 pin 19
         PA12   CN10, pin 12
    TXD: PC6    CN10, pin 4
         PA11   CN10, pin 14

  To configure USART6 as the console:

    CONFIG_STM32_USART6=y
    CONFIG_USART6_SERIALDRIVER=y
    CONFIG_USART6_SERIAL_CONSOLE=y
    CONFIG_USART6_RXBUFSIZE=256
    CONFIG_USART6_TXBUFSIZE=256
    CONFIG_USART6_BAUD=115200
    CONFIG_USART6_BITS=8
    CONFIG_USART6_PARITY=0
    CONFIG_USART6_2STOP=0

  Virtual COM Port
  ----------------
  Yet another option is to use UART2 and the USB virtual COM port.  This
  option may be more convenient for long term development, but is painful
  to use during board bring-up.

  Solder Bridges.  This configuration requires:

  - SB62 and SB63 Open: PA2 and PA3 on STM32 MCU are disconnected to D1
    and D0 (pin 7 and pin 8) on Arduino connector CN9 and ST Morpho
    connector CN10.

  - SB13 and SB14 Closed:  PA2 and PA3 on STM32F103C8T6 (ST-LINK MCU) are
    connected to PA3 and PA2 on STM32 MCU to have USART communication
    between them. Thus SB61, SB62 and SB63 should be OFF.

  Configuring USART2 is the same as given above.

  Question:  What BAUD should be configure to interface with the Virtual
  COM port?  115200 8N1?

  Default
  -------
  As shipped, SB62 and SB63 are open and SB13 and SB14 closed, so the
  virtual COM port is enabled.

Shields
=======

  RS-232 from Cutedigi.com
  ------------------------
  Supports a single RS-232 connected via

    Nucleo CN9  STM32F4x1RE  Cutedigi
    ----------- ------------ --------
    Pin 1  PA3  USART2_RX    RXD
    Pin 2  PA2  USART2_TX    TXD

  Support for this shield is enabled by selecting USART2 and configuring
  SB13, 14, 62, and 63 as described above under "Serial Consoles"

  Itead Joystick Shield
  ---------------------
  See http://imall.iteadstudio.com/im120417014.html for more information
  about this joystick.

  Itead Joystick Connection:

    --------- ----------------- ---------------------------------
    ARDUINO   ITEAD             NUCLEO-F4x1
    PIN NAME  SIGNAL            SIGNAL
    --------- ----------------- ---------------------------------
     D3       Button E Output   PB3
     D4       Button D Output   PB5
     D5       Button C Output   PB4
     D6       Button B Output   PB10
     D7       Button A Output   PA8
     D8       Button F Output   PA9
     D9       Button G Output   PC7
     A0       Joystick Y Output PA0  ADC1_0
     A1       Joystick X Output PA1  ADC1_1
    --------- ----------------- ---------------------------------

    All buttons are pulled on the shield.  A sensed low value indicates
    when the button is pressed.

    NOTE: Button F cannot be used with the default USART1 configuration
    because PA9 is configured for USART1_RX by default.  Use select
    different USART1 pins in the board.h file or select a different
    USART or select CONFIG_NUCLEO_F401RE_AJOY_MINBUTTONS which will
    eliminate all but buttons A, B, and C.

  Itead Joystick Signal interpretation:

    --------- ----------------------- ---------------------------
    BUTTON     TYPE                    NUTTX ALIAS
    --------- ----------------------- ---------------------------
    Button A  Large button A          JUMP/BUTTON 3
    Button B  Large button B          FIRE/BUTTON 2
    Button C  Joystick select button  SELECT/BUTTON 1
    Button D  Tiny Button D           BUTTON 6
    Button E  Tiny Button E           BUTTON 7
    Button F  Large Button F          BUTTON 4
    Button G  Large Button G          BUTTON 5
    --------- ----------------------- ---------------------------

  Itead Joystick configuration settings:

    System Type -> STM32 Peripheral Support
      CONFIG_STM32_ADC1=y              : Enable ADC1 driver support

    Drivers
      CONFIG_ANALOG=y                  : Should be automatically selected
      CONFIG_ADC=y                     : Should be automatically selected
      CONFIG_INPUT=y                   : Select input device support
      CONFIG_AJOYSTICK=y               : Select analog joystick support

  There is nothing in the configuration that currently uses the joystick.
  For testing, you can add the following configuration options to enable the
  analog joystick example at apps/examples/ajoystick:

    CONFIG_NSH_ARCHINIT=y
    CONFIG_EXAMPLES_AJOYSTICK=y
    CONFIG_EXAMPLES_AJOYSTICK_DEVNAME="/dev/ajoy0"
    CONFIG_EXAMPLES_AJOYSTICK_SIGNO=13

  STATUS:
  2014-12-04:
    - Without ADC DMA support, it is not possible to sample both X and Y
      with a single ADC.  Right now, only one axis is being converted.
    - There is conflicts with some of the Arduino data pins and the
      default USART1 configuration.  I am currently running with USART1
      but with CONFIG_NUCLEO_F401RE_AJOY_MINBUTTONS to eliminate the
      conflict.
    - Current showstopper: I appear to be getting infinite interrupts as
      soon as joystick button interrupts are enabled.

Configurations
==============

  knsh:
  ----

    This is identical to the nsh configuration below except that (1) NuttX
    is built as a PROTECTED mode, monolithic module and the user applications
    are built separately and, as a consequence, (2) some features that are
    only availabled in the FLAT build are disabled.

    It is recommends to use a special make command; not just 'make' but make
    with the following two arguments:

        make pass1 pass2

    In the normal case (just 'make'), make will attempt to build both user-
    and kernel-mode blobs more or less interleaved.  That actual works!
    However, for me it is very confusing so I prefer the above make command:
    Make the user-space binaries first (pass1), then make the kernel-space
    binaries (pass2)

    NOTES:

    1. At the end of the build, there will be several files in the top-level
       NuttX build directory:

       PASS1:
         nuttx_user.elf    - The pass1 user-space ELF file
         nuttx_user.hex    - The pass1 Intel HEX format file (selected in defconfig)
         User.map          - Symbols in the user-space ELF file

       PASS2:
         nuttx             - The pass2 kernel-space ELF file
         nuttx.hex         - The pass2 Intel HEX file (selected in defconfig)
         System.map        - Symbols in the kernel-space ELF file

       The J-Link programmer will except files in .hex, .mot, .srec, and .bin
       formats.

    2. Combining .hex files.  If you plan to use the .hex files with your
       debugger or FLASH utility, then you may need to combine the two hex
       files into a single .hex file.  Here is how you can do that.

       a. The 'tail' of the nuttx.hex file should look something like this
          (with my comments added):

            $ tail nuttx.hex
            # 00, data records
            ...
            :10 9DC0 00 01000000000800006400020100001F0004
            :10 9DD0 00 3B005A0078009700B500D400F300110151
            :08 9DE0 00 30014E016D0100008D
            # 05, Start Linear Address Record
            :04 0000 05 0800 0419 D2
            # 01, End Of File record
            :00 0000 01 FF

          Use an editor such as vi to remove the 05 and 01 records.

       b. The 'head' of the nuttx_user.hex file should look something like
          this (again with my comments added):

            $ head nuttx_user.hex
            # 04, Extended Linear Address Record
            :02 0000 04 0801 F1
            # 00, data records
            :10 8000 00 BD89 01084C800108C8110208D01102087E
            :10 8010 00 0010 00201C1000201C1000203C16002026
            :10 8020 00 4D80 01085D80010869800108ED83010829
            ...

          Nothing needs to be done here.  The nuttx_user.hex file should
          be fine.

       c. Combine the edited nuttx.hex and un-edited nuttx_user.hex
          file to produce a single combined hex file:

          $ cat nuttx.hex nuttx_user.hex >combined.hex

       Then use the combined.hex file with the to write the FLASH image.
       If you do this a lot, you will probably want to invest a little time
       to develop a tool to automate these steps.

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh for the
    Nucleo-F401RE board.  The Configuration enables the serial interfaces
    on UART2.  Support for builtin applications is enabled, but in the base
    configuration no builtin applications are selected (see NOTES below).

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration uses the Generic ARM EABI toolchain
       for Linux.  That can easily be reconfigured, of course.

       CONFIG_HOST_LINUX=y                 : Builds under Linux
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL=y : Generic EABI toolchain for Linux

    3. Although the default console is USART2 (which would correspond to
       the Virtual COM port) I have done all testing with the console
       device configured for USART1 (see instruction above under "Serial
       Consoles).  I have been using a TTL-to-RS-232 converter.

    4. This example has been used to verify the OTGFS functionality.  USB is
       not enabled in the default configuration but can be enabled with the
       following settings:

         CONFIG_STM32L4_OTGFS=y

         CONFIG_USBDEV=y
         CONFIG_USBDEV_SELFPOWERED=y

     These will enable the USB CDC/ACM serial device

         CONFIG_CDCACM=y
         CONFIG_CDCACM_EP0MAXPACKET=64
         CONFIG_CDCACM_EPINTIN=1
         CONFIG_CDCACM_EPINTIN_FSSIZE=64
         CONFIG_CDCACM_EPINTIN_HSSIZE=64
         CONFIG_CDCACM_EPBULKOUT=3
         CONFIG_CDCACM_EPBULKOUT_FSSIZE=64
         CONFIG_CDCACM_EPBULKOUT_HSSIZE=512
         CONFIG_CDCACM_EPBULKIN=2
         CONFIG_CDCACM_EPBULKIN_FSSIZE=64
         CONFIG_CDCACM_EPBULKIN_HSSIZE=512
         CONFIG_CDCACM_NRDREQS=4
         CONFIG_CDCACM_NWRREQS=4
         CONFIG_CDCACM_BULKIN_REQLEN=96
         CONFIG_CDCACM_RXBUFSIZE=257
         CONFIG_CDCACM_TXBUFSIZE=193
         CONFIG_CDCACM_VENDORID=0x0525
         CONFIG_CDCACM_PRODUCTID=0xa4a7
         CONFIG_CDCACM_VENDORSTR="NuttX"
         CONFIG_CDCACM_PRODUCTSTR="CDC/ACM Serial"

         CONFIG_SERIAL_REMOVABLE=y

    These will enable the USB serial example at apps/examples/usbserial

         CONFIG_BOARDCTL_USBDEVCTRL=y

         CONFIG_EXAMPLES_USBSERIAL=y
         CONFIG_EXAMPLES_USBSERIAL_BUFSIZE=512
         CONFIG_EXAMPLES_USBSERIAL_TRACEINIT=y
         CONFIG_EXAMPLES_USBSERIAL_TRACECLASS=y
         CONFIG_EXAMPLES_USBSERIAL_TRACETRANSFERS=y
         CONFIG_EXAMPLES_USBSERIAL_TRACECONTROLLER=y
         CONFIG_EXAMPLES_USBSERIAL_TRACEINTERRUPTS=y

    Optional USB debug features:

         CONFIG_DEBUG_FEATURES=y
         CONFIG_DEBUG_USB=y
         CONFIG_ARCH_USBDUMP=y
         CONFIG_USBDEV_TRACE=y
         CONFIG_USBDEV_TRACE_NRECORDS=128
         CONFIG_USBDEV_TRACE_STRINGS=y
         CONFIG_USBDEV_TRACE_INITIALIDSET=y

         CONFIG_NSH_USBDEV_TRACE=y
         CONFIG_NSH_USBDEV_TRACEINIT=y
         CONFIG_NSH_USBDEV_TRACECLASS=y
         CONFIG_NSH_USBDEV_TRACETRANSFERS=y
         CONFIG_NSH_USBDEV_TRACECONTROLLER=y
         CONFIG_NSH_USBDEV_TRACEINTERRUPTS=y

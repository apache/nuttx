README
======

This README discusses issues unique to NuttX configurations for the ST
STM32L4R9AI Discovery board from ST Micro.  See

  https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-eval-tools/stm32-mcu-eval-tools/stm32-mcu-discovery-kits/32l4r9idiscovery.html

STM32L4R9AI:

  Microprocessor: 32-bit ARM Cortex M4 at 120MHz STM32L4R9AI
  Memory:         2048 KB Flash and 192+64+384 KB SRAM
  ADC:            1x12-bit, 5 MSPS A/D converter: up to 14 external channels
  DAC:            2 channels
  DFSDM:          4 filters, 8 channels
  DMA:            16-stream DMA controllers with FIFOs and burst support
  Timers:         Up to 11 timers: up to eight 16-bit, two 32-bit timers, two
                  watchdog timers, and a SysTick timer
  GPIO:           Up to 131 I/O ports with interrupt capability
  I2C:            Up to 4 x I2C interfaces
  USARTs:         Up to 3 USARTs, 2 UARTs, 1 LPUART
  SPIs:           Up to 3 SPIs
  SAIs:           Up to 2 dual-channel audio interfaces
  CAN interface
  SDIO interface
  OCTOSPI interface
  Camera interface
  USB:            USB 2.0 full-speed device/host/OTG controller with on-chip PHY
  CRC calculation unit
  RTC

Board features:

  Peripherals:    1 d-pad joystick, 2 x LED, AMOLED display, USC OTG FS,
                  2 x MEMS Digital Microphones, SAI codec, 16 Mbit PSRAM,
                  512 Mbit OCTOSPI Flash, current ammeter
  Debug:          Serial wire debug and JTAG interfaces

  Uses a STM32F103 to provide a ST-Link for programming, debug similar to the
  OpenOcd FTDI function - USB to JTAG front-end.

Contents
========

  - mbed
  - Hardware
    - Button
    - LED
    - U[S]ARTs and Serial Consoles
  - Segger J-Link
  - LQFP64
  - Configurations

mbed
====

  The STM32L4R9AI-DISCO includes boot loader from mbed:

    https://mbed.org/handbook/Homepage

  Using the mbed loader:

  1. Connect the board to the host PC using the USB connector.
  2. A new file system will appear called DIS_L4R9AI; open it with Windows
     Explorer (assuming that you are using Windows).
  3. Drag and drop nuttx.bin into the MBED window.  This will load the
     nuttx.bin binary into the board.  The DIS_L49RAIO window will
     close then re-open and the board will be running the new code.

Hardware
========

  Buttons
  -------
  B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
  microcontroller.

  LEDs
  ----
  The STM32L4R9AI-DISCO board provides two user LEDs, LD1 (orange) and LD2 (green).
  PB0 is LD1 (orange)
  PH4 is LD2 (green)
    - When the I/O is HIGH value, the LED is on.
    - When the I/O is LOW, the LED is off.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/stm32_autoleds.c. The LEDs are used to encode OS-related
  events as follows when the green LED (PH4) is available:

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

  Thus if LD2 is on, NuttX has successfully booted and is, apparently,
  running normally. If LD2 is flashing at approximately 2Hz, then a fatal error
  has been detected and the system has halted.

  U[S]ARTs and Serial Consoles
  ----------------------------

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

    CONFIG_STM32L4_USART1=y
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

    CONFIG_STM32L4_USART2=y
    CONFIG_USART2_SERIALDRIVER=y
    CONFIG_USART2_SERIAL_CONSOLE=y
    CONFIG_USART2_RXBUFSIZE=256
    CONFIG_USART2_TXBUFSIZE=256
    CONFIG_USART2_BAUD=115200
    CONFIG_USART2_BITS=8
    CONFIG_USART2_PARITY=0
    CONFIG_USART2_2STOP=0

  UART4
  ------
  Pins and Connectors:

    RXD: PA1 -> CN11 D5
    TXD: PA0 -> CN17 A4

  To configure USART4 as the console:

    CONFIG_STM32L4_UART4=y
    CONFIG_USART4_SERIALDRIVER=y
    CONFIG_USART4_SERIAL_CONSOLE=y
    CONFIG_USART4_RXBUFSIZE=512
    CONFIG_USART4_TXBUFSIZE=256
    CONFIG_USART4_BAUD=2000000
    CONFIG_USART4_BITS=8
    CONFIG_USART4_PARITY=0
    CONFIG_USART4_2STOP=0

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

Segger J-Link
=============

  Reference: https://www.segger.com/downloads/application-notes/AN00021

  1. Connect J-Link VTref      (1) to pin VDD
  2. Connect J-Link SWDIO      (7) to pin PA13
  3. Connect J-Link SWCLK      (9) to pin PA14
  4. Connect J-Link SWO       (13) to pin PB3
  5. Connect J-Link RESET     (15) to pin NRST
  6. Connect J-Link 5V-Supply (19) to pin 5V
  7. Connect J-Link GND        (4) to pin GND

  Jumpers on CN4 (ST-Link) must be removed for external debug.

Configurations
==============

  knsh:
  ----

    This is identical to the nsh configuration below except that (1) NuttX
    is built as a PROTECTED mode, monolithic module and the user applications
    are built separately and, as a consequence, (2) some features that are
    only available in the FLAT build are disabled.

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
    STM32L4R9AI-DISCO board.  The Configuration enables the serial interfaces
    on UART4.  Support for builtin applications is enabled, but in the base
    configuration no builtin applications are selected (see NOTES below).

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

    2. By default, this configuration uses the Generic ARM EABI toolchain
       for Linux.  That can easily be reconfigured, of course.

       CONFIG_HOST_LINUX=y                 : Builds under Linux
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL=y : Generic EABI toolchain for Linux

    3. The default console is UART4

    4. This example can be used to verify the OTGFS functionality. USB is
       not enabled in the default configuration but can be enabled with the
       following settings: (TODO: need to test!)

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

  nxhello:
  -------

    A simple NSH example using apps/examples/nxhello, a very simply test of
    basic NX functionality.

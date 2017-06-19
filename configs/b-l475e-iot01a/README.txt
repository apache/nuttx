README
======

  This README discusses the port of NuttX to the STMicro B-L475E-IOT01A
  Discovery kit powered by STM32L475VG Cortex-M4.  This board targets IoT
  nodes with a choice of connectivity options including WiFi, Bluetooth LE,
  NFC, and sub-GHZ RF at 868 or 915 MHz, as well as a long list of various
  environmental sensors.

Contents
========

  o STATUS
  o Board Features
  o LEDs and Buttons
  o Serial Console
  o Configurations

STATUS
======

  o 2017-06-10:  I have no hardware in hand and I am not sure that I will
    even pursue this port.  This README is really no more than a thought
    experiment at the present time.

    A few days ago, I did add support for the STM32L4x5 MCU family to
    NuttX.  But no work has yet been done for this board port other
    than writing this README file.

  o 2017-06-13:  I just learned that development boards will not be
    available for another month.

Board Features
==============

  B-L475E-IOT01A Discovery kit key features and specifications:

  o MCU:  STM32L475 Series MCU based on ARM Cortex-M4 core with 1 MB
    Flash memory, 128 KB SRAM
  o Storage: 64 Mbit (8MB)  Quad-SPI Flash memory (Macronix)
  o Connectivity:
    - Bluetooth 4.1 LE module (SPBTLE-RF)
    - Sub-GHz (868 or 915 MHz) low-power-programmable RF module (SPSGRF-868
      or SPSGRF-915)
    - Wi-Fi module based on Inventek ISM43362-M3G-L44 (802.11 b/g/n
      compliant)
    - Dynamic NFC tag based on M24SR with its printed NFC antenna
  o Sensors:
    - 2x digital omni-directional microphones (MP34DT01)
    - Capacitive digital sensor for relative humidity and temperature
      (HTS221)
    - 3-axis magnetometer (LIS3MDL)
    - 3D accelerometer and 3D gyroscope (LSM6DSL)
    - 260-1260 hPa absolute digital output barometer (LPS22HB)
    - Time-of-Flight and gesture-detection sensor (VL53L0X
  o USB – 1x micro USB OTG port (Full speed)
  o Expansion – Arduino UNO V3 headers, PMOD header
  o Debugging – On-board ST-LINK/V2-1 debugger/programmer with USB
    re-enumeration capability: mass storage, virtual COM port and debug
    port
  o Misc – 2 push-buttons (user and reset)
  o Power Supply – 5V via ST LINK USB VBUS or external sources

  The board supports ARM mbed online compiler, but can also be programmed
  using IDEs such as IAR, Keil, and GCC-based IDEs.  STMicro also provides
  HAL libraries and code samples as part of the STM32Cube Package, as well
  as X-CUBE-AWS expansion software to connect to the Amazon Web Services
  (AWS) IoT platform.

  NOTES:

  1. The board usese Wi-Fi® module Inventek ISM43362-M3G-L44 (802.11 b/g/n
     compliant), which consists of BCM43362 and STM32F205 host processor
     that has a standard SPI or UART interface capability.  It means you
     will only use AT command to talk with Wi-Fi® module by SPI. All the
     tcp/ip stack is  built-in STM32F205 in Wi-Fi® module.

     This cannot integrate cleanly with the NuttX network stack.  A
     USERSOCK option was recently added that would permit implementation
     of the Inventek support in an applications.  But that would then
     preclude the 6LoWPAN integration into IPv6.

  2. The board uses Bluetooth® V4.1 module (SPBTLE-RF), which has built-in
     BLE stack.  Similar with wifi, you only use simple AT command to talk
     with this BLE module.

  3. STMicro provides contiki 6lowpan for mesh.
     http://www.st.com/en/embedded-software/osxcontiki6lp.html but mesh
     network is not popular in the market, star network is the mainstream
     for its simplicity and robustness.

LEDs and Buttons
================

  The black button B1 located on top side is the reset of the STM32L475VGT6.

  The blue button B1 located top side is available to be used as a digital
  input or as alternate function Wake-up.  When the button is depressed the logic state is "0", otherwise the logic state is "1".

  Two green LEDs (LD1 and LD2), located on the top side are available for the user. To light a LED a high logic state "1" should be written in the corresponding GPIO.

  Reference Color Name    Comment
    B2      blue  Wake-up Alternate function Wake-up
    LD1     green LED1    PA5 (alternate with ARD.D13)
    LD2     green LED2    PB14

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  selected.  In that case, the usage by the board port is defined in
  include/board.h and src/lpc31_leds.c. The LEDs are used to encode
  OS-related events as follows:

    SYMBOL                Meaning                     LED state
                                                    LED2     LED1
    -------------------  -----------------------  -------- --------
    LED_STARTED          NuttX has been started     OFF      OFF
    LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
    LED_IRQSENABLED      Interrupts enabled         OFF      OFF
    LED_STACKCREATED     Idle stack created         ON       OFF
    LED_INIRQ            In an interrupt            N/C      N/C
    LED_SIGNAL           In a signal handler        N/C      N/C
    LED_ASSERTION        An assertion failed        N/C      N/C
    LED_PANIC            The system has crashed     N/C      Blinking
    LED_IDLE             MCU is is sleep mode         Not used

  Thus if LED2 is statically on, NuttX has successfully booted and is,
  apparently, running normmally.  If LED1 is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

  NOTE: That LED2 is not used after completion of booting and may
  be used by other board-specific logic.

  Of course, if CONFIG_ARCH_LEDS is not selected, then both LEDs are
  available for use by other logic.

Serial Console
==============

  Arduino Serial Shield
  ---------------------
  An TLL-to-RS232 Converter shield may be used with UART4:

    UART4:
    -------------- ----------------  ------------------
    STM32L475VGTx   Board Signal     Arduino Connector
    -------------- ----------------  ------------------
    UART4_RX PA1   ARD.D0-UART4_RX   CN3 pin1 RX/D0
    UART4_TX PA0   ARD.D1-UART4_TX   CN3 pin2 TX/D1
    -------------- ----------------  ------------------

  Virtual COM Port
  ----------------
  The serial interface USART1 is directly available as a virtual COM port
  of the PC connected to the ST-LINK/V2-1 USB connector CN7.

    USART1:
    -------------- ---------------- --------------
    STM32L475VGTx  Board Signal     STM32F103CBT6
    -------------- ---------------- --------------
    USART1_TX PB6  ST-LINK-UART1_TX USART2_RX PA3
    UAART1_RX PB7  ST-LINK-UART1_RX USART2_TX PA2
    -------------- ---------------- --------------

  The virtual COM port settings are configured as: 115200 b/s, 8 bits data,
  no parity, 1 stop bit, no flow control.

  Other Options
  -------------

    USART2 - Available on CN10 if solder bridges closed.
    -------------- ----------------  ---------------------------
    STM32L475VGTx  Board Signal      PMOD / Solder Bridges
    -------------- ----------------  ---------------------------
    USART2_RX PD4  PMOD-UART2_RX     CN10 pin1 or 2 (SB12, SB14)
    USART2_TX PD5  PMOD-UART2_TX     CN10 pin2 TX/D1 (SB20)
    -------------- ----------------  ---------------------------

    USART3 - Dedicated to ISM43362-M3G-L44 Serial-to-Wifi Module.
    -------------- ----------------  ------------------
    STM32L475VGTx  Board Signal      Arduino Connector
    -------------- ----------------  ------------------
    USART3_RX PD9  INTERNAL-UART3_RX CN3 pin1 RX/D0
    USART3_TX PD8  INTERNAL-UART3_TX CN3 pin2 TX/D1
    -------------- ----------------  ------------------

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each  B-L475E-IOT01A configuration is maintained in a sub-directory and
  can be selected as follow:

    cd tools
    ./configure.sh  b-l475e-iot01a/<subdir>
    cd -

  Before building, make sure that:

    1. The PATH environment variable include the correct path to the
       directory than holds your toolchain binaries.
    2. Check the .config file.  Make sure that the configuration is set for
       your build platform (e.g., Linux vs. Windows) and that the toolchain
       is set for the toolchain type you are using.

  The <subdir> that is provided above as an argument to the
  tools/configure.sh must be is one of those listed below.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply,
  nuttx.

    make oldconfig
    make

  Where 'make oldconfig' brings the configuration up to data with the current configuration data and 'make' will compile all of the source
  files and generate the final binary.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on USART1 (i.e., for ST-Link Virtual COM port).

  3. All of these configurations are set up to build under Windows using the
     "GNU Tools for ARM Embedded Processors" that is maintained by ARM
     (unless stated otherwise in the description of the configuration).

       https://launchpad.net/gcc-arm-embedded

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Window environment
       CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : GNU ARM EABI toolchain

Configuration sub-directories
-----------------------------

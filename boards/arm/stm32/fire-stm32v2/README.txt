README
======

This README discusses issues unique to NuttX configurations for the M3
Wildfire development board (STM32F103VET6).  See http://firestm32.taobao.com

This configuration should support both the version 2 and version 3 of the
Wildfire board (using NuttX configuration options).  However, only version 2
has been verified.

Contents
========

  - Pin Configuration
  - DFU and JTAG
  - OpenOCD
  - LEDs
  - RTC
  - M3 Wildfire-specific Configuration Options
  - Configurations

Pin Configuration
=================
--- ------ -------------- -------------------------------------------------------------------
PIN NAME   SIGNAL         NOTES
--- ------ -------------- -------------------------------------------------------------------

1   PE2    PE2-C-RCLK     Camera (P9)
2   PE3    PE3-USB-M      USB2.0
3   PE4    PE4-BEEP       LS1 Bell (v2)
           PE4            10Mbps ENC28J60 Interrupt (v3)
4   PE5    (no name)      10Mbps ENC28J60 Interrupt (v2)
           PE5            KEY1, Low when closed (pulled high if open) (v3)
5   PE6
6   VBAT   BT1            Battery (BT1)
7   PC13                  Header 7X2
8   PC14   PC14/OSC32-IN  Y2 32.768KHz
9   PC15   PC15/OSC32-OUT Y2 32.768KHz
10  VSS_5  DGND
11  VDD_5  3V3
12  OSC_IN                Y1 8MHz
13  OSC_OUT               Y1 8MHz
14  NRST   REST1          Reset switch
15  PC0
16  PC1    PC1/ADC123-IN11 Potentiometer (R16)
17  PC2
18  PC3    PC3-LED1       LED1, Active low (pulled high)
19  VSSA   DGND
20  VREF-  DGND
21  VREF+  3V3
22  VDDA   3V3
23  PA0    PA0-C-VSYNC    Camera (P9)
24  PA1    PC1/ADC123-IN1
25  PA2    PA2-US2-TX     MAX3232, DB9 D7

--- ------ -------------- -------------------------------------------------------------------
PIN NAME   SIGNAL         NOTES
--- ------ -------------- -------------------------------------------------------------------

26  PA3    PA3-US2-RX     MAX3232, DB9 D7
27  VSS_4  DGND
28  VDD_4  3V3
29  PA4    PA4-SPI1-NSS   10Mbit ENC28J60, SPI 2M FLASH
30  PA5    PA5-SPI1-SCK   2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
31  PA6    PA6-SPI1-MISO  2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
32  PA7    PA7-SPI1-MOSI  2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
33  PC4    PC4-LED2       LED2, Active low (pulled high)
34  PC5    PC5-LED3       LED3, Active low (pulled high)
35  PB0    PB0-KEY1       KEY1, Low when closed (pulled high if open) (v2)
           PB0            Header P5 (v3)
36  PB1    PB1-KEY2       KEY2, Low when closed (pulled high if open)
37  PB2    BOOT1/DGND
38  PE7    PE7-FSMC_D4    2.4" TFT + Touchscreen
39  PE8    PE8-FSMC_D5    2.4" TFT + Touchscreen
40  PE9    PE9-FSMC_D6    2.4" TFT + Touchscreen
41  PE10   PE10-FSMC_D7   2.4" TFT + Touchscreen
42  PE11   PE11-FSMC_D8   2.4" TFT + Touchscreen
43  PE12   PE12-FSMC_D9   2.4" TFT + Touchscreen
44  PE13   PE13-FSMC_D10  2.4" TFT + Touchscreen
45  PE14   PE14-FSMC_D11  2.4" TFT + Touchscreen
46  PE15   PE15-FSMC_D12  2.4" TFT + Touchscreen
47  PB10   PB10-C-DO_2    Camera (P9)
48  PB11   PB11-MP3-RST   MP3
           PB11-C-DO_3    Camera (P9)
49  VSS_1  DGND
50  VDD_1  3V3

--- ------ -------------- -------------------------------------------------------------------
PIN NAME   SIGNAL         NOTES
--- ------ -------------- -------------------------------------------------------------------

51  PB12   PB12-SPI2-NSS  MP3
           PB12-C-DO_4    Camera (P9)
52  PB13   PB13-SPI2-SCK  MP3
           PB13-C-DO_5    Camera (P9)
53  PB14   PB14-SPI2-MISO MP3
           PB14-C-DO_6    Camera (P9)
54  PB15   PB15-SPI2-MOSI MP3
           PB15-C-DO_7    Camera (P9)
55  PD8    PD8-FSMC_D13   2.4" TFT + Touchscreen
56  PD9    PD9-FSMC_D14   2.4" TFT + Touchscreen
57  PD10   PD10-FSMC_D15  2.4" TFT + Touchscreen
58  PD11   PD11-FSMC_A16  2.4" TFT + Touchscreen
59  PD12   C-LED_EN       Camera (P9)
60  PD13   PD13-LCD/LIGHT 2.4" TFT + Touchscreen
61  PD14   PD14-FSMC_D0   2.4" TFT + Touchscreen
62  PD15   PD15-FSMC_D1   2.4" TFT + Touchscreen
63  PC6    PC6-MP3-XDCS   MP3
           PC6-C-SIO_C    Camera (P9)
64  PC7    PC7-MP3-DREQ   MP3
           PC7-C-SIO_D    Camera (P9)
65  PC8    PC8-SDIO-D0    SD card, pulled high
66  PC9    PC9-SDIO-D1    SD card, pulled high
67  PA8    PA8-C-XCLK     Camera (P9)
68  PA9    PA9-US1-TX     MAX3232, DB9 D8
69  PA10   PA10-US1-RX    MAX3232, DB9 D8
70  PA11   PA11-USBDM     USB2.0
71  PA12   PA12-USBDP     USB2.0
72  PA13   PA13-JTMS      JTAG
73  N/C
74  VSS_2  DGND
75  VDD_2  3V3

--- ------ -------------- -------------------------------------------------------------------
PIN NAME   SIGNAL         NOTES
--- ------ -------------- -------------------------------------------------------------------

76  PA14   PA14-JTCK      JTAG
77  PA15   PA15-JTDI      JTAG
78  PC10   PC10-SDIO-D2   SD card, pulled high
79  PC11   PC10-SDIO-D3   SD card, pulled high
80  PC12   PC12-SDIO-CLK  SD card
81  PD0    PD0-FSMC_D2    2.4" TFT + Touchscreen
82  PD1    PD1-FSMC_D3    2.4" TFT + Touchscreen
83  PD2    PD2-SDIO-CMD   SD card, pulled high
84  PD3    PD3-C-WEN      Camera (P9)
85  PD4    PD4-FSMC_NOE   2.4" TFT + Touchscreen
86  PD5    PD5-FSMC_NWE   2.4" TFT + Touchscreen
87  PD6    PD6-C-OE       Camera (P9)
88  PD7    PD7-FSMC_NE1   2.4" TFT + Touchscreen
89  PB3    PB3-JTDO       JTAG
90  PB4    PB4-NJTRST     JTAG
91  PB5    PB5-C-WRST     Camera (P9)
92  PB6    PB6-I2C1-SCL   2.4" TFT + Touchscreen, AT24C02
93  PB7    PB7-I2C1-SDA   2.4" TFT + Touchscreen, AT24C02
94  BOOT0  SW3            3V3 or DGND
95  PB8    PB8-CAN-RX     CAN transceiver, Header 2H
           PB8-C-DO_0     Camera (P9)
96  PB9    PB9-CAN-TX     CAN transceiver, Header 2H
           PB9-C-DO_1     Camera (P9)
97  PE0    PE0-C-RRST     Camera (P9)
98  PE1    PE1-FSMC_NBL1  2.4" TFT + Touchscreen
99  VSS_3  DGND
100 VDD_3  3V3

DFU and JTAG
============

  Enbling Support for the DFU Bootloader
  --------------------------------------
  The linker files in these projects can be configured to indicate that you
  will be loading code using STMicro built-in USB Device Firmware Upgrade (DFU)
  loader or via some JTAG emulator.  You can specify the DFU bootloader by
  adding the following line:

    CONFIG_STM32_DFU=y

  to your .config file. Most of the configurations in this directory are set
  up to use the DFU loader.

  If CONFIG_STM32_DFU is defined, the code will not be positioned at the beginning
  of FLASH (0x08000000) but will be offset to 0x08003000.  This offset is needed
  to make space for the DFU loader and 0x08003000 is where the DFU loader expects
  to find new applications at boot time.  If you need to change that origin for some
  other bootloader, you will need to edit the file(s) ld.script.dfu for the
  configuration.

  The DFU SE PC-based software is available from the STMicro website,
  http://www.st.com.  General usage instructions:

  1. Convert the NuttX Intel Hex file (nuttx.hex) into a special DFU
     file (nuttx.dfu)... see below for details.
  2. Connect the M3 Wildfire board to your computer using a USB
     cable.
  3. Start the DFU loader on the M3 Wildfire board.  You do this by
     resetting the board while holding the "Key" button.  Windows should
     recognize that the DFU loader has been installed.
  3. Run the DFU SE program to load nuttx.dfu into FLASH.

  What if the DFU loader is not in FLASH?  The loader code is available
  inside of the Demo directory of the USBLib ZIP file that can be downloaded
  from the STMicro Website.  You can build it using RIDE (or other toolchains);
  you will need a JTAG emulator to burn it into FLASH the first time.

  In order to use STMicro's built-in DFU loader, you will have to get
  the NuttX binary into a special format with a .dfu extension.  The
  DFU SE PC_based software installation includes a file "DFU File Manager"
  conversion program that a file in Intel Hex format to the special DFU
  format.  When you successfully build NuttX, you will find a file called
  nutt.hex in the top-level directory.  That is the file that you should
  provide to the DFU File Manager.  You will end up with a file called
  nuttx.dfu that you can use with the STMicro DFU SE program.

  Enabling JTAG
  -------------
  If you are not using the DFU, then you will probably also need to enable
  JTAG support.  By default, all JTAG support is disabled but there NuttX
  configuration options to enable JTAG in various different ways.

  These configurations effect the setting of the SWJ_CFG[2:0] bits in the AFIO
  MAPR register.  These bits are used to configure the SWJ and trace alternate
  function I/Os. The SWJ (SerialWire JTAG) supports JTAG or SWD access to the
  Cortex debug port.  The default state in this port is for all JTAG support
  to be disabled.

  CONFIG_STM32_JTAG_FULL_ENABLE - sets SWJ_CFG[2:0] to 000 which enables full
    SWJ (JTAG-DP + SW-DP)

  CONFIG_STM32_JTAG_NOJNTRST_ENABLE - sets SWJ_CFG[2:0] to 001 which enable
    full SWJ (JTAG-DP + SW-DP) but without JNTRST.

  CONFIG_STM32_JTAG_SW_ENABLE - sets SWJ_CFG[2:0] to 010 which would set JTAG-DP
    disabled and SW-DP enabled.

  The default setting (none of the above defined) is SWJ_CFG[2:0] set to 100
  which disable JTAG-DP and SW-DP.

OpenOCD
=======

I have also used OpenOCD with the M3 Wildfire.  In this case, I used
the Olimex USB ARM OCD.  See the script in boards/arm/stm32/fire-stm32v2/tools/oocd.sh
for more information.  Using the script:

1) Start the OpenOCD GDB server

   cd <nuttx-build-directory>
   boards/arm/stm32/fire-stm32v2/tools/oocd.sh $PWD

2) Load NuttX

   cd <nuttx-built-directory>
   arm-none-eabi-gdb nuttx
   gdb> target remote localhost:3333
   gdb> mon reset
   gdb> mon halt
   gdb> load nuttx

3) Running NuttX

   gdb> mon reset
   gdb> c

LEDs
====

The M3 Wildfire has 3 LEDs labeled LED1, LED2 and LED3.  These LEDs are not
used by the NuttX port unless CONFIG_ARCH_LEDS is defined.  In that case, the
usage by the board port is defined in include/board.h and src/up_autoleds.c.
The LEDs are used to encode OS-related events as follows:

                                        /* LED1   LED2   LED3 */
  #define LED_STARTED                0  /* OFF    OFF    OFF */
  #define LED_HEAPALLOCATE           1  /* ON     OFF    OFF */
  #define LED_IRQSENABLED            2  /* OFF    ON     OFF */
  #define LED_STACKCREATED           3  /* OFF    OFF    OFF */

  #define LED_INIRQ                  4  /* NC     NC    ON  (momentary) */
  #define LED_SIGNAL                 5  /* NC     NC    ON  (momentary) */
  #define LED_ASSERTION              6  /* NC     NC    ON  (momentary) */
  #define LED_PANIC                  7  /* NC     NC    ON  (2Hz flashing) */
  #undef  LED_IDLE                      /* Sleep mode indication not supported */

RTC
===

  The STM32 RTC may configured using the following settings.

    CONFIG_RTC - Enables general support for a hardware RTC. Specific
      architectures may require other specific settings.
    CONFIG_RTC_HIRES - The typical RTC keeps time to resolution of 1
      second, usually supporting a 32-bit time_t value.  In this case,
      the RTC is used to &quot;seed&quot; the normal NuttX timer and the
      NuttX timer provides for higher resolution time. If CONFIG_RTC_HIRES
      is enabled in the NuttX configuration, then the RTC provides higher
      resolution time and completely replaces the system timer for purpose of
      date and time.
    CONFIG_RTC_FREQUENCY - If CONFIG_RTC_HIRES is defined, then the
      frequency of the high resolution RTC must be provided.  If CONFIG_RTC_HIRES
      is not defined, CONFIG_RTC_FREQUENCY is assumed to be one.
    CONFIG_RTC_ALARM - Enable if the RTC hardware supports setting of an alarm.
      A callback function will be executed when the alarm goes off.

  In hi-res mode, the STM32 RTC operates only at 16384Hz.  Overflow interrupts
  are handled when the 32-bit RTC counter overflows every 3 days and 43 minutes.
  A BKP register is incremented on each overflow interrupt creating, effectively,
  a 48-bit RTC counter.

  In the lo-res mode, the RTC operates at 1Hz.  Overflow interrupts are not handled
  (because the next overflow is not expected until the year 2106).

   WARNING:  Overflow interrupts are lost whenever the STM32 is powered down.  The
   overflow interrupt may be lost even if the STM32 is powered down only momentarily.
   Therefore hi-res solution is only useful in systems where the power is always on.

M3 Wildfire-specific Configuration Options
============================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=stm32

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_STM32
       CONFIG_ARCH_CHIP_STM32F103VE

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=fire-stm32v2 (for the M3 Wildfire development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_FIRE_STM32V2=y  (Version 2)
       CONFIG_ARCH_BOARD_FIRE_STM32V3=y  (Version 3)

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x00010000 (64Kb)

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x20000000

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
        stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

  Individual subsystems can be enabled:
    AHB
    ---
    CONFIG_STM32_DMA1
    CONFIG_STM32_DMA2
    CONFIG_STM32_CRC
    CONFIG_STM32_FSMC
    CONFIG_STM32_SDIO

    APB1
    ----
    CONFIG_STM32_TIM2
    CONFIG_STM32_TIM3
    CONFIG_STM32_TIM4
    CONFIG_STM32_TIM5
    CONFIG_STM32_TIM6
    CONFIG_STM32_TIM7
    CONFIG_STM32_WWDG
    CONFIG_STM32_IWDG
    CONFIG_STM32_SPI2
    CONFIG_STM32_SPI4
    CONFIG_STM32_USART2
    CONFIG_STM32_USART3
    CONFIG_STM32_UART4
    CONFIG_STM32_UART5
    CONFIG_STM32_I2C1
    CONFIG_STM32_I2C2
    CONFIG_STM32_USB
    CONFIG_STM32_CAN1
    CONFIG_STM32_BKP
    CONFIG_STM32_PWR
    CONFIG_STM32_DAC1
    CONFIG_STM32_DAC2
    CONFIG_STM32_USB

    APB2
    ----
    CONFIG_STM32_ADC1
    CONFIG_STM32_ADC2
    CONFIG_STM32_TIM1
    CONFIG_STM32_SPI1
    CONFIG_STM32_TIM8
    CONFIG_STM32_USART1
    CONFIG_STM32_ADC3

  Timer and I2C devices may need to the following to force power to be applied
  unconditionally at power up.  (Otherwise, the device is powered when it is
  initialized).

    CONFIG_STM32_FORCEPOWER

  Timer devices may be used for different purposes.  One special purpose is
  to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
  is defined (as above) then the following may also be defined to indicate that
  the timer is intended to be used for pulsed output modulation, ADC conversion,
  or DAC conversion.  Note that ADC/DAC require two definition:  Not only do you have
  to assign the timer (n) for used by the ADC or DAC, but then you also have to
  configure which ADC or DAC (m) it is assigned to.

    CONFIG_STM32_TIMn_PWM   Reserve timer n for use by PWM, n=1,..,8
    CONFIG_STM32_TIMn_ADC   Reserve timer n for use by ADC, n=1,..,8
    CONFIG_STM32_TIMn_ADCm  Reserve timer n to trigger ADCm, n=1,..,8, m=1,..,3
    CONFIG_STM32_TIMn_DAC   Reserve timer n for use by DAC, n=1,..,8
    CONFIG_STM32_TIMn_DACm  Reserve timer n to trigger DACm, n=1,..,8, m=1,..,2

  For each timer that is enabled for PWM usage, we need the following additional
  configuration settings:

    CONFIG_STM32_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}

  NOTE: The STM32 timers are each capable of generating different signals on
  each of the four channels with different duty cycles.  That capability is
  not supported by this driver:  Only one output channel per timer.

  Alternate pin mappings.  The M3 Wildfire board requires only CAN1 remapping
  On the M3 Wildfire board pin PB9 is wired as TX and pin PB8 is wired as RX.
  Which then makes the proper connection through the CAN transceiver SN65HVD230
  out to the CAN D-type 9-pn male connector where pin 2 is CANL and pin 7 is CANH.

    CONFIG_STM32_TIM1_FULL_REMAP
    CONFIG_STM32_TIM1_PARTIAL_REMAP
    CONFIG_STM32_TIM2_FULL_REMAP
    CONFIG_STM32_TIM2_PARTIAL_REMAP_1
    CONFIG_STM32_TIM2_PARTIAL_REMAP_2
    CONFIG_STM32_TIM3_FULL_REMAP
    CONFIG_STM32_TIM3_PARTIAL_REMAP
    CONFIG_STM32_TIM4_REMAP
    CONFIG_STM32_USART1_REMAP
    CONFIG_STM32_USART2_REMAP
    CONFIG_STM32_USART3_FULL_REMAP
    CONFIG_STM32_USART3_PARTIAL_REMAP
    CONFIG_STM32_SPI1_REMAP
    CONFIG_STM32_SPI3_REMAP
    CONFIG_STM32_I2C1_REMAP
    CONFIG_STM32_CAN1_REMAP1
    CONFIG_STM32_CAN1_REMAP2
    CONFIG_STM32_CAN2_REMAP

  JTAG Enable settings (by default JTAG-DP and SW-DP are disabled):
    CONFIG_STM32_JTAG_FULL_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
    CONFIG_STM32_JTAG_NOJNTRST_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
      but without JNTRST.
    CONFIG_STM32_JTAG_SW_ENABLE - Set JTAG-DP disabled and SW-DP enabled

  STM32F103Z specific device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=1,2,3) or UART
           m (m=4,5) for the console and ttys0 (default is the USART1).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

    CONFIG_STM32_SPI_INTERRUPTS - Select to enable interrupt driven SPI
      support. Non-interrupt-driven, poll-waiting is recommended if the
      interrupt rate would be to high in the interrupt driven case.
    CONFIG_STM32_SPI_DMA - Use DMA to improve SPI transfer performance.
      Cannot be used with CONFIG_STM32_SPI_INTERRUPT.

    CONFIG_STM32_SDIO_DMA - Support DMA data transfers.  Requires
      CONFIG_STM32_SDIO and CONFIG_STM32_DMA2.
    CONFIG_STM32_SDIO_PRI - Select SDIO interrupt priority.  Default: 128
    CONFIG_STM32_SDIO_DMAPRIO - Select SDIO DMA interrupt priority.
      Default:  Medium
    CONFIG_STM32_SDIO_WIDTH_D1_ONLY - Select 1-bit transfer mode.  Default:
      4-bit transfer mode.

  M3 Wildfire CAN Configuration

    CONFIG_CAN - Enables CAN support (one or both of CONFIG_STM32_CAN1 or
      CONFIG_STM32_CAN2 must also be defined)
    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_CAN_FIFOSIZE - The size of the circular buffer of CAN messages.
      Default: 8
    CONFIG_CAN_NPENDINGRTR - The size of the list of pending RTR requests.
      Default: 4
    CONFIG_CAN_LOOPBACK - A CAN driver may or may not support a loopback
      mode for testing. The STM32 CAN driver does support loopback mode.
    CONFIG_STM32_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN1
      is defined.
    CONFIG_STM32_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN2
      is defined.
    CONFIG_STM32_CAN_TSEG1 - The number of CAN time quanta in segment 1.
      Default: 6
    CONFIG_STM32_CAN_TSEG2 - the number of CAN time quanta in segment 2.
      Default: 7
    CONFIG_STM32_CAN_REGDEBUG - If CONFIG_DEBUG_FEATURES is set, this will generate an
      dump of all CAN registers.

  M3 Wildfire LCD Hardware Configuration

    CONFIG_LCD_LANDSCAPE - Define for 320x240 display "landscape"
      support. Default is this 320x240 "landscape" orientation
      (this setting is informative only... not used).
    CONFIG_LCD_PORTRAIT - Define for 240x320 display "portrait"
      orientation support.  In this orientation, the M3 Wildfire's
      LCD ribbon cable is at the bottom of the display. Default is
      320x240 "landscape" orientation.
    CONFIG_LCD_RPORTRAIT - Define for 240x320 display "reverse
      portrait" orientation support.  In this orientation, the
      M3 Wildfire's LCD ribbon cable is at the top of the display.
      Default is 320x240 "landscape" orientation.
    CONFIG_LCD_BACKLIGHT - Define to support a backlight.
    CONFIG_LCD_PWM - If CONFIG_STM32_TIM1 is also defined, then an
      adjustable backlight will be provided using timer 1 to generate
      various pulse widthes.  The granularity of the settings is
      determined by CONFIG_LCD_MAXPOWER.  If CONFIG_LCD_PWM (or
      CONFIG_STM32_TIM1) is not defined, then a simple on/off backlight
      is provided.
    CONFIG_LCD_RDSHIFT - When reading 16-bit gram data, there appears
      to be a shift in the returned data.  This value fixes the offset.
      Default 5.

    The LCD driver dynamically selects the LCD based on the reported LCD
    ID value.  However, code size can be reduced by suppressing support for
    individual LCDs using:

    CONFIG_STM32_AM240320_DISABLE
    CONFIG_STM32_SPFD5408B_DISABLE

Configurations
==============

Each M3 Wildfire configuration is maintained in a sub-directory and
can be selected as follow:

    tools/configure.sh fire-stm32v2:<subdir>

Where <subdir> is one of the following:

  nsh
  ---
    Configure the NuttShell (nsh) located at examples/nsh. The nsh configuration
    contains support for some built-in applications that can be enabled by making
    some additional minor change to the configuration file.

    Reconfiguring:  This configuration uses to the kconfig-mconf configuration tool
    to control the configuration.  See the section entitled "NuttX Configuration
    Tool" in the top-level README.txt file.

    Start Delays:  If no SD card is present in the slot, or if the network is not
    connected, then there will be long start-up delays before you get the NSH
    prompt.  If I am focused on ENC28J60 debug, I usually disable MMC/SD so that
    I don't have to bother with the SD card:

      CONFIG_STM32_SDIO=n
      CONFIG_MMCSD=n

    STATUS:  The board port is basically functional. Not all features have been
    verified.  The ENC28J60 network is not yet functional.  Networking is
    enabled by default in this configuration for testing purposes.  To use this
    configuration, the network must currently be disabled.  To do this using
    the kconfig-mconf configuration tool:

    > make menuconfig

    Then de-select "Networking Support" -> "Networking Support"

    UPDATE:  The primary problem with the ENC29J60 is a v2 board issue:  The
    SPI FLASH and the ENC28J60 shared the same SPI chip select signal (PA4-SPI1-NSS).
    In order to finish the debug of the ENC28J60, it may be necessary to lift
    the SPI FLASH chip select pin from the board.

README.txt
==========

This is the README file for the port of NuttX to the Freescale Kinetis
KwiStick K40.  Refer to the Freescale web site for further information
about this part:
http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=KWIKSTIK-K40

The Kwikstik is used with the FreeScale Tower System (mostly just to
provide a simple UART connection)

Contents
========

  o Kinetis KwikStik-K40 Features
  o Kinetis KwikStik-K60 Pin Configuration
    - On-Board Connections
    - Connections via the General Purpose Tower Plug-in (TWRPI) Socket
    - Connections via the Tower Primary Connector Side A
    - Connections via the Tower Primary Connector Side B
    - TWR-SER Serial Board Connection
  o KwikStik-K40-specific Configuration Options
  o Configurations

Kinetis KwikStik-K40 Features:
==============================

  o Kinetis K40 MCU in 144 LQFP
    - 100 MHz ARM Cortex-M4 core
    - 256Kb program flash, 256Kb FlexMemory
    - Full-speed USB 2.0 device
    - Low-pwer segment LCD controller
    - SPI, UART, CAN and more
  o Large segment LCD display with 306 segments
  o 2.3mm audio output and 2 micro USB connectors
  o Omnidirectional microphone and a buzzer
  o On-board Segger J-Link debugger interface
  o Infrared communication port
  o microSD card slot
  o Capacitive touch sensing interface
  o Freescale Tower System connectivity for UART, timers, CAN, SPI, I2C, and DAC
  o Freescale Tower plug-in (TWRPI) socket connectivity for ADC, SPI, I2C, and GPIO

Kinetis KwikStik-K40 Pin Configuration
======================================

On-Board Connections
------------------- -------------------------- -------- -------------------
FEATURE             CONNECTION                 PORT/PIN PIN FUNCTION
------------------- -------------------------- -------- -------------------
Audio Jack Output   Audio Amp On               PTE28    PTE28
                    Audio Output               DAC1_OUT DAC1_OUT
                    Volume Up                  PTD10    PTD10
                    Volume Down                PTD11    PTD11
Buzzer              Audio Out                  PTA8     FTM1_CH0
Microphone          Microphone input           PTA7     ADC0_SE10
SD Card Slot        SD Clock                   PTE2     SDHC0_DCLK
                    SD Command                 PTE3     SDHC0_CMD
                    SD Data0                   PTD12    SDHC0_D4
                    SD Data1                   PTD13    SDHC0_D5
                    SD Data2                   PTD14    SDHC0_D6
                    SD Data3                   PTD15    SDHC0_D7
                    SD Card Detect             PTE27    PTE27
                    SD Card On                 PTE6     PTE6
Infrared Port       IR Transmit                PTE4     IR_TX
                    IR Receive                 PTA13    CMP2_IN0
Touch Pads          E1 / Touch                 PTB0     TSI0_CH0
                    E2 / Touch                 PTA4     TSI0_CH5
                    E3 / Touch                 PTA24    PTA24
                    E4 / Touch                 PTA25    PTA25
                    E5 / Touch                 PTA26    PTA26
                    E6 / Touch                 PTA27    PTA27

Connections via the General Purpose Tower Plug-in (TWRPI) Socket
------------------- -------------------------- -------- -------------------
FEATURE             CONNECTION                 PORT/PIN PIN FUNCTION
------------------- -------------------------- -------- -------------------
General Purpose     TWRPI AN0 (J8 Pin 8)       ?        ADC0_DP0/ADC1_DP3
TWRPI Socket        TWRPI AN1 (J8 Pin 9)       ?        ADC0_DM0/ADC1_DM3
                    TWRPI AN2 (J8 Pin 12)      ?        ADC1_DP0/ADC0_DP3
                    TWRPI ID0 (J8 Pin 17)      ?        ADC0_DP1
                    TWRPI ID1 (J8 Pin 18)      ?        ADC0_DM1
                    TWRPI I2C SCL (J9 Pin 3)   PTC10    I2C1_SCL
                    TWRPI I2C SDA (J9 Pin 4)   PTC11    I2C1_SDA
                    TWRPI SPI MISO (J9 Pin 9)  PTB23    SPI2_SIN
                    TWRPI SPI MOSI (J9 Pin 10) PTB22    SPI2_SOUT
                    TWRPI SPI SS (J9 Pin 11)   PTB20    SPI2_PCS0
                    TWRPI SPI CLK (J9 Pin 12)  PTB21    SPI2_SCK
                    TWRPI GPIO0 (J9 Pin 15)    PTC12    PTC12
                    TWRPI GPIO1 (J9 Pin 16)    PTB9     PTB9
                    TWRPI GPIO2 (J9 Pin 17)    PTB10    PTB10
                    TWRPI GPIO3 (J9 Pin 18)    PTC5     PTC5
                    TWRPI GPIO4 (J9 Pin 19)    PTA5     PTA5

The KwikStik features an expansion card-edge connector that interfaces to the Primary Elevator board in a Tower system (Primary side).

Connections via the Tower Primary Connector Side A
--- -------------------- --------------------------------
PIN NAME                 USAGE
--- -------------------- --------------------------------
A9  GPIO9 / CTS1         PTE10/UART_CTS
A43 RXD1                 PTE9/UART_RX
A44 TXD1                 PTE8/UART_TX
A63 RSTOUT_b             PTA9/FTM1_CH1

Connections via the Tower Primary Connector Side B
--- -------------------- --------------------------------
PIN NAME                 USAGE
--- -------------------- --------------------------------
B21 GPIO1 / RTS1         PTE7/UART_RTS
B37 PWM7                 PTA8/FTM1_CH0
B38 PWM6                 PTA9/FTM1_CH1
B41 CANRX0               PTE25/CAN1_RX
B42 CANTX0               PTE24/CAN1_TX
B44 SPI0_MISO            PTA17/SPI0_SIN
B45 SPI0_MOSI            PTA16/SPI0_SOUT
B46 SPI0_CS0_b           PTA14/SPI0_PCS0
B48 SPI0_CLK             PTA15/SPI0_SCK
B50 SCL1                 PTE1/I2C1_SCL
B51 SDA1                 PTE0/I2C1_SDA
B52 GPIO5 / SD_CARD_DET  PTA16

TWR-SER Serial Board Connection
===============================

The serial board connects into the tower and then maps to the tower pins to
yet other functions (see TWR-SER.pdf).

For the serial port, the following jumpers are required:

  J15: 1-2 (default)
  J17: 1-2 (default)
  J18: 1-2 (default)
  J19: 1-2 (default)

The two connections map as follows:

  A41 RXD0  - Not connected
  A42 TXD0  - Not connected
  A43 RXD1  - ELE_RXD (connects indirectory to DB-9 connector J8)
  A44 TXD1  - ELE_TXD (connects indirectory to DB-9 connector J8)

Finally, we can conclude that

  UART5 (PTE8/9) is associated with the DB9 connector

KwikStik-K40-specific Configuration Options
============================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM4=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=k40

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_MK40X256VLQ100

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=kwikstik-k40 (for the KwikStik-K40 development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_KWIKSTIK_K40=y

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

  Individual subsystems can be enabled:

    CONFIG_KINETIS_TRACE    -- Enable trace clocking on power up.
    CONFIG_KINETIS_FLEXBUS  -- Enable flexbus clocking on power up.
    CONFIG_KINETIS_UART0    -- Support UART0
    CONFIG_KINETIS_UART1    -- Support UART1
    CONFIG_KINETIS_UART2    -- Support UART2
    CONFIG_KINETIS_UART3    -- Support UART3
    CONFIG_KINETIS_UART4    -- Support UART4
    CONFIG_KINETIS_UART5    -- Support UART5
    CONFIG_KINETIS_ENET     -- Support Ethernet (K60 only)
    CONFIG_KINETIS_RNGB     -- Support the random number generator(K60 only)
    CONFIG_KINETIS_FLEXCAN0 -- Support FlexCAN0
    CONFIG_KINETIS_FLEXCAN1 -- Support FlexCAN1
    CONFIG_KINETIS_SPI0     -- Support SPI0
    CONFIG_KINETIS_SPI1     -- Support SPI1
    CONFIG_KINETIS_SPI2     -- Support SPI2
    CONFIG_KINETIS_I2C0     -- Support I2C0
    CONFIG_KINETIS_I2C1     -- Support I2C1
    CONFIG_KINETIS_I2S      -- Support I2S
    CONFIG_KINETIS_DAC0     -- Support DAC0
    CONFIG_KINETIS_DAC1     -- Support DAC1
    CONFIG_KINETIS_ADC0     -- Support ADC0
    CONFIG_KINETIS_ADC1     -- Support ADC1
    CONFIG_KINETIS_CMP      -- Support CMP
    CONFIG_KINETIS_VREF     -- Support VREF
    CONFIG_KINETIS_SDHC     -- Support SD host controller
    CONFIG_KINETIS_FTM0     -- Support FlexTimer 0
    CONFIG_KINETIS_FTM1     -- Support FlexTimer 1
    CONFIG_KINETIS_FTM2     -- Support FlexTimer 2
    CONFIG_KINETIS_LPTMR0   -- Support the low power timer 0
    CONFIG_KINETIS_RTC      -- Support RTC
    CONFIG_KINETIS_SLCD     -- Support the segment LCD (K40 only)
    CONFIG_KINETIS_EWM      -- Support the external watchdog
    CONFIG_KINETIS_CMT      -- Support Carrier Modulator Transmitter
    CONFIG_KINETIS_USBOTG   -- Support USB OTG (see also CONFIG_USBHOST and CONFIG_USBDEV)
    CONFIG_KINETIS_USBDCD   -- Support the USB Device Charger Detection module
    CONFIG_KINETIS_LLWU     -- Support the Low Leakage Wake-Up Unit
    CONFIG_KINETIS_TSI      -- Support the touch screen interface
    CONFIG_KINETIS_FTFL     -- Support FLASH
    CONFIG_KINETIS_DMA      -- Support DMA
    CONFIG_KINETIS_CRC      -- Support CRC
    CONFIG_KINETIS_PDB      -- Support the Programmable Delay Block
    CONFIG_KINETIS_PIT      -- Support Programmable Interval Timers
    CONFIG_ARM_MPU          -- Support the MPU

  Kinetis interrupt priorities (Default is the mid priority)  These should
  not be set because they can cause unhandled, nested interrupts.  All
  interrupts need to be at the default priority in the current design.

    CONFIG_KINETIS_UART0PRIO
    CONFIG_KINETIS_UART1PRIO
    CONFIG_KINETIS_UART2PRIO
    CONFIG_KINETIS_UART3PRIO
    CONFIG_KINETIS_UART4PRIO
    CONFIG_KINETIS_UART5PRIO

    CONFIG_KINETIS_SDHC_PRIO

  PIN Interrupt Support

    CONFIG_KINETIS_GPIOIRQ          -- Enable pin interrupt support.  Also needs
      one or more of the following:
    CONFIG_KINETIS_PORTAINTS -- Support 32 Port A interrupts
    CONFIG_KINETIS_PORTBINTS -- Support 32 Port B interrupts
    CONFIG_KINETIS_PORTCINTS -- Support 32 Port C interrupts
    CONFIG_KINETIS_PORTDINTS -- Support 32 Port D interrupts
    CONFIG_KINETIS_PORTEINTS -- Support 32 Port E interrupts

  Kinetis K40 specific device driver settings

    CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn (n=0..5) for the
      console and ttys0 (default is the UART0).
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_UARTn_BAUD - The configure BAUD of the UART.
    CONFIG_UARTn_BITS - The number of bits.  Must be either 8 or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity

  KwikStik-K40 LCD Hardware Configuration

    CONFIG_LCD_LANDSCAPE - Define for 320x240 display "landscape"
      support. Default is this 320x240 "landscape" orientation
      (this setting is informative only... not used).
    CONFIG_LCD_PORTRAIT - Define for 240x320 display "portrait"
      orientation support.  In this orientation, the KwikStik-K40's
      LCD ribbon cable is at the bottom of the display. Default is
      320x240 "landscape" orientation.
    CONFIG_LCD_RPORTRAIT - Define for 240x320 display "reverse
      portrait" orientation support.  In this orientation, the
      KwikStik-K40's LCD ribbon cable is at the top of the display.
      Default is 320x240 "landscape" orientation.
    CONFIG_LCD_BACKLIGHT - Define to support an adjustable backlight
      using timer 1.  The granularity of the settings is determined
      by CONFIG_LCD_MAXPOWER.  Requires CONFIG_KINETIS_TIM1.

Configurations
==============

Each KwikStik-K40 configuration is maintained in a sub-directory and
can be selected as follow:

    tools/configure.sh kwikstik-k40:<subdir>

Where <subdir> is one of the following:

  ostest:
  ------
    This configuration directory, performs a simple OS test using
    examples/ostest.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default platform/toolchain:

       CONFIG_HOST_LINUX=y                 : Linux (Cygwin under Windows okay too).
       CONFIG_ARM_TOOLCHAIN_BUILDROOT=y : Buildroot (arm-nuttx-elf-gcc)
       CONFIG_ARM_TOOLCHAIN_BUILDROOT_OABI=y      : The older OABI version
       CONFIG_RAW_BINARY=y                 : Output formats: ELF and raw binary

README
======

  This port is for the FRDM-K28F development board that features the
  MK28FN2M0VMI15 MCU.  The features of the FRDM-K28F board are:

    o Kinetis MK28FN2M0VMI15 MCU (ARM Cortex-M4 at150 MHz, 1 MB SRAM, 2 MB flash,
      HS and FS USB, 169 MAPBGA package).
    o Kinetis K20 MCU (K20DX128VFM5) based OpenSDA circuit.
    o Dual-role high-speed and full-speed USB interface with the micro-B USB
      connector via the high-speed switch.
    o One 256 Mbit (32 MB) on-board QuadSPI memory at 1.8 V.
    o One 128 Mbit (16 MB) on-board mobile SDRAM memory at 3.3 V.
    o Multiple independent voltage domains: VDD_CORE, VDD, VBAT, and VDDIO_E.
    o FlexIO socket which enables you to connect an optional TFT Proto 5"
      CAPACITIVE from MikroElektronika (5" display board with capacitive touch).
    o Easy access to the MCU input/output through Arduino R3-compatible I/O
      connectors to connect external add-on boards.
    o Flexible power supply option—OpenSDA v2.2 USB, Kinetis K28 USB, or an
      external source.
    o FXOS8700CQ—6-axis sensor with accelerometer and magnetometer.
    o RGB LED.
    o Two mechanical push-buttons for user input and one for the reset.
    o Programmable OpenSDA v2.2 debug circuit supporting the DAP-Link interface
      software which provides:

      - Mass-Storage Device (MSD) flash programming interface.
      - CMSIS-DAP debug interface over a driverless USB HID connection providing
        run-control debugging and compatibility with the IDE tools.
      - Virtual serial port interface.
      - Open-source CMSIS-DAP software project.

Contents
========

  o STATUS
  o Serial Console
  o LEDs and Buttons
  o Configurations

STATUS
======

  2018-05-05:  The basic NSH port appears to be fully functional.  I do see
    one anomaly:  There is a significant, long start up delay.  This delay
    seems to be variable from immediate to several seconds.  I have not
    studied the cause, but there are the symptoms:

    This debug on, I see this output after a reset:

      __start: Reset status: 00:00

      NuttShell (NSH) NuttX-7.24
      nsh> help
      help usage:  help [-v] [<cmd>]

        [           cmp         false       mkdir       rm          true
        ?           dirname     free        mh          rmdir       uname
        basename    dd          help        mount       set         umount
        break       df          hexdump     mv          sh          unset
        cat         echo        kill        mw          sleep       usleep
        cd          exec        ls          ps          test        xd
        cp          exit        mb          pwd         time

      Builtin Apps:
      nsh>

  The delay occurs between the reset and the __start debug output.  __start
  is the reset handler so this is very early in the logic.  On a reset, it
  vectors to __start.  That particular debug message is output after most
  low-level initialization has occurred.  So the delay is something within
  the initialization sequence at the beginning of __start.  My suspicion is
  the delays associated with the clock configuration.

Serial Console
==============

    ----- --------------- -------------------------------
    GPIO  LPUART FUNCTION BOARD CONFIGURATION
    ----- --------------- -------------------------------
    PTA1  LPUART0_RX      PTA1  GPIO0
    PTA15 LPUART0_RX      PTA15 FXIO0_D21
    PTB14 LPUART0_RX      PTB14
    PTB16 LPUART0_RX      PTB16 SDRAM_D17
    PTC25 LPUART0_RX      PTC25 LPUART0_RX_TGTMCU
    PTD6  LPUART0_RX      PTD6  Arduino_D17_ADC0_SE7b
    PTA2  LPUART0_TX      PTA2  INT
    PTA14 LPUART0_TX      PTA14 FXIO0_D20
    PTB15 LPUART0_TX            N/C
    PTB17 LPUART0_TX      PTB17 SDRAM_D16
    PTC24 LPUART0_TX      PTC24 LPUART0_TX_TGTMCU
    PTD7  LPUART0_TX      PTD7  SDRAM_CKE
    PTA3  LPUART0_RTS     PTA3
    PTA17 LPUART0_RTS     PTA17 FXIO0_D23
    PTB2  LPUART0_RTS     PTB2  Arduino_D19_ADC0_SE12/I2C0_SCL/SDRAM_WE
    PTB12 LPUART0_RTS     PTB12 Arduino_D5_FTM1_CH0/FTM0_CH4
    PTC27 LPUART0_RTS     PTC27 FXOS8700CQ_RESET
    PTD4  LPUART0_RTS     PTD4  SDRAM_A10
    PTA0  LPUART0_CTS     PTA0  K28F_SWD_CLK
    PTA16 LPUART0_CTS     PTA16 FXIO0_D22
    PTB3  LPUART0_CTS     PTB3  Arduino_D18_ADC0_SE13/I2C0_SDA/SDRAM_CS0
    PTB13 LPUART0_CTS     PTB13 Arduino_D6_FTM1_CH1/FTM0_CH5
    PTC26 LPUART0_CTS     PTC26 FXOS8700CQ_INT
    PTD5  LPUART0_CTS     PTD5  SDRAM_A9
    ----- --------------- -------------------------------
    PTD8  LPUART1_RX      PTD8  FXIO0_D24
    PTC3  LPUART1_RX      PTC3  CLKOUT
    PTE1  LPUART1_RX      PTE1  QSPIA0_SCLK
    PTC4  LPUART1_TX      PTC4  SDRAM_A19
    PTD9  LPUART1_TX      PTD9  FXIO0_D25
    PTE0  LPUART1_TX      PTE0  QSPIA0_DATA3
    PTD10 LPUART1_RTS     PTD10 FXIO0_D26
    PTC1  LPUART1_RTS     PTC1  SDRAM_A21
    PTE3  LPUART1_RTS     PTE3  QSPIA0_DATA2
    PTC2  LPUART1_CTS     PTC1  SDRAM_A21
    PTD11 LPUART1_CTS     PTD11 FXIO0_D27
    PTE2  LPUART1_CTS     PTE2  QSPIA0_DATA0
    ----- --------------- -------------------------------
    PTA25 LPUART2_RX      PTA25 SDHC0_D0/Arduino_D0_LPUART2_RX
    PTD2  LPUART2_RX      PTD2  SDRAM_A12
    PTE13 LPUART2_RX            N/C
    PTE17 LPUART2_RX            N/C
    PTA24 LPUART2_TX      PTA24 SDHC0_D1/Arduino_D1_LPUART2_TX
    PTD3  LPUART2_TX      PTD3  SDRAM_A11
    PTE12 LPUART2_TX      PTE12 I2S0_TX_BCLK
    PTE16 LPUART2_TX            N/C
    PTD0  LPUART2_RTS     PTD0  Button_LLWU_P12
    PTA27 LPUART2_RTS     PTA27 SDHC0_CMD
    PTE19 LPUART2_RTS           N/C
    PTA26 LPUART2_CTS     PTA26 SDHC0_DCLK
    PTD1  LPUART2_CTS     PTD1  Arduino_D16_ADC0_SE5b
    PTE18 LPUART2_CTS           N/C
    ----- --------------- -------------------------------
    PTA29 LPUART3_RX      PTA29 SDHC0_D2
    PTB10 LPUART3_RX      PTB10 SDRAM_D19
    PTC16 LPUART3_RX      PTC16 SDRAM_DQM2
    PTE5  LPUART3_RX      PTE5  QSPIA0_SS0/USB0_SOF_OUT
    PTA28 LPUART3_TX      PTA28 SDHC0_D3
    PTB11 LPUART3_TX      PTB11 SDRAM_D18
    PTC17 LPUART3_TX      PTC17 SDRAM_DQM3
    PTE4  LPUART3_TX      PTE4  QSPIA0_DATA1
    PTB8  LPUART3_RTS     PTB8  SDRAM_D21
    PTA31 LPUART3_RTS     PTA31
    PTC18 LPUART3_RTS     PTC18 Arduino_D7
    PTE7  LPUART3_RTS     PTE7  I2S0_RXD0/LEDRGB_GREEN
    PTA30 LPUART3_CTS     PTA30
    PTB9  LPUART3_CTS     PTB9  SDRAM_D20
    PTC19 LPUART3_CTS     PTC19 Arduino_D8
    PTE6  LPUART3_CTS     PTE6  I2S0_MCK/LEDRGB_RED
    ----- --------------- -------------------------------
    PTA21 LPUART4_RX      PTA21 TE/FXIO0_D9
    PTC14 LPUART4_RX      PTC14 SDRAM_D25
    PTE21 LPUART4_RX            N/C
    PTA20 LPUART4_TX      PTA20 RD/FXIO0_D8
    PTC15 LPUART4_TX      PTC15 SDRAM_D24
    PTE20 LPUART4_TX            N/C
    PTA23 LPUART4_RTS     PTA23 WR/FXIO0_D7
    PTC12 LPUART4_RTS     PTC12 SDRAM_D27
    PTE23 LPUART4_RTS           N/C
    PTA22 LPUART4_CTS     PTA22 CS/FXIO0_D6
    PTC13 LPUART4_CTS     PTC13 SDRAM_D26
    PTE22 LPUART4_CTS           N/C
    ----- --------------- -------------------------------

  Arduino RS-232 Shield
  ---------------------

    ----- --------------- -------------------------------
    GPIO  LPUART FUNCTION BOARD CONFIGURATION
    ----- --------------- -------------------------------
    PTA25 LPUART2_RX      PTA25 SDHC0_D0/Arduino_D0_LPUART2_RX
    PTA24 LPUART2_TX      PTA24 SDHC0_D1/Arduino_D1_LPUART2_TX
    ----- --------------- -------------------------------

  Note:  PTA24 and PTA25 are shared between Micro SD Card circuit and
  Arduino connectors. Remove R106 and R107 or R94 and R11 as necessary to
  prevent contention.

  Virtual serial port
  -------------------

  A serial port connection is available between the OpenSDA v2.2 MCU and
  pins PTC24 and PTC25 of the K28 MCU:

    ----- --------------- -------------------------------
    GPIO  LPUART FUNCTION BOARD CONFIGURATION
    ----- --------------- -------------------------------
    PTC25 LPUART0_RX      PTC25 LPUART0_RX_TGTMCU
    PTC24 LPUART0_TX      PTC24 LPUART0_TX_TGTMCU
    ----- --------------- -------------------------------

  Default Console
  ---------------

    Unless otherwise noted, LPUART0 (the virtual serial port) is the console
    used in all Freedom-K28F configurations.

  RGB LED
  -------
  An RGB LED is connected through GPIO as shown below:

    LED    K28
    ------ -------------------------------------------------------
    RED    PTE6
    GREEN  PTE7
    BLUE   PTE8

  If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
  the Freedom K28.  Usage of these LEDs is defined in include/board.h and
  src/k28_leds.c.  The following definitions describe how NuttX controls the
  LEDs:

    SYMBOL                Meaning                 LED state
                                                  RED   GREEN  BLUE
    -------------------  -----------------------  -----------------
    LED_STARTED          NuttX has been started    OFF  OFF  OFF
    LED_HEAPALLOCATE     Heap has been allocated   OFF  OFF  ON
    LED_IRQSENABLED      Interrupts enabled        OFF  OFF  ON
    LED_STACKCREATED     Idle stack created        OFF  ON   OFF
    LED_INIRQ            In an interrupt          (no change)
    LED_SIGNAL           In a signal handler      (no change)
    LED_ASSERTION        An assertion failed      (no change)
    LED_PANIC            The system has crashed    FLASH OFF OFF
    LED_IDLE             K28 is in sleep mode     (Optional, not used)

  Buttons
  -------
  Two push button switches, SW2 and SW3, are available on the FRDM-K28F
  board. SW2 is connected to PTA4 and SW3 is connected to PTD0.
  Beside the general purpose IO function, both SW2 and SW3 can be used
  as a low-leakage wakeup (LLWU) source.

    Switch    GPIO Function
    --------- ---------------------------------------------------------------
    SW2       PTA4/NMI_B
    SW3       PTD0/LLWU_P12


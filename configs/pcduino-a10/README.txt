README
======

  This directory contains the port of NuttX to the pcDuino v1 board
  See http://www.pcduino.com/ for information about pcDuino Lite, v1,
  and v2.  These boards are based around the Allwinner A10 Cortex-A8 CPU.
  I have not compared these boards in detail, but I believe that the
  differences are cosmetic.  This port was developed on the v1 board, but
  the others may be compatible:
  
  pcDuino Lite (See http://www.pcduino.com/?page_id=1707)

    ITEMS                DETAILS
    -------------------- ---------------------------------------------------
    CPU                  1GHz ARM Cortex A8
    GPU                  OpenGL ES2.0, OpenVG 1.1 Mali 400 core
    DRAM                 512B
    Onboard Storage      NO Flash, microSD card (TF) slot for up to 32GB
    Video Output         HDMI
    Extension Interface  2.54mm Headers
    Network interface    10/100Mbps RJ45 and USB WiFi extension (not included)
    Power                5V, 2000mA
    Overall Size         125mm X 52mm

  pcDuino v1 (http://www.pcduino.com/?page_id=12)

    ITEMS                DETAILS
    -------------------- ---------------------------------------------------
    Items                Details
    CPU                  1GHz ARM Cortex A8
    GPU                  OpenGL ES2.0, OpenVG 1.1 Mali 400 core
 *  DRAM                 1GB
 *  Onboard Storage      2GB Flash, microSD card (TF) slot for up to 32GB
    Video Output         HDMI
    Extension Interface  2.54mm Headers
    Network interface    10/100Mbps RJ45 and USB WiFi extension (not included)
    Power                5V, 2000mA
    Overall Size         125mm X 52mm

  pcDuino v2 (http://www.pcduino.com/?page_id=1618)

    ITEMS                DETAILS
    -------------------- ---------------------------------------------------
    Items                Details
    CPU                  1GHz ARM Cortex A8
    GPU                  OpenGL ES2.0, OpenVG 1.1 Mali 400 core
    DRAM                 1GB
    Onboard Storage      2GB Flash, microSD card (TF) slot for up to 32GB
    Video Output         HDMI
 *  Extension Interface  Arduino Headers
 *  Network interface    10/100Mbps RJ45 and on-board WiFi module
    Power                5V, 2000mA
    Overall Size         125mm X 52mm

  Main features of the Allwinner A10
  (See http://www.allwinnertech.com/en/product/a10.html):

  CPU
    - ARM Cortex™-A8
    - 32KB I-Cache
    - 32KB D-Cache
    - 256KB L2 Cache

  GPU
    - ARM Mali-400

  Video
    - UHD 2160P video decoding
    - 3D video decoding
    - Support various video decoding formats, including VP8, AVS, H. 264
      MVC, VC-1, MPEG-1,2,4, etc
    - H.264 HP video encoding up to 1080p @ 30 fps or dual-channel 720p @ 30
      fps

  Display
    - Multi-channel HD display
    - Integrated HDMI 1.4
    - YPbPr, CVBS, VGA
    - Multiple LCD interfaces, including CPU, RGB, LVDS up to Full HD

  Memory
    - 32-bit DDR2/DDR3
    - Memory capacity up to 16G bits
    - SLC/MLC/TLC/DDR NAND
    - 8 flash chips, 64-bit ECC

        Memory capacity up to 64GB
        Support NAND of 5xnm, 4xnm, 3xnm, 2xnm, etc
        Support NAND of Samsung, Toshiba, Hynix, etc

  Boot Devices
    - NAND Flash
    - SPI NOR Flash
    - SD Card
    - USB

Contents
========

  - pcDuino v1 Connectors
  - Serial Console
  - LEDs
  - Buttons

pcDuino v1 Connectors
=====================

  TOP
  ---
  - HDMI
  - RJ45
  - USB Host (2)

  - J11
     1. UART-Rx / GPIO0            UART2_RX
     2. UART-Tx / GPIO1            UART2_TX
     3. GPIO3 / GPIO2              GPIO2
     4. PWM0 / GPIO3               PWM0
     5. GPIO4                      GPIO3
     6. PWM1 / GPIO5               PWM1
     7. PWM2 /GPIO6                PWM2
     8. GPIO7                      GPIO4

  - J8
     1. GPIO8                      GPIO5
     2. PWM3 / GPIO9               PWM3
     3. SPI_CS / GPIO10 / PWM4     SPI0_CS
     4. SPI_MOSI / GPIO11 / PWM5   SPI0_MOSI
     5. SPI_MISO / GPIO12          SPI0_MISO
     6. SPI_CLK / GPIO13           SPI0_CLK
     7. Gnd
     8. ARef
     9. I2C-SDA                    TWI2_SDA
    10. I2C-SCK                    TWI2_SCK

  - J12
     1. ADC0
     2. ADC1
     3. ADC2
     4. ADC3
     5. ADC4
     6. ADC5

  - J9
     1. 5V
     2. Gnd
     3. Gnd
     4. 5V
     5. 3.3V
     6. Reset
     7. 5V
     8. NC

  - J5 Debug Port
     1. Rx                         UART0-RX
     2. Gnd                        GND
     2. Tx                         UART0-TX

  - J6 SPI2
     1. SPI2_MISO
     2. DC_5V
     3. SPI2_CLK
     4. SPI2_MOSI
     5. RESET#
     6. GND

  - J7 SPI0
     1. SPI0_MISO
     2. DC_5V
     3. SPI0_CLK
     4. SPI0_MOSI
     5. RESET#
     6. GND

  - J10
     1. GPIO6
     2. GPIO8
     3. GPIO7
     4. GPIO9

  Bottom
  ------
  - USB OTG
  - DC Power IN (USB)
  - microSD card slot

Serial Console
==============

  Two UART connections are available:

    UART0 is available on J5 Debug Port.
    UART2 is available on J11

  By default, the serial console will be provided on UART0 in all of these
  configurations.

LEDs
====

  The pcDuino v1 has four green LEDs; three can be controlled from software.
  Two are tied to ground and, hence, illuminated by driving the output pins
  to a high value:

    1. LED1 SPI0_CLK  SPI0_CLK/UART5_RX/EINT23/PI11
    2. LED5 IPSOUT    From the PMU (not controllable by software)

  And two are pull high and, hence, illuminated by grounding the output:

    3. LED3 RX_LED    LCD1_D16/ATAD12/KP_IN6/SMC_DET/EINT16/CSI1_D16/PH16
    4. LED4 TX_LED    LCD1_D15/ATAD11/KP_IN5/SMC_VPPPP/EINT15/CSI1_D15/PH15

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/stm32_leds.c. The LEDs are used to encode OS-related
  events as follows:

    SYMBOL            Meaning                      LED state
                                               LED1 LED3 LED4
    ----------------- -----------------------  ---- ---- ------------
    LED_STARTED       NuttX has been started   ON   OFF  OFF
    LED_HEAPALLOCATE  Heap has been allocated  OFF  ON   OFF
    LED_IRQSENABLED   Interrupts enabled       ON   ON   OFF
    LED_STACKCREATED  Idle stack created       ON   ON   OFF
    LED_INIRQ         In an interrupt          N/C  N/C  Soft glow
    LED_SIGNAL        In a signal handler      N/C  N/C  Soft glow
    LED_ASSERTION     An assertion failed      N/C  N/C  Soft glow
    LED_PANIC         The system has crashed   N/C  N/C  2Hz Flashing
    LED_IDLE          MCU is is sleep mode         Not used

  After booting, LED1 and 3 are not longer used by the system and can be used for
  other purposes by the application (Of course, all LEDs are available to the
  application if CONFIG_ARCH_LEDS is not defined.

Buttons
=======

  There are a total of five switches on-board.  All pulled high and, hence,
  will be sensed as low when closed.

    SW1 Reset     (not available to software)
    SW2 UBOOT     UBOOT_SEL (?)
    SW3 Key_Back  LCD1_D17/ATAD13/KP_IN7/SMC_VCCEN/EINT17/CSI1_D17/PH17
    SW4 Key_Home  LCD1_D18/ATAD14/KP_OUT0/SMC_SLK/EINT18/CSI1_D18/PH18
    SW5 Key_Menu  LCD1_D19/ATAD15/KP_OUT1/SMC_SDA/EINT19/CSI1_D19/PH19


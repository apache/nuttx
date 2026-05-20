README
======

ST Nucleo-H753ZI Board Support for Apache NuttX
================================================

  MCU:     STM32H753ZIT6 (Arm Cortex-M7, 480 MHz max, 2 MB Flash, 1 MB RAM)
  Board:   NUCLEO-H753ZI (MB1364)
  Vendor:  STMicroelectronics
  NuttX:   12.11+

References
----------

  UM2407  - Nucleo-H753ZI User Manual (Rev 4)
  DS12110 - STM32H753ZI Datasheet (Rev 9)
  RM0433  - STM32H7x3 Reference Manual

Board Overview
--------------

  The Nucleo-H753ZI is a development board featuring the STM32H753ZIT6
  microcontroller with the following on-board resources:

    - 3 user LEDs: LD1 (Green/PB0), LD2 (Orange/PE1), LD3 (Red/PB14)
    - 1 user push-button: B1 (PC13)
    - ST-LINK/V3 debugger/programmer (provides 8 MHz MCO to MCU HSE)
    - Ethernet connector (RMII interface)
    - USB OTG FS connector
    - Arduino Uno V3 expansion headers (Zio)
    - ST Morpho expansion headers

Menuconfig-Driven Architecture
-------------------------------

  This board support package is designed around a menuconfig-driven
  architecture. Nearly all hardware options are selectable at build
  time through Kconfig without modifying any source files. This
  includes:

  Clock Source Selection
    Via menuconfig: Board Clock Configuration -> HSE clock source
      - 8 MHz (default): Uses ST-LINK MCO, no hardware changes needed
      - 25 MHz: Uses external crystal at X3 (requires HW modification)
    Both configurations produce identical system performance (see
    Clock Configuration section below).

  SPI Bus and Pin Set Selection
    Via menuconfig: SPI Configuration -> SPIx Configuration
      - Enable/disable each SPI bus independently (SPI1-SPI6)
      - Select alternative pin sets per bus (2-4 options per bus)
      - CS pins are NOT configured at the SPI bus level; each
        peripheral registers its own CS pin dynamically at runtime

  I2C Bus, Pin Set and Frequency Selection
    Via menuconfig: I2C Configuration -> I2Cx Configuration
      - Enable/disable each I2C bus independently (I2C1-I2C4)
      - Select alternative pin sets per bus (2-5 options per bus)
      - Configure default bus frequency per bus (100/400/1000 kHz)
      - Kconfig prevents conflicting pin assignments at compile time
        (e.g., I2C1 pinset 1 and I2C4 pinset 4 share PB6/PB7)

  LED Control Mode
    Via menuconfig: LED Control Mode
      - Automatic: Kernel controls LEDs to indicate OS state
      - User: Application controls LEDs via /dev/userleds
      - Disabled: LED subsystem completely disabled

  Button Configuration
    Via menuconfig: Button Configuration
      - Enable/disable button support
      - Configure number of buttons (1-32)
      - Include/exclude built-in button (PC13)
      - Specify additional button GPIO pins via string
        (e.g., "PF15,PG14,PG9,PE0")
      - Compile-time EXTI conflict validation

  Peripheral Module Configuration
    Via menuconfig: SPI MODULES / I2C MODULES
      - ST7796 LCD (SPI): bus, CS pin, DC pin, RESET pin, LED pin,
        color depth, orientation, rotation, framebuffer size,
        SPI frequency
      - MFRC522 RFID (SPI): bus, CS pin, CS polarity, IRQ pin,
        IRQ trigger type
      - SSD1306 OLED (I2C): bus, I2C address, frequency,
        brightness, resolution (128x64 or 128x32), device path

Clock Configuration
-------------------

  Available clock sources on Nucleo-H753ZI:

    HSI:   64 MHz RC factory-trimmed
    CSI:   4 MHz RC oscillator
    LSI:   32 kHz RC
    HSE:   8 MHz from ST-LINK MCO (default) or 25 MHz crystal (X3)
    LSE:   32.768 kHz crystal (X2)
    HSI48: 48 MHz RC (dedicated for USB)

  Both HSE configurations produce identical system clocks:

    SYSCLK  = 400 MHz  (VOS1 maximum; VOS0 not used by NuttX default)
    CPUCLK  = 400 MHz
    HCLK    = 200 MHz  (AHB, SYSCLK/2)
    PCLK1   = 100 MHz  (APB1, HCLK/2)
    PCLK2   = 100 MHz  (APB2, HCLK/2)
    PCLK3   = 100 MHz  (APB3, HCLK/2)
    PCLK4   = 100 MHz  (APB4, HCLK/2)
    Timers  = 200 MHz  (2 x PCLK when APB prescaler != 1)

  PLL Summary (8 MHz HSE, default):

    PLL1: 8/1 * 100 = 800 MHz VCO
      P = 400 MHz  -> SYSCLK
      Q = 200 MHz  -> SPI1/2/3, SDMMC
      R = 200 MHz

    PLL2: 8/1 * 75 = 600 MHz VCO
      P =  75 MHz  -> ADC1/2/3, SPI4/5
      Q =  25 MHz  -> FDCAN1/2
      R = 200 MHz

    PLL3: 8/2 * 100 = 400 MHz VCO
      P = 200 MHz
      Q =  25 MHz
      R =  40 MHz

  PLL Summary (25 MHz HSE, optional):

    PLL1: 25/5 * 160 = 800 MHz VCO  (same outputs as 8 MHz config)
    PLL2: 25/2 * 48  = 600 MHz VCO  (same outputs; FDCAN via HSE direct)
    PLL3: 25/2 * 64  = 800 MHz VCO

  Peripheral kernel clocks:

    I2C1/2/3/4  HSI      16 MHz  (workaround: NuttX I2C driver requires HSI)
    SPI1/2/3    PLL1Q   200 MHz
    SPI4/5      PLL2P    75 MHz
    SPI6        PCLK4   100 MHz
    ADC1/2/3    PLL2P    75 MHz  (within 80 MHz limit per DS12110)
    FDCAN1/2    PLL2Q    25 MHz  (8 MHz config) / HSE direct (25 MHz config)
    SDMMC1/2    PLL1Q   200 MHz
    USB1/2      HSI48    48 MHz

  FLASH wait states: 4 (conservative; safe for VOS1 up to 480 MHz HCLK).
  Minimum required for 200 MHz HCLK at VOS1 is 2 wait states.

  Hardware configuration for 8 MHz HSE (factory default, no changes):
    SB45=ON, SB44=OFF, SB46=OFF, SB3=OFF, SB4=OFF

  Hardware configuration for 25 MHz HSE (requires modification):
    Install 25 MHz crystal at X3.
    SB3=ON, SB4=ON, SB45=OFF, SB44=OFF, SB46=OFF

GPIO Pin Mapping (On-board Peripherals)
----------------------------------------

  Function          Pin    AF    Notes
  ----------------  -----  ----  ----------------------------
  LD1 (Green LED)   PB0    -     Output, active high
  LD2 (Orange LED)  PE1    -     Output, active high
  LD3 (Red LED)     PB14   -     Output, active high
  B1 (User Button)  PC13   EXTI  Input, pull-down
  USART3 TX (VCP)   PD8    AF7   ST-LINK virtual COM port
  USART3 RX (VCP)   PD9    AF7   ST-LINK virtual COM port
  USART6 TX         PG14   AF7
  USART6 RX         PG9    AF7
  I2C1 SCL (opt1)   PB6    AF4   Arduino D10
  I2C1 SDA (opt1)   PB7    AF4   Arduino D9
  I2C1 SCL (opt2)   PB8    AF4   Morpho
  I2C1 SDA (opt2)   PB9    AF4   Morpho
  FDCAN1 RX         PB8    AF9
  FDCAN1 TX         PB9    AF9
  USB OTG FS DM     PA11   AF10
  USB OTG FS DP     PA12   AF10
  USB VBUS          PA9    -
  ETH RMII REF_CLK  PA1    AF11
  ETH RMII CRS_DV   PA7    AF11
  ETH RMII TX_EN    PG11   AF11
  ETH RMII TXD0     PG13   AF11
  ETH RMII TXD1     PB13   AF11
  ETH RMII RXD0     PC4    AF11
  ETH RMII RXD1     PC5    AF11
  ETH MDIO          PA2    AF11
  ETH MDC           PC1    AF11
  HSE IN            PH0    -     ST-LINK MCO (8 MHz default)
  HSE OUT           PH1    -     Reserved (X3 crystal)
  SWCLK             PA14   AF0
  SWDIO             PA13   AF0

  Note: Ethernet requires solder bridge changes when using Zio/Morpho
  headers. See UM2407 Rev 4, page 28 for solder bridge table.

SPI Interfaces
--------------

  Six SPI buses available (SPI1-SPI6). Each bus supports multiple
  pin sets selectable via menuconfig. CS pins are registered
  dynamically per peripheral.

  SPI bus -> kernel clock -> max SCK:
    SPI1/2/3: PLL1Q (200 MHz) -> 100 MHz max SCK
    SPI4/5:   PLL2P  (75 MHz) ->  37.5 MHz max SCK
    SPI6:     PCLK4 (100 MHz) ->  50 MHz max SCK

  SPI1 pin sets:
    Set 1 (Arduino): PA5(SCK), PA6(MISO), PA7(MOSI)
    Set 2:           PB3(SCK), PB4(MISO), PB5(MOSI)
    Set 3:           PG11(SCK), PG9(MISO), PD7(MOSI)

  SPI2 pin sets:
    Set 1: PB13(SCK), PB14(MISO), PB15(MOSI)
    Set 2: PA12(SCK), PC2(MISO), PC1(MOSI)
    Set 3: PD3(SCK), PC2(MISO), PC3(MOSI)

  SPI3 pin sets:
    Set 1: PC10(SCK), PC11(MISO), PC12(MOSI)
    Set 2: PB3(SCK), PB4(MISO), PB5(MOSI)

  SPI4 pin sets:
    Set 1: PE12(SCK), PE13(MISO), PE14(MOSI)
    Set 2: PE2(SCK), PE5(MISO), PE6(MOSI)

  SPI5 pin sets:
    Set 1: PF7(SCK), PF8(MISO), PF9(MOSI)
    Set 2: PK0(SCK), PJ11(MISO), PJ10(MOSI)

  SPI6 pin sets:
    Set 1: PG13(SCK), PG12(MISO), PG14(MOSI)
    Set 2: PA5(SCK), PA6(MISO), PA7(MOSI)

I2C Interfaces
--------------

  Four I2C buses available (I2C1-I2C4). All use HSI (16 MHz) as
  kernel clock (NuttX I2C driver requirement). Bus frequency
  configurable per bus via menuconfig (100/400/1000 kHz).

  I2C1 pin sets:
    Set 1: PB6(SCL), PB7(SDA) - AF4 (Arduino D10/D9)
    Set 2: PB8(SCL), PB9(SDA) - AF4 (Morpho)

  I2C2 pin sets:
    Set 1: PB10(SCL), PB11(SDA) - AF4
    Set 2: PF1(SCL), PF0(SDA) - AF4
    Set 3: PH4(SCL), PH5(SDA) - AF4

  I2C3 pin sets:
    Set 1: PA8(SCL), PC9(SDA) - AF4
    Set 2: PH7(SCL), PH8(SDA) - AF4

  I2C4 pin sets:
    Set 1: PD12(SCL), PD13(SDA) - AF4
    Set 2: PF14(SCL), PF15(SDA) - AF4
    Set 3: PH11(SCL), PH12(SDA) - AF4
    Set 4: PB6(SCL), PB7(SDA) - AF6 (conflicts with I2C1 Set 1)
    Set 5: PB8(SCL), PB9(SDA) - AF6 (conflicts with I2C1 Set 2)

  Kconfig enforces conflicting pin exclusions at build time.

FDCAN (CAN FD)
--------------

  FDCAN1: RX=PB8, TX=PB9
  Kernel clock: 25 MHz (PLL2Q for 8 MHz HSE; HSE direct for 25 MHz HSE)
  Supported bitrates: 125 kbps, 250 kbps, 500 kbps, 1 Mbps (and FD rates)

USB
---

  USB OTG FS: PA11(DM), PA12(DP), PA9(VBUS), PG6(PWRON), PG7(OVERCURRENT)
  Kernel clock: HSI48 (48 MHz, dedicated USB oscillator)
  Supports: USB Device, USB Host, USB Monitor

SDMMC
-----

  Kernel clock: PLL1Q (200 MHz)
  Init frequency: 200 MHz / (2*240) = ~416 kHz
  Transfer frequency: 200 MHz / (2*2) = 50 MHz (25 MB/s)

ADC
---

  ADC1/2/3 kernel clock: PLL2P (75 MHz, within 80 MHz HW limit)
  Available channels exposed: INP3, INP4, INP5, INP7, INP8, INP10,
  INP11, INP12, INP13, INP14, INP15, INP18, INP19

Supported Peripheral Modules
------------------------------

  The following modules are supported with full Kconfig configuration:

  ST7796 (SPI LCD, 480x320 IPS)
    Configurable: SPI bus, CS/DC/RESET/LED pins, CS polarity,
    color depth (RGB444/RGB565/RGB666), orientation (portrait/landscape
    and reverse variants), 180-degree rotation, SPI frequency,
    framebuffer size. Framebuffer interface (/dev/fb0) compatible
    with LVGL and other graphics libraries.

  SSD1306 (I2C OLED, 128x64 or 128x32)
    Configurable: I2C bus, I2C address (0x3C/0x3D/custom), bus
    frequency, brightness (0-100%), resolution, device path,
    device number.

  MFRC522 (SPI RFID, 13.56 MHz)
    Configurable: SPI bus, device ID, CS pin, CS polarity,
    optional IRQ pin, IRQ trigger (falling/rising/both edges).
    Device node: /dev/rfid0

  NRF24L01 (SPI 2.4 GHz transceiver)
    CS=PA4, CE=PF12, IRQ=PD15

  MMC/SD Card (SPI)
    CS=PD15, Card-detect=PF12 (EXTI)

  LSM6DSL (I2C 6-axis IMU)
    INT1=PB4, INT2=PB5

  LSM303AGR (I2C magnetometer/accelerometer)
  LSM9DS1 (I2C 9-axis IMU)
  LPS22HB (I2C pressure sensor, INT1=PB10)
  PCA9635 (I2C LED controller, addr=0x40, bus=I2C1)

Additional Features
-------------------

  PROGMEM MTD: Internal flash exposed as MTD character device
  RTC: Hardware RTC driver (/dev/rtc0)
  PWM: TIM1 configured (CH1-CH4 + complementary outputs on PE8-PE14)
  ROMFS: Optional auto-mount of built-in ROMFS image
  GPIO driver: /dev/gpioN support (BOARD_NGPIOIN=1, NGPIOOUT=3, NGPIOINT=1)
  USB MSC: Mass storage class support

LED Behavior (Automatic Mode)
-------------------------------

  LED_STARTED       Red=OFF, Green=OFF, Orange=OFF
  LED_HEAPALLOCATE  Red=OFF, Green=OFF, Orange=ON
  LED_IRQSENABLED   Red=OFF, Green=ON,  Orange=OFF
  LED_STACKCREATED  Red=OFF, Green=ON,  Orange=ON
  LED_INIRQ         Red=N/C, Green=N/C, Orange=GLOW
  LED_SIGNAL        Red=N/C, Green=GLOW,Orange=N/C
  LED_ASSERTION     Red=GLOW,Green=N/C, Orange=GLOW
  LED_PANIC         Red=BLINK,Green=OFF,Orange=N/C
  LED_IDLE          Red=ON,  Green=OFF, Orange=OFF

Configurations
--------------

  nsh
    Basic NuttShell (NSH) via USART3/VCP at 115200 baud.
    User-controlled LEDs (/dev/userleds) via examples/leds.
    Generic GPIO driver (/dev/gpioN) via examples/gpio.
    D-cache and I-cache enabled. DTCM enabled.
    CONFIG_ARCH_LEDS disabled (user LED mode).

  button_driver
    NuttShell via USART3/VCP at 115200 baud.
    Demonstrates the menuconfig-driven button system with 11 buttons:
      Button 0: built-in PC13
      Buttons 1-10: PB1, PD0, PG4, PD5, PE6, PE3, PD12, PF9, PD15, PE11
    Each button assigned to a unique EXTI line (no conflicts).
    Automatic LEDs enabled (CONFIG_ARCH_LEDS_CPU_ACTIVITY).
    User LEDs also available (/dev/userleds).
    Button device at /dev/buttons via examples/buttons.
    IRQ-driven button events (CONFIG_ARCH_IRQBUTTONS).

  socketcan
    NuttShell via USART3/VCP at 115200 baud.
    FDCAN1 configured as SocketCAN network interface.
    Arbitration bitrate: 500 kbps.
    Data bitrate: 500 kbps (CAN FD capable).
    Includes canutils: candump, cansend.
    Network stack enabled (CONFIG_NET_CAN).
    CAN error frames, socket options, write buffers enabled.
    HPWORK thread for FDCAN interrupt handling.
    procfs mounted for runtime inspection.

Source Organization
-------------------

  src/
    stm32_boot.c              - Boot and early initialization
    stm32_bringup.c           - Driver registration (bringup)
    stm32_appinitialize.c     - Application-level initialization
    stm32_boot_image.c        - MCUboot image support
    nucleo-h753zi.h           - Board-internal definitions and prototypes

    drivers/driver_bus/
      stm32_spi.c             - SPI bus initialization (SPI1-SPI6)
      stm32_i2c.c             - I2C bus initialization (I2C1-I2C4)

    drivers/driver_generic/
      stm32_adc.c             - ADC setup
      stm32_buttons.c         - Button driver (up to 32 buttons)
      stm32_gpio.c            - Generic GPIO driver
      stm32_pwm.c             - PWM output setup
      stm32_userleds.c        - User LED driver

    drivers/driver_middleware/
      stm32_autoleds.c        - Automatic LED OS state indication
      stm32_composite.c       - USB composite device
      stm32_progmem.c         - Internal flash MTD
      stm32_reset.c           - Board reset support
      stm32_romfs_initialize.c- ROMFS auto-mount
      stm32_uid.c             - Unique device ID
      stm32_usb.c             - USB OTG FS initialization
      stm32_usbmsc.c          - USB mass storage class

    drivers/driver_modules/
      stm32_lsm303agr.c       - LSM303AGR IMU driver
      stm32_lsm6dsl.c         - LSM6DSL IMU driver
      stm32_lsm9ds1.c         - LSM9DS1 IMU driver
      stm32_mfrc522.c         - MFRC522 RFID driver
      stm32_mmcsd.c           - MMC/SD card (SPI) driver
      stm32_nrf24l01.c        - NRF24L01 wireless driver
      stm32_pca9635.c         - PCA9635 LED controller driver
      stm32_ssd1306.c         - SSD1306 OLED driver
      stm32_st7796.c          - ST7796 LCD + framebuffer driver

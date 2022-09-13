/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32f4discovery.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_STM32F4DISCOVERY_SRC_STM32F4DISCOVERY_H
#define __BOARDS_ARM_STM32_STM32F4DISCOVERY_SRC_STM32F4DISCOVERY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif

#define PCA9635_I2CBUS  1
#define PCA9635_I2CADDR 0x40

/* Assume that we have everything */

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1
#define HAVE_SDIO       1
#define HAVE_CS43L22    1
#define HAVE_RTC_DRIVER 1
#define HAVE_NETMONITOR 1
#define HAVE_HCIUART    1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#endif

/* Can't support USB device if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_STM32_SDIO)
#  undef HAVE_SDIO
#endif

#undef  SDIO_MINOR     /* Any minor number, default 0 */
#define SDIO_SLOTNO 0  /* Only one slot */

#ifdef HAVE_SDIO
#  if !defined(CONFIG_NSH_MMCSDSLOTNO)
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  elif CONFIG_NSH_MMCSDSLOTNO != 0
#    warning "Only one MMC/SD slot, slot 0"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  endif

#  if defined(CONFIG_NSH_MMCSDMINOR)
#    define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define SDIO_MINOR 0
#  endif
#endif

/* The CS43L22 depends on the CS43L22 driver, I2C1, and I2S3 support */

#if !defined(CONFIG_AUDIO_CS43L22) || !defined(CONFIG_STM32_I2C1) || \
    !defined(CONFIG_STM32_I2S3)
#  undef HAVE_CS43L22
#endif

#ifdef HAVE_CS43L22
  /* The CS43L22 communicates on I2C1, I2C address 0x1a for control
   * operations
   */

#  define CS43L22_I2C_BUS      1
#  define CS43L22_I2C_ADDRESS  (0x94 >> 1)

  /* The CS43L22 transfers data on I2S3 */

#  define CS43L22_I2S_BUS      3
#endif

/* Check if we can support the RTC driver */

#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* NSH Network monitor  */

#if !defined(CONFIG_NET) || !defined(CONFIG_STM32_EMACMAC)
#  undef HAVE_NETMONITOR
#endif

#if !defined(CONFIG_NSH_NETINIT_THREAD) || !defined(CONFIG_ARCH_PHY_INTERRUPT) || \
    !defined(CONFIG_NETDEV_PHY_IOCTL) || !defined(CONFIG_NET_UDP)
#  undef HAVE_NETMONITOR
#endif

/* The NSH Network Monitor cannot be used with the STM32F4DIS-BB base board.
 * That is because the LAN8720 is configured in REF_CLK OUT mode.  In that
 * mode, the PHY interrupt is not supported.  The NINT pin serves instead as
 * REFLCK0.
 */

#ifdef CONFIG_STM32F4DISBB
#  undef HAVE_NETMONITOR
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Check if we have the prerequisites for an HCI UART */

#if !defined(CONFIG_STM32_HCIUART) || !defined(CONFIG_BLUETOOTH_UART)
#  undef HAVE_HCIUART
#elif defined(CONFIG_STM32_USART1_HCIUART)
#  define HCIUART_SERDEV HCIUART1
#elif defined(CONFIG_STM32_USART2_HCIUART)
#  define HCIUART_SERDEV HCIUART2
#elif defined(CONFIG_STM32_USART3_HCIUART)
#  define HCIUART_SERDEV HCIUART3
#elif defined(CONFIG_STM32_USART6_HCIUART)
#  define HCIUART_SERDEV HCIUART6
#elif defined(CONFIG_STM32_UART7_HCIUART)
#  define HCIUART_SERDEV HCIUART7
#elif defined(CONFIG_STM32_UART8_HCIUART)
#  define HCIUART_SERDEV HCIUART8
#else
#  error No HCI UART specifified
#endif

/* STM32F4 Discovery GPIOs **************************************************/

/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
#define GPIO_LED4       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)

#define GPIO_CS43L22_RESET  (GPIO_OUTPUT|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN4)

/* Digital Joystick 5-WAY */

#define GPIO_JOY_UP       (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN2)
#define GPIO_JOY_CENTER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN3)
#define GPIO_JOY_LEFT     (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN4)
#define GPIO_JOY_DOWN     (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN5)
#define GPIO_JOY_RIGHT    (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN6)

/* LoRa SX127x */

#define GPIO_SX127X_DIO0    (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN0)

#define GPIO_SX127X_RESET   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|\
                             GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN4)

/* PWM
 *
 * The STM32F4 Discovery has no real on-board PWM devices, but the board can
 * be configured to output a pulse train using TIM4 CH2 on PD13.
 */

#define STM32F4DISCOVERY_PWMTIMER   4
#define STM32F4DISCOVERY_PWMCHANNEL 2

/* Capture
 *
 * The STM32F4 Discovery has no real on-board pwm capture devices, but the
 * board can be configured to capture pwm using TIM3 CH2 PB5.
 */

#define STM32F4DISCOVERY_CAPTURETIMER   3
#define STM32F4DISCOVERY_CAPTURECHANNEL 2

/* SPI chip selects */

#define GPIO_CS_MEMS      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)

#define GPIO_MAX31855_CS  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN8)

#define GPIO_MAX6675_CS   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN8)

#define GPIO_SX127X_CS    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN8)

#define GPIO_MAX7219_CS   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN3)

#define GPIO_GS2200M_CS   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)

#define GPIO_ENC28J60_CS    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                             GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

#define GPIO_ENC28J60_RESET (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                             GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN1)

#define GPIO_ENC28J60_INTR  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|\
                             GPIO_OPENDRAIN|GPIO_PORTE|GPIO_PIN4)

/* Use same pins as ENC28J60 to W5500 */

#define GPIO_W5500_CS      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                             GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

#define GPIO_W5500_RESET   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                             GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN1)

#define GPIO_W5500_INTR     (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|\
                             GPIO_OPENDRAIN|GPIO_PORTE|GPIO_PIN4)

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PC0  OTG_FS_PowerSwitchOn
 * PD5  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN0)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|\
                           GPIO_SPEED_100MHz|GPIO_PUSHPULL|\
                           GPIO_PORTD|GPIO_PIN5)
#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN5)
#endif

/* UG-2864AMBAG01 or UG-2864HSWEG01 OLED Display (SPI 4-wire):
 *
 * --------------------------+----------------------------------------------
 * Connector CON10 J1:      | STM32F4Discovery
 * --------------+-----------+----------------------------------------------
 * CON10 J1:     | CON20 J2: | P1/P2:
 * --------------+-----------+----------------------------------------------
 * 1  3v3        | 3,4 3v3   | P2 3V
 * 3  /RESET     | 8 /RESET  | P2 PB6 (Arbitrary selection)
 * 5  /CS        | 7 /CS     | P2 PB7 (Arbitrary selection)
 * 7  A0|D/C     | 9 A0|D/C  | P2 PB8 (Arbitrary selection)
 * 9  LED+ (N/C) | -----     | -----
 * 2  5V Vcc     | 1,2 Vcc   | P2 5V
 * 4  DI         | 18 D1/SI  | P1 PA7 (GPIO_SPI1_MOSI == GPIO_SPI1_MOSI_1(1))
 * 6  SCLK       | 19 D0/SCL | P1 PA5 (GPIO_SPI1_SCK == GPIO_SPI1_SCK_1(1))
 * 8  LED- (N/C) | -----     | ------
 * 10 GND        | 20 GND    | P2 GND
 * --------------+-----------+----------------------------------------------
 * (1) Required because of on-board MEMS
 * -------------------------------------------------------------------------
 */

#if defined(CONFIG_LCD_UG2864AMBAG01) || defined(CONFIG_LCD_UG2864HSWEG01) || \
    defined(CONFIG_LCD_SSD1351)
#  define GPIO_OLED_RESET (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_OLED_CS    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN7)
#  define GPIO_OLED_A0    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_OLED_DC    GPIO_OLED_A0
#endif

/* Display JLX12864G */

#define STM32_LCD_RST     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN6)

#define STM32_LCD_CS      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN7)

#define STM32_LCD_RS      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN8)

/* STM32F4DIS-BB MicroSD
 *
 * ---------- ------------- ------------------------------
 * PIO        SIGNAL        Comments
 * ---------- ------------- ------------------------------
 * PB15       NCD           Pulled up externally
 * PC9        DAT1          Configured by driver
 * PC8        DAT0          "        " "" "    "
 * PC12       CLK           "        " "" "    "
 * PD2        CMD           "        " "" "    "
 * PC11       CD/DAT3       "        " "" "    "
 * PC10       DAT2          "        " "" "    "
 * ---------- ------------- ------------------------------
 */

#if defined(CONFIG_STM32F4DISBB) && defined(CONFIG_STM32_SDIO)
#  define GPIO_SDIO_NCD   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|\
                           GPIO_PORTB|GPIO_PIN15)
#endif

/* STM32F4DIS-BB LAN8720
 *
 * ---------- ------------- ------------------------------
 * PIO        SIGNAL        Comments
 * ---------- ------------- ------------------------------
 * PB11       TXEN           Configured by driver
 * PB12       TXD0          "        " "" "    "
 * PB13       TXD1          "        " "" "    "
 * PC4        RXD0/MODE0    "        " "" "    "
 * PC5        RXD1/MODE1    "        " "" "    "
 * PA7        CRS_DIV/MODE2 "        " "" "    "
 * PA2        MDIO          "        " "" "    "
 * PC1        MDC           "        " "" "    "
 * PA1        NINT/REFCLK0  "        " "" "    "
 * PE2        DAT2          "        " "" "    "
 * ---------- ------------- ------------------------------
 */

#if defined(CONFIG_STM32F4DISBB) && defined(CONFIG_STM32_ETHMAC)
#  define GPIO_EMAC_NINT  (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|\
                           GPIO_PORTA|GPIO_PIN1)
#  define GPIO_EMAC_NRST  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f4discovery
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_i2sdev_initialize
 *
 * Description:
 *   Called to configure I2S chip select GPIO pins for the stm32f4discovery
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_i2sdev_initialize(void);

/****************************************************************************
 * Name: stm32_bh1750initialize
 *
 * Description:
 *   Called to configure an I2C and to register BH1750FVI for the
 *   stm32f4discovery board.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BH1750FVI
int stm32_bh1750initialize(const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_lpwaninitialize
 *
 * Description:
 *   Initialize SX127X LPWAN interaface.
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X
int stm32_lpwaninitialize(void);
#endif

/****************************************************************************
 * Name: stm32_mmcsdinitialize
 *
 * Description:
 *   Sets up MMC/SD interface.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_SPI
int stm32_mmcsd_initialize(int port, int minor);
#endif

/****************************************************************************
 * Name: nunchuck_initialize
 *
 * Description:
 *   Initialize and register the button joystick driver
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_NUNCHUCK
int nunchuck_initialize(char *devname);
#endif

/****************************************************************************
 * Name: stm32_max7219init
 *
 * Description:
 *   Initialize and register the max7219 numeric display controller
 *
 ****************************************************************************/

#ifdef CONFIG_LEDS_MAX7219
int stm32_max7219init(const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_ds1307_init
 *
 * Description:
 *   Initialize and register the DS1307 RTC
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DS1307
int stm32_ds1307_init(void);
#endif

/****************************************************************************
 * Name: stm32_st7032init
 *
 * Description:
 *   Initialize and register the Sitronix ST7032i
 *
 ****************************************************************************/

int stm32_st7032init(const char *devpath);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the STM32F4Discovery board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_capture_setup
 *
 * Description:
 *  Initialize pwm capture support
 *
 ****************************************************************************/

#ifdef CONFIG_CAPTURE
int stm32_capture_setup(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_CAN_CHARDRIVER
int stm32_can_setup(void);
#endif

/****************************************************************************
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for external memory usage
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemgpios(const uint32_t *gpios, int ngpios);
#endif

/****************************************************************************
 * Name: stm32_extmemaddr
 *
 * Description:
 *   Initialize address line GPIOs for external memory access
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemaddr(int naddrs);
#endif

/****************************************************************************
 * Name: stm32_extmemdata
 *
 * Description:
 *   Initialize data line GPIOs for external memory access
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemdata(int ndata);
#endif

/****************************************************************************
 * Name: stm32_led_pminitialize
 *
 * Description:
 *   Enable logic to use the LEDs on the STM32F4Discovery to support power
 *   management testing
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void stm32_led_pminitialize(void);
#endif

/****************************************************************************
 * Name: stm32_pm_buttons
 *
 * Description:
 *   Configure the user button of the STM32f4discovery board as EXTI,
 *   so it is able to wakeup the MCU from the PM_STANDBY mode
 *
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_ARCH_IDLE_CUSTOM) && \
    defined(CONFIG_PM_BUTTONS)
void stm32_pm_buttons(void);
#endif

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_STM32_SDIO)
int stm32_sdio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ****************************************************************************/

#ifdef HAVE_NETMONITOR
void weak_function stm32_netinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_zerocross_initialize
 *
 * Description:
 *   Initialize and register the zero cross driver
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ZEROCROSS
int stm32_zerocross_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_max31855initialize
 *
 * Description:
 *   Initialize and register the MAX31855 Temperature Sensor driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   bus     - Bus number (for hardware that has multiple SPI interfaces)
 *   devid   - ID associated to the device. E.g., 0, 1, 2, etc.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_MAX31855
int stm32_max31855initialize(const char *devpath, int bus,
                             uint16_t devid);
#endif

/****************************************************************************
 * Name: stm32_mlx90614init
 *
 * Description:
 *   Called to configure an I2C and to register MLX90614 for the
 *   stm32f103-minimum board.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_MLX90614
int stm32_mlx90614init(const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_max6675initialize
 *
 * Description:
 *   Initialize and register the max6675 driver
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_MAX6675
int stm32_max6675initialize(const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_cs43l22_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the CS43L22 device.  This function will register the
 *   driver as /dev/cs43l22[x] where x is determined by the minor device
 *   number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef HAVE_CS43L22
int stm32_cs43l22_initialize(int minor);
#endif /* HAVE_CS43L22 */

/****************************************************************************
 * Name: stm32_pca9635_initialize
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   LED PWM chip.  This function will register the driver as /dev/leddrv0.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PCA9635PW
int stm32_pca9635_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_rgbled_setup
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   RGB LED driver.  This function will register the driver as /dev/rgbled0.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RGBLED
int stm32_rgbled_setup(void);
#endif

/****************************************************************************
 * Name: stm32_timer_driver_setup
 *
 * Description:
 *   Configure the timer driver.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *             form /dev/timer0
 *   timer   - The timer's number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int stm32_timer_driver_setup(const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: xen1210_archinitialize
 *
 * Description:
 *   Each board that supports an xen1210 device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   configure the accelerometer device.  This function will register the
 *   driver as /dev/accelN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_XEN1210
int xen1210_archinitialize(int minor);
#endif

/****************************************************************************
 * Name: hciuart_dev_initialize
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   Bluetooth HCI UART driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef HAVE_HCIUART
int hciuart_dev_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_gs2200m_initialize
 *
 * Description:
 *   Configure the gs2200m driver.
 *
 * Input Parameters:
 *   devpath - The full path to the device.
 *   bus     - The SPI bus number
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_WL_GS2200M
int stm32_gs2200m_initialize(const char *devpath, int bus);
#endif

/****************************************************************************
 * Name: stm32_djoy_initialize
 *
 * Description:
 *   Initialize and register the discrete joystick driver
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_DJOYSTICK
int stm32_djoy_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_STM32F4DISCOVERY_SRC_STM32F4DISCOVERY_H */

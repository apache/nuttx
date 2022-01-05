/****************************************************************************
 * boards/arm/stm32/shenzhou/src/shenzhou.h
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

#ifndef __BOARDS_ARM_STM32_SHENZHOUL_SRC_SHENZHOU_H
#define __BOARDS_ARM_STM32_SHENZHOUL_SRC_SHENZHOU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

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

/* Shenzhou GPIO Configuration **********************************************/

/* STM3240G-EVAL GPIOs ******************************************************/

/* Ethernet
 *
 * -- ---- -------------- ---------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ---------------------------------------------------
 * 24 PA1  MII_RX_CLK     Ethernet PHY   NOTE:  Despite the MII labeling of
 *         RMII_REF_CLK   Ethernet PHY   these signals, the DM916AEP is
 * 25 PA2  MII_MDIO       Ethernet PHY   actually configured to work in RMII
 * 48 PB11 MII_TX_EN      Ethernet PHY    mode.
 * 51 PB12 MII_TXD0       Ethernet PHY
 * 52 PB13 MII_TXD1       Ethernet PHY
 * 16 PC1  MII_MDC        Ethernet PHY
 * 34 PC5  MII_INT        Ethernet PHY
 * 55 PD8  MII_RX_DV      Ethernet PHY.  Requires CONFIG_STM32_ETH_REMAP
 * 55 PD8  RMII_CRSDV     Ethernet PHY.  Requires CONFIG_STM32_ETH_REMAP
 * 56 PD9  MII_RXD0       Ethernet PHY.  Requires CONFIG_STM32_ETH_REMAP
 * 57 PD10 MII_RXD1       Ethernet PHY.  Requires CONFIG_STM32_ETH_REMAP
 *
 * The board desdign can support a 50MHz external clock to drive the PHY
 * (U9).  However, on my board, U9 is not present.
 *
 * 67 PA8  MCO            DM9161AEP
 */

#ifdef CONFIG_STM32_ETHMAC
#  define GPIO_MII_INT   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN5)
#endif

/* Wireless
 *
 * -- ---- -------------- ---------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ---------------------------------------------------
 * 26 PA3  315M_VT
 * 17 PC2  WIRELESS_INT
 * 18 PC3  WIRELESS_CE    To the NRF24L01 2.4G wireless module
 * 59 PD12 WIRELESS_CS    To the NRF24L01 2.4G wireless module
 */

#define GPIO_WIRELESS_CS  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN12)

/* Buttons
 *
 * -- ---- -------------- ---------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ---------------------------------------------------
 * 23 PA0  WAKEUP         Connected to KEY4.  Active low: Closing KEY4 pulls
 *                                                         WAKEUP to ground.
 * 47 PB10 USERKEY        Connected to KEY2
 * 33 PC4  USERKEY2       Connected to KEY1
 * 7  PC13 TAMPER         Connected to KEY3
 */

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON     BUTTON_KEY1
#define MAX_IRQBUTTON     BUTTON_KEY4
#define NUM_IRQBUTTONS    (BUTTON_KEY4 - BUTTON_KEY1 + 1)

#define GPIO_BTN_WAKEUP   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)
#define GPIO_BTN_USERKEY  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN10)
#define GPIO_BTN_USERKEY2 (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN4)
#define GPIO_BTN_TAMPER   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)

/* LEDs
 *
 * -- ---- -------------- ---------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ---------------------------------------------------
 * 83 PD2  LED1           Active low: Pulled high
 * 84 PD3  LED2           Active low: Pulled high
 * 85 PD4  LED3           Active low: Pulled high
 * 88 PD7  LED4           Active low: Pulled high
 */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN2)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN3)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN4)
#define GPIO_LED4       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)

/* TFT LCD
 *
 * -- ---- -------------- ---------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ---------------------------------------------------
 * 37 PB2  DATA_LE        To TFT LCD. (CN13, ping 28)
 * 96 PB9  F_CS           To both the TFT LCD (CN13, pin 30) and to the
 *                                         W25X16 SPI FLASH
 * 34 PC5  TP_INT         JP6.  To TFT LCD (CN13) module (CN13, pin 26)
 * 65 PC8  LCD_CS         Active low: Pulled high (CN13, pin 19)
 * 66 PC9  TP_CS          Active low: Pulled high (CN13, pin 31)
 * 78 PC10 SPI3_SCK       To TFT LCD (CN13, pin 29)
 * 79 PC11 SPI3_MISO      To TFT LCD (CN13, pin 25)
 * 80 PC12 SPI3_MOSI      To TFT LCD (CN13, pin 27)
 * 58 PD11 SD_CS          Active low: Pulled high
 *                                       (See also TFT LCD CN13, pin 32)
 * 60 PD13 LCD_RS         To TFT LCD (CN13, pin 20)
 * 61 PD14 LCD_WR         To TFT LCD (CN13, pin 21).
 *                                       Schematic is wrong LCD_WR is PB14.
 * 62 PD15 LCD_RD         To TFT LCD (CN13, pin 22)
 * 97 PE0  DB00           To TFT LCD (CN13, pin 3)
 * 98 PE1  DB01           To TFT LCD (CN13, pin 4)
 * 1  PE2  DB02           To TFT LCD (CN13, pin 5)
 * 2  PE3  DB03           To TFT LCD (CN13, pin 6)
 * 3  PE4  DB04           To TFT LCD (CN13, pin 7)
 * 4  PE5  DB05           To TFT LCD (CN13, pin 8)
 * 5  PE6  DB06           To TFT LCD (CN13, pin 9)
 * 38 PE7  DB07           To TFT LCD (CN13, pin 10)
 * 39 PE8  DB08           To TFT LCD (CN13, pin 11)
 * 40 PE9  DB09           To TFT LCD (CN13, pin 12)
 * 41 PE10 DB10           To TFT LCD (CN13, pin 13)
 * 42 PE11 DB11           To TFT LCD (CN13, pin 16)
 * 43 PE12 DB12           To TFT LCD (CN13, pin 15)
 * 44 PE13 DB13           To TFT LCD (CN13, pin 16)
 * 45 PE14 DB14           To TFT LCD (CN13, pin 17)
 * 46 PE15 DB15           To TFT LCD (CN13, pin 18)
 *
 * NOTE:  The backlight signl NC_BL (CN13, pin 24) is pulled high and not
 * under software control
 *
 * On LCD module:
 * -- -------------- --------------------------------------------------------
 * PN SIGNAL         NOTES
 * -- -------------- --------------------------------------------------------
 * 3  DB01           To LCD DB1
 * 4  DB00           To LCD DB0
 * 5  DB03           To LCD DB3
 * 6  DB02           To LCD DB2
 * 7  DB05           To LCD DB5
 * 8  DB04           To LCD DB4
 * 9  DB07           To LCD DB7
 * 10 DB06           To LCD DB6
 * 11 DB09           To LCD DB9
 * 12 DB08           To LCD DB8
 * 13 DB11           To LCD DB11
 * 14 DB10           To LCD DB10
 * 15 DB13           To LCD DB13
 * 16 DB12           To LCD DB12
 * 17 DB15           To LCD DB15
 * 18 DB14           To LCD DB14
 * 19 RS             To LCD RS
 * 20 /LCD_CS        To LCD CS
 * 21 /RD            To LCD RD
 * 22 /WR            To LCD WR
 * 23 BL_EN          (Not referenced)
 * 24 /RESET
 * 25 /INT           To Touch IC /INT
 * 26 MISO           To Touch IC DOUT; To AT45DB161B SO; To SD card DAT0
 * 27 LE             To 74HC573 that controls LCD 8-bit/16-bit mode
 * 28 MOSI           To Touch IC DIN; To AT45DB161B SI; To SD card CMD
 * 29 /DF_CS         To AT45DB161B Data Flash /CS
 * 30 SCLK           To Touch IC DCLK; To AT45DB161B SCK; To SD card CLK
 * 31 /SD_CS         To SD card /CS
 * 31 /TP_CS         To Touch IC CS
 */

/* TFT LCD GPIOs */

#define GPIO_LCD_D0OUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN0)
#define GPIO_LCD_D1OUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN1)
#define GPIO_LCD_D2OUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2)
#define GPIO_LCD_D3OUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_LCD_D4OUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_LCD_D5OUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)
#define GPIO_LCD_D6OUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN6)
#define GPIO_LCD_D7OUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN7)
#define GPIO_LCD_D8OUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN8)
#define GPIO_LCD_D9OUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN9)
#define GPIO_LCD_D10OUT (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN10)
#define GPIO_LCD_D11OUT (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN11)
#define GPIO_LCD_D12OUT (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)
#define GPIO_LCD_D13OUT (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN13)
#define GPIO_LCD_D14OUT (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN14)
#define GPIO_LCD_D15OUT (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN15)

#define GPIO_LCD_D0IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN0)
#define GPIO_LCD_D1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN1)
#define GPIO_LCD_D2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN2)
#define GPIO_LCD_D3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN3)
#define GPIO_LCD_D4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN4)
#define GPIO_LCD_D5IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN5)
#define GPIO_LCD_D6IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN6)
#define GPIO_LCD_D7IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN7)
#define GPIO_LCD_D8IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN8)
#define GPIO_LCD_D9IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN9)
#define GPIO_LCD_D10IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN10)
#define GPIO_LCD_D11IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN11)
#define GPIO_LCD_D12IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN12)
#define GPIO_LCD_D13IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN13)
#define GPIO_LCD_D14IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN14)
#define GPIO_LCD_D15IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTE|GPIO_PIN15)

#define GPIO_LCD_RS     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN13)
#define GPIO_LCD_CS     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN8)
#define GPIO_LCD_RD     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN15)
#define GPIO_LCD_WR     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN14)
#define GPIO_LCD_LE     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN2)

/* Bit band addresses */

#define STM32_GPIOB_OFFSET (STM32_GPIOB_BASE - STM32_PERIPH_BASE)
#define STM32_GPIOC_OFFSET (STM32_GPIOC_BASE - STM32_PERIPH_BASE)
#define STM32_GPIOD_OFFSET (STM32_GPIOD_BASE - STM32_PERIPH_BASE)

#define LCD_BIT_CLEAR(offs,pin) \
  (STM32_PERIPHBB_BASE + ((offs + STM32_GPIO_BRR_OFFSET) << 5) + ((pin) << 2))
#define LCD_BIT_SET(offs,pin) \
  (STM32_PERIPHBB_BASE + ((offs + STM32_GPIO_BSRR_OFFSET) << 5) + ((pin) << 2))
#define LCD_BIT_READ(offs,pin) \
  (STM32_PERIPHBB_BASE + ((offs + STM32_GPIO_ODR_OFFSET) << 5) + ((pin) << 2))

#define LCD_RS_CLEAR    LCD_BIT_CLEAR(STM32_GPIOD_OFFSET, 13) /* GPIO_PORTD|GPIO_PIN13 */
#define LCD_RS_SET      LCD_BIT_SET(STM32_GPIOD_OFFSET, 13)
#define LCD_RS_READ     LCD_BIT_READ(STM32_GPIOD_OFFSET, 13)
#define LCD_CS_CLEAR    LCD_BIT_CLEAR(STM32_GPIOC_OFFSET, 8)  /* GPIO_PORTC|GPIO_PIN8 */
#define LCD_CS_SET      LCD_BIT_SET(STM32_GPIOC_OFFSET, 8)
#define LCD_CS_READ     LCD_BIT_READ(STM32_GPIOC_OFFSET, 8)
#define LCD_RD_CLEAR    LCD_BIT_CLEAR(STM32_GPIOD_OFFSET, 15) /* GPIO_PORTD|GPIO_PIN15 */
#define LCD_RD_SET      LCD_BIT_SET(STM32_GPIOD_OFFSET, 15)
#define LCD_RD_READ     LCD_BIT_READ(STM32_GPIOD_OFFSET, 15)
#define LCD_WR_CLEAR    LCD_BIT_CLEAR(STM32_GPIOB_OFFSET, 14) /* GPIO_PORTB|GPIO_PIN14 */
#define LCD_WR_SET      LCD_BIT_SET(STM32_GPIOB_OFFSET, 14)
#define LCD_WR_READ     LCD_BIT_READ(STM32_GPIOB_OFFSET, 14)
#define LCD_LE_CLEAR    LCD_BIT_CLEAR(STM32_GPIOB_OFFSET, 2)  /* GPIO_PORTB|GPIO_PIN2 */
#define LCD_LE_SET      LCD_BIT_SET(STM32_GPIOB_OFFSET, 2)
#define LCD_LE_READ     LCD_BIT_READ(STM32_GPIOB_OFFSET, 2)

#define LCD_CRL         STM32_GPIOE_CRL
#define LCD_CRH         STM32_GPIOE_CRH
#define LCD_INPUT       0x44444444        /* Floating input */
#define LCD_OUTPUT      0x33333333        /* Push/pull output */
#define LCD_ODR         STM32_GPIOE_ODR
#define LCD_IDR         STM32_GPIOE_IDR

/* Touchscreen IC on the LCD module */

#define GPIO_TP_INT     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN5)
#define GPIO_TP_CS      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN9)

/* AT45DB161B Data Flash on the LCD module */

#define GPIO_LCDDF_CS   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN9)

/* SD card on the LCD module */

#define GPIO_LCDSD_CS   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)

/* RS-485
 *
 * -- ---- -------------- ---------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ---------------------------------------------------
 * 88 PD7  485_DIR        SP3485 read enable (not)
 */

/* To be provided */

/* USB
 *
 * -- ---- -------------- ---------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ---------------------------------------------------
 * 95 PB8  USB_PWR        Drives USB VBUS
 */

#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN8)

/* Audio DAC
 *
 * -- ---- -------------- ---------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ---------------------------------------------------
 */

/* To be provided */

/* SPI FLASH
 *
 * -- ---- -------------- ---------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ---------------------------------------------------
 * 96 PB9  F_CS           To both the TFT LCD (CN13) and to the
 *                               W25X16 SPI FLASH
 */

#define GPIO_FLASH_CS   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN9)

/* SD Card
 *
 * -- ---- -------------- ---------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ---------------------------------------------------
 * 53 PB14 SD_CD          Active low: Pulled high.
 *                                    Schematic is wrong LCD_WR is PB14.
 * 58 PD11 SD_CS          Active low: Pulled high
 *                                    (See also TFT LCD CN13, pin 32)
 */

#define GPIO_SD_CD      (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN14)
#define GPIO_SD_CS      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)

/* Relays */

#define NUM_RELAYS      2
#define GPIO_RELAYS_R00 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN0)
#define GPIO_RELAYS_R01 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the STM3240G-EVAL
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the STM3240G-EVAL board.
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
 * Name: stm32_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_ADS7843E
int stm32_tsc_setup(int minor);
#endif

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
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
 * Name: stm32_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.  Requires CONFIG_DISABLE_MOUNTPOINT=n
 *   and CONFIG_STM32_SPI1=y
 *
 ****************************************************************************/

int stm32_sdinitialize(int minor);

/****************************************************************************
 * Name: stm32_w25initialize
 *
 * Description:
 *   Initialize and register the W25 FLASH file system.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_W25
int stm32_w25initialize(int minor);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_SHENZHOUL_SRC_SHENZHOU_H */

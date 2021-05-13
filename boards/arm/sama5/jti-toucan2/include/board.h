/****************************************************************************
 * boards/arm/sama5/jti-toucan2/include/board.h
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

#ifndef __BOARDS_ARM_SAMA5_JTI_TOUCAN2_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAMA5_JTI_TOUCAN2_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#  include <nuttx/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* After power-on reset, the SAMA5 device is running on a 12MHz internal RC.
 * These definitions will configure operational clocking.
 */

/* On-board crystal frequencies */

#define BOARD_MAINOSC_FREQUENCY    (24000000)  /* MAINOSC: 12MHz crystal on-board */
#define BOARD_SLOWCLK_FREQUENCY    (32768)     /* Slow Clock: 32.768KHz */

#if defined(CONFIG_SAMA5_BOOT_SDRAM)
/* When booting from SDRAM, NuttX is loaded in SDRAM by an intermediate
 * bootloader.
 * That bootloader had to have already configured the PLL and SDRAM for
 * proper operation.
 *
 * In this case, we do not reconfigure the clocking.
 * Rather, we need to query the register settings to determine the clock
 * frequencies.
 * We can only assume that the Main clock source is the on-board 24MHz
 * crystal.
 */

#  include <arch/board/board_sdram.h>

#elif defined(CONFIG_JTI_TOUCAN2_492MHZ)

/* This configuration results in a CPU clock of 492MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_492mhz.h>

#endif

/* DBGU  */


/* UART1 console */
#define PIO_UART1_RXD     PIO_UART1_RXD_1
#define PIO_UART1_TXD     PIO_UART1_TXD_1

/* UART3 is connected to the main I/O connector, IOSET 1
 *
 *   ----  ---------- -------------
 *   J2      BOARD       SAMA5D2
 *   PIN      NAME     PIO  FUNCTION
 *   ----- ---------- ------ -------------
 *    17    RS232_RX   PC12 URXD3
 *     4    RS232_TX   PC13 UTXD3
 *   ----- ---------------- -------------
 */

/* IOSET1 pins are declared with _3 suffix for some reason */
#define PIO_UART3_RXD  PIO_UART3_RXD_3
#define PIO_UART3_TXD  PIO_UART3_TXD_3

/* FLEXCOM1 is connected to a Silicon Labs BGM210P022 Bluetooth SOC allowing either SPI or UART/USART mode to the Bluetooth module, IOSET 1 (the only option)
  MOSI & MISO swapped compared to default (at the SI device), but this is configurable in the Bluethooth module.
 
 *   -----    ----------------- ------------------------------------------
 *   BT U22  BOARD                SAMA5D2
 *   PIN      NAME               PIO                 FUNCTION
 *   ----- -------------------- ------------------------------------------
 *    21   BLUETOOTH_CTS_NPCS0  PA25  FLEXCOM1_IO3  SPI NPCS0 or USART CTS  
 *    22   BLUETOOTH_RXD_MOSI   PA24  FLEXCOM1_IO0  SPI MOSI  or USART TXD
 *    23   BLUETOOTH_TXD_MISO   PA23  FLEXCOM1_IO1  SPI MISO  or USART RXD
 *    24   BLUETOOTH_SPCK       PA22  FLEXCOM1_IO2  SPI SPCK  or USART SCK
 *    25   BLUETOOTH_RTS_NPCS1  PA26  FLEXCOM1_IO4  SPI NPCS1 or USART RTS
 *   ----- ----------------     ------------------------------------------
 */

/* pins as per default; no options */

/* FLEXCOM2 is connected as a UART to the GPS receiver, IOSET 1
 *
 *   -----    ------------- -------------
 *   GPS U19  BOARD            SAMA5D2
 *   PIN      NAME          PIO  FUNCTION
 *   ----- ---------------- -------------
 *    20    SERIAL_FROM_GPS PA7 FLEXCOM2
 *    21    SERIAL_TO_GPS   PA6 FLEXCOM2
 *   ----- ---------------- -------------
 */
/* pin defs for IOSET1 are _2 for some odd reason */
#define PIO_FLEXCOM2_IO0  PIO_FLEXCOM2_IO0_2
#define PIO_FLEXCOM2_IO1  PIO_FLEXCOM2_IO1_2

/* work around as the VBASE definitions in _sama5d2x_memorymap.h don't seem right */
//#define SAM_USART2_VBASE SAM_FLEXCOM2_VBASE
//#define SAM_IRQ_USART2 SAM_IRQ_FLEXCOM2

/* FLEXCOM3 is connected as a UART a TJA1021 KLINE/LIN interface, IOSET 2
 
 *   -------  --------------  -----------------------------
 *   LIN U13      BOARD       SAMA5D2
 *   PIN          NAME        PIO         FUNCTION
 *   ------- ---------------  -----------------------------
 *    1      SERIAL_FROM_LIN  PC19  FLEXCOM3_IO1  USART RXD    
 *    4      SERIAL_TO_LIN    PC20  FLEXCOM3_IO0  USART TXD
 *   ------- ---------------  -----------------------------
 */

/* pin defs for IOSET2 are _3 for some odd reason */
#define PIO_FLEXCOM3_IO0  PIO_FLEXCOM3_IO0_3
#define PIO_FLEXCOM3_IO1  PIO_FLEXCOM3_IO1_3

/* work around as the VBASE definitions in _sama5d2x_memorymap.h don't seem right */
//#define SAM_USART3_VBASE SAM_FLEXCOM3_VBASE
//#define SAM_IRQ_USART3 SAM_IRQ_FLEXCOM3


/* FLEXCOM4 is connected as an SPI interface to a MAX31855K thermocouple amplifier, IOSET 1
*/
#define PIO_FLEXCOM4_IO0  PIO_FLEXCOM4_IO0_1
#define PIO_FLEXCOM4_IO1  PIO_FLEXCOM4_IO1_1
#define PIO_FLEXCOM4_IO2  PIO_FLEXCOM4_IO2_1
#define PIO_FLEXCOM4_IO3  PIO_FLEXCOM4_IO3_1
#define PIO_FLEXCOM4_IO4  PIO_FLEXCOM4_IO4_1

/* PCK pins

 * PCLK2 (IOSET 2) is routed to an on board testpoint (TP21) to allow the 32kHz clock to be checked for accuracy */

#define PIO_PMC_PCK2 PIO_PMC_PCK2_2

/* PWM pins

 *   -------  --------------------------------  ------------  -------
 *                                                BOARD       SAMA5D2
 *    USE                                          NAME        PIO     
 *   -----------------------------------------  ------------  -------
 *    0-5V output 1, Main connector J2 pin 18    5V_OUT_1      PA31      
 *    0-5V output 2, Main connector J2 pin 19    5V_OUT_2      PB6
 *    Loweside driver, Main connector J2 pin 12  LOWSIDE_OUT   PB8   
 *   ------- ---------------  -----------------  -----------  --------
*/

#define PIO_PWM0_L PIO_PWM0_L0
#define PIO_PWM2_L PIO_PWM0_L2
#define PIO_PWM3_L PIO_PWM0_L3

/* SPI0 used for MT25 flash and AT25 EEPROM on IOSET1 */
#define PIO_SPI0_MISO PIO_SPI0_MISO_1
#define PIO_SPI0_MOSI PIO_SPI0_MOSI_1
#define PIO_SPI0_SPCK PIO_SPI0_SPCK_1
#define PIO_SPI0_NPCS0 PIO_SPI0_NPCS0_1
#define PIO_SPI0_NPCS1 PIO_SPI0_NPCS1_1

/*SPI1 not used */

/* TWI0 is used for the ACT8946 power managenement device and FUSB302 USB C Controller. IOSET3 */
/* pin defs for IOSET3 are _4 for some odd reason */
#define PIO_TWI0_D PIO_TWI0_D_4
#define PIO_TWI0_CK PIO_TWI0_CK_4

/* TWI1 is used for the right hand ambient light and proximity detector and LED driver chips. IOSET2 */
/* pin defs for IOSET2 are _1 for some odd reason */
#define PIO_TWI1_D PIO_TWI1_D_1
#define PIO_TWI1_CK PIO_TWI1_CK_1

/*CAN0 IOSET2*/

#define PIO_CAN0_RX PIO_CAN0_RX_2
#define PIO_CAN0_TX PIO_CAN0_TX_2
#define SAM_PID_CAN0 SAM_PID_MCAN00 

/*CAN1 default IOSET */
#define SAM_PID_CAN1 SAM_PID_MCAN10 

/* LCDC  */
#define PIO_LCD_DAT2      PIO_LCD_DAT2_1
#define PIO_LCD_DAT3      PIO_LCD_DAT3_1
#define PIO_LCD_DAT4      PIO_LCD_DAT4_1
#define PIO_LCD_DAT5      PIO_LCD_DAT5_1
#define PIO_LCD_DAT6      PIO_LCD_DAT6_1
#define PIO_LCD_DAT7      PIO_LCD_DAT7_1
#define PIO_LCD_DAT10     PIO_LCD_DAT10_1
#define PIO_LCD_DAT11     PIO_LCD_DAT11_1
#define PIO_LCD_DAT12     PIO_LCD_DAT12_1
#define PIO_LCD_DAT13     PIO_LCD_DAT13_1
#define PIO_LCD_DAT14     PIO_LCD_DAT14_1
#define PIO_LCD_DAT15     PIO_LCD_DAT15_1
#define PIO_LCD_DAT18     PIO_LCD_DAT16_1
#define PIO_LCD_DAT19     PIO_LCD_DAT17_1
#define PIO_LCD_DAT20     PIO_LCD_DAT18_1
#define PIO_LCD_DAT21     PIO_LCD_DAT21_1
#define PIO_LCD_DAT22     PIO_LCD_DAT22_1
#define PIO_LCD_DAT23     PIO_LCD_DAT23_1

#define PIO_LCD_DEN		  PIO_LCD_DEN_1
#define PIO_LCD_DISP      PIO_LCD_DISP_2
#define PIO_LCD_HSYNC     PIO_LCD_HSYNC_2
#define PIO_LCD_PCK       PIO_LCD_PCK_1 
#define PIO_LCD_PWM       PIO_LCD_PWM_2
#define PIO_LCD_VSYNC     PIO_LCD_VSYNC_2

#define BOARD_LCDC_OUTPUT_BPP 24       /* Output format to H/W is 24BPP RGB */
#define BOARD_LCDC_WIDTH      800      /* Display width (pixels) */
#define BOARD_LCDC_HEIGHT     480      /* Display height (rows) */
#define BOARD_LCDC_VSPW       10        /* Vertical pulse width (lines) */
#define BOARD_LCDC_HSPW       40      /* Horizontal pulse width (LCDDOTCLK) */
#define BOARD_LCDC_VFPW       22       /* Vertical front porch (lines) */
#define BOARD_LCDC_VBPW       23        /* Vertical back porch (lines) */
#define BOARD_LCDC_HFPW       210      /* Horizontal front porch (LCDDOTCLK) */
#define BOARD_LCDC_HBPW       46       /* Horizontal back porch (LCDDOTCLK) */




/* PIO pins */

//not sure if need to define board specific PIO here?

//#define PIO_TX      IO_

/* SDIO - Used for both Port 0 & 1 ******************************************/

/* 386 KHz for initial inquiry stuff */

//#define BOARD_SDMMC_IDMODE_PRESCALER    SDMMC_SYSCTL_SDCLKFS_DIV256
//#define BOARD_SDMMC_IDMODE_DIVISOR      SDMMC_SYSCTL_DVS_DIV(2)

/* 24.8MHz for other modes */

//#define BOARD_SDMMC_MMCMODE_PRESCALER   SDMMC_SYSCTL_SDCLKFS_DIV8
//#define BOARD_SDMMC_MMCMODE_DIVISOR     SDMMC_SYSCTL_DVS_DIV(1)

//#define BOARD_SDMMC_SD1MODE_PRESCALER   SDMMC_SYSCTL_SDCLKFS_DIV8
//#define BOARD_SDMMC_SD1MODE_DIVISOR     SDMMC_SYSCTL_DVS_DIV(1)

//#define BOARD_SDMMC_SD4MODE_PRESCALER   SDMMC_SYSCTL_SDCLKFS_DIV8
//#define BOARD_SDMMC_SD4MODE_DIVISOR     SDMMC_SYSCTL_DVS_DIV(1)

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__
  .macro config_sdram
  .endm
#endif /* __ASSEMBLY__ */

#endif /* __BOARDS_ARM_SAMA5_JTI_TOUCAN2_INCLUDE_BOARD_H */

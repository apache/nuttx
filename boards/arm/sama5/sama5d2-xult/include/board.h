/****************************************************************************
 * boards/arm/sama5/sama5d2-xult/include/board.h
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

#ifndef __BOARDS_ARM_SAMA5_SAMA5D2_XULT_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAMA5_SAMA5D2_XULT_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#  include <nuttx/irq.h>
#endif

/* Clocking *****************************************************************/

/* On-board crystal frequencies */

#define BOARD_MAINOSC_FREQUENCY    (12000000)  /* MAINOSC: 12MHz crystal on-board */
#define BOARD_SLOWCLK_FREQUENCY    (32768)     /* Slow Clock: 32.768KHz */
#define BOARD_UPLL_FREQUENCY       (480000000) /* USB PLL: 480MHz */

/* After power-on reset, the SAMA5 device is running on a 12MHz internal RC.
 * These definitions will configure operational clocking.
 */

#if defined(CONFIG_SAMA5_BOOT_SDRAM)
/* When booting from SDRAM, NuttX is loaded in SDRAM by an intermediate
 * bootloader.
 * That bootloader had to have already configured the PLL and SDRAM for
 * proper operation.
 *
 * In this case, we don not reconfigure the clocking.
 * Rather, we need to query the register settings to determine the clock
 * frequencies.
 * We can only assume that the Main clock source is the on-board 12MHz
 * crystal.
 */

#  include <arch/board/board_sdram.h>

#elif defined(CONFIG_SAMA5D2XULT_384MHZ)

/* OHCI Only.
 * This is an alternative slower configuration that will produce a 48MHz
 * USB clock with the required accuracy using only PLLA.
 * When PPLA is used to clock OHCI, an additional requirement is the
 * PLLACK be a multiple of 48MHz.
 * This setup results in a CPU clock of 384MHz.
 *
 * This case is only interesting for experimentation.
 */

#  include <arch/board/board_384mhz.h>

#elif defined(CONFIG_SAMA5D2XULT_498MHZ)

/* This is the configuration results in a CPU clock of 498MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_498mhz.h>

#elif defined(CONFIG_SAMA5D2XULT_528MHZ)

/* This is the configuration results in a CPU clock of 528MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_528mhz.h>

#else /* #elif defined(CONFIG_SAMA5D2XULT_396MHZ) */

/* This is the configuration provided in the Atmel example code.
 * This setup results in a CPU clock of 396MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_396mhz.h>

#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LCD Interface, Geometry and Timing ***************************************/

/* This configuration applies only to the TM7000 LCD/Touchscreen module.
 * Other LCDs will require changes.
 *
 * NOTE:
 * The TM7000 user manual claims that the hardware interface is
 * 18-bit RGB666.
 * If you select that, you will get a very pink display (because the
 * upper, "red" bits floating high).
 * By trial and error, the 24-bit select was found to produce the correct
 * color output.
 *
 * NOTE:
 * Timings come from the smaller SAMA5D3x-EK LCD and have not been
 * optimized for this display.
 */

#define BOARD_LCDC_OUTPUT_BPP 24       /* Output format is 24 bpp RGB888 */
#define BOARD_LCDC_WIDTH      800      /* Display width (pixels) */
#define BOARD_LCDC_HEIGHT     480      /* Display height (rows) */
#define BOARD_LCDC_MCK_MUL2   1        /* Source clock is 2*Mck (vs Mck) */
#define BOARD_LCDC_PIXCLK_INV 1        /* Invert pixel clock (falling edge) */
#define BOARD_LCDC_GUARDTIME  9        /* Guard time (frames) */
#define BOARD_LCDC_VSPW       2        /* Vertical pulse width (lines) */
#define BOARD_LCDC_HSPW       128      /* Horizontal pulse width (DOTCLK) */
#define BOARD_LCDC_VFPW       37       /* Vertical front porch (lines) */
#define BOARD_LCDC_VBPW       8        /* Vertical back porch (lines) */
#define BOARD_LCDC_HFPW       168      /* Horizontal front porch (DOTCLK) */
#define BOARD_LCDC_HBPW       88       /* Horizontal back porch (DOTCLK) */

/* Pixel clock rate in Hz (HS period * VS period * BOARD_LCDC_FRAMERATE). */

#define BOARD_LCDC_FRAMERATE  50       /* Frame rate in Hz */
#define BOARD_LCDC_HSPERIOD \
  (BOARD_LCDC_HSPW + BOARD_LCDC_HBPW + BOARD_LCDC_WIDTH + BOARD_LCDC_HFPW)
#define BOARD_LCDC_VSPERIOD \
  (BOARD_LCDC_VSPW + BOARD_LCDC_VBPW + BOARD_LCDC_HEIGHT + BOARD_LCDC_VFPW)
#define BOARD_LCDC_PIXELCLOCK \
  (BOARD_LCDC_HSPERIOD * BOARD_LCDC_VSPERIOD * BOARD_LCDC_FRAMERATE)

/* Backlight prescaler value and PWM output polarity */

#define BOARD_LCDC_PWMPS  LCDC_LCDCFG6_PWMPS_DIV1
#define BOARD_LCDC_PWMPOL LCDC_LCDCFG6_PWMPOL

/* LCDC PIO configuration ***************************************************/

#define PIO_LCD_DAT2      PIO_LCD_DAT2_2
#define PIO_LCD_DAT3      PIO_LCD_DAT3_2
#define PIO_LCD_DAT4      PIO_LCD_DAT4_2
#define PIO_LCD_DAT5      PIO_LCD_DAT5_2
#define PIO_LCD_DAT6      PIO_LCD_DAT6_2
#define PIO_LCD_DAT7      PIO_LCD_DAT7_2
#define PIO_LCD_DAT10     PIO_LCD_DAT10_2
#define PIO_LCD_DAT11     PIO_LCD_DAT11_2
#define PIO_LCD_DAT12     PIO_LCD_DAT12_2
#define PIO_LCD_DAT13     PIO_LCD_DAT13_2
#define PIO_LCD_DAT14     PIO_LCD_DAT14_2
#define PIO_LCD_DAT15     PIO_LCD_DAT15_2
#define PIO_LCD_DAT18     PIO_LCD_DAT18_2
#define PIO_LCD_DAT19     PIO_LCD_DAT19_2
#define PIO_LCD_DAT20     PIO_LCD_DAT20_2
#define PIO_LCD_DAT21     PIO_LCD_DAT21_2
#define PIO_LCD_DAT22     PIO_LCD_DAT22_2
#define PIO_LCD_DAT23     PIO_LCD_DAT23_2
#define PIO_LCD_DEN       PIO_LCD_DEN_2
#define PIO_LCD_DISP      PIO_LCD_DISP_1
#define PIO_LCD_HSYNC     PIO_LCD_HSYNC_1
#define PIO_LCD_PCK       PIO_LCD_PCK_2
#define PIO_LCD_PWM       PIO_LCD_PWM_1
#define PIO_LCD_VSYNC     PIO_LCD_VSYNC_1

/* Touch screen TWI */

#define PIO_TWI1_CK      (PIO_TWI1_CK_1 | PIO_CFG_PULLUP | \
                          PIO_CFG_DEGLITCH | PIO_DRIVE_HIGH)
#define PIO_TWI1_D       (PIO_TWI1_D_1  | PIO_CFG_PULLUP | \
                          PIO_CFG_DEGLITCH | PIO_DRIVE_HIGH)

/* EMAC MII/RMII connection to KSZ8081 Ethernet PHY *************************/

/* SAMA5D27 Interface        KSZ8081 Interface
 * ---- ------------ ------- ----------------------------------
 * PIO  Usage        Pin     Function
 * ---- ------------ ------- ----------------------------------
 * PB9  COL          COL     Collision Detect Output
 * PB8  CRS          CRS     Carrier Sense Output
 * PB22 MDC          MDC     Management Interface Clock Input
 * PB23 MDIO         MDIO    Management Interface Data I/O
 * PB18 RX0          RX0     Receive Data Output 0
 * PB19 RX1          RX1     Receive Data Output 1
 * PB10 RX2          RX2     Receive Data Output 2
 * PB11 RX3          RX3     Receive Data Output 3
 * PB7  RXCK         RXCK    Receive Clock Output
 * PB16 RXDV         RXDV    Receive Data Valid Output
 * PB17 RXER         RXER    Receive Error Output
 * PB20 TX0          TX0     Transmit Data Input 0
 * PB21 TX1          TX1     Transmit Data Input 1
 * PB12 TX2          TX2     Transmit Data Input 2
 * PB13 TX3          TX3     Transmit Data Input 3
 * PB14 TXCK         TXCK    Transmit Clock Input
 * PB15 TXEN         TXEN    Transmit Enable Input
 * PB6  TXER         TXER    Transmit Error Input
 * ---- ------------ ------- ----------------------------------
 */

#define PIO_EMAC0_COL     PIO_EMAC0_COL_3
#define PIO_EMAC0_CRS     PIO_EMAC0_CRS_3
#define PIO_EMAC0_MDC     PIO_EMAC0_MDC_3
#define PIO_EMAC0_MDIO    PIO_EMAC0_MDIO_3
#define PIO_EMAC0_RX0     PIO_EMAC0_RX0_3
#define PIO_EMAC0_RX1     PIO_EMAC0_RX1_3
#define PIO_EMAC0_RX2     PIO_EMAC0_RX2_3
#define PIO_EMAC0_RX3     PIO_EMAC0_RX3_3
#define PIO_EMAC0_RXCK    PIO_EMAC0_RXCK_3
#define PIO_EMAC0_RXDV    PIO_EMAC0_RXDV_3
#define PIO_EMAC0_RXER    PIO_EMAC0_RXER_3
#define PIO_EMAC0_TX0     PIO_EMAC0_TX0_3
#define PIO_EMAC0_TX1     PIO_EMAC0_TX1_3
#define PIO_EMAC0_TX2     PIO_EMAC0_TX2_3
#define PIO_EMAC0_TX3     PIO_EMAC0_TX3_3
#define PIO_EMAC0_TXCK    PIO_EMAC0_TXCK_3
#define PIO_EMAC0_TXEN    PIO_EMAC0_TXEN_3
#define PIO_EMAC0_TXER    PIO_EMAC0_TXER_3

/* Image Sensor Controller - ISC Pin Definitions ****************************/

#define PIO_ISC_D0        PIO_ISC_D0_3
#define PIO_ISC_D1        PIO_ISC_D1_3
#define PIO_ISC_D2        PIO_ISC_D2_3
#define PIO_ISC_D3        PIO_ISC_D3_3
#define PIO_ISC_D4        PIO_ISC_D4_3
#define PIO_ISC_D5        PIO_ISC_D5_3
#define PIO_ISC_D6        PIO_ISC_D6_3
#define PIO_ISC_D7        PIO_ISC_D7_3
#define PIO_ISC_D8        PIO_ISC_D8_3
#define PIO_ISC_D9        PIO_ISC_D9_3
#define PIO_ISC_D10       PIO_ISC_D10_3
#define PIO_ISC_D11       PIO_ISC_D11_3
#define PIO_ISC_FIELD     PIO_ISC_FIELD_3
#define PIO_ISC_HSYNC     PIO_ISC_HSYNC_3
#define PIO_ISC_MCK       PIO_ISC_MCK_3
#define PIO_ISC_PCK       PIO_ISC_PCK_3
#define PIO_ISC_VSYNC     PIO_ISC_VSYNC_3

/* QSPI0 definitions ********************************************************/

/* There is a QSPI Flash on board the SAMA5D2-XULT.
 * The QSPI Flash is connected to QSPI0 IOSET3 and can be used for
 * boot flash.
 *
 *   ------------------------------ ------------------- ---------------------
 *   SAMA5D2 PIO                    SIGNAL              USAGE
 *   ------------------------------ ------------------- ---------------------
 *   PA23                           QSPI0_CS            QSPI Chip Selection
 *   PA24                           QSPI0_IO0           QSPI Data0
 *   PA25                           QSPI0_IO1           QSPI Data1
 *   PA26                           QSPI0_IO2           QSPI Data2
 *   PA27                           QSPI0_IO3           QSPI Data3
 *   PA22                           QSPI0_SCK           QSPI Clock
 *   ------------------------------ ------------------- ---------------------
 */

#define PIO_QSPI0_CS      PIO_QSPI0_CS_3
#define PIO_QSPI0_IO0     PIO_QSPI0_IO0_3
#define PIO_QSPI0_IO1     PIO_QSPI0_IO1_3
#define PIO_QSPI0_IO2     PIO_QSPI0_IO2_3
#define PIO_QSPI0_IO3     PIO_QSPI0_IO3_3
#define PIO_QSPI0_SCK     PIO_QSPI0_SCK_3

/* LED definitions **********************************************************/

/* There is an RGB LED on board the SAMA5D2-XULT.
 * The RED component is driven by the SDHC_CD pin (PA13) and so will not
 * be used.  The LEDs are provided VDD_LED and so bringing the LED low will
 * will illuminated the LED.
 *
 *   ------------------------------ ------------------- ---------------------
 *   SAMA5D2 PIO                    SIGNAL              USAGE
 *   ------------------------------ ------------------- ---------------------
 *   PA13                           SDHC_CD_PA13        Red LED
 *   PB5                            LED_GREEN_PB5       Green LED
 *   PB0                            LED_BLUE_PB0        Blue LED
 *   ------------------------------ ------------------- ---------------------
 */

#ifndef CONFIG_ARCH_LEDS

/* LED index values for use with board_userled() */

#define BOARD_GREEN       0
#define BOARD_BLUE        1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_GREEN_BIT  (1 << BOARD_GREEN)
#define BOARD_BLUE_BIT   (1 << BOARD_BLUE)

#else

/* LED index values for use with board_userled() */

#define BOARD_BLUE        0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_BLUE_BIT   (1 << BOARD_BLUE)
#endif

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows.  Note that only the GREEN LED is used in this case
 *
 *      SYMBOL            Val    Meaning                   Green LED
 *      ----------------- ---   -----------------------  -----------
 */

#define LED_STARTED       0  /* NuttX has been started     OFF       */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated    OFF       */
#define LED_IRQSENABLED   0  /* Interrupts enabled         OFF       */
#define LED_STACKCREATED  1  /* Idle stack created         ON        */
#define LED_INIRQ         2  /* In an interrupt            N/C       */
#define LED_SIGNAL        2  /* In a signal handler        N/C       */
#define LED_ASSERTION     2  /* An assertion failed        N/C       */
#define LED_PANIC         3  /* The system has crashed     Flash     */
#undef  LED_IDLE             /* MCU is is sleep mode       Not used  */

/* Thus if the Green LED is statically on, NuttX has successfully  booted
 * and is, apparently, running normally.
 * If LED is flashing at approximately 2Hz, then a fatal error has been
 * detected and the system has halted.
 */

/* Button definitions *******************************************************/

/* A single button, PB_USER (PB6), is available on the SAMA5D2-XULT
 *
 *  ------------------------------ ------------------- ----------------------
 *  SAMA5D2 PIO                    SIGNAL              USAGE
 *  ------------------------------ ------------------- ----------------------
 *  PB6                            USER_PB_PB6         PB_USER push button
 *  ------------------------------ ------------------- ----------------------
 *
 *  Closing PB_USER will bring PB6 to ground so 1) PB6 should have a weak
 * pull-up, and 2) when PB_USER is pressed, a low value will be senses.
 */

#define BUTTON_USER       0
#define NUM_BUTTONS       1

#define BUTTON_USER_BIT   (1 << BUTTON_USER)

/* Pin disambiguation *******************************************************/

/* Alternative pin selections are provided with a numeric suffix like _1, _2,
 * etc. Drivers, however, will use the pin selection without the numeric
 * suffix.
 * Additional definitions are required in this board.h file.
 * For example, if we wanted the PCK0on PB26, then the following definition
 * should appear in the board.h header file for that board:
 *
 *   #define PIO_PMC_PCK0 PIO_PMC_PCK0_1
 *
 * The PCK logic will then automatically configure PB26 as the PCK0 pin.
 */

/* DEBUG / DBGU Port (J1).  There is a TTL serial connection available on
 * pins 2 and 3 of the DEBUG connector.  This may be driven by UART1,
 * depending upon the setting of JP2 (DBGU_PE on the schematic, DEBUG_DIS
 * on the board):
 *
 *   ---- ------------------------ -------------
 *   J1   SCHEMATIC                   SAMA5D2
 *   PIN  NAME(s)                  PIO  FUNCTION
 *   ---- ------------------------ -------------
 *    2   DBGU_TXD  DBGU_UTXD1_PD3 PD3  UTXD1
 *    3   DBGU_RXD  DBGU_URXD1_PD2 PD2  URXD1
 *   ---- ------------------------ -------------
 */

#define PIO_UART1_RXD     PIO_UART1_RXD_1
#define PIO_UART1_TXD     PIO_UART1_TXD_1

/* Standard UART on Arduino connector (J22) is UART2.
 *
 *   ---- ------- -------------
 *   J22  BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    7   URXD2   PD4 UART2 URXD2
 *    8   UTXD2   PD5 UART2 UTXD2
 *   ---- ------- -------------
 */

#define PIO_UART2_RXD     PIO_UART2_RXD_2
#define PIO_UART2_TXD     PIO_UART2_TXD_2

/* Standard UART on Arduino connector (J17) is UART3.
 *
 *   ---- ------- -------------
 *   J17  BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    27   URXD3  PB11 UART3 URXD3
 *    28   UTXD3  PB12 UART3 UTXD3
 *   ---- ------- -------------
 */

#define PIO_UART3_RXD     PIO_UART3_RXD_1
#define PIO_UART3_TXD     PIO_UART3_TXD_1

/* Standard UART on Arduino connector (J21) is FLEXCOM4.
 *
 *   ---- ------- -------------
 *   J21  BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    7   F4_TXD  PD12 FLEXCOM4
 *    8   F4_RXD  PD13 FLEXCOM4
 *   ---- ------- -------------
 */

#define PIO_FLEXCOM4_IO0  PIO_FLEXCOM4_IO0_2
#define PIO_FLEXCOM4_IO1  PIO_FLEXCOM4_IO1_2
#define PIO_FLEXCOM4_IO2  PIO_FLEXCOM4_IO1_2
#define PIO_FLEXCOM4_IO3  PIO_FLEXCOM4_IO3_2
#define PIO_FLEXCOM4_IO4  PIO_FLEXCOM4_IO4_2

/* PWM channel 0 */

#define PIO_PWM0_H        PIO_PWM0_H0

/* Other USARTs are available on J22:
 *
 *   ---- ------- -------------
 *   J22  BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    3   F0_TXD  PB28 FLEXCOM0
 *    4   F0_RXD  PB29 FLEXCOM0
 *    5   F3_TXD  PB23 FLEXCOM3
 *    6   F3_RXD  PB22 FLEXCOM3
 *   ---- ------- -------------
 */

#define PIO_FLEXCOM3_IO0  PIO_FLEXCOM3_IO0_2
#define PIO_FLEXCOM3_IO1  PIO_FLEXCOM3_IO1_2
#define PIO_FLEXCOM3_IO2  PIO_FLEXCOM3_IO1_2
#define PIO_FLEXCOM3_IO3  PIO_FLEXCOM3_IO3_2
#define PIO_FLEXCOM3_IO4  PIO_FLEXCOM3_IO4_2

#define PIO_FLEXCOM2_IO0  PIO_FLEXCOM2_IO0_2
#define PIO_FLEXCOM2_IO1  PIO_FLEXCOM2_IO1_2
#define PIO_FLEXCOM2_IO3  PIO_FLEXCOM2_IO3_2
#define PIO_FLEXCOM2_IO4  PIO_FLEXCOM2_IO4_2

/* UARTs available on EXT1
 *
 *   ---- ------- -------------
 *   EXT1 BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    13  UART_RX PA23 FLEXCOM1
 *    14  UART_TX PA24 FLEXCOM1
 *   ---- ------- ---- --------
 */

/* UARTs available on EXT2
 *
 *   ---- ------- -------------
 *   EXT2 BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    13  UART_RX PB29 FLEXCOM0
 *    14  UART_TX PB28 FLEXCOM0
 *   ---- ------- ---- --------
 */

/* SPIs available on EXT1
 *
 *   ---- ------- -------------
 *   EXT1 BOARD      SAMA5D2
 *   PIN  NAME     PIO  FUNCTION
 *   ---- ------- -------------
 *    15  SPI_SS   PD29 SPI1
 *    16  SPI_MOSI PD26 SPI1
 *    17  SPI_MISO PD27 SPI1
 *    18  SPI_SCK  PD25 SPI1
 *   ---- ------- ---- --------
 */

#define PIO_SPI1_MISO     PIO_SPI1_MISO_1
#define PIO_SPI1_MOSI     PIO_SPI1_MOSI_1
#define PIO_SPI1_NPCS1    PIO_SPI1_NPCS1_1
#define PIO_SPI1_SPCK     PIO_SPI1_SPCK_1

/* SPI0 Definition on EXP */

#define PIO_SPI0_MISO     PIO_SPI0_MISO_1
#define PIO_SPI0_MOSI     PIO_SPI0_MOSI_1
#define PIO_SPI0_NPCS0    PIO_SPI0_NPCS0_1
#define PIO_SPI0_SPCK     PIO_SPI0_SPCK_1

/* CANs are available on J9:
 *
 *   ---- ------- -------------
 *   J9   BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    5   CANRX1  PC27 MCAN1-RX
 *    6   CANTX1  PC26 MCAN1-TX
 *    7   CANRX0  PC11 MCAN0-RX
 *    8   CANTX0  PC10 MCAN0-TX
 *   ---- ------- -------------
 */

#define PIO_MCAN0_RX      PIO_MCAN0_RX_2
#define PIO_MCAN0_TX      PIO_MCAN0_TX_2

/* SDIO - Used for both Port 0 & 1 ******************************************/

/* 386 KHz for initial inquiry stuff */

#define BOARD_SDMMC_IDMODE_PRESCALER    SDMMC_SYSCTL_SDCLKFS_DIV256
#define BOARD_SDMMC_IDMODE_DIVISOR      SDMMC_SYSCTL_DVS_DIV(2)

/* 24.8MHz for other modes */

#define BOARD_SDMMC_MMCMODE_PRESCALER   SDMMC_SYSCTL_SDCLKFS_DIV8
#define BOARD_SDMMC_MMCMODE_DIVISOR     SDMMC_SYSCTL_DVS_DIV(1)

#define BOARD_SDMMC_SD1MODE_PRESCALER   SDMMC_SYSCTL_SDCLKFS_DIV8
#define BOARD_SDMMC_SD1MODE_DIVISOR     SDMMC_SYSCTL_DVS_DIV(1)

#define BOARD_SDMMC_SD4MODE_PRESCALER   SDMMC_SYSCTL_SDCLKFS_DIV8
#define BOARD_SDMMC_SD4MODE_DIVISOR     SDMMC_SYSCTL_DVS_DIV(1)

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__
  .macro config_sdram
  .endm
#endif /* __ASSEMBLY__ */

#endif /* __BOARDS_ARM_SAMA5_SAMA5D2_XULT_INCLUDE_BOARD_H */

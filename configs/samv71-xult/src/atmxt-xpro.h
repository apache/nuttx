/************************************************************************************
 * configs/samv71-xult/src/atmxt-xpro.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIGS_ATMXT_XPRO_SRC_ATMXT_XPRO_H
#define __CONFIGS_ATMXT_XPRO_SRC_ATMXT_XPRO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#define HAVE_MAXTOUCH    1
#define HAVE_ILI9488_SMC 1
#undef  HAVE_ILI9488_SPI    /* Not yet suppported */

/* maXTouch and LCD only available if the maXTouch Xplained Pro is connected */

#ifndef CONFIG_SAMV71XULT_MXTXPLND
#  undef HAVE_MAXTOUCH
#  undef HAVE_ILI9488_SMC
#  undef HAVE_ILI9488_SPI
#endif

/* maXTouch is only available if the maXTouch driver is enabled */

#if defined(HAVE_MAXTOUCH) && !defined(CONFIG_INPUT_MXT)
#  warning maXTouch support not enabled (CONFIG_INPUT_MXT)
#  undef HAVE_MAXTOUCH
#endif

/* The maXTouch interface requires TWIHS0 */

#if defined(HAVE_MAXTOUCH) && !defined(CONFIG_SAMV7_TWIHS0)
#  warning maXTouch support requires TWIHS0
#  undef HAVE_MAXTOUCH
#endif

/* Verify the maXTouch connector configuration */

#ifdef HAVE_MAXTOUCH
/* maXTouch Explained Pro connect on EXT1 */

#  if defined(CONFIG_SAMV71XULT_MXTXPLND_EXT1)
#    ifndef CONFIG_SAMV7_GPIOD_IRQ
#      warning maXTouch on EXT1 requires CONFIG_SAMV7_GPIOD_IRQ
#      undef HAVE_MAXTOUCH
#    endif

/* maXTouch Explained Pro connect on EXT2 */

#  elif defined(CONFIG_SAMV71XULT_MXTXPLND_EXT2)
#    ifndef CONFIG_SAMV7_GPIOA_IRQ
#      warning maXTouch on EXT2 requires CONFIG_SAMV7_GPIOA_IRQ
#      undef HAVE_MAXTOUCH
#    endif

/* maXTouch Explained Pro connect on LCD (EXT4) */

#  elif defined(CONFIG_SAMV71XULT_MXTXPLND_LCD)
#    ifndef CONFIG_SAMV7_GPIOD_IRQ
#      warning maXTouch on EXT4 requires CONFIG_SAMV7_GPIOD_IRQ
#      undef HAVE_MAXTOUCH
#    endif

#  else
#    warning maXTouch requires CONFIG_SAMV71XULT_MXTXPLND_EXT1 or EXT2
#    undef HAVE_MAXTOUCH
#  endif
#endif

/* ILI9488 LCD */

#ifdef HAVE_ILI9488_SMC
/* ILI9488 parallel mode requires use of LCD connector and SMC and DMA support */

#  ifndef CONFIG_SAMV71XULT_MXTXPLND_LCD
#    warning The ILI8488 LCD must be connected on LCD EXT4 (CONFIG_SAMV71XULT_MXTXPLND_LCD)
#    undef HAVE_ILI9488_SMC
#  endif

/* ILI9488 parallel mode requires SMC/EBI and XDMAC support */

#  ifndef CONFIG_SAMV7_SMC
#    warning The ILI8488 LCD requires SMC support (CONFIG_SAMV7_SMC)
#    undef HAVE_ILI9488_SMC
#  endif

#  ifndef CONFIG_SAMV7_XDMAC
#    warning The ILI8488 LCD requires DMA support (CONFIG_SAMV7_XDMAC)
#    undef HAVE_ILI9488_SMC
#  endif
#endif

#ifdef HAVE_ILI9488_SPI
/* ILI9488 serial mode requires use of EXT1 or EXT2 connector */

#  if !defined(CONFIG_SAMV71XULT_MXTXPLND_EXT1) && !defined(CONFIG_SAMV71XULT_MXTXPLND_EXT2)
#    warning serial ILI9488 must be connected or EXT1 or EXT2 (CONFIG_SAMV71XULT_MXTXPLND_EXT1/2)
#    undef HAVE_ILI9488_SPI
#  endif

/* ILI9488 serial mode requires SPI0 */

#  ifndef CONFIG_SAMV7_SPI0
#    warning Serial ILI9488 requires SPI0 support
#    undef HAVE_ILI9488_SPI
#  endif
#endif

/* SAMV71-XULT GPIO Pin Definitions *************************************************/

/* maXTouch Xplained Pro LCD
 *
 * maXTouch Xplained Pro Standard Extension Header
 * -----------------------------------------------
 * This LCD could be connected either via EXT1 or EXT2 using the 2x10
 * 20-pin cable and the maXTouch Xplained Pro standard extension
 * header.  Access is then performed in SPI mode.
 *
 * ---- -------- ---- ----------- ---- ----------- ----------------------------------
 *                       SAMV71-XULT               maxTouch Xplained Pro
 * PIN  FUNCTION EXT1 FUNC        EXT2 FUNC        Description
 * ---- -------- ---- ----------- ---- ----------- ----------------------------------
 *  1   ID        -    -           -    -          Communication line to ID chip
 *  2   GND       -    -           -    -          Ground
 *  3   N/C      PC31  -          PD30  -
 *  4   N/C      PA19  -          PC13  -
 *  5   GPIO     PB3  GPIO        PA6  GPIO        Command/Data Select
 *  6   N/C      PB2   -          PD11  -
 *  7   PWM      PA0  PWMC0_PWMH0 PC19 PWMC0_PMWH2 Backlight control
 *  8   N/C      PC30  -          PD26  -
 *  9   GPIO/IRQ PD28 GPIO        PA2  GPIO        IRQ from maXTouch controller
 *  10  GPIO     PA5  GPIO        PA24 GPIO        RESET signal for maXTouch and LCD controller
 *  11  I2C SDA  PA3  TWID0       PA3  TWID0       I2C Data line for maXTouch controller
 *  12  I2C SCL  PA4  TWICK0      PA4  TWICK0      I2C Clock line for maXTouch controller
 *  13  N/C      PB0   -          PA21  -
 *  14  N/C      PB1   -          PB4   -
 *  15  CS       PD25 GPIO        PD27 GPIO        CS line for LCD controller
 *  16  SPI MOSI PD21 SPI0_MOSI   PD21 SPI0_MOSI   SPI Data to LCD controller
 *  17  SPI MISO PD20 SPI0_MISO   PD20 SPI0_MISO   SPI Data from LCD controller
 *  18  SPI SCK  PD22 SPI0_SPCK   PD22 SPI0_SPCK   SPI Clock line
 *  19  GND       -    -           -      -        Ground
 *  20  VCC       -    -           -      -        Target supply voltage
 * ---- -------- ---- ----------- ---- ----------- ----------------------------------
 */

#ifdef CONFIG_SAMV71XULT_MXTXPLND
#  if defined(CONFIG_SAMV71XULT_MXTXPLND_EXT1)
#    define GPIO_ILI9488_RST    (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                                 GPIO_PORT_PIOA | GPIO_PIN5)
#    define GPIO_ILI9488_BLOFF  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                                 GPIO_PORT_PIOA | GPIO_PIN0)

/* maXTouch definitions when connected via EXT1 */

#    ifdef HAVE_MAXTOUCH
#      define GPIO_MXT_CHG      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                                 GPIO_INT_FALLING | GPIO_PORT_PIOD | GPIO_PIN28)
#      define IRQ_MXT_CHG       SAM_IRQ_PD28
#    endif /* HAVE_MAXTOUCH */

/* ILI9488 serial mode definitions when connected via EXT1 */

#    ifdef HAVE_ILI9488_SPI
#      define GPIO_ILI9488_CDS  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                                 GPIO_PORT_PIOB | GPIO_PIN3)
#      define GPIO_ILI9488_CS   (PIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                                 GPIO_PORT_PIOD | GPIO_PIN25)
#      define ILI9488_PORT      SPI0_CS1
#    endif /* HAVE_ILI9488_SPI */

#  elif defined(CONFIG_SAMV71XULT_MXTXPLND_EXT2)
/* General definitions when connected via EXT2 */

#    define GPIO_ILI9488_RST    (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                                 GPIO_PORT_PIOA | GPIO_PIN24)
#    define GPIO_ILI9488_BLOFF  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                                 GPIO_PORT_PIOC | GPIO_PIN19)

/* maXTouch definitions when connected via EXT2 */

#    ifdef HAVE_MAXTOUCH
#      define GPIO_MXT_CHG      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                                 GPIO_INT_FALLING | GPIO_PORT_PIOA | GPIO_PIN2)
#      define IRQ_MXT_CHG       SAM_IRQ_PA2
#    endif /* HAVE_MAXTOUCH */

/* ILI9488 serial mode definitions when connected via EXT2 */

#    ifdef HAVE_ILI9488_SPI
#      define GPIO_ILI9488_CDS  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                                 GPIO_PORT_PIOA | GPIO_PIN6)
#      define GPIO_ILI9488_CS   (PIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                                 GPIO_PORT_PIOD | GPIO_PIN27)
#      define MXTXLPND_PORT     SPI0_CS3
#    endif /* HAVE_ILI9488_SPI */

/* maXTouch Xplained Pro Xplained Pro LCD Connector *********************************/
/*
 * Only the parallel mode is supported by this BSP (via SMC/EBI).  The switch mode
 * selector on the back of the maXtouch should be set in the OFF-ON-OFF positions
 * to select 16-bit color mode.
 *
 * ----------------- ------------- --------------------------------------------------
 *        LCD            SAMV71    Description
 * Pin  Function     Pin  Function
 * ---- ------------ ---- -------- --------------------------------------------------
 *  1   ID            -    -       Chip ID communication line
 *  2   GND           -   GND      Ground
 *  3   D0           PC0  D0       Data line
 *  4   D1           PC1  D1       Data line
 *  5   D2           PC2  D2       Data line
 *  6   D3           PC3  D3       Data line
 *  7   GND           -   GND      Ground
 *  8   D4           PC4  D4       Data line
 *  9   D5           PC5  D5       Data line
 * 10   D6           PC6  D6       Data line
 * 11   D7           PC7  D7       Data line
 * 12   GND           -   GND      Ground
 * 13   D8           PE0  D8       Data line
 * 14   D9           PE1  D9       Data line
 * 15   D10          PE2  D10      Data line
 * 16   D11          PE3  D11      Data line
 * 17   GND           -   GND      Ground
 * 18   D12          PE4  D12      Data line
 * 19   D13          PE5  D13      Data line
 * 20   D14          PA15 D14      Data line
 * 21   D15          PA16 D15      Data line
 * 22   GND           -   GND      Ground
 * 23   D16           -    -       Data line
 * 24   D17           -    -       Data line
 * 25   N/C           -    -
 * 26   N/C           -    -
 * 27   GND           -   GND      Ground
 * 28   N/C           -    -
 * 29   N/C           -    -
 * 30   N/C           -    -
 * 31   N/C           -    -
 * 32   GND           -   GND      Ground
 * 33   PCLK/        PC30 GPIO     SMC: Pixel clock Display RAM select.
 *      CMD_DATA_SEL               SPI: One address line of the MCU for displays where it
 *                                      is possible to select either the register or the
 *                                      data interface
 * 34   VSYNC/CS     PD19 NCS3     SMC: Vertical synchronization.
 *                                 SPI: Chip select
 * 35   HSYNC/WE     PC8  NWE      SMC: Horizontal synchronization
 *                                 SPI: Write enable signal
 * 36   DATA ENABLE/ PC11 NRD      SMC: Data enable signal
 *      RE                         SPI: Read enable signal
 * 37   SPI SCK       -    -       SPI: Clock for SPI
 * 38   SPI MOSI      -    -       SPI: Master out slave in line of SPI
 * 39   SPI MISO      -    -       SPI: Master in slave out line of SPI
 * 40   SPI SS        -    -       SPI: Slave select for SPI
 * 41   N/C           -    -
 * 42   TWI SDA      PA3  TWD0     I2C data line (maXTouch®)
 * 43   TWI SCL      PA4  TWCK0    I2C clock line (maXTouch)
 * 44   IRQ1         PD28 WKUP5    maXTouch interrupt line
 * 45   N/C          PA2  WKUP2
 * 46   PWM          PC9  TIOB7    Backlight control
 * 47   RESET        PC13 GPIO     Reset for both display and maxTouch
 * 48   VCC           -    -       3.3V power supply for extension board
 * 49   VCC           -    -       3.3V power supply for extension board
 * 50   GND           -    -       Ground
 * ---- ------------ ---- -------- --------------------------------------------------
 */

#  elif defined(CONFIG_SAMV71XULT_MXTXPLND_LCD)
/* General definitions when connected via LCD (EXT4) */

#    define GPIO_ILI9488_RST    (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                                 GPIO_PORT_PIOC | GPIO_PIN13)
#    define GPIO_ILI9488_BLOFF  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                                 GPIO_PORT_PIOC | GPIO_PIN9)

/* maXTouch definitions when connected via LCD (EXT4) */

#    ifdef HAVE_MAXTOUCH
#      define GPIO_MXT_CHG      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                                 GPIO_INT_FALLING | GPIO_PORT_PIOD | GPIO_PIN28)
#      define IRQ_MXT_CHG        SAM_IRQ_PD28
#    endif /* HAVE_MAXTOUCH */

/* ILI9488 parallel mode definitions when connected via LCD (EXT4) */

#    define GPIO_ILI9488_CDS    (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                                 GPIO_PORT_PIOC | GPIO_PIN30)

#  if 1 /* Until PWM support is available */
#    define GPIO_ILI9488_BKL    (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                                 GPIO_PORT_PIOC | GPIO_PIN9)
#  else
#    define GPIO_ILI9488_BKL    GPIO_TC7_TIOB
#  endif
#  endif /* CONFIG_SAMV71XULT_MXTXLND_xyz */

/* In all configurations, the touchscreen communicates on TWI0, I2C address 0x4a */

#  ifdef HAVE_MAXTOUCH

#    define MXT_TWI_BUS         0
#    define MXT_I2C_ADDRESS     0x4a

#  endif /* HAVE_MAXTOUCH */
#endif /* CONFIG_SAMV71XULT_MXTXPLND */

/************************************************************************************
 * Public Types
 ************************************************************************************/

struct atmxt_config_s
{
  uint16_t addr;
  uint8_t id;
  uint8_t nbytes;
  FAR const uint8_t *bytes;
};

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#ifdef HAVE_MAXTOUCH
/* List of configuration settings.  Terminated with an entry with nbytes == 0 and
 * bytes == NULL;
 */

extern const struct atmxt_config_s g_atmxt_config[];
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_ATMXT_XPRO_SRC_ATMXT_XPRO_H */

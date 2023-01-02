/****************************************************************************
 * boards/arm/stm32/stm32ldiscovery/src/stm32_lcd.c
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

/* References:
 *   - Based on the NuttX LCD1602 driver.
 *   - "STM32L100xx, STM32L151xx, STM32L152xx and STM32L162xx advanced
 *     ARM-based 32-bit MCUs", STMicroelectronics, RM0038
 *   - "STM32L1 discovery kits: STM32L-DISCOVERY and 32L152CDISCOVERY,"
 *     STMicroelectronics, UM1079
 *   - STM32L-Discovery Firmware Pack V1.0.2 (for character encoding)
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/ascii.h>
#include <nuttx/streams.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lcd/slcd_ioctl.h>
#include <nuttx/lcd/slcd_codec.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "hardware/stm32_lcd.h"

#include "stm32ldiscovery.h"

#ifdef CONFIG_STM32_LCD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Define CONFIG_DEBUG_LCD_INFO to enable detailed LCD debug output. */

#ifndef CONFIG_LIBC_SLCDCODEC
#  error "This SLCD driver requires CONFIG_LIBC_SLCDCODEC"
#endif

/* The ever-present MIN/MAX macros ******************************************/

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

/* LCD **********************************************************************/

/* LCD.  The STM32L152RBT6 supports either a 4x32 or 8x28.  The STM32L-
 * Discovery has an LCD 24 segments, 4 commons.  See stm32ldiscovery.h for
 * the pin mapping.
 */

/* Macro to convert an LCD register offset and bit number into a bit-band
 * address:
 */

#define SLCD_OFFSET           (STM32_LCD_BASE - STM32_PERIPH_BASE)
#define SLCD_BBADDR(o,b)      (STM32_PERIPHBB_BASE + ((SLCD_OFFSET + (o)) << 5) + ((b) << 2))

/* Some useful bit-band addresses */

#define SLCD_CR_LCDEN_BB      SLCD_BBADDR(STM32_LCD_CR_OFFSET,0)
#define SLCD_SR_UDR_BB        SLCD_BBADDR(STM32_LCD_SR_OFFSET,2)

/* LCD characteristics */

#define SLCD_NROWS            1
#define SLCD_NCHARS           6
#define SLCD_MAXCONTRAST      7

/* An ASCII character may need to be decorated with a colon or decimal
 * point
 */

#define SLCD_DP               0x01
#define SLCD_COLON            0x02
#define SLCD_NBARS            4

/* Macros used for set/reset the LCD bar */

#define SLCD_BAR0_ON          g_slcdstate.bar[1] |= 8
#define SLCD_BAR0_OFF         g_slcdstate.bar[1] &= ~8
#define SLCD_BAR1_ON          g_slcdstate.bar[0] |= 8
#define SLCD_BAR1_OFF         g_slcdstate.bar[0] &= ~8
#define SLCD_BAR2_ON          g_slcdstate.bar[1] |= 2
#define SLCD_BAR2_OFF         g_slcdstate.bar[1] &= ~2
#define SLCD_BAR3_ON          g_slcdstate.bar[0] |= 2
#define SLCD_BAR3_OFF         g_slcdstate.bar[0] &= ~2

/* These definitions support the logic of slcd_writemem()
 *
 * ---------- ----- ----- ----- ----- -------
 * LCD SIGNAL COM3  COM2  COM1  COM0  RAM BIT
 *
 * ---------- ----- ----- ----- ----- -------
 * LCD SEG0   1N    1P    1D    1E    Bit 0
 * LCD SEG1   1DP   1COL  1C    1M    Bit 1
 * LCD SEG2   2N    2P    2D    2E    Bit 2
 * LCD SEG3   2DP   2COL  2C    2M    Bit 7
 * LCD SEG4   3N    3P    3D    3E    Bit 8
 * LCD SEG5   3DP   3COL  3C    3M    Bit 9
 * LCD SEG6   4N    4P    4D    4E    Bit 10
 * LCD SEG7   4DP   4COL  4C    4M    Bit 11
 * LCD SEG8   5N    5P    5D    5E    Bit 12
 * LCD SEG9   BAR2  BAR3  5C    5M    Bit 13
 * LCD SEG10  6N    6P    6D    6E    Bit 14
 * LCD SEG11  BAR0  BAR1  6C    6M    Bit 15
 * LCD SEG12  6J    6K    6A    6B    Bit 16
 * LCD SEG13  6H    6Q    6F    6G    Bit 17
 * LCD SEG14  5J    5K    5A    5B    Bit 18
 * LCD SEG15  5H    5Q    5F    5G    Bit 19
 * LCD SEG16  4J    4K    4A    4B    Bit 20
 * LCD SEG17  4H    4Q    4F    4G    Bit 21
 * LCD SEG18  3J    3K    3A    3B    Bit 24
 * LCD SEG19  3H    3Q    3F    3G    Bit 25
 * LCD SEG20  2J    2K    2A    2B    Bit 26
 * LCD SEG21  2H    2Q    2F    2G    Bit 27
 * LCD SEG22  1J    1K    1A    1B    Bit 28
 * LCD SEG23  1H    1Q    1F    1G    Bit 29
 * ---------- ----- ----- ----- ----- --------

 * ---------------- ------ ------ ------ ------- ------- --------------------
 * LCD       CHAR 1 CHAR 2 CHAR 3 CHAR 4 CHAR 5  CHAR 6  MASKS
 *  SIGNAL    3210   3210   3210   3210  32  10  32  10
 * --------- ------ ------ ------ ------ --  --- --  --- --------------------
 * LCD SEG0  1      0      0      0     0   0   0   0  CHAR 1: 0xcffffffc
 * LCD SEG1  0      0      0      0     0   0   0   0  CHAR 1: 0xcffffffc
 * LCD SEG2  0      1      0      0     0   0   0   0  CHAR 2: 0xf3ffff7b
 * LCD SEG3  0      1      0      0     0   0   0   0  CHAR 2: 0xf3ffff7b
 * LCD SEG4  0      0      1      0     0   0   0   0  CHAR 3: 0xfcfffcff
 * LCD SEG5  0      0      1      0     0   0   0   0  CHAR 3: 0xfcfffcff
 * LCD SEG6  0      0      0      1     0   0   0   0  CHAR 4: 0xffcff3ff
 * LCD SEG7  0      0      0      1     0   0   0   0  CHAR 4: 0xffcff3ff
 * LCD SEG8  0      0      0      0     1   1   0   0  CHAR 5: 0xfff3cfff/
 *                                                             0xfff3efff
 * LCD SEG9  0      0      0      0     0   1   0   0  CHAR 5: 0xfff3cfff/
 *                                                             0xfff3efff
 * LCD SEG10 0      0      0      0     0   0   1   1  CHAR 6: 0xfffc3fff/
 *                                                             0xfffcbfff
 * LCD SEG11 0      0      0      0     0   0   0   1  CHAR 6: 0xfffc3fff/
 *                                                             0xfffcbfff
 * LCD SEG12 0      0      0      0     0   0   1   1  CHAR 6: 0xfffc3fff/
 *                                                             0xfffcbfff
 * LCD SEG13 0      0      0      0     0   0   1   1  CHAR 6: 0xfffc3fff/
 *                                                             0xfffcbfff
 * LCD SEG14 0      0      0      0     1   1   0   0  CHAR 5: 0xfff3cfff/
 *                                                             0xfff3efff
 * LCD SEG15 0      0      0      0     1   1   0   0  CHAR 5: 0xfff3cfff/
 *                                                             0xfff3efff
 * LCD SEG16 0      0      0      1     0   0   0   0  CHAR 4: 0xffcff3ff
 * LCD SEG17 0      0      0      1     0   0   0   0  CHAR 4: 0xffcff3ff
 * LCD SEG18 0      0      1      0     0   0   0   0  CHAR 3: 0xfcfffcff
 * LCD SEG19 0      0      1      0     0   0   0   0  CHAR 3: 0xfcfffcff
 * LCD SEG20 0      1      0      0     0   0   0   0  CHAR 2: 0xf3ffff7b
 * LCD SEG21 0      1      0      0     0   0   0   0  CHAR 2: 0xf3ffff7b
 * LCD SEG22 1      0      0      0     0   0   0   0  CHAR 1: 0xcffffffc
 * LCD SEG23 1      0      0      0     0   0   0   0  CHAR 1: 0xcffffffc
 * --------- ------ ------ ------ ------- ------- ---------------------------
 */

/* SLCD_CHAR1_MASK  COM0-3 0xcffffffc ..11 .... .... .... .... .... .... ..11
 */

#define SLCD_CHAR1_MASK0      0xcffffffc
#define SLCD_CHAR1_MASK1      SLCD_CHAR1_MASK0
#define SLCD_CHAR1_MASK2      SLCD_CHAR1_MASK0
#define SLCD_CHAR1_MASK3      SLCD_CHAR1_MASK0
#define SLCD_CHAR1_UPDATE0(s) (((uint32_t)(s) & 0x0c) << 26) | \
                              ((uint32_t)(s) & 0x03)
#define SLCD_CHAR1_UPDATE1(s) SLCD_CHAR1_UPDATE0(s)
#define SLCD_CHAR1_UPDATE2(s) SLCD_CHAR1_UPDATE0(s)
#define SLCD_CHAR1_UPDATE3(s) SLCD_CHAR1_UPDATE0(s)

/* SLCD_CHAR2_MASK  COM0-3 0xf3ffff03 .... 22.. .... .... .... .... 2... .2..
 */

#define SLCD_CHAR2_MASK0      0xf3ffff7b
#define SLCD_CHAR2_MASK1      SLCD_CHAR2_MASK0
#define SLCD_CHAR2_MASK2      SLCD_CHAR2_MASK0
#define SLCD_CHAR2_MASK3      SLCD_CHAR2_MASK0
#define SLCD_CHAR2_UPDATE0(s) (((uint32_t)(s) & 0x0c) << 24) | \
                              (((uint32_t)(s) & 0x02) << 6) | \
                              (((uint32_t)(s) & 0x01) << 2)
#define SLCD_CHAR2_UPDATE1(s) SLCD_CHAR2_UPDATE0(s)
#define SLCD_CHAR2_UPDATE2(s) SLCD_CHAR2_UPDATE0(s)
#define SLCD_CHAR2_UPDATE3(s) SLCD_CHAR2_UPDATE0(s)

/* SLCD_CHAR3_MASK  COM0-3 0xfcfffcff .... ..33 .... .... .... ..33 .... ....
 */

#define SLCD_CHAR3_MASK0      0xfcfffcff
#define SLCD_CHAR3_MASK1      SLCD_CHAR3_MASK0
#define SLCD_CHAR3_MASK2      SLCD_CHAR3_MASK0
#define SLCD_CHAR3_MASK3      SLCD_CHAR3_MASK0
#define SLCD_CHAR3_UPDATE0(s) (((uint32_t)(s) & 0x0c) << 22) | \
                              (((uint32_t)(s) & 0x03) << 8)
#define SLCD_CHAR3_UPDATE1(s) SLCD_CHAR3_UPDATE0(s)
#define SLCD_CHAR3_UPDATE2(s) SLCD_CHAR3_UPDATE0(s)
#define SLCD_CHAR3_UPDATE3(s) SLCD_CHAR3_UPDATE0(s)

/* SLCD_CHAR4_MASK  COM0-3 0xffcff3ff .... .... ..44 .... .... 44.. .... ....
 */

#define SLCD_CHAR4_MASK0      0xffcff3ff
#define SLCD_CHAR4_MASK1      SLCD_CHAR4_MASK0
#define SLCD_CHAR4_MASK2      SLCD_CHAR4_MASK0
#define SLCD_CHAR4_MASK3      SLCD_CHAR4_MASK0
#define SLCD_CHAR4_UPDATE0(s) (((uint32_t)(s) & 0x0c) << 18) | \
                              (((uint32_t)(s) & 0x03) << 10)
#define SLCD_CHAR4_UPDATE1(s) SLCD_CHAR4_UPDATE0(s)
#define SLCD_CHAR4_UPDATE2(s) SLCD_CHAR4_UPDATE0(s)
#define SLCD_CHAR4_UPDATE3(s) SLCD_CHAR4_UPDATE0(s)

/* SLCD_CHAR5_MASK  COM0-1 0xfff3cfff .... .... .... 55.. ..55 .... .... ....
 *                  COM2-3 0xfff3efff .... .... .... 55.. ...5 .... .... ....
 */

#define SLCD_CHAR5_MASK0      0xfff3cfff
#define SLCD_CHAR5_MASK1      SLCD_CHAR5_MASK0
#define SLCD_CHAR5_MASK2      0xfff3efff
#define SLCD_CHAR5_MASK3      SLCD_CHAR5_MASK2
#define SLCD_CHAR5_UPDATE0(s) (((uint32_t)(s) & 0x0c) << 16) | \
                              (((uint32_t)(s) & 0x03) << 12)
#define SLCD_CHAR5_UPDATE1(s) SLCD_CHAR5_UPDATE0(s)
#define SLCD_CHAR5_UPDATE2(s) (((uint32_t)(s) & 0x0c) << 16) | \
                              (((uint32_t)(s) & 0x01) << 12)
#define SLCD_CHAR5_UPDATE3(s) SLCD_CHAR5_UPDATE2(s)

/* SLCD_CHAR6_MASK  COM0-1 0xfffc3fff .... .... .... ..66 66.. .... .... ....
 *                  COM2-3 0xfffc3fff .... .... .... ..66 .6.. .... .... ....
 */

#define SLCD_CHAR6_MASK0       0xfffc3fff
#define SLCD_CHAR6_MASK1      SLCD_CHAR6_MASK0
#define SLCD_CHAR6_MASK2       0xfffcbfff
#define SLCD_CHAR6_MASK3      SLCD_CHAR6_MASK2
#define SLCD_CHAR6_UPDATE0(s) (((uint32_t)(s) & 0x04) << 15) | \
                              (((uint32_t)(s) & 0x08) << 13) | \
                              (((uint32_t)(s) & 0x03) << 14)
#define SLCD_CHAR6_UPDATE1(s) SLCD_CHAR6_UPDATE0(s)
#define SLCD_CHAR6_UPDATE2(s) (((uint32_t)(s) & 0x04) << 15) | \
                              (((uint32_t)(s) & 0x08) << 13) | \
                              (((uint32_t)(s) & 0x03) << 14)
#define SLCD_CHAR6_UPDATE3(s) SLCD_CHAR6_UPDATE2(s)

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* Global SLCD state */

struct stm32_slcdstate_s
{
  bool initialized;             /* True: Completed initialization sequence */
  uint8_t curpos;               /* The current cursor position */
  uint8_t buffer[SLCD_NCHARS];  /* SLCD ASCII content */
  uint8_t options[SLCD_NCHARS]; /* With colon or decimal point decoration */
  uint8_t bar[2];               /* Controls the bars on the far right of the SLCD */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* Debug */

#ifdef CONFIG_DEBUG_LCD_INFO
static void slcd_dumpstate(const char *msg);
static void slcd_dumpslcd(const char *msg);
#else
#  define slcd_dumpstate(msg)
#  define slcd_dumpslcd(msg)
#endif

/* Internal utilities */

static void slcd_clear(void);
static uint8_t slcd_getcontrast(void);
static int slcd_setcontrast(uint8_t contrast);
static void slcd_writebar(void);
static inline uint16_t slcd_mapch(uint8_t ch);
static inline void slcd_writemem(uint16_t segset, int curpos);
static void slcd_writech(uint8_t ch, uint8_t curpos, uint8_t options);
static void slcd_appendch(uint8_t ch, uint8_t options);
static void slcd_action(enum slcdcode_e code, uint8_t count);

/* Character driver methods */

static ssize_t slcd_read(struct file *, char *, size_t);
static ssize_t slcd_write(struct file *, const char *, size_t);
static int slcd_ioctl(struct file *filep, int cmd, unsigned long arg);
static int slcd_poll(struct file *filep, struct pollfd *fds,
                     bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the driver state structure (there is no retained state
 * information)
 */

static const struct file_operations g_slcdops =
{
  NULL,          /* open */
  NULL,          /* close */
  slcd_read,     /* read */
  slcd_write,    /* write */
  NULL,          /* seek */
  slcd_ioctl,    /* ioctl */
  NULL,          /* truncate */
  NULL,          /* mmap */
  slcd_poll      /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/* LCD state data */

static struct stm32_slcdstate_s g_slcdstate;

/* LCD Mapping
 *
 *              A
 *          ---------    _
 *         |\   |J  /|  |_| COL
 *        F| H  |  K |B
 *         |  \ | /  |   _
 *         --G-- --M-+  |_| COL
 *         |   /| \  |
 *        E|  Q |  N |C
 *         | /  |P  \|   _
 *          ---------   |_| DP
 *              D
 *
 * LCD character 16-bit-encoding:
 * { E , D , P , N,   M , C , COL , DP,   B , A , K , J,   G , F , Q , H }
 */

#warning "Encodings for all punctuation are incomplete"

/* Space and ASCII punctuation: 0x20-0x2f */

static const uint16_t g_slcdpunct1[ASCII_0 -  ASCII_SPACE] =
{
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,  /* <space> ! " # $ % & '  */
  0x0000, 0x0000, 0xa0dd, 0x0000, 0x0000, 0xa000, 0x0000, 0x00c0   /*       () * + , - . /  */
};

/* ASCII numerals 0-9: 0x30-0x39 */

static const uint16_t g_slcdnummap[ASCII_COLON - ASCII_0] =
{
  0x5f00, 0x4200, 0xf500, 0x6700, 0xea00, 0xaf00, 0xbf00, 0x4600,  /* 0-7 */
  0xff00, 0xef00                                                   /* 8-9 */
};

/* ASCII punctuation: 0x3a-0x40 */

static const uint16_t g_slcdpunct2[ASCII_A - ASCII_COLON] =
{
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000           /*  : ; < = > ? @   */
};

/* Upper case letters A-Z: 0x41-0x5a.  Also lower case letters a-z:
 * 0x61-0x7a
 */

static const uint16_t g_slcdalphamap[ASCII_LBRACKET - ASCII_A] =
{
  0xfe00, 0x6714, 0x1d00, 0x4714, 0x9d00, 0x9c00, 0x3f00, 0xfa00,  /* A-H */
  0x0014, 0x5300, 0x9841, 0x1900, 0x5a48, 0x5a09, 0x5f00, 0xfc00,  /* I-P */
  0x5f01, 0xfc01, 0xaf00, 0x0414, 0x5b00, 0x18c0, 0x5a81, 0x00c9,  /* Q-X */
  0x0058, 0x05c0                                                   /* y-Z */
};

/* ASCII punctuation: 0x5b-0x60 */

static const uint16_t g_slcdpunct3[ASCII_a -  ASCII_LBRACKET] =
{
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000                   /*  [ \ ] ^ _  <right quote>  */
};

/* ASCII punctuation: 0x7b-0x7e */

static const uint16_t g_slcdpunct4[ASCII_DEL -  ASCII_LBRACE] =
{
  0x0000, 0x0000, 0x0000, 0x0000                                    /*  { | } ~  */
};

/* All GPIOs that need to be configured for the STM32L-Discovery LCD */

static uint32_t g_slcdgpio[BOARD_SLCD_NGPIOS] =
{
  BOARD_SLCD_COM0,  BOARD_SLCD_COM1, BOARD_SLCD_COM2, BOARD_SLCD_COM3,

  BOARD_SLCD_SEG0,  BOARD_SLCD_SEG1,  BOARD_SLCD_SEG2,  BOARD_SLCD_SEG3,
  BOARD_SLCD_SEG4,  BOARD_SLCD_SEG5,  BOARD_SLCD_SEG6,  BOARD_SLCD_SEG7,
  BOARD_SLCD_SEG8,  BOARD_SLCD_SEG9,  BOARD_SLCD_SEG10, BOARD_SLCD_SEG11,
  BOARD_SLCD_SEG12, BOARD_SLCD_SEG13, BOARD_SLCD_SEG14, BOARD_SLCD_SEG15,
  BOARD_SLCD_SEG16, BOARD_SLCD_SEG17, BOARD_SLCD_SEG18, BOARD_SLCD_SEG19,
  BOARD_SLCD_SEG20, BOARD_SLCD_SEG21, BOARD_SLCD_SEG22, BOARD_SLCD_SEG23
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slcd_dumpstate
 ****************************************************************************/

#ifdef CONFIG_DEBUG_LCD_INFO
static void slcd_dumpstate(const char *msg)
{
  lcdinfo("%s:\n", msg);
  lcdinfo("  curpos: %d\n",
          g_slcdstate.curpos);
  lcdinfo("  Display: [%c%c%c%c%c%c]\n",
          g_slcdstate.buffer[0], g_slcdstate.buffer[1],
          g_slcdstate.buffer[2], g_slcdstate.buffer[3],
          g_slcdstate.buffer[4], g_slcdstate.buffer[5]);
  lcdinfo("  Options: [%d%d%d%d%d%d]\n",
          g_slcdstate.options[0], g_slcdstate.options[1],
          g_slcdstate.options[2], g_slcdstate.options[3],
          g_slcdstate.options[4], g_slcdstate.options[5]);
  lcdinfo("  Bar:     %02x %02x\n",
          g_slcdstate.bar[0], g_slcdstate.bar[1]);
}
#endif

/****************************************************************************
 * Name: slcd_dumpslcd
 ****************************************************************************/

#ifdef CONFIG_DEBUG_LCD_INFO
static void slcd_dumpslcd(const char *msg)
{
  lcdinfo("%s:\n", msg);
  lcdinfo("  CR: %08x FCR: %08x SR: %08x CLR: %08x\n",
          getreg32(STM32_LCD_CR), getreg32(STM32_LCD_FCR),
          getreg32(STM32_LCD_SR), getreg32(STM32_LCD_CLR));
  lcdinfo("  RAM0L: %08x RAM1L: %08x RAM2L: %08x RAM3L: %08x\n",
          getreg32(STM32_LCD_RAM0L), getreg32(STM32_LCD_RAM1L),
          getreg32(STM32_LCD_RAM2L), getreg32(STM32_LCD_RAM3L));
}
#endif

/****************************************************************************
 * Name: slcd_clear
 ****************************************************************************/

static void slcd_clear(void)
{
  uint32_t regaddr;

  linfo("Clearing\n");

  /* Make sure that any previous transfer is complete.  The firmware sets
   * the UDR each it modifies the LCD_RAM. The UDR bit stays set until the
   * end of the update.  During this time the LCD_RAM is write protected.
   */

  while ((getreg32(STM32_LCD_SR) & LCD_SR_UDR) != 0);

  /* Write all zerios in to the LCD RAM */

  for (regaddr = STM32_LCD_RAML(0); regaddr <= STM32_LCD_RAMH(7); regaddr++)
    {
      putreg32(0, regaddr);
    }

  /* Set all buffered data to undecorated spaces and home the cursor */

  memset(g_slcdstate.buffer, ' ', SLCD_NCHARS);
  memset(g_slcdstate.options, 0, SLCD_NCHARS);
  g_slcdstate.curpos = 0;

  /* Set the UDR bit to transfer the updated data to the second level
   * buffer.
   */

  putreg32(1, SLCD_SR_UDR_BB);
}

/****************************************************************************
 * Name: slcd_getcontrast
 ****************************************************************************/

static uint8_t slcd_getcontrast(void)
{
  return (getreg32(STM32_LCD_FCR) & LCD_FCR_CC_MASK) >> LCD_FCR_CC_SHIFT;
}

/****************************************************************************
 * Name: slcd_setcontrast
 ****************************************************************************/

static int slcd_setcontrast(uint8_t contrast)
{
  uint32_t regval;
  int ret = OK;

  /* Make sure that the contrast setting is within range */

  if (contrast > 7)
    {
      contrast = 7;
      ret = -ERANGE;
    }

  regval = getreg32(STM32_LCD_FCR);
  regval &= ~LCD_FCR_CC_MASK;
  regval |= contrast << LCD_FCR_CC_SHIFT;
  putreg32(regval, STM32_LCD_FCR);

  lcdinfo("contrast: %" PRId32 " FCR: %08x\n",
          getreg32(STM32_LCD_FCR), contrast);

  return ret;
}

/****************************************************************************
 * Name: slcd_writebar
 ****************************************************************************/

static void slcd_writebar(void)
{
  uint32_t regval;

  lcdinfo("bar: %02x %02x\n", g_slcdstate.bar[0], g_slcdstate.bar[1]);
  slcd_dumpslcd("BEFORE WRITE");

  /* Make sure that any previous transfer is complete.  The firmware sets
   * the UDR each it modifies the LCD_RAM. The UDR bit stays set until the
   * end of the update.  During this time the LCD_RAM is write protected.
   */

  while ((getreg32(STM32_LCD_SR) & LCD_SR_UDR) != 0);

  /* Update the BAR */

  regval  = getreg32(STM32_LCD_RAM2L);
  regval &= 0xffff5fff;
  regval |= (uint32_t)(g_slcdstate.bar[0] << 12);
  putreg32(regval, STM32_LCD_RAM2L);

  regval  = getreg32(STM32_LCD_RAM3L);
  regval &= 0xffff5fff;
  regval |= (uint32_t)(g_slcdstate.bar[1] << 12);
  putreg32(regval, STM32_LCD_RAM3L);

  /* Set the UDR bit to transfer the updated data to the second level
   * buffer.
   */

  putreg32(1, SLCD_SR_UDR_BB);
  slcd_dumpslcd("AFTER WRITE");
}

/****************************************************************************
 * Name: slcd_mapch
 ****************************************************************************/

static inline uint16_t slcd_mapch(uint8_t ch)
{
  /* ASCII control characters, the forward delete character, period, colon,
   * and all 8-bit ASCII character have already been handled prior to this
   * function.
   */

  /* Return spaces all control characters (this should not happen) */

  if (ch < ASCII_SPACE)
    {
      return 0x0000;
    }

  /* Handle space and the first block of puncutation */

  if (ch < ASCII_0)
    {
      return g_slcdpunct1[(int)ch - ASCII_SPACE];
    }

  /* Handle numbers */

  else if (ch < ASCII_COLON)
    {
      return g_slcdnummap[(int)ch - ASCII_0];
    }

  /* Handle the next block of puncutation */

  else if (ch < ASCII_A)
    {
      return g_slcdpunct2[(int)ch - ASCII_COLON];
    }

  /* Handle upper case letters */

  else if (ch < ASCII_LBRACKET)
    {
      return g_slcdalphamap[(int)ch - ASCII_A];
    }

  /* Handle the next block of puncutation */

  else if (ch < ASCII_a)
    {
      return g_slcdpunct3[(int)ch - ASCII_LBRACKET];
    }

  /* Handle lower case letters (by mapping them to upper case */

  else if (ch < ASCII_LBRACE)
    {
      return g_slcdalphamap[(int)ch - ASCII_a];
    }

  /* Handle the final block of puncutation */

  else if (ch < ASCII_DEL)
    {
      return g_slcdpunct4[(int)ch - ASCII_LBRACE];
    }

  /* Ignore 8-bit ASCII and DEL (this should not happen) */

  return 0x0000;
}

/****************************************************************************
 * Name: slcd_writemem
 ****************************************************************************/

static inline void slcd_writemem(uint16_t segset, int curpos)
{
  uint8_t segments[4];
  uint32_t ram0;
  uint32_t ram1;
  uint32_t ram2;
  uint32_t ram3;
  int i;
  int j;

  lcdinfo("segset: %04x curpos: %d\n", segset, curpos);
  slcd_dumpslcd("BEFORE WRITE");

  /* Isolate the least significant bits
   *
   * LCD character 16-bit-encoding:
   * { E , D , P , N,   M , C , COL , DP,   B , A , K , J,   G , F , Q , H }
   *
   * segments[0] = { E , D , P , N }
   * segments[1] = { M , C , COL , DP }
   * segments[2] = { B , A , K , J }
   * segments[3] = { G , F , Q , H }
   */

  for (i = 12, j = 0; j < 4; i -= 4, j++)
    {
      segments[j] = (segset >> i) & 0x0f;
    }

  lcdinfo("segments: %02x %02x %02x %02x\n",
          segments[0], segments[1], segments[2], segments[3]);

  /* Make sure that any previous transfer is complete.  The firmware sets
   * the UDR each it modifies the LCD_RAM. The UDR bit stays set until the
   * end of the update.  During this time the LCD_RAM is write protected.
   */

  while ((getreg32(STM32_LCD_SR) & LCD_SR_UDR) != 0);

  /* Now update the SLCD memory for the character at this cursor position by
   * decoding the bit-mapped value
   */

  ram0 = getreg32(STM32_LCD_RAM0L);
  ram1 = getreg32(STM32_LCD_RAM1L);
  ram2 = getreg32(STM32_LCD_RAM2L);
  ram3 = getreg32(STM32_LCD_RAM3L);

  switch (curpos)
    {
    case 0:
      ram0 &= SLCD_CHAR1_MASK0;
      ram0 |= SLCD_CHAR1_UPDATE0(segments[0]);

      ram1 &= SLCD_CHAR1_MASK1;
      ram1 |= SLCD_CHAR1_UPDATE1(segments[1]);

      ram2 &= SLCD_CHAR1_MASK2;
      ram2 |= SLCD_CHAR1_UPDATE2(segments[2]);

      ram3 &= SLCD_CHAR1_MASK3;
      ram3 |= SLCD_CHAR1_UPDATE3(segments[3]);
      break;

    case 1:
      ram0 &= SLCD_CHAR2_MASK0;
      ram0 |= SLCD_CHAR2_UPDATE0(segments[0]);

      ram1 &= SLCD_CHAR2_MASK1;
      ram1 |= SLCD_CHAR2_UPDATE1(segments[1]);

      ram2 &= SLCD_CHAR2_MASK2;
      ram2 |= SLCD_CHAR2_UPDATE2(segments[2]);

      ram3 &= SLCD_CHAR2_MASK3;
      ram3 |= SLCD_CHAR2_UPDATE3(segments[3]);
      break;

    case 2:
      ram0 &= SLCD_CHAR3_MASK0;
      ram0 |= SLCD_CHAR3_UPDATE0(segments[0]);

      ram1 &= SLCD_CHAR3_MASK1;
      ram1 |= SLCD_CHAR3_UPDATE1(segments[1]);

      ram2 &= SLCD_CHAR3_MASK2;
      ram2 |= SLCD_CHAR3_UPDATE2(segments[2]);

      ram3 &= SLCD_CHAR3_MASK3;
      ram3 |= SLCD_CHAR3_UPDATE3(segments[3]);
      break;

    case 3:
      ram0 &= SLCD_CHAR4_MASK0;
      ram0 |= SLCD_CHAR4_UPDATE0(segments[0]);

      ram1 &= SLCD_CHAR4_MASK1;
      ram1 |= SLCD_CHAR4_UPDATE1(segments[1]);

      ram2 &= SLCD_CHAR4_MASK2;
      ram2 |= SLCD_CHAR4_UPDATE2(segments[2]);

      ram3 &= SLCD_CHAR4_MASK3;
      ram3 |= SLCD_CHAR4_UPDATE3(segments[3]);
      break;

    case 4:
      ram0 &= SLCD_CHAR5_MASK0;
      ram0 |= SLCD_CHAR5_UPDATE0(segments[0]);

      ram1 &= SLCD_CHAR5_MASK1;
      ram1 |= SLCD_CHAR5_UPDATE1(segments[1]);

      ram2 &= SLCD_CHAR5_MASK2;
      ram2 |= SLCD_CHAR5_UPDATE2(segments[2]);

      ram3 &= SLCD_CHAR5_MASK3;
      ram3 |= SLCD_CHAR5_UPDATE3(segments[3]);
      break;

    case 5:
      ram0 &= SLCD_CHAR6_MASK0;
      ram0 |= SLCD_CHAR6_UPDATE0(segments[0]);

      ram1 &= SLCD_CHAR6_MASK1;
      ram1 |= SLCD_CHAR6_UPDATE1(segments[1]);

      ram2 &= SLCD_CHAR6_MASK2;
      ram2 |= SLCD_CHAR6_UPDATE2(segments[2]);

      ram3 &= SLCD_CHAR6_MASK3;
      ram3 |= SLCD_CHAR6_UPDATE3(segments[3]);
      break;

    default:
      return;
  }

  putreg32(ram0, STM32_LCD_RAM0L);
  putreg32(ram1, STM32_LCD_RAM1L);
  putreg32(ram2, STM32_LCD_RAM2L);
  putreg32(ram3, STM32_LCD_RAM3L);

  /* Set the UDR bit to transfer the updated data to the second level
   * buffer.
   */

  putreg32(1, SLCD_SR_UDR_BB);
  slcd_dumpslcd("AFTER WRITE");
}

/****************************************************************************
 * Name: slcd_writech
 ****************************************************************************/

static void slcd_writech(uint8_t ch, uint8_t curpos, uint8_t options)
{
  uint16_t segset;

  /* Map the character code to a 16-bit encoded value */

  segset = slcd_mapch(ch);

  /* Check if the character should be decorated with a decimal point or
   * colon
   */

  if ((options & SLCD_DP) != 0)
    {
      segset |= 0x0002;
    }
  else if ((options & SLCD_COLON) != 0)
    {
      segset |= 0x0020;
    }

  lcdinfo("ch: [%c] options: %02x segset: %04x\n", ch, options, segset);

  /* Decode the value and write it to the SLCD segment memory */

  slcd_writemem(segset, curpos);

  /* Save these values in the state structure */

  g_slcdstate.buffer[curpos]  = ch;
  g_slcdstate.options[curpos] = options;

  slcd_dumpstate("AFTER WRITE");
}

/****************************************************************************
 * Name: slcd_appendch
 ****************************************************************************/

static void slcd_appendch(uint8_t ch, uint8_t options)
{
  lcdinfo("ch: [%c] options: %02x\n", ch, options);

  /* Write the character at the current cursor position */

  slcd_writech(ch, g_slcdstate.curpos, options);
  if (g_slcdstate.curpos < (SLCD_NCHARS - 1))
    {
      g_slcdstate.curpos++;
    }

  slcd_dumpstate("AFTER APPEND");
}

/****************************************************************************
 * Name: slcd_action
 ****************************************************************************/

static void slcd_action(enum slcdcode_e code, uint8_t count)
{
  lcdinfo("Action: %d count: %d\n", code, count);
  slcd_dumpstate("BEFORE ACTION");

  switch (code)
    {
      /* Erasure */

      case SLCDCODE_BACKDEL:         /* Backspace (backward delete) N characters */
        {
          int tmp;

          /* If we are at the home position or if the count is zero, then
           * ignore the action
           */

          if (g_slcdstate.curpos < 1 || count < 1)
            {
              break;
            }

          /* Otherwise, BACKDEL is like moving the cursor back N characters
           * then doing a forward deletion.  Decrement the cursor position
           * and fall through.
           */

           tmp = (int)g_slcdstate.curpos - count;
           if (tmp < 0)
             {
               tmp   = 0;
               count = g_slcdstate.curpos;
             }

           /* Save the updated cursor positions */

           g_slcdstate.curpos = tmp;
         }

      case SLCDCODE_FWDDEL:          /* DELete (forward delete) N characters moving text */
        if (count > 0)
          {
            int nchars;
            int nmove;
            int i;

            /* How many characters are to the right of the cursor position
             * (including the one at the cursor position)?  Then get the
             * number of characters to move.
             */

            nchars = SLCD_NCHARS - g_slcdstate.curpos;
            nmove  = MIN(nchars, count) - 1;

            /* Move all characters after the current cursor position left
             * by 'nmove' characters
             */

            for (i = g_slcdstate.curpos + nmove; i < SLCD_NCHARS - 1; i++)
              {
                slcd_writech(g_slcdstate.buffer[i - nmove], i,
                             g_slcdstate.options[i - nmove]);
              }

            /* Erase the last 'nmove' characters on the display */

            for (i = SLCD_NCHARS - nmove; i < SLCD_NCHARS; i++)
              {
                slcd_writech(' ', i, 0);
              }
          }
        break;

      case SLCDCODE_ERASE:           /* Erase N characters from the cursor position */
        if (count > 0)
          {
            int last;
            int i;

            /* Get the last position to clear and make sure that the last
             * position is on the SLCD.
             */

            last = g_slcdstate.curpos + count - 1;
            if (last >= SLCD_NCHARS)
              {
                last = SLCD_NCHARS - 1;
              }

            /* Erase N characters after the current cursor position left by
             * one
             */

            for (i = g_slcdstate.curpos; i < last; i++)
              {
                slcd_writech(' ', i, 0);
              }
          }
        break;

      case SLCDCODE_CLEAR:           /* Home the cursor and erase the entire display */
        {
          /* This is like HOME followed by ERASEEOL.  Home the cursor and
           * fall through.
           */

          g_slcdstate.curpos = 0;
        }

      case SLCDCODE_ERASEEOL:        /* Erase from the cursor position to the end of line */
        {
          int i;

          /* Erase characters after the current cursor position to the end
           * of the line
           */

          for (i = g_slcdstate.curpos; i < SLCD_NCHARS; i++)
            {
              slcd_writech(' ', i, 0);
            }
        }
        break;

      /* Cursor movement */

      case SLCDCODE_HOME:            /* Cursor home */
        {
          g_slcdstate.curpos = 0;
        }
        break;

      case SLCDCODE_END:             /* Cursor end */
        {
          g_slcdstate.curpos = SLCD_NCHARS - 1;
        }
        break;

      case SLCDCODE_LEFT:            /* Cursor left by N characters */
        {
          int tmp = (int)g_slcdstate.curpos - count;

          /* Don't permit movement past the beginning of the SLCD */

          if (tmp < 0)
            {
              tmp = 0;
            }

          /* Save the new cursor position */

          g_slcdstate.curpos = (uint8_t)tmp;
        }
        break;

      case SLCDCODE_RIGHT:           /* Cursor right by N characters */
        {
          int tmp = (int)g_slcdstate.curpos + count;

          /* Don't permit movement past the end of the SLCD */

          if (tmp >= SLCD_NCHARS)
            {
              tmp = SLCD_NCHARS - 1;
            }

          /* Save the new cursor position */

          g_slcdstate.curpos = (uint8_t)tmp;
        }
        break;

      case SLCDCODE_UP:              /* Cursor up by N lines */
      case SLCDCODE_DOWN:            /* Cursor down by N lines */
      case SLCDCODE_PAGEUP:          /* Cursor up by N pages */
      case SLCDCODE_PAGEDOWN:        /* Cursor down by N pages */
        break;                       /* Not supportable on this SLCD */

      /* Blinking */

      case SLCDCODE_BLINKSTART:      /* Start blinking with current cursor position */
      case SLCDCODE_BLINKEND:        /* End blinking after the current cursor position */
      case SLCDCODE_BLINKOFF:        /* Turn blinking off */
        break;                       /* Not implemented */

      /* These are actually unreportable errors */

      default:
      case SLCDCODE_NORMAL:          /* Not a special keycode */
        break;
    }

  slcd_dumpstate("AFTER ACTION");
}

/****************************************************************************
 * Name: slcd_read
 ****************************************************************************/

static ssize_t slcd_read(struct file *filep, char *buffer,
                         size_t len)
{
  int ret = 0;
  int i;

  /* Try to read the entire display.  Notice that the seek offset
   * (filep->f_pos) is ignored.  It probably should be taken into account
   * and also updated after each read and write.
   */

  for (i = 0; i < SLCD_NCHARS && ret < len; i++)
    {
      /* Return the character */

      *buffer++ = g_slcdstate.buffer[i];
      ret++;

      /* Check if the character is decorated with a following period or
       * colon
       */

      if (ret < len && g_slcdstate.buffer[i] != 0)
        {
          if ((g_slcdstate.buffer[i] & SLCD_DP) != 0)
            {
              *buffer++ = '.';
              ret++;
            }
          else if ((g_slcdstate.buffer[i] & SLCD_COLON) != 0)
            {
              *buffer++ = ':';
              ret++;
            }
        }
    }

  slcd_dumpstate("READ");
  return ret;
}

/****************************************************************************
 * Name: slcd_write
 ****************************************************************************/

static ssize_t slcd_write(struct file *filep,
                          const char *buffer, size_t len)
{
  struct lib_meminstream_s instream;
  struct slcdstate_s state;
  enum slcdret_e result;
  uint8_t ch;
  uint8_t count;
  uint8_t prev = ' ';
  bool valid = false;

  /* Initialize the stream for use with the SLCD CODEC */

  lib_meminstream(&instream, buffer, len);

  /* Prime the pump.  This is messy, but necessary to handle decoration on a
   * character based on any following period or colon.
   */

  memset(&state, 0, sizeof(struct slcdstate_s));
  result = slcd_decode(&instream.public, &state, &prev, &count);

  lcdinfo("slcd_decode returned result=%d char=%d count=%d\n",
           result, prev, count);

  switch (result)
    {
      case SLCDRET_CHAR:
        valid = true;
        break;

      case SLCDRET_SPEC:
        {
          slcd_action((enum slcdcode_e)prev, count);
          prev = ' ';
        }
        break;

      case SLCDRET_EOF:
        return 0;
    }

  /* Now decode and process every byte in the input buffer */

  while ((result = slcd_decode(&instream.public,
                               &state, &ch, &count)) != SLCDRET_EOF)
    {
      lcdinfo("slcd_decode returned result=%d char=%d count=%d\n",
              result, ch, count);

      if (result == SLCDRET_CHAR)          /* A normal character was returned */
        {
          /* Check for ASCII control characters */

          if (ch < ASCII_SPACE)
            {
              /* All are ignored except for backspace and carriage return */

              if (ch == ASCII_BS)
                {
                  /* If there is a pending character, then output it now
                   * before performing the action.
                   */

                  if (valid)
                    {
                      slcd_appendch(prev, 0);
                      prev = ' ';
                      valid = false;
                    }

                  /* Then perform the backward deletion */

                  slcd_action(SLCDCODE_BACKDEL, 1);
                }
              else if (ch == ASCII_CR)
                {
                  /* If there is a pending character, then output it now
                   * before performing the action.
                   */

                  if (valid)
                    {
                      slcd_appendch(prev, 0);
                      prev = ' ';
                      valid = false;
                    }

                  /* Then perform the carriage return */

                  slcd_action(SLCDCODE_HOME, 0);
                }
            }

          /* Handle characters decoreated with a period or a colon */

          else if (ch == '.')
            {
              /* Write the previous character with the decimal point
               * appended
               */

              slcd_appendch(prev, SLCD_DP);
              prev = ' ';
              valid = false;
            }
          else if (ch == ':')
            {
              /* Write the previous character with the colon appended */

              slcd_appendch(prev, SLCD_COLON);
              prev = ' ';
              valid = false;
            }

          /* Handle ASCII_DEL */

          else if (ch == ASCII_DEL)
            {
              /* If there is a pending character, then output it now before
               * performing the action.
               */

              if (valid)
                {
                  slcd_appendch(prev, 0);
                  prev = ' ';
                  valid = false;
                }

              /* Then perform the forward deletion */

              slcd_action(SLCDCODE_FWDDEL, 1);
            }

          /* The rest of the 7-bit ASCII characters are fair game */

          else if (ch < 128)
            {
              /* Write the previous character if it valid */

              if (valid)
                {
                  slcd_appendch(prev, 0);
                }

              /* There is now a valid output character */

              prev = ch;
              valid = true;
            }
        }
      else /* (result == SLCDRET_SPEC) */  /* A special SLCD action was returned */
        {
          /* If there is a pending character, then output it now before
           * performing the action.
           */

          if (valid)
            {
              slcd_appendch(prev, 0);
              prev = ' ';
              valid = false;
            }

          /* Then perform the action */

          slcd_action((enum slcdcode_e)ch, count);
        }
    }

  /* Handle any unfinished output */

  if (valid)
    {
      slcd_appendch(prev, 0);
    }

  /* Assume that the entire input buffer was processed */

  return (ssize_t)len;
}

/****************************************************************************
 * Name: slcd_poll
 ****************************************************************************/

static int slcd_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  switch (cmd)
    {
      /* SLCDIOC_GETATTRIBUTES:  Get the attributes of the SLCD
       *
       * argument:  Pointer to struct slcd_attributes_s in which values
       *            will be returned
       */

      case SLCDIOC_GETATTRIBUTES:
        {
          struct slcd_attributes_s *attr =
              (struct slcd_attributes_s *)((uintptr_t)arg);

          lcdinfo("SLCDIOC_GETATTRIBUTES:\n");

          if (!attr)
            {
              return -EINVAL;
            }

          attr->nrows         = SLCD_NROWS;
          attr->ncolumns      = SLCD_NCHARS;
          attr->nbars         = SLCD_NBARS;
          attr->maxcontrast   = SLCD_MAXCONTRAST;
          attr->maxbrightness = 0;
        }
        break;

      /* SLCDIOC_CURPOS:  Get the SLCD cursor positioni (rows x characters)
       *
       * argument:  Pointer to struct slcd_curpos_s in which values will be
       *            returned
       */

      case SLCDIOC_CURPOS:
        {
          struct slcd_curpos_s *curpos =
              (struct slcd_curpos_s *)((uintptr_t)arg);

          lcdinfo("SLCDIOC_CURPOS: row=0 column=%d\n", g_slcdstate.curpos);

          if (!curpos)
            {
              return -EINVAL;
            }

          curpos->row    = 0;
          curpos->column = g_slcdstate.curpos;
        }
        break;

      /* SLCDIOC_SETBAR: Set bars on a bar display
       *
       * argument:  32-bit bitset, with each bit corresponding to one bar.
       */

      case SLCDIOC_SETBAR:
        {
          lcdinfo("SLCDIOC_SETBAR: arg=0x%02lx\n", arg);

          /* Format the bar */

          g_slcdstate.bar[0] = 0;
          g_slcdstate.bar[1] = 0;

          if ((arg & 1) != 0)
            {
              SLCD_BAR0_ON;
            }

          if ((arg & 2) != 0)
            {
              SLCD_BAR1_ON;
            }

          if ((arg & 4) != 0)
            {
              SLCD_BAR2_ON;
            }

          if ((arg & 8) != 0)
            {
              SLCD_BAR3_ON;
            }

          /* Write the bar to SLCD memory */

          slcd_writebar();
        }
        break;

      /* SLCDIOC_GETCONTRAST: Get the current contrast setting
       *
       * argument:  Pointer type int that will receive the current contrast
       *            setting
       */

      case SLCDIOC_GETCONTRAST:
        {
          int *contrast = (int *)((uintptr_t)arg);
          if (!contrast)
            {
              return -EINVAL;
            }

          *contrast = (int)slcd_getcontrast();
          lcdinfo("SLCDIOC_GETCONTRAST: contrast=%d\n", *contrast);
        }
        break;

      /* SLCDIOC_SETCONTRAST: Set the contrast to a new value
       *
       * argument:  The new contrast value
       */

      case SLCDIOC_SETCONTRAST:
        {
          lcdinfo("SLCDIOC_SETCONTRAST: arg=%ld\n", arg);

          if (arg > SLCD_MAXCONTRAST)
            {
              return -ERANGE;
            }

          return slcd_setcontrast((uint8_t)arg);
        }
        break;

      case SLCDIOC_GETBRIGHTNESS:  /* Get the current brightness setting */
      case SLCDIOC_SETBRIGHTNESS:  /* Set the brightness to a new value */
      default:
        return -ENOTTY;
    }

  return OK;
}

/****************************************************************************
 * Name: slcd_poll
 ****************************************************************************/

static int slcd_poll(struct file *filep, struct pollfd *fds,
                     bool setup)
{
  if (setup)
    {
      /* Data is always available to be read / Data can always be written */

      poll_notify(&fds, 1, POLLIN | POLLOUT);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_slcd_initialize
 *
 * Description:
 *   Initialize the STM32L-Discovery LCD hardware and register the character
 *   driver as /dev/slcd0.
 *
 ****************************************************************************/

int stm32_slcd_initialize(void)
{
  uint32_t regval;
  int ret = OK;
  int i;

  /* Only initialize the driver once. */

  if (!g_slcdstate.initialized)
    {
      lcdinfo("Initializing\n");

      /* Configure LCD GPIO pins */

      for (i = 0; i < BOARD_SLCD_NGPIOS; i++)
        {
          stm32_configgpio(g_slcdgpio[i]);
        }

      /* Enable the External Low-Speed (LSE) oscillator and select it as the
       * LCD clock source.
       *
       * NOTE: LCD clocking should already be enabled in the RCC APB1ENR
       * register.
       */

      stm32_rcc_enablelse();

      lcdinfo("APB1ENR: %08" PRIx32 " CSR: %08" PRIx32 "\n",
              getreg32(STM32_RCC_APB1ENR), getreg32(STM32_RCC_CSR));

      /* Set the LCD prescaler and divider values */

      regval  = getreg32(STM32_LCD_FCR);
      regval &= ~(LCD_FCR_DIV_MASK | LCD_FCR_PS_MASK);
      regval |= (LCD_FCR_PS_DIV1 |  LCD_FCR_DIV(31));
      putreg32(regval, STM32_LCD_FCR);

      /* Wait for the FCRSF flag to be set */

      lcdinfo("Wait for FCRSF, FSR: %08" PRIx32 " SR: %08" PRIx32 "\n",
              getreg32(STM32_LCD_FCR), getreg32(STM32_LCD_SR));

      while ((getreg32(STM32_LCD_SR) & LCD_SR_FCRSF) == 0);

      /* Set the duty (1/4), bias (1/3), and the internal voltage source
       * (VSEL=0)
       */

      regval  = getreg32(STM32_LCD_CR);
      regval &= ~(LCD_CR_BIAS_MASK | LCD_CR_DUTY_MASK | LCD_CR_VSEL);
      regval |= (LCD_CR_DUTY_1TO4 | LCD_CR_BIAS_1TO3);
      putreg32(regval, STM32_LCD_CR);

      /* SEG[31:28] are multiplexed with SEG[43:40] */

      regval |= LCD_CR_MUX_SEG;
      putreg32(regval, STM32_LCD_CR);

      /* Set the contrast to the mean value */

      regval  = getreg32(STM32_LCD_FCR);
      regval &= ~LCD_FCR_CC_MASK;
      regval |=  LCD_FCR_CC_VLCD(4);
      putreg32(regval, STM32_LCD_FCR);

      /* No dead time */

      regval &= ~LCD_FCR_DEAD_MASK;
      putreg32(regval, STM32_LCD_FCR);

      /* Set the pulse-on duration to 4/ck_ps */

      regval &= ~LCD_FCR_PON_MASK;
      regval |= LCD_FCR_PON(4);
      putreg32(regval, STM32_LCD_FCR);

      /* Wait Until the LCD FCR register is synchronized */

      lcdinfo("Wait for FCRSF, FSR: %08" PRIx32 " SR: %08" PRIx32 "\n",
              getreg32(STM32_LCD_FCR), getreg32(STM32_LCD_SR));

      while ((getreg32(STM32_LCD_SR) & LCD_SR_FCRSF) == 0);

      /* Enable LCD peripheral */

      putreg32(1, SLCD_CR_LCDEN_BB);

      /* Wait Until the LCD is enabled and the LCD booster is ready */

      lcdinfo("Wait for LCD_SR_ENS and LCD_SR_RDY, "
              "CR: %08" PRIx32 " SR: %08" PRIx32 "\n",
              getreg32(STM32_LCD_CR), getreg32(STM32_LCD_SR));

      while ((getreg32(STM32_LCD_SR) & (LCD_SR_ENS | LCD_SR_RDY)) !=
             (LCD_SR_ENS | LCD_SR_RDY));

      /* Disable blinking */

      regval  = getreg32(STM32_LCD_FCR);
      regval &= ~(LCD_FCR_BLINKF_MASK | LCD_FCR_BLINK_MASK);
      regval |=  (LCD_FCR_BLINK_DISABLE | LCD_FCR_BLINKF_DIV32);
      putreg32(regval, STM32_LCD_FCR);

      slcd_dumpslcd("AFTER INITIALIZATION");

      /* Register the LCD device driver */

      ret = register_driver("/dev/slcd0", &g_slcdops, 0644, &g_slcdstate);
      g_slcdstate.initialized = true;

      /* Then clear the display */

      slcd_clear();
      slcd_dumpstate("AFTER INITIALIZATION");
    }

  return ret;
}

#endif /* CONFIG_STM32_LCD */

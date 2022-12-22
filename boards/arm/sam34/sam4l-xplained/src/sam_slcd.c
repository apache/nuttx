/****************************************************************************
 * boards/arm/sam34/sam4l-xplained/src/sam_slcd.c
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

/* TODO: Add support for additional pixels:  B0-B2, G0-G7, and E0-E7,
 *       probably via ioctl calls.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
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
#include "sam_gpio.h"
#include "sam4l_periphclks.h"
#include "hardware/sam4l_lcdca.h"

#include "sam4l-xplained.h"

#if defined(CONFIG_SAM34_LCDCA) && defined(CONFIG_SAM4L_XPLAINED_SLCD1MODULE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_LIBC_SLCDCODEC
#  error This SLCD driver requires CONFIG_LIBC_SLCDCODEC
#endif

#if !defined(CONFIG_SAM34_OSC32K) && !defined(CONFIG_SAM34_RC32K)
#  error This SLCD driver requires that either CONFIG_SAM34_OSC32K or
#  error CONFIG_SAM34_RC32K be selected in the board configuration
#endif

/* The ever-present MIN/MAX macros ******************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

/* LCD **********************************************************************/

/* LCD characteristics.  The logic in this driver is not portable; it is
 * tailored for the SAM4l Xplained Pro's LED1 module.  However, in an effort
 * to add some reusability to this module, some of the tunable settings are
 * included here as BOARD_ definitions (although they do not appear in the
 * board.h header file.
 */

#define SLCD_NROWS             1
#define SLCD_NCHARS            5
#define SLCD_NBARS             4
#define SLCD_MAXCONTRAST       63

#define BOARD_SLCD_NCOM        4
#define BOARD_SLCD_NSEG        24
#define SLCD_NPINS            (BOARD_SLCD_NCOM+BOARD_SLCD_NSEG+1)

/* An ASCII character may need to be decorated with a preceding decimal
 * point
 */

#define SLCD_DP                0x01

/* LCD controller bias configuration. */

#undef  BOARD_XBIAS
#define BOARD_LPWAVE           1

/* LCD controller initial contrast setting. */

#define BOARD_INITIAL_CONTRAST (SLCD_MAXCONTRAST / 2)

/* LCD controller timing configuration */

#define BOARD_TIM_PRES         0  /* Clock prescaler {0|LCDCA_TIM_PRESC} */
#define BOARD_TIM_CLOCKDIV     8  /* Clock divider {1..8} */
#define BOARD_TIM_FC0          2  /* Frame 0 configuration {0..31} */
#define BOARD_TIM_FC1          2  /* Frame 1 configuration {0..31} */
#define BOARD_TIM_FC2          1  /* Frame 2 configuration {0..31} */

/* LCD controller configuration */

#if BOARD_SLCD_NCOM < 2
#  define LCD_DUTY             LCDCA_CFG_DUTY_STATIC  /* Static COM0 */
#elif BOARD_SLCD_NCOM < 3
#  define LCD_DUTY             LCDCA_CFG_DUTY_1TO2    /* 1/2 COM[0:1] */
#elif BOARD_SLCD_NCOM < 4
#  define LCD_DUTY             LCDCA_CFG_DUTY_1TO3    /* 1/3 COM[0:2] */
#elif BOARD_SLCD_NCOM < 5
#  define LCD_DUTY             LCDCA_CFG_DUTY_1TO4    /* 1/4 COM[0:3] */
#else
#  error Value of BOARD_SLCD_NCOM not supported
#endif

/* LCD Mapping
 *
 *              a
 *          ---------
 *         |\   |h  /|
 *        f| g  |  i |b
 *         |  \ | /  |
 *         --j-- --k-+
 *         |   /| \  |
 *        e|  l |  n |c
 *      _  | /  |m  \|  _
 *   B | |  ---------  | | B
 *      -       d       -
 *
 * ----- ---- ---- ---- ----- ----------------------------------------------
 *       COM0 COM1 COM2 COM3  Comments
 * ----- ---- ---- ---- ----- ----------------------------------------------
 * SEG0  G1   G2   G4   G3    Atmel logo, 4 stage battery-, Dot-point-,
 * SEG1  G0   G6   G7   G5    usband play indicator
 * SEG2  E7   E5   E3   E1    4 stage wireless-, AM-, PM- Volt- and milli
 * SEG3  E6   E4   E2   E0    voltindicator
 * SEG4  A0-h A0-i A0-k A0-n  1st 14-segment character
 * SEG5  B3   A0-f A0-e A0-d
 * SEG6  A0-a A0-b A0-c B4
 * SEG7  A0-g A0-j A0-l A0-m
 * SEG8  A1-h A1-i A1-k A1-n  2nd 14-segment character
 * SEG9  B2   A1-f A1-e A1-d
 * SEG10 A1-a A1-b A1-c B5
 * SEG11 A1-g A1-j A1-l A1-m
 * SEG12 A2-h A2-i A2-k A2-n  3rd 14-segment character
 * SEG13 B1   A2-f A2-e A2-d
 * SEG14 A2-a A2-b A2-c B6
 * SEG15 A2-g A2-j A2-l A2-m
 * SEG16 A3-h A3-i A3-k A3-n  4th 14-segment character
 * SEG17 B0   A3-f A3-e A3-d
 * SEG18 A3-a A3-b A3-c B7
 * SEG19 A3-g A3-j A3-l A3-m
 * SEG20 A4-h A4-i A4-k A4-n  5th 14-segment character. Celsius and
 * SEG21 B8   A4-f A4-e A4-d    Fahrenheit indicator
 * SEG22 A4-a A4-b A4-c B9
 * SEG23 A4-g A4-j A4-l A4-m
 */

#define SLCD_A0_STARTSEG 4
#define SLCD_A0_ENDSEG   7
#define SLCD_A1_STARTSEG 8
#define SLCD_A1_ENDSEG   11
#define SLCD_A2_STARTSEG 12
#define SLCD_A2_ENDSEG   15
#define SLCD_A3_STARTSEG 16
#define SLCD_A3_ENDSEG   19
#define SLCD_A4_STARTSEG 20
#define SLCD_A4_ENDSEG   23

#define SLCD_NB          10   /* Number of 'B' segments B0-B9 */
#define SLCD_NG          8    /* Number of 'G' segments G0-G7 */
#define SLCD_NE          8    /* Number of 'E' segments G0-G7 */

/* Named pixels */

#define SLCD_MINUS       (&g_binfo[0])
#define SLCD_H           (&g_binfo[1])
#define SLCD_M           (&g_binfo[2])
#define SLCD_DP0         (&g_binfo[3])
#define SLCD_DP1         (&g_binfo[4])
#define SLCD_DP2         (&g_binfo[5])
#define SLCD_DP3         (&g_binfo[6])
#define SLCD_DP4         (&g_binfo[7])
#define SLCD_CENTIGRADE  (&g_binfo[8])
#define SLCD_FAHRENHEIT  (&g_binfo[9])

#define SLCD_ATMEL       (&g_ginfo[0])
#define SLCD_BAR0        (&g_ginfo[1])
#define SLCD_BAR1        (&g_ginfo[2])
#define SLCD_BAR2        (&g_ginfo[3])
#define SLCD_BAR3        (&g_ginfo[4])
#define SLCD_COLON       (&g_ginfo[5])
#define SLCD_USB         (&g_ginfo[6])
#define SLCD_PAY         (&g_ginfo[7])

#define SLCD_RNG0        (&g_einfo[0])
#define SLCD_RNG1        (&g_einfo[1])
#define SLCD_RNG2        (&g_einfo[2])
#define SLCD_RNG3        (&g_einfo[3])
#define SLCD_MV          (&g_einfo[4])
#define SLCD_V           (&g_einfo[5])
#define SLCD_PM          (&g_einfo[6])
#define SLCD_AM          (&g_einfo[7])

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* Global SLCD state */

struct sam_slcdstate_s
{
  bool initialized;             /* True: Completed initialization sequence */
  uint8_t curpos;               /* The current cursor position */
  uint8_t buffer[SLCD_NCHARS];  /* SLCD ASCII content */
  uint8_t options[SLCD_NCHARS]; /* Ornamentations */
};

/* Describes one pixel */

struct slcd_pixel_s
{
  uint8_t segment;
  uint8_t com;
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

#if 0 /* Not used */
static void slcd_clear(void);
#endif
static void slcd_setpixel(const struct slcd_pixel_s *info);
static void slcd_clrpixel(const struct slcd_pixel_s *info);
static inline void slcd_setdp(uint8_t curpos);
static inline void slcd_clrdp(uint8_t curpos);
static uint8_t slcd_getcontrast(void);
static int slcd_setcontrast(unsigned int contrast);
static void slcd_writech(uint8_t ch, uint8_t curpos, uint8_t options);
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

/* This is the driver state structure
 * (there is no retained state information)
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
  slcd_poll      /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/* LCD state data */

static struct sam_slcdstate_s g_slcdstate;

/* LCD pin configurations */

static gpio_pinset_t g_slcdgpio[SLCD_NPINS] =
{
  GPIO_LCDCA_COM0,  GPIO_LCDCA_COM1, GPIO_LCDCA_COM2, GPIO_LCDCA_COM3,

  GPIO_LCDCA_SEG0,  GPIO_LCDCA_SEG1,  GPIO_LCDCA_SEG2,  GPIO_LCDCA_SEG3,
  GPIO_LCDCA_SEG4,  GPIO_LCDCA_SEG5,  GPIO_LCDCA_SEG6,  GPIO_LCDCA_SEG7,
  GPIO_LCDCA_SEG8,  GPIO_LCDCA_SEG9,  GPIO_LCDCA_SEG10, GPIO_LCDCA_SEG11,
  GPIO_LCDCA_SEG12, GPIO_LCDCA_SEG13, GPIO_LCDCA_SEG14, GPIO_LCDCA_SEG15,
  GPIO_LCDCA_SEG16, GPIO_LCDCA_SEG17, GPIO_LCDCA_SEG18, GPIO_LCDCA_SEG19,
  GPIO_LCDCA_SEG20, GPIO_LCDCA_SEG21, GPIO_LCDCA_SEG22, GPIO_LCDCA_SEG23,

  GPIO_LCD1_BL
};

/* First segment of each character */

static const uint8_t g_startseg[SLCD_NCHARS] =
{
  SLCD_A0_STARTSEG, SLCD_A1_STARTSEG, SLCD_A2_STARTSEG, SLCD_A3_STARTSEG,
  SLCD_A4_STARTSEG
};

/* Pixel position for each 'B' segment */

static const struct slcd_pixel_s g_binfo[SLCD_NB] =
{
  {17, 0},
  {13, 0},
  {9, 0},
  {5, 0},
  {6, 3},
  {10, 3},
  {14, 3},
  {18, 3},
  {21, 0},
  {22, 3}
};

/* Pixel position for each 'G' segment */

static const struct slcd_pixel_s g_ginfo[SLCD_NG] =
{
  {1, 0},
  {0, 0},
  {0, 1},
  {0, 3},
  {0, 2},
  {1, 3},
  {1, 1},
  {1, 2}
};

/* Pixel position for each 'E' segment */

static const struct slcd_pixel_s g_einfo[SLCD_NE] =
{
  {3, 3},
  {2, 3},
  {3, 2},
  {2, 2},
  {3, 1},
  {2, 1},
  {3, 0},
  {2, 0}
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
  lcdinfo("  Display: [%c%c%c%c%c]\n",
          g_slcdstate.buffer[0],
          g_slcdstate.buffer[1],
          g_slcdstate.buffer[2],
          g_slcdstate.buffer[3],
          g_slcdstate.buffer[4]);
  lcdinfo("  Options: [%d%d%d%d%d]\n",
          g_slcdstate.options[0],
          g_slcdstate.options[1],
          g_slcdstate.options[2],
          g_slcdstate.options[3],
          g_slcdstate.options[4]);
}
#endif

/****************************************************************************
 * Name: slcd_dumpslcd
 ****************************************************************************/

#ifdef CONFIG_DEBUG_LCD_INFO
static void slcd_dumpslcd(const char *msg)
{
  lcdinfo("%s:\n", msg);
  lcdinfo("    CFG: %08x    TIM: %08x    SR: %08x\n",
          getreg32(SAM_LCDCA_CFG), getreg32(SAM_LCDCA_TIM),
          getreg32(SAM_LCDCA_SR));
  lcdinfo("    DR0: %02x %08x DR1: %02x %08x\n",
          getreg32(SAM_LCDCA_DRH0), getreg32(SAM_LCDCA_DRL0),
          getreg32(SAM_LCDCA_DRH1), getreg32(SAM_LCDCA_DRL1));
  lcdinfo("    DR2: %02x %08x DR3: %02x %08x\n",
          getreg32(SAM_LCDCA_DRH2), getreg32(SAM_LCDCA_DRL2),
          getreg32(SAM_LCDCA_DRH3), getreg32(SAM_LCDCA_DRL3));
  lcdinfo("   BCFG: %08x CSRCFG: %08x CMCFG: %08x ACMCFG: %08x\n",
          getreg32(SAM_LCDCA_BCFG), getreg32(SAM_LCDCA_CSRCFG),
          getreg32(SAM_LCDCA_CMCFG), getreg32(SAM_LCDCA_ACMCFG));
  lcdinfo(" ABMCFG: %08x    IMR: %08x   VER: %08x\n",
          getreg32(SAM_LCDCA_ABMCFG), getreg32(SAM_LCDCA_IMR),
          getreg32(SAM_LCDCA_VERSION));
}
#endif

/****************************************************************************
 * Name: slcd_clear
 ****************************************************************************/

#if 0 /* Not used */
static void slcd_clear(void)
{
  linfo("Clearing\n");

  /* Clear display memory */

  putreg32(LCDCA_CR_CDM, SAM_LCDCA_CR);
}
#endif

/****************************************************************************
 * Name: slcd_setpixel
 ****************************************************************************/

static void slcd_setpixel(const struct slcd_pixel_s *info)
{
  uintptr_t regaddr;
  uint32_t regval;

  regaddr = SAM_LCDCA_DRL(info->com);
  regval  = getreg32(regaddr);
  regval |= (1 << info->segment);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: slcd_clrpixel
 ****************************************************************************/

static void slcd_clrpixel(const struct slcd_pixel_s *info)
{
  uintptr_t regaddr;
  uint32_t regval;

  regaddr = SAM_LCDCA_DRL(info->com);
  regval  = getreg32(regaddr);
  regval &= ~(1 << info->segment);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: slcd_setdp
 ****************************************************************************/

static inline void slcd_setdp(uint8_t curpos)
{
  /* Set the decimal point before the current cursor position
   *
   *  B3 B4 B5 B6 B7
   *  .O .O .O .O .O
   */

  slcd_setpixel(&g_binfo[curpos + 3]);
}

/****************************************************************************
 * Name: slcd_clrdp
 ****************************************************************************/

static inline void slcd_clrdp(uint8_t curpos)
{
  /* Set the decimal point before the current cursor position
   *
   *  B3 B4 B5 B6 B7
   *  .O .O .O .O .O
   */

  slcd_clrpixel(&g_binfo[curpos + 3]);
}

/****************************************************************************
 * Name: slcd_getcontrast
 ****************************************************************************/

static uint8_t slcd_getcontrast(void)
{
  uint32_t regval;
  uint32_t ucontrast;
  int32_t  scontrast;

  /* Get the current contrast value */

  regval    = getreg32(SAM_LCDCA_CFG);
  ucontrast = (regval & LCDCA_CFG_FCST_MASK) >> LCDCA_CFG_FCST_SHIFT;

  /* Sign extend and translate the 6 bit signed value
   *
   * Unsigned   Signed Extended Translated
   * Value      Hex       Dec
   * ---------- --------- ----- -----------
   * 0000 001f  0000 001f 31     63
   * 0000 0000  0000 0000 0      32
   * 0000 0020  ffff ffe0 -32    0
   */

  scontrast   = (int32_t)(ucontrast << (32 - 6));
  scontrast >>= (32 - 6);
  return scontrast + 32;
}

/****************************************************************************
 * Name: slcd_setcontrast
 ****************************************************************************/

static int slcd_setcontrast(unsigned int contrast)
{
  uint32_t regval;
  int scontrast;
  int ret = OK;

  /* Make sure that the contrast setting is within range */

  if (contrast > SLCD_MAXCONTRAST)
    {
      contrast = SLCD_MAXCONTRAST;
      ret = -ERANGE;
    }

  /* Translate to get a signed value:
   *
   * Input  Translated Value Masked Value
   *        Dec Hex
   * ------ --- ------------ -----------
   * 63 ->  31  0000 0001f   0000 0001f
   * 32 ->   0  0000 00000   0000 00000
   * 0  -> -32  ffff fffe0   0000 00020
   */

  scontrast = (int)contrast - 32;

  /* Set the new contrast value */

  regval  = getreg32(SAM_LCDCA_CFG);
  regval &= ~LCDCA_CFG_FCST_MASK;
  regval |= LCDCA_CFG_FCST(scontrast);
  putreg32(regval, SAM_LCDCA_CFG);

  lcdinfo("contrast: %d CFG: %08x\n", contrast, getreg32(SAM_LCDCA_CFG));
  return ret;
}

/****************************************************************************
 * Name: slcd_writech
 ****************************************************************************/

static void slcd_writech(uint8_t ch, uint8_t curpos, uint8_t options)
{
  uint8_t segment;

  /* "LCDCA handles up to four ASCII characters tables, configured in
   *  Character Mapping Configuration register (CMCFG). Instead of handling
   *  each segments in display memory for a selected digit, user writes ASCII
   *  code in Character Mapping Control Register (CMCR) to display the
   *  corresponding character.
   *
   * "User can then drive several digits with few operations:
   *
   * "1. Select the Type of Digit (CMCFG.TDG),
   * "2. Write the Start Segment value (CMCFG.STSEG) of the first digit,
   * "3. Select Digit Reverse Mode (CMCFG.DREV) if required. If DREV is one,
   *     segment index is decremented,
   * "4. Then write ASCII code in CMCR register."
   */

  segment = g_startseg[curpos];
  putreg32(LCDCA_CMCFG_TDG_14S4C | LCDCA_CMCFG_STSEG(segment),
           SAM_LCDCA_CMCFG);
  putreg32(ch, SAM_LCDCA_CMDR);

  /* Check if we need to decorate the character with a preceding dot. */

  if ((options & SLCD_DP) != 0)
    {
      slcd_setdp(curpos);
    }
  else
    {
      slcd_clrdp(curpos);
    }

  /* Save these values in the state structure */

  g_slcdstate.buffer[curpos]  = ch;
  g_slcdstate.options[curpos] = options;
  slcd_dumpstate("AFTER WRITE");
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

      case SLCDCODE_BACKDEL:  /* Backspace (backward delete) N characters */
        {
          int tmp;

          /* If we are at the home position or if the count is zero,
           * then ignore the action
           */

          if (g_slcdstate.curpos < 1 || count < 1)
            {
              break;
            }

          /* Otherwise, BACKDEL is like moving the cursor back N characters
           * then doing a forward deletion.
           * Decrement the cursor position and fall through.
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

      case SLCDCODE_FWDDEL:   /* DELete (forward delete) N characters moving text */
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

            /* Move all characters after the current cursor position left by
             * 'nmove' characters
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

      case SLCDCODE_ERASE:   /* Erase N characters from the cursor position */
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

            /* Erase N characters after the current cursor position left
             * by one
             */

            for (i = g_slcdstate.curpos; i < last; i++)
              {
                slcd_writech(' ', i, 0);
              }
          }
        break;

      case SLCDCODE_CLEAR:  /* Home the cursor and erase the entire display */
        {
          /* This is like HOME followed by ERASEEOL.
           * Home the cursor and fall through.
           */

          g_slcdstate.curpos = 0;
        }

      case SLCDCODE_ERASEEOL:  /* Erase from the cursor position to the end of line */
        {
          int i;

          /* Erase characters after the current cursor position to the end of
           * the line
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

static ssize_t slcd_read(struct file *filep,
                         char *buffer, size_t len)
{
  int ret = 0;
  int i;

  /* Try to read the entire display.  Notice that the seek offset
   * (filep->f_pos) is ignored.  It probably should be taken into account
   * and also updated after each read and write.
   */

  for (i = 0; i < SLCD_NCHARS && ret < len; i++)
    {
      /* Check if the character is decorated with a preceding period */

      if (ret < len && g_slcdstate.options[i] != 0)
        {
          if ((g_slcdstate.options[i] & SLCD_DP) != 0)
            {
              *buffer++ = '.';
              ret++;
            }
        }

      /* Return the character */

      *buffer++ = g_slcdstate.buffer[i];
      ret++;
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
  uint8_t options;

  /* Initialize the stream for use with the SLCD CODEC */

  lib_meminstream(&instream, buffer, len);

  /* Initialize the SLCD decode state buffer */

  memset(&state, 0, sizeof(struct slcdstate_s));

  /* Decode and process every byte in the input buffer */

  options = 0;
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
                  /* Perform the backward deletion */

                  slcd_action(SLCDCODE_BACKDEL, 1);
                }
              else if (ch == ASCII_CR)
                {
                  /* Perform the carriage return */

                  slcd_action(SLCDCODE_HOME, 0);
                }

              /* Ignore dots before control characters (all of them?) */

               options = 0;
            }

          /* Handle characters decoreated with a preceding period */

          else if (ch == '.')
            {
              /* The next character will need a dot in front of it */

              options |= SLCD_DP;
            }

          /* Handle ASCII_DEL */

          else if (ch == ASCII_DEL)
            {
              /* Perform the forward deletion */

              slcd_action(SLCDCODE_FWDDEL, 1);
              options = 0;
            }

          /* The rest of the 7-bit ASCII characters are fair game */

          else if (ch < 128)
            {
              lcdinfo("ch: %c[%02x] options: %02x\n", ch, ch, options);

              /* Write the character at the current cursor position */

              slcd_writech(ch, g_slcdstate.curpos, options);
              options = 0;

              /* And advance the cursor position */

              if (g_slcdstate.curpos < (SLCD_NCHARS - 1))
                {
                  g_slcdstate.curpos++;
                }

               slcd_dumpstate("AFTER APPEND");
            }
        }
      else /* (result == SLCDRET_SPEC) */  /* A special SLCD action was returned */
        {
          /* Then Perform the action */

          slcd_action((enum slcdcode_e)ch, count);
        }
    }

  /* Ignore any dots with no following characters */

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
       * argument:  Pointer to struct slcd_attributes_s in which values will
       *            be returned
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

          if ((arg & 1) != 0)
            {
              slcd_setpixel(SLCD_RNG0);
            }
          else
            {
              slcd_clrpixel(SLCD_RNG0);
            }

          if ((arg & 2) != 0)
            {
              slcd_setpixel(SLCD_RNG1);
            }
          else
            {
              slcd_clrpixel(SLCD_RNG1);
            }

          if ((arg & 4) != 0)
            {
              slcd_setpixel(SLCD_RNG2);
            }
          else
            {
              slcd_clrpixel(SLCD_RNG2);
            }

          if ((arg & 8) != 0)
            {
              slcd_clrpixel(SLCD_RNG3);
            }
          else
            {
              slcd_setpixel(SLCD_RNG3);
            }
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

          return slcd_setcontrast((unsigned int)arg);
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
 * Name: sam_slcd_initialize
 *
 * Description:
 *   Initialize the SAM4L Xplained Pro LCD hardware and register the
 *   character driver as /dev/slcd0.
 *
 ****************************************************************************/

int sam_slcd_initialize(void)
{
  uint32_t regval;
  int ret = OK;
  int i;

  /* Only initialize the driver once. */

  if (!g_slcdstate.initialized)
    {
      lcdinfo("Initializing\n");

      /* Configure LCD GPIO pins */

      for (i = 0; i < SLCD_NPINS; i++)
        {
          sam_configgpio(g_slcdgpio[i]);
        }

      /* Enable APB clock for LCDCA */

      sam_lcdca_enableclk();

      /* Here we require that either CONFIG_SAM34_OSC32K or
       * CONFIG_SAM34_RC32K is defined in the configuration.
       * In that case, the source clock was initialized during boot up
       * and we can be assured that it is read for use now.
       */

      /* Disable the LCD controller and frame counters */

      putreg32(LCDCA_CR_DIS | LCDCA_CR_FC0DIS | LCDCA_CR_FC1DIS |
               LCDCA_CR_FC2DIS, SAM_LCDCA_CR);

      /* Configure LCD controller timing */

      regval = BOARD_TIM_PRES | LCDCA_TIM_CLKDIV(BOARD_TIM_CLOCKDIV) |
               LCDCA_TIM_FC0(BOARD_TIM_FC0) | LCDCA_TIM_FC1(BOARD_TIM_FC1) |
               LCDCA_TIM_FC2(BOARD_TIM_FC2);
      putreg32(regval, SAM_LCDCA_TIM);

      /* Setup the LCD controller configuration.
       *
       * External bias generation (XBIAS):  Controlled by board setting
       * Waveform mode:  Controlled by board setting
       * Blank LCD:  No
       * Lock:  No
       * Duty: Controlled by board setting
       * Fine Contrast (FCST): 0
       * Number of segments (NSU): Controlled by board setting
       */

      regval =
#ifdef BOARD_XBIAS
               LCDCA_CFG_XBIAS |
#endif
#ifndef BOARD_LPWAVE
               LCDCA_CFG_WMOD |
#endif
               LCD_DUTY |
               LCDCA_CFG_FCST(0) |
               LCDCA_CFG_NSU(BOARD_SLCD_NSEG);

      putreg32(regval, SAM_LCDCA_CFG);

      /* Provide an initial contrast setting */

      slcd_setcontrast(BOARD_INITIAL_CONTRAST);

      /* Turn off blanking of display segments */

      regval = getreg32(SAM_LCDCA_CFG);
      regval &= ~LCDCA_CFG_BLANK;
      putreg32(regval, SAM_LCDCA_CFG);

      /* Enable the display controller */

      putreg32(LCDCA_CR_EN, SAM_LCDCA_CR);

      /* Clear all display memory */

      putreg32(LCDCA_CR_CDM, SAM_LCDCA_CR);

      /* Wait for the LCD to be fully enabled */

      while ((getreg32(SAM_LCDCA_SR) & LCDCA_SR_EN) == 0);

      /* Enable frame counters */

      putreg32(LCDCA_CR_FC0EN, SAM_LCDCA_CR);
      while ((getreg32(SAM_LCDCA_SR) & LCDCA_SR_FC0S) == 0);

      putreg32(LCDCA_CR_FC1EN, SAM_LCDCA_CR);
      while ((getreg32(SAM_LCDCA_SR) & LCDCA_SR_FC1S) == 0);

      putreg32(LCDCA_CR_FC2EN, SAM_LCDCA_CR);
      while ((getreg32(SAM_LCDCA_SR) & LCDCA_SR_FC2S) == 0);

      /* Make sure that blinking and circular shifting is off */

      putreg32(LCDCA_CR_BSTOP | LCDCA_CR_CSTOP, SAM_LCDCA_CR);

      /* Disable any automated display */

      regval = getreg32(SAM_LCDCA_ACMCFG);
      regval &= ~LCDCA_ACMCFG_EN;
      putreg32(regval, SAM_LCDCA_ACMCFG);

      /* Initialize display memory */

      putreg32(0, SAM_LCDCA_DRL0);
      putreg32(0, SAM_LCDCA_DRH0);
      putreg32(0, SAM_LCDCA_DRL1);
      putreg32(0, SAM_LCDCA_DRH1);
      putreg32(0, SAM_LCDCA_DRL2);
      putreg32(0, SAM_LCDCA_DRH2);
      putreg32(0, SAM_LCDCA_DRL3);
      putreg32(0, SAM_LCDCA_DRH3);

      /* Turn on the Atmel pixel */

      slcd_setpixel(SLCD_ATMEL);

      /* Register the LCD device driver */

      ret = register_driver("/dev/slcd0", &g_slcdops, 0644, &g_slcdstate);
      g_slcdstate.initialized = true;

      /* Turn on the backlight */

      sam_gpiowrite(GPIO_LCD1_BL, true);

      slcd_dumpstate("AFTER INITIALIZATION");
      slcd_dumpslcd("AFTER INITIALIZATION");
    }

  return ret;
}

#endif /* CONFIG_SAM34_LCDCA && CONFIG_SAM4L_XPLAINED_SLCD1MODULE */

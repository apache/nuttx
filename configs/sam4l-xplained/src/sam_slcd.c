/****************************************************************************
 * configs/sam4l-xlplained/src/sam_slcd.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/ascii.h>
#include <nuttx/streams.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lcd/slcd_ioctl.h>
#include <nuttx/lcd/slcd_codec.h>

#include "up_arch.h"
#include "sam_gpio.h"
#include "sam4l_periphclks.h"
#include "chip/sam_lcdca.h"

#include "sam4l-xplained.h"

#ifdef CONFIG_SAM34_LCDCA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Define CONFIG_DEBUG_LCD to enable detailed LCD debug output. Verbose debug
 * must also be enabled.
 */

#ifndef CONFIG_LIB_SLCDCODEC
#  error This SLCD driver requires CONFIG_LIB_SLCDCODEC
#endif

#if !defined(CONFIG_SAM34_OSC32K) && !defined(CONFIG_SAM34_RC32K)
#  error This SLCD driver requires that either CONFIG_SAM34_OSC32K or
#  error CONFIG_SAM34_RC32K be selected in the board configuration
#endif

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#  undef CONFIG_DEBUG_LCD
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LCD
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
#define SLCD_NCHARS            6
#define SLCD_MAXCONTRAST       63

#define BOARD_SLCD_NCOM        4
#define BOARD_SLCD_NSEG        40
#define SLCD_NPINS            (BOARD_SLCD_NCOM+BOARD_SLCD_NSEG+1)

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
#  define LCD_DUTY             LCDCA_CFG_DUTY_1TO2    /*  1/2 COM[0:1] */
#elif BOARD_SLCD_NCOM < 4
#  define LCD_DUTY             LCDCA_CFG_DUTY_1TO3    /* 1/3 COM[0:2] */
#elif BOARD_SLCD_NCOM < 5
#  define LCD_DUTY             LCDCA_CFG_DUTY_1TO4    /* 1/4 COM[0:3] */
#else
#  error Value of BOARD_SLCD_NCOM not supported
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg              dbg
#  define lcdvdbg             vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* SLCD incoming stream structure */

struct slcd_instream_s
{
  struct lib_instream_s stream;
  FAR const char *buffer;
  ssize_t nbytes;
};

/* Global SLCD state */

struct sam_slcdstate_s
{
  bool initialized;             /* True: Completed initialization sequence */
  uint8_t curpos;               /* The current cursor position */
  uint8_t buffer[SLCD_NCHARS];  /* SLCD ASCII content */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/
/* Debug */

#if defined(CONFIG_DEBUG_LCD) && defined(CONFIG_DEBUG_VERBOSE)
static void slcd_dumpstate(FAR const char *msg);
static void slcd_dumpslcd(FAR const char *msg);
#else
#  define slcd_dumpstate(msg)
#  define slcd_dumpslcd(msg)
#endif

/* Internal utilities */

static void slcd_clear(void);
static int slcd_getstream(FAR struct lib_instream_s *instream);
static uint8_t slcd_getcontrast(void);
static int slcd_setcontrast(int contrast);
static void slcd_writech(uint8_t ch, uint8_t curpos);
static void slcd_appendch(uint8_t ch);
static void slcd_action(enum slcdcode_e code, uint8_t count);

/* Character driver methods */

static ssize_t slcd_read(FAR struct file *, FAR char *, size_t);
static ssize_t slcd_write(FAR struct file *, FAR const char *, size_t);
static int slcd_ioctl(FAR struct file *filp, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int slcd_poll(FAR struct file *filp, FAR struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the driver state structure (there is no retained state information) */

static const struct file_operations g_slcdops =
{
  0,             /* open */
  0,             /* close */
  slcd_read,     /* read */
  slcd_write,    /* write */
  0,             /* seek */
  slcd_ioctl     /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , slcd_poll    /* poll */
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
  GPIO_LCDCA_SEG24, GPIO_LCDCA_SEG25, GPIO_LCDCA_SEG26, GPIO_LCDCA_SEG27,
  GPIO_LCDCA_SEG28, GPIO_LCDCA_SEG29, GPIO_LCDCA_SEG30,

  GPIO_LCD1_BL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slcd_dumpstate
 ****************************************************************************/

#if defined(CONFIG_DEBUG_LCD) && defined(CONFIG_DEBUG_VERBOSE)
static void slcd_dumpstate(FAR const char *msg)
{
  lcdvdbg("%s:\n", msg);
  lcdvdbg("  curpos: %d\n",
          g_slcdstate.curpos);
  lcdvdbg("  Display: [%c%c%c%c%c%c]\n",
          g_slcdstate.buffer[0], g_slcdstate.buffer[1], g_slcdstate.buffer[2],
          g_slcdstate.buffer[3], g_slcdstate.buffer[4], g_slcdstate.buffer[5]);
}
#endif

/****************************************************************************
 * Name: slcd_dumpslcd
 ****************************************************************************/

#if defined(CONFIG_DEBUG_LCD) && defined(CONFIG_DEBUG_VERBOSE)
static void slcd_dumpslcd(FAR const char *msg)
{
  lcdvdbg("%s:\n", msg);
  lcdvdbg("    CFG: %08x    TIM: %08x    SR: %08x\n",
          getreg32(SAM_LCDCA_CFG), getreg32(SAM_LCDCA_TIM),
          getreg32(SAM_LCDCA_SR));
  lcdvdbg("    DR0: %08x %08x           DR1: %08x %08x\n",
          getreg32(SAM_LCDCA_DRL0), getreg32(SAM_LCDCA_DRH0),
          getreg32(SAM_LCDCA_DRL1), getreg32(SAM_LCDCA_DRH1));
  lcdvdbg("    DR2: %08x %08x           DR3: %08x %08x\n",
          getreg32(SAM_LCDCA_DRL2), getreg32(SAM_LCDCA_DRH2),
          getreg32(SAM_LCDCA_DRL3), getreg32(SAM_LCDCA_DRH3));
  lcdvdbg("   BCFG: %08x CSRCFG: %08x CMCFG: %08x ACMCFG: %08x\n",
          getreg32(SAM_LCDCA_BCFG), getreg32(SAM_LCDCA_CSRCFG),
          getreg32(SAM_LCDCA_CMCFG), getreg32(SAM_LCDCA_ACMCFG));
  lcdvdbg(" ABMCFG: %08x    IMR: %08x   VER: %08x\n",
          getreg32(SAM_LCDCA_ABMCFG), getreg32(SAM_LCDCA_IMR),
          getreg32(SAM_LCDCA_VERSION));
}
#endif

/****************************************************************************
 * Name: slcd_clear
 ****************************************************************************/

static void slcd_clear(void)
{
  uint32_t regaddr;

  lvdbg("Clearing\n");
#warning Missing logic
}

/****************************************************************************
 * Name: slcd_getstream
 *
 * Description:
 *   Get one character from the keyboard.
 *
 ****************************************************************************/

static int slcd_getstream(FAR struct lib_instream_s *instream)
{
  FAR struct slcd_instream_s *slcdstream = (FAR struct slcd_instream_s *)instream;

  DEBUGASSERT(slcdstream && slcdstream->buffer);
  if (slcdstream->nbytes > 0)
    {
      slcdstream->nbytes--;
      slcdstream->stream.nget++;
      return (int)*slcdstream->buffer++;
    }

  return EOF;
}

/****************************************************************************
 * Name: slcd_getcontrast
 ****************************************************************************/

static uint8_t slcd_getcontrast(void)
{
  uint32_t regval;
  uint32_t ucontrast;
  int32_t  scontrast;

  /* Get the current contast value */

  regval    = getreg32(SAM_LCDCA_CFG);
  ucontrast = (regval & LCDCA_CFG_FCST_MASK) >> LCDCA_CFG_FCST_MASK;

  /* Sign extend and translate the 6 bit signed value
   *
   * Unsigned   Signed Extended Translated
   * Value      Hex       Dec
   * ---------- --------- ----- -----------
   * 0000 001f  0000 001f 31     63
   * 0000 0000  0000 0000 0      32
   * 0000 0020  ffff ffe0 -32    0
   */

  scontrast   = (int32_t)(ucontrast << (32-6));
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

  /* Set the new contast value */

  regval  = getreg32(SAM_LCDCA_CFG);
  regval &= ~LCDCA_CFG_FCST_MASK;
  regval |= LCDCA_CFG_FCST(scontrast);
  putreg32(regval, SAM_LCDCA_CFG);

  lcdvdbg("contrast: %d CFG: %08x\n", contrast, getreg32(SAM_LCDCA_CFG));
  return ret;
}

/****************************************************************************
 * Name: slcd_writech
 ****************************************************************************/

static void slcd_writech(uint8_t ch, uint8_t curpos)
{
  uint16_t segset;

  /* Get a set describing the segment settings */

#warning Missing logic

  /* Decode the value and write it to the SLCD segment memory */

  /* Save these values in the state structure */

  g_slcdstate.buffer[curpos]  = ch;
  slcd_dumpstate("AFTER WRITE");
}

/****************************************************************************
 * Name: slcd_appendch
 ****************************************************************************/

static void slcd_appendch(uint8_t ch, uint8)
{
  lcdvdbg("ch: %c[%02x]\n", isprint(ch) ? ch : '.', ch);

  /* Write the character at the current cursor position */

  slcd_writech(ch, g_slcdstate.curpos);
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
  lcdvdbg("Action: %d count: %d\n", code, count);
  slcd_dumpstate("BEFORE ACTION");

  switch (code)
    {
      /* Erasure */

      case SLCDCODE_BACKDEL:         /* Backspace (backward delete) N characters */
        {
          int tmp;

          /* If we are at the home position or if the count is zero, then ignore the action */

          if (g_slcdstate.curpos < 1 || count < 1)
            {
              break;
            }

          /* Otherwise, BACKDEL is like moving the cursor back N characters then doing a
           * forward deletion.  Decrement the cursor position and fall through.
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

            /* Move all characters after the current cursor position left by 'nmove' characters */

            for (i = g_slcdstate.curpos + nmove; i < SLCD_NCHARS - 1; i++)
              {
                slcd_writech(g_slcdstate.buffer[i-nmove], i);
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

            /* Erase N characters after the current cursor position left by one */

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

          /* Erase characters after the current cursor position to the end of the line */

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

static ssize_t slcd_read(FAR struct file *filp, FAR char *buffer, size_t len)
{
  int ret = 0;
  int i;

  /* Try to read the entire display.  Notice that the seek offset
   * (filp->f_pos) is ignored.  It probably should be taken into account
   * and also updated after each read and write.
   */

  for (i = 0; i < SLCD_NCHARS && ret < len; i++)
    {
      /* Return the character */

      *buffer++ = g_slcdstate.buffer[i];
      ret++;

      /* Check if the character is decorated with a folling period or colon */

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

static ssize_t slcd_write(FAR struct file *filp,
                          FAR const char *buffer, size_t len)
{
  struct slcd_instream_s instream;
  struct slcdstate_s state;
  enum slcdret_e result;
  uint8_t ch;
  uint8_t count;
  uint8_t prev = ' ';
  bool valid = false;

  /* Initialize the stream for use with the SLCD CODEC */

  instream.stream.get  = slcd_getstream;
  instream.stream.nget = 0;
  instream.buffer      = buffer;
  instream.nbytes      = len;

  /* Prime the pump.  This is messy, but necessary to handle decoration on a
   * character based on any following period or colon.
   */

  memset(&state, 0, sizeof(struct slcdstate_s));
  result = slcd_decode(&instream.stream, &state, &prev, &count);

  lcdvdbg("slcd_decode returned result=%d char=%d count=%d\n",
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

  while ((result = slcd_decode(&instream.stream, &state, &ch, &count)) != SLCDRET_EOF)
    {
      lcdvdbg("slcd_decode returned result=%d char=%d count=%d\n",
              result, ch, count);

      if (result == SLCDRET_CHAR)          /* A normal character was returned */
        {
          /* Check for ASCII control characters */

          if (ch < ASCII_SPACE)
            {
              /* All are ignored except for backspace and carriage return */

              if (ch == ASCII_BS)
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

                  /* Then perform the backward deletion */

                  slcd_action(SLCDCODE_BACKDEL, 1);
                }
              else if (ch == ASCII_CR)
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

                  /* Then perform the carriage return */

                  slcd_action(SLCDCODE_HOME, 0);
                }
            }

          /* Handle characters decoreated with a period or a colon */

          else if (ch == '.')
            {
              /* Write the previous character with the decimal point appended */

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

              /* Then perform the foward deletion */

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

static int slcd_ioctl(FAR struct file *filp, int cmd, unsigned long arg)
{
  switch (cmd)
    {

      /* SLCDIOC_GETATTRIBUTES:  Get the attributes of the SLCD
       *
       * argument:  Pointer to struct slcd_attributes_s in which values will be
       *            returned
       */

      case SLCDIOC_GETATTRIBUTES:
        {
          FAR struct slcd_attributes_s *attr = (FAR struct slcd_attributes_s *)((uintptr_t)arg);

          lcdvdbg("SLCDIOC_GETATTRIBUTES:\n");

          if (!attr)
            {
              return -EINVAL;
            }

          attr->nrows         = SLCD_NROWS;
          attr->ncolumns      = SLCD_NCHARS;
          attr->nbars         = 0;
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
          FAR struct slcd_curpos_s *curpos = (FAR struct slcd_curpos_s *)((uintptr_t)arg);

          lcdvdbg("SLCDIOC_CURPOS: row=0 column=%d\n", g_slcdstate.curpos);

          if (!curpos)
            {
              return -EINVAL;
            }

          curpos->row    = 0;
          curpos->column = g_slcdstate.curpos;
        }
        break;

      /* SLCDIOC_GETCONTRAST: Get the current contrast setting
       *
       * argument:  Pointer type int that will receive the current contrast
       *            setting
       */

      case SLCDIOC_GETCONTRAST:
        {
          FAR int *contrast = (FAR int *)((uintptr_t)arg);
          if (!contrast)
            {
              return -EINVAL;
            }

          *contrast = (int)slcd_getcontrast();
          lcdvdbg("SLCDIOC_GETCONTRAST: contrast=%d\n", *contrast);
        }
        break;

      /* SLCDIOC_SETCONTRAST: Set the contrast to a new value
       *
       * argument:  The new contrast value
       */

      case SLCDIOC_SETCONTRAST:
        {
          lcdvdbg("SLCDIOC_SETCONTRAST: arg=%ld\n", arg);

          if (arg > SLCD_MAXCONTRAST)
            {
              return -ERANGE;
            }

          return slcd_setcontrast((int)arg);
        }
        break;

      case SLCDIOC_SETBAR:         /* Get bar levels */
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

#ifndef CONFIG_DISABLE_POLL
static int slcd_poll(FAR struct file *filp, FAR struct pollfd *fds,
                        bool setup)
{
  if (setup)
    {
      /* Data is always avaialble to be read / Data can always be written */

      fds->revents |= (fds->events & (POLLIN|POLLOUT));
      if (fds->revents != 0)
        {
          sem_post(fds->sem);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_slcd_initialize
 *
 * Description:
 *   Initialize the SAM4L Xplained Pro LCD hardware and register the character
 *   driver as /dev/slcd.
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
      lcdvdbg("Initializing\n");

      /* Configure LCD GPIO pins */

      for (i = 0; i < SLCD_NPINS; i++)
        {
          sam_configgpio(g_slcdgpio[i]);
        }

      /* Enable APB clock for LCDCA */

      sam_lcdca_enableclk();

      /* Here we require that either CONFIG_SAM34_OSC32K or CONFIG_SAM34_RC32K
       * is defined in the configuration.  In that case, the source clock was
       * initialized during boot up and we can be assured that it is read for
       * use now.
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
               LCDCA_CFG_NSU(BOARD_SLCD_NSEG));

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

      putreg32(LCDCA_CR_BSTOP | , SAM_LCDCA_CR);
      putreg32(LCDCA_CR_CSTOP, SAM_LCDCA_CR);

      /* Disable any automated display */

      regval = getreg32(SAM_LCDCA_ACMCFG);
      regval &= ~LCDCA_ACMCFG_EN;
      putreg32(regval, SAM_LCDCA_ACMCFG);

      /* Initialize display memory */

      putreg32(LCDCA_DRL_MASK, SAM_LCDCA_DRL0);
      putreg32(LCDCA_DRH_MASK, SAM_LCDCA_DRH0);
      putreg32(LCDCA_DRL_MASK, SAM_LCDCA_DRL1);
      putreg32(LCDCA_DRH_MASK, SAM_LCDCA_DRH1);
      putreg32(LCDCA_DRL_MASK, SAM_LCDCA_DRL2);
      putreg32(LCDCA_DRH_MASK, SAM_LCDCA_DRH2);
      putreg32(LCDCA_DRL_MASK, SAM_LCDCA_DRL3);
      putreg32(LCDCA_DRH_MASK, SAM_LCDCA_DRH3);

      /* Register the LCD device driver */

      ret = register_driver("/dev/slcd", &g_slcdops, 0644, &g_slcdstate);
      g_slcdstate.initialized = true;

      /* Turn on the backlight */

      sam_gpiowrite(GPIO_LCD1_BL, true);
      slcd_dumpstate("AFTER INITIALIZATION");
    }

  return ret;
}

#endif /* CONFIG_SAM34_LCDCA */

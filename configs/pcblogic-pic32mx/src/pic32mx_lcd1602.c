/****************************************************************************
 * configs/pcblocic-pic32mx/src/pic32mx_lcd1602.c
 *
 * This logic supports the connection of an LCD1602 LCD to the PCB Logic
 * PIC32MX board.  The LCD1602 is based on the Hitachi HD44780U LCD
 * controller
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

/* LCD pin mapping (see configs/pcblogic-pic32mx/README.txt)
 *
 *  ----------------------------------- ---------- ----------------------------------
 *  PIC32                               LCD1602    PCBLogic PIN
 *  PIN  SIGNAL NAME                    PIN NAME(s)
 *  ----------------------------------- ---------- ----------------------------------
 *                                      1.  Vss    --> Powerpoint GND
 *                                      2.  Vdd    --> Powerpoint USB+5V
 *                                      3.  Vee    N/C To ground via 10K potentiometer
 *   44  AN15/OCFB/PMALL/PMA0/CN12/RB15 4.  RS       4 PMA0, Selects registers
 *   82  PMRD/CN14/RD5                  5.  RW      82 PMRD/PMWR, Selects read or write
 *   81  OC5/PMWR/CN13/RD4              6.  E       81 PMENB, Starts data read/write
 *   93  PMD0/RE0                       7.  D0      93 PMD0
 *   94  PMD1/RE1                       8.  D1      94 PMD1
 *   98  PMD2/RE2                       9.  D2      98 PMD2
 *   99  PMD3/RE3                       10. D3      99 PMD3
 *  100  PMD4/RE4                       11. D4     100 PMD4
 *    3  PMD5/RE5                       12. D5       3 PMD5
 *    4  PMD6/RE6                       13. D6       4 PMD6
 *    5  PMD7/RE7                       14. D7       5 PMD7
 *                                      15. A      N/C To Vcc (5V) via 10K potentiometer
 *                                      16. K      --> Powerpoint GND
 *  ----------------------------------- ---------- ----------------------------------
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <ctype.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/ascii.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lcd/hd4478ou.h>
#include <nuttx/lcd/slcd_ioctl.h>
#include <nuttx/lcd/slcd_codec.h>

#include "up_arch.h"
#include "pic32mx-pmp.h"
#include "pic32mx-int.h"
#include "pic32mx.h"
#include "pcblogic-pic32mx.h"

#ifdef CONFIG_LCD_LCD1602

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_PIC32MX_PMP
#  error "CONFIG_PIC32MX_PMP is required to use the LCD"
#endif

/* The ever-present MIN/MAX macros ******************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

/* Pin configuration ********************************************************/
/* RB15, RS -- High values selects data */

#define GPIO_LCD_RS   (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTB|GPIO_PIN15)

/* LCD **********************************************************************/

#define LCD_NROWS        2
#define LCD_NCOLUMNS     16
#define LCD_NCHARS       (LCD_NROWS * LCD_NCOLUMNS)

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* SLCD incoming stream structure */

struct lcd_instream_s
{
  struct lib_instream_s stream;
  FAR const char *buffer;
  ssize_t nbytes;
};

/* Global LCD state */

struct lcd1602_2
{
  bool initialized; /* True: Completed initialization sequence */
  uint8_t currow;   /* Current row */
  uint8_t curcol;   /* Current column */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/
/* Debug */

#ifdef CONFIG_DEBUG_LCD_INFO
static void lcd_dumpstate(FAR const char *msg);
static void lcd_dumpstream(FAR const char *msg,
                           FAR const struct lcd_instream_s *stream);
#else
#  define lcd_dumpstate(msg)
#  define lcd_dumpstream(msg, stream)
#endif

/* Internal functions */

static int lcd_getstream(FAR struct lib_instream_s *instream);
static void lcd_wrcommand(uint8_t cmd);
static void lcd_wrdata(uint8_t data);
static uint8_t lcd_rddata(void);
static uint8_t lcd_readch(uint8_t row, uint8_t column);
static void lcd_writech(uint8_t ch, uint8_t row, uint8_t column);
static void lcd_appendch(uint8_t ch);
static void lcd_action(enum slcdcode_e code, uint8_t count);

/* Character driver operations */

static ssize_t lcd_read(FAR struct file *, FAR char *, size_t);
static ssize_t lcd_write(FAR struct file *, FAR const char *, size_t);
static int lcd_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int lcd_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Character driver operations */

static const struct file_operations g_lcdops =
{
  0,             /* open */
  0,             /* close */
  lcd_read,      /* read */
  lcd_write,     /* write */
  0,             /* seek */
  lcd_ioctl      /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , lcd_poll     /* poll */
#endif
};

/* This is the driver state structure */

static struct lcd1602_2 g_lcd1602;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lcd_dumpstate
 ****************************************************************************/

#ifdef CONFIG_DEBUG_LCD_INFO
static void lcd_dumpstate(FAR const char *msg)
{
  uint8_t buffer[LCD_NCOLUMNS];
  uint8_t ch;
  int row;
  int column;

  lcdinfo("%s:\n", msg);
  lcdinfo("  currow: %d curcol: %d\n",
          g_lcd1602.currow, g_lcd1602.curcol);

  for (row = 0, column = 0; row < LCD_NROWS; )
    {
      ch = lcd_readch(row, column);
      buffer[column] = isprint(ch) ? ch : '.';
      if (++column >= LCD_NCOLUMNS)
        {
          lcdinfo("  [%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c]\n",
                  buffer[0],  buffer[1],  buffer[2],  buffer[3],
                  buffer[4],  buffer[5],  buffer[6],  buffer[7],
                  buffer[8],  buffer[9],  buffer[10], buffer[11],
                  buffer[12], buffer[13], buffer[14], buffer[15]);

          column = 0;
          row++;
        }
    }
}
#endif

/****************************************************************************
 * Name: lcd_dumpstate
 ****************************************************************************/

#ifdef CONFIG_DEBUG_LCD_INFO
static void lcd_dumpstream(FAR const char *msg,
                           FAR const struct lcd_instream_s *stream)
{
  lcdinfo("%s:\n", msg);
  lcdinfo("  nget: %d nbytes: %d\n",
          stream->stream.nget, stream->nbytes);
  lib_dumpbuffer("STREAM", stream->buffer, stream->nbytes);
}
#endif

/****************************************************************************
 * Name: lcd_getstream
 *
 * Description:
 *   Get one character from the keyboard.
 *
 ****************************************************************************/

static int lcd_getstream(FAR struct lib_instream_s *instream)
{
  FAR struct lcd_instream_s *lcdstream = (FAR struct lcd_instream_s *)instream;

  DEBUGASSERT(lcdstream && lcdstream->buffer);
  if (lcdstream->nbytes > 0)
    {
      lcdstream->nbytes--;
      lcdstream->stream.nget++;
      return (int)*lcdstream->buffer++;
    }

  return EOF;
}

/****************************************************************************
 * Name: lcd_wrcommand
 *
 * Description:
 *   Configure to write an LCD command
 *
 ****************************************************************************/

static void lcd_wrcommand(uint8_t cmd)
{
  /* Address bit A0 is RS.  Set the address latch to A0=0 */

  putreg32(1, PIC32MX_PMP_ADDRCLR);

  /* And write the command to the data out register */

  putreg32((uint32_t)cmd, PIC32MX_PMP_DOUT);
}

/****************************************************************************
 * Name: lcd_wrdata
 *
 * Description:
 *   Configure to read or write LCD data
 *
 ****************************************************************************/

static void lcd_wrdata(uint8_t data)
{
  /* Address bit A0 is RS.  Set the address latch to A0=1 */

  putreg32(1, PIC32MX_PMP_ADDRSET);

  /* And write the data to the data out register */

  putreg32((uint32_t)data, PIC32MX_PMP_DOUT);
}

/****************************************************************************
 * Name: lcd_rddata
 *
 * Description:
 *   Configure to read or write LCD data
 *
 ****************************************************************************/

static uint8_t lcd_rddata(void)
{
  /* Address bit A0 is RS.  Set the address latch to A0=1 */

  putreg32(1, PIC32MX_PMP_ADDRSET);

  /* And read the data to the data in register */

  return (uint8_t)getreg32(PIC32MX_PMP_DIN);
}

/****************************************************************************
 * Name: lcd_readch
 ****************************************************************************/

static uint8_t lcd_readch(uint8_t row, uint8_t column)
{
  uint8_t addr;

  /* Set the cursor position.  Internally, the HD44780U supports a display
   * size of up to 2x40 addressed as follows:
   *
   * Column  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 ... 39
   * Row 0  00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f ... 27
   * Ro1 1  40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f ... 67
  */

  addr = column;
  if (row > 0)
    {
      addr |= HD4478OU_DDRAM_ROW1;
    }

  lcd_wrcommand(HD4478OU_DDRAM_AD(addr));

  /* And write the character here */

  return lcd_rddata();
}

/****************************************************************************
 * Name: lcd_writech
 ****************************************************************************/

static void lcd_writech(uint8_t ch, uint8_t row, uint8_t column)
{
  uint8_t addr;

  /* Set the cursor position.  Internally, the HD44780U supports a display
   * size of up to 2x40 addressed as follows:
   *
   * Column  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 ... 39
   * Row 0  00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f ... 27
   * Ro1 1  40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f ... 67
  */

  addr = column;
  if (row > 0)
    {
      addr |= HD4478OU_DDRAM_ROW1;
    }

  lcd_wrcommand(HD4478OU_DDRAM_AD(addr));

  /* And write the character here */

  lcd_wrdata(ch);
}

/****************************************************************************
 * Name: lcd_appendch
 ****************************************************************************/

static void lcd_appendch(uint8_t ch)
{
  if (g_lcd1602.curcol < LCD_NCOLUMNS)
    {
      lcd_writech(ch, g_lcd1602.currow, g_lcd1602.curcol);
      g_lcd1602.curcol++;
    }
}

/****************************************************************************
 * Name: lcd_action
 ****************************************************************************/

static void lcd_action(enum slcdcode_e code, uint8_t count)
{
  lcdinfo("Action: %d count: %d\n", code, count);
  lcd_dumpstate("BEFORE ACTION");

  switch (code)
    {
      /* Erasure */

      case SLCDCODE_BACKDEL:         /* Backspace (backward delete) N characters */
        {
          int tmp;

          /* If we are at the home position or if the count is zero, then ignore the action */

          if (g_lcd1602.curcol < 1 || count < 1)
            {
              break;
            }

          /* Otherwise, BACKDEL is like moving the cursor back N characters then doing a
           * forward deletion.  Decrement the cursor position and fall through.
           */

           tmp = (int)g_lcd1602.curcol - count;
           if (tmp < 0)
             {
               tmp   = 0;
               count = g_lcd1602.curcol;
             }

           /* Save the updated cursor positions */

           g_lcd1602.curcol = tmp;
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

            nchars = LCD_NCOLUMNS - g_lcd1602.curcol;
            nmove  = MIN(nchars, count) - 1;

            /* Move all characters after the current cursor position left by 'nmove' characters */

            for (i = g_lcd1602.curcol + nmove; i < LCD_NCOLUMNS - 1; i++)
              {
                uint8_t ch = lcd_readch(g_lcd1602.currow, i);
                lcd_writech(ch, g_lcd1602.currow, i - nmove);
              }

            /* Erase the last 'nmove' characters on the display */

            for (i = LCD_NCOLUMNS - nmove; i < LCD_NCOLUMNS; i++)
              {
                lcd_writech(' ', i, 0);
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

            last = g_lcd1602.curcol + count - 1;
            if (last >= LCD_NCOLUMNS)
              {
                last = LCD_NCOLUMNS - 1;
              }

            /* Erase N characters after the current cursor position left by one */

            for (i = g_lcd1602.curcol; i < last; i++)
              {
                lcd_writech(' ', g_lcd1602.currow, i);
              }
          }
        break;

      case SLCDCODE_CLEAR:           /* Home the cursor and erase the entire display */
        {
          /* Clear the display */

          lcd_wrcommand(HD4478OU_CLEAR);

          /* And home the cursor */

          g_lcd1602.currow = 0;
          g_lcd1602.curcol = 0;
        }
        break;

      case SLCDCODE_ERASEEOL:        /* Erase from the cursor position to the end of line */
        {
          int i;

          /* Erase characters after the current cursor position to the end of the line */

          for (i = g_lcd1602.curcol; i < LCD_NCOLUMNS; i++)
            {
              lcd_writech(' ', g_lcd1602.currow, i);
            }
        }
        break;

      /* Cursor movement */

      case SLCDCODE_HOME:            /* Cursor home */
        {
          g_lcd1602.currow = 0;
          g_lcd1602.curcol = 0;
        }
        break;

      case SLCDCODE_END:             /* Cursor end */
        {
          g_lcd1602.curcol = LCD_NCOLUMNS - 1;
        }
        break;

      case SLCDCODE_LEFT:            /* Cursor left by N characters */
        {
          int tmp = (int)g_lcd1602.curcol - count;

          /* Don't permit movement past the beginning of the SLCD */

          if (tmp < 0)
            {
              tmp = 0;
            }

          /* Save the new cursor position */

          g_lcd1602.curcol = (uint8_t)tmp;
        }
        break;

      case SLCDCODE_RIGHT:           /* Cursor right by N characters */
        {
          int tmp = (int)g_lcd1602.curcol + count;

          /* Don't permit movement past the end of the SLCD */

          if (tmp >= LCD_NCOLUMNS)
            {
              tmp = LCD_NCOLUMNS - 1;
            }

          /* Save the new cursor position */

          g_lcd1602.curcol = (uint8_t)tmp;
        }
        break;

      case SLCDCODE_UP:              /* Cursor up by N lines */
        {
          int tmp = (int)g_lcd1602.currow - count;

          /* Don't permit movement past the top of the SLCD */

          if (tmp < 0)
            {
              tmp = 0;
            }

          /* Save the new cursor position */

          g_lcd1602.currow = (uint8_t)tmp;
        }
        break;

      case SLCDCODE_DOWN:            /* Cursor down by N lines */
        {
          int tmp = (int)g_lcd1602.currow + count;

          /* Don't permit movement past the bottom of the SLCD */

          if (tmp >= LCD_NROWS)
            {
              tmp = LCD_NROWS - 1;
            }

          /* Save the new cursor position */

          g_lcd1602.currow = (uint8_t)tmp;
        }
        break;

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

  lcd_dumpstate("AFTER ACTION");
}

/****************************************************************************
 * Name: lcd_read
 ****************************************************************************/

static ssize_t lcd_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  uint8_t row;
  uint8_t column;
  int nread;

  /* Try to read the entire display.  Notice that the seek offset
   * (filep->f_pos) is ignored.  It probably should be taken into account
   * and also updated after each read and write.
   */

  row    = 0;
  column = 0;

  for (nread = 0; nread < len; nread++)
    {
      *buffer++ = lcd_readch(row, column);
      if (++column >= LCD_NCOLUMNS)
        {
          column = 0;
          if (++row >= LCD_NROWS)
            {
              break;
            }
        }
    }

  return nread;
}

/****************************************************************************
 * Name: lcd_write
 ****************************************************************************/

static ssize_t lcd_write(FAR struct file *filep,  FAR const char *buffer,
                         size_t len)
{
  struct lcd_instream_s instream;
  struct slcdstate_s state;
  enum slcdret_e result;
  uint8_t ch;
  uint8_t count;

  /* Initialize the stream for use with the SLCD CODEC */

  instream.stream.get  = lcd_getstream;
  instream.stream.nget = 0;
  instream.buffer      = buffer;
  instream.nbytes      = len;

  lcd_dumpstream("BEFORE WRITE", &instream);

  /* Now decode and process every byte in the input buffer */

  memset(&state, 0, sizeof(struct slcdstate_s));
  while ((result = slcd_decode(&instream.stream, &state, &ch, &count)) != SLCDRET_EOF)
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

                  lcd_action(SLCDCODE_BACKDEL, 1);
                }
              else if (ch == ASCII_CR)
                {
                  /* Perform the carriage return */

                  g_lcd1602.curcol = 0;
                  lcd_action(SLCDCODE_DOWN, 1);
                }
            }

          /* Handle ASCII_DEL */

          else if (ch == ASCII_DEL)
            {
              /* Perform the forward deletion */

              lcd_action(SLCDCODE_FWDDEL, 1);
            }

          /* The rest of the 7-bit ASCII characters are fair game */

          else if (ch < 128)
            {
              /* Write the character if it valid */

              lcd_appendch(ch);
            }
        }
      else /* (result == SLCDRET_SPEC) */  /* A special SLCD action was returned */
        {
          /* Then Perform the action */

          lcd_action((enum slcdcode_e)ch, count);
        }
    }

  /* Assume that the entire input buffer was processed */

  lcd_dumpstream("AFTER WRITE", &instream);
  return (ssize_t)len;
}

/****************************************************************************
 * Name: lcd_ioctl
 ****************************************************************************/

static int lcd_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
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

          lcdinfo("SLCDIOC_GETATTRIBUTES:\n");

          if (!attr)
            {
              return -EINVAL;
            }

          attr->nrows         = LCD_NROWS;
          attr->ncolumns      = LCD_NCOLUMNS;
          attr->nbars         = 0;
          attr->maxcontrast   = 0;
          attr->maxbrightness = 0
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

          lcdinfo("SLCDIOC_CURPOS: row=%d column=%d\n", g_lcd1602.currow, g_lcd1602.curcol);

          if (!curpos)
            {
              return -EINVAL;
            }

          curpos->row    = g_lcd1602.currow;
          curpos->column = g_lcd1602.curcol;
        }
        break;

      case SLCDIOC_SETBAR:         /* SLCDIOC_SETBAR: Set bars on a bar display */
      case SLCDIOC_GETCONTRAST:    /* SLCDIOC_GETCONTRAST: Get the current contrast setting */
      case SLCDIOC_SETCONTRAST:    /* SLCDIOC_SETCONTRAST: Set the contrast to a new value */
      case SLCDIOC_GETBRIGHTNESS:  /* Get the current brightness setting */
      case SLCDIOC_SETBRIGHTNESS:  /* Set the brightness to a new value */
      default:
        return -ENOTTY;
    }

  return OK;
}

/****************************************************************************
 * Name: lcd_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int lcd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  if (setup)
    {
      /* Data is always avaialble to be read */

      fds->revents |= (fds->events & (POLLIN|POLLOUT));
      if (fds->revents != 0)
        {
          nxsem_post(fds->sem);
        }
    }
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  up_lcd1602_initialize
 *
 * Description:
 *   Initialize the LCD1602 hardware and register the character driver as
 *   /dev/lcd1602.  Prototype is in include/nuttx/lcd/hd4478ou.h.
 *
 ****************************************************************************/

int up_lcd1602_initialize(void)
{
  uint32_t regval;
  int ret = OK;

  /* Only initialize the driver once. */

  if (!g_lcd1602.initialized)
    {
      lcdinfo("Initializing\n");

      /* PMP Master mode configuration */
      /* Make sure that interrupts are disabled */

      putreg32(INT_PMP, PIC32MX_INT_IEC1CLR);

      /* Stop and reset the PMP module and clear the mode and control registers. */

      putreg32(0, PIC32MX_PMP_MODE);
      putreg32(0, PIC32MX_PMP_AEN);
      putreg32(0, PIC32MX_PMP_CON);
      putreg32(0, PIC32MX_PMP_ADDR);

      /* Set LCD timing values, PMP master mode 3, 8-bit mode, no address
       * increment, and no interrupts.
       */

      regval = (PMP_MODE_WAITE_RD(0) | PMP_MODE_WAITM(3) | PMP_MODE_WAITB_1TPB |
                PMP_MODE_MODE_MODE1 | PMP_MODE_MODE8 | PMP_MODE_INCM_NONE |
                PMP_MODE_IRQM_NONE);
      putreg32(regval, PIC32MX_PMP_MODE);

      /* Enable the PMP for reading and writing
       *   PMRD/PMWR is active high (1=RD; 0=WR)
       *   PMENB is active high.
       *   No chip selects
       *   Address latch is active high
       *   Enable PMRD/PMWR, PMENB, and the PMP.
       */


      regval = (PMP_CON_RDSP | PMP_CON_WRSP | PMP_CON_ALP |
                PMP_CON_CSF_ADDR1415 | PMP_CON_PTRDEN | PMP_CON_PTWREN |
                PMP_CON_ADRMUX_NONE | PMP_CON_ON);
      putreg32(regval, PIC32MX_PMP_CON);

      /* Configure and enable the LCD */
      /* Wait > 15 milliseconds afer Vdd > 4.5V */

      up_mdelay(100);

      /* Select the 8-bit interface. BF cannot be checked before this command.
       * This needs to be done a few times with some magic delays.
       */

      lcd_wrcommand(HD4478OU_FUNC | HD4478OU_FUNC_DL8D | HD4478OU_FUNC_N1);
      up_mdelay(50);
      lcd_wrcommand(HD4478OU_FUNC | HD4478OU_FUNC_DL8D | HD4478OU_FUNC_N1);
      up_udelay(50);
      lcd_wrcommand(HD4478OU_FUNC | HD4478OU_FUNC_DL8D | HD4478OU_FUNC_N1);
      lcd_wrcommand(HD4478OU_FUNC | HD4478OU_FUNC_DL8D | HD4478OU_FUNC_N1);

      /* Configure the display */

      lcd_wrcommand(HD4478OU_DISPLAY);                       /* Display, cursor, and blink off */
      lcd_wrcommand(HD4478OU_CLEAR);                         /* Clear the display */
      lcd_wrcommand(HD4478OU_INPUT | HD4478OU_INPUT_INCR);   /* Increment mode */
      lcd_wrcommand(HD4478OU_DISPLAY | HD4478OU_DISPLAY_ON); /* Display on, cursor and blink off */
      lcd_wrcommand(HD4478OU_DDRAM_AD(0));                   /* Select DDRAM RAM AD=0 */

      /* Register the LCD device driver */

      ret = register_driver("/dev/lcd1602", &g_lcdops, 0644, &g_lcd1602);
      g_lcd1602.initialized = true;
    }

  return ret;
}

#endif /* CONFIG_LCD_LCD1602 */

/****************************************************************************
 * boards/mips/pic32mx/sure-pic32mx/src/pic32mx_lcd1602.c
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

/* This logic supports the connection of an LCD1602 LCD to the PCB Logic
 * PIC32MX board.  The LCD1602 is based on the Hitachi HD44780U LCD
 * controller
 */

/* LCD pin mapping (see boards/sure-pic32mx/README.txt)
 *
 *  --------------------- ---------- ----------------------------------
 *  PIC32                  Sure JP1   Sure Signal Description
 *  PIN  SIGNAL NAME      PIN NAME(s)
 *  --------------------- ---------- ----------------------------------
 *   34  Vbus             1.  +5V    +5V VBUS device mode
 *                                    To GND via capacitor
 *                        2.  GND    GND
 *   49  RD1              3.  Vo     Transistor circuit driven by PWM2
 *   44  PMA0/AN15/RB15   4.  RS     PMA0, Selects registers
 *   53  PMRD/RD5         5.  RW     PMRD/PMWR, Selects read or write
 *   45  PMPCS1/RD11      6.  E      Starts data read/write
 *   60  PMD0/RE0         7.  DB0    PMD0
 *   61  PMD1/RE1         8.  DB1    PMD1
 *   62  PMD2/RE2         9.  DB2    PMD2
 *   63  PMD3/RE3         10. DB3    PMD3
 *   64  PMD4/RE4         11. DB4    PMD4
 *    1  PMD5/RE5         12. DB5    PMD5
 *    2  PMD6/RE6         13. DB6    PMD6
 *    3  PMD7/RE7         14. DB7    PMD7
 *                        15. A      +5V_DUSB
 *   46 INT0/RD0          16. K      Transistor circuit driven by PWM1
 *  --------------------- ---------- ----------------------------------
 *
 *  Vbus power also requires Vbuson/AN5/RB5
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/ascii.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lcd/hd4478ou.h>
#include <nuttx/lcd/slcd_ioctl.h>
#include <nuttx/lcd/slcd_codec.h>
#include <nuttx/semaphore.h>

#include "mips_internal.h"
#include "pic32mx_ioport.h"
#include "pic32mx_int.h"
#include "pic32mx.h"
#include "sure-pic32mx.h"

#ifdef CONFIG_LCD_LCD1602

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_LCD_MAXCONTRAST
#  define CONFIG_LCD_MAXCONTRAST 100
#endif

#ifndef CONFIG_LCD_MAXPOWER
#  define CONFIG_LCD_MAXPOWER 100
#endif

/* The ever-present MIN/MAX macros ******************************************/

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

/* LCD **********************************************************************/

#define LCD_NROWS        2
#define LCD_NCOLUMNS     16
#define LCD_NCHARS       (LCD_NROWS * LCD_NCOLUMNS)

#define NOP              __asm__ __volatile__ ("nop");

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* Global LCD state */

struct lcd1602_2
{
  bool initialized;   /* True: Completed initialization sequence */
  uint8_t currow;     /* Current row */
  uint8_t curcol;     /* Current column */
  uint8_t brightness; /* Current brightness */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* Debug */

#ifdef CONFIG_DEBUG_LCD_INFO
static void lcd_dumpstate(const char *msg);
static void lcd_dumpstream(const char *msg,
                           const struct lib_meminstream_s *stream);
#else
#  define lcd_dumpstate(msg)
#  define lcd_dumpstream(msg, stream)
#endif

/* Internal functions */

static int lcd_getstream(struct lib_instream_s *instream);
static void lcd_brightness(uint8_t brightness);
static void lcd_shortdelay(int delay);
static void lcd_wrcommand(uint8_t cmd);
static void lcd_wrdata(uint8_t data);
static uint8_t lcd_rddata(void);
static uint8_t lcd_readstatus(void);
static void lcd_waitbusy(void);
static uint8_t lcd_readch(uint8_t row, uint8_t column);
static void lcd_writech(uint8_t ch, uint8_t row, uint8_t column);
static void lcd_appendch(uint8_t ch);
static void lcd_action(enum slcdcode_e code, uint8_t count);

/* Character driver operations */

static ssize_t lcd_read(struct file *, char *, size_t);
static ssize_t lcd_write(struct file *, const char *, size_t);
static int lcd_ioctl(struct file *filep, int cmd, unsigned long arg);
static int lcd_poll(struct file *filep, struct pollfd *fds,
                    bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Character driver operations */

static const struct file_operations g_lcdops =
{
  NULL,          /* open */
  NULL,          /* close */
  lcd_read,      /* read */
  lcd_write,     /* write */
  NULL,          /* seek */
  lcd_ioctl,     /* ioctl */
  lcd_poll       /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
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
static void lcd_dumpstate(const char *msg)
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
static void lcd_dumpstream(const char *msg,
                           const struct lib_meminstream_s *stream)
{
  lcdinfo("%s:\n", msg);
  lcdinfo("  nget: %d nbytes: %d\n",
          stream->public.nget, stream->buflen);
  lib_dumpbuffer("STREAM", stream->buffer, stream->buflen);
}
#endif

/****************************************************************************
 * Name: lcd_brightness
 *
 * Description:
 *   Enable for disable LCD lighting.
 *
 ****************************************************************************/

static void lcd_brightness(uint8_t brightness)
{
  /* The LIGHT and COMP pins are label PWM1 and PWM2 and so are obviously
   * intended to support modulated outputs.  However, here for simplicity,
   * they are just treated as on/off discretes outputs.
   */

  if (brightness > 0)
    {
      /* Turn the LCD light on */

      pic32mx_gpiowrite(GPIO_LCD_LIGHT, true);
      NOP; NOP; NOP;
      pic32mx_gpiowrite(GPIO_LCD_COMP, true);
      NOP; NOP;
      pic32mx_gpiowrite(GPIO_LCD_PWR, true);
    }
  else
    {
      /* Turn the LCD light off */

      pic32mx_gpiowrite(GPIO_LCD_PWR, false);
      pic32mx_gpiowrite(GPIO_LCD_COMP, false);
      pic32mx_gpiowrite(GPIO_LCD_LIGHT, false);
    }

  g_lcd1602.brightness = brightness;
}

/****************************************************************************
 * Name: lcd_shortdelay
 *
 * Description:
 *   Small delays are needed to make some of the LCD operations work.
 *
 ****************************************************************************/

static void lcd_shortdelay(int delay)
{
  volatile int loop;

  /* On a 32MHz MCU, this should amount to about 300NS per loop */

  while (delay-- > 0)
    {
      for (loop = 0; loop < 1; loop++)
        {
          NOP;
        }
    }
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
  /* Make sure that the LCD is available */

  lcd_waitbusy();

  /* Select DB0-15 as outputs (only DB-0-7 are actually used) */

  putreg16(0, PIC32MX_IOPORTE_TRIS);

  /* Set up to write the command */

  pic32mx_gpiowrite(GPIO_LCD_RS, false); /* Select command */
  pic32mx_gpiowrite(GPIO_LCD_RW, false); /* Select write */
  lcd_shortdelay(2);

  pic32mx_gpiowrite(GPIO_LCD_E, true);   /* Enable transfer */
  lcd_shortdelay(1);

  /* Write the command to the LCD */

  putreg16(cmd, PIC32MX_IOPORTE_PORT);
  lcd_shortdelay(1);
  pic32mx_gpiowrite(GPIO_LCD_E, false);
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
  /* Make sure that the LCD is available */

  lcd_waitbusy();

  /* Select DB0-15 as outputs (only DB-0-7 are actually used) */

  putreg16(0, PIC32MX_IOPORTE_TRIS);

  /* Set up to write the data */

  pic32mx_gpiowrite(GPIO_LCD_RS, true);    /* Select data */
  pic32mx_gpiowrite(GPIO_LCD_RW, false);   /* Select write */
  lcd_shortdelay(2);

  pic32mx_gpiowrite(GPIO_LCD_E, true);     /* Enable transfer */
  lcd_shortdelay(1);

  /* Write the data to the LCD */

  putreg16(data, PIC32MX_IOPORTE_PORT);    /* Write the data */
  lcd_shortdelay(1);
  pic32mx_gpiowrite(GPIO_LCD_E, false);
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
  /* Make sure that the LCD is available */

  lcd_waitbusy();

  /* Setup to read data */

  pic32mx_gpiowrite(GPIO_LCD_RS, true);    /* Select data */
  pic32mx_gpiowrite(GPIO_LCD_RW, true);    /* Select read */
  lcd_shortdelay(2);

  pic32mx_gpiowrite(GPIO_LCD_E, true);     /* Enable transfer */
  lcd_shortdelay(1);

  putreg16(0xff, PIC32MX_IOPORTE_TRISSET); /* Set DB0-7 as inputs */
  pic32mx_gpiowrite(GPIO_LCD_E, false);    /* Disable transfer */

  /* Read the data from the LCD */

  return (uint8_t)getreg16(PIC32MX_IOPORTE_PORT);
}

/****************************************************************************
 * Name: lcd_readstatus
 *
 * Description:
 *   Read the DDRAM address and busy bit.
 *
 ****************************************************************************/

static uint8_t lcd_readstatus(void)
{
  uint8_t status;

  /* Set up to read BUSY/AD information */

  putreg16(0xff, PIC32MX_IOPORTE_TRISSET); /* Set DB0-7 as inputs */
  pic32mx_gpiowrite(GPIO_LCD_RS, false);   /* Select command */
  pic32mx_gpiowrite(GPIO_LCD_RW, true);    /* Select read */
  lcd_shortdelay(2);

  pic32mx_gpiowrite(GPIO_LCD_E, true);     /* Enable transfer */
  lcd_shortdelay(1);

  /* Read the status from the LCD */

  status = (uint8_t)getreg16(PIC32MX_IOPORTE_PORT);
  lcd_shortdelay(1);
  pic32mx_gpiowrite(GPIO_LCD_E, false);

  return status;
}

/****************************************************************************
 * Name: lcd_waitbusy
 *
 * Description:
 *   Check LCD status and wait until the BUSY flag is no long set.
 *
 ****************************************************************************/

static void lcd_waitbusy(void)
{
  while ((lcd_readstatus() & HD4478OU_BF) != 0);
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

          /* If we are at the home position or if the count is zero,
           * then ignore the action
           */

          if (g_lcd1602.curcol < 1 || count < 1)
            {
              break;
            }

           /* Otherwise, BACKDEL is like moving the cursor back N characters
            * then doing a forward deletion.
            * Decrement the cursor position and fall through.
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

            /* Move all characters after the current cursor position left by
             * 'nmove' characters
             */

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

            /* Erase N characters after the current cursor position left by
             * one
             */

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

          /* Erase characters after the current cursor position to the end of
           * the line
           */

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

static ssize_t lcd_read(struct file *filep, char *buffer, size_t len)
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

static ssize_t lcd_write(struct file *filep,  const char *buffer,
                         size_t len)
{
  struct lib_meminstream_s instream;
  struct slcdstate_s state;
  enum slcdret_e result;
  uint8_t ch;
  uint8_t count;

  /* Initialize the stream for use with the SLCD CODEC */

  lib_meminstream(&instream, buffer, len);
  lcd_dumpstream("BEFORE WRITE", &instream);

  /* Now decode and process every byte in the input buffer */

  memset(&state, 0, sizeof(struct slcdstate_s));
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

static int lcd_ioctl(struct file *filep, int cmd, unsigned long arg)
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

          attr->nrows         = LCD_NROWS;
          attr->ncolumns      = LCD_NCOLUMNS;
          attr->nbars         = 0;
          attr->maxcontrast   = CONFIG_LCD_MAXCONTRAST;
          attr->maxbrightness = CONFIG_LCD_MAXPOWER;
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

          lcdinfo("SLCDIOC_CURPOS: row=%d column=%d\n",
                   g_lcd1602.currow, g_lcd1602.curcol);

          if (!curpos)
            {
              return -EINVAL;
            }

          curpos->row    = g_lcd1602.currow;
          curpos->column = g_lcd1602.curcol;
        }
        break;

      /* SLCDIOC_GETBRIGHTNESS: Get the current brightness setting
       *
       * argument:  Pointer type int that will receive the current brightness
       *            setting
       */

      case SLCDIOC_GETBRIGHTNESS:
        {
          int *brightness = (int *)((uintptr_t)arg);
          if (!brightness)
            {
              return -EINVAL;
            }

          *brightness = (int)g_lcd1602.brightness;
          lcdinfo("SLCDIOC_GETCONTRAST: brightness=%d\n", *brightness);
        }
        break;

      /* SLCDIOC_SETBRIGHTNESS: Set the brightness to a new value
       *
       * argument:  The new brightness value
       */

      case SLCDIOC_SETBRIGHTNESS:
        {
          lcdinfo("SLCDIOC_SETCONTRAST: arg=%ld\n", arg);

          if (arg > CONFIG_LCD_MAXPOWER)
            {
              return -ERANGE;
            }

          lcd_brightness((uint8_t)arg);
        }
        break;

      case SLCDIOC_SETBAR:         /* SLCDIOC_SETBAR: Set bars on a bar display */
      case SLCDIOC_GETCONTRAST:    /* SLCDIOC_GETCONTRAST: Get the current contrast setting */
      case SLCDIOC_SETCONTRAST:    /* SLCDIOC_SETCONTRAST: Set the contrast to a new value */
      default:
        return -ENOTTY;
    }

  return OK;
}

/****************************************************************************
 * Name: lcd_poll
 ****************************************************************************/

static int lcd_poll(struct file *filep, struct pollfd *fds,
                        bool setup)
{
  if (setup)
    {
      /* Data is always available to be read */

      poll_notify(&fds, 1, POLLIN | POLLOUT);
    }

  return OK;
}

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
  int ret = OK;

  /* Only initialize the driver once. */

  if (!g_lcd1602.initialized)
    {
      lcdinfo("Initializing\n");

      /* Configure GPIO pins */

      putreg16(0, PIC32MX_IOPORTE_TRIS);       /* Set DB0-15 as outputs */
      pic32mx_configgpio(GPIO_LCD_RS);         /* RS: Selects command or data */
      pic32mx_configgpio(GPIO_LCD_RW);         /* RW: Selects read or write */
      pic32mx_configgpio(GPIO_LCD_E);          /* E:  Starts transfer */

      /* Configure LCD power in the OFF state */

      pic32mx_configgpio(GPIO_LCD_LIGHT);       /* K */
      pic32mx_configgpio(GPIO_LCD_COMP);        /* Vo */
      pic32mx_configgpio(GPIO_LCD_PWR);         /* Vbuson/AN5/RB5 controls +5V USB */
      g_lcd1602.brightness = 0;                 /* Remember the light is off */

      /* A small delay is necessary between when GPIO_LCD_E was set up as an
       * output with initial value of 0 and this operation. That delay should
       * be well covered by the intervening GPIO configurations.
       */

      pic32mx_gpiowrite(GPIO_LCD_E, true);     /* Enable transfer */

      /* Configure and enable the LCD */

      /* Delay for 4.1MS or more */

      up_mdelay(5);

      /* Select the 8-bit interface. BF cannot be checked before this
       * command. This needs to be done a few times with some magic delays.
       *
       * Function set: 5x7 Style | N=2R | DL=8D
       */

      lcd_wrcommand(HD4478OU_FUNC | HD4478OU_FUNC_F5X7 |
                    HD4478OU_FUNC_N1 | HD4478OU_FUNC_DL8D);
      up_udelay(100);            /* Delay more than 100uS */
      lcd_wrcommand(HD4478OU_FUNC | HD4478OU_FUNC_F5X7 |
                    HD4478OU_FUNC_N1 | HD4478OU_FUNC_DL8D);
      up_udelay(40);             /* Delay more than 40uS */
      lcd_wrcommand(HD4478OU_FUNC | HD4478OU_FUNC_F5X7 |
                    HD4478OU_FUNC_N1 | HD4478OU_FUNC_DL8D);
      lcd_waitbusy();
      lcd_wrcommand(HD4478OU_FUNC | HD4478OU_FUNC_F5X7 |
                    HD4478OU_FUNC_N1 | HD4478OU_FUNC_DL8D);
      lcd_waitbusy();

      /* Display ON, cursor OFF, blink OFF */

      lcd_wrcommand(HD4478OU_DISPLAY | HD4478OU_DISPLAY_ON);
      lcd_waitbusy();

      /* Clear the display and home the cursor */

      lcd_wrcommand(HD4478OU_CLEAR); /* Clear display */
      lcd_waitbusy();
      lcd_wrcommand(HD4478OU_RETURN); /* Return home: AC=0 */
      lcd_waitbusy();

      /* Entry Mode Set:
       *
       * - Increment address by one,
       * - Shift cursor to right (display is not shifted)
       */

      lcd_wrcommand(HD4478OU_INPUT | HD4478OU_INPUT_INCR);

      /* Register the LCD device driver */

      ret = register_driver("/dev/lcd1602", &g_lcdops, 0644, &g_lcd1602);
      g_lcd1602.initialized = true;
    }

  return ret;
}

#endif /* CONFIG_LCD_LCD1602 */

/****************************************************************************
 * drivers/lcd/ht16k33_14seg.c
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

/* Alphanumeric display driver for HOLTEK HT16K33 (and VINKA VK16K33 clone)
 * This driver is specific for a 0.54" 14-segment LED HT16K33 Backpack
 * module with 4 14-segment digits (2 Kingbright 5241AS display).
 * Note: the model I'm testing uses the VK16K33.
 *
 * This is how the displays are connected:
 * Left Display: Digit 1: Catode connected to COM3
 * Left Display: Digit 2: Catode connected to COM2
 * Right Display: Digit 1: Catode connected to COM1
 * Right Display: Digit 2: Catode connected to COM0
 *
 * 14-Segment  |   LED Controller
 * -------------------------------
 * 8  - DP     |   ROW14 - 11
 * 13 - p      |   ROW6  - 19
 * 2  - n      |   ROW11 - 14
 * 4  - m      |   ROW12 - 13
 * 5  - l      |   ROW13 - 12
 * 6  - k      |   ROW7  - 18
 * 14 - j      |   ROW10 - 15
 * 15 - h      |   ROW9  - 16
 * 17 - g      |   ROW8  - 17
 * 18 - f      |   ROW5  - 20
 * 1  - e      |   ROW4  - 21
 * 7  - d      |   ROW3  - 22
 * 9  - c      |   ROW2  - 23
 * 10 - b      |   ROW1  - 24
 * 12 - a      |   ROW0  - 25
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/ascii.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lcd/slcd_codec.h>
#include <nuttx/lcd/slcd_ioctl.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/ht16k33.h>

#ifndef CONFIG_LIBC_SLCDCODEC
# error please also select Library Routines, Segment LCD CODEC
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_LCD_HT16K33)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C frequency */

#ifndef CONFIG_HT16K33_I2C_FREQ
#  define CONFIG_HT16K33_I2C_FREQ 400000
#endif

#ifndef CONFIG_LCD_HT16K33_NUMBER_MODULES
#  define CONFIG_LCD_HT16K33_NUMBER_MODULES 1
#endif

#define HT16K33_MAX_ROW    1
#define HT16K33_MAX_COL    4 * CONFIG_LCD_HT16K33_NUMBER_MODULES

/* Device naming ************************************************************/

#define DEVNAME_FMT    "/dev/slcd%d"
#define DEVNAME_FMTLEN (9 + 3 + 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ht16k33_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t    row;               /* Current row position to write on display  */
  uint8_t    col;               /* Current col position to write on display  */
  uint8_t    buffer[HT16K33_MAX_COL];
  bool       pendscroll;
  mutex_t    lock;
};

struct lcd_instream_s
{
  struct lib_instream_s stream;
  FAR const char *buffer;
  ssize_t nbytes;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void ht16k33_write_cmd(FAR struct ht16k33_dev_s *priv,
                                     int dev_id, uint8_t cmd);

static inline void ht16k33_write_data(FAR struct ht16k33_dev_s *priv,
                                      int dev_id, uint8_t cmd,
                                      uint8_t *values, int nbytes);

static inline void ht16k33_setcontrast(FAR struct ht16k33_dev_s *priv,
                                       int dev_id, int8_t contrast);

static void lcd_scroll_up(FAR struct ht16k33_dev_s *priv);

static void ht16k33_clear_display(FAR struct ht16k33_dev_s *priv);

/* Character driver methods */

static ssize_t ht16k33_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t ht16k33_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static off_t   ht16k33_seek(FAR struct file *filep, off_t offset,
                            int whence);
static int     ht16k33_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ht16k33fops =
{
  NULL,           /* open */
  NULL,           /* close */
  ht16k33_read,   /* read */
  ht16k33_write,  /* write */
  ht16k33_seek,   /* seek */
  ht16k33_ioctl,  /* ioctl */
  NULL            /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ht16k33_write_cmd
 *
 * Description:
 *   Write an Instruction command to HT16K33
 *
 ****************************************************************************/

static inline void ht16k33_write_cmd(FAR struct ht16k33_dev_s *priv,
                                     int dev_id, uint8_t cmd)
{
  struct i2c_msg_s msg;
  uint8_t data[1];
  int ret;

  /* Prepare data to send */

  data[0] = cmd;

  /* Setup the HT16K33 Command */

  msg.frequency = CONFIG_HT16K33_I2C_FREQ;   /* I2C frequency */
  msg.addr      = HT16K33_I2C_ADDR + dev_id; /* 7-bit address */
  msg.flags     = 0;                         /* Write transaction */
  msg.buffer    = data;                      /* Transfer from this address */
  msg.length    = 1;                         /* Send one byte */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      lcderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: ht16k33_write_data
 *
 * Description:
 *   Write a Data command to HT16K33
 *
 ****************************************************************************/

static inline void ht16k33_write_data(FAR struct ht16k33_dev_s *priv,
                                      int dev_id, uint8_t cmd,
                                      uint8_t *values, int nbytes)
{
  struct i2c_msg_s msg;
  uint8_t data[16];
  int ret;
  int i;

  /* Prepare data to send */

  data[0] = cmd;

  for (i = 0; i < nbytes; i++)
    {
      data[i + 1] = values[i];
    }

  /* Setup the message to write data to HT16K33 */

  msg.frequency = CONFIG_HT16K33_I2C_FREQ;   /* I2C frequency */
  msg.addr      = HT16K33_I2C_ADDR + dev_id; /* 7-bit address */
  msg.flags     = 0;                         /* Write transaction */
  msg.buffer    = data;                      /* Transfer from here */
  msg.length    = nbytes + 1;                /* Send cmd + nbytes */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      lcderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }
}

static inline void ht16k33_setcontrast(FAR struct ht16k33_dev_s *priv,
                                       int dev_id, int8_t contrast)
{
  int i;

  if (contrast < HT16K33_CONTRAST_MIN)
    {
      contrast = HT16K33_CONTRAST_MIN;
    }
  else if (contrast > HT16K33_CONTRAST_MAX)
    {
      contrast = HT16K33_CONTRAST_MAX;
    }

  for (i = 0; i < CONFIG_LCD_HT16K33_NUMBER_MODULES; i++)
    {
      ht16k33_write_cmd(priv, i, HT16K33_DIMMING_SET | (contrast & 0x0f));
    }
}

/****************************************************************************
 * Name: lcd_getdata
 *
 * Description:
 *  Simulate reading data from LCD, we are reading from internal buffer
 *
 ****************************************************************************/

static inline uint8_t lcd_getdata(FAR struct ht16k33_dev_s *priv)
{
  uint8_t data;
  data = priv->buffer[priv->row * priv->col];
  return data;
}

/****************************************************************************
 * Name: rc2addr
 *
 * Description:
 *  This converts a row/column pair to a screen memory address.
 *
 ****************************************************************************/

static inline uint8_t rc2addr(FAR struct ht16k33_dev_s *priv)
{
  /* Each module has 4 digits they correspond to these columns:
   *
   * col0: 0x00 - 0x01, col1: 0x02 - 0x03,
   * col2: 0x04 - 0x05, col3: 0x06 - 0x07
   */

  return (priv->col % 4) * 0x02;
}

/****************************************************************************
 * Name: addr2rc
 *
 * Description:
 *  This converts a screen memory address to a row/column pair.
 *
 ****************************************************************************/

static inline void addr2rc(FAR struct ht16k33_dev_s *priv, uint8_t addr,
                           FAR uint8_t *row, FAR uint8_t *col)
{
  *row = 0;
  *col = addr / 2;
}

/****************************************************************************
 * Name: lcd_set_curpos
 *
 * Description:
 *  This sets the cursor position based on row, column addressing.
 *
 * Input Parameters:
 *  priv - device instance
 *
 ****************************************************************************/

static void lcd_set_curpos(FAR struct ht16k33_dev_s *priv)
{
  uint8_t addr;
  int dev_id;

  addr = rc2addr(priv);
  dev_id = priv->col / 4;

  /* Define the memory address position */

  ht16k33_write_cmd(priv, dev_id, HT16K33_DISP_DATA_ADDR | addr);
}

/****************************************************************************
 * Name: lcd_putdata
 *
 * Description:
 *  Write a byte to the LCD and update column/row position
 *
 ****************************************************************************/

static inline void lcd_putdata(FAR struct ht16k33_dev_s *priv, uint8_t data)
{
  uint8_t segment[2];
  uint8_t addr;
  uint8_t cmd;
  int dev_id;

  /* Get current display memory position */

  addr = rc2addr(priv);

  /* Setup the memory command */

  cmd = HT16K33_DISP_DATA_ADDR | addr;

  /* Get the segments setting */

  segment[0] = asciito14seg[data - 32] & 0xff;
  segment[1] = (asciito14seg[data - 32] & 0xff00) >> 8;

  dev_id = priv->col / 4;

  /* Send data to display */

  ht16k33_write_data(priv, dev_id, cmd, segment, 2);

  /* Save it in the buffer because we cannot read from display */

  priv->buffer[priv->col * priv->row] = data;

  /* Update col/row positions */

  priv->col++;

  if (priv->col >= HT16K33_MAX_COL)
    {
      priv->col = 0;
      priv->row++;
    }

  if (priv->row >= HT16K33_MAX_ROW)
    {
      priv->pendscroll = true;
      priv->row        = HT16K33_MAX_ROW - 1;
    }

  /* Update cursor position */

  lcd_set_curpos(priv);
}

/****************************************************************************
 * Name: lcd_scroll_up
 *
 * Description:
 *  Scroll the display up, and clear the new (last) line.
 *
 ****************************************************************************/

static void lcd_scroll_up(FAR struct ht16k33_dev_s *priv)
{
  FAR uint8_t *data;
  int currow;
  int curcol;

  data = (FAR uint8_t *)kmm_malloc(HT16K33_MAX_COL);
  if (NULL == data)
    {
      lcdinfo("Failed to allocate buffer in lcd_scroll_up()\n");
      return;
    }

  for (currow = 1; currow < HT16K33_MAX_ROW; ++currow)
    {
      priv->row = currow;
      for (curcol = 0; curcol < HT16K33_MAX_COL; ++curcol)
        {
          priv->col = curcol;
          data[curcol] = lcd_getdata(priv);
        }

      priv->col = 0;
      priv->row = currow - 1;
      lcd_set_curpos(priv);
      for (curcol = 0; curcol < HT16K33_MAX_COL; ++curcol)
        {
          lcd_putdata(priv, data[curcol]);
        }
    }

  ht16k33_clear_display(priv);

  kmm_free(data);
  return;
}

/****************************************************************************
 * Name: ht16k33_clear_display
 *
 * Description:
 *  Clear the display writing space (' ') to all positions
 *
 ****************************************************************************/

static void ht16k33_clear_display(FAR struct ht16k33_dev_s *priv)
{
  int curcol;

  priv->col = 0;
  priv->row = HT16K33_MAX_ROW - 1;
  lcd_set_curpos(priv);
  for (curcol = 0; curcol < HT16K33_MAX_COL; ++curcol)
    {
      lcd_putdata(priv, ' ');
    }

  priv->col = 0;
  priv->row = HT16K33_MAX_ROW - 1;
  lcd_set_curpos(priv);
}

/****************************************************************************
 * Name: lcd_codec_action
 *
 * Description:
 *  Perform an 'action' as per the Segment LCD codec.
 *
 * Input Parameters:
 *  priv - device instance
 *  code - SLCD code action code
 *  count - count param for those actions that take it
 *
 ****************************************************************************/

static void lcd_codec_action(FAR struct ht16k33_dev_s *priv,
                             enum slcdcode_e code, uint8_t count)
{
  switch (code)
    {
      /* Erasure */

      case SLCDCODE_BACKDEL:         /* Backspace (backward delete)
                                      * N characters
                                      */
        {
          if (count <= 0)            /* we need to delete more 0 positions */
            {
              break;
            }
          else
            {
              if (count > priv->col) /* saturate to preceding columns
                                      * available
                                      */
                {
                  count = priv->col;
                }

              priv->col = priv->col - count;
              lcd_set_curpos(priv);
            }

          /* ... and conscientiously fall through to next case ... */
        }

      case SLCDCODE_FWDDEL:          /* Delete (forward delete) N characters
                                      * moving text
                                      */
        {
          if (count <= 0)            /* we need to delete more 0 positions */
            {
              break;
            }
          else
            {
              uint8_t start;
              uint8_t end;
              uint8_t i;
              uint8_t data;

              start = priv->col + count;

              if (start >= HT16K33_MAX_COL)    /* nothing left */
                {
                  break;
                }

              end = start + count;
              if (end > HT16K33_MAX_COL)      /* saturate */
                {
                  end = HT16K33_MAX_COL;
                }

              for (i = priv->col; i < end; ++start, ++i) /* like memmove */
                {
                  priv->col = start;
                  lcd_set_curpos(priv);
                  data = lcd_getdata(priv);
                  priv->col = i;
                  lcd_set_curpos(priv);
                  lcd_putdata(priv, data);
                }

              for (; i < HT16K33_MAX_COL; ++i) /* much like memset */
                {
                  lcd_putdata(priv, ' ');
                }

              lcd_set_curpos(priv);
            }
        }
        break;

      case SLCDCODE_ERASE:           /* Erase N characters from the cursor
                                      * position
                                      */
        if (count > 0)
          {
            uint8_t end;
            uint8_t i;

            end = priv->col + count;
            if (end > HT16K33_MAX_COL)
              {
                end = HT16K33_MAX_COL;
              }

            for (i = priv->col; i < end; ++i)
              {
                lcd_putdata(priv, ' ');
              }

            lcd_set_curpos(priv);
          }
        break;

      case SLCDCODE_CLEAR:           /* Home the cursor and erase the entire
                                      * display
                                      */
        {
          /* ht16k33_write_cmd(priv, HT16K33_CLEAR_DISPLAY); */
        }
        break;

      case SLCDCODE_ERASEEOL:        /* Erase from the cursor position to
                                      * the end of line
                                      */
        {
          uint8_t i;

          for (i = priv->col; i < HT16K33_MAX_COL; ++i)
            {
              lcd_putdata(priv, ' ');
            }

          lcd_set_curpos(priv);
        }
        break;

      /* Cursor movement */

      case SLCDCODE_LEFT:            /* Cursor left by N characters */
        {
          if (count > priv->col)
            {
              priv->col = 0;
            }
          else
            {
              priv->col -= count;
            }

          lcd_set_curpos(priv);
        }
        break;

      case SLCDCODE_RIGHT:           /* Cursor right by N characters */
        {
          priv->col += count;
          if (priv->col >= HT16K33_MAX_COL)
            {
              priv->col = HT16K33_MAX_COL - 1;
            }

          lcd_set_curpos(priv);
        }
        break;

      case SLCDCODE_UP:              /* Cursor up by N lines */
        {
          if (count > priv->row)
            {
              priv->row = 0;
            }
          else
            {
              priv->row -= count;
            }

          lcd_set_curpos(priv);
        }
        break;

      case SLCDCODE_DOWN:            /* Cursor down by N lines */
        {
          priv->row += count;
          if (priv->row >= HT16K33_MAX_ROW)
            {
              priv->row = HT16K33_MAX_ROW - 1;
            }

          lcd_set_curpos(priv);
        }
        break;

      case SLCDCODE_HOME:            /* Cursor home */
        {
          priv->col = 0;
          lcd_set_curpos(priv);
        }
        break;

      case SLCDCODE_END:             /* Cursor end */
        {
          priv->col = HT16K33_MAX_COL - 1;
          lcd_set_curpos(priv);
        }
        break;

      case SLCDCODE_PAGEUP:          /* Cursor up by N pages */
      case SLCDCODE_PAGEDOWN:        /* Cursor down by N pages */
        break;                       /* Not supportable on this SLCD */

      /* Blinking */

      case SLCDCODE_BLINKSTART:      /* Start blinking with current cursor
                                      * position
                                      */
        ht16k33_write_cmd(priv, 0, HT16K33_DISPLAY_SETUP |
                                   DISPLAY_SETUP_BLINK_2HZ);
        break;

      case SLCDCODE_BLINKEND:        /* End blinking after the current cursor
                                      * position
                                      */
      case SLCDCODE_BLINKOFF:        /* Turn blinking off */
        ht16k33_write_cmd(priv, 0, HT16K33_DISPLAY_SETUP |
                                   DISPLAY_SETUP_BLINK_OFF);
        break;                       /* Not implemented */

      /* These are actually unreportable errors */

      default:
      case SLCDCODE_NORMAL:          /* Not a special keycode */
        break;
    }
}

/****************************************************************************
 * Name: lcd_getstream
 *
 * Description:
 *   Get one character from the LCD codec stream.
 *
 ****************************************************************************/

static int lcd_getstream(FAR struct lib_instream_s *instream)
{
  FAR struct lcd_instream_s *lcdstream =
    (FAR struct lcd_instream_s *)instream;

  if (lcdstream->nbytes > 0)
    {
      lcdstream->nbytes--;
      lcdstream->stream.nget++;
      return (int)*lcdstream->buffer++;
    }

  return EOF;
}

/****************************************************************************
 * Name: lcd_init
 *
 * Description:
 *  perform the initialization sequence to get the LCD into a known state.
 *
 ****************************************************************************/

static void lcd_init(FAR struct ht16k33_dev_s *priv)
{
  uint8_t data;
  int i;

  for (i = 0; i < CONFIG_LCD_HT16K33_NUMBER_MODULES; i++)
    {
      /* Initialize the Display: Turn ON Oscillator */

      data = HT16K33_SYSTEM_SETUP | SYSTEM_SETUP_OSC_ON;

      ht16k33_write_cmd(priv, i, data);

      /* Clear display */

      ht16k33_clear_display(priv);

      /* Display ON */

      data = HT16K33_DISPLAY_SETUP | DISPLAY_SETUP_DISP_ON;

      ht16k33_write_cmd(priv, i, data);
    }
}

/****************************************************************************
 * Name: lcd_curpos_to_fpos
 *
 * Description:
 *   Convert a screen cursor pos (row,col) to a file logical offset.  This
 *   includes 'synthesized' line feeds at the end of screen lines.
 *
 ****************************************************************************/

static void lcd_curpos_to_fpos(FAR struct ht16k33_dev_s *priv,
                               uint8_t row, uint8_t col, FAR off_t *fpos)
{
  /* the logical file position is the linear position plus any synthetic LF */

  *fpos = (row * HT16K33_MAX_COL) + col + row;
}

/****************************************************************************
 * Name: ht16k33_read
 ****************************************************************************/

static ssize_t ht16k33_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ht16k33_write
 ****************************************************************************/

static ssize_t ht16k33_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ht16k33_dev_s *priv = inode->i_private;
  struct lcd_instream_s instream;
  struct slcdstate_s state;
  enum slcdret_e result;
  uint8_t ch;
  uint8_t count;

  nxmutex_lock(&priv->lock);

  /* Initialize the stream for use with the SLCD CODEC */

  instream.stream.get  = lcd_getstream;
  instream.stream.nget = 0;
  instream.buffer      = buffer;
  instream.nbytes      = buflen;

  /* Now decode and process every byte in the input buffer */

  memset(&state, 0, sizeof(struct slcdstate_s));
  while ((result = slcd_decode(&instream.stream, &state, &ch, &count)) !=
         SLCDRET_EOF)
    {
      /* Is there some pending scroll? */

      if (priv->pendscroll)
        {
          lcd_scroll_up(priv);
          priv->pendscroll = false;
        }

      if (result == SLCDRET_CHAR)          /* A normal character was returned */
        {
          /* Check for ASCII control characters */

          if (ch == ASCII_TAB)
            {
              /* TODO: define what TAB should do */
            }
          else if (ch == ASCII_VT)
            {
              /* Turn the backlight on */

              /* TODO: lcd_backlight(priv, true); */
            }
          else if (ch == ASCII_FF)
            {
              /* Turn the backlight off */

              /* TODO: lcd_backlight(priv, false); */
            }
          else if (ch == ASCII_CR)
            {
              /* Perform a Home */

              priv->col = 0;
              lcd_set_curpos(priv);
            }
          else if (ch == ASCII_SO)
            {
              /* TODO: We don't have cursor */
            }
          else if (ch == ASCII_SI)
            {
              /* Perform the re-initialize */

              lcd_init(priv);
              priv->row = 0;
              priv->col = 0;
            }
          else if (ch == ASCII_LF)
            {
              /* unixian line term; go to start of next line */

              priv->row += 1;
              if (priv->row >= HT16K33_MAX_ROW)
                {
                  priv->pendscroll = true;
                  priv->row = HT16K33_MAX_ROW - 1;
                }

              priv->col = 0;
              lcd_set_curpos(priv);
            }
          else if (ch == ASCII_BS)
            {
              /* Perform the backward deletion */

              lcd_codec_action(priv, SLCDCODE_BACKDEL, 1);
            }
          else if (ch == ASCII_DEL)
            {
              /* Perform the forward deletion */

              lcd_codec_action(priv, SLCDCODE_FWDDEL, 1);
            }
          else
            {
              /* Just print it! */

              lcd_putdata(priv, ch);
            }
        }
      else /* (result == SLCDRET_SPEC) */  /* A special SLCD action was returned */
        {
          lcd_codec_action(priv, (enum slcdcode_e)ch, count);
        }
    }

  /* Wherever we wound up, update our logical file pos to reflect it */

  lcd_curpos_to_fpos(priv, priv->row, priv->col, &filep->f_pos);

  nxmutex_unlock(&priv->lock);
  return buflen;
}

/****************************************************************************
 * Name: ht16k33_seek
 *
 * Description:
 *   Seek the logical file pointer to the specified position.  This is
 *   probably not very interesting except possibly for (SEEK_SET, 0) to
 *   rewind the pointer for a subsequent read().
 *   The file pointer is logical, and includes synthesized LF chars at the
 *   end of the display lines.
 *
 ****************************************************************************/

static off_t ht16k33_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ht16k33_dev_s *priv =
    (FAR struct ht16k33_dev_s *)inode->i_private;
  off_t maxpos;
  off_t pos;

  nxmutex_lock(&priv->lock);

  maxpos = HT16K33_MAX_ROW * HT16K33_MAX_COL + (HT16K33_MAX_ROW - 1);
  pos    = filep->f_pos;

  switch (whence)
    {
      case SEEK_CUR:
        pos += offset;
        if (pos > maxpos)
          {
            pos = maxpos;
          }
        else if (pos < 0)
          {
            pos = 0;
          }

        filep->f_pos = pos;
        break;

      case SEEK_SET:
        pos = offset;
        if (pos > maxpos)
          {
            pos = maxpos;
          }
        else if (pos < 0)
          {
            pos = 0;
          }

        filep->f_pos = pos;
        break;

      case SEEK_END:
        pos = maxpos + offset;
        if (pos > maxpos)
          {
            pos = maxpos;
          }
        else if (pos < 0)
          {
            pos = 0;
          }

        filep->f_pos = pos;
        break;

      default:

        /* Return EINVAL if the whence argument is invalid */

        pos = (off_t)-EINVAL;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return pos;
}

/****************************************************************************
 * Name: ht16k33_ioctl
 *
 * Description:
 *   Perform device operations that are outside the standard I/O model.
 *
 ****************************************************************************/

static int ht16k33_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg)
{
  switch (cmd)
    {
      case SLCDIOC_GETATTRIBUTES: /* Get the attributes of the SLCD */
        {
          FAR struct slcd_attributes_s *attr =
            (FAR struct slcd_attributes_s *)((uintptr_t)arg);

          lcdinfo("SLCDIOC_GETATTRIBUTES:\n");

          if (!attr)
            {
              return -EINVAL;
            }

          attr->nrows         = HT16K33_MAX_ROW;
          attr->ncolumns      = HT16K33_MAX_COL;
          attr->nbars         = 0;
          attr->maxcontrast   = 0;
          attr->maxbrightness = 16;  /* 'brightness' for us is the backlight */
        }
        break;

      case SLCDIOC_CURPOS:        /* Get the SLCD cursor position */
        {
          FAR struct inode *inode = filep->f_inode;
          FAR struct ht16k33_dev_s *priv =
            (FAR struct ht16k33_dev_s *)inode->i_private;
          FAR struct slcd_curpos_s *attr =
            (FAR struct slcd_curpos_s *)((uintptr_t)arg);

          attr->row    = priv->row;
          attr->column = priv->col;
        }
        break;

      case SLCDIOC_GETBRIGHTNESS: /* Get the current brightness setting */
        {
          FAR struct inode *inode = filep->f_inode;
          FAR struct ht16k33_dev_s *priv =
            (FAR struct ht16k33_dev_s *)inode->i_private;

          nxmutex_lock(&priv->lock);
          *(FAR int *)((uintptr_t)arg) = 1; /* Hardcoded */
          nxmutex_unlock(&priv->lock);
        }
        break;

      case SLCDIOC_SETBRIGHTNESS: /* Set the brightness to a new value */
        {
          FAR struct inode *inode = filep->f_inode;
          FAR struct ht16k33_dev_s *priv =
            (FAR struct ht16k33_dev_s *)inode->i_private;

          nxmutex_lock(&priv->lock);
          ht16k33_setcontrast(priv, 0, (uint8_t)arg);
          nxmutex_unlock(&priv->lock);
        }
        break;

      case SLCDIOC_SETBAR:        /* Set bars on a bar display */
      case SLCDIOC_GETCONTRAST:   /* Get the current contrast setting */
      case SLCDIOC_SETCONTRAST:   /* Set the contrast to a new value */
      default:
        return -ENOTTY;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ht16k33_register
 *
 * Description:
 *   Register the HT16K33 character device as 'devpath'
 *
 * Input Parameters:
 *   devno   - The device number to register. E.g., "/dev/slcd0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             HT16K33
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ht16k33_register(int devno, FAR struct i2c_master_s *i2c)
{
  FAR struct ht16k33_dev_s *priv;
  char devname[DEVNAME_FMTLEN];
  int ret;

  /* Initialize the HT16K33 device structure */

  priv = (FAR struct ht16k33_dev_s *)
         kmm_malloc(sizeof(struct ht16k33_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Setup priv with initial values */

  priv->i2c        = i2c;
  priv->col        = 0;
  priv->row        = 0;
  priv->pendscroll = false;

  nxmutex_init(&priv->lock);

  /* Initialize the display */

  lcd_init(priv);

  /* Create the character device name */

  snprintf(devname, sizeof(devname), DEVNAME_FMT, devno);

  /* Register the driver */

  ret = register_driver(devname, &g_ht16k33fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SPI && CONFIG_HT16K33 */

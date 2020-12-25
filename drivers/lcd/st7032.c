/****************************************************************************
 * drivers/sensors/st7032.c
 * Alphanumeric LCD driver for ST7032i (tested on JLX1602G-390)
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2018 Alan Carvalho de Assis
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Basic initialization based on Olav Kallhovd driver.
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/ascii.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lcd/slcd_codec.h>
#include <nuttx/lcd/slcd_ioctl.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/st7032.h>

#ifndef CONFIG_LIB_SLCDCODEC
# error please also select Library Routines, Segment LCD CODEC
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_LCD_ST7032)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C frequency */

#ifndef CONFIG_ST7032_I2C_FREQ
#  define CONFIG_ST7032_I2C_FREQ 400000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct st7032_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t    row;               /* Current row position to write on display  */
  uint8_t    col;               /* Current col position to write on display  */
  uint8_t    buffer[ST7032_MAX_ROW * ST7032_MAX_COL];
  bool       pendscroll;
  sem_t sem_excl;
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

static inline void st7032_write_inst(FAR struct st7032_dev_s *priv,
                                     uint8_t cmd);

static inline void st7032_write_data(FAR struct st7032_dev_s *priv,
                                     uint8_t value);

static inline void st7032_setcontrast(FAR struct st7032_dev_s *priv,
                                      int8_t contrast);

static void lcd_scroll_up(FAR struct st7032_dev_s *priv);

/* Character driver methods */

static int     st7032_open(FAR struct file *filep);
static int     st7032_close(FAR struct file *filep);
static ssize_t st7032_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t st7032_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static off_t   st7032_seek(FAR struct file *filep, off_t offset, int whence);
static int     st7032_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_st7032fops =
{
  st7032_open,   /* open */
  st7032_close,  /* close */
  st7032_read,   /* read */
  st7032_write,  /* write */
  st7032_seek,   /* seek */
  st7032_ioctl,  /* ioctl */
  NULL,          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st7032_write_inst
 *
 * Description:
 *   Write an Instruction command to ST7032
 *
 ****************************************************************************/

static inline void st7032_write_inst(FAR struct st7032_dev_s *priv,
                                     uint8_t cmd)
{
  struct i2c_msg_s msg;
  uint8_t data[2];
  int ret;

  /* Prepare data to send */

  data[0] = 0x00;
  data[1] = cmd;

  /* Setup the ST7032 Co command */

  msg.frequency = CONFIG_ST7032_I2C_FREQ;   /* I2C frequency */
  msg.addr      = ST7032_I2C_ADDR;          /* 7-bit address */
  msg.flags     = 0;                        /* Write transaction, beginning with START */
  msg.buffer    = (FAR uint8_t *) data;     /* Transfer from this address */
  msg.length    = 2;                        /* Send two bytes */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      lcderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }

  /* Delay 30us */

  nxsig_usleep(30);
}

/****************************************************************************
 * Name: st7032_write_data
 *
 * Description:
 *   Write a Data command to ST7032
 *
 ****************************************************************************/

static inline void st7032_write_data(FAR struct st7032_dev_s *priv,
                                     uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t data[2];
  int ret;

  /* Prepare data to send */

  data[0] = ST7032_CTRLBIT_RS;
  data[1] = value;

  /* Setup the ST7032 Co command */

  msg.frequency = CONFIG_ST7032_I2C_FREQ;   /* I2C frequency */
  msg.addr      = ST7032_I2C_ADDR;          /* 7-bit address */
  msg.flags     = 0;                        /* Write transaction, beginning with START */
  msg.buffer    = (FAR uint8_t *) data;     /* Transfer from this address */
  msg.length    = 2;                        /* Send two bytes: Co command + cmd */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      lcderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }

  /* Delay 30us */

  nxsig_usleep(30);
}

static inline void st7032_setcontrast(FAR struct st7032_dev_s *priv,
                                      int8_t contrast)
{
  if (contrast < ST7032_CONTRAST_MIN)
    {
      contrast = ST7032_CONTRAST_MIN;
    }
  else if (contrast > ST7032_CONTRAST_MAX)
    {
      contrast = ST7032_CONTRAST_MAX;
    }

  st7032_write_inst(priv, ST7032_CONTRAST_SET | (contrast & 0x0f));

  st7032_write_inst(priv, (contrast >> 4) | ST7032_POWER_ICON_CTRL_SET |
                           POWER_ICON_BOST_CTRL_BON);
}

/****************************************************************************
 * Name: lcd_getdata
 *
 * Description:
 *  Simulate reading data from LCD, we are reading from internal buffer
 *
 ****************************************************************************/

static inline uint8_t lcd_getdata(FAR struct st7032_dev_s *priv)
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

static inline uint8_t rc2addr(FAR struct st7032_dev_s *priv)
{
  /* line0 @ 0x00 - 0x27, line1 @ 0x40-0x67 */

  return priv->row * 0x40 + priv->col;
}

/****************************************************************************
 * Name: addr2rc
 *
 * Description:
 *  This converts a screen memory address to a row/column pair.
 *
 ****************************************************************************/

static inline void addr2rc(FAR struct st7032_dev_s *priv, uint8_t addr,
                           FAR uint8_t *row, FAR uint8_t *col)
{
  *row = addr / 0x40;
  *col = addr % 0x40;
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

static void lcd_set_curpos(FAR struct st7032_dev_s *priv)
{
  uint8_t addr;
  addr = rc2addr(priv);
  st7032_write_inst(priv, ST7032_SET_DDRAM_ADDR | addr); /* set DDRAM address */
}

/****************************************************************************
 * Name: lcd_putdata
 *
 * Description:
 *  Write a byte to the LCD and update column/row position
 *
 ****************************************************************************/

static inline void lcd_putdata(FAR struct st7032_dev_s *priv, uint8_t data)
{
  /* Send data to display */

  st7032_write_data(priv, data);

  /* Save it in the buffer because we cannot read from display */

  priv->buffer[priv->col * priv->row] = data;

  /* Update col/row positions */

  priv->col++;

  if (priv->col >= ST7032_MAX_COL)
    {
      priv->col = 0;
      priv->row++;
    }

  if (priv->row >= ST7032_MAX_ROW)
    {
      priv->pendscroll = true;
      priv->row        = ST7032_MAX_ROW - 1;
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

static void lcd_scroll_up(FAR struct st7032_dev_s *priv)
{
  FAR uint8_t *data;
  int currow;
  int curcol;

  data = (FAR uint8_t *)kmm_malloc(ST7032_MAX_COL);
  if (NULL == data)
    {
      lcdinfo("Failed to allocate buffer in lcd_scroll_up()\n");
      return;
    }

  /* Clear display */

  st7032_write_inst(priv, ST7032_CLEAR_DISPLAY);

  for (currow = 1; currow < ST7032_MAX_ROW; ++currow)
    {
      priv->row = currow;
      for (curcol = 0; curcol < ST7032_MAX_COL; ++curcol)
        {
          priv->col = curcol;
          data[curcol] = lcd_getdata(priv);
        }

      priv->col = 0;
      priv->row = currow - 1;
      lcd_set_curpos(priv);
      for (curcol = 0; curcol < ST7032_MAX_COL; ++curcol)
        {
          lcd_putdata(priv, data[curcol]);
        }
    }

  priv->col = 0;
  priv->row = ST7032_MAX_ROW - 1;
  lcd_set_curpos(priv);
  for (curcol = 0; curcol < ST7032_MAX_COL; ++curcol)
    {
      lcd_putdata(priv, ' ');
    }

  priv->col = 0;
  priv->row = ST7032_MAX_ROW - 1;
  lcd_set_curpos(priv);

  kmm_free(data);
  return;
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

static void lcd_codec_action(FAR struct st7032_dev_s *priv,
                             enum slcdcode_e code, uint8_t count)
{
  switch (code)
    {
      /* Erasure */

      case SLCDCODE_BACKDEL:         /* Backspace (backward delete) N characters */
        {
          if (count <= 0)            /* we need to delete more 0 positions */
            {
              break;
            }
          else
            {
              if (count > priv->col) /* saturate to preceding columns available */
                {
                  count = priv->col;
                }

              priv->col = priv->col - count;
              lcd_set_curpos(priv);
            }

          /* ... and conscientiously fall through to next case ... */
        }

      case SLCDCODE_FWDDEL:          /* Delete (forward delete) N characters, moving text */
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

              if (start >= ST7032_MAX_COL)    /* silly case of nothing left */
                {
                  break;
                }

              end = start + count;
              if (end > ST7032_MAX_COL)      /* saturate */
                {
                  end = ST7032_MAX_COL;
                }

              for (i = priv->col; i < end; ++start, ++i) /* much like memmove */
                {
                  priv->col = start;
                  lcd_set_curpos(priv);
                  data = lcd_getdata(priv);
                  priv->col = i;
                  lcd_set_curpos(priv);
                  lcd_putdata(priv, data);
                }

              for (; i < ST7032_MAX_COL; ++i) /* much like memset */
                {
                  lcd_putdata(priv, ' ');
                }

              lcd_set_curpos(priv);
            }
        }
        break;

      case SLCDCODE_ERASE:           /* Erase N characters from the cursor position */
        if (count > 0)
          {
            uint8_t end;
            uint8_t i;

            end = priv->col + count;
            if (end > ST7032_MAX_COL)
              {
                end = ST7032_MAX_COL;
              }

            for (i = priv->col; i < end; ++i)
              {
                lcd_putdata(priv, ' ');
              }

            lcd_set_curpos(priv);
          }
        break;

      case SLCDCODE_CLEAR:           /* Home the cursor and erase the entire display */
        {
          st7032_write_inst(priv, ST7032_CLEAR_DISPLAY);
        }
        break;

      case SLCDCODE_ERASEEOL:        /* Erase from the cursor position to the end of line */
        {
          uint8_t i;

          for (i = priv->col; i < ST7032_MAX_COL; ++i)
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
          if (priv->col >= ST7032_MAX_COL)
            {
              priv->col = ST7032_MAX_COL - 1;
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
          if (priv->row >= ST7032_MAX_ROW)
            {
              priv->row = ST7032_MAX_ROW - 1;
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
          priv->col = ST7032_MAX_COL - 1;
          lcd_set_curpos(priv);
        }
        break;

      case SLCDCODE_PAGEUP:          /* Cursor up by N pages */
      case SLCDCODE_PAGEDOWN:        /* Cursor down by N pages */
        break;                       /* Not supportable on this SLCD */

      /* Blinking */

      case SLCDCODE_BLINKSTART:      /* Start blinking with current cursor position */
        st7032_write_inst(priv, ST7032_DISPLAY_ON_OFF | DISPLAY_ON_OFF_D |
                                DISPLAY_ON_OFF_C | DISPLAY_ON_OFF_B);
        break;

      case SLCDCODE_BLINKEND:        /* End blinking after the current cursor position */
      case SLCDCODE_BLINKOFF:        /* Turn blinking off */
        st7032_write_inst(priv, ST7032_DISPLAY_ON_OFF | DISPLAY_ON_OFF_D |
                                DISPLAY_ON_OFF_C);
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

static void lcd_init(FAR struct st7032_dev_s *priv)
{
  uint8_t data;

  /* Initialize the Display */

  data = ST7032_FUNCTION_SET | FUNCTION_SET_DL | FUNCTION_SET_N |
         FUNCTION_SET_IS;

  st7032_write_inst(priv, data);

  data = ST7032_INT_OSC_FREQ | INT_OSC_FREQ_BS | INT_OSC_FREQ_F2;

  st7032_write_inst(priv, data);

  data = ST7032_POWER_ICON_CTRL_SET | POWER_ICON_BOST_CTRL_ION;

  st7032_write_inst(priv, data);

  /* Set contrast */

  st7032_setcontrast(priv, DEFAULT_CONTRAST);

  data = ST7032_FOLLOWER_CTRL | FOLLOWER_CTRL_FON | FOLLOWER_CTRL_RAB2;

  st7032_write_inst(priv, data);

  /* Turn ON Display and Cursor Blinking */

  data = ST7032_DISPLAY_ON_OFF | DISPLAY_ON_OFF_D | DISPLAY_ON_OFF_C |
         DISPLAY_ON_OFF_B;

  st7032_write_inst(priv, data);

  /* Increasing Mode: Writing from Left to Right */

  data = ST7032_ENTRY_MODE_SET | ENTRY_MODE_SET_ID;

  st7032_write_inst(priv, data);

  /* Clear Display */

  data = ST7032_CLEAR_DISPLAY;

  st7032_write_inst(priv, data);
}

/****************************************************************************
 * Name: lcd_curpos_to_fpos
 *
 * Description:
 *   Convert a screen cursor pos (row,col) to a file logical offset.  This
 *   includes 'synthesized' line feeds at the end of screen lines.
 *
 ****************************************************************************/

static void lcd_curpos_to_fpos(FAR struct st7032_dev_s *priv,
                              uint8_t row, uint8_t col, FAR off_t *fpos)
{
  /* the logical file position is the linear position plus any synthetic LF */

  *fpos = (row * ST7032_MAX_COL) + col + row;
}

/****************************************************************************
 * Name: st7032_open
 *
 * Description:
 *   This function is called whenever the ST7032 device is opened.
 *
 ****************************************************************************/

static int st7032_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: st7032_close
 *
 * Description:
 *   This routine is called when the LM-75 device is closed.
 *
 ****************************************************************************/

static int st7032_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: st7032_read
 ****************************************************************************/

static ssize_t st7032_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: st7032_write
 ****************************************************************************/

static ssize_t st7032_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct st7032_dev_s *priv = inode->i_private;
  struct lcd_instream_s instream;
  struct slcdstate_s state;
  enum slcdret_e result;
  uint8_t ch;
  uint8_t count;

  nxsem_wait(&priv->sem_excl);

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
              /* Blink Cursor? Shouldn't it be just 4 spaces to indicate
               * TAB?
               */

              st7032_write_inst(priv, ST7032_DISPLAY_ON_OFF |
                                      DISPLAY_ON_OFF_D | DISPLAY_ON_OFF_C |
                                      DISPLAY_ON_OFF_B);
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
              st7032_write_inst(priv, ST7032_DISPLAY_ON_OFF |
                                      DISPLAY_ON_OFF_D);
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
              if (priv->row >= ST7032_MAX_ROW)
                {
                  priv->pendscroll = true;
                  priv->row = ST7032_MAX_ROW - 1;
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

  nxsem_post(&priv->sem_excl);
  return buflen;
}

/****************************************************************************
 * Name: st7032_seek
 *
 * Description:
 *   Seek the logical file pointer to the specified position.  This is
 *   probably not very interesting except possibly for (SEEK_SET, 0) to
 *   rewind the pointer for a subsequent read().
 *   The file pointer is logical, and includes synthesized LF chars at the
 *   end of the display lines.
 *
 ****************************************************************************/

static off_t st7032_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct st7032_dev_s *priv =
    (FAR struct st7032_dev_s *)inode->i_private;
  off_t maxpos;
  off_t pos;

  nxsem_wait(&priv->sem_excl);

  maxpos = ST7032_MAX_ROW * ST7032_MAX_COL + (ST7032_MAX_ROW - 1);
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

  nxsem_post(&priv->sem_excl);
  return pos;
}

/****************************************************************************
 * Name: st7032_ioctl
 *
 * Description:
 *   Perform device operations that are outside the standard I/O model.
 *
 ****************************************************************************/

static int st7032_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  switch (cmd)
    {
      case SLCDIOC_GETATTRIBUTES: /* Get the attributes of the SLCD */
        {
          FAR struct inode *inode = filep->f_inode;
          FAR struct slcd_attributes_s *attr =
            (FAR struct slcd_attributes_s *)((uintptr_t)arg);

          lcdinfo("SLCDIOC_GETATTRIBUTES:\n");

          if (!attr)
            {
              return -EINVAL;
            }

          attr->nrows         = ST7032_MAX_ROW;
          attr->ncolumns      = ST7032_MAX_COL;
          attr->nbars         = 0;
          attr->maxcontrast   = 0;
          attr->maxbrightness = 1;  /* 'brightness' for us is the backlight */
        }
        break;

      case SLCDIOC_CURPOS:        /* Get the SLCD cursor position */
        {
          FAR struct inode *inode = filep->f_inode;
          FAR struct st7032_dev_s *priv =
            (FAR struct st7032_dev_s *)inode->i_private;
          FAR struct slcd_curpos_s *attr =
            (FAR struct slcd_curpos_s *)((uintptr_t)arg);

          attr->row    = priv->row;
          attr->column = priv->col;
        }
        break;

      case SLCDIOC_GETBRIGHTNESS: /* Get the current brightness setting */
        {
          FAR struct inode *inode = filep->f_inode;
          FAR struct st7032_dev_s *priv =
            (FAR struct st7032_dev_s *)inode->i_private;

          nxsem_wait(&priv->sem_excl);
          *(FAR int *)((uintptr_t)arg) = 1; /* Hardcoded */
          nxsem_post(&priv->sem_excl);
        }
        break;

      case SLCDIOC_SETBRIGHTNESS: /* Set the brightness to a new value */
        {
          FAR struct inode *inode = filep->f_inode;
          FAR struct st7032_dev_s *priv =
            (FAR struct st7032_dev_s *)inode->i_private;

          nxsem_wait(&priv->sem_excl);

          /* TODO: set display contrast */

          nxsem_post(&priv->sem_excl);
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
 * Name: st7032_register
 *
 * Description:
 *   Register the ST7032 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             ST7032
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int st7032_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct st7032_dev_s *priv;
  int ret;

  /* Initialize the ST7032 device structure */

  priv = (FAR struct st7032_dev_s *)kmm_malloc(sizeof(struct st7032_dev_s));
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

  nxsem_init(&priv->sem_excl, 0, 1);

  /* Initialize the display */

  lcd_init(priv);

  /* Register the driver */

  ret = register_driver(devpath, &g_st7032fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SPI && CONFIG_ST7032 */

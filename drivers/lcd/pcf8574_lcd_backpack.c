/****************************************************************************
 * drivers/lcd/pcf8574_lcd_backpack.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <poll.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/ascii.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lcd/slcd_codec.h>
#include <nuttx/lcd/pcf8574_lcd_backpack.h>

#ifndef CONFIG_LIBC_SLCDCODEC
#  error please also select Library Routines, Segment LCD CODEC
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The PCF8574 is a 100 KHz device */

#define I2C_FREQ            100000

/* timing characteristics of the LCD interface */

#define DELAY_US_NYBBLE0    20
#define DELAY_US_NYBBLE1    10
#define DELAY_US_WRITE      40
#define DELAY_US_HOMECLEAR  2000

/* HD44780 commands */

#define CMD_CLEAR           0x01
#define CMD_HOME            0x02
#define CMD_CURSOR_ON_SOLID 0x0e
#define CMD_CURSOR_OFF      0x0c
#define CMD_CURSOR_ON_BLINK 0x0f
#define CMD_SET_CGADDR      0x40
#define CMD_SET_DDADDR      0x80

#define MAX_OPENCNT     (255)   /* Limit of uint8_t */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pcf8574_lcd_dev_s
{
  FAR struct i2c_master_s *i2c;              /* I2C interface */
  struct pcf8574_lcd_backpack_config_s cfg;  /* gpio configuration */
  uint8_t bl_bit;                            /* current backlight bit */
  uint8_t refs;                              /* Number of references */
  uint8_t unlinked;                          /* We are unlinked, so teardown
                                              * on last close */
  mutex_t lock;                              /* mutex */
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

/* Character driver methods */

static int pcf8574_lcd_open(FAR struct file *filep);
static int pcf8574_lcd_close(FAR struct file *filep);
static ssize_t pcf8574_lcd_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);
static ssize_t pcf8574_lcd_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen);
static off_t pcf8574_lcd_seek(FAR struct file *filep, off_t offset,
                              int whence);
static int pcf8574_lcd_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
static int pcf8574_lcd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int pcf8574_lcd_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pcf8574_lcd_fops =
{
  pcf8574_lcd_open,             /* open */
  pcf8574_lcd_close,            /* close */
  pcf8574_lcd_read,             /* read */
  pcf8574_lcd_write,            /* write */
  pcf8574_lcd_seek,             /* seek */
  pcf8574_lcd_ioctl,            /* ioctl */
  pcf8574_lcd_poll              /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , pcf8574_lcd_unlink          /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pca8574_write
 *
 * Description:
 *  primitive I2C write operation for the PCA8574, which is the IO expander
 *  device used on the board.  The board essentially byte-bangs the
 *  parallel interface in nybble mode much as one might with a conventional
 *  GPIO based interface.  The I2C interface simply sets the state of the
 *  8 IO lines to control the 4 data, 3 control, and one for backlight,
 *  signals.
 *
 ****************************************************************************/

static void pca8574_write(FAR struct pcf8574_lcd_dev_s *priv, uint8_t data)
{
  struct i2c_config_s config;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = I2C_FREQ;
  config.address = priv->cfg.addr;
  config.addrlen = 7;

  /* Write the value */

  ret = i2c_write(priv->i2c, &config, &data, 1);
  if (ret < 0)
    {
      lcdinfo("pca8574_write() failed: %d\n", ret);
      return;
    }

  return;
}

/****************************************************************************
 * Name: pca8574_read
 *
 * Description:
 *  primitive I2C read operation for the PCA8574, which is the IO expander
 *  device used on the board.  The PCF8574 is 'interesting' in that it
 *  doesn't really have a data direction register, but instead the outputs
 *  are current-limited when high, so by setting an IO line high, you are
 *  also making it an input.  Consequently, before using this method, you'll
 *  need to perform a pca8574_write() setting the bits you are interested in
 *  reading to 1's, then call this method.
 *
 ****************************************************************************/

static int pca8574_read(FAR struct pcf8574_lcd_dev_s *priv, uint8_t *data)
{
  struct i2c_config_s config;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = I2C_FREQ;
  config.address = priv->cfg.addr;
  config.addrlen = 7;

  /* Read the value */

  ret = i2c_read(priv->i2c, &config, data, 1);
  if (ret < 0)
    {
      lcdinfo("pca8574_read() failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lcd_backlight
 *
 * Description:
 *  turn on, or off, the LCD backlight
 *
 ****************************************************************************/

static void lcd_backlight(FAR struct pcf8574_lcd_dev_s *priv, bool blon)
{
  uint8_t data;

  data = ((blon && priv->cfg.bl_active_high) ||
          (!blon && !priv->cfg.bl_active_high)) ? (1 << priv->cfg.bl) : 0;
  pca8574_write(priv, data);
  priv->bl_bit = data;
}

/****************************************************************************
 * Name: rc2addr
 *
 * Description:
 *  This converts a row/column pair to a screen memory address.
 *
 ****************************************************************************/

static inline uint8_t rc2addr(FAR struct pcf8574_lcd_dev_s *priv,
                              uint8_t row, uint8_t col)
{
  if (row < 2)
    {
      /* 1 and 2 line displays are simple; line0 @ 0x00, line1 @ 0x40 */

      return row * 0x40 + col;
    }
  else
    {
      /* 4 line displays are interesting; third line really is a continuation
       * of first line, and fourth line is a continuation of second.
       */

      return (row - 2) * 0x40 + (col + priv->cfg.cols);
    }
}

/****************************************************************************
 * Name: addr2rc
 *
 * Description:
 *  This converts a screen memory address to a row/column pair.
 *
 ****************************************************************************/

static inline void addr2rc(FAR struct pcf8574_lcd_dev_s *priv, uint8_t addr,
                           FAR uint8_t *row, FAR uint8_t *col)
{
  *row = addr / 0x40;
  *col = addr % 0x40;

  if (*col >= priv->cfg.cols)
    {
      /* 4 line displays have third and fourth lines really as continuation
       * of the first and second.
       */

      *row += 2;
      *col -= priv->cfg.cols;
    }
}

/****************************************************************************
 * Name: prepare_nybble
 *
 * Description:
 *  This is a bit tedious, but scramble the bits of the nybble into position
 *  as per this board's particular wiring.  Most boards are either on the
 *  top four bits, or bottom four, so a shift would do typically in those
 *  cases, but this gives us ultimate flexibility.
 *
 ****************************************************************************/

uint8_t prepare_nybble(FAR struct pcf8574_lcd_dev_s *priv, uint8_t nybble)
{
  uint8_t lcddata = 0;

  if (nybble & 0x08)
    {
      lcddata |= (1 << priv->cfg.d7);
    }

  if (nybble & 0x04)
    {
      lcddata |= (1 << priv->cfg.d6);
    }

  if (nybble & 0x02)
    {
      lcddata |= (1 << priv->cfg.d5);
    }

  if (nybble & 0x01)
    {
      lcddata |= (1 << priv->cfg.d4);
    }

  return lcddata;
}

/****************************************************************************
 * Name: unprepare_nybble
 *
 * Description:
 *  This is the opposite of prepare_nybble(), and is used to unscramble bits
 *  when reading data from the display, as per board wiring.
 *
 ****************************************************************************/

uint8_t unprepare_nybble(FAR struct pcf8574_lcd_dev_s *priv, uint8_t lcddata)
{
  uint8_t data = 0;

  if (lcddata & (1 << priv->cfg.d7))
    {
      data |= 0x08;
    }

  if (lcddata & (1 << priv->cfg.d6))
    {
      data |= 0x04;
    }

  if (lcddata & (1 << priv->cfg.d5))
    {
      data |= 0x02;
    }

  if (lcddata & (1 << priv->cfg.d4))
    {
      data |= 0x01;
    }

  return data;
}

/****************************************************************************
 * Name: latch_nybble
 *
 * Description:
 *  Latch a nybble on the LCD bus.  This is done for each of two halves of a
 *  write operation in 4-bit mode.  The 'rs' param is false for command
 *  transfers, and true for data transfers.
 *
 ****************************************************************************/

static void latch_nybble(FAR struct pcf8574_lcd_dev_s *priv, uint8_t nybble,
                         bool rs)
{
  uint8_t lcddata;
  uint8_t en_bit;
  uint8_t rs_bit;

  en_bit = 1 << priv->cfg.en;
  rs_bit = rs ? (1 << priv->cfg.rs) : 0;

  /* Put the nybble, preserving backlight, reset R/~W and maybe RS */

  lcddata = prepare_nybble(priv, nybble) | priv->bl_bit | rs_bit;
  pca8574_write(priv, lcddata);

  /* Now set EN */

  lcddata |= en_bit;
  pca8574_write(priv, lcddata);
  up_udelay(DELAY_US_NYBBLE0);  /* setup */

  /* Latch on EN falling edge */

  lcddata &= ~en_bit;
  pca8574_write(priv, lcddata);
  up_udelay(DELAY_US_NYBBLE1);  /* hold */
}

/****************************************************************************
 * Name: load_nybble
 *
 * Description:
 *  Load a nybble from the LCD bus.  This is done for each of two halves of a
 *  read operation in 4-bit mode.  The 'rs' param is false for command
 *  transfers (the only one is to read status and the address register), and
 *  true for data transfers.
 *
 ****************************************************************************/

static uint8_t load_nybble(FAR struct pcf8574_lcd_dev_s *priv, bool rs)
{
  uint8_t lcddata;
  uint8_t en_bit;
  uint8_t rs_bit;
  uint8_t rw_bit;
  uint8_t data;

  en_bit = 1 << priv->cfg.en;
  rs_bit = rs ? (1 << priv->cfg.rs) : 0;
  rw_bit = 1 << priv->cfg.rw;

  /* Put highs on the data lines, preserve, set R/~W and maybe RS */

  lcddata = prepare_nybble(priv, 0x0f) | priv->bl_bit | rw_bit | rs_bit;
  pca8574_write(priv, lcddata);

  /* Now set EN */

  lcddata |= en_bit;
  pca8574_write(priv, lcddata);
  up_udelay(DELAY_US_NYBBLE0);  /* setup */

  /* Now read the data */

  pca8574_read(priv, &data);
  data = unprepare_nybble(priv, data);

  /* Transaction completed on EN falling edge */

  lcddata &= ~en_bit;
  pca8574_write(priv, lcddata);
  up_udelay(DELAY_US_NYBBLE1);  /* hold */

  return data;
}

/****************************************************************************
 * Name: lcd_putcmd
 *
 * Description:
 *  Write a command to the LCD.  Most of the time this is done in nybble
 *  mode in two phases, but in special cases (like initialization) we do not
 *  do two phases.
 *
 ****************************************************************************/

static void lcd_putcmd(FAR struct pcf8574_lcd_dev_s *priv, uint8_t data)
{
  latch_nybble(priv, data >> 4, false);
  latch_nybble(priv, data, false);
  up_udelay(DELAY_US_WRITE);
}

/****************************************************************************
 * Name: lcd_putdata
 *
 * Description:
 *  Write a byte to the LCD.  This is used both for screen data and for
 *  character generator data, depending on a previous command that selected
 *  which ever is the destination.
 *
 ****************************************************************************/

static inline void lcd_putdata(FAR struct pcf8574_lcd_dev_s *priv,
                               uint8_t data)
{
  latch_nybble(priv, data >> 4, true);
  latch_nybble(priv, data, true);
  up_udelay(DELAY_US_WRITE);
}

/****************************************************************************
 * Name: lcd_getdata
 *
 * Description:
 *  Read a data byte from the LCD.
 *
 ****************************************************************************/

static inline uint8_t lcd_getdata(FAR struct pcf8574_lcd_dev_s *priv)
{
  uint8_t data;

  data = (load_nybble(priv, true) << 4) | load_nybble(priv, true);
  return data;
}

/****************************************************************************
 * Name: lcd_getcmd
 *
 * Description:
 *  Read a command byte from the LCD.  There really is only one such read:
 *  get 'busy' status, and current address value.
 *
 ****************************************************************************/

static inline uint8_t lcd_getcmd(FAR struct pcf8574_lcd_dev_s *priv)
{
  uint8_t data;
  data = (load_nybble(priv, false) << 4) | load_nybble(priv, false);
  return data;
}

/****************************************************************************
 * Name: lcd_read_busy_addr
 *
 * Description:
 *  Read the busy flag, and, optionally, the current value of the address
 *  register (data or character generator dependent on a previous command).
 *
 ****************************************************************************/

static bool lcd_read_busy_addr(FAR struct pcf8574_lcd_dev_s *priv,
                               FAR uint8_t *addr)
{
  uint8_t data = lcd_getcmd(priv);

  if (NULL != addr)
    {
      *addr = data & 0x7f;
    }

  return (data & 0x80) ? true : false;
}

/****************************************************************************
 * Name: lcd_init
 *
 * Description:
 *  perform the initialization sequence to get the LCD into a known state.
 *
 ****************************************************************************/

static void lcd_init(FAR struct pcf8574_lcd_dev_s *priv)
{
  /* Wait for more than 15 ms after Vcc for the LCD to stabilize */

  nxsig_usleep(50000);

  /* Perform the init sequence.  This sequence of commands is constructed so
   * that it will get the device into nybble mode irrespective of what state
   * the device is currently in (could be 8 bit, 4 bit nyb 0, 4 bit nyb 1).
   * By sending the 'set 8-bit mode' three times, we will definitely end up
   * in 8 bit mode, and then we can reliably transition to 4 bit mode for
   * the remainder of operations.
   */

  /* Send Command 0x30, set 8-bit mode, and wait > 4.1 ms */

  latch_nybble(priv, 0x30 >> 4, false);
  nxsig_usleep(5000);

  /* Send Command 0x30, set 8-bit mode, and wait > 100 us */

  latch_nybble(priv, 0x30 >> 4, false);
  nxsig_usleep(5000);

  /* Send Command 0x30, set 8-bit mode */

  latch_nybble(priv, 0x30 >> 4, false);
  nxsig_usleep(200);

  /* now Function set: Set interface to be 4 bits long (only 1 cycle write
   * for the first time).
   */

  latch_nybble(priv, 0x20 >> 4, false);
  nxsig_usleep(5000);

  /* Function set: DL=0;Interface is 4 bits, N=1 (2 Lines), F=0 (5x8 dots
   * font)
   */

  lcd_putcmd(priv, 0x28);

  /* Display Off: D=0 (Display off), C=0 (Cursor Off), B=0 (Blinking Off) */

  lcd_putcmd(priv, 0x08);

  /* Display Clear */

  lcd_putcmd(priv, CMD_CLEAR);
  up_udelay(DELAY_US_HOMECLEAR);        /* clear needs extra time */

  /* Entry Mode Set: I/D=1 (Increment), S=0 (No shift) */

  lcd_putcmd(priv, 0x06);

  /* Display On, Cursor Off */

  lcd_putcmd(priv, 0x0c);
}

/****************************************************************************
 * Name: lcd_create_char
 *
 * Description:
 *   This creates a custom character pattern.  There can be 8 5x8 patterns.
 *   The bitmap proceeds top to bottom, msb-lsb, and is right justified (i.e.
 *   only bits 4-0 are used).  By convention, you are meant to always leave
 *   the last line (byte) zero so that the cursor can use this line, but this
 *   is not strictly required.
 *
 * Input Parameters:
 *   priv     - device instance
 *   idxchar  - which character is being imaged; 0 - 7
 *   chardata - the character image bitmap; must be 8 bytes always
 *
 ****************************************************************************/

static void lcd_create_char(FAR struct pcf8574_lcd_dev_s *priv,
                            uint8_t idxchar, FAR const uint8_t *chardata)
{
  int nidx;
  uint8_t addr;

  lcd_read_busy_addr(priv, &addr);
  lcd_putcmd(priv, CMD_SET_CGADDR | (idxchar << 3));    /* set CGRAM address */

  for (nidx = 0; nidx < 8; ++nidx)
    {
      lcd_putdata(priv, chardata[nidx]);
    }

  lcd_putcmd(priv, CMD_SET_DDADDR | addr);      /* restore DDRAM address */
}

/****************************************************************************
 * Name: lcd_set_curpos
 *
 * Description:
 *  This sets the cursor position based on row, column addressing.
 *
 * Input Parameters:
 *  priv - device instance
 *  row - row position
 *  col - column position
 *
 ****************************************************************************/

static void lcd_set_curpos(FAR struct pcf8574_lcd_dev_s *priv,
                           uint8_t row, uint8_t col)
{
  uint8_t addr;

  addr = rc2addr(priv, row, col);
  lcd_putcmd(priv, CMD_SET_DDADDR | addr);      /* set DDRAM address */
}

/****************************************************************************
 * Name: lcd_get_curpos
 *
 * Description:
 *  This gets the cursor position based on row, column addressing.
 *
 * Input Parameters:
 *  priv - device instance
 *  row - row position
 *  col - column position
 *
 ****************************************************************************/

static void lcd_get_curpos(FAR struct pcf8574_lcd_dev_s *priv,
                           FAR uint8_t *row, FAR uint8_t *col)
{
  uint8_t addr;

  lcd_read_busy_addr(priv, &addr);
  addr2rc(priv, addr, row, col);
}

/****************************************************************************
 * Name: lcd_scroll_up
 *
 * Description:
 *  Scroll the display up, and clear the new (last) line.
 *
 ****************************************************************************/

static void lcd_scroll_up(FAR struct pcf8574_lcd_dev_s *priv)
{
  FAR uint8_t *data;
  int nrow;
  int ncol;

  data = (FAR uint8_t *)kmm_malloc(priv->cfg.cols);
  if (NULL == data)
    {
      lcdinfo("Failed to allocate buffer in lcd_scroll_up()\n");
      return;
    }

  for (nrow = 1; nrow < priv->cfg.rows; ++nrow)
    {
      lcd_set_curpos(priv, nrow, 0);
      for (ncol = 0; ncol < priv->cfg.cols; ++ncol)
        {
          data[ncol] = lcd_getdata(priv);
        }

      lcd_set_curpos(priv, nrow - 1, 0);
      for (ncol = 0; ncol < priv->cfg.cols; ++ncol)
        {
          lcd_putdata(priv, data[ncol]);
        }
    }

  lcd_set_curpos(priv, priv->cfg.rows - 1, 0);
  for (ncol = 0; ncol < priv->cfg.cols; ++ncol)
    {
      lcd_putdata(priv, ' ');
    }

  lcd_set_curpos(priv, priv->cfg.rows - 1, 0);

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

static void lcd_codec_action(FAR struct pcf8574_lcd_dev_s *priv,
                             enum slcdcode_e code, uint8_t count)
{
  switch (code)
    {
      /* Erasure */

    case SLCDCODE_BACKDEL:  /* Backspace (backward delete) N characters */
      {
        if (count <= 0)  /* silly case */
          {
            break;
          }
        else
          {
            uint8_t row;
            uint8_t col;

            lcd_get_curpos(priv, &row, &col);
            if (count > col)    /* saturate to preceding columns available */
              {
                count = col;
              }

            lcd_set_curpos(priv, row, col - count);
          }

        /* ... and conscientiously fall through to next case ... */
      }

    case SLCDCODE_FWDDEL:  /* Delete (forward delete) N characters, moving text */
      {
        if (count <= 0)  /* silly case */
          {
            break;
          }
        else
          {
            uint8_t row;
            uint8_t col;
            uint8_t start;
            uint8_t end;
            uint8_t nidx;
            uint8_t data;

            lcd_get_curpos(priv, &row, &col);
            start = col + count;

            if (start >= priv->cfg.cols)  /* silly case of nothing left */
              {
                break;
              }

            end = start + count;
            if (end > priv->cfg.cols)   /* saturate */
              {
                end = priv->cfg.cols;
              }

            for (nidx = col; nidx < end; ++start, ++nidx)  /* much like memmove */
              {
                lcd_set_curpos(priv, row, start);
                data = lcd_getdata(priv);
                lcd_set_curpos(priv, row, nidx);
                lcd_putdata(priv, data);
              }

            for (; nidx < priv->cfg.cols; ++nidx)  /* much like memset */
              {
                lcd_putdata(priv, ' ');
              }

            lcd_set_curpos(priv, row, col);
          }
      }
      break;

    case SLCDCODE_ERASE:  /* Erase N characters from the cursor position */
      if (count > 0)
        {
          uint8_t row;
          uint8_t col;
          uint8_t end;
          uint8_t nidx;

          lcd_get_curpos(priv, &row, &col);
          end = col + count;
          if (end > priv->cfg.cols)
            {
              end = priv->cfg.cols;
            }

          for (nidx = col; nidx < end; ++nidx)
            {
              lcd_putdata(priv, ' ');
            }

          lcd_set_curpos(priv, row, col);
        }
      break;

    case SLCDCODE_CLEAR:  /* Home the cursor and erase the entire display */
      {
        lcd_putcmd(priv, CMD_CLEAR);
        up_udelay(DELAY_US_HOMECLEAR);  /* clear needs extra time */
      }
      break;

    case SLCDCODE_ERASEEOL:  /* Erase from the cursor position to the end of
                              * line */
      {
        uint8_t row;
        uint8_t col;
        uint8_t nidx;

        lcd_get_curpos(priv, &row, &col);

        for (nidx = col; nidx < priv->cfg.cols; ++nidx)
          {
            lcd_putdata(priv, ' ');
          }

        lcd_set_curpos(priv, row, col);
      }
      break;

      /* Cursor movement */

    case SLCDCODE_LEFT:  /* Cursor left by N characters */
      {
        uint8_t row;
        uint8_t col;

        lcd_get_curpos(priv, &row, &col);
        if (count > col)
          {
            col = 0;
          }
        else
          {
            col -= count;
          }

        lcd_set_curpos(priv, row, col);
      }
      break;

    case SLCDCODE_RIGHT:  /* Cursor right by N characters */
      {
        uint8_t row;
        uint8_t col;

        lcd_get_curpos(priv, &row, &col);
        col += count;
        if (col >= priv->cfg.cols)
          {
            col = priv->cfg.cols - 1;
          }

        lcd_set_curpos(priv, row, col);
      }
      break;

    case SLCDCODE_UP:  /* Cursor up by N lines */
      {
        uint8_t row;
        uint8_t col;

        lcd_get_curpos(priv, &row, &col);
        if (count > row)
          {
            row = 0;
          }
        else
          {
            row -= count;
          }

        lcd_set_curpos(priv, row, col);
      }
      break;

    case SLCDCODE_DOWN:  /* Cursor down by N lines */
      {
        uint8_t row;
        uint8_t col;

        lcd_get_curpos(priv, &row, &col);
        row += count;
        if (row >= priv->cfg.rows)
          {
            row = priv->cfg.rows - 1;
          }

        lcd_set_curpos(priv, row, col);
      }
      break;

    case SLCDCODE_HOME:  /* Cursor home */
      {
        uint8_t row;
        uint8_t col;

        lcd_get_curpos(priv, &row, &col);
        lcd_set_curpos(priv, row, 0);
      }
      break;

    case SLCDCODE_END:  /* Cursor end */
      {
        uint8_t row;
        uint8_t col;

        lcd_get_curpos(priv, &row, &col);
        lcd_set_curpos(priv, row, priv->cfg.cols - 1);
      }
      break;

    case SLCDCODE_PAGEUP:    /* Cursor up by N pages */
    case SLCDCODE_PAGEDOWN:  /* Cursor down by N pages */
      break;                 /* Not supportable on this SLCD */

      /* Blinking */

    case SLCDCODE_BLINKSTART:  /* Start blinking with current cursor position */
      lcd_putcmd(priv, CMD_CURSOR_ON_BLINK);
      break;

    case SLCDCODE_BLINKEND:  /* End blinking after the current cursor
                              * position */
    case SLCDCODE_BLINKOFF:  /* Turn blinking off */
      lcd_putcmd(priv, CMD_CURSOR_OFF);
      break;                 /* Not implemented */

      /* These are actually unreportable errors */

    default:
    case SLCDCODE_NORMAL:  /* Not a special keycode */
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
 * Name: lcd_fpos_to_curpos
 *
 * Description:
 *   Convert a file logical offset to a screen cursor pos (row,col).  This
 *   discounts 'synthesized' line feeds at the end of screen lines.
 *
 ****************************************************************************/

static void lcd_fpos_to_curpos(FAR struct pcf8574_lcd_dev_s *priv,
                               off_t fpos, FAR uint8_t *row,
                               FAR uint8_t *col, FAR bool *onlf)
{
  int virtcols;

  virtcols = (priv->cfg.cols + 1);

  /* Determine if this is a 'virtual' position (on the synthetic LF) */

  *onlf = (priv->cfg.cols == fpos % virtcols);

  /* Adjust off any preceding synthetic LF's to get linear position */

  fpos -= fpos / virtcols;

  /* Compute row/col from linear position */

  *row = fpos / priv->cfg.cols;
  *col = fpos % priv->cfg.cols;
}

/****************************************************************************
 * Name: lcd_curpos_to_fpos
 *
 * Description:
 *   Convert a screen cursor pos (row,col) to a file logical offset.  This
 *   includes 'synthesized' line feeds at the end of screen lines.
 *
 ****************************************************************************/

static void lcd_curpos_to_fpos(FAR struct pcf8574_lcd_dev_s *priv,
                               uint8_t row, uint8_t col, FAR off_t *fpos)
{
  /* the logical file position is the linear position plus any synthetic LF */

  *fpos = (row * priv->cfg.cols) + col + row;
}

/****************************************************************************
 * Name: pcf8574_lcd_open
 *
 * Description:
 *   requisite device 'open' method; we don't do anything special
 *
 ****************************************************************************/

static int pcf8574_lcd_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pcf8574_lcd_dev_s *priv =
    (FAR struct pcf8574_lcd_dev_s *)inode->i_private;

  /* Increment the reference count */

  nxmutex_lock(&priv->lock);
  if (priv->refs == MAX_OPENCNT)
    {
      return -EMFILE;
    }
  else
    {
      priv->refs++;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: pcf8574_lcd_close
 *
 * Description:
 *   requisite device 'close' method; we don't do anything special
 *
 ****************************************************************************/

static int pcf8574_lcd_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pcf8574_lcd_dev_s *priv =
    (FAR struct pcf8574_lcd_dev_s *)inode->i_private;
  int ret;

  /* Decrement the reference count */

  nxmutex_lock(&priv->lock);

  if (priv->refs == 0)
    {
      ret = -EIO;
    }
  else
    {
      priv->refs--;

      /* If we had previously unlinked, but there were open references at
       * the time, we need to do the final teardown now.
       */

      if (priv->refs == 0 && priv->unlinked)
        {
          /* We have no real teardown at present */
        }

      ret = OK;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: pcf8574_lcd_read
 *
 * Description:
 *  This simply reads as much of the display memory as possible.  This is
 *  generally not very interesting, but we do it in a way that allows us to
 *  'cat' the LCD contents via the shell.
 *
 ****************************************************************************/

static ssize_t pcf8574_lcd_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pcf8574_lcd_dev_s *priv =
    (FAR struct pcf8574_lcd_dev_s *)inode->i_private;
  int nidx;
  uint8_t addr;
  uint8_t row;
  uint8_t col;
  bool onlf;

  nxmutex_lock(&priv->lock);

  /* Get current cursor position so we can restore it */

  lcd_read_busy_addr(priv, &addr);

  /* Convert file position to row/col address and position DDADDR there */

  lcd_fpos_to_curpos(priv, filep->f_pos, &row, &col, &onlf);
  lcd_set_curpos(priv, row, col);

  /* Read as much of the display as possible */

  nidx = 0;
  while (nidx < buflen && row < priv->cfg.rows)
    {
      /* Synthesize end-of-line LF and advance to start of next row */

      if (onlf)
        {
          /* Synthesize LF for all but last row */

          if (row < priv->cfg.rows - 1)
            {
              buffer[nidx] = '\x0a';
              onlf = false;
              ++filep->f_pos;
              ++nidx;
            }

          ++row;
          col = 0;
          continue;
        }

      /* If we are at start of line we will need to update DDRAM address */

      if (0 == col)
        {
          lcd_set_curpos(priv, row, 0);
        }

      buffer[nidx] = lcd_getdata(priv);

      ++filep->f_pos;
      ++nidx;
      ++col;

      /* If we are now at the end of a line, we setup for the synthetic LF */

      if (priv->cfg.cols == col)
        {
          onlf = true;
        }
    }

  lcd_putcmd(priv, CMD_SET_DDADDR | addr);      /* Restore DDRAM address */

  nxmutex_unlock(&priv->lock);
  return nidx;
}

/****************************************************************************
 * Name: pcf8574_lcd_write
 *
 * Description:
 *   Output a sequence of characters to the device.
 *
 ****************************************************************************/

static ssize_t pcf8574_lcd_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pcf8574_lcd_dev_s *priv =
    (FAR struct pcf8574_lcd_dev_s *)inode->i_private;
  struct lcd_instream_s instream;
  uint8_t row;
  uint8_t col;
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

  /* Get the current cursor position now; we'll keep track of it as we go */

  lcd_get_curpos(priv, &row, &col);

  /* Now decode and process every byte in the input buffer */

  memset(&state, 0, sizeof(struct slcdstate_s));
  while ((result =
          slcd_decode(&instream.stream, &state, &ch, &count)) != SLCDRET_EOF)
    {
      if (result == SLCDRET_CHAR)       /* A normal character was returned */
        {
          /* Check for ASCII control characters */

          if (ch == ASCII_TAB)
            {
              lcd_putcmd(priv, CMD_CURSOR_ON_BLINK);
            }
          else if (ch == ASCII_VT)
            {
              /* Turn the backlight on */

              lcd_backlight(priv, true);
            }
          else if (ch == ASCII_FF)
            {
              /* Turn the backlight off */

              lcd_backlight(priv, false);
            }
          else if (ch == ASCII_CR)
            {
              /* Perform a Home */

              lcd_putcmd(priv, CMD_HOME);
              up_udelay(DELAY_US_HOMECLEAR);    /* home needs extra time */
              row = 0;
              col = 0;
            }
          else if (ch == ASCII_SO)
            {
              lcd_putcmd(priv, CMD_CURSOR_OFF);
            }
          else if (ch == ASCII_SI)
            {
              /* Perform the re-initialize */

              lcd_init(priv);
              row = 0;
              col = 0;
            }
          else if (ch == ASCII_LF)
            {
              /* unixian line term; go to start of next line */

              row += 1;
              if (row >= priv->cfg.rows)
                {
                  lcd_scroll_up(priv);
                  row = priv->cfg.rows - 1;
                }

              col = 0;
              lcd_set_curpos(priv, row, col);
            }
          else if (ch == ASCII_BS)
            {
              /* Perform the backward deletion */

              lcd_codec_action(priv, SLCDCODE_BACKDEL, 1);

              lcd_get_curpos(priv, &row, &col);
            }
          else if (ch == ASCII_DEL)
            {
              /* Perform the forward deletion */

              lcd_codec_action(priv, SLCDCODE_FWDDEL, 1);

              lcd_get_curpos(priv, &row, &col);
            }
          else
            {
              /* All others are fair game.  See if we need to wrap line. */

              if (col >= priv->cfg.cols)
                {
                  row += 1;
                  if (row >= priv->cfg.rows)
                    {
                      lcd_scroll_up(priv);
                      row = priv->cfg.rows - 1;
                    }

                  col = 0;
                  lcd_set_curpos(priv, row, col);
                }

              lcd_putdata(priv, ch);
              ++col;
            }
        }

      /* A special SLCD action was returned */

      else  /* (result == SLCDRET_SPEC) */
        {
          lcd_codec_action(priv, (enum slcdcode_e)ch, count);

          /* we can't know what happened, so it's easier just to re-inquire
           * as to where we are.
           */

          lcd_get_curpos(priv, &row, &col);
        }
    }

  /* Wherever we wound up, update our logical file pos to reflect it */

  lcd_curpos_to_fpos(priv, row, col, &filep->f_pos);

  nxmutex_unlock(&priv->lock);
  return buflen;
}

/****************************************************************************
 * Name: pcf8574_lcd_seek
 *
 * Description:
 *   Seek the logical file pointer to the specified position.  This is
 *   probably not very interesting except possibly for (SEEK_SET, 0) to
 *   rewind the pointer for a subsequent read().
 *   The file pointer is logical, and includes synthesized LF chars at the
 *   end of the display lines.
 *
 ****************************************************************************/

static off_t pcf8574_lcd_seek(FAR struct file *filep, off_t offset,
                              int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pcf8574_lcd_dev_s *priv =
    (FAR struct pcf8574_lcd_dev_s *)inode->i_private;
  off_t pos;
  int maxpos;

  nxmutex_lock(&priv->lock);

  maxpos = priv->cfg.rows * priv->cfg.cols + (priv->cfg.rows - 1);
  pos = filep->f_pos;

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

      pos = (off_t) - EINVAL;
      break;
    }

  nxmutex_unlock(&priv->lock);
  return pos;
}

/****************************************************************************
 * Name: pcf8574_lcd_ioctl
 *
 * Description:
 *   Perform device operations that are outside the standard I/O model.
 *
 ****************************************************************************/

static int pcf8574_lcd_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  switch (cmd)
    {
    case SLCDIOC_GETATTRIBUTES:  /* SLCDIOC_GETATTRIBUTES: Get the
                                  * attributes of the SLCD */
      {
        FAR struct inode *inode = filep->f_inode;
        FAR struct pcf8574_lcd_dev_s *priv =
          (FAR struct pcf8574_lcd_dev_s *)inode->i_private;
        FAR struct slcd_attributes_s *attr =
          (FAR struct slcd_attributes_s *)((uintptr_t) arg);

        lcdinfo("SLCDIOC_GETATTRIBUTES:\n");

        if (!attr)
          {
            return -EINVAL;
          }

        attr->nrows = priv->cfg.rows;
        attr->ncolumns = priv->cfg.cols;
        attr->nbars = 0;
        attr->maxcontrast = 0;
        attr->maxbrightness = 1;  /* 'brightness' for us is the backlight */
      }
      break;

    case SLCDIOC_CURPOS:  /* SLCDIOC_CURPOS: Get the SLCD cursor position */
      {
        FAR struct inode *inode = filep->f_inode;
        FAR struct pcf8574_lcd_dev_s *priv =
          (FAR struct pcf8574_lcd_dev_s *)inode->i_private;
        FAR struct slcd_curpos_s *attr =
          (FAR struct slcd_curpos_s *)((uintptr_t) arg);
        uint8_t row;
        uint8_t col;

        nxmutex_lock(&priv->lock);

        lcd_get_curpos(priv, &row, &col);
        attr->row = row;
        attr->column = col;

        nxmutex_unlock(&priv->lock);
      }
      break;

    case SLCDIOC_GETBRIGHTNESS:  /* Get the current brightness setting */
      {
        FAR struct inode *inode = filep->f_inode;
        FAR struct pcf8574_lcd_dev_s *priv =
          (FAR struct pcf8574_lcd_dev_s *)inode->i_private;
        bool bon;

        bon = (priv->bl_bit && priv->cfg.bl_active_high) ||
              (!priv->bl_bit && !priv->cfg.bl_active_high);
        *(FAR int *)((uintptr_t) arg) = bon ? 1 : 0;
      }
      break;

    case SLCDIOC_SETBRIGHTNESS:  /* Set the brightness to a new value */
      {
        FAR struct inode *inode = filep->f_inode;
        FAR struct pcf8574_lcd_dev_s *priv =
          (FAR struct pcf8574_lcd_dev_s *)inode->i_private;

        nxmutex_lock(&priv->lock);
        lcd_backlight(priv, arg ? true : false);
        nxmutex_unlock(&priv->lock);
      }
      break;

    case SLCDIOC_CREATECHAR:  /* Create a custom character pattern */
      {
        FAR struct inode *inode = filep->f_inode;
        FAR struct pcf8574_lcd_dev_s *priv =
          (FAR struct pcf8574_lcd_dev_s *)inode->i_private;
        FAR struct slcd_createchar_s *attr =
          (FAR struct slcd_createchar_s *)((uintptr_t) arg);

        nxmutex_lock(&priv->lock);
        lcd_create_char(priv, attr->idx, attr->bmp);
        nxmutex_unlock(&priv->lock);
      }
      break;

    case SLCDIOC_SETBAR:       /* SLCDIOC_SETBAR: Set bars on a bar display */
    case SLCDIOC_GETCONTRAST:  /* SLCDIOC_GETCONTRAST: Get the current
                                * contrast setting */
    case SLCDIOC_SETCONTRAST:  /* SLCDIOC_SETCONTRAST: Set the contrast to a
                                * new value */
    default:
      return -ENOTTY;
    }

  return OK;
}

/****************************************************************************
 * Name: pcf8574_lcd_poll
 ****************************************************************************/

static int pcf8574_lcd_poll(FAR struct file *filep, FAR struct pollfd *fds,
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
 * Name: pcf8574_lcd_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int pcf8574_lcd_unlink(FAR struct inode *inode)
{
  FAR struct pcf8574_lcd_dev_s *priv =
    (FAR struct pcf8574_lcd_dev_s *)inode->i_private;
  int ret = OK;

  nxmutex_lock(&priv->lock);

  priv->unlinked = true;

  /* If there are no open references to the driver then tear it down now */

  if (priv->refs == 0)
    {
      /* We have no real teardown at present */

      ret = OK;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcf8574_lcd_backpack_register
 *
 * Description:
 *  Register a character driver that is an I2C LCD 'backpack' for the
 *  ever-popular HD44780 based 16x2 LCD via pcf8574 I2C IO expander.
 *
 ****************************************************************************/

int pcf8574_lcd_backpack_register(FAR const char *devpath,
                                  FAR struct i2c_master_s *i2c,
                                  FAR struct pcf8574_lcd_backpack_config_s
                                  *cfg)
{
  FAR struct pcf8574_lcd_dev_s *priv;
  int ret;

  /* Sanity check on geometry */

  if (cfg->rows < 1 || cfg->rows > 4)
    {
      lcdinfo("Display rows must be 1-4\n");
      return -EINVAL;
    }

  if ((cfg->cols < 1 || cfg->cols > 64) ||
      (cfg->rows == 4 && cfg->cols > 32))
    {
      lcdinfo("Display cols must be 1-64, and may not be part of a 4x40 "
              "configuration\n");
      return -EINVAL;
    }

  /* Initialize the device structure */

  priv = (FAR struct pcf8574_lcd_dev_s *)
           kmm_malloc(sizeof(struct pcf8574_lcd_dev_s));
  if (!priv)
    {
      lcdinfo("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->cfg = *cfg;
  priv->bl_bit = priv->cfg.bl_active_high ? 0 : (1 << priv->cfg.bl);
  priv->refs = 0;
  priv->unlinked = false;
  nxmutex_init(&priv->lock);

  /* Initialize */

  lcd_init(priv);

  /* If SLCD is console enable the backlight from start */

#ifdef CONFIG_SLCD_CONSOLE
  lcd_backlight(priv, true);
#endif

  /* Register the character driver */

  ret = register_driver(devpath, &g_pcf8574_lcd_fops, 0666, priv);
  if (ret < 0)
    {
      lcdinfo("Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
    }

  lcdinfo("pcf8574_lcd_backpack driver loaded successfully!\n");
  return ret;
}

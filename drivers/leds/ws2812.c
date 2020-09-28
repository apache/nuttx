/****************************************************************************
 * drivers/sensors/ws2812.c
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/leds/ws2812.h>

#ifdef CONFIG_WS2812

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These are the tuning parameters to meeting timeing requirements */

#if CONFIG_WS2812_FREQUENCY == 4000000
#  define WS2812_RST_CYCLES 30          /* 60us (>50us)*/
#  define WS2812_ZERO_BYTE  0b01000000  /* 250 ns (200ns - 500ns) */
#  define WS2812_ONE_BYTE   0b01110000  /* 750 ns (550ns - 850ns) */
#elif CONFIG_WS2812_FREQUENCY == 8000000
#  define WS2812_RST_CYCLES 60          /* 60us (>50us)*/
#  define WS2812_ZERO_BYTE  0b01100000  /* 250 ns (200ns - 500ns) */
#  define WS2812_ONE_BYTE   0b01111100  /* 750 ns (550ns - 850ns) */
#else
#  error "Unsupported SPI Frequency"
#endif

#define WS2812_BYTES_PER_LED  8 * 3
#define WS2812_RW_PIXEL_SIZE  4

/* Transmit buffer looks like:
 * [<----reset bytes---->|<-RGBn->...<-RGB0->|<----reset bytes---->]
 *
 * It is important that this is shipped as close to one chunk as possible
 * in order to meet timeing requirements and to keep MOSI from going high
 * between transactions.  Some chips will leave MOSI at the state of the
 * MSB of the last byte for this reason it is recommended to shift the
 * bytes that represents the zero and one so that the MSB is 1. The reset
 * clocks will pad the shortend low at the end.
 */

#define TXBUFF_SIZE(n) (WS2812_RST_CYCLES * 2 + n * WS2812_BYTES_PER_LED)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ws2812_dev_s
{
  FAR struct spi_dev_s *spi;  /* SPI interface */
  uint16_t nleds;             /* Number of addressable LEDs */
  uint8_t *tx_buf;            /* Buffer for write transaction and state */
  sem_t exclsem;              /* Assures exclusive access to the driver */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void ws2812_configspi(FAR struct spi_dev_s *spi);
static void ws2812_pack(FAR uint8_t *buf, uint32_t rgb);

/* Character driver methods */

static int     ws2812_open(FAR struct file *filep);
static int     ws2812_close(FAR struct file *filep);
static ssize_t ws2812_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t ws2812_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static off_t   ws2812_seek(FAR struct file *filep, off_t offset, int whence);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ws2812fops =
{
  ws2812_open,    /* open */
  ws2812_close,   /* close */
  ws2812_read,    /* read */
  ws2812_write,   /* write */
  ws2812_seek,    /* seek */
  NULL,           /* ioctl */
  NULL,           /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL            /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ws2812_configspi
 *
 * Description:
 *   Set the SPI bus configuration
 *
 ****************************************************************************/

static inline void ws2812_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the WS2812
   * There is no CS on this device we just use MOSI and it is exclusive
   */

  SPI_LOCK(spi, true);  /* Exclusive use of the bus */
  SPI_SETMODE(spi, SPIDEV_MODE3);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_WS2812_FREQUENCY);
}

/****************************************************************************
 * Name: ws2812_pack
 *
 * Description:
 *   This writes the expanded SPI transaction to the transaction buffer
 *   for a given 24bit RGB value.
 *
 * Input Parameters:
 *   buf - The location in the transmit buffer to write.
 *   rgb - A 24bit RGB color 8bit red, 8-bit green, 8-bit blue
 *
 ****************************************************************************/

static void ws2812_pack(FAR uint8_t *buf, uint32_t rgb)
{
  uint8_t bit_idx;
  uint8_t byte_idx;
  uint8_t offset = 0;
  uint8_t color;
  uint32_t grb;

  grb = (rgb & 0x00ff00) << 8;
  grb |= (rgb & 0xff0000) >> 8;
  grb |= rgb & 0x0000ff;

  for (byte_idx = 0; byte_idx < 3; byte_idx++)
    {
      color = (uint8_t)(grb >> (8 * (2 - byte_idx)));
      for (bit_idx = 0; bit_idx < 8; bit_idx++)
        {
          if (color & (1 << (7 - bit_idx)))
            {
              buf[offset] = WS2812_ONE_BYTE;
            }
          else
            {
              buf[offset] = WS2812_ZERO_BYTE;
            }

          offset++;
        }
    }
}

/****************************************************************************
 * Name: ws2812_open
 *
 * Description:
 *   This function is called whenever the WS2812 device is opened.
 *
 ****************************************************************************/

static int ws2812_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ws2812_close
 *
 * Description:
 *   This routine is called when the WS2812 device is closed.
 *
 ****************************************************************************/

static int ws2812_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ws2812_read
 ****************************************************************************/

static ssize_t ws2812_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ws2812_write
 *
 * Description:
 *   This routine is called when writing to the WS2812 device. Data buffer
 *   should be an array of 32bit values holding 24bits of color information
 *   in host byte ordering 0x**rrggbb.
 *
 ****************************************************************************/

static ssize_t ws2812_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ws2812_dev_s *priv = inode->i_private;
  FAR uint8_t *tx_pixel;
  FAR uint32_t *pixel_buf = (FAR uint32_t *)buffer;
  size_t cur_led;
  size_t start_led;
  size_t end_led;
  size_t written = 0;

  if (buffer == NULL)
    {
      lederr("ERROR: Buffer is null\n");
      set_errno(EINVAL);
      return -1;
    }

  /* We need at least one display, so 1 byte */

  if (buflen < 1)
    {
      lederr("ERROR: You need to control at least 1 LED!\n");
      set_errno(EINVAL);
      return -1;
    }

  if ((buflen % WS2812_RW_PIXEL_SIZE) != 0)
    {
      lederr("ERROR: LED values must be 24bit packed in 32bit\n");
      set_errno(EINVAL);
      return -1;
    }

  nxsem_wait(&priv->exclsem);

  start_led = filep->f_pos / WS2812_RW_PIXEL_SIZE;
  tx_pixel = priv->tx_buf + WS2812_RST_CYCLES + \
             start_led * WS2812_BYTES_PER_LED;

  end_led = start_led + (buflen / WS2812_RW_PIXEL_SIZE) - 1;
  ledinfo("Start: %d End: %d\n", start_led, end_led);

  if (end_led  > (priv->nleds -1))
    {
      end_led = priv->nleds - 1;
    }

  for (cur_led = start_led; cur_led <= end_led; cur_led++)
    {
      ws2812_pack(tx_pixel, *pixel_buf & 0xffffff);
      pixel_buf++;
      tx_pixel += WS2812_BYTES_PER_LED;
      written += WS2812_RW_PIXEL_SIZE;
    }

  SPI_SNDBLOCK(priv->spi, priv->tx_buf, TXBUFF_SIZE(priv->nleds));

  /* Update LED position and handle case were we wrote the last LED */

  filep->f_pos += written;
  if (end_led == (priv->nleds - 1))
    {
      filep->f_pos -= WS2812_RW_PIXEL_SIZE;
    }

  nxsem_post(&priv->exclsem);

  return written;
}

/****************************************************************************
 * Name: ws2812_seek
 *
 * Description:
 *   This routine is called when seeking the WS2812 device. This can be used
 *   to address the starting LED to write.  This should be done on a full
 *   color boundary which is 32bits. e.g. LED0 - offset 0 LED 8.
 *
 ****************************************************************************/

static off_t ws2812_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ws2812_dev_s *priv = inode->i_private;

  off_t maxpos;
  off_t pos;

  if ((offset % WS2812_RW_PIXEL_SIZE) != 0)
    {
      return (off_t)-EINVAL;
    }

  nxsem_wait(&priv->exclsem);

  maxpos = (priv->nleds - 1) * WS2812_RW_PIXEL_SIZE;
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

  nxsem_post(&priv->exclsem);
  return pos;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ws2812_leds_register
 *
 * Description:
 *   Register the WS2812 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/leds0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             WS2812
 *   nleds   - Number of addressable LEDs
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ws2812_leds_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                         uint16_t nleds)
{
  FAR struct ws2812_dev_s *priv;
  int ret;
  int led;

  /* Initialize the WS2812 device structure */

  priv = (FAR struct ws2812_dev_s *)kmm_malloc(sizeof(struct ws2812_dev_s));
  if (!priv)
    {
      lederr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->nleds = nleds;
  priv->tx_buf = (FAR uint8_t *)kmm_zalloc(TXBUFF_SIZE(priv->nleds));
  if (!priv->tx_buf)
    {
      lederr("ERROR: Failed to allocate tx buffer\n");
      kmm_free(priv);
      return -ENOMEM;
    }

  /* Mark LED section of TX buffer as off */

  for (led = 0; led < priv->nleds; led++)
    {
      ws2812_pack(
        priv->tx_buf + WS2812_RST_CYCLES + led * WS2812_BYTES_PER_LED,
        0);
    }

  priv->spi = spi;
  ws2812_configspi(priv->spi);

  nxsem_init(&priv->exclsem, 0, 1);

  SPI_SNDBLOCK(priv->spi, priv->tx_buf, TXBUFF_SIZE(priv->nleds));

  /* Register the character driver */

  ret = register_driver(devpath, &g_ws2812fops, 0666, priv);
  if (ret < 0)
    {
      lederr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv->tx_buf);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_WS2812 */

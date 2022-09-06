/****************************************************************************
 * drivers/leds/ws2812.c
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
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/ws2812.h>

#ifndef CONFIG_WS2812_NON_SPI_DRIVER
#include <nuttx/spi/spi.h>
#endif /* CONFIG_WS2812_NON_SPI_DRIVER */

#ifdef CONFIG_WS2812

/****************************************************************************
 * ######## ATTENTION #######
 * This file contains code that supports two separate ws2812 upper-half
 * models:
 *
 * If CONFIG_WS2812_NON_SPI_DRIVER is NOT defined the older upper-half
 * code that uses SPI to send the serial data will be built.
 *
 * If WS2812_NEW_MODEL_DRIVER is defined the upper-half code that does
 * not relay on SPI will be built.
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WS2812_RW_PIXEL_SIZE  4

#ifdef CONFIG_WS2812_NON_SPI_DRIVER

#else /* CONFIG_WS2812_NON_SPI_DRIVER */

/* In order to meet the signaling timing requirements, the waveforms required
 * to represent a 0/1 symbol are created by specific SPI bytes defined here.
 *
 * Only two target frequencies: 4 MHz and 8 MHz. However, given the tolerance
 * allowed in the WS2812 timing specs, two ranges around those target
 * frequencies can be used for better flexibility. Extreme frequencies
 * rounded to the nearest multiple of 100 kHz which meets the specs.
 * Try to avoid using the extreme frequencies.
 *
 * If using an LED different to the WS2812 (e.g. WS2812B) check its timing
 * specs, which may vary slightly, to decide which frequency is safe to use.
 *
 * WS2812 specs:
 * T0H range: 200ns - 500ns
 * T1H range: 550ns - 850ns
 * Reset: low signal >50us
 */

#if CONFIG_WS2812_FREQUENCY >= 3600000 && CONFIG_WS2812_FREQUENCY <= 5000000
#  define WS2812_ZERO_BYTE  0b01000000 /* 200ns at 5 MHz, 278ns at 3.6 MHz */
#  define WS2812_ONE_BYTE   0b01110000 /* 600ns at 5 MHz, 833ns at 3.6 MHz */
#elif CONFIG_WS2812_FREQUENCY >= 5900000 && CONFIG_WS2812_FREQUENCY <= 9000000
#  define WS2812_ZERO_BYTE  0b01100000 /* 222ns at 9 MHz, 339ns at 5.9 MHz */
#  define WS2812_ONE_BYTE   0b01111100 /* 556ns at 9 MHz, 847ns at 5.9 MHz */
#else
#  error "Unsupported SPI Frequency"
#endif

/* Reset bytes
 * Number of empty bytes to create the reset low pulse
 * Aiming for 60 us, safely above the 50us required.
 */

#define WS2812_RST_CYCLES (CONFIG_WS2812_FREQUENCY * 60 / 1000000 / 8)

#define WS2812_BYTES_PER_LED  (8 * 3)

/* Transmit buffer looks like:
 * [<----N reset bytes---->|<-RGBn->...<-RGB0->|<----1 reset byte---->]
 *
 * It is important that this is shipped as close to one chunk as possible
 * in order to meet timing requirements and to keep MOSI from going high
 * between transactions.  Some chips will leave MOSI at the state of the
 * MSB of the last byte for this reason it is recommended to shift the
 * bits that represents the zero or one waveform so that the MSB is 0.
 * The reset byte after the RGB data will pad the shortened low at the end.
 */

#define TXBUFF_SIZE(n) (WS2812_RST_CYCLES + n * WS2812_BYTES_PER_LED + 1)

#endif /* CONFIG_WS2812_NON_SPI_DRIVER */

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifndef CONFIG_WS2812_NON_SPI_DRIVER

struct ws2812_dev_s
{
  FAR struct spi_dev_s *spi;  /* SPI interface */
  uint16_t nleds;             /* Number of addressable LEDs */
  FAR uint8_t *tx_buf;        /* Buffer for write transaction and state */
  mutex_t lock;               /* Assures exclusive access to the driver */
};

#endif /* CONFIG_WS2812_NON_SPI_DRIVER */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_WS2812_NON_SPI_DRIVER

static inline void ws2812_configspi(FAR struct spi_dev_s *spi);
static void ws2812_pack(FAR uint8_t *buf, uint32_t rgb);

#endif /* CONFIG_WS2812_NON_SPI_DRIVER */

/* Character driver methods */

#ifdef CONFIG_WS2812_NON_SPI_DRIVER

static ssize_t ws2812_open(FAR struct file *filep);

static ssize_t ws2812_close(FAR struct file *filep);

#endif /* CONFIG_WS2812_NON_SPI_DRIVER */

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
#ifdef CONFIG_WS2812_NON_SPI_DRIVER
  ws2812_open,    /* open */
  ws2812_close,   /* close */
#else /* CONFIG_WS2812_NON_SPI_DRIVER */
  NULL,           /* open */
  NULL,           /* close */
#endif /* CONFIG_WS2812_NON_SPI_DRIVER */
  ws2812_read,    /* read */
  ws2812_write,   /* write */
  ws2812_seek,    /* seek */
  NULL,           /* ioctl */
  NULL            /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

/****************************************************************************
 * #### TODO ####
 *
 * Consider supporting mmap by returning memory buffer using...
 *       file_ioctl(filep, FIOC_MMAP, (unsigned long)((uintptr_t)&addr));
 * Code using this would be non-portable across architectures as the format
 * of the buffer can be different.
 *
 * Consider supporting rectangular arrays of ws2812s as a video output
 * device.
 *
 ****************************************************************************/

/****************************************************************************
 * Table of Gamma Correction Values
 *
 * This table is based on:
 *                y = 255 * (x / 255)^2.6
 ****************************************************************************/

static const uint8_t ws2812_gamma[256] =
{
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,
  1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   3,
  3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,
  6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  10,
  11,  11,  11,  12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,
  17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  24,  24,  25,
  25,  26,  27,  27,  28,  29,  29,  30,  31,  31,  32,  33,  34,  34,  35,
  36,  37,  38,  38,  39,  40,  41,  42,  42,  43,  44,  45,  46,  47,  48,
  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,
  64,  65,  66,  68,  69,  70,  71,  72,  73,  75,  76,  77,  78,  80,  81,
  82,  84,  85,  86,  88,  89,  90,  92,  93,  94,  96,  97,  99,  100, 102,
  103, 105, 106, 108, 109, 111, 112, 114, 115, 117, 119, 120, 122, 124, 125,
  127, 129, 130, 132, 134, 136, 137, 139, 141, 143, 145, 146, 148, 150, 152,
  154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182,
  184, 186, 188, 191, 193, 195, 197, 199, 202, 204, 206, 209, 211, 213, 215,
  218, 220, 223, 225, 227, 230, 232, 235, 237, 240, 242, 245, 247, 250, 252,
  255
};

/****************************************************************************
 * Table for HSV to RGB conversion
 ****************************************************************************/

  static const uint8_t hsv_rgb[43] =
  {
      0,   6,  12,  18,  24,  30,  36,  43,  49,  55,
     61,  67,  73,  79,  85,  91,  97, 103, 109, 115,
     121, 128, 134, 140, 146, 152, 158, 164, 170, 176,
     182, 188, 194, 200, 206, 213, 219, 225, 231, 237,
     243, 249, 255
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_WS2812_NON_SPI_DRIVER

/****************************************************************************
 * Name: ws2812_open
 *
 * Description:
 *   Prepare the ws2812 for use.  This method just calls the lower-half
 *   open routine if one exists.
 *
 * Input Parameters:
 *   filep    - Pointer system file data
 *
 * Returned Value:
 *   A pointer to an internal structure used by rp2040_ws2812
 *
 ****************************************************************************/

ssize_t ws2812_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct ws2812_dev_s *priv  = inode->i_private;
  int                      res;

  res = priv->open(filep);

  return res;
}

/****************************************************************************
 * Name: ws2812_close
 *
 * Description:
 *   Cleanup after use.  This method just calls the lower-half
 *   open routine if one exists.
 *
 * Input Parameters:
 *   filep    - Pointer system file data
 *
 * Returned Value:
 *   OK if successful, or an error code on failure.
 *
 ****************************************************************************/

static int ws2812_close(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct ws2812_dev_s *priv  = inode->i_private;
  int                      res   = OK;

  if (priv != NULL  &&  priv->close != NULL)
    {
      res = priv->close(filep);
    }

  return res;
}

/****************************************************************************
 * Name: ws2812_write
 * Description:
 *   Updates the data buffer with the supplied data any then sends the data
 *   to the LEDs.  A write length of zero does not update data but will
 *   re-send the data to the leds.
 *
 * Input Parameter:
 *   filep    - Pointer system file data
 *   data     - Data to send.
 *   len      - Length of data in bytes.
 *
 * Returned Value:
 *   number of bytes written on success, ERROR if write fails.
 *
 ****************************************************************************/

ssize_t ws2812_write(FAR struct file *filep,
                     FAR const char  *data,
                     size_t           len)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct ws2812_dev_s *priv  = inode->i_private;
  ssize_t                  res;

  if ((len % WS2812_RW_PIXEL_SIZE) != 0)
    {
      lederr("ERROR: LED values must be 24bit packed in 32bit\n");
      return -EINVAL;
    }

  res = priv->write(filep, data, len);

  return res;
}

/****************************************************************************
 * Name: ws2812_read
 * Description:
 *   Fetches data from the pixel buffer.
 *
 * Input Parameter:
 *   filep    - Pointer system file data
 *   data     - pointer to receive buffer.
 *   len      - Length to read in bytes.
 *
 * Returned Value:
 *   number of bytes read on success, ERROR if read fails.
 *
 ****************************************************************************/

ssize_t ws2812_read(FAR struct file *filep,
                    FAR char        *data,
                    size_t           len)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct ws2812_dev_s *priv  = inode->i_private;
  ssize_t                  res;

  if (priv == NULL  ||  priv->read == NULL)
    {
      return -ENOSYS;
    }

  if ((len % WS2812_RW_PIXEL_SIZE) != 0)
    {
      lederr("ERROR: LED values must be packed in 32bit words.\n");
      return -EINVAL;
    }

  res = priv->read(filep, data, len);

  return res;
}

#else /* CONFIG_WS2812_NON_SPI_DRIVER */

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
      return -EINVAL;
    }

  /* We need at least one LED, so 1 byte */

  if (buflen < 1)
    {
      lederr("ERROR: You need to control at least 1 LED!\n");
      return -EINVAL;
    }

  if ((buflen % WS2812_RW_PIXEL_SIZE) != 0)
    {
      lederr("ERROR: LED values must be 24bit packed in 32bit\n");
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);

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

  nxmutex_unlock(&priv->lock);
  return written;
}

#endif /* CONFIG_WS2812_NON_SPI_DRIVER */

/****************************************************************************
 * Name: ws2812_seek
 *
 * Description:
 *   This routine is called when seeking the WS2812 device. This can be used
 *   to address the starting LED to write.  This should be done on a full
 *   color boundary which is 32bits. e.g. LED0 - offset 0, LED 8 - offset 32
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

  nxmutex_lock(&priv->lock);

  maxpos = (priv->nleds - 1) * WS2812_RW_PIXEL_SIZE;
  pos    = filep->f_pos;

  switch (whence)
    {
      case SEEK_CUR:
        pos += offset;
        filep->f_pos = pos;
        break;

      case SEEK_SET:
        pos = offset;
        break;

      case SEEK_END:
        pos = maxpos + offset + 4;
        break;

      default:

        /* Return EINVAL if the whence argument is invalid */

        nxmutex_unlock(&priv->lock);
        return (off_t)-EINVAL;
    }

  if (pos > maxpos)
    {
      pos = maxpos;
    }
  else if (pos < 0)
    {
      pos = 0;
    }

  filep->f_pos = pos;

  nxmutex_unlock(&priv->lock);
  return pos;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_WS2812_NON_SPI_DRIVER

/****************************************************************************
 * Name: ws2812_register
 *
 * Description:
 *   Initialize a ws2812 device as a LEDs interface.
 *
 * Input Parameters:
 *   dev_path  - The full path to the driver to register. E.g., "/dev/leds0"
 *   count     - The number of ws2812s in the chain
 *   has_white - Set true if the ws2812s in the chain have while LEDs
 *   slow_leds - Set true to support older 400 kHz leds.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ws2812_register(FAR const char          *dev_path,
                    FAR struct ws2812_dev_s *dev_data)
{
  /* Register the character driver */

  int ret = register_driver(dev_path, &g_ws2812fops, 0666, dev_data);
  if (ret < 0)
    {
      lederr("ERROR: Failed to register ws2812 driver: %d\n", ret);
    }

  return ret;
}

#else /* CONFIG_WS2812_NON_SPI_DRIVER */

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

  priv->nleds  = nleds;
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

  nxmutex_init(&priv->lock);

  SPI_SNDBLOCK(priv->spi, priv->tx_buf, TXBUFF_SIZE(priv->nleds));

  /* Register the character driver */

  ret = register_driver(devpath, &g_ws2812fops, 0666, priv);
  if (ret < 0)
    {
      lederr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv->tx_buf);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_WS2812_NON_SPI_DRIVER */

/****************************************************************************
 * Name: ws2812_hsv_to_rgb
 *
 * Description:
 *   Convert a set of hue, saturation and value numbers to an RGB pixel.
 *
 *   Representative "hue" values:
 *     Red       0
 *     Yellow   42
 *     Green    86
 *     Cyan    128
 *     Blue    170
 *     Magenta 212
 *
 *   "Saturation" values run from 0 (gray) to 255 (pure color)
 *
 *   "Value" values run from 0 (black) to 255 (full brightness)
 *
 * Input Parameters:
 *   hue        in range (0-255) (red -> 0, green -> 85, blue -> 170)
 *   saturation in range (0-255)
 *   value      in range (0-255)
 *
 * Returned Value:
 *   A 32-bit pixel in 0x00RRGGBB format.
 *
 ****************************************************************************/

uint32_t ws2812_hsv_to_rgb(uint8_t hue,
                           uint8_t saturation,
                           uint8_t value)
{
  uint32_t val = value      + 1;   /* move value to range 1...256 */
  uint32_t sat = saturation + 1;   /* move value to range 1...256 */
  uint16_t  r;
  uint16_t  g;
  uint16_t  b;

  /* ===== Compute full saturation R,G,B based on hue =====
   *
   * These computed values are inverted from the normal
   * sense. (0 -> full color   255 -> black) in preparation
   * for the saturation adjustment.
   */

  if (hue < 86)
    {
      /* Color between Red and Green */

      b = 255;
      if (hue < 43)
        {
          /* 0 - 42     Color between Red and Yellow */

          r = 0;
          g = 255 - hsv_rgb[hue];
        }
      else
        {
          /* 43 - 85     Color between Yellow and Green */

          r = hsv_rgb[hue - 43];
          g = 0;
        }
    }
  else if (hue < 171)
    {
      /* Color between Green and Blue */

      r = 255;
      if (hue < 128)
        {
          /* 86 - 127    Color between Green and Cyan */

          g = 0;
          b = 255 - hsv_rgb[hue - 86];
        }
      else
        {
          /* 128 - 170    Color between Cyan and Blue */

          g = hsv_rgb[hue - 128];
          b = 0;
        }
    }
  else
    {
      /* Color between Blue and Red */

      g = 255;
      if (hue < 214)
        {
          /* 171 - 213   Color between Blue and Magenta */

          b = 0;
          r = 255 - hsv_rgb[hue - 171];
        }
      else
        {
          /* 214 - 255   Color between Magenta and Red */

          b = hsv_rgb[hue - 214];
          r = 0;
        }
    }

  /* This step scales the color for saturation and inverts
   * back to 255 -> bright and 0 -> black.
   */

  r = 255 - ((r * sat) >> 8);
  g = 255 - ((g * sat) >> 8);
  b = 255 - ((b * sat) >> 8);

  /* compute the return value using the r, g, and b values scaled
   * by the value parameter
   */

  return   (((r * val) << 8) & 0xff0000)
         | (((g * val) << 0) & 0x00ff00)
         | (((b * val) >> 8) & 0x0000ff);
}

/****************************************************************************
 * Name: ws2812_gamma_correct
 *
 * Description:
 *   Applies a gamma correction to the supplied pixel.
 *
 * Input Parameters:
 *   a 32-bit pixel with 8-bit color components.
 *
 * Returned Value:
 *   A 32-bit gamma corrected pixel.
 *
 ****************************************************************************/

uint32_t ws2812_gamma_correct(uint32_t pixel)
{
  uint32_t     res;
  FAR uint8_t *in  = (FAR uint8_t *)&pixel;
  FAR uint8_t *out = (FAR uint8_t *)&res;

  *out++ = ws2812_gamma[*in++];
  *out++ = ws2812_gamma[*in++];
  *out++ = ws2812_gamma[*in++];
  *out   = ws2812_gamma[*in];

  return res;
}

#endif /* CONFIG_WS2812 */

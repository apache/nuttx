/****************************************************************************
 * drivers/sensors/max7219.c
 * Maxim MAX7219 driver used to control 7-segment display.
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2018 Alan Carvalho de Assis
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/leds/max7219.h>

#if defined(CONFIG_SPI) && defined(CONFIG_LEDS_MAX7219)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI frequency */

#ifndef CONFIG_MAX7219_FREQUENCY
#  define CONFIG_MAX7219_FREQUENCY 5000000
#endif

#ifndef CONFIG_MAX7219_INTENSITY
#  define CONFIG_MAX7219_INTENSITY 8
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct max7219_dev_s
{
  FAR struct spi_dev_s *spi; /* SPI interface */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void max7219_configspi(FAR struct spi_dev_s *spi);
static inline void max7219_write16(FAR struct max7219_dev_s *priv,
                                   uint16_t value);

/* Character driver methods */

static int     max7219_open(FAR struct file *filep);
static int     max7219_close(FAR struct file *filep);
static ssize_t max7219_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t max7219_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_max7219fops =
{
  max7219_open,   /* open */
  max7219_close,  /* close */
  max7219_read,   /* read */
  max7219_write,  /* write */
  NULL,           /* seek */
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
 * Name: max7219_configspi
 *
 * Description:
 *   Set the SPI bus configuration
 *
 ****************************************************************************/

static inline void max7219_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the MAX7219 */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_MAX7219_FREQUENCY);
}

/****************************************************************************
 * Name: max7219_write16
 *
 * Description:
 *   Write 16-bit to MAX7219
 *
 ****************************************************************************/

static inline void max7219_write16(FAR struct max7219_dev_s *priv,
                                   uint16_t value)
{
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);

  /* Configure the SPI */

  max7219_configspi(priv->spi);

  /* Select */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);

  /* Send 16 bits (2 bytes) */

  SPI_SNDBLOCK(priv->spi, &value, 2);

  /* De-select */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: max7219_open
 *
 * Description:
 *   This function is called whenever the MAX7219 device is opened.
 *
 ****************************************************************************/

static int max7219_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: max7219_close
 *
 * Description:
 *   This routine is called when the LM-75 device is closed.
 *
 ****************************************************************************/

static int max7219_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: max7219_read
 ****************************************************************************/

static ssize_t max7219_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: max7219_write
 ****************************************************************************/

static ssize_t max7219_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct max7219_dev_s *priv = inode->i_private;
  uint16_t data;
  uint8_t digits[8];
  uint8_t digcnt = 0;
  uint8_t i;

  if (buffer == NULL)
    {
      lederr("ERROR: Buffer is null\n");
      return -1;
    }

  /* We need at least one display, so 1 byte */

  if (buflen < 1)
    {
      lederr("ERROR: You need to control at least 1 digit!\n");
      return -1;
    }

  /* Get and count the valid digits */

  for (i = 0; i < buflen; i++)
    {
      if (buffer[i] >= '0' && buffer[i] <= '9')
        {
          digits[digcnt] = buffer[i] - '0';
          digcnt++;
        }
      else
        {
          switch (buffer[i])
            {
            case '-':
              digits[digcnt] = 0x0a;
              digcnt++;
              break;

            case 'E':
              digits[digcnt] = 0x0b;
              digcnt++;
              break;

            case 'H':
              digits[digcnt] = 0x0c;
              digcnt++;
              break;

            case 'L':
              digits[digcnt] = 0x0d;
              digcnt++;
              break;

            case 'P':
              digits[digcnt] = 0x0e;
              digcnt++;
              break;

            case ' ':
              digits[digcnt] = 0x0f;
              digcnt++;
              break;

            case '.':
              if (i == 0)
                {
                  digits[0] = 0x0f | 0x80;
                  digcnt++;
                }
              else
                {
                  digits[digcnt - 1] |= 0x80;
                }
              break;

            default:
              lederr("Digit code unsupported\n");
              break;
            }
        }

      /* If we reach 8 digits then stop */

      if (digcnt >= 8 && buffer[i + 1] != '.')
        {
          ledwarn("ERROR: The limit is 8 digits!\n");
          break;
        }
    }

  /* Blank the display */

  for (i = 0; i < 8; i++)
    {
      data = (MAX7219_DIGIT0 + i) | (0x0f << 8);
      max7219_write16(priv, data);
    }

  /* Write each digit */

  for (i = 0; i < digcnt; i++)
    {
      data = (MAX7219_DIGIT0 + i) | (digits[digcnt - i - 1]) << 8;
      max7219_write16(priv, data);
    }

  /* All bytes were written */

  return buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max7219_leds_register
 *
 * Description:
 *   Register the MAX7219 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             MAX7219
 *   addr    - The I2C address of the LM-75.  The base I2C address of the
 *             MAX7219 is 0x48.  Bits 0-3 can be controlled to get 8 unique
 *             addresses from 0x48 through 0x4f.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int max7219_leds_register(FAR const char *devpath, FAR struct spi_dev_s *spi)
{
  FAR struct max7219_dev_s *priv;
  uint16_t data;
  int ret;
  int i;

  /* Initialize the MAX7219 device structure */

  priv = (FAR struct max7219_dev_s *)kmm_malloc(sizeof(struct max7219_dev_s));
  if (!priv)
    {
      lederr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi = spi;

  /* Setup defined intensity */

  data = (MAX7219_INTENSITY) |
         (DISPLAY_INTENSITY(CONFIG_MAX7219_INTENSITY) << 8);
  max7219_write16(priv, data);

  /* Enable decoding for all digits */

  data = (MAX7219_DECODE_MODE) | (ENABLE_DECODE << 8);
  max7219_write16(priv, data);

  /* Display all digits */

  data = (MAX7219_SCAN_LIMIT) | (0x07 << 8);
  max7219_write16(priv, data);

  /* Leave the shutdown mode */

  data = (MAX7219_SHUTDOWN) | (MAX7219_POWER_ON << 8);
  max7219_write16(priv, data);

  /* Blank the display */

  for (i = 0; i < 8; i++)
    {
      data = (MAX7219_DIGIT0 + i) | (0x0f << 8);
      max7219_write16(priv, data);
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_max7219fops, 0666, priv);
  if (ret < 0)
    {
      lederr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SPI && CONFIG_MAX7219 */

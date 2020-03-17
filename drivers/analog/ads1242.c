/****************************************************************************
 * drivers/sensors/ads1242.c
 * Character driver for the MCP3426 Differential Input 16 Bit Delta/Sigma ADC
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 DS-Automotion GmbH. All rights reserved.
 *   Author: Alexander Entinger <a.entinger@ds-automotion.com>
 *           Gregory Nutt <gnutt@nuttx.org>
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
#include <stdbool.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>

#include <nuttx/analog/ads1242.h>

#if defined(CONFIG_SPI) && defined(CONFIG_ADC_ADS1242)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ads1242_dev_s
{
  FAR struct spi_dev_s *spi;  /* Pointer to the SPI instance */
  uint32_t osc_period_us;     /* Period of the oscillator attached to the ADS1242 in us */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI Helpers */

static void    ads1242_lock(FAR struct spi_dev_s *spi);
static void    ads1242_unlock(FAR struct spi_dev_s *spi);
static void    ads1242_reset(FAR struct ads1242_dev_s *dev);
static void    ads1242_perform_selfgain_calibration(
                 FAR struct ads1242_dev_s *dev);
static void    ads1242_perform_selfoffset_calibration(
                 FAR struct ads1242_dev_s *dev);
static void    ads1242_perform_systemoffset_calibration(
                 FAR struct ads1242_dev_s *dev);
static void    ads1242_read_conversion_result(FAR struct ads1242_dev_s *dev,
                 FAR uint32_t *conversion_result);

static void    ads1242_write_reg(FAR struct ads1242_dev_s *dev,
                 uint8_t const reg_addr, uint8_t const reg_value);
static void    ads1242_read_reg(FAR struct ads1242_dev_s *dev,
                 uint8_t const reg_addr, FAR uint8_t *reg_value);

static void    ads1242_set_gain(FAR struct ads1242_dev_s *dev,
                 ADS1242_GAIN_SELECTION const gain_selection);
static void    ads1242_set_positive_input(FAR struct ads1242_dev_s *dev,
                 ADS1242_POSITIVE_INPUT_SELECTION const pos_in_sel);
static void    ads1242_set_negative_input(FAR struct ads1242_dev_s *dev,
                 ADS1242_NEGATIVE_INPUT_SELECTION const neg_in_sel);
static bool    ads1242_is_data_ready(FAR struct ads1242_dev_s *dev);

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_INFO)
static void    ads1242_print_regs(FAR struct ads1242_dev_s *dev,
                 char const *msg);
#endif /* CONFIG_DEBUG_FEATURES && CONFIG_DEBUG_INFO */

/* Character driver methods */

static int     ads1242_open(FAR struct file *filep);
static int     ads1242_close(FAR struct file *filep);
static ssize_t ads1242_read(FAR struct file *, FAR char *, size_t);
static ssize_t ads1242_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     ads1242_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ads1242_fops =
{
  ads1242_open,
  ads1242_close,
  ads1242_read,
  ads1242_write,
  NULL,
  ads1242_ioctl,
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads1242_lock
 *
 * Description:
 *   Lock and configure the SPI bus.
 *
 ****************************************************************************/

static void ads1242_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, ADS1242_SPI_MODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, ADS1242_SPI_FREQUENCY);
}

/****************************************************************************
 * Name: ads1242_unlock
 *
 * Description:
 *   Unlock the SPI bus.
 *
 ****************************************************************************/

static void ads1242_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: ads1242_reset
 ****************************************************************************/

static void ads1242_reset(FAR struct ads1242_dev_s *dev)
{
  ads1242_lock(dev->spi);

  SPI_SELECT(dev->spi, 0, true);         /* Set nADC_SPI_CS to low which
                                          * selects the ADS1242 */
  SPI_SEND(dev->spi, ADS1242_CMD_RESET); /* Issue reset command */
  SPI_SELECT(dev->spi, 0, false);        /* Set nADC_SPI_CS to high which
                                          * deselects the ADS1242 */
  up_mdelay(100);                        /* Wait a little so the device has
                                          * time to perform a proper reset */

  ads1242_unlock(dev->spi);
}

/****************************************************************************
 * Name: ads1242_perform_selfgain_calibration
 ****************************************************************************/

static void ads1242_perform_selfgain_calibration(FAR struct ads1242_dev_s
              *dev)
{
  ads1242_lock(dev->spi);

  SPI_SELECT(dev->spi, 0, true);
  SPI_SEND(dev->spi, ADS1242_CMD_SELF_GAIN_CALIB);
  SPI_SELECT(dev->spi, 0, false);

  ads1242_unlock(dev->spi);
}

/****************************************************************************
 * Name: ads1242_perform_selfoffset_calibration
 ****************************************************************************/

static void ads1242_perform_selfoffset_calibration(FAR struct ads1242_dev_s
              *dev)
{
  ads1242_lock(dev->spi);

  SPI_SELECT(dev->spi, 0, true);
  SPI_SEND(dev->spi, ADS1242_CMD_SELF_OFFSET_CALIB);
  SPI_SELECT(dev->spi, 0, false);

  ads1242_unlock(dev->spi);
}

/****************************************************************************
 * Name: ads1242_perform_systemoffset_calibration
 ****************************************************************************/

static void
  ads1242_perform_systemoffset_calibration(FAR struct ads1242_dev_s *dev)
{
  ads1242_lock(dev->spi);

  SPI_SELECT(dev->spi, 0, true);
  SPI_SEND(dev->spi, ADS1242_CMD_SYSTEM_OFFSET_CALIB);
  SPI_SELECT(dev->spi, 0, false);

  ads1242_unlock(dev->spi);
}

/****************************************************************************
 * Name: ads1242_read_conversion_result
 ****************************************************************************/

static void ads1242_read_conversion_result(FAR struct ads1242_dev_s *dev,
                                           FAR uint32_t *conversion_result)
{
  ads1242_lock(dev->spi);

  SPI_SELECT(dev->spi, 0, true);
  SPI_SEND(dev->spi, ADS1242_CMD_READ_DATA);

  /* Delay between last SCLK edge for DIN and first SCLK edge for DOUT:
   * RDATA, RDATAC, RREG, WREG: Min 50 x tOSC Periods
   */

  up_udelay(50 * dev->osc_period_us);

  *conversion_result = 0;

  /* 1st Byte = MSB
   * 2nd Byte = Mid-Byte
   * 3rd Byte = LSB
   */

  *conversion_result |= ((uint32_t)(SPI_SEND(dev->spi, 0xff))) << 16;
  *conversion_result |= ((uint32_t)(SPI_SEND(dev->spi, 0xff))) << 8;
  *conversion_result |= ((uint32_t)(SPI_SEND(dev->spi, 0xff))) << 0;

  SPI_SELECT(dev->spi, 0, false);

  ads1242_unlock(dev->spi);
}

/****************************************************************************
 * Name: ads1242_write_reg
 *
 * Description:
 *    Write to the registers starting with the register address specified as
 *    part of the instruction. The number of registers that will be written
 *    is one plus the value of the second byte.
 *
 ****************************************************************************/

static void ads1242_write_reg(FAR struct ads1242_dev_s *dev,
                              uint8_t const reg_addr,
                              uint8_t const reg_value)
{
  ads1242_lock(dev->spi);

  SPI_SELECT(dev->spi, 0, true);
  SPI_SEND(dev->spi, ADS1242_CMD_WRITE_REGISTER | reg_addr);
  SPI_SEND(dev->spi, 0x00); /* Write 1 Byte */
  SPI_SEND(dev->spi, reg_value);
  SPI_SELECT(dev->spi, 0, false);

  ads1242_unlock(dev->spi);
}

/****************************************************************************
 * Name: ads1242_read_reg
 *
 * Description:
 *   Output the data from up to 16 registers starting with the register
 *   address specified as part of the instruction.  The number of registers
 *   read will be one plus the second byte count. If the count exceeds the
 *   remaining registers, the addresses wrap back to the beginning.
 *
 ****************************************************************************/

static void ads1242_read_reg(FAR struct ads1242_dev_s *dev,
                             uint8_t const reg_addr, FAR uint8_t *reg_value)
{
  ads1242_lock(dev->spi);

  SPI_SELECT(dev->spi, 0, true);
  SPI_SEND(dev->spi, ADS1242_CMD_READ_REGISTER | reg_addr);
  SPI_SEND(dev->spi, 0x00);  /* Read 1 Byte */

  /* Delay between last SCLK edge for DIN and first SCLK edge for DOUT:
   * RDATA, RDATAC, RREG, WREG: Min 50 x tOSC Periods
   */

  up_udelay(50 * dev->osc_period_us);

  *reg_value = SPI_SEND(dev->spi, 0xff);

  SPI_SELECT(dev->spi, 0, false);

  ads1242_unlock(dev->spi);
}

/****************************************************************************
 * Name: ads1242_set_gain
 ****************************************************************************/

static void ads1242_set_gain(FAR struct ads1242_dev_s *dev,
                             ADS1242_GAIN_SELECTION const gain_selection)
{
  uint8_t setup_reg_value = 0;

  ads1242_read_reg(dev, ADS1242_REG_SETUP, &setup_reg_value);
  setup_reg_value &= ~(ADS1242_REG_SETUP_BIT_PGA2 |
                       ADS1242_REG_SETUP_BIT_PGA1 |
                       ADS1242_REG_SETUP_BIT_PGA0);
  setup_reg_value |= (uint8_t)(gain_selection);
  ads1242_write_reg(dev, ADS1242_REG_SETUP, setup_reg_value);

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_INFO)
  ads1242_print_regs(dev, "ads1242_set_gain");
#endif

  /* It is necessary to perform a offset calibration after setting the gain */

  ads1242_perform_selfoffset_calibration(dev);
}

/****************************************************************************
 * Name: ads1242_set_positive_input
 ****************************************************************************/

static void ads1242_set_positive_input(FAR struct ads1242_dev_s *dev,
              ADS1242_POSITIVE_INPUT_SELECTION const pos_in_sel)
{
  uint8_t mux_reg_value = 0;

  ads1242_read_reg(dev, ADS1242_REG_MUX, &mux_reg_value);
  mux_reg_value &= ~(ADS1242_REG_MUX_BIT_PSEL3 | ADS1242_REG_MUX_BIT_PSEL2 |
                     ADS1242_REG_MUX_BIT_PSEL1 | ADS1242_REG_MUX_BIT_PSEL0);
  mux_reg_value |= (uint8_t)(pos_in_sel);
  ads1242_write_reg(dev, ADS1242_REG_MUX, mux_reg_value);

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_INFO)
  ads1242_print_regs(dev, "ads1242_set_positive_input");
#endif
}

/****************************************************************************
 * Name: ads1242_set_negative_input
 ****************************************************************************/

static void ads1242_set_negative_input(FAR struct ads1242_dev_s *dev,
              ADS1242_NEGATIVE_INPUT_SELECTION const neg_in_sel)
{
  uint8_t mux_reg_value = 0;

  ads1242_read_reg(dev, ADS1242_REG_MUX, &mux_reg_value);
  mux_reg_value &= ~(ADS1242_REG_MUX_BIT_NSEL3 | ADS1242_REG_MUX_BIT_NSEL2 |
                     ADS1242_REG_MUX_BIT_NSEL1 | ADS1242_REG_MUX_BIT_NSEL0);
  mux_reg_value |= (uint8_t)(neg_in_sel);
  ads1242_write_reg(dev, ADS1242_REG_MUX, mux_reg_value);

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_INFO)
  ads1242_print_regs(dev, "ads1242_set_negative_input");
#endif
}

/****************************************************************************
 * Name: ads1242_set_negative_input
 ****************************************************************************/

static bool ads1242_is_data_ready(FAR struct ads1242_dev_s *dev)
{
  uint8_t acr_reg_value = 0xff;

  ads1242_read_reg(dev, ADS1242_REG_ACR, &acr_reg_value);
  return (acr_reg_value & ADS1242_REG_ACR_BIT_NDRDY) == 0;
}

/****************************************************************************
 * Name: ads1242_print_regs
 ****************************************************************************/

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_INFO)
static void ads1242_print_regs(FAR struct ads1242_dev_s *dev,
                               char const *msg)
{
  uint8_t setup_reg_value = 0;
  uint8_t mux_reg_value   = 0;
  uint8_t acr_reg_value   = 0;

  ainfo("%s\n", msg);

  ads1242_read_reg(dev, ADS1242_REG_SETUP, &setup_reg_value);
  ads1242_read_reg(dev, ADS1242_REG_MUX, &mux_reg_value);
  ads1242_read_reg(dev, ADS1242_REG_ACR, &acr_reg_value);

  ainfo("SETUP  %02X\n", setup_reg_value);
  ainfo("MUX    %02X\n", mux_reg_value);
  ainfo("ACR    %02X\n", acr_reg_value);
}
#endif /* CONFIG_DEBUG_FEATURES && CONFIG_DEBUG_INFO */

/****************************************************************************
 * Name: ads1242_open
 ****************************************************************************/

static int ads1242_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads1242_dev_s *priv = inode->i_private;

  ads1242_reset(priv);
  up_mdelay(100);

  ads1242_perform_selfgain_calibration(priv);
  up_mdelay(100);

  /* SPEED = 1 -> fMod = fOsc / 256 (fMod = Modulator Clock Speed)
   * BUFEN = 1 -> Internal input buffer enabled -> results in a very high
   *              impedance input for the ADC ~ 5 GOhm
   */

  ads1242_write_reg(priv, ADS1242_REG_ACR,
                    ADS1242_REG_ACR_BIT_SPEED | ADS1242_REG_ACR_BIT_BUFEN);

  ads1242_perform_selfoffset_calibration(priv);
  up_mdelay(100);

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_INFO)
  ads1242_print_regs(priv, "ads1242_open");
#endif

  return OK;
}

/****************************************************************************
 * Name: ads1242_close
 ****************************************************************************/

static int ads1242_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads1242_dev_s *priv = inode->i_private;

  ads1242_reset(priv);
  up_mdelay(100);

  return OK;
}

/****************************************************************************
 * Name: ads1242_read
 ****************************************************************************/

static ssize_t ads1242_read(FAR struct file *filep,
                            FAR char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ads1242_write
 ****************************************************************************/

static ssize_t ads1242_write(FAR struct file *filep,
                             FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ads1242_ioctl
 ****************************************************************************/

static int ads1242_ioctl (FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads1242_dev_s *priv = inode->i_private;

  int ret = OK;

  switch (cmd)
    {
    /* Read the result of an analog conversion */

    case ANIOC_ADS2142_READ:
      {
          FAR uint32_t *data = (FAR uint32_t *)((uintptr_t)arg);
          ads1242_read_conversion_result(priv, data);
      }
      break;

      /* Set the gain of the ADC */

    case ANIOC_ADS2142_SET_GAIN:
      {
        ads1242_set_gain(priv, (ADS1242_GAIN_SELECTION)(arg));
      }
      break;

      /* Set the positive input of the ADC */

    case ANIOC_ADS2142_SET_POSITIVE_INPUT:
      {
        ads1242_set_positive_input(priv,
                                   (ADS1242_POSITIVE_INPUT_SELECTION)(arg));
      }
      break;

      /* Set the negative input of the ADC */

    case ANIOC_ADS2142_SET_NEGATIVE_INPUT:
      {
        ads1242_set_negative_input(priv,
                                   (ADS1242_NEGATIVE_INPUT_SELECTION)(arg));
      }
      break;

      /* Check if data is ready to be read */

    case ANIOC_ADS2142_IS_DATA_READY:
      {
        FAR bool *is_data_ready = (FAR bool *)((uintptr_t)arg);
        *is_data_ready = ads1242_is_data_ready(priv);
      }
      break;

      /* Perform a system offset calibration - Note: Zero input signal must
       * be applied.
       */

    case ANIOC_ADS2142_DO_SYSTEM_OFFSET_CALIB:
      {
        ads1242_perform_systemoffset_calibration(priv);
      }
      break;

      /* Command was not recognized */

    default:
       _err("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads1242_register
 *
 * Description:
 *   Register the ADS1242 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/ads1242"
 *   spi - An instance of the SPI interface to use to communicate with
 *         ADS1242
 *   osc_freq_hz - The frequency of the ADS1242 oscillator in Hz. Required
 *                 for calculating the minimum delay periods when accessing
 *                 the device via SPI.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ads1242_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                     uint32_t const osc_freq_hz)
{
  FAR struct ads1242_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the ADS1242 device structure */

  priv =
    (FAR struct ads1242_dev_s *)kmm_malloc(sizeof(struct ads1242_dev_s));
  if (priv == NULL)
    {
       _err("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi = spi;

  float const osc_period_us = (1000.0 * 1000.0) / ((float)(osc_freq_hz));
  priv->osc_period_us = (uint32_t)(osc_period_us);

  /* Register the character driver */

  ret = register_driver(devpath, &g_ads1242_fops, 0666, priv);
  if (ret < 0)
    {
       _err("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_SPI && CONFIG_ADC_ADS1242 */

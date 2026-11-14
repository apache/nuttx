/****************************************************************************
 * drivers/power/supply/npzero.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Character driver for the Nanopower nPZero G1S power-saving IC.
 *
 * Reference: NPZ-G1S-DS-V1.0 (datasheet) and NPZ-G1S-ERRATA-V3.0.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/debug.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/power/npzero.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The nPZero supports the I2C standard mode only */

#ifndef CONFIG_NPZERO_I2C_FREQUENCY
#  define CONFIG_NPZERO_I2C_FREQUENCY 100000
#endif

/* Registers used by the workaround of the errata #11 (IREF trim) */

#define NPZERO_TRIM_UNLOCK       0x6f
#define NPZERO_TRIM_UNLOCK_KEY   0x60
#define NPZERO_TRIM_REV          0x6a
#define NPZERO_TRIM_REV_G1S      0x18
#define NPZERO_TRIM_IREF         0x65
#define NPZERO_TRIM_IREF_LOW     0x18
#define NPZERO_TRIM_IREF_HIGH    0x19
#define NPZERO_TRIM_IREF_MIN     0x1a
#define NPZERO_TRIM_CTRL         0x67
#define NPZERO_TRIM_LOT          0x76
#define NPZERO_TRIM_LOT_MIN      0x69f54f00

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct npzero_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C bus */
  struct i2c_config_s config;   /* I2C configuration */
  mutex_t lock;                 /* Exclusive access */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int npzero_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_npzero_fops =
{
  NULL,            /* open */
  NULL,            /* close */
  NULL,            /* read */
  NULL,            /* write */
  NULL,            /* seek */
  npzero_ioctl,    /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: npzero_read
 ****************************************************************************/

static int npzero_read(FAR struct npzero_dev_s *priv, uint8_t reg,
                       FAR uint8_t *buf, size_t len)
{
  int ret;

  ret = i2c_writeread(priv->i2c, &priv->config, &reg, 1, buf, len);
  if (ret < 0)
    {
      pwrerr("ERROR: read of 0x%02x failed: %d\n", reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: npzero_write
 ****************************************************************************/

static int npzero_write(FAR struct npzero_dev_s *priv, uint8_t reg,
                        uint8_t val)
{
  uint8_t buf[2];
  int ret;

  buf[0] = reg;
  buf[1] = val;

  ret = i2c_write(priv->i2c, &priv->config, buf, 2);
  if (ret < 0)
    {
      pwrerr("ERROR: write of 0x%02x failed: %d\n", reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: npzero_write16
 *
 * Description:
 *   Write a 16-bit value to a pair of low and high byte registers.
 *
 ****************************************************************************/

static int npzero_write16(FAR struct npzero_dev_s *priv, uint8_t reg,
                          uint16_t val)
{
  int ret;

  ret = npzero_write(priv, reg, val & 0xff);
  if (ret >= 0)
    {
      ret = npzero_write(priv, reg + 1, val >> 8);
    }

  return ret;
}

/****************************************************************************
 * Name: npzero_errata11
 *
 * Description:
 *   Workaround of the errata #11: incorrect IREF trim values can reset the
 *   nPZero in Idle state.  It must be done after each reset of the nPZero.
 *
 ****************************************************************************/

static int npzero_errata11(FAR struct npzero_dev_s *priv)
{
  uint8_t lot[4];
  uint8_t iref;
  uint8_t rev;
  int ret;

  ret = npzero_write(priv, NPZERO_TRIM_UNLOCK, NPZERO_TRIM_UNLOCK_KEY);
  if (ret < 0)
    {
      return ret;
    }

  ret = npzero_read(priv, NPZERO_TRIM_REV, &rev, 1);
  if (ret >= 0 && rev == NPZERO_TRIM_REV_G1S)
    {
      ret = npzero_read(priv, NPZERO_TRIM_IREF, &iref, 1);
      if (ret >= 0)
        {
          ret = npzero_read(priv, NPZERO_TRIM_LOT, lot, sizeof(lot));
        }

      if (ret >= 0)
        {
          uint32_t check = (uint32_t)lot[3] << 24 | (uint32_t)lot[2] << 16 |
                           (uint32_t)lot[1] << 8 | lot[0];

          if (check < NPZERO_TRIM_LOT_MIN || iref < NPZERO_TRIM_IREF_MIN)
            {
              iref = NPZERO_TRIM_IREF_LOW;
            }
          else
            {
              iref = NPZERO_TRIM_IREF_HIGH;
            }

          ret = npzero_write(priv, NPZERO_TRIM_IREF, iref);
        }

      if (ret >= 0)
        {
          ret = npzero_write(priv, NPZERO_TRIM_CTRL, 0);
        }
    }

  /* Always lock the trim registers again */

  if (npzero_write(priv, NPZERO_TRIM_UNLOCK, 0) < 0 && ret >= 0)
    {
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: npzero_check_periph
 *
 * Description:
 *   Check the configuration of a peripheral.
 *
 ****************************************************************************/

static int npzero_check_periph(FAR const struct npzero_periph_s *periph)
{
  if (periph->pwmod == NPZERO_PWMOD_DISABLED)
    {
      return OK;
    }

  /* The PSWMOD 0 (voltage detection) can lock up the polling */

  if ((periph->pwmod != NPZERO_PWMOD_PERIODIC &&
       periph->pwmod != NPZERO_PWMOD_ALWAYSON) ||
      periph->tmod > NPZERO_TMOD_ASYNCINT ||
      periph->pswmod < NPZERO_PSWMOD_SWITCH ||
      periph->pswmod > NPZERO_PSWMOD_LOGICINV ||
      periph->intmod > NPZERO_INTMOD_OUT_LOW ||
      periph->dtype > NPZERO_DTYPE_UINT8 ||
      periph->spimode > 3 || periph->retries > 3 ||
      periph->addr > NPZERO_ADDRP_ADDR_MASK ||
      periph->nread > NPZERO_ADDRP_ADDR_MASK ||
      periph->ninit > 127)
    {
      return -EINVAL;
    }

  if (periph->tinit == 2 || periph->tinit > NPZERO_WAIT_4096 ||
      periph->twait == 2 || periph->twait > NPZERO_WAIT_4096)
    {
      return -EINVAL;
    }

  /* Zero and one are not valid polling periods */

  if (periph->pwmod == NPZERO_PWMOD_PERIODIC && periph->period < 2)
    {
      return -EINVAL;
    }

  if ((periph->ninit > 0 && periph->init == NULL) ||
      (periph->spi && periph->nread > 0 && periph->read == NULL))
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: npzero_check
 *
 * Description:
 *   Check the configuration of the nPZero and build the content of its
 *   SRAM.  The initialization commands of the peripherals are stored in
 *   order and without gaps, with the read commands of the SPI peripherals
 *   after their initialization commands.
 *
 ****************************************************************************/

static int npzero_check(FAR const struct npzero_config_s *config,
                        FAR uint8_t *sram, FAR size_t *sramlen)
{
  FAR const struct npzero_periph_s *periph;
  size_t len = 0;
  size_t ninit;
  int ret;
  int i;

  if ((config->hostmode != NPZERO_HOSTMODE_SWITCH &&
       config->hostmode != NPZERO_HOSTMODE_LOGIC &&
       config->hostmode != NPZERO_HOSTMODE_LOGICINV) ||
      config->sclksel > NPZERO_SCLK_XO ||
      config->sclkdiv > NPZERO_SCLKDIV_16 ||
      config->adcclk > NPZERO_ADCCLK_1024HZ ||
      config->i2cpull == 2 || config->i2cpull > NPZERO_I2CPULL_AUTO ||
      config->clkout > NPZERO_CLKOUT_1KHZ ||
      config->pswint > 0x0f || config->timeout == 0 ||
      (config->wakeup & ~(NPZERO_SYSCFG1_WUP_P(1) | NPZERO_SYSCFG1_WUP_P(2) |
                          NPZERO_SYSCFG1_WUP_P(3) | NPZERO_SYSCFG1_WUP_P(4) |
                          NPZERO_SYSCFG1_WUP_A1 |
                          NPZERO_SYSCFG1_WUP_A2)) != 0 ||
      config->adcover[0] > 0x1f || config->adcunder[0] > 0x1f ||
      config->adcover[1] > 0x3f || config->adcunder[1] > 0x3f)
    {
      return -EINVAL;
    }

  /* The ADC clocks derived from the XO need the XO system clock */

  if (config->adcclk != NPZERO_ADCCLK_SCLK &&
      config->sclksel != NPZERO_SCLK_XO)
    {
      return -EINVAL;
    }

  for (i = 0; i < NPZERO_NPERIPH; i++)
    {
      if (config->intpull[i] == 2 ||
          config->intpull[i] > NPZERO_INTPULL_50K)
        {
          return -EINVAL;
        }

      periph = config->periph[i];
      if (periph == NULL || periph->pwmod == NPZERO_PWMOD_DISABLED)
        {
          continue;
        }

      ret = npzero_check_periph(periph);
      if (ret < 0)
        {
          return ret;
        }

      /* I2C commands are pairs of register address and value */

      ninit = periph->spi ? periph->ninit : 2 * periph->ninit;
      if (len + ninit > NPZERO_SRAM_SIZE)
        {
          return -E2BIG;
        }

      if (ninit > 0)
        {
          memcpy(&sram[len], periph->init, ninit);
          len += ninit;
        }

      if (periph->spi && periph->nread > 0)
        {
          if (len + periph->nread > NPZERO_SRAM_SIZE)
            {
              return -E2BIG;
            }

          memcpy(&sram[len], periph->read, periph->nread);
          len += periph->nread;
        }
    }

  *sramlen = len;
  return OK;
}

/****************************************************************************
 * Name: npzero_write_sram
 *
 * Description:
 *   Write the whole SRAM.  Sequential writes can corrupt the SRAM (errata
 *   #2), so the bytes are written one at a time in the order of the
 *   workaround.
 *
 ****************************************************************************/

static int npzero_write_sram(FAR struct npzero_dev_s *priv,
                             FAR const uint8_t *sram)
{
  unsigned int i;
  unsigned int k;
  int ret;

  for (i = 0; i < NPZERO_SRAM_SIZE; i++)
    {
      k = ((~i & 0x07) << 3) + ((i & 0x38) >> 3) + (i & 0x40);

      ret = npzero_write(priv, NPZERO_SRAM + k, sram[k]);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: npzero_config_periph
 ****************************************************************************/

static int npzero_config_periph(FAR struct npzero_dev_s *priv, int n,
                                FAR const struct npzero_periph_s *periph)
{
  uint8_t regval;
  int ret;

  if (periph == NULL || periph->pwmod == NPZERO_PWMOD_DISABLED)
    {
      return npzero_write(priv, NPZERO_CFGP(n), 0);
    }

  regval = periph->pwmod << NPZERO_CFGP_PWMOD_SHIFT |
           periph->tmod << NPZERO_CFGP_TMOD_SHIFT |
           periph->pswmod << NPZERO_CFGP_PSWMOD_SHIFT |
           periph->intmod << NPZERO_CFGP_INTMOD_SHIFT;

  ret = npzero_write(priv, NPZERO_CFGP(n), regval);
  if (ret < 0)
    {
      return ret;
    }

  regval = periph->dtype << NPZERO_MODP_DTYPE_SHIFT;
  if (periph->inverted)
    {
      regval |= NPZERO_MODP_CMOD;
    }

  if (periph->seqrw)
    {
      regval |= NPZERO_MODP_SEQRW;
    }

  if (periph->swap)
    {
      regval |= NPZERO_MODP_SWPRREG;
    }

  if (periph->spi)
    {
      regval |= periph->spimode << NPZERO_MODP_SPIMOD_SHIFT;
    }
  else if (periph->wunak)
    {
      regval |= NPZERO_MODP_WUNAK;
    }

  ret = npzero_write(priv, NPZERO_MODP(n), regval);
  if (ret < 0)
    {
      return ret;
    }

  ret = npzero_write16(priv, NPZERO_PERP_L(n), periph->period);
  if (ret < 0)
    {
      return ret;
    }

  ret = npzero_write(priv, NPZERO_NCMDP(n), periph->ninit);
  if (ret < 0)
    {
      return ret;
    }

  /* ADDR_P is the I2C address, or the number of SPI read bytes */

  if (periph->spi)
    {
      regval = NPZERO_ADDRP_SPIEN | periph->nread;
    }
  else
    {
      regval = periph->addr;
    }

  ret = npzero_write(priv, NPZERO_ADDRP(n), regval);
  if (ret < 0)
    {
      return ret;
    }

  ret = npzero_write(priv, NPZERO_RREGP(n), periph->spi ? 0 : periph->rreg);
  if (ret < 0)
    {
      return ret;
    }

  ret = npzero_write16(priv, NPZERO_THROVP_L(n), periph->throver);
  if (ret < 0)
    {
      return ret;
    }

  ret = npzero_write16(priv, NPZERO_THRUNP_L(n), periph->thrunder);
  if (ret < 0)
    {
      return ret;
    }

  ret = npzero_write(priv, NPZERO_TWTP(n), periph->twtp);
  if (ret < 0)
    {
      return ret;
    }

  regval = periph->twait << NPZERO_TCFGP_TWT_SHIFT |
           periph->tinit << NPZERO_TCFGP_TINIT_SHIFT;
  if (!periph->spi)
    {
      regval |= periph->retries << NPZERO_TCFGP_I2CRET_SHIFT;
    }

  return npzero_write(priv, NPZERO_TCFGP(n), regval);
}

/****************************************************************************
 * Name: npzero_configure
 ****************************************************************************/

static int npzero_configure(FAR struct npzero_dev_s *priv,
                            FAR const struct npzero_config_s *config)
{
  uint8_t sram[NPZERO_SRAM_SIZE];
  size_t sramlen;
  uint8_t regval;
  int ret;
  int i;

  if (config == NULL)
    {
      return -EINVAL;
    }

  memset(sram, 0, sizeof(sram));
  ret = npzero_check(config, sram, &sramlen);
  if (ret < 0)
    {
      pwrerr("ERROR: invalid configuration: %d\n", ret);
      return ret;
    }

  /* Global settings */

  ret = npzero_write16(priv, NPZERO_TOUT_L, config->timeout);
  if (ret < 0)
    {
      return ret;
    }

  regval = config->wakeup;
  if (config->wakeupall)
    {
      regval |= NPZERO_SYSCFG1_WUPMOD;
    }

  ret = npzero_write(priv, NPZERO_SYSCFG1, regval);
  if (ret < 0)
    {
      return ret;
    }

  regval = config->adcclk << NPZERO_SYSCFG2_ADC_CLK_SHIFT;
  if (config->sclkdiv != NPZERO_SCLKDIV_1)
    {
      regval |= NPZERO_SYSCFG2_SCLK_DIV_EN |
                (config->sclkdiv - 1) << NPZERO_SYSCFG2_SCLK_DIV_SHIFT;
    }

  if (config->sclksel == NPZERO_SCLK_XO)
    {
      regval |= NPZERO_SYSCFG2_SCLK_SEL;
    }

  if (config->adcext)
    {
      regval |= NPZERO_SYSCFG2_ADC_EXT_ON;
    }

  ret = npzero_write(priv, NPZERO_SYSCFG2, regval);
  if (ret < 0)
    {
      return ret;
    }

  regval = config->i2cpull << NPZERO_SYSCFG3_I2C_PUP_SHIFT |
           config->clkout << NPZERO_SYSCFG3_CLKOUT_SHIFT;
  if (config->iostrength)
    {
      regval |= NPZERO_SYSCFG3_IO_STR;
    }

  if (config->spiauto)
    {
      regval |= NPZERO_SYSCFG3_SPI_AUTO;
    }

  ret = npzero_write(priv, NPZERO_SYSCFG3, regval);
  if (ret < 0)
    {
      return ret;
    }

  regval = config->pswint | config->hostmode << NPZERO_PSWCTL_PSWH_SHIFT;
  if (config->gateboost)
    {
      regval |= NPZERO_PSWCTL_PSW_EN_VN;
    }

  ret = npzero_write(priv, NPZERO_PSWCTL, regval);
  if (ret < 0)
    {
      return ret;
    }

  regval = 0;
  for (i = 0; i < NPZERO_NPERIPH; i++)
    {
      regval |= config->intpull[i] << (2 * i);
    }

  ret = npzero_write(priv, NPZERO_INTCFG, regval);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialization commands of the peripherals */

  if (sramlen > 0)
    {
      ret = npzero_write_sram(priv, sram);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Peripherals */

  for (i = 0; i < NPZERO_NPERIPH; i++)
    {
      ret = npzero_config_periph(priv, i + 1, config->periph[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* ADC thresholds */

  ret = npzero_write(priv, NPZERO_THROVA1, config->adcover[0]);
  if (ret >= 0)
    {
      ret = npzero_write(priv, NPZERO_THRUNA1, config->adcunder[0]);
    }

  if (ret >= 0)
    {
      ret = npzero_write(priv, NPZERO_THROVA2, config->adcover[1]);
    }

  if (ret >= 0)
    {
      ret = npzero_write(priv, NPZERO_THRUNA2, config->adcunder[1]);
    }

  return ret;
}

/****************************************************************************
 * Name: npzero_ioctl
 ****************************************************************************/

static int npzero_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct npzero_dev_s *priv = filep->f_inode->i_private;
  uint8_t buf[2];
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case NPZEROIOC_GETID:
        {
          FAR uint8_t *id = (FAR uint8_t *)((uintptr_t)arg);

          DEBUGASSERT(id != NULL);
          ret = npzero_read(priv, NPZERO_ID_REG, id, 1);
        }
        break;

      case NPZEROIOC_GETSTATUS:
        {
          FAR struct npzero_status_s *status =
            (FAR struct npzero_status_s *)((uintptr_t)arg);

          DEBUGASSERT(status != NULL);
          ret = npzero_read(priv, NPZERO_STA1, buf, 2);
          if (ret >= 0)
            {
              status->sta1 = buf[0];
              status->sta2 = buf[1];
            }
        }
        break;

      case NPZEROIOC_CONFIGURE:
        ret = npzero_configure(priv,
                 (FAR const struct npzero_config_s *)((uintptr_t)arg));
        break;

      case NPZEROIOC_GETVALUE:
        {
          FAR struct npzero_value_s *value =
            (FAR struct npzero_value_s *)((uintptr_t)arg);

          DEBUGASSERT(value != NULL);
          if (value->periph < 1 || value->periph > NPZERO_NPERIPH)
            {
              ret = -EINVAL;
              break;
            }

          ret = npzero_read(priv, NPZERO_VALP_L(value->periph), buf, 2);
          if (ret >= 0)
            {
              value->value = (uint16_t)buf[1] << 8 | buf[0];
            }
        }
        break;

      case NPZEROIOC_GETADC:
        {
          FAR struct npzero_adc_s *adc =
            (FAR struct npzero_adc_s *)((uintptr_t)arg);

          DEBUGASSERT(adc != NULL);
          ret = npzero_read(priv, NPZERO_ADC_CORE, buf, 2);
          if (ret >= 0)
            {
              adc->core = buf[0];
              adc->ext  = buf[1];
            }
        }
        break;

      case NPZEROIOC_IDLE:
        ret = npzero_write(priv, NPZERO_IDLE_RST, NPZERO_IDLE_RST_IDLE);
        break;

      case NPZEROIOC_RESET:
        ret = npzero_write(priv, NPZERO_IDLE_RST, NPZERO_IDLE_RST_RESET);
        break;

      case NPZEROIOC_READREG:
        {
          FAR struct npzero_reg_s *reg =
            (FAR struct npzero_reg_s *)((uintptr_t)arg);

          DEBUGASSERT(reg != NULL);
          ret = npzero_read(priv, reg->reg, &reg->val, 1);
        }
        break;

      case NPZEROIOC_WRITEREG:
        {
          FAR const struct npzero_reg_s *reg =
            (FAR const struct npzero_reg_s *)((uintptr_t)arg);

          DEBUGASSERT(reg != NULL);

          /* Single SRAM writes can corrupt the SRAM (errata #2) */

          if (reg->reg >= NPZERO_SRAM)
            {
              ret = -EACCES;
              break;
            }

          ret = npzero_write(priv, reg->reg, reg->val);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: npzero_probe
 *
 * Description:
 *   Check the ID of the nPZero and apply the workaround of the errata #11
 *   if the nPZero was reset.
 *
 ****************************************************************************/

static int npzero_probe(FAR struct npzero_dev_s *priv)
{
  uint8_t buf[3];
  int ret;

  /* Read ID, STA1 and STA2 */

  ret = npzero_read(priv, NPZERO_ID_REG, buf, 3);
  if (ret < 0)
    {
      return ret;
    }

  if (buf[0] != NPZERO_ID)
    {
      pwrerr("ERROR: unexpected ID 0x%02x\n", buf[0]);
      return -ENODEV;
    }

  /* No wake-up flag: the nPZero was reset and its trim values need the
   * workaround of the errata #11.
   */

  if (buf[2] == 0 && (buf[1] & (NPZERO_STA1_FTOUT | NPZERO_STA1_FA1 |
                                NPZERO_STA1_FA2)) == 0)
    {
      ret = npzero_errata11(priv);
      if (ret < 0)
        {
          pwrerr("ERROR: errata #11 workaround failed: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: npzero_register
 *
 * Description:
 *   Check the nPZero G1S on the I2C bus, apply the errata workarounds that
 *   are needed after each reset of the nPZero and register the character
 *   driver.
 *
 * Input Parameters:
 *   devpath - The path of the device, e.g. "/dev/npzero0"
 *   i2c     - The I2C bus of the nPZero
 *   addr    - The 7-bit I2C address, NPZERO_I2C_ADDR
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int npzero_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr)
{
  FAR struct npzero_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL);

  priv = kmm_zalloc(sizeof(struct npzero_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c              = i2c;
  priv->config.frequency = CONFIG_NPZERO_I2C_FREQUENCY;
  priv->config.address   = addr;
  priv->config.addrlen   = 7;
  nxmutex_init(&priv->lock);

  ret = npzero_probe(priv);
  if (ret < 0)
    {
      goto errout;
    }

  ret = register_driver(devpath, &g_npzero_fops, 0666, priv);
  if (ret < 0)
    {
      pwrerr("ERROR: register_driver failed: %d\n", ret);
      goto errout;
    }

  return OK;

errout:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}

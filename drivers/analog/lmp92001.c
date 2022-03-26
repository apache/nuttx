/****************************************************************************
 * drivers/analog/lmp92001.c
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/dac.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/analog/lmp92001.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#if !defined(CONFIG_I2C)
#  error I2C Support Required.
#endif

#if defined(CONFIG_LMP92001)

#define LMP92001_REG_TEST   0x01u

#define LMP92001_REG_ID     0x0eu
#define LMP92001_REG_VER    0x0fu

#define LMP92001_REG_SGEN   0x10u
#define LMP92001_REG_SGPI   0x11u
#define LMP92001_REG_SHIL   0x12u
#define LMP92001_REG_SLOL   0x13u

#define LMP92001_REG_CGEN   0x14u
#define LMP92001_REG_CDAC   0x15u
#define LMP92001_REG_CGPO   0x16u
#define LMP92001_REG_CINH   0x17u
#define LMP92001_REG_CINL   0x18u
#define LMP92001_REG_CAD1   0x19u
#define LMP92001_REG_CAD2   0x1au
#define LMP92001_REG_CAD3   0x1bu
#define LMP92001_REG_CTRIG  0x1cu

#define LMP92001_REG_ADC1   0x20u
#define LMP92001_REG_ADC2   0x21u
#define LMP92001_REG_ADC3   0x22u
#define LMP92001_REG_ADC4   0x23u
#define LMP92001_REG_ADC5   0x24u
#define LMP92001_REG_ADC6   0x25u
#define LMP92001_REG_ADC7   0x26u
#define LMP92001_REG_ADC8   0x27u
#define LMP92001_REG_ADC9   0x28u
#define LMP92001_REG_ADC10  0x29u
#define LMP92001_REG_ADC11  0x2au
#define LMP92001_REG_ADC12  0x2bu
#define LMP92001_REG_ADC13  0x2cu
#define LMP92001_REG_ADC14  0x2du
#define LMP92001_REG_ADC15  0x2eu
#define LMP92001_REG_ADC16  0x2fu
#define LMP92001_REG_ADC17  0x30u

#define LMP92001_REG_LIH1   0x40u
#define LMP92001_REG_LIH2   0x41u
#define LMP92001_REG_LIH3   0x42u
#define LMP92001_REG_LIH9   0x43u
#define LMP92001_REG_LIH10  0x44u
#define LMP92001_REG_LIH11  0x45u
#define LMP92001_REG_LIL1   0x46u
#define LMP92001_REG_LIL2   0x47u
#define LMP92001_REG_LIL3   0x48u
#define LMP92001_REG_LIL9   0x49u
#define LMP92001_REG_LIL10  0x4au
#define LMP92001_REG_LIL11  0x4bu

#define LMP92001_REG_CREF   0x66u

#define LMP92001_REG_DAC1   0x80u
#define LMP92001_REG_DAC2   0x81u
#define LMP92001_REG_DAC3   0x82u
#define LMP92001_REG_DAC4   0x83u
#define LMP92001_REG_DAC5   0x84u
#define LMP92001_REG_DAC6   0x85u
#define LMP92001_REG_DAC7   0x86u
#define LMP92001_REG_DAC8   0x87u
#define LMP92001_REG_DAC9   0x88u
#define LMP92001_REG_DAC10  0x89u
#define LMP92001_REG_DAC11  0x8au
#define LMP92001_REG_DAC12  0x8bu

#define LMP92001_REG_DALL   0x90u

#define LMP92001_REG_BLK0   0xf0u
#define LMP92001_REG_BLK1   0xf1u
#define LMP92001_REG_BLK2   0xf2u
#define LMP92001_REG_BLK3   0xf3u
#define LMP92001_REG_BLK4   0xf4u
#define LMP92001_REG_BLK5   0xf5u

#define LMP92001_SGEN_BUSY  (1 << 7u)
#define LMP92001_SGEN_RDYN  (1 << 6u)
#define LMP92001_SGEN_HV    (1 << 2u)
#define LMP92001_SGEN_LV    (1 << 1u)
#define LMP92001_SGEN_GPI   (1 << 0u)

#define LMP92001_SGPI_GPI7  (1 << 7u)
#define LMP92001_SGPI_GPI6  (1 << 6u)
#define LMP92001_SGPI_GPI5  (1 << 5u)
#define LMP92001_SGPI_GPI4  (1 << 4u)
#define LMP92001_SGPI_GPI3  (1 << 3u)
#define LMP92001_SGPI_GPI2  (1 << 2u)
#define LMP92001_SGPI_GPI1  (1 << 1u)
#define LMP92001_SGPI_GPI0  (1 << 0u)

#define LMP92001_CGEN_RST   (1 << 7u)
#define LMP92001_CGEN_TOD   (1 << 2u)
#define LMP92001_CGEN_LCK   (1 << 1u)
#define LMP92001_CGEN_STRT  (1 << 0u)

#define LMP92001_CDAC_GANG  (1 << 2u)
#define LMP92001_CDAC_OLVL  (1 << 1u)
#define LMP92001_CDAC_OFF   (1 << 0u)
#define LMP92001_CDAC_ON    (0u)

#define LMP92001_CGPO_GPO7  (1 << 7u)
#define LMP92001_CGPO_GPO6  (1 << 6u)
#define LMP92001_CGPO_GPO5  (1 << 5u)
#define LMP92001_CGPO_GPO4  (1 << 4u)
#define LMP92001_CGPO_GPO3  (1 << 3u)
#define LMP92001_CGPO_GPO2  (1 << 2u)
#define LMP92001_CGPO_GPO1  (1 << 1u)
#define LMP92001_CGPO_GPO0  (1 << 0u)

#define LMP92001_CTRIG_SNGL (1 << 0u)

#define LMP92001_CREF_AEXT  (1 << 2u)
#define LMP92001_CREF_DEXT  (1 << 1u)

#define LMP92001_ADC_MAX_CHANNELS 17u
#define LMP92001_DAC_MAX_CHANNELS 12u
#define LMP92001_GPIO_MAX_PINS     8u

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lmp92001_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t addr;                  /* I2C address */

#ifdef CONFIG_ADC
  FAR const struct adc_callback_s *cb;
  uint8_t adc_channels_enabled;
  uint8_t adc_channels[LMP92001_ADC_MAX_CHANNELS];
#endif

#ifdef CONFIG_IOEXPANDER
  struct ioexpander_dev_s gpio_dev;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* LMP92001 Helpers. */

static int lmp92001_i2c_write(FAR struct lmp92001_dev_s *priv,
                              FAR const uint8_t *buffer, int buflen);
static int lmp92001_i2c_read(FAR struct lmp92001_dev_s *priv,
                             uint8_t reg, FAR uint8_t *buffer, int buflen);

static int lmp92001_dac_setref(FAR struct lmp92001_dev_s *priv,
                               enum lmp92001_ref_e ref);
static int lmp92001_dac_updateall(FAR struct lmp92001_dev_s *priv,
                                  uint16_t value);

static int lmp92001_adc_setref(FAR struct lmp92001_dev_s *priv,
                               enum lmp92001_ref_e ref);
static void lmp92001_adc_extractchannel(FAR struct lmp92001_dev_s *priv,
                                        enum lmp92001_adc_enable_e channels);
static int lmp92001_adc_enablechannel(FAR struct lmp92001_dev_s *priv,
                                      enum lmp92001_adc_enable_e channels);
static int lmp92001_adc_singleshot(FAR struct lmp92001_dev_s *priv);
static int lmp92001_adc_continuousconv(FAR struct lmp92001_dev_s *priv);
static int lmp92001_adc_readchannel(FAR struct lmp92001_dev_s *priv,
                                    FAR struct adc_msg_s *msg);

/* DAC Interface. */

#ifdef CONFIG_DAC
static void lmp92001_dac_reset(FAR struct dac_dev_s *dev);
static int  lmp92001_dac_setup(FAR struct dac_dev_s *dev);
static void lmp92001_dac_shutdown(FAR struct dac_dev_s *dev);
static void lmp92001_dac_txint(FAR struct dac_dev_s *dev, bool enable);
static int  lmp92001_dac_send(FAR struct dac_dev_s *dev,
                              FAR struct dac_msg_s *msg);
static int  lmp92001_dac_ioctl(FAR struct dac_dev_s *dev, int cmd,
                               unsigned long arg);
#endif

/* ADC Interface. */

#ifdef CONFIG_ADC
static int  lmp92001_adc_bind(FAR struct adc_dev_s *dev,
                              FAR const struct adc_callback_s *callback);
static void lmp92001_adc_reset(FAR struct adc_dev_s *dev);
static int  lmp92001_adc_setup(FAR struct adc_dev_s *dev);
static void lmp92001_adc_shutdown(FAR struct adc_dev_s *dev);
static void lmp92001_adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  lmp92001_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                               unsigned long arg);
#endif

/* I/O Expander Interface. */

#ifdef CONFIG_IOEXPANDER
static int lmp92001_gpio_direction(FAR struct ioexpander_dev_s *dev,
                                   uint8_t pin, int dir);
static int lmp92001_gpio_option(FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, int opt, void *regval);
static int lmp92001_gpio_writepin(FAR struct ioexpander_dev_s *dev,
                                  uint8_t pin, bool value);
static int lmp92001_gpio_readpin(FAR struct ioexpander_dev_s *dev,
                                 uint8_t pin, FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int lmp92001_gpio_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                       FAR uint8_t *pins, FAR bool *values,
                                       int count);
static int lmp92001_gpio_multireadpin(FAR struct ioexpander_dev_s *dev,
                                      FAR uint8_t *pins, FAR bool *values,
                                      int count);
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *lmp92001_gpio_attach(FAR struct ioexpander_dev_s *dev,
                                      ioe_pinset_t pinset,
                                      ioe_callback_t callback,
                                      FAR void *arg);
static int lmp92001_gpio_detach(FAR struct ioexpander_dev_s *dev,
                                FAR void *handle);
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lmp92001_dev_s g_devpriv;

#ifdef CONFIG_DAC
static const struct dac_ops_s g_dacops =
{
  lmp92001_dac_reset,           /* ao_reset */
  lmp92001_dac_setup,           /* ao_setup */
  lmp92001_dac_shutdown,        /* ao_shutdown */
  lmp92001_dac_txint,           /* ao_txint */
  lmp92001_dac_send,            /* ao_send */
  lmp92001_dac_ioctl            /* ao_ioctl */
};

static struct dac_dev_s g_dacdev =
{
  &g_dacops,    /* ad_ops */
  &g_devpriv    /* ad_priv */
};
#endif

#ifdef CONFIG_ADC
static const struct adc_ops_s g_adcops =
{
  lmp92001_adc_bind,            /* ao_bind */
  lmp92001_adc_reset,           /* ao_reset */
  lmp92001_adc_setup,           /* ao_setup */
  lmp92001_adc_shutdown,        /* ao_shutdown */
  lmp92001_adc_rxint,           /* ao_rxint */
  lmp92001_adc_ioctl            /* ao_ioctl */
};

static struct adc_dev_s g_adcdev =
{
  &g_adcops,    /* ad_ops */
  &g_devpriv    /* ad_priv */
};
#endif

#ifdef CONFIG_IOEXPANDER
static const struct ioexpander_ops_s g_gpio_ops =
{
  lmp92001_gpio_direction,
  lmp92001_gpio_option,
  lmp92001_gpio_writepin,
  lmp92001_gpio_readpin,
  lmp92001_gpio_readpin
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , lmp92001_gpio_multiwritepin
  , lmp92001_gpio_multireadpin
  , lmp92001_gpio_multireadpin
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , lmp92001_gpio_attach
  , lmp92001_gpio_detach
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lmp92001_i2c_write
 *
 * Description:
 *   Transmits a buffer.
 *
 ****************************************************************************/

static int lmp92001_i2c_write(FAR struct lmp92001_dev_s *priv,
                              FAR const uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_LMP92001_I2C_FREQUENCY,
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)buffer;  /* Override const */
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      aerr("LMP92001 I2C transfer failed: %d", ret);
      return ret;
    }

  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: lmp92001_i2c_read
 *
 * Description:
 *   Reads a buffer.
 *
 ****************************************************************************/

static int lmp92001_i2c_read(FAR struct lmp92001_dev_s *priv,
                             uint8_t reg, FAR uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg[2];
  int ret;

  /* Setup for the transfer */

  msg[0].frequency = CONFIG_LMP92001_I2C_FREQUENCY,
  msg[0].addr      = priv->addr,
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_LMP92001_I2C_FREQUENCY,
  msg[1].addr      = priv->addr,
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      aerr("LMP92001 I2C transfer failed: %d", ret);
      return ret;
    }

  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: lmp92001_dac_setref
 *
 * Description:
 *   Sets the DACs reference (internal or external).
 *
 ****************************************************************************/

static int lmp92001_dac_setref(FAR struct lmp92001_dev_s *priv,
                               enum lmp92001_ref_e ref)
{
  uint8_t value = 0;

  uint8_t const BUFFER_SIZE = 2u;
  uint8_t buffer[BUFFER_SIZE];

  int ret = OK;

  ret = lmp92001_i2c_read(priv, LMP92001_REG_CREF, &value, 1);
  if (ret < 0)
    {
      aerr("LMP92001 DAC set reference failed: %d", ret);
      return ret;
    }

  if (ref == LMP92001_REF_EXTERNAL)
    {
      value |= LMP92001_CREF_DEXT;
    }
  else
    {
      value &= ~LMP92001_CREF_DEXT;
    }

  buffer[0] = LMP92001_REG_CREF;
  buffer[1] = value;
  ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      aerr("LMP92001 DAC set reference failed: %d", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: lmp92001_dac_updateall
 *
 * Description:
 *   Write to all DACx registers simultaneously.
 *
 ****************************************************************************/

static int lmp92001_dac_updateall(FAR struct lmp92001_dev_s *priv,
                                  uint16_t value)
{
  uint8_t const BUFFER_SIZE = 3u;
  uint8_t buffer[BUFFER_SIZE];

  int ret = OK;

  buffer[0] = LMP92001_REG_DALL;
  buffer[1] = (uint8_t)(value >> 8u);
  buffer[2] = (uint8_t)(value & 0xffu);

  ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      aerr("LMP92001 DAC update all failed: %d", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: lmp92001_dac_reset
 *
 * Description:
 *   Reset the DAC device. Called early to initialize the hardware. This
 *   is called, before ao_setup() and on error conditions.
 *
 ****************************************************************************/

#ifdef CONFIG_DAC
static void lmp92001_dac_reset(FAR struct dac_dev_s *dev)
{
}

/****************************************************************************
 * Name: lmp92001_dac_setup
 *
 * Description:
 *   Configure the DAC. This method is called the first time that the DAC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching DAC interrupts. Interrupts
 *   are all disabled upon return.
 *
 ****************************************************************************/

static int lmp92001_dac_setup(FAR struct dac_dev_s *dev)
{
  FAR struct lmp92001_dev_s *priv =
    (FAR struct lmp92001_dev_s *)dev->ad_priv;

  uint8_t const BUFFER_SIZE = 2u;
  uint8_t buffer[BUFFER_SIZE];

  int ret = OK;

  /* Enable DAC channels. */

  buffer[0] = LMP92001_REG_CDAC;
  buffer[1] = LMP92001_CDAC_ON;
  ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      aerr("LMP92001 DAC setup failed: %d", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: lmp92001_dac_shutdown
 *
 * Description:
 *   Disable the DAC. This method is called when the DAC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void lmp92001_dac_shutdown(FAR struct dac_dev_s *dev)
{
}

/****************************************************************************
 * Name: lmp92001_dac_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void lmp92001_dac_txint(FAR struct dac_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: lmp92001_dac_send
 *
 * Description:
 *   This method will send one message on the DAC.
 *
 ****************************************************************************/

static int lmp92001_dac_send(FAR struct dac_dev_s *dev,
                             FAR struct dac_msg_s *msg)
{
  FAR struct lmp92001_dev_s *priv =
    (FAR struct lmp92001_dev_s *)dev->ad_priv;

  uint8_t const BUFFER_SIZE = 3u;
  uint8_t buffer[BUFFER_SIZE];

  int ret;

  /* Sanity check */

  DEBUGASSERT(priv->i2c != NULL);

  ainfo("Channel: %d - Value: %08x\n", msg->am_channel, msg->am_data);

  /* Set up the message to send.
   * We follow the same conventions as the datasheet.
   * ie. channels number start from 1 not from zero.
   */

  buffer[0] = (msg->am_channel - 1) + LMP92001_REG_DAC1;
  buffer[1] = (uint8_t)(msg->am_data >> 8u);
  buffer[2] = (uint8_t)(msg->am_data & 0xffu);

  ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      aerr("LMP92001 DAC send failed: %d", ret);
      return ret;
    }

  dac_txdone(&g_dacdev);

  return ret;
}

/****************************************************************************
 * Name: lmp92001_dac_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 ****************************************************************************/

static int lmp92001_dac_ioctl(FAR struct dac_dev_s *dev, int cmd,
                               unsigned long arg)
{
  FAR struct lmp92001_dev_s *priv =
    (FAR struct lmp92001_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_LMP92001_DAC_SET_REF:
        {
          ret = lmp92001_dac_setref(priv, (enum lmp92001_ref_e)(arg));
        }
        break;

      case ANIOC_LMP92001_DAC_UPDATEALL:
        {
          ret = lmp92001_dac_updateall(priv, (uint16_t)(arg));
        }
        break;

      /* Command was not recognized */

    default:
       aerr("LMP92001 ERROR: Unrecognized cmd: %d\n", cmd);
       ret = -ENOTTY;
       break;
    }

  return ret;
}
#endif /* CONFIG_DAC */

/****************************************************************************
 * Name: lmp92001_adc_setref
 *
 * Description:
 *   Sets the ADCs reference (internal or external).
 *
 ****************************************************************************/

static int lmp92001_adc_setref(FAR struct lmp92001_dev_s *priv,
                               enum lmp92001_ref_e ref)
{
  uint8_t value = 0;

  uint8_t const BUFFER_SIZE = 2u;
  uint8_t buffer[BUFFER_SIZE];

  int ret = OK;

  ret = lmp92001_i2c_read(priv, LMP92001_REG_CREF, &value, 1);
  if (ret < 0)
    {
      aerr("LMP92001 ADC set reference failed: %d", ret);
      return ret;
    }

  if (ref == LMP92001_REF_EXTERNAL)
    {
      value |= LMP92001_CREF_AEXT;
    }
  else
    {
      value &= ~LMP92001_CREF_AEXT;
    }

  buffer[0] = LMP92001_REG_CREF;
  buffer[1] = value;

  ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      aerr("LMP92001 ADC set reference failed: %d", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: lmp92001_adc_extractchannel
 *
 * Description:
 *   Extract channels from the enum.
 *
 ****************************************************************************/

static void lmp92001_adc_extractchannel(FAR struct lmp92001_dev_s *priv,
                                        enum lmp92001_adc_enable_e channels)
{
  uint8_t i;
  uint8_t j;
  uint16_t tmp;

  i   = 0u;
  j   = priv->adc_channels_enabled;
  tmp = channels;

  /* Channels to be enabled are the bits set in the argument channels. */

  while (tmp != 0u)
    {
      if (tmp & 1u)
        {
          priv->adc_channels_enabled++;
          priv->adc_channels[j] = i + 1;

          ainfo("Channel %d to be enabled\n", priv->adc_channels[j]);

          j++;
        }

      tmp >>= 1u;
      i++;
    }

  ainfo("Number of channels to be enabled: %d\n",
        priv->adc_channels_enabled);
}

/****************************************************************************
 * Name: lmp92001_adc_enablechannel
 *
 * Description:
 *   Enables the given ADC channels.
 *
 ****************************************************************************/

static int lmp92001_adc_enablechannel(FAR struct lmp92001_dev_s *priv,
                                      enum lmp92001_adc_enable_e channels)
{
  uint8_t cad1 = 0u;
  uint8_t cad2 = 0u;
  uint8_t cad3 = 0u;

  uint8_t const BUFFER_SIZE = 2u;
  uint8_t buffer[BUFFER_SIZE];

  int ret = OK;

  if (priv->adc_channels_enabled < LMP92001_ADC_MAX_CHANNELS &&
      channels != 0)
    {
      lmp92001_adc_extractchannel(priv, channels);

      ret = lmp92001_i2c_read(priv, LMP92001_REG_CAD1, &cad1, 1);
      if (ret < 0)
        {
          aerr("LMP92001 ADC enable channel failed %d\n", ret);
          return ret;
        }

      ret = lmp92001_i2c_read(priv, LMP92001_REG_CAD2, &cad2, 1);
      if (ret < 0)
        {
          aerr("LMP92001 ADC enable channel failed %d\n", ret);
          return ret;
        }

      ret = lmp92001_i2c_read(priv, LMP92001_REG_CAD3, &cad3, 1);
      if (ret < 0)
        {
          aerr("LMP92001 ADC enable channel failed %d\n", ret);
          return ret;
        }

      cad1 |= channels & 0x000ffu;
      cad2 |= (channels >> 8u) & 0x000ffu;
      cad3 |= (channels >> 16u) & 0x1u;

      if (cad1 > 0)
        {
          buffer[0] = LMP92001_REG_CAD1;
          buffer[1] = cad1;

          ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
          if (ret < 0)
            {
              aerr("LMP92001 ADC enable channel failed %d\n", ret);
              return ret;
            }
        }

      if (cad2 > 0)
        {
          buffer[0] = LMP92001_REG_CAD2;
          buffer[1] = cad2;

          ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
          if (ret < 0)
            {
              aerr("LMP92001 ADC enable channel failed %d\n", ret);
              return ret;
            }
        }

      if (cad3 > 0)
        {
          buffer[0] = LMP92001_REG_CAD3;
          buffer[1] = cad3;

          ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
          if (ret < 0)
            {
              aerr("LMP92001 ADC enable channel failed %d\n", ret);
              return ret;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: lmp92001_adc_singleshot
 *
 * Description:
 *   Starts a single shot conversion.
 *
 ****************************************************************************/

static int lmp92001_adc_singleshot(FAR struct lmp92001_dev_s *priv)
{
  uint8_t const BUFFER_SIZE = 2u;
  uint8_t buffer[BUFFER_SIZE];

  uint8_t sgen_value = 0;

  int ret = OK;

  /* Lock registers, this will also force CGEN.STRT to 0. */

  buffer[0] = LMP92001_REG_CGEN;
  buffer[1] = LMP92001_CGEN_LCK;
  ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      aerr("LMP92001 ADC Single shot failed %d\n", ret);
      return ret;
    }

  /* Trigger a single shot conversion. */

  buffer[0] = LMP92001_REG_CTRIG;
  buffer[1] = LMP92001_CTRIG_SNGL;
  ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      aerr("LMP92001 ADC Single shot failed %d\n", ret);
      return ret;
    }

  /* Wait for the BUSY flag. */

  do
    {
      lmp92001_i2c_read(priv, LMP92001_REG_SGEN, &sgen_value, 1);
    }
  while ((sgen_value & LMP92001_SGEN_BUSY) != 0);

  /* Unlock registers.
   * It's not needed to read the registers first and verify if some
   * bits were set. All the other bits should already be cleared.
   */

  buffer[0] = LMP92001_REG_CGEN;
  buffer[1] = 0;
  ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      aerr("LMP92001 ADC Single shot failed %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: lmp92001_adc_continuousconv
 *
 * Description:
 *   Starts the continuous conversion.
 *
 ****************************************************************************/

static int lmp92001_adc_continuousconv(FAR struct lmp92001_dev_s *priv)
{
  uint8_t const BUFFER_SIZE = 2u;
  uint8_t buffer[BUFFER_SIZE];

  uint8_t sgen_value = 0u;

  int ret = OK;

  /* Set CGEN.STRT bit and lock registers */

  buffer[0] = LMP92001_REG_CGEN;
  buffer[1] = LMP92001_CGEN_STRT | LMP92001_CGEN_LCK;

  ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      aerr("LMP92001 ADC Continuous conversion failed %d\n", ret);
      return ret;
    }

  /* Wait for the BUSY flag. */

  do
    {
      lmp92001_i2c_read(priv, LMP92001_REG_SGEN, &sgen_value, 1);
    }
  while ((sgen_value & LMP92001_SGEN_BUSY) != 0);

  /* Unlock the registers and keep STRT set. */

  buffer[0] = LMP92001_REG_CGEN;
  buffer[1] = LMP92001_CGEN_STRT;
  buffer[1] &= ~LMP92001_CGEN_LCK;

  ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      aerr("LMP92001 ADC Continuous conversion failed %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: lmp92001_adc_readchannel
 *
 * Description:
 *   Read from an ADC channel.
 *
 ****************************************************************************/

static int lmp92001_adc_readchannel(FAR struct lmp92001_dev_s *priv,
                                    FAR struct adc_msg_s *msg)
{
  uint8_t buffer[2];
  uint8_t channel;

  int ret = OK;

  /* Adding LMP92001_REG_ADC1 to a channel gives the channel's register.
   * Reminder: Channels numbering is the same as the datasheet.
   *           First channel is called Channel1.
   * Note: ADC registers are 16-bit wide.
   */

  channel = LMP92001_REG_ADC1 + msg->am_channel - 1;

  ret = lmp92001_i2c_read(priv, channel, buffer, 2);
  if (ret < 0)
    {
      aerr("LMP92001 ADC read failed: %d\n", ret);
      return ret;
    }

  msg->am_data = (buffer[0] << 8u) | buffer[1];

  return ret;
}

#ifdef CONFIG_ADC
/****************************************************************************
 * Name: lmp92001_adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *  This  must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int lmp92001_adc_bind(FAR struct adc_dev_s *dev,
                              FAR const struct adc_callback_s *callback)
{
  FAR struct lmp92001_dev_s *priv =
    (FAR struct lmp92001_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);

  priv->cb = callback;

  return OK;
}

/****************************************************************************
 * Name: lmp92001_adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before ao_setup() and on error conditions.
 *
 ****************************************************************************/

static void lmp92001_adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct lmp92001_dev_s *priv =
    (FAR struct lmp92001_dev_s *)dev->ad_priv;

  priv->adc_channels_enabled = 0;
}

/****************************************************************************
 * Name: lmp92001_adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int lmp92001_adc_setup(FAR struct adc_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: lmp92001_adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void lmp92001_adc_shutdown(FAR struct adc_dev_s *dev)
{
}

/****************************************************************************
 * Name: lmp92001_adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void lmp92001_adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: lmp92001_adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int lmp92001_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                              unsigned long arg)
{
  FAR struct lmp92001_dev_s *priv =
    (FAR struct lmp92001_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_LMP92001_ADC_ENABLE:
        {
          ret = lmp92001_adc_enablechannel(priv,
                                           (enum lmp92001_adc_enable_e)arg);
        }
        break;

      case ANIOC_LMP92001_ADC_SET_REF:
        {
          ret = lmp92001_adc_setref(priv, (enum lmp92001_ref_e)(arg));
        }
        break;

      case ANIOC_TRIGGER:
      case ANIOC_LMP92001_ADC_SINGLESHOT_CONV:
        {
          struct adc_msg_s msg;

          int i;

          ret = lmp92001_adc_singleshot(priv);
          if (ret < 0)
            {
              break;
            }

          for (i = 0; i < priv->adc_channels_enabled; i++)
            {
              msg.am_channel = priv->adc_channels[i];

              ret = lmp92001_adc_readchannel(priv, &msg);

              priv->cb->au_receive(&g_adcdev, priv->adc_channels[i],
                                   msg.am_data);
            }
        }
        break;

      case ANIOC_LMP92001_ADC_READ_CHANNEL:
        {
          FAR struct adc_msg_s *msg =
            (FAR struct adc_msg_s *)((uintptr_t)arg);

          ret = lmp92001_adc_singleshot(priv);
          if (ret < 0)
            {
              break;
            }

          ret = lmp92001_adc_readchannel(priv, msg);
        }
        break;

      case ANIOC_LMP92001_ADC_CONTINUOUS_CONV:
        {
          struct adc_msg_s msg;

          int i;

          ret = lmp92001_adc_continuousconv(priv);
          if (ret < 0)
            {
              break;
            }

          for (i = 0; i < priv->adc_channels_enabled; i++)
            {
              msg.am_channel = priv->adc_channels[i];

              ret = lmp92001_adc_readchannel(priv, &msg);

              priv->cb->au_receive(&g_adcdev, priv->adc_channels[i],
                                    msg.am_data);
            }
        }
        break;

      /* Command was not recognized */

      default:
        aerr("LMP92001 ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_ADC */

/****************************************************************************
 * Name: lmp92001_gpio_direction
 *
 * Description:
 *   Set the direction of an ioexpander pin. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   dir - One of the IOEXPANDER_DIRECTION_ macros
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER
static int lmp92001_gpio_direction(FAR struct ioexpander_dev_s *dev,
                                   uint8_t pin, int direction)
{
  FAR struct lmp92001_dev_s *priv = (FAR struct lmp92001_dev_s *)dev;

  uint8_t regval;
  uint8_t const BUFFER_SIZE = 2u;
  uint8_t buffer[BUFFER_SIZE];

  int ret;

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  DEBUGASSERT(priv != NULL && pin < LMP92001_GPIO_MAX_PINS);

  gpioinfo("I2C addr=%02x pin=%u direction=%s\n",
           priv->addr, pin,
           (direction == IOEXPANDER_DIRECTION_IN) ? "IN" : "OUT");

  ret = lmp92001_i2c_read(priv, LMP92001_REG_CGPO, &regval, 1);
  if (ret < 0)
    {
      gpioerr("LMP92001 GPIO set direction failed: %d\n", ret);
      return ret;
    }

  buffer[0] = LMP92001_REG_CGPO;
  if (direction == IOEXPANDER_DIRECTION_IN)
    {
      buffer[1] = regval | (1 << pin);
    }
  else
    {
      buffer[1] = regval & ~(1 << pin);
    }

  ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      gpioerr("LMP92001 GPIO set direction failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lmp92001_gpio_option
 *
 * Description:
 *   Set pin options. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   opt - One of the IOEXPANDER_OPTION_ macros
 *   val - The option's value
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int lmp92001_gpio_option(FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, int opt, void *regval)
{
  return OK;
}

/****************************************************************************
 * Name: lmp92001_gpio_writepin
 *
 * Description:
 *   Set the pin level. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   val - The pin level. Usually TRUE will set the pin high,
 *         except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int lmp92001_gpio_writepin(FAR struct ioexpander_dev_s *dev,
                                  uint8_t pin, bool value)
{
  FAR struct lmp92001_dev_s *priv = (FAR struct lmp92001_dev_s *)dev;

  uint8_t regval;
  uint8_t const BUFFER_SIZE = 2u;
  uint8_t buffer[BUFFER_SIZE];

  int ret;

  ret = lmp92001_i2c_read(priv, LMP92001_REG_CGPO, &regval, 1);
  if (ret < 0)
    {
      gpioerr("LMP92001 GPIO write pin failed: %d\n", ret);
      return ret;
    }

  buffer[0] = LMP92001_REG_CGPO;

  if (value)
    {
      buffer[1] = regval | (1 << pin);
    }
  else
    {
      buffer[1] = regval & ~(value << pin);
    }

  ret = lmp92001_i2c_write(priv, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      gpioerr("LMP92001 GPIO write pin failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lmp92001_gpio_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *   written to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   value  - Pointer to a buffer where the pin level is stored. Usually TRUE
 *            if the pin is high.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int lmp92001_gpio_readpin(FAR struct ioexpander_dev_s *dev,
                                 uint8_t pin, FAR bool *value)
{
  FAR struct lmp92001_dev_s *priv = (FAR struct lmp92001_dev_s *)dev;

  uint8_t regval;

  int ret;

  DEBUGASSERT(priv != NULL &&  pin < LMP92001_GPIO_MAX_PINS &&
              value != NULL);

  gpioinfo("I2C addr=%02x, pin=%u\n", priv->addr, pin);

  ret = lmp92001_i2c_read(priv, LMP92001_REG_SGPI, &regval, 1);
  if (ret < 0)
    {
      gpioerr("LMP92001 GPIO read pin failed: %d\n", ret);
      return ret;
    }

  *value = (bool)(regval >> pin) & 1u;

  return ret;
}

/****************************************************************************
 * Name: lmp92001_gpio_multiwritepin
 *
 * Description:
 *   Set the pin level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pins - The list of pin indexes to alter in this call
 *   val - The list of pin levels.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int lmp92001_gpio_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                       FAR uint8_t *pins, FAR bool *values,
                                       int count)
{
}

/****************************************************************************
 * Name: lmp92001_gpio_multireadpin
 *
 * Description:
 *   Read the actual level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The list of pin indexes to read
 *   values - Pointer to a buffer where the pin levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int lmp92001_gpio_multireadpin(FAR struct ioexpander_dev_s *dev,
                                      FAR uint8_t *pins, FAR bool *values,
                                      int count)
{
}
#endif

/****************************************************************************
 * Name: lmp92001_gpio_attach
 *
 * Description:
 *   Attach and enable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   pinset   - The set of pin events that will generate the callback
 *   callback - The pointer to callback function.  NULL will detach the
 *              callback.
 *   arg      - User-provided callback argument
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  This handle may be
 *   used later to detach and disable the pin interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *lmp92001_gpio_attach(FAR struct ioexpander_dev_s *dev,
                                       ioe_pinset_t pinset,
                                       ioe_callback_t callback,
                                       FAR void *arg)
{
}

/****************************************************************************
 * Name: lmp92001_gpio_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by lmp92001_gpio_attach()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int lmp92001_gpio_detach(FAR struct ioexpander_dev_s *dev,
                                FAR void *handle)
{
}
#endif
#endif /* CONFIG_IOEXPANDER */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lmp92001_dac_initialize
 *
 * Description:
 *   Initialize DAC
 *
 * Input Parameters:
 *   I2C Port number
 *   Device address
 *
 * Returned Value:
 *   Valid LMP92001 device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_DAC
FAR struct dac_dev_s *lmp92001_dac_initialize(FAR struct i2c_master_s *i2c,
                                              uint8_t addr)
{
  FAR struct lmp92001_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the LMP92001 device structure */

  priv       = (FAR struct lmp92001_dev_s *)g_dacdev.ad_priv;
  priv->i2c  = i2c;
  priv->addr = addr;

  return &g_dacdev;
}
#endif

/****************************************************************************
 * Name: lmp92001_adc_initialize
 *
 * Description:
 *   Initialize ADC
 *
 * Input Parameters:
 *   I2C Port number
 *   Device address
 *
 * Returned Value:
 *   Valid LMP92001 device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
FAR struct adc_dev_s *lmp92001_adc_initialize(FAR struct i2c_master_s *i2c,
                                               uint8_t addr)
{
  FAR struct lmp92001_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the LMP92001 device structure */

  priv       = (FAR struct lmp92001_dev_s *)g_adcdev.ad_priv;
  priv->cb   = NULL;
  priv->i2c  = i2c;
  priv->addr = addr;

  return &g_adcdev;
}
#endif

#ifdef CONFIG_IOEXPANDER
FAR struct ioexpander_dev_s *
lmp92001_gpio_initialize(FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct lmp92001_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the LMP92001 device structure */

  priv = &g_devpriv;

  priv->gpio_dev.ops = &g_gpio_ops;
  priv->i2c = i2c;
  priv->addr = addr;

  return &priv->gpio_dev;
}
#endif

#endif /* CONFIG_LMP92001 */

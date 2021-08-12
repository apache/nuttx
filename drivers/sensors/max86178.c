/****************************************************************************
 * drivers/sensors/max86178.c
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
#include <stdlib.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <math.h>

#include <sys/types.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/max86178.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/wqueue.h>

#include "max86178_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_MAX86178_I2C
# define MAX86178_I2C_MAX_FREQ   400000    /* Maximum SCL frequency 400kHz */
# define MAX86178_I2C_MIN_FREQ   100000    /* Minimum SCL frequency 100kHz */
# define MAX86178_I2C_ADDR_LEN   7         /* Only support 7bit I2C address */

#else /* CONFIG_SENSORS_MAX86178_SPI */
# define MAX86178_SPI_MAX_FREQ   25000000  /* Maximum SCLK frequency 25MHz */
# define MAX86178_SPI_MIN_FREQ   100000    /* Minimum SCLK frequency 100kHz */
# define MAX86178_SPI_NBITS      8         /* Only support 8 bits per word */

#endif

/* MAX86178 AFE contains ECG, PPG and BioZ AFE. CUrrently ECG and PPG is
 * used. Perhaps we will add BioZ AFE in the furture.
 */

#define MAX86178_ECG_IDX         0         /* ECG0 AFE index */
#define MAX86178_PPG_IDX         1         /* PPG0 (Green) AFE index */
#define MAX86178_IDX_NUM         2         /* Total AFE number. */

/* Default settings */

/* The 9 settings below are involved with each other and some other register
 * values which are not listed here. It means that ECG, PPG and BioZ sample
 * rates can't be set easily. Later we will add a function to calculate them
 * for any case, or seek a best one in an list of some combinations of these
 * settings, or set them manually in menuconfig. We're not sure which method
 * is most suitable, thus currently their values are limited.
 */

#define MAX86178_ECG_INTVL_DFT   4000      /* Default ECG interval = 4 ms */
#define MAX86178_PPG_INTVL_DFT   4000      /* Default PPG interval = 4 ms */
#define MAX86178_MDIV_DFT        250       /* Default MDIV for PLL = 8 MHz */
#define MAX86178_REF_CLK_DFT     32000     /* Default REF_CLK = 32kHz */

/* Default for ECG_PLL = 4 MHz */

#define MAX86178_ECG_FDIV_DFT    MAX86178_PLLCFG4_ECGFDIV_2
#define MAX86178_ECG_NDIV_DFT    125       /* ECG_ADC_CLK = 32kHz */
#define MAX86178_ECG_PLL_DFT     4000000   /* ECG_PLL_CLK = 4MHz */

/* Default ratio for divide ECG_ADC_CLK of 32 kHz to 250Hz sample rate */

#define MAX86178_ECG_DEC_DFT     MAX86178_ECGCFG1_DEC_RATE_128
#define MAX86178_PPG_DIV_DFT     128       /* For REF_CLK->FR_CLK = 250Hz */

/* The ECG range settings should vary with practices according to the
 * hardware design, and is determined through experiments.
 */

#define MAX86178_ECG_INARGE_DFT  MAX86178_ECGCFG2_INARGE_0
#define MAX86178_ECG_INAGAIN_DFT MAX86178_ECGCFG2_INAGAIN_1
#define MAX86178_ECG_PGAGAIN_DFT MAX86178_ECGCFG2_PGAGAIN_4

#define MAX86178_ECG_SR_MAX      2048        /* ECG sample rate = 2048 max. */
#define MAX86178_ECG_SR_MIN      64          /* ECG sample rate = 64 min. */
#define MAX86178_ECG_ADC_CLK_MAX 32768       /* ECG_ADC_CLK = 32768 Hz max. */
#define MAX86178_ECG_ADC_CLK_MIN 19000       /* ECG_ADC_CLK = 19.0kHz min. */
#define MAX86178_ECG_DECRATE_NUM 6           /* ECG_DEC_RATE has 6 choices. */

#define MAX86178_ONE_SECOND      1000000.0f  /* 1 second = 1000000 us */

#define MAX86178_CEILING(x, y)   ((x) + ((y)- 1)) / (y)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Sensor struct */

struct max86178_sensor_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct sensor_lowerhalf_s lower;            /* Lower half sensor driver */
  unsigned int              interval;         /* Sensor interval */
  unsigned int              batch;            /* Sensor batch */
  unsigned int              fifowtm;          /* Sensor fifo water marker */
  unsigned int              last_update;      /* Last update flag */
  float                     factor;           /* Readouts * factor = inputs */
  bool                      fifoen;           /* Sensor fifo enable */
  bool                      activated;        /* Sensor working state */
};

/* Device struct */

struct max86178_dev_s
{
  struct max86178_sensor_s     dev[MAX86178_IDX_NUM]; /* Sensor struct */
  uint64_t                     timestamp;             /* Units is us */
  FAR const struct max86178_config_s
                               *config;               /* The board config */
  struct work_s                work;                  /* Interrupt handler */
  uint32_t                     pllclk;                /* PLL CLK frequency */
  uint16_t                     fifowtm;               /* fifo water marker */
  bool                         fifoen;                /* Sensor fifo enable */
  bool                         activated;             /* Device state */
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* Interface (I2C/SPI) functions */

static int      max86178_readregs(FAR struct max86178_dev_s *priv,
                                  uint8_t startaddr, FAR uint8_t *recvbuf,
                                  size_t nbytes);
static int      max86178_writeregs(FAR struct max86178_dev_s *priv,
                                   uint8_t startaddr, FAR uint8_t *sendbuf,
                                   size_t nbytes);
static int      max86178_readsingle(FAR struct max86178_dev_s *priv,
                                    uint8_t regaddr,  uint8_t *regval);
static int      max86178_writesingle(FAR struct max86178_dev_s *priv,
                                     uint8_t regaddr, uint8_t regval);
#ifdef CONFIG_SENSORS_MAX86178_SPI
static int      max86178_configspi(FAR struct max86178_dev_s *priv);
#endif

/* MAX86178 handle functions */

static int      max86178_checkid(FAR struct max86178_dev_s *priv);
static int      max86178_softreset(FAR struct max86178_dev_s *priv);
static void     max86178_enable(FAR struct max86178_dev_s *priv,
                                bool enable);
static int      max86178_ecg_enable(FAR struct max86178_dev_s *priv,
                                    bool enable);
static int      max86178_ppg_enable(FAR struct max86178_dev_s *priv,
                                    bool enable);
static int      max86178_fifo_read(FAR struct max86178_dev_s *priv);
static uint32_t max86178_ppg_calcudata(FAR struct max86178_dev_s *priv,
                                       uint32_t sample);
static float    max86178_ecg_calcudata(FAR struct max86178_dev_s *priv,
                                       uint32_t sample);
static int      max86178_ppg_setfps(FAR struct max86178_dev_s *priv,
                                    float *freq);
static int      max86178_ecg_setsr(FAR struct max86178_dev_s *priv,
                                   float *freq);

/* Sensor ops functions */

static int      max86178_activate(FAR struct sensor_lowerhalf_s *lower,
                                  bool enable);
static int      max86178_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                      FAR unsigned int *period_us);
static int      max86178_batch(FAR struct sensor_lowerhalf_s *lower,
                               FAR unsigned int *latency_us);

/* Sensor interrupt functions */

static int      max86178_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                           ioe_pinset_t pinset,
                                           FAR void *arg);
static void     max86178_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const struct sensor_ops_s g_max86178_ecg_ops =
{
  .activate     = max86178_activate,
  .set_interval = max86178_set_interval,
  .batch        = max86178_batch,
};

static const struct sensor_ops_s g_max86178_ppg_ops =
{
  .activate     = max86178_activate,
  .set_interval = max86178_set_interval,
  .batch        = max86178_batch,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max86178_configspi
 *
 * Description:
 *   Set SPI mode, frequency and bits per word, according to max86178_dev_s->
 *   max86178_config_s.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   OK for success, a negated errno for error.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_MAX86178_SPI
static int max86178_configspi(FAR struct max86178_dev_s *priv)
{
  int freq = priv->config->freq;

  /* Set SPI frequency in an acceptable range. */

  if (freq > MAX86178_SPI_MAX_FREQ)
    {
      freq = MAX86178_SPI_MAX_FREQ;
    }
  else if (freq < MAX86178_SPI_MIN_FREQ)
    {
      freq = MAX86178_SPI_MIN_FREQ;
    }

  if (SPI_SETFREQUENCY(priv->config->spi, freq) != freq)
    {
      return -EIO;
    }

  /* MAX86178 SPI supports only mode0 and mode3. */

  SPI_SETMODE(priv->config->spi, SPIDEV_MODE0);

  /* Set number of bits per word. */

  SPI_SETBITS(priv->config->spi, MAX86178_SPI_NBITS);

  return OK;
}
#endif

/****************************************************************************
 * Name: max86178_readregs
 *
 * Description:
 *   Read MAX86178 registers from a start address. Except FIFODATA, each
 *   register has 8 bits and the register address will automatically increase
 *   until 0xff. Reading FIFODATA is an exception, where the register address
 *   will stay at FIFODATA(0x0c) and the samples in FIFO will be read one by
 *   one. In FIFO reading, note that each sample has 3 bytes.
 *
 * Input Parameters:
 *   priv      - Device struct.
 *   startaddr - Address of the 1st register to be read.
 *   recvbuf   - A pointer to the buffer which stores the readouts.
 *   nwords    - The numbers of word (8bits per word) to be read.
 *
 * Returned Value:
 *   OK for success, a negated errno for error.
 *
 ****************************************************************************/

static int max86178_readregs(FAR struct max86178_dev_s *priv,
                             uint8_t startaddr, FAR uint8_t *recvbuf,
                             size_t nwords)
{
  int ret;

#ifdef CONFIG_SENSORS_MAX86178_I2C
  struct i2c_config_s config;
  int freq = priv->config->freq;

  /* Set up the I2C configuration */

  if (freq > MAX86178_I2C_MAX_FREQ)
    {
      freq = MAX86178_I2C_MAX_FREQ;
    }
  else if (freq < MAX86178_I2C_MIN_FREQ)
    {
      freq = MAX86178_I2C_MIN_FREQ;
    }

  config.frequency = freq;
  config.address = priv->config->i2c_addr;
  config.addrlen = MAX86178_I2C_ADDR_LEN;

  /* After slave address with W, it send 1 byte of start address. Then it
   * repeats a start, sends slave address with R, and finally reads nbytes.
   */

  ret = i2c_writeread(priv->config->i2c, &config, &startaddr, 1,
                      recvbuf, nwords);
  if (ret < 0)
    {
      snerr("I2C writeread failed: %d\n", ret);
      return ret;
    }

  return OK;

#else /* CONFIG_SENSORS_MAX86178_SPI */

  /* For reading, the 1st byte is start address and the following bit is 1. */

  uint8_t txbuf[2] = {
    startaddr, 0x80,
    };

  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  ret = max86178_configspi(priv);
  if (ret < 0)
    {
      snerr("SPI configuration failed: %d\n", ret);
      SPI_LOCK(priv->config->spi, false);
      return ret;
    }

  /* Set CS to low which selects the device. */

  SPI_SELECT(priv->config->spi, priv->config->cs, true);

  /* First end start address and 0x80(means reading). Then read a block. */

  SPI_EXCHANGE(priv->config->spi, txbuf, NULL, 2);
  SPI_EXCHANGE(priv->config->spi, NULL, recvbuf, nwords);

  /* Deselect the device and release the SPI bus. */

  SPI_SELECT(priv->config->spi, priv->config->cs, false);
  SPI_LOCK(priv->config->spi, false);

  return OK;

#endif
}

/****************************************************************************
 * Name: max86178_writeregs
 *
 * Description:
 *   Write MAX86178 registers from a start address. The register address will
 *   automatically increase until 0xFF. DO NOT use on FIFODATA register. The
 *   datasheet didn't describe what will happen when writing several bytes
 *   to FIFODATA register. It's not sure if the address will increase or not.
 *
 * Input Parameters:
 *   priv      - Device struct.
 *   startaddr - Address of the 1st register to be written.
 *   recvbuf   - A pointer to the buffer which stores data to be written.
 *   nwords    - The numbers of word (8bits per word) to be written.
 *
 * Returned Value:
 *   OK for success, a negated errno for error.
 *
 ****************************************************************************/

static int max86178_writeregs(FAR struct max86178_dev_s *priv,
                              uint8_t startaddr, FAR uint8_t *sendbuf,
                              size_t nwords)
{
  int ret;

#ifdef CONFIG_SENSORS_MAX86178_I2C
  struct i2c_config_s config;
  int freq = priv->config->freq;

  /* Set up the I2C configuration */

  if (freq > MAX86178_I2C_MAX_FREQ)
    {
      freq = MAX86178_I2C_MAX_FREQ;
    }
  else if (freq < MAX86178_I2C_MIN_FREQ)
    {
      freq = MAX86178_I2C_MIN_FREQ;
    }

  config.frequency = freq;
  config.address = priv->config->i2c_addr;
  config.addrlen = MAX86178_I2C_ADDR_LEN;

  /* After slave address with W, it sends 1 byte of start address without a
   * STOP. When rbuflen, the i2c_writeread()'s parameter, is a negative, the
   * second i2c message didn't have a START condition and is a continuation
   * of the previous transfer without a STOP. So it will send |rbuflen| bytes
   * then send a STOP condition. See nuttx/drivers/i2c/i2c_wrtieread.c and
   * nuttx/include/nuttx/i2c/i2c_master.h for detail.
   */

  ret = i2c_writeread(priv->config->i2c, &config, &startaddr, 1,
                      sendbuf, -nwords);
  if (ret < 0)
    {
      snerr("I2C write failed: %d\n", ret);
      return ret;
    }

  return OK;

#else /* CONFIG_SENSORS_MAX86178_SPI */

  /* For writing, the 1st byte is start address and the following bit is 0. */

  uint8_t txbuf[2] = {
    startaddr, 0x00,
    };

  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  ret = max86178_configspi(priv);
  if (ret < 0)
    {
      snerr("SPI configuration failed: %d\n", ret);
      SPI_LOCK(priv->config->spi, false);
      return ret;
    }

  /* Set CS to low which selects the device. */

  SPI_SELECT(priv->config->spi, priv->config->cs, true);

  /* First end start address and 0x00(means writing). Then write a block. */

  SPI_EXCHANGE(priv->config->spi, txbuf, NULL, 2);
  SPI_EXCHANGE(priv->config->spi, sendbuf, NULL, nwords);

  /* Deselect the device and release the SPI bus. */

  SPI_SELECT(priv->config->spi, priv->config->cs, false);
  SPI_LOCK(priv->config->spi, false);

  return OK;

#endif
}

/****************************************************************************
 * Name: max86178_readsingle
 *
 * Description:
 *   Read the value of a single 8-bit register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Address of the register to be read from.
 *   regval  - A pointer to the variable to stores the readout.
 *
 * Returned Value:
 *   OK for success, a negated errno for error.
 *
 ****************************************************************************/

static int max86178_readsingle(FAR struct max86178_dev_s *priv,
                               uint8_t regaddr, uint8_t *regval)
{
  return max86178_readregs(priv, regaddr, regval, 1);
}

/****************************************************************************
 * Name: max86178_writesingle
 *
 * Description:
 *   Write the value to a single 8-bit register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Address of the destination register.
 *   regval  - The value to be written.
 *
 * Returned Value:
 *   OK for success, a negated errno for error.
 *
 ****************************************************************************/

static int max86178_writesingle(FAR struct max86178_dev_s *priv,
                                uint8_t regaddr, uint8_t regval)
{
  return max86178_writeregs(priv, regaddr, &regval, 1);
}

/****************************************************************************
 * Name: max86178_checkid
 *
 * Description:
 *   Read and verify the MAX86178 chip ID
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   OK for success, a negated errno for error.
 *
 ****************************************************************************/

static int max86178_checkid(FAR struct max86178_dev_s *priv)
{
  uint8_t devid;
  int ret;

  ret = max86178_readsingle(priv, MAX86178_REG_PARTID, &devid);
  if (ret < 0)
    {
      return ret;
    }

  if (devid != MAX86178_PARTID)
    {
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_softreset
 *
 * Description:
 *   All registers valuse will be set to power-on-reset state.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   OK for success, a negated errno for error.
 *
 ****************************************************************************/

static int max86178_softreset(FAR struct max86178_dev_s *priv)
{
  uint8_t regval;

  /* Disable PLL */

  max86178_readsingle(priv, MAX86178_REG_PLLCFG1, &regval);
  regval = regval & (~MAX86178_PLLCFG1_PLL_EN);
  max86178_writesingle(priv, MAX86178_REG_PLLCFG1, regval);

  /* Whatever the other bits in MAX86178_REG_SYSCFG1 are, they will be 0
   * (default) after softreset. Thus there's no need to read them out.
   */

  regval = MAX86178_SYSCFG1_RESET;

  return max86178_writesingle(priv, MAX86178_REG_SYSCFG1, regval);
}

/****************************************************************************
 * Name: max86178_enable
 *
 * Description:
 *   MAX86178 enter normal mode with PLL on, or shutdown mode with PLL off.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   enable - true(enter normal mode) and false(enter shutdown mode).
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void max86178_enable(FAR struct max86178_dev_s *priv, bool enable)
{
  uint8_t regval;

  if (enable)
    {
      /* If one wants to enabled the device when the device is not enabled,
       * enable it. If the device has been enabled, do nothing.
       */

      if (priv->activated == false)
        {
          /* Read and write MAX86178_SYSCFG1_NORMAL to exit shutdown mode */

          max86178_readsingle(priv, MAX86178_REG_SYSCFG1, &regval);
          regval = regval & (~MAX86178_SYSCFG1_SHDN_MASK);
          regval = regval | MAX86178_SYSCFG1_NORMAL;
          max86178_writesingle(priv, MAX86178_REG_PLLCFG1, regval);

          priv->activated = true;
        }
    }
  else
    {
      /* If one wants to disabled the device when the device is enabled,
       * disable it. Otherwise if the device has been disabled, do nothing.
       */

      if (priv->activated == true)
        {
          /* Read and write PLLCFG_PLL_EN to disable PLL if it's on. */

          max86178_readsingle(priv, MAX86178_REG_PLLCFG1, &regval);
          if (regval & MAX86178_PLLCFG1_PLL_EN_MASK)
            {
              regval = regval & (~MAX86178_PLLCFG1_PLL_EN_MASK);
              regval = regval | MAX86178_PLLCFG1_PLL_DIS;
              max86178_writesingle(priv, MAX86178_REG_PLLCFG1, regval);
            }

          /* Read and write MAX86178_SYSCFG1_NORMAL to enter shutdown mode. */

          max86178_readsingle(priv, MAX86178_REG_SYSCFG1, &regval);
          regval = regval & (~MAX86178_SYSCFG1_SHDN_MASK);
          regval = regval | MAX86178_SYSCFG1_SHDN;
          max86178_writesingle(priv, MAX86178_REG_PLLCFG1, regval);

          priv->activated = false;
        }
    }
}

/****************************************************************************
 * Name: max86178_ecg_enable
 *
 * Description:
 *   Enable or disable sensor device.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int max86178_ecg_enable(FAR struct max86178_dev_s *priv, bool enable)
{
  uint8_t regval;
  int i;

  if (enable)
    {
      /* To enable ECG, ensure the MAX86178 is enabled. */

      max86178_enable(priv, enable);

      /* Read and write PLL settings (PLL CLK = 8 MHz) */

      max86178_readsingle(priv, MAX86178_REG_PLLCFG6, &regval);
      regval = regval & (~MAX86178_PLLCFG6_CLKFRQSEL_MASK);
      regval = regval | MAX86178_PLLCFG6_REFCLK_32K;
      max86178_writesingle(priv, MAX86178_REG_PLLCFG1, regval);
      regval = MAX86178_MDIV_DFT;
      max86178_writesingle(priv, MAX86178_REG_MDIVLSB, regval);

      /* Read and write MAX86178_PLLCFG_PLL_EN to enable PLL */

      max86178_readsingle(priv, MAX86178_REG_PLLCFG1, &regval);
      regval = regval & (~MAX86178_PLLCFG1_MDIV_MSB_MASK);
      regval = regval & (~MAX86178_PLLCFG1_PLL_EN_MASK);
      regval = regval | MAX86178_PLLCFG1_PLL_EN;
      max86178_writesingle(priv, MAX86178_REG_PLLCFG1, regval);

      /* Wait until the PLL is locked or time is out. */

      for (i = 0; i < MAX86178_PLL_PHASE_LOCK_TIME; i++)
        {
          up_mdelay(1);

          max86178_readsingle(priv, MAX86178_REG_STAT3, &regval);
          if (regval & MAX86178_STAT3_PHASE_LOCK)
            {
              break;
            }
        }

      if (i == MAX86178_PLL_PHASE_LOCK_TIME)
        {
          return -ETIME;
        }

      /* Set ECG gain to default */

      max86178_readsingle(priv, MAX86178_REG_ECGCFG2, &regval);
      regval = regval & MAX86178_ECGCFG2_IPOL_MASK;
      regval = regval | MAX86178_ECG_PGAGAIN_DFT | MAX86178_ECG_INARGE_DFT |
                        MAX86178_ECG_INAGAIN_DFT;
      max86178_writesingle(priv, MAX86178_REG_ECGCFG2, regval);

      /* Connect ECG block inputs to MAX86178 pins */

      max86178_readsingle(priv, MAX86178_REG_ECGCFG3, &regval);
      regval = regval & (~MAX86178_ECGCFG3_MUXSEL_MASK);
      regval = regval | MAX86178_ECGCFG3_MUXSEL_EL123;
      max86178_writesingle(priv, MAX86178_REG_ECGCFG3, regval);

      /* Enable ECG */

      max86178_readsingle(priv, MAX86178_REG_ECGCFG1, &regval);
      regval = regval & (~MAX86178_ECGCFG1_ECG_EN_MASK);
      regval = regval | MAX86178_ECGCFG1_ECG_EN;
      max86178_writesingle(priv, MAX86178_REG_PLLCFG1, regval);
    }
  else
    {
      /* Disconnect ECG block inputs from MAX86178 pins */

      max86178_readsingle(priv, MAX86178_REG_ECGCFG3, &regval);
      regval = regval & (~MAX86178_ECGCFG3_MUXSEL_MASK);
      regval = regval | MAX86178_ECGCFG3_MUXSEL_NC;
      max86178_writesingle(priv, MAX86178_REG_ECGCFG3, regval);

      /* Disable ECG. */

      max86178_readsingle(priv, MAX86178_REG_ECGCFG1, &regval);
      regval = regval & (~MAX86178_ECGCFG1_ECG_EN_MASK);
      regval = regval | MAX86178_ECGCFG1_ECG_DIS;
      max86178_writesingle(priv, MAX86178_REG_PLLCFG1, regval);
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_ppg_enable
 *
 * Description:
 *   Enable or disable sensor device.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int max86178_ppg_enable(FAR struct max86178_dev_s *priv, bool enable)
{
  uint8_t regval;

  if (enable)
    {
      /* To enable PPG, ensure the MAX86178 is enabled. */

      max86178_enable(priv, enable);

      /* Disable PPG2 channel */

      max86178_readsingle(priv, MAX86178_REG_PPGCFG2, &regval);
      regval = regval & (~MAX86178_PPGCFG2_PPG2PWRDN_MASK);
      regval = regval | MAX86178_PPGCFG2_PPG2PWRDN;
      max86178_writesingle(priv, MAX86178_REG_PPGCFG2, regval);

      /* MEAS1 selects LED driver A to drive LED1 */

      max86178_readsingle(priv, MAX86178_REG_MEAS1SEL, &regval);
      regval = regval & (~MAX86178_MEASXSEL_DRVA_MASK);
      regval = regval | MAX86178_MEASXSEL_DRVA_LED1;
      max86178_writesingle(priv, MAX86178_REG_MEAS1SEL, regval);

      /* PPG1 selects PD1 */

      max86178_readsingle(priv, MAX86178_REG_MEAS1CFG5, &regval);
      regval = regval & (~MAX86178_MEASXCFG5_PD1SEL_MASK);
      regval = regval | MAX86178_MEASXCFG5_PD1SEL_PPG1;
      max86178_writesingle(priv, MAX86178_REG_MEAS1CFG5, regval);

      /* Selects LED current */

      max86178_readsingle(priv, MAX86178_REG_MEAS1CFG4, &regval);
      regval = regval & (~MAX86178_MEASXCFG4_LEDRGE_MASK);
      regval = regval | MAX86178_MEASXCFG4_LEDRGE_32MA;
      max86178_writesingle(priv, MAX86178_REG_MEAS1LEDA, regval);
      max86178_readsingle(priv, MAX86178_REG_MEAS1LEDA, &regval);
      regval = 0xff;
      max86178_writesingle(priv, MAX86178_REG_MEAS1LEDA, regval);

      /* Enable PPG MEAS1 */

      max86178_readsingle(priv, MAX86178_REG_PPGCFG1, &regval);
      regval = regval & (~MAX86178_PPGCFG1_MEAS1EN_MASK);
      regval = regval | MAX86178_PPGCFG1_MEAS1EN;
      max86178_writesingle(priv, MAX86178_REG_PPGCFG1, regval);
    }
  else
    {
      /* Disable PPG MEAS1 */

      max86178_readsingle(priv, MAX86178_REG_PPGCFG1, &regval);
      regval = regval & (~MAX86178_PPGCFG1_MEAS1EN_MASK);
      regval = regval | MAX86178_PPGCFG1_MEAS1DIS;
      max86178_writesingle(priv, MAX86178_REG_PPGCFG1, regval);
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_fifo_read
 *
 * Description:
 *   Read all available data in FIFO.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   1. ECG and PPG are synchronous sampled. Asynchronous sampling need
 *      timing data to determine the timestamp, and will be implented later.
 *   2. One of two PPG channels works at the same time. Perhaps we will add
 *      procession for 2 channels working the same time. Currently, only 1
 *      channel working at the same time is enough.
 *
 ****************************************************************************/

static int max86178_fifo_read(FAR struct max86178_dev_s *priv)
{
  struct sensor_event_ecg *temp_ecg;
  struct sensor_event_ppg *temp_ppg;
  FAR uint8_t *fifodata;
  uint32_t temp_sample;
  uint32_t counter_ecg = 0;
  uint32_t counter_ppg = 0;
  uint16_t num = 0;
  uint8_t temp_num;
  uint16_t i;

  temp_ecg = kmm_malloc(CONFIG_SENSORS_MAX86178_FIFO_SLOTS_NUMBER
                       * sizeof(struct sensor_event_ecg));
  temp_ppg = kmm_malloc(CONFIG_SENSORS_MAX86178_FIFO_SLOTS_NUMBER
                       * sizeof(struct sensor_event_ppg));
  memset(temp_ecg, 0x00, sizeof(temp_ecg));
  memset(temp_ppg, 0x00, sizeof(temp_ppg));

  /* Get number of samples in FIFO */

  max86178_readsingle(priv, MAX86178_REG_FIFOCNT1, &temp_num);
  if (temp_num & MAX86178_FIFOCNT1_CNT_MSB_MASK)
    {
      num = 256;
    }

  max86178_readsingle(priv, MAX86178_REG_FIFOCNT1, &temp_num);
  num = num + temp_num;

  /* Allocate a block for storing FIFO data */

  fifodata = kmm_malloc(priv->fifowtm * MAX86178_FIFO_BYTES_PER_DATA);
  if (fifodata == NULL)
    {
      snerr("ERROR: Failed to allocate space to store FIFO data\n");
      return -ENOMEM;
    }

  /* Read the FIFO in number of FIFO watermark */

  max86178_readregs(priv, MAX86178_REG_FIFODATA, fifodata, sizeof(fifodata));

  /* Deal each sample in FIFO. The last sample is the newest sample. */

  for (i = 0 ; i < num; i++)
    {
      temp_sample =
            (fifodata[i * MAX86178_FIFO_BYTES_PER_DATA] << 16) |
            (fifodata[i * MAX86178_FIFO_BYTES_PER_DATA + 1] << 8) |
            (fifodata[i * MAX86178_FIFO_BYTES_PER_DATA + 2]);

      switch (temp_sample & MAX86178_FIFOTAG_MASK_PRE)
        {
          case MAX86178_FIFOTAG_PRE_MEAS1:       /* PPG MEAS1 data tag. */
            {
              temp_ppg[counter_ppg].ppg =
                                   max86178_ppg_calcudata(priv, temp_sample);
              temp_ppg[counter_ppg].timestamp = priv->timestamp
                                   - priv->dev[MAX86178_PPG_IDX].interval
                                   * (priv->dev[MAX86178_PPG_IDX].fifowtm
                                   - counter_ppg - 1);
              counter_ppg++;
            }
            break;

          case MAX86178_FIFOTAG_PRE_ECG:         /* ECG data tag. */
            {
              temp_ecg[counter_ecg].ecg =
                                  max86178_ecg_calcudata(priv, temp_sample);
              temp_ecg[counter_ecg].timestamp = priv->timestamp
                                   - priv->dev[MAX86178_ECG_IDX].interval
                                   * (priv->dev[MAX86178_ECG_IDX].fifowtm
                                   - counter_ecg - 1);
              counter_ecg++;
            }
            break;

          default:                               /* Other data tags. */
            {
              /* Corresponding procession for the other tags will be added in
               * future.
               */
            }
            break;
        }
    }

  /* Release fifodata after dealing is done */

  kmm_free(fifodata);

  /* Since sometimes some extra special samples are saved in FIFO, the ECG
   * and PPG samples might not reach the watermark when the total samples
   * reached the total watermark. In this case, timestamp should be modified.
   */

  if (counter_ecg < CONFIG_SENSORS_MAX86178_FIFO_SLOTS_NUMBER)
    {
       for (i = 0; i < counter_ecg; i++)
         {
           temp_ecg[i].timestamp =
                       temp_ecg[i].timestamp
                       + priv->dev[MAX86178_ECG_IDX].interval
                       * (CONFIG_SENSORS_MAX86178_FIFO_SLOTS_NUMBER
                       - counter_ecg);
         }
    }

  if (counter_ppg < CONFIG_SENSORS_MAX86178_FIFO_SLOTS_NUMBER)
    {
       for (i = 0; i < counter_ppg; i++)
         {
           temp_ppg[i].timestamp =
                       temp_ppg[i].timestamp
                       + priv->dev[MAX86178_PPG_IDX].interval
                       * (CONFIG_SENSORS_MAX86178_FIFO_SLOTS_NUMBER
                       - counter_ppg);
         }
    }

  if (counter_ecg)
    {
      priv->dev[MAX86178_ECG_IDX].lower.push_event(
            priv->dev[MAX86178_ECG_IDX].lower.priv,
            temp_ecg,
            sizeof(struct sensor_event_ecg) * counter_ecg);
    }

  if (counter_ppg)
    {
      priv->dev[MAX86178_PPG_IDX].lower.push_event(
            priv->dev[MAX86178_PPG_IDX].lower.priv,
            temp_ppg,
            sizeof(struct sensor_event_ppg) * counter_ppg);
    }

  /* Release ecg and ppg data after push events have been done */

  kmm_free(temp_ecg);
  kmm_free(temp_ppg);

  return OK;
}

/****************************************************************************
 * Name: max86178_ppg_caludata
 *
 * Description:
 *   Calculate PPG value from origin sample data.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   sample - Origin PPG sample data from FIFO
 *
 * Returned Value:
 *   Calculated PPG value.
 *
 ****************************************************************************/

static uint32_t max86178_ppg_calcudata(FAR struct max86178_dev_s *priv,
                                       uint32_t sample)
{
  return sample & (~MAX86178_FIFOTAG_MASK_PRE);
}

/****************************************************************************
 * Name: max86178_ecg_calcudata
 *
 * Description:
 *   Calculate ECG value from origin sample data.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   sample - Origin ECG sample data from FIFO
 *
 * Returned Value:
 *   Calculated ECG value.
 *
 ****************************************************************************/

static float max86178_ecg_calcudata(FAR struct max86178_dev_s *priv,
                                    uint32_t sample)
{
  float temp_float;
  int temp_int;

  /* ECG value is a 2's complement code 18bits number. The MSB 6bits are tag.
   * The 17th bit is the sign bit.
   */

  sample = sample & (~MAX86178_FIFOTAG_MASK_POST);
  if (sample & MAX86178_ECG_SIGN_MASK)
    {
      sample = sample | MAX86178_FIFOTAG_MASK_POST;
    }

  temp_int = sample;
  temp_float = (float)temp_int;

  return (temp_float * priv->dev[MAX86178_ECG_IDX].factor);
}

/****************************************************************************
 * Name: max86178_ppg_setfps
 *
 * Description:
 *   Calculate and set the divide ratio for disired ECG sample rate.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   freq - Disired sample rate (Hz)
 *
 * Returned Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int max86178_ppg_setfps(FAR struct max86178_dev_s *priv, float *freq)
{
  uint16_t frdiv;
  uint8_t frdivregs[2];

  /* Currently timing data is disabled, then the FR_CLK comes from REF_CLK.
   * Enable any timing data will make the FR_CLK comes indirectly from PLL,
   * when the calculation shall be much more complex. If someday we need to
   * sample combined ECG and PPG asynchronously in case, we will consider
   * this confusing suitation.
   */

  frdiv = MAX86178_REF_CLK_DFT / *freq;
  if (frdiv < MAX86178_FRCLKDIV_MIN)
    {
      *freq = MAX86178_FRCLKDIV_MIN;
    }
  else if (frdiv > MAX86178_FRCLKDIV_MAX)
    {
      *freq = MAX86178_FRCLKDIV_MAX;
    }

  *freq = MAX86178_REF_CLK_DFT / frdiv;
  frdivregs[0] = frdiv >> 8;
  frdivregs[1] = frdiv;

  return max86178_writeregs(priv, MAX86178_REG_FRCLKDIVH, frdivregs, 2);
}

/****************************************************************************
 * Name: max86178_ecg_setsr
 *
 * Description:
 *   Calculate and set the divide ratio for disired PPG frame rate.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   freq - Disired sample rate (Hz)
 *
 * Returned Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int max86178_ecg_setsr(FAR struct max86178_dev_s *priv, float *freq)
{
  uint16_t decrate;
  uint16_t ndiv;
  uint8_t decrate_options[MAX86178_ECG_DECRATE_NUM] = {
    MAX86178_ECGCFG1_DEC_RATE_16, MAX86178_ECGCFG1_DEC_RATE_32,
    MAX86178_ECGCFG1_DEC_RATE_64, MAX86178_ECGCFG1_DEC_RATE_128,
    MAX86178_ECGCFG1_DEC_RATE_256, MAX86178_ECGCFG1_DEC_RATE_512,
    };

  uint16_t decratelist[MAX86178_ECG_DECRATE_NUM] = {
    MAX86178_ECG_DEC_RATE_16, MAX86178_ECG_DEC_RATE_32,
    MAX86178_ECG_DEC_RATE_64, MAX86178_ECG_DEC_RATE_128,
    MAX86178_ECG_DEC_RATE_256, MAX86178_ECG_DEC_RATE_512,
    };

  uint8_t regval;
  uint8_t i;

  if (*freq < MAX86178_ECG_SR_MIN)
    {
      *freq = MAX86178_ECG_SR_MIN;
    }
  else if (*freq > MAX86178_ECG_SR_MAX)
    {
      *freq = MAX86178_ECG_SR_MAX;
    }

  /* From little one to great one, the first DEC_RATE which makes ECG_ADC_CLK
   * greater than MAX86178_ECG_ADC_CLK_MIN, is the only DEC_RATE which can
   * make ECG_ADC_CLK to be in range of [MAX86178_ECG_ADC_CLK_MIN,
   * MAX86178_ECG_ADC_CLK_MAX].
   */

  for (i = 0; i < MAX86178_ECG_DECRATE_NUM; i++)
    {
      if (*freq * decratelist[i] > MAX86178_ECG_ADC_CLK_MIN)
        {
          break;
        }
    }

  decrate = decratelist[i];

  ndiv = MAX86178_ECG_PLL_DFT / (*freq * decrate);
  *freq = MAX86178_ECG_PLL_DFT / (ndiv * decrate);

  /* Set the registers */

  max86178_readsingle(priv, MAX86178_REG_ECGCFG1, &regval);
  regval = regval & (~MAX86178_ECGCFG1_DEC_RATE_MASK);
  regval = regval | decrate_options[i];
  max86178_writesingle(priv, MAX86178_REG_ECGCFG1, regval);

  max86178_readsingle(priv, MAX86178_REG_PLLCFG4, &regval);
  regval = regval & (~MAX86178_PLLCFG4_ECGNDIVH_MASK);
  regval = regval | ((ndiv >> 8) << MAX86178_PLLCFG4_ECGNDIVH_OFST);
  max86178_writesingle(priv, MAX86178_REG_PLLCFG4, regval);
  regval = ndiv;
  max86178_writesingle(priv, MAX86178_REG_ECGNDIVLSB, regval);

  return OK;
}

/****************************************************************************
 * Name: max86178_activate
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   lower  - The instance of lower half sensor driver
 *   enable - true(enable) and false(disable)
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int max86178_activate(FAR struct sensor_lowerhalf_s *lower,
                             bool enable)
{
  FAR struct max86178_sensor_s *sensor =
                               (FAR struct max86178_sensor_s *)lower;
  FAR struct max86178_dev_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL);

  if (lower->type == SENSOR_TYPE_ECG)
    {
      priv = (struct max86178_dev_s *)(sensor - MAX86178_ECG_IDX);
      if (sensor->activated != enable)
        {
          ret = max86178_ecg_enable(priv, enable);
          if (ret < 0)
            {
              snerr("Failed to enable light sensor: %d\n", ret);
              return ret;
            }

          if (enable)
            {
              IOEXP_SETOPTION(priv->config->ioedev,
                              priv->config->intpin,
                              IOEXPANDER_OPTION_INTCFG,
                              IOEXPANDER_VAL_FALLING);
            }

          sensor->activated = enable;
        }
    }
  else if(lower->type == SENSOR_TYPE_PPG)
    {
      priv = (struct max86178_dev_s *)(sensor - MAX86178_PPG_IDX);
      if (sensor->activated != enable)
        {
          ret = max86178_ppg_enable(priv, enable);
          if (ret < 0)
            {
              snerr("Failed to enable light sensor: %d\n", ret);
              return ret;
            }

          if (enable)
            {
              IOEXP_SETOPTION(priv->config->ioedev,
                              priv->config->intpin,
                              IOEXPANDER_OPTION_INTCFG,
                              IOEXPANDER_VAL_FALLING);
            }

          sensor->activated = enable;
        }
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  /* MAX86178 will be disabled if ECG and PPG are both disabled */

  if (priv->dev[MAX86178_ECG_IDX].activated == false
      && priv->dev[MAX86178_PPG_IDX].activated == false)
    {
      IOEXP_SETOPTION(priv->config->ioedev,
                      priv->config->intpin,
                      IOEXPANDER_OPTION_INTCFG,
                      IOEXPANDER_VAL_DISABLE);
      max86178_enable(priv, false);
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_set_interval
 *
 * Description:
 *   Set the sensor output data period in microseconds for a given sensor.
 *   If *period_us > max_delay it will be truncated to max_delay and if
 *   *period_us < min_delay it will be replaced by min_delay.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
 *   period_us  - The time between report data, in us. It may by overwrite
 *                by lower half driver.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int max86178_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR unsigned int *period_us)
{
  FAR struct max86178_sensor_s *sensor =
                                       (FAR struct max86178_sensor_s *)lower;
  FAR struct max86178_dev_s * priv;
  float freq;
  int ret;

  /* Sanity check */

  DEBUGASSERT(sensor != NULL && period_us != NULL);

  freq = MAX86178_ONE_SECOND / *period_us;

  if (lower->type == SENSOR_TYPE_ECG)
    {
      priv = (struct max86178_dev_s *)(sensor - MAX86178_ECG_IDX);

      /* Find the period that matches best.  */

      ret = max86178_ecg_setsr(priv, &freq);
      if (ret < 0)
        {
          snerr("Failed to set interval: %d\n", ret);
          return ret;
        }

      *period_us = MAX86178_ONE_SECOND / freq;
      priv->dev[MAX86178_ECG_IDX].interval = *period_us;
    }
  else if(lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      priv = (struct max86178_dev_s *)(sensor - MAX86178_PPG_IDX);

      /* Find the period that matches best.  */

      ret = max86178_ppg_setfps(priv, &freq);
      if (ret < 0)
        {
          snerr("Failed to set interval: %d\n", ret);
          return ret;
        }

      *period_us = MAX86178_ONE_SECOND / freq;
      priv->dev[MAX86178_PPG_IDX].interval = *period_us;
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: max86178_batch
 *
 * Description:
 *   Set sensor's maximum report latency in microseconds.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
 *   latency_us - the time between batch data, in us. It may by overwrite
 *                by lower half driver.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int max86178_batch(FAR struct sensor_lowerhalf_s *lower,
                          FAR unsigned int *latency_us)
{
  FAR struct max86178_sensor_s *sensor =
                                       (FAR struct max86178_sensor_s *)lower;
  FAR struct max86178_dev_s * priv;
  uint32_t max_latency;
  uint8_t regval;

  /* Sanity check */

  DEBUGASSERT(sensor != NULL && latency_us != NULL);

  max_latency = sensor->lower.batch_number * sensor->interval;
  if (*latency_us > max_latency)
    {
      *latency_us = max_latency;
    }
  else if (*latency_us < sensor->interval && *latency_us > 0)
    {
      *latency_us = sensor->interval;
    }

  sensor->fifowtm = MAX86178_CEILING(*latency_us, sensor->interval);
  *latency_us = sensor->fifowtm * sensor->interval;
  sensor->batch = *latency_us;

  if (sensor->fifowtm > 1)
    {
      sensor->fifoen = true;
      if (lower->type == SENSOR_TYPE_ECG)
        {
          priv = (struct max86178_dev_s *)(sensor - MAX86178_ECG_IDX);
        }
      else if(lower->type == SENSOR_TYPE_PPG)
        {
          priv = (struct max86178_dev_s *)(sensor - MAX86178_PPG_IDX);
        }
      else
        {
          snerr("Failed to match sensor type.\n");
          return -EINVAL;
        }
    }
  else
    {
      sensor->fifoen = false;
    }

  if (priv->dev[MAX86178_ECG_IDX].fifoen == false
      && priv->dev[MAX86178_PPG_IDX].fifoen == false)
    {
      /* Read and the remaining FIFO data. */

      if (priv->fifoen)
        {
          max86178_fifo_read(priv);
        }

      priv->fifoen = false;

      /* Disable interrupt pin */

      max86178_readsingle(priv, MAX86178_REG_PINFUNC, &regval);
      regval = regval & (~MAX86178_PINFUNC_INT1_FCFG_MASK);
      regval = regval | MAX86178_PINFUNC_INT1_DISABLE;
      max86178_writesingle(priv, MAX86178_REG_PINFUNC, regval);

      /* Disable FIFO_A_FULL interrupt */

      max86178_readsingle(priv, MAX86178_REG_INT1EN1, &regval);
      regval = regval & (~MAX86178_INTXEN1_AFULL_EN);
      max86178_writesingle(priv, MAX86178_REG_INT1EN1, regval);
    }
  else
    {
      priv->fifoen = true;
      priv->fifowtm = priv->dev[MAX86178_ECG_IDX].fifowtm
                    + priv->dev[MAX86178_PPG_IDX].fifowtm;

      /* Set water mark. STAT1_FIFOAFULL is set when FIFO reached 256 -
       * FIFOAFULL, where 256 is the FIFO size (uint in samples, 3Bytes per
       * sample).
       */

      regval = MAX86178_FIFO_SIZE - priv->fifowtm;
      max86178_writesingle(priv, MAX86178_REG_FIFOAFULL, regval);

      /* Set interrupt pin of MAX86178 */

      max86178_readsingle(priv, MAX86178_REG_PINFUNC, &regval);
      regval = regval & (~MAX86178_PINFUNC_INT1_FCFG_MASK);
      regval = regval | MAX86178_PINFUNC_INT1_RDCLR;
      max86178_writesingle(priv, MAX86178_REG_PINFUNC, regval);

      max86178_readsingle(priv, MAX86178_REG_PINCFG, &regval);
      regval = regval & (~MAX86178_PINCFG_INT1_OCFG_MASK);
      regval = regval | MAX86178_PINCFG_INT1_OD;
      max86178_writesingle(priv, MAX86178_REG_PINCFG, regval);

      /* Enable FIFO_A_FULL interrupt */

      max86178_readsingle(priv, MAX86178_REG_INT1EN1, &regval);
      regval = regval | MAX86178_INTXEN1_AFULL_EN;
      max86178_writesingle(priv, MAX86178_REG_INT1EN1, regval);
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_interrupt_handler
 *
 * Description:
 *   Handle the sensor interrupt.
 *
 * Input Parameters:
 *   dev     - ioexpander device.
 *   pinset  - Interrupt pin.
 *   arg     - Device struct.
 *
 * Returned Value:
 *   0 for success, a negated errno for any failure.
 *
 ****************************************************************************/

static int max86178_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                      ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the MAX86178 INTx
   * pin, when an event, such as FIFO is almost full, has occured.
   */

  FAR struct max86178_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp */

  priv->timestamp = sensor_get_timestamp();

  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long, and neither should we
   * lock the I2C/SPI bus within an interrupt.
   */

  work_queue(HPWORK, &priv->work, max86178_worker, priv, 0);
  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_DISABLE);

  return OK;
}

/****************************************************************************
 * Name: max86178_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data. We should not do
 *   this in a interrupt since it might take too long, and neither should we
 *   lock the I2C/SPI bus within an interrupt.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void max86178_worker(FAR void *arg)
{
  FAR struct max86178_dev_s *priv = arg;
  uint8_t status[5];

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_FALLING);

  /* Read registers from STAT1 to STAT5, where all interrupts come from. */

  max86178_readregs(priv, MAX86178_REG_STAT1, status, 5);

  /* If it's a FIFO almost full interrupt indicating FIFO reached the set
   * watermark, read and push the data to topics.
   */

  if (status[0] & MAX86178_STAT1_FIFOAFULL && priv->fifoen)
    {
      max86178_fifo_read(priv);
    }

  /* Procession for the other types of interrupt will be added later. */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max86178_register
 *
 * Description:
 *   Register the MAX86178 character device.
 *
 * Input Parameters:
 *   devno  - The full path to the driver to register. E.g., "/dev/max86178"
 *   config - An instance of the SPI interface to use to communicate with
 *             MAX86178
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int max86178_register(int devno, FAR const struct max86178_config_s *config)
{
  FAR struct max86178_dev_s *priv;
  int ioehandle;
  int ret;

  /* Sanity check */

  DEBUGASSERT(config != NULL);

  /* Initialize the MAX86178 device structure */

  priv = kmm_malloc(sizeof(*priv));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->config = config;

  priv->dev[MAX86178_ECG_IDX].lower.ops = &g_max86178_ecg_ops;
  priv->dev[MAX86178_ECG_IDX].lower.type = SENSOR_TYPE_ECG;
  priv->dev[MAX86178_ECG_IDX].lower.batch_number =
                              CONFIG_SENSORS_MAX86178_FIFO_SLOTS_NUMBER;
  priv->dev[MAX86178_ECG_IDX].interval = MAX86178_ECG_INTVL_DFT;

  priv->dev[MAX86178_PPG_IDX].lower.ops = &g_max86178_ppg_ops;
  priv->dev[MAX86178_PPG_IDX].lower.type = SENSOR_TYPE_PPG;
  priv->dev[MAX86178_PPG_IDX].lower.batch_number =
                              CONFIG_SENSORS_MAX86178_FIFO_SLOTS_NUMBER;
  priv->dev[MAX86178_PPG_IDX].interval = MAX86178_PPG_INTVL_DFT;

  /* Check the part ID */

  ret = max86178_checkid(priv);
  if (ret < 0)
    {
      snerr("ERROR: Device ID doesn't match: %d\n", ret);
      goto err;
    }

  /* Device soft-reset */

  ret = max86178_softreset(priv);
  if (ret < 0)
    {
      snerr("ERROR: Device can't be reset: %d\n", ret);
      goto err;
    }

  ret = IOEXP_SETDIRECTION(priv->config->ioedev, priv->config->intpin,
                           IOEXPANDER_DIRECTION_IN_PULLDOWN);
  if (ret < 0)
    {
      snerr("Failed to set direction: %d\n", ret);
      goto err;
    }

  ioehandle = IOEP_ATTACH(priv->config->ioedev, (1 << priv->config->intpin),
                          max86178_interrupt_handler, priv);
  if (ioehandle == 0)
    {
      snerr("Failed to attach: %d\n", ret);
      ret = -EIO;
      goto err;
    }

  ret = IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                        IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_DISABLE);
  if (ret < 0)
    {
      snerr("Failed to set option: %d\n", ret);
      goto err;
    }

  /* Register the character driver */

  ret = sensor_register((&(priv->dev[MAX86178_ECG_IDX].lower)), devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register ECG driver: %d\n", ret);
      goto err;
    }

  ret = sensor_register((&(priv->dev[MAX86178_PPG_IDX].lower)), devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register PPG driver: %d\n", ret);
      sensor_unregister((&(priv->dev[MAX86178_ECG_IDX].lower)), devno);
      goto err;
    }

  return ret;

err:
  IOEP_DETACH(priv->config->ioedev, max86178_interrupt_handler);
  kmm_free(priv);
  return ret;
}

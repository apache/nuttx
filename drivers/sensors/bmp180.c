/****************************************************************************
 * drivers/sensors/bmp180.c
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

/* Character driver for the Freescale BMP1801 Barometer Sensor */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmp180.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMP180)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMP180_ADDR         0x77
#define BMP180_FREQ         100000
#define DEVID               0x55

#define BMP180_AC1_MSB      0xaa
#define BMP180_AC1_LSB      0xab
#define BMP180_AC2_MSB      0xac
#define BMP180_AC2_LSB      0xad
#define BMP180_AC3_MSB      0xae
#define BMP180_AC3_LSB      0xaf
#define BMP180_AC4_MSB      0xb0
#define BMP180_AC4_LSB      0xb1
#define BMP180_AC5_MSB      0xb2
#define BMP180_AC5_LSB      0xb3
#define BMP180_AC6_MSB      0xb4
#define BMP180_AC6_LSB      0xb5
#define BMP180_B1_MSB       0xb6
#define BMP180_B1_LSB       0xb7
#define BMP180_B2_MSB       0xb8
#define BMP180_B2_LSB       0xb9
#define BMP180_MB_MSB       0xba
#define BMP180_MB_LSB       0xbb
#define BMP180_MC_MSB       0xbc
#define BMP180_MC_LSB       0xbd
#define BMP180_MD_MSB       0xbe
#define BMP180_MD_LSB       0xbf

#define BMP180_DEVID        0xd0
#define BMP180_SOFT_RESET   0xe0
#define BMP180_CTRL_MEAS    0xf4
#define BMP180_ADC_OUT_MSB  0xf6
#define BMP180_ADC_OUT_LSB  0xf7
#define BMP180_ADC_OUT_XLSB 0xf8

#define BMP180_READ_TEMP    0x2e
#define BMP180_READ_PRESS   0x34

#define BMP180_NOOVERSAMPLE 0x00
#define BMP180_OVERSAMPLE2X 0x70
#define BMP180_OVERSAMPLE4X 0xb0
#define BMP180_OVERSAMPLE8X 0xc0

/* Current Oversampling */

#define CURRENT_OSS         (BMP180_OVERSAMPLE8X)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct bmp180_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* BMP180 I2C address */
  int freq;                     /* BMP180 Frequency <= 3.4MHz */
  int16_t bmp180_cal_ac1;       /* Calibration coefficients */
  int16_t bmp180_cal_ac2;
  int16_t bmp180_cal_ac3;
  uint16_t bmp180_cal_ac4;
  uint16_t bmp180_cal_ac5;
  uint16_t bmp180_cal_ac6;
  int16_t bmp180_cal_b1;
  int16_t bmp180_cal_b2;
  int16_t bmp180_cal_mb;
  int16_t bmp180_cal_mc;
  int16_t bmp180_cal_md;
  int32_t bmp180_utemp;         /* Uncompensated temperature read from BMP180 */
  int32_t bmp180_upress;        /* Uncompensated pressure read from BMP180 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t bmp180_getreg8(FAR struct bmp180_dev_s *priv,
                              uint8_t regaddr);
static uint16_t bmp180_getreg16(FAR struct bmp180_dev_s *priv,
                                uint8_t regaddr);
static void bmp180_putreg8(FAR struct bmp180_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);
static void bmp180_updatecaldata(FAR struct bmp180_dev_s *priv);
static void bmp180_read_press_temp(FAR struct bmp180_dev_s *priv);
static int bmp180_getpressure(FAR struct bmp180_dev_s *priv);

/* Character driver methods */

static ssize_t bmp180_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t bmp180_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bmp180fops =
{
  NULL,                         /* open */
  NULL,                         /* close */
  bmp180_read,                  /* read */
  bmp180_write,                 /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmp180_getreg8
 *
 * Description:
 *   Read from an 8-bit BMP180 register
 *
 ****************************************************************************/

static uint8_t bmp180_getreg8(FAR struct bmp180_dev_s *priv, uint8_t regaddr)
{
  struct i2c_config_s config;
  uint8_t regval = 0;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read the register value */

  ret = i2c_read(priv->i2c, &config, &regval, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  return regval;
}

/****************************************************************************
 * Name: bmp180_getreg16
 *
 * Description:
 *   Read two 8-bit from a BMP180 register
 *
 ****************************************************************************/

static uint16_t bmp180_getreg16(FAR struct bmp180_dev_s *priv,
                                uint8_t regaddr)
{
  struct i2c_config_s config;
  uint16_t msb;
  uint16_t lsb;
  uint16_t regval = 0;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Register to read */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read register */

  ret = i2c_read(priv->i2c, &config, (uint8_t *)&regval, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* MSB and LSB are inverted */

  msb = (regval & 0xff);
  lsb = (regval & 0xff00) >> 8;

  regval = (msb << 8) | lsb;

  return regval;
}

/****************************************************************************
 * Name: bmp180_putreg8
 *
 * Description:
 *   Write to an 8-bit BMP180 register
 *
 ****************************************************************************/

static void bmp180_putreg8(FAR struct bmp180_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t data[2];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  data[0] = regaddr;
  data[1] = regval;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (uint8_t *) &data, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
    }
}

/****************************************************************************
 * Name: bmp180_checkid
 *
 * Description:
 *   Read and verify the BMP180 chip ID
 *
 ****************************************************************************/

static int bmp180_checkid(FAR struct bmp180_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID */

  devid = bmp180_getreg8(priv, BMP180_DEVID);
  sninfo("devid: 0x%02x\n", devid);

  if (devid != (uint16_t)DEVID)
    {
      /* ID is not Correct */

      snerr("ERROR: Wrong Device ID!\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bmp180_updatecaldata
 *
 * Description:
 *   Update Calibration Coefficient Data
 *
 ****************************************************************************/

static void bmp180_updatecaldata(FAR struct bmp180_dev_s *priv)
{
  /* AC1 */

  priv->bmp180_cal_ac1 = (int16_t) bmp180_getreg16(priv, BMP180_AC1_MSB);

  /* AC2 */

  priv->bmp180_cal_ac2 = (int16_t) bmp180_getreg16(priv, BMP180_AC2_MSB);

  /* AC3 */

  priv->bmp180_cal_ac3 = (int16_t) bmp180_getreg16(priv, BMP180_AC3_MSB);

  /* AC4 */

  priv->bmp180_cal_ac4 = bmp180_getreg16(priv, BMP180_AC4_MSB);

  /* AC5 */

  priv->bmp180_cal_ac5 = bmp180_getreg16(priv, BMP180_AC5_MSB);

  /* AC6 */

  priv->bmp180_cal_ac6 = bmp180_getreg16(priv, BMP180_AC6_MSB);

  /* B1 */

  priv->bmp180_cal_b1 = (int16_t) bmp180_getreg16(priv, BMP180_B1_MSB);

  /* B2 */

  priv->bmp180_cal_b2 = (int16_t) bmp180_getreg16(priv, BMP180_B2_MSB);

  /* MB */

  priv->bmp180_cal_mb = (int16_t) bmp180_getreg16(priv, BMP180_MB_MSB);

  /* MC */

  priv->bmp180_cal_mc = (int16_t) bmp180_getreg16(priv, BMP180_MC_MSB);

  /* MD */

  priv->bmp180_cal_md = (int16_t) bmp180_getreg16(priv, BMP180_MD_MSB);
}

/****************************************************************************
 * Name: bmp180_read_press_temp
 *
 * Description:
 *   Read raw pressure and temperature from BMP180 and store it in the
 *   bmp180_dev_s structure.
 *
 ****************************************************************************/

static void bmp180_read_press_temp(FAR struct bmp180_dev_s *priv)
{
  uint8_t oss = CURRENT_OSS;

  /* Issue a read temperature command */

  bmp180_putreg8(priv, BMP180_CTRL_MEAS, BMP180_READ_TEMP);

  /* Wait 5ms */

  nxsig_usleep(5000);

  /* Read temperature */

  priv->bmp180_utemp = bmp180_getreg16(priv, BMP180_ADC_OUT_MSB);

  /* Issue a read pressure command */

  bmp180_putreg8(priv, BMP180_CTRL_MEAS, (BMP180_READ_PRESS | oss));

  /* Delay 25.5ms (to OverSampling 8X) */

  nxsig_usleep(25500);

  /* Read pressure */

  priv->bmp180_upress = bmp180_getreg16(priv, BMP180_ADC_OUT_MSB) << 8;
  priv->bmp180_upress |= bmp180_getreg8(priv, BMP180_ADC_OUT_XLSB);
  priv->bmp180_upress = priv->bmp180_upress >> (8 - (oss >> 6));

  sninfo("Uncompensated temperature = %" PRId32 "\n", priv->bmp180_utemp);
  sninfo("Uncompensated pressure = %" PRId32 "\n", priv->bmp180_upress);
}

/****************************************************************************
 * Name: bmp180_getpressure
 *
 * Description:
 *   Calculate the Barometric Pressure using the temperature compensated
 *   See Freescale AN3785 and BMP1801 data sheet for details
 *
 ****************************************************************************/

static int bmp180_getpressure(FAR struct bmp180_dev_s *priv)
{
  int32_t x1;
  int32_t x2;
  int32_t x3;
  int32_t b3;
  int32_t b5;
  int32_t b6;
  int32_t press;
  int32_t temp;
  uint32_t b4;
  uint32_t b7;
  uint8_t oss = (CURRENT_OSS >> 6);

  /* Check if coefficient data were read correctly */

  if ((priv->bmp180_cal_ac1 == 0) || (priv->bmp180_cal_ac2 == 0) ||
      (priv->bmp180_cal_ac3 == 0) || (priv->bmp180_cal_ac4 == 0) ||
      (priv->bmp180_cal_ac5 == 0) || (priv->bmp180_cal_ac6 == 0) ||
      (priv->bmp180_cal_b1 == 0) || (priv->bmp180_cal_b2 == 0) ||
      (priv->bmp180_cal_mb == 0) || (priv->bmp180_cal_mc == 0) ||
      (priv->bmp180_cal_md == 0))
    {
      bmp180_updatecaldata(priv);
    }

  /* Read temperature and pressure */

  bmp180_read_press_temp(priv);

  /* Feed raw sensor data to entropy pool */

  add_sensor_randomness((priv->bmp180_utemp << 16) ^ priv->bmp180_upress);

  /* Calculate true temperature */

  x1   = ((priv->bmp180_utemp - priv->bmp180_cal_ac6) *
          priv->bmp180_cal_ac5) >> 15;
  x2   = (priv->bmp180_cal_mc << 11) / (x1 + priv->bmp180_cal_md);
  b5   = x1 + x2;

  temp = (b5 + 8) >> 4;
  sninfo("Compensated temperature = %" PRId32 "\n", temp);
  UNUSED(temp);

  /* Calculate true pressure */

  b6 = b5 - 4000;
  x1 = (priv->bmp180_cal_b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (priv->bmp180_cal_ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int32_t) priv->bmp180_cal_ac1) * 4 + x3) << oss) + 2) >> 2;
  x1 = (priv->bmp180_cal_ac3 * b6) >> 13;
  x2 = (priv->bmp180_cal_b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (priv->bmp180_cal_ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) (priv->bmp180_upress - b3) * (50000 >> oss));

  if (b7 < 0x80000000)
    {
      press = (b7 << 1) / b4;
    }
  else
    {
      press = (b7 / b4) << 1;
    }

  x1 = (press >> 8) * (press >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * press) >> 16;

  press = press + ((x1 + x2 + 3791) >> 4);

  sninfo("Compressed pressure = %" PRId32 "\n", press);
  return press;
}

/****************************************************************************
 * Name: bmp180_read
 ****************************************************************************/

static ssize_t bmp180_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmp180_dev_s *priv  = inode->i_private;
  FAR uint32_t            *press = (FAR uint32_t *) buffer;

  if (!buffer)
    {
      snerr("ERROR: Buffer is null\n");
      return -1;
    }

  if (buflen != 4)
    {
      snerr("ERROR: You can't read something "
            "other than 32 bits (4 bytes)\n");
      return -1;
    }

  /* Get the pressure compensated */

  *press = (int32_t) bmp180_getpressure(priv);

  /* Return size of uint32_t (4 bytes) */

  return 4;
}

/****************************************************************************
 * Name: bmp180_write
 ****************************************************************************/

static ssize_t bmp180_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmp180_register
 *
 * Description:
 *   Register the BMP180 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BMP180
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp180_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct bmp180_dev_s *priv;
  int ret;

  /* Initialize the BMP180 device structure */

  priv = (FAR struct bmp180_dev_s *)kmm_malloc(sizeof(struct bmp180_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = BMP180_ADDR;
  priv->freq = BMP180_FREQ;

  /* Check Device ID */

  ret = bmp180_checkid(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* Read the coefficient value */

  bmp180_updatecaldata(priv);

  /* Register the character driver */

  ret = register_driver(devpath, &g_bmp180fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BMP180 driver loaded successfully!\n");
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BMP180 */

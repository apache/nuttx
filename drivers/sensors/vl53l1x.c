/****************************************************************************
 * drivers/sensors/vl53l1x.c
 * Character driver for the ST vl53l1x distance.
 *
 *   Copyright (C) 2019 Acutronics Robotics
 *   Author: Acutronics Robotics (Juan Flores Muñoz) <juan@erlerobotics.com>
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
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/vl53l1x.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_VL53L1X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VL53L1X_FREQ  100000
#define VL53L1X_ADDR  0x29

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct vl53l1x_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* VL53L0X I2C address */
  int freq;                     /* VL53L0X Frequency */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_vl51l1x_default_config[] =
{
  0x00,
  0x00,
  0x00,
  0x01,
  0x02,
  0x00,
  0x02,
  0x08,
  0x00,
  0x08,
  0x10,
  0x01,
  0x01,
  0x00,
  0x00,
  0x00,
  0x00,
  0xff,
  0x00,
  0x0f,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x20,
  0x0b,
  0x00,
  0x00,
  0x02,
  0x0a,
  0x21,
  0x00,
  0x00,
  0x05,
  0x00,
  0x00,
  0x00,
  0x00,
  0xc8,
  0x00,
  0x00,
  0x38,
  0xff,
  0x01,
  0x00,
  0x08,
  0x00,
  0x00,
  0x01,
  0xdb,
  0x0f,
  0x01,
  0xf1,
  0x0d,
  0x01,
  0x68,
  0x00,
  0x80,
  0x08,
  0xb8,
  0x00,
  0x00,
  0x00,
  0x00,
  0x0f,
  0x89,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x01,
  0x0f,
  0x0d,
  0x0e,
  0x0e,
  0x00,
  0x00,
  0x02,
  0xc7,
  0xff,
  0x9b,
  0x00,
  0x00,
  0x00,
  0x01,
  0x00,
  0x00
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C operations */

static uint8_t vl53l1x_getreg8(FAR struct vl53l1x_dev_s *priv,
                               uint16_t regaddr);
static uint16_t vl53l1x_getreg16(FAR struct vl53l1x_dev_s *priv,
                                 uint16_t regaddr);
static uint32_t vl53l1x_getreg32(FAR struct vl53l1x_dev_s *priv,
                                 uint16_t regaddr);
static void vl53l1x_putreg8(FAR struct vl53l1x_dev_s *priv,
                            uint16_t regaddr, uint8_t regval);
static void vl53l1x_putreg16(FAR struct vl53l1x_dev_s *priv,
                             uint16_t regaddr, uint16_t regval);
static void vl53l1x_putreg32(FAR struct vl53l1x_dev_s *priv,
                             uint16_t regaddr, uint32_t regval);

/* Basic driver operations */

static void vl53l1x_startranging(FAR struct vl53l1x_dev_s *priv);
static void vl53l1x_stopranging(FAR struct vl53l1x_dev_s *priv);
static void vl53l1x_sensorinit(FAR struct vl53l1x_dev_s *priv);
static void vl53l1x_getdistance(FAR struct vl53l1x_dev_s *priv,
                                FAR uint16_t *distance);

/* Set data operations */

static void vl53l1x_settimingbudget(FAR struct vl53l1x_dev_s *priv,
                                    uint16_t ms_value);
static void vl53l1x_setdistancemode(FAR struct vl53l1x_dev_s *priv,
                                    uint16_t mode);

/* Get data operations */

static void vl53l1x_gettimingbudget(FAR struct vl53l1x_dev_s *priv,
                                    FAR uint16_t *ms_value);
static void vl53l1x_getdistancemode(FAR struct vl53l1x_dev_s *priv,
                                    FAR uint16_t *dm);
static void vl53l1x_getid(FAR struct vl53l1x_dev_s *priv, FAR uint16_t *id);
static void vl53l1x_getoffset(FAR struct vl53l1x_dev_s *priv,
                              FAR int16_t *offset);
static void vl53l1x_getintstate(FAR struct vl53l1x_dev_s *priv,
                                FAR uint8_t *state);

/* Other operations */

static void vl53l1x_clearint(FAR struct vl53l1x_dev_s *priv);
static void vl53l1x_dataready(FAR struct vl53l1x_dev_s *priv,
                              FAR uint8_t *dataready);
static void vl53l1x_tempupdate(FAR struct vl53l1x_dev_s *priv);
static void vl53l1x_calibrateoffset(FAR struct vl53l1x_dev_s *priv,
                                    uint16_t target_distance,
                                    FAR int16_t *offset);

/* Character driver methods */

static void vl53l1x_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t vl53l1x_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static void vl53l1x_ioctl(FAR struct file *filep, int cmd, uint16_t arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_vl53l1xfops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  vl53l1x_read,         /* read */
  vl53l1x_write,        /* write */
  NULL,                 /* seek */
  vl53l1x_ioctl,        /* ioctl */
  NULL                  /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vl53l1x_SensorInit
 *
 * Description:
 *  This function loads the 135 bytes default values to initialize the
 *  sensor.
 *
 ****************************************************************************/

static void vl53l1x_sensorinit(FAR struct vl53l1x_dev_s *priv)
{
  uint8_t addr = 0x00;
  uint8_t tmp = 0;

  for (addr = 0x2d; addr <= 0x87; addr++)
    {
      vl53l1x_putreg8(priv, addr, g_vl51l1x_default_config[addr - 0x2d]);
    }

  vl53l1x_startranging(priv);

  while (tmp == 0)
    {
      vl53l1x_dataready(priv, &tmp);
    }

  vl53l1x_clearint(priv);
  vl53l1x_stopranging(priv);
  vl53l1x_putreg8(priv, TIMEOUT_MACROP_LOOP_BOUND, 0x09);
  vl53l1x_putreg8(priv, 0x0b, 0);
}

/****************************************************************************
 * Name: vl53l1x_clearint
 *
 * Description:
 *   This function clears the interrupt, to be called after a ranging data
 *   reading to arm the interrupt for the next data ready event.
 *
 ****************************************************************************/

static void vl53l1x_clearint(FAR struct vl53l1x_dev_s *priv)
{
  vl53l1x_putreg8(priv, INTERRUPT_CLEAR, 0x01);
}

/****************************************************************************
 * Name: vl53l1x_startranging
 *
 * Description:
 *   This function starts the ranging distance operation. The ranging ops
 *   is continuous. The clear interrupt has to be done after each get data to
 *   allow the interrupt to raise when the next data is ready.
 *
 ****************************************************************************/

static void vl53l1x_startranging(FAR struct vl53l1x_dev_s *priv)
{
  vl53l1x_putreg8(priv, SYSTEM_MODE, 0x40);
}

/****************************************************************************
 * Name: vl53l1x_StopRanging
 *
 * Description:
 *   This function stops the ranging.
 *
 ****************************************************************************/

static void vl53l1x_stopranging(FAR struct vl53l1x_dev_s *priv)
{
  vl53l1x_putreg8(priv, SYSTEM_MODE, 0x00);
}

/****************************************************************************
 * Name: vl53l1x_dataready
 *
 * Description:
 *   This function checks if the new ranging data is available by polling the
 *   dedicated register.
 *
 ****************************************************************************/

static void vl53l1x_dataready(FAR struct vl53l1x_dev_s *priv,
                              FAR uint8_t *dataready)
{
  uint8_t temp;
  uint8_t intpol;

  temp = vl53l1x_getreg8(priv, GPIO_STATUS);
  vl53l1x_getintstate(priv, &intpol);

  /* Read in the register to check if a new value is available */

  if ((temp & 1) == intpol)
    {
      *dataready = 1;
    }
  else
    {
      *dataready = 0;
    }
}

/****************************************************************************
 * Name: vl53l1x_getinterruptstate
 *
 * Description:
 *   This function returns the current interrupt polarity.
 *
 ****************************************************************************/

static void vl53l1x_getintstate(FAR struct vl53l1x_dev_s *priv,
                                FAR uint8_t *state)
{
  uint8_t temp;

  temp   = vl53l1x_getreg8(priv, GPIO_MUX_CTRL);
  temp   = temp & 0x10;
  *state = !(temp >> 4);
}

/****************************************************************************
 * Name: VL53L1X_GetTimingBudgetInMs
 *
 * Description:
 *   This function returns the timing budget in ms.
 *
 ****************************************************************************/

static void vl53l1x_gettimingbudget(FAR struct vl53l1x_dev_s *priv,
                                    FAR uint16_t *ms_value)
{
  uint16_t temp;

  temp = vl53l1x_getreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI);

  switch (temp)
    {
    case 0x001d:
      *ms_value = 15;
      break;

    case 0x0051:
    case 0x001e:
      *ms_value = 20;
      break;

    case 0x00d6:
    case 0x0060:
      *ms_value = 33;
      break;

    case 0x1ae:
    case 0x00ad:
      *ms_value = 50;
      break;

    case 0x02e1:
    case 0x01cc:
      *ms_value = 100;
      break;

    case 0x03e1:
    case 0x02d9:
      *ms_value = 200;
      break;

    case 0x0591:
    case 0x048f:
      *ms_value = 500;
      break;

    default:
      *ms_value = 0;
      break;
    }
}

/****************************************************************************
 * Name: vl53l1x_settimingbudget
 *
 * Description:
 *   This function programs the timing budget in ms.
 *
 ****************************************************************************/

static void vl53l1x_settimingbudget(FAR struct vl53l1x_dev_s *priv,
                                    uint16_t ms_value)
{
  uint16_t dm;
  vl53l1x_getdistancemode(priv, &dm);

  /* Short Distance */

  if (dm == 1)
    {
      switch (ms_value)
        {
        case 15:
          {
            vl53l1x_putreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI, 0x01d);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x0027);
          }
          break;

        case 20:
          {
            vl53l1x_putreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI, 0x0051);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x006e);
          }
          break;

        case 33:
          {
            vl53l1x_putreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI, 0x00d6);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x006e);
          }
          break;

        case 50:
          {
            vl53l1x_putreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI, 0x1ae);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x01e8);
          }
          break;

        case 100:
          {
            vl53l1x_putreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI, 0x02e1);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x0388);
          }
          break;

        case 200:
          {
            vl53l1x_putreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI, 0x03e1);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x0496);
          }
          break;

        case 500:
          {
            vl53l1x_putreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI, 0x0591);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x05c1);
          }
          break;

        default:
          break;
        }
    }
  else
    {
      switch (ms_value)
        {
        case 20:
          {
            vl53l1x_putreg16(priv, PHASECAL_TIMEOUT_MACRO, 0x001e);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x0022);
          }
          break;

        case 33:
          {
            vl53l1x_putreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI, 0x0060);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x006e);
          }
          break;

        case 50:
          {
            vl53l1x_putreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI, 0x00ad);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x00c6);
          }
          break;

        case 100:
          {
            vl53l1x_putreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI, 0x01cc);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x01ea);
          }
          break;

        case 200:
          {
            vl53l1x_putreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI, 0x02d9);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x02f8);
          }
          break;

        case 500:
          {
            vl53l1x_putreg16(priv, RANGE_CFG_TIMEOUT_MACRO_HI, 0x048f);
            vl53l1x_putreg16(priv, RANGE_TIMEOUT_MACRO_HI, 0x04a4);
          }
          break;

        default:
          break;
        }
    }
}

/****************************************************************************
 * Name: vl53l1x_setdistancemode
 *
 * Description:
 *   This function programs the distance mode (1=short, 2=long(default)).
 *   Short mode max distance is limited to 1.3 m but better ambient immunity.
 *   Long mode can range up to 4 m in the dark with 200 ms timing budget.
 *
 ****************************************************************************/

static void vl53l1x_setdistancemode(FAR struct vl53l1x_dev_s *priv,
                                    uint16_t dm)
{
  uint16_t tb;

  vl53l1x_gettimingbudget(priv, &tb);
  switch (dm)
    {
    case 1:
      {
        vl53l1x_putreg8(priv, PHASECAL_TIMEOUT_MACRO, 0x14);
        vl53l1x_putreg8(priv, RANGE_VCSEL_PERIOD_A, 0x07);
        vl53l1x_putreg8(priv, RANGE_VCSEL_PERIOD_B, 0x05);
        vl53l1x_putreg8(priv, RANGE_CFG_VALID_PHASE, 0x38);
        vl53l1x_putreg16(priv, SD_CFG_WOI_SD0, 0x0705);
        vl53l1x_putreg16(priv, SD_CFG_INIT_PHASE, 0x0606);
      }
      break;

    case 2:
      {
        vl53l1x_putreg8(priv, PHASECAL_TIMEOUT_MACRO, 0x0a);
        vl53l1x_putreg8(priv, RANGE_VCSEL_PERIOD_A, 0x0f);
        vl53l1x_putreg8(priv, RANGE_VCSEL_PERIOD_B, 0x0d);
        vl53l1x_putreg8(priv, RANGE_CFG_VALID_PHASE, 0xb8);
        vl53l1x_putreg16(priv, SD_CFG_WOI_SD0, 0x0f0d);
        vl53l1x_putreg16(priv, SD_CFG_INIT_PHASE, 0x0e0e);
      }
      break;

    default:
      break;
    }

  vl53l1x_settimingbudget(priv, tb);
}

/****************************************************************************
 * Name: vl53l1x_getdistancemode
 *
 * Description:
 *   This function returns the current distance mode (1=short, 2=long).
 *
 ****************************************************************************/

static void vl53l1x_getdistancemode(FAR struct vl53l1x_dev_s *priv,
                                    FAR uint16_t *dm)
{
  uint8_t mode_read;

  mode_read = vl53l1x_getreg8(priv, PHASECAL_TIMEOUT_MACRO);
  if (mode_read == 0x14)
    {
      *dm = 1;
    }

  if (mode_read == 0x0a)
    {
      *dm = 2;
    }
}

/****************************************************************************
 * Name: vl53l1x_getid
 *
 * Description:
 *  This function returns the sensor id, sensor Id must be 0xeeac.
 *
 ****************************************************************************/

static void vl53l1x_getid(FAR struct vl53l1x_dev_s *priv, FAR uint16_t *id)
{
  uint16_t tmp = 0;

  tmp = vl53l1x_getreg16(priv, VL53L1_GET_ID);
  *id = tmp;
}

/****************************************************************************
 * Name: vl53l1x_GetDistance
 *
 * Description:
 *  This function returns the distance measured by the sensor in mm.
 *
 ****************************************************************************/

static void vl53l1x_getdistance(FAR struct vl53l1x_dev_s *priv,
                                FAR uint16_t *distance)
{
  uint16_t tmp;

  tmp = vl53l1x_getreg16(priv, VL53L1_GET_DISTANCE);
  *distance = tmp;
}

/****************************************************************************
 * Name: vl53l1x_getoffset
 *
 * Description:
 *  This function returns the programmed offset correction value in mm.
 *
 ****************************************************************************/

static void vl53l1x_getoffset(FAR struct vl53l1x_dev_s *priv,
                              FAR int16_t *offset)
{
  uint16_t temp;

  temp = vl53l1x_getreg16(priv, RANGE_OFFSET_MM);

  temp = temp << 3;
  temp = temp >> 5;

  *offset = (int16_t) (temp);
}

/****************************************************************************
 * Name: vl53l1x_tempupdate
 *
 * Description:
 *  This function performs the temperature calibration. It is recommended to
 *  call this function any time the temperature might have changed by more
 *  than 8 deg C without sensor ranging activity for an extended period.
 *
 ****************************************************************************/

static void vl53l1x_tempupdate(FAR struct vl53l1x_dev_s *priv)
{
  uint8_t tmp = 0;

  vl53l1x_putreg8(priv, TIMEOUT_MACROP_LOOP_BOUND, 0x81);
  vl53l1x_putreg8(priv, 0x0b, 0x92);

  vl53l1x_startranging(priv);

  while (tmp == 0)
    {
      vl53l1x_dataready(priv, &tmp);
    }

  vl53l1x_clearint(priv);
  vl53l1x_stopranging(priv);

  vl53l1x_putreg8(priv, TIMEOUT_MACROP_LOOP_BOUND, 0x09);
  vl53l1x_putreg8(priv, 0x0b, 0);
}

/****************************************************************************
 * Name: vl53l1x_calibrateoffset
 *
 * Description:
 *  The function returns the offset value found and programs the offset
 *  compensation into the device.
 *
 ****************************************************************************/

static void vl53l1x_calibrateoffset(FAR struct vl53l1x_dev_s *priv,
                                    uint16_t target_distance,
                                    FAR int16_t *offset)
{
  uint8_t i = 0;
  uint8_t tmp;
  int16_t average_distance = 0;
  uint16_t distance;

  vl53l1x_putreg16(priv, RANGE_OFFSET_MM, 0x0);
  vl53l1x_putreg16(priv, INNER_OFFSET_MM, 0x0);
  vl53l1x_putreg16(priv, OUTER_OFFSET_MM, 0x0);

  vl53l1x_startranging(priv);

  for (i = 0; i < 50; i++)
    {
      while (tmp == 0)
        {
          vl53l1x_dataready(priv, &tmp);
        }

      vl53l1x_getdistance(priv, &distance);
      vl53l1x_clearint(priv);

      average_distance = average_distance + distance;
    }

  vl53l1x_stopranging(priv);

  average_distance = average_distance / 50;
  *offset = target_distance - average_distance;

  vl53l1x_putreg16(priv, RANGE_OFFSET_MM, *offset *4);
}

/****************************************************************************
 * Name: vl53l1x_getreg8
 *
 * Description:
 *   Read from an 8-bit vl53l1x register
 *
 ****************************************************************************/

static uint8_t vl53l1x_getreg8(FAR struct vl53l1x_dev_s *priv,
                               uint16_t regaddr)
{
  struct i2c_config_s config;

  uint8_t regval = 0;
  uint16_t ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, (uint8_t *)&regaddr, 2);
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

static uint16_t vl53l1x_getreg16(FAR struct vl53l1x_dev_s *priv,
                                 uint16_t regaddr)
{
  struct i2c_config_s config;

  uint16_t msb;
  uint16_t lsb;
  uint16_t regval = 0;
  uint8_t reg_addr_aux[2];
  uint8_t ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  /* Split the I2C direction */

  reg_addr_aux[0] = (regaddr >> 8) & 0xff;
  reg_addr_aux[1] = regaddr;

  /* Register to read */

  sninfo("Reg %02x %\n", reg_addr_aux[0], reg_addr_aux[1]);
  ret = i2c_write(priv->i2c, &config, (uint8_t *)&reg_addr_aux, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read register */

  ret = i2c_read(priv->i2c, &config, (uint8_t *) & regval, 2);
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
 * Name: vl53l1x_getreg32
 *
 * Description:
 *   Read 4 8-bit from a vl53l1x register
 *
 ****************************************************************************/

static uint32_t vl53l1x_getreg32(FAR struct vl53l1x_dev_s *priv,
                                 uint16_t regaddr)
{
  struct i2c_config_s config;

  uint16_t byte[4];
  uint32_t regval = 0;
  uint8_t ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  /* Register to read */

  ret = i2c_write(priv->i2c, &config, (FAR uint8_t *)&regaddr, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read register */

  ret = i2c_read(priv->i2c, &config, (FAR uint8_t *) & regval, 4);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* The bytes are in the opposite order of importance */

  byte[0] = (regval & 0xff);
  byte[1] = (regval & 0xff00) >> 8;
  byte[2] = (regval & 0xffff00) >> 16;
  byte[3] = (regval & 0xffffff00) >> 24;

  regval = (byte[0] << 24) | (byte[1] << 16) | (byte[2] << 8) | byte[3];

  return regval;
}

/****************************************************************************
 * Name: vl53l1x_putreg8
 *
 * Description:
 *   Write to an 8-bit vl53l1x register
 *
 ****************************************************************************/

static void vl53l1x_putreg8(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                            uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t data[3];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  data[0] = (regaddr >> 8) & 0xff;
  data[1] = regaddr;
  data[2] = regval & 0xff;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (uint8_t *) & data, 3);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
    }
}

/****************************************************************************
 * Name: vl53l1x_putreg16
 *
 * Description:
 *   Write to an 16-bit vl53l1x register
 *
 ****************************************************************************/

static void vl53l1x_putreg16(FAR struct vl53l1x_dev_s *priv,
                             uint16_t regaddr, uint16_t regval)
{
  struct i2c_config_s config;
  uint8_t data[4];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  data[0] = (regaddr >> 8) & 0xff;
  data[1] = regaddr;
  data[2] = (regval >> 8) & 0xff;
  data[3] = regval & 0xff;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (uint8_t *) & data, 4);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
    }
}

/****************************************************************************
 * Name: vl53l1x_putreg32
 *
 * Description:
 *   Write to an 32-bit vl53l1x register
 *
 ****************************************************************************/

static void vl53l1x_putreg32(FAR struct vl53l1x_dev_s *priv,
                             uint16_t regaddr, uint32_t regval)
{
  struct i2c_config_s config;
  uint8_t data[7];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  data[0] = (regaddr >> 8) & 0xff;
  data[1] = regaddr;
  data[2] = (regval >> 24) & 0xff;
  data[4] = (regval >> 16) & 0xff;
  data[5] = (regval >> 8) & 0xff;
  data[6] = regval & 0xff;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (uint8_t *) & data, 7);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
    }
}

/****************************************************************************
 * Name: vl53l1x_read
 ****************************************************************************/

static void vl53l1x_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct vl53l1x_dev_s *priv = inode->i_private;
  FAR uint16_t *aux = (FAR uint16_t *) buffer;

  vl53l1x_startranging(priv);
  vl53l1x_getdistance(priv, aux);
  vl53l1x_stopranging(priv);
}

/****************************************************************************
 * Name: vl53l1x_write
 ****************************************************************************/

static ssize_t vl53l1x_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: vl53l1x_ioctl
 ****************************************************************************/

static void vl53l1x_ioctl(FAR struct file *filep, int cmd, uint16_t arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct vl53l1x_dev_s *priv = inode->i_private;

  switch (cmd)
    {
    case SNIOC_DISTANCESHORT:
      {
        sninfo("Set distance up to 1.3M\n");
        vl53l1x_setdistancemode(priv, 1);
      }
      break;

    case SNIOC_DISTANCELONG:
      {
        sninfo("Set distance up to 4M\n");
        vl53l1x_setdistancemode(priv, 2);
      }
      break;

    case SNIOC_CALIBRATE:
      {
        sninfo("Calibrating distance\n");
        int16_t offset;
        vl53l1x_getoffset(priv, (int16_t *)&offset);
        vl53l1x_calibrateoffset(priv, arg, (int16_t *)&offset);
      }
      break;

    case SNIOC_TEMPUPDATE:
      {
        sninfo("Recalculating due to temperature change\n");
        vl53l1x_tempupdate(priv);
      }
      break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vl53l1x_register
 *
 * Description:
 *   Register the vl53l1x character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/tof"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             vl53l1x TOF
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int vl53l1x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct vl53l1x_dev_s *priv;
  int ret = 0;
  uint16_t id;

  /* Initialize the vl53l1x device structure */

  priv = (FAR struct vl53l1x_dev_s *)kmm_malloc(
    sizeof(struct vl53l1x_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = VL53L1X_ADDR;
  priv->freq = VL53L1X_FREQ;

  vl53l1x_sensorinit(priv);

  vl53l1x_getid(priv, &id);
  if (id != 0xeacc)
    {
      snerr("ERROR: Failed sensor ID %04x\n", id);
      kmm_free(priv);
      return 0;
    }

  register_driver(devpath, &g_vl53l1xfops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return 0;
    }

  sninfo("vl53l1x driver loaded successfully!\n");
  return 1;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_VL53L1X */

/****************************************************************************
 * boards/arm/cxd56xx/drivers/sensors/bmp280_scu.c
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
#include <stdio.h>
#include <fixedmath.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmp280.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMP280_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI280_SCU_DECI_PRESS
#  define PRESS_SEQ_TYPE SEQ_TYPE_DECI
#else
#  define PRESS_SEQ_TYPE SEQ_TYPE_NORMAL
#endif
#ifdef CONFIG_SENSORS_BMI280_SCU_DECI_TEMP
#  define TEMP_SEQ_TYPE SEQ_TYPE_DECI
#else
#  define TEMP_SEQ_TYPE SEQ_TYPE_NORMAL
#endif

#define BMP280_ADDR         0x76
#define BMP280_FREQ         400000
#define DEVID               0x58

#define BMP280_DIG_T1_LSB   0x88
#define BMP280_DIG_T1_MSB   0x89
#define BMP280_DIG_T2_LSB   0x8a
#define BMP280_DIG_T2_MSB   0x8b
#define BMP280_DIG_T3_LSB   0x8c
#define BMP280_DIG_T3_MSB   0x8d
#define BMP280_DIG_P1_LSB   0x8e
#define BMP280_DIG_P1_MSB   0x8f
#define BMP280_DIG_P2_LSB   0x90
#define BMP280_DIG_P2_MSB   0x91
#define BMP280_DIG_P3_LSB   0x92
#define BMP280_DIG_P3_MSB   0x93
#define BMP280_DIG_P4_LSB   0x94
#define BMP280_DIG_P4_MSB   0x95
#define BMP280_DIG_P5_LSB   0x96
#define BMP280_DIG_P5_MSB   0x97
#define BMP280_DIG_P6_LSB   0x98
#define BMP280_DIG_P6_MSB   0x99
#define BMP280_DIG_P7_LSB   0x9a
#define BMP280_DIG_P7_MSB   0x9b
#define BMP280_DIG_P8_LSB   0x9c
#define BMP280_DIG_P8_MSB   0x9d
#define BMP280_DIG_P9_LSB   0x9e
#define BMP280_DIG_P9_MSB   0x9f

#define BMP280_DEVID        0xd0
#define BMP280_SOFT_RESET   0xe0
#define BMP280_STAT         0xf3
#define BMP280_CTRL_MEAS    0xf4
#define BMP280_CONFIG       0xf5
#define BMP280_PRESS_MSB    0xf7
#define BMP280_PRESS_LSB    0xf8
#define BMP280_PRESS_XLSB   0xf9
#define BMP280_TEMP_MSB     0xfa
#define BMP280_TEMP_LSB     0xfb
#define BMP280_TEMP_XLSB    0xfc

/* power mode */

#define BMP280_SLEEP_MODE   0x00
#define BMP280_FORCED_MODE  0x01
#define BMP280_NORMAL_MODE  0x03

/* oversampling */

#define BMP280_OVERSAMP_SKIPPED 0x00
#define BMP280_OVERSAMP_1X      0x01
#define BMP280_OVERSAMP_2X      0x02
#define BMP280_OVERSAMP_4X      0x03
#define BMP280_OVERSAMP_8X      0x04
#define BMP280_OVERSAMP_16X     0x05

/* BMP280 have press and temp, pressure respectively in 24 bits.
 */

#define BMP280PRESS_BYTESPERSAMPLE  3
#define BMP280PRESS_ELEMENTSIZE     3
#define BMP280TEMP_BYTESPERSAMPLE   3
#define BMP280TEMP_ELEMENTSIZE      3

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct bmp280_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* BMP280 I2C address */
  int freq;                     /* BMP280 Frequency <= 3.4MHz */
  int port;                     /* I2C port */
  struct seq_s *seq;            /* Sequencer */
  int id;                       /* Sequencer id */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t bmp280_getreg8(FAR struct bmp280_dev_s *priv,
                              uint8_t regaddr);
static void bmp280_putreg8(FAR struct bmp280_dev_s *priv,
                           uint8_t regaddr,
                           uint8_t regval);

/* Character driver methods */

static int bmp280_open_press(FAR struct file *filep);
static int bmp280_open_temp(FAR struct file *filep);
static int bmp280_close_press(FAR struct file *filep);
static int bmp280_close_temp(FAR struct file *filep);
static ssize_t bmp280_read_press(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t bmp280_read_temp(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t bmp280_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int bmp280_ioctl_press(FAR struct file *filep, int cmd,
                              unsigned long arg);
static int bmp280_ioctl_temp(FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_bmp280pressfops =
{
  bmp280_open_press,            /* open */
  bmp280_close_press,           /* close */
  bmp280_read_press,            /* read */
  bmp280_write,                 /* write */
  0,                            /* seek */
  bmp280_ioctl_press,           /* ioctl */
  0                             /* unlink */
};

static const struct file_operations g_bmp280tempfops =
{
  bmp280_open_temp,             /* open */
  bmp280_close_temp,            /* close */
  bmp280_read_temp,             /* read */
  bmp280_write,                 /* write */
  0,                            /* seek */
  bmp280_ioctl_temp,            /* ioctl */
  0                             /* unlink */
};

/* SCU instructions for pick pressure sensing data. */

static const uint16_t g_bmp280pressinst[] =
{
  SCU_INST_SEND(BMP280_PRESS_MSB),
  SCU_INST_RECV(BMP280PRESS_BYTESPERSAMPLE) | SCU_INST_LAST,
};

/* SCU instructions for pick temperature sensing data. */

static const uint16_t g_bmp280tempinst[] =
{
  SCU_INST_SEND(BMP280_TEMP_MSB),
  SCU_INST_RECV(BMP280TEMP_BYTESPERSAMPLE) | SCU_INST_LAST,
};

/* driver reference counter */

static int g_refcnt_press = 0;
static int g_refcnt_temp = 0;

/* senser calibration parameters */

static struct bmp280_press_adj_s  g_press_adj;
static struct bmp280_temp_adj_s   g_temp_adj;

/* Sequencer instance */

static FAR struct seq_s *g_seq_press = NULL;
static FAR struct seq_s *g_seq_temp = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmp280_getreg8
 *
 * Description:
 *   Read from an 8-bit BMP280 register
 *
 ****************************************************************************/

static uint8_t bmp280_getreg8(FAR struct bmp280_dev_s *priv, uint8_t regaddr)
{
  uint8_t regval = 0;
  uint16_t inst[2];

  /* Send register to read and get the next byte */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_RECV(1) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 2, &regval, 1);

  return regval;
}

/****************************************************************************
 * Name: bmp280_putreg8
 *
 * Description:
 *   Write to an 8-bit BMP280 register
 *
 ****************************************************************************/

static void bmp280_putreg8(FAR struct bmp280_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  uint16_t inst[2];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_SEND(regval) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 2, NULL, 0);
}

/****************************************************************************
 * Name: bmp280_checkid
 *
 * Description:
 *   Read and verify the BMP280 chip ID
 *
 ****************************************************************************/

static int bmp280_checkid(FAR struct bmp280_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID */

  devid = bmp280_getreg8(priv, BMP280_DEVID);
  sninfo("devid: 0x%02x\n", devid);

  if (devid != (uint16_t)DEVID)
    {
      /* ID is not Correct */

      snerr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bmp280_get_calib_param
 *
 * Description:
 *   Read Calibration Coefficient Data
 *
 ****************************************************************************/

static int bmp280_get_calib_param_press(FAR struct bmp280_dev_s *priv)
{
  /* Read calibration values */

  g_press_adj.dig_p1 =
        ((uint16_t)bmp280_getreg8(priv, BMP280_DIG_P1_MSB) << 8) |
         bmp280_getreg8(priv, BMP280_DIG_P1_LSB);
  g_press_adj.dig_p2 =
        ((int16_t)bmp280_getreg8(priv, BMP280_DIG_P2_MSB) << 8) |
        bmp280_getreg8(priv, BMP280_DIG_P2_LSB);
  g_press_adj.dig_p3 =
        ((int16_t)bmp280_getreg8(priv, BMP280_DIG_P3_MSB) << 8) |
        bmp280_getreg8(priv, BMP280_DIG_P3_LSB);
  g_press_adj.dig_p4 =
        ((int16_t)bmp280_getreg8(priv, BMP280_DIG_P4_MSB) << 8) |
        bmp280_getreg8(priv, BMP280_DIG_P4_LSB);
  g_press_adj.dig_p5 =
        ((int16_t)bmp280_getreg8(priv, BMP280_DIG_P5_MSB) << 8) |
        bmp280_getreg8(priv, BMP280_DIG_P5_LSB);
  g_press_adj.dig_p6 =
        ((int16_t)bmp280_getreg8(priv, BMP280_DIG_P6_MSB) << 8) |
        bmp280_getreg8(priv, BMP280_DIG_P6_LSB);
  g_press_adj.dig_p7 =
        ((int16_t)bmp280_getreg8(priv, BMP280_DIG_P7_MSB) << 8) |
        bmp280_getreg8(priv, BMP280_DIG_P7_LSB);
  g_press_adj.dig_p8 =
        ((int16_t)bmp280_getreg8(priv, BMP280_DIG_P8_MSB) << 8) |
        bmp280_getreg8(priv, BMP280_DIG_P8_LSB);
  g_press_adj.dig_p9 =
        ((int16_t)bmp280_getreg8(priv, BMP280_DIG_P9_MSB) << 8) |
        bmp280_getreg8(priv, BMP280_DIG_P9_LSB);

  return OK;
}

/****************************************************************************
 * Name: bmp280_get_calib_param_temp
 *
 * Description:
 *   Read Calibration Coefficient Data
 *
 ****************************************************************************/

static int bmp280_get_calib_param_temp(FAR struct bmp280_dev_s *priv)
{
  /* Read calibration values */

  g_temp_adj.dig_t1 =
        ((uint16_t)bmp280_getreg8(priv, BMP280_DIG_T1_MSB) << 8) |
        bmp280_getreg8(priv, BMP280_DIG_T1_LSB);
  g_temp_adj.dig_t2 =
        ((int16_t)bmp280_getreg8(priv, BMP280_DIG_T2_MSB) << 8) |
        bmp280_getreg8(priv, BMP280_DIG_T2_LSB);
  g_temp_adj.dig_t3 =
        ((int16_t)bmp280_getreg8(priv, BMP280_DIG_T3_MSB) << 8) |
        bmp280_getreg8(priv, BMP280_DIG_T3_LSB);

  return OK;
}

/****************************************************************************
 * Name: bmp280_set_power_mode
 *
 * Description:
 *   set power mode
 *
 ****************************************************************************/

static void bmp280_set_power_mode(FAR struct bmp280_dev_s *priv,
                                  uint8_t value)
{
  uint8_t v_data_u8 = 0;

  v_data_u8 = bmp280_getreg8(priv, BMP280_CTRL_MEAS);
  v_data_u8 = ((v_data_u8 & ~0x03) | value);
  bmp280_putreg8(priv, BMP280_CTRL_MEAS, v_data_u8);
}

/****************************************************************************
 * Name: bmp280_set_oversamp_press
 *
 * Description:
 *   set oversampling rate
 *
 ****************************************************************************/

static void bmp280_set_oversamp_press(FAR struct bmp280_dev_s *priv,
                                      uint8_t value)
{
  uint8_t v_data_u8 = 0;

  v_data_u8 = bmp280_getreg8(priv, BMP280_CTRL_MEAS);
  v_data_u8 = ((v_data_u8 & ~(0x07 << 2)) | ((value << 2) & (0x07 << 2)));
  bmp280_putreg8(priv, BMP280_CTRL_MEAS, v_data_u8);
}

/****************************************************************************
 * Name: bmp280_set_oversamp_temp
 *
 * Description:
 *   set oversampling rate
 *
 ****************************************************************************/

static void bmp280_set_oversamp_temp(FAR struct bmp280_dev_s *priv,
                                     uint8_t value)
{
  uint8_t v_data_u8 = 0;

  v_data_u8 = bmp280_getreg8(priv, BMP280_CTRL_MEAS);
  v_data_u8 = ((v_data_u8 & ~(0x07 << 5)) | ((value << 5) & (0x07 << 5)));
  bmp280_putreg8(priv, BMP280_CTRL_MEAS, v_data_u8);
}

/****************************************************************************
 * Name: bmp280_set_standby
 *
 * Description:
 *   set standby duration
 *
 ****************************************************************************/

static int bmp280_set_standby(FAR struct bmp280_dev_s *priv, uint8_t value)
{
  uint8_t v_data_u8;
  uint8_t v_sb_u8;

  /* Set the standby duration value */

  v_data_u8 = bmp280_getreg8(priv, BMP280_CONFIG);
  v_data_u8 = (v_data_u8 & ~(0x07 << 5)) | (value << 5);
  bmp280_putreg8(priv, BMP280_CONFIG, v_data_u8);

  /* Check the standby duration value */

  v_data_u8 = bmp280_getreg8(priv, BMP280_CONFIG);
  v_sb_u8 = (v_data_u8 >> 5) & 0x07;

  if (v_sb_u8 != value)
    {
      snerr("Failed to set value for standby time.");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: bmp280_initialize
 *
 * Description:
 *   Initialize BMP280 pressure sensor
 *
 ****************************************************************************/

static int bmp280_initialize(FAR struct bmp280_dev_s *priv)
{
  int ret;

  ret = bmp280_set_standby(priv, BMP280_STANDBY_05_MS);

  if (ret != OK)
    {
      snerr("Failed to set value for standby time.");
      return ERROR;
    }

  return OK;
}

static int bmp280_seqinit_press(FAR struct bmp280_dev_s *priv)
{
  DEBUGASSERT(!g_seq_press);

  /* Open sequencer */

  g_seq_press = seq_open(PRESS_SEQ_TYPE, SCU_BUS_I2C0);
  if (!g_seq_press)
    {
      return -ENOENT;
    }

  priv->seq = g_seq_press;

  seq_setaddress(priv->seq, priv->addr);

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq, g_bmp280pressinst,
                     itemsof(g_bmp280pressinst));
  seq_setsample(priv->seq, BMP280PRESS_BYTESPERSAMPLE, 0,
                BMP280PRESS_ELEMENTSIZE, false);

  return OK;
}

static int bmp280_seqinit_temp(FAR struct bmp280_dev_s *priv)
{
  DEBUGASSERT(!g_seq_temp);

  /* Open sequencer */

  g_seq_temp = seq_open(TEMP_SEQ_TYPE, SCU_BUS_I2C0);
  if (!g_seq_temp)
    {
      return -ENOENT;
    }

  priv->seq = g_seq_temp;

  seq_setaddress(priv->seq, priv->addr);

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq, g_bmp280tempinst,
                     itemsof(g_bmp280tempinst));
  seq_setsample(priv->seq, BMP280TEMP_BYTESPERSAMPLE, 0,
                BMP280TEMP_ELEMENTSIZE, false);

  return OK;
}

/****************************************************************************
 * Name: bmp280_open_press
 *
 * Description:
 *   This function is called whenever the BMP280 device is opened.
 *
 ****************************************************************************/

static int bmp280_open_press(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmp280_dev_s *priv  = inode->i_private;
  int ret;

  /* first pressure device initialize */

  if (g_refcnt_press == 0)
    {
      /* Open and set sequencer */

      ret = bmp280_seqinit_press(priv);
      if (ret)
        {
          return ret;
        }

      bmp280_get_calib_param_press(priv);
      bmp280_set_oversamp_press(priv, BMP280_OVERSAMP_4X);

      if (g_refcnt_temp == 0)
        {
          bmp280_set_power_mode(priv, BMP280_NORMAL_MODE);
          bmp280_initialize(priv);
        }
    }
  else
    {
      /* Set existing sequencer */

      priv->seq = g_seq_press;
    }

  g_refcnt_press++;

  return OK;
}

/****************************************************************************
 * Name: bmp280_open_temp
 *
 * Description:
 *   This function is called whenever the BMP280 device is opened.
 *
 ****************************************************************************/

static int bmp280_open_temp(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmp280_dev_s *priv  = inode->i_private;
  int ret;

  /* first temperature device initialize */

  if (g_refcnt_temp == 0)
    {
      /* Open and set sequencer */

      ret = bmp280_seqinit_temp(priv);
      if (ret)
        {
          return ret;
        }

      bmp280_get_calib_param_temp(priv);
      bmp280_set_oversamp_temp(priv, BMP280_OVERSAMP_1X);

      if (g_refcnt_press == 0)
        {
          bmp280_set_power_mode(priv, BMP280_NORMAL_MODE);
          bmp280_initialize(priv);
        }
    }
  else
    {
      /* Set existing sequencer */

      priv->seq = g_seq_temp;
    }

  g_refcnt_temp++;

  return OK;
}

/****************************************************************************
 * Name: bmp280_close_press
 *
 * Description:
 *   This routine is called when the BMP280 device is closed.
 *
 ****************************************************************************/

static int bmp280_close_press(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmp280_dev_s *priv  = inode->i_private;

  if (g_refcnt_press <= 1)
    {
      bmp280_set_oversamp_press(priv, BMP280_OVERSAMP_SKIPPED);

      /* if close last BMP280 device, enter sleep mode */

      if (g_refcnt_temp == 0)
        {
          bmp280_set_power_mode(priv, BMP280_SLEEP_MODE);
        }

      seq_close(g_seq_press);
      g_seq_press = NULL;
    }
  else
    {
      seq_ioctl(priv->seq, priv->id, SCUIOC_FREEFIFO, 0);
    }

  g_refcnt_press--;

  return OK;
}

/****************************************************************************
 * Name: bmp280_close_temp
 *
 * Description:
 *   This routine is called when the BMP280 device is closed.
 *
 ****************************************************************************/

static int bmp280_close_temp(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmp280_dev_s *priv  = inode->i_private;

  if (g_refcnt_temp <= 1)
    {
      bmp280_set_oversamp_temp(priv, BMP280_OVERSAMP_SKIPPED);

      /* if close last BMP280 device, enter sleep mode */

      if (g_refcnt_press == 0)
        {
          bmp280_set_power_mode(priv, BMP280_SLEEP_MODE);
        }

      seq_close(g_seq_temp);
      g_seq_temp = NULL;
    }
  else
    {
      seq_ioctl(priv->seq, priv->id, SCUIOC_FREEFIFO, 0);
    }

  g_refcnt_temp--;

  return OK;
}

/****************************************************************************
 * Name: bmp280_read_press
 ****************************************************************************/

static ssize_t bmp280_read_press(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmp280_dev_s *priv  = inode->i_private;

  buflen = buflen / BMP280PRESS_BYTESPERSAMPLE * BMP280PRESS_BYTESPERSAMPLE;
  buflen = seq_read(priv->seq, priv->id, buffer, buflen);

  return buflen;
}

/****************************************************************************
 * Name: bmp280_read_temp
 ****************************************************************************/

static ssize_t bmp280_read_temp(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmp280_dev_s *priv  = inode->i_private;

  buflen = buflen / BMP280TEMP_BYTESPERSAMPLE * BMP280TEMP_BYTESPERSAMPLE;
  buflen = seq_read(priv->seq, priv->id, buffer, buflen);

  return buflen;
}

/****************************************************************************
 * Name: bm p280_write
 ****************************************************************************/

static ssize_t bmp280_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bmp280_ioctrl_press
 ****************************************************************************/

static int bmp280_ioctl_press(FAR struct file *filep, int cmd,
                              unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bmp280_dev_s *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      /* Get sensitivity adjustment value
       * Arg: Pointer to struct bmp280_press_adj_s
       */

      case SNIOC_GETADJ:
        {
          struct bmp280_press_adj_s *user = (struct bmp280_press_adj_s *)
                                            (uintptr_t)arg;

          user->dig_p1 = g_press_adj.dig_p1;
          user->dig_p2 = g_press_adj.dig_p2;
          user->dig_p3 = g_press_adj.dig_p3;
          user->dig_p4 = g_press_adj.dig_p4;
          user->dig_p5 = g_press_adj.dig_p5;
          user->dig_p6 = g_press_adj.dig_p6;
          user->dig_p7 = g_press_adj.dig_p7;
          user->dig_p8 = g_press_adj.dig_p8;
          user->dig_p9 = g_press_adj.dig_p9;
        }
        break;

      case SNIOC_SETSTB:
        {
          ret = bmp280_set_standby(priv, arg);
        }
        break;

      default:
        {
          if (_SCUIOCVALID(cmd))
            {
              /* Redirect SCU commands */

              ret = seq_ioctl(priv->seq, priv->id, cmd, arg);
            }
          else
            {
              snerr("Unrecognized cmd: %d\n", cmd);
              ret = - ENOTTY;
            }
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: bmp280_ioctrl_temp
 ****************************************************************************/

static int bmp280_ioctl_temp(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bmp280_dev_s *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      /* Get sensitivity adjustment value
       * Arg: Pointer to struct bmp280_temp_adj_s
       */

      case SNIOC_GETADJ:
        {
          struct bmp280_temp_adj_s *user = (struct bmp280_temp_adj_s *)
                                           (uintptr_t)arg;

          user->dig_t1 = g_temp_adj.dig_t1;
          user->dig_t2 = g_temp_adj.dig_t2;
          user->dig_t3 = g_temp_adj.dig_t3;
        }
        break;

      default:
        {
          if (_SCUIOCVALID(cmd))
            {
              /* Redirect SCU commands */

              ret = seq_ioctl(priv->seq, priv->id, cmd, arg);
            }
          else
            {
              snerr("Unrecognized cmd: %d\n", cmd);
              ret = - ENOTTY;
            }
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmp280_init
 *
 * Description:
 *   Initialize the I2C interface to use to communicate with BMP280
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BMP280
 *   port    - I2C port number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp280_init(FAR struct i2c_master_s *i2c, int port)
{
  struct bmp280_dev_s tmp;
  struct bmp280_dev_s *priv = &tmp;
  int ret;

  /* Setup temporary device structure for initialization */

  priv->i2c = i2c;
  priv->addr = BMP280_ADDR;
  priv->freq = BMP280_FREQ;
  priv->port = port;

  /* Check Device ID */

  ret = bmp280_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: bmp280press_register
 *
 * Description:
 *   Register the BMP280 pressure sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The base path to the driver to register. E.g., "/dev/press0"
 *   dev     - An instance of the I2C interface to use to communicate with
 *             BMP280
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp280press_register(FAR const char *devpath, int minor,
                    FAR struct i2c_master_s *i2c, int port)
{
  FAR struct bmp280_dev_s *priv;
  char path[12];
  int ret;

  priv = (FAR struct bmp280_dev_s *)kmm_malloc(sizeof(struct bmp280_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->seq = NULL;
  priv->id = minor;
  priv->addr = BMP280_ADDR;
  priv->freq = BMP280_FREQ;
  priv->port = port;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_bmp280pressfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BMP280 pressure driver loaded successfully!\n");
  return ret;
}

/****************************************************************************
 * Name: bmp280temp_register
 *
 * Description:
 *   Register the BMP280 temperature sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The base path to the driver to register. E.g., "/dev/temp"
 *   dev     - An instance of the I2C interface to use to communicate with
 *             BMP280
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp280temp_register(FAR const char *devpath, int minor,
                    FAR struct i2c_master_s *i2c, int port)
{
  FAR struct bmp280_dev_s *priv;
  char path[12];
  int ret;

  priv = (FAR struct bmp280_dev_s *)kmm_malloc(sizeof(struct bmp280_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->seq = NULL;
  priv->id = minor;
  priv->addr = BMP280_ADDR;
  priv->freq = BMP280_FREQ;
  priv->port = port;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_bmp280tempfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BMP280 temperature driver loaded successfully!\n");
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BMP280_SCU */

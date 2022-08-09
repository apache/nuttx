/****************************************************************************
 * boards/arm/cxd56xx/drivers/sensors/lt1pa01_scu.c
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
#include <arch/types.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lt1pa01.h>
#include <nuttx/irq.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LT1PA01_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LT1PA01_ADDR                  0x44    /* I2C Slave Address */
#define LT1PA01_DEVICEID              0xC8    /* Device ID */
#define LT1PA01_ALS_BYTESPERSAMPLE    2
#define LT1PA01_PROX_BYTESPERSAMPLE   1
#define LT1PA01_ELEMENTSIZE           0

/* LT1PA01 Registers */

#define LT1PA01_ID                    0x00
#define LT1PA01_CONFIG0               0x01
#define LT1PA01_CONFIG1               0x02
#define LT1PA01_CONFIG2               0x03
#define LT1PA01_INTCONFIG             0x04
#define LT1PA01_PROX_INT_TL           0x05
#define LT1PA01_PROX_INT_TH           0x06
#define LT1PA01_ALS_INT_TL            0x07
#define LT1PA01_ALS_INT_TLH           0x08
#define LT1PA01_ALS_INT_TH            0x09
#define LT1PA01_PROX_DATA             0x0A
#define LT1PA01_ALS_DATA_HB           0x0B
#define LT1PA01_ALS_DATA_LB           0x0C
#define LT1PA01_PROX_AMBIR            0x0D
#define LT1PA01_CONFIG3               0x0E

/* Register ID */

#define LT1PA01_ID_MASK               0xF8

/* Register CONFIG0 */

#define LT1PA01_PROX_DIS              (0 << 5)
#define LT1PA01_PROX_EN               (1 << 5)
#define LT1PA01_PROX_SLP_800MS        (0 << 2)
#define LT1PA01_PROX_SLP_200MS        (1 << 2)
#define LT1PA01_PROX_SLP_100MS        (2 << 2)
#define LT1PA01_PROX_IRDR_DRV0        (0 << 0)  /* 3.6mA */
#define LT1PA01_PROX_IRDR_DRV1        (1 << 0)  /* 7.2mA */
#define LT1PA01_PROX_IRDR_DRV2        (2 << 0)  /* 10.8mA */
#define LT1PA01_PROX_IRDR_DRV3        (3 << 0)  /* 14.4mA */

/* Register CONFIG1 */

#define LT1PA01_INT_ALG_COMPARATOR    (0 << 7)
#define LT1PA01_INT_ALG_HYSTERESIS    (1 << 7)
#define LT1PA01_ALS_DIS               (0 << 2)
#define LT1PA01_ALS_EN                (1 << 2)
#define LT1PA01_ALS_ALS_RANGE0        (0 << 0)  /* 62.5 Lux */
#define LT1PA01_ALS_ALS_RANGE1        (1 << 0)  /* 125 Lux */
#define LT1PA01_ALS_ALS_RANGE2        (2 << 0)  /* 1000 Lux */
#define LT1PA01_ALS_ALS_RANGE3        (3 << 0)  /* 2000 Lux */
#define LT1PA01_ALS_ALS_RANGE_MASK    (3 << 0)

/* Register INTCONFIG */

#define LT1PA01_PORX_INT_FLG          (1 << 7)
#define LT1PA01_PORX_INT_PRST1        (0 << 5)
#define LT1PA01_PORX_INT_PRST2        (1 << 5)
#define LT1PA01_PORX_INT_PRST4        (2 << 5)
#define LT1PA01_PORX_INT_PRST8        (3 << 5)
#define LT1PA01_PORX_INT_PRST_MASK    (3 << 5)
#define LT1PA01_PWR_FAIL              (1 << 4)
#define LT1PA01_ALS_INT_FLG           (1 << 3)
#define LT1PA01_ALS_INT_PRST1         (0 << 1)
#define LT1PA01_ALS_INT_PRST2         (1 << 1)
#define LT1PA01_ALS_INT_PRST4         (2 << 1)
#define LT1PA01_ALS_INT_PRST8         (3 << 1)
#define LT1PA01_INT_CFG_ALS_AND_PROX  (1 << 0)
#define LT1PA01_INT_CFG_ALS_OR_PROX   (0 << 0)

/* Register INTCONFIG */

#define LT1PA01_SW_RESET              0x38

/* PROX INT threshold default value */

#define LT1PA01_PROX_INT_TL_DEFAULT   0x03
#define LT1PA01_PROX_INT_TH_DEFAULT   0x03

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/**
 * @brief Structure for lt1pa01 device
 */

struct lt1pa01_dev_s
{
  struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;             /* I2C address */
  int port;                 /* I2C port */
  struct seq_s *seq;        /* Sequencer instance */
  int minor;                /* Minor device number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int lt1pa01_open_als(struct file *filep);
static int lt1pa01_open_prox(struct file *filep);
static int lt1pa01_close_als(struct file *filep);
static int lt1pa01_close_prox(struct file *filep);
static ssize_t lt1pa01_read_als(struct file *filep, char *buffer,
                                size_t buflen);
static ssize_t lt1pa01_read_prox(struct file *filep, char *buffer,
                                 size_t buflen);
static ssize_t lt1pa01_write(struct file *filep, const char *buffer,
                             size_t buflen);
static int lt1pa01_ioctl_als(struct file *filep, int cmd,
                             unsigned long arg);
static int lt1pa01_ioctl_prox(struct file *filep, int cmd,
                              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Ambient light sensor */

static const struct file_operations g_lt1pa01alsfops =
{
  lt1pa01_open_als,            /* open */
  lt1pa01_close_als,           /* close */
  lt1pa01_read_als,            /* read */
  lt1pa01_write,               /* write */
  NULL,                        /* seek */
  lt1pa01_ioctl_als,           /* ioctl */
  NULL                         /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                       /* unlink */
#endif
};

/* Proximity sensor */

static const struct file_operations g_lt1pa01proxfops =
{
  lt1pa01_open_prox,           /* open */
  lt1pa01_close_prox,          /* close */
  lt1pa01_read_prox,           /* read */
  lt1pa01_write,               /* write */
  NULL,                        /* seek */
  lt1pa01_ioctl_prox,          /* ioctl */
  NULL                         /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                       /* unlink */
#endif
};

/* SCU instructions for pick ambient light sensing data. */

static const uint16_t g_lt1pa01alsinst[] =
{
  SCU_INST_SEND(LT1PA01_ALS_DATA_HB),
  SCU_INST_RECV(LT1PA01_ALS_BYTESPERSAMPLE) | SCU_INST_LAST,
};

#ifndef CONFIG_LT1PA01_PROXIMITY_INTERRUPT
/* SCU instructions for pick proximity sensing data. */

static const uint16_t g_lt1pa01proxinst[] =
{
  SCU_INST_SEND(LT1PA01_PROX_DATA),
  SCU_INST_RECV(LT1PA01_PROX_BYTESPERSAMPLE) | SCU_INST_LAST,
};
#endif

/* Reference count */

static int g_als_refcnt = 0;
#ifndef CONFIG_LT1PA01_PROXIMITY_INTERRUPT
static int g_prox_refcnt = 0;
#endif

/* Sequencer instance */

static struct seq_s *g_als_seq = NULL;
#ifndef CONFIG_LT1PA01_PROXIMITY_INTERRUPT
static struct seq_s *g_prox_seq = NULL;
#endif

#ifdef CONFIG_LT1PA01_PROXIMITY_INTERRUPT
/* Proximity interrupt config */

static uint8_t g_prox_lthreshold = LT1PA01_PROX_INT_TL_DEFAULT;
static uint8_t g_prox_hthreshold = LT1PA01_PROX_INT_TH_DEFAULT;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lt1pa01_getreg8
 *
 * Description:
 *   Read from an 8-bit LT1PA01 register
 *
 ****************************************************************************/

static uint8_t lt1pa01_getreg8(struct lt1pa01_dev_s *priv,
                               uint8_t regaddr)
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
 * Name: lt1pa01_putreg8
 *
 * Description:
 *   Write to an 8-bit LT1PA01 register
 *
 ****************************************************************************/

static void lt1pa01_putreg8(struct lt1pa01_dev_s *priv,
                            uint8_t regaddr, uint8_t regval)
{
  uint16_t inst[2];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_SEND(regval) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 2, NULL, 0);
}

/****************************************************************************
 * Name: lt1pa01_checkid
 *
 * Description:
 *   Read and verify the LT1PA01 chip ID
 *
 ****************************************************************************/

static int lt1pa01_checkid(struct lt1pa01_dev_s *priv)
{
  uint8_t id;

  /* Read Device ID */

  id = lt1pa01_getreg8(priv, LT1PA01_ID);
  sninfo("LT1PA01 ID:%02X\n", id);

  if ((id & LT1PA01_ID_MASK) != LT1PA01_DEVICEID)
    {
      /* Device ID is not Correct */

      snerr("Wrong Device ID! %02x (Exp:%02x)\n", id, LT1PA01_DEVICEID);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: lt1pa01als_seqinit
 *
 * Description:
 *   Initialize SCU sequencer.
 *
 ****************************************************************************/

static int lt1pa01als_seqinit(struct lt1pa01_dev_s *priv)
{
  DEBUGASSERT(g_als_seq == NULL);

  /* Open sequencer */

  g_als_seq = seq_open(SEQ_TYPE_NORMAL, SCU_BUS_I2C0);
  if (!g_als_seq)
    {
      return -ENOENT;
    }

  priv->seq = g_als_seq;

  seq_setaddress(priv->seq, priv->addr);

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq,
                     g_lt1pa01alsinst,
                     itemsof(g_lt1pa01alsinst));
  seq_setsample(priv->seq,
                LT1PA01_ALS_BYTESPERSAMPLE,
                0,
                LT1PA01_ELEMENTSIZE,
                false);

  return OK;
}

#ifndef CONFIG_LT1PA01_PROXIMITY_INTERRUPT
/****************************************************************************
 * Name: lt1pa01prox_seqinit
 *
 * Description:
 *   Initialize SCU sequencer.
 *
 ****************************************************************************/

static int lt1pa01prox_seqinit(struct lt1pa01_dev_s *priv)
{
  DEBUGASSERT(g_prox_seq == NULL);

  /* Open sequencer */

  g_prox_seq = seq_open(SEQ_TYPE_NORMAL, SCU_BUS_I2C0);
  if (!g_prox_seq)
    {
      return -ENOENT;
    }

  priv->seq = g_prox_seq;

  seq_setaddress(priv->seq, priv->addr);

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq,
                     g_lt1pa01proxinst,
                     itemsof(g_lt1pa01proxinst));
  seq_setsample(priv->seq,
                LT1PA01_PROX_BYTESPERSAMPLE,
                0,
                LT1PA01_ELEMENTSIZE,
                false);

  return OK;
}
#endif

/****************************************************************************
 * Name: lt1pa01_open_als
 *
 * Description:
 *   This function is called whenever the LT1PA01 device is opened.
 *
 ****************************************************************************/

static int lt1pa01_open_als(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct lt1pa01_dev_s *priv = inode->i_private;
  uint8_t val;

  if (g_als_refcnt == 0)
    {
      int ret;

      ret = lt1pa01als_seqinit(priv);
      if (ret < 0)
        {
          return ret;
        }

      /* ALS Enable */

      val = lt1pa01_getreg8(priv, LT1PA01_CONFIG1);
      val |= (LT1PA01_ALS_EN | LT1PA01_ALS_ALS_RANGE2);
      lt1pa01_putreg8(priv, LT1PA01_CONFIG1, val);
    }
  else
    {
      /* Set existing sequencer */

      priv->seq = g_als_seq;
    }

  g_als_refcnt++;

  return OK;
}

/****************************************************************************
 * Name: lt1pa01_open_prox
 *
 * Description:
 *   This function is called whenever the LT1PA01 device is opened.
 *
 ****************************************************************************/

static int lt1pa01_open_prox(struct file *filep)
{
#ifndef CONFIG_LT1PA01_PROXIMITY_INTERRUPT
  struct inode *inode = filep->f_inode;
  struct lt1pa01_dev_s *priv = inode->i_private;
  uint8_t val;

  if (g_prox_refcnt == 0)
    {
      int ret;

      ret = lt1pa01prox_seqinit(priv);
      if (ret < 0)
        {
          return ret;
        }

      /* PROX Enable */

      val = LT1PA01_PROX_EN | LT1PA01_PROX_SLP_100MS;
      lt1pa01_putreg8(priv, LT1PA01_CONFIG0, val);
    }
  else
    {
      /* Set existing sequencer */

      priv->seq = g_prox_seq;
    }

  g_prox_refcnt++;
#endif

  return OK;
}

/****************************************************************************
 * Name: lt1pa01_close_als
 *
 * Description:
 *   This routine is called when the LT1PA01 device is closed.
 *
 ****************************************************************************/

static int lt1pa01_close_als(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct lt1pa01_dev_s *priv = inode->i_private;
  uint8_t val;

  g_als_refcnt--;

  seq_ioctl(priv->seq, priv->minor, SCUIOC_STOP, 0);

  if (g_als_refcnt == 0)
    {
      /* ALS Disable */

      val = lt1pa01_getreg8(priv, LT1PA01_CONFIG1);
      val &= ~LT1PA01_ALS_EN;
      val &= ~LT1PA01_ALS_ALS_RANGE_MASK;
      lt1pa01_putreg8(priv, LT1PA01_CONFIG1, val);

      seq_close(g_als_seq);
      g_als_seq = NULL;
    }
  else
    {
      seq_ioctl(priv->seq, priv->minor, SCUIOC_FREEFIFO, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: lt1pa01_close_prox
 *
 * Description:
 *   This routine is called when the LT1PA01 device is closed.
 *
 ****************************************************************************/

static int lt1pa01_close_prox(struct file *filep)
{
#ifndef CONFIG_LT1PA01_PROXIMITY_INTERRUPT
  struct inode *inode = filep->f_inode;
  struct lt1pa01_dev_s *priv = inode->i_private;

  g_prox_refcnt--;

  seq_ioctl(priv->seq, priv->minor, SCUIOC_STOP, 0);

  if (g_prox_refcnt == 0)
    {
      /* PROX Disable */

      lt1pa01_putreg8(priv, LT1PA01_CONFIG0, 0);

      seq_close(g_prox_seq);
      g_prox_seq = NULL;
    }
  else
    {
      seq_ioctl(priv->seq, priv->minor, SCUIOC_FREEFIFO, 0);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: lt1pa01_read_als
 ****************************************************************************/

static ssize_t lt1pa01_read_als(struct file *filep, char *buffer,
                                size_t len)
{
  struct inode *inode = filep->f_inode;
  struct lt1pa01_dev_s *priv = inode->i_private;

  len = len / LT1PA01_ALS_BYTESPERSAMPLE * LT1PA01_ALS_BYTESPERSAMPLE;
  len = seq_read(priv->seq, priv->minor, buffer, len);

  return len;
}

/****************************************************************************
 * Name: lt1pa01_read_prox
 ****************************************************************************/

static ssize_t lt1pa01_read_prox(struct file *filep, char *buffer,
                                 size_t len)
{
  struct inode *inode = filep->f_inode;
  struct lt1pa01_dev_s *priv = inode->i_private;

  len = len / LT1PA01_PROX_BYTESPERSAMPLE * LT1PA01_PROX_BYTESPERSAMPLE;

#ifdef CONFIG_LT1PA01_PROXIMITY_INTERRUPT
  if (len)
    {
      len = LT1PA01_PROX_BYTESPERSAMPLE;
      *(uint8_t *)buffer = lt1pa01_getreg8(priv, LT1PA01_PROX_DATA);
    }
#else
  len = seq_read(priv->seq, priv->minor, buffer, len);
#endif

  return len;
}

/****************************************************************************
 * Name: lt1pa01_write
 ****************************************************************************/

static ssize_t lt1pa01_write(struct file *filep, const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lt1pa01_ioctl_als
 ****************************************************************************/

static int lt1pa01_ioctl_als(struct file *filep, int cmd,
                             unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct lt1pa01_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      default:
        {
          if (_SCUIOCVALID(cmd))
            {
              /* Redirect SCU commands */

              ret = seq_ioctl(priv->seq, priv->minor, cmd, arg);
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
 * Name: lt1pa01_ioctl_prox
 ****************************************************************************/

static int lt1pa01_ioctl_prox(struct file *filep, int cmd,
                              unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct lt1pa01_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
#ifdef CONFIG_LT1PA01_PROXIMITY_INTERRUPT
      case SNIOC_SETPROXLTHRESHOLD:
        {
          g_prox_lthreshold = (uint8_t)arg;
        }
        break;

      case SNIOC_SETPROXHTHRESHOLD:
        {
          g_prox_hthreshold = (uint8_t)arg;
        }
        break;

      case SNIOC_STARTPROXMEASUREMENT:
        {
          uint8_t val;

          /* PROX Interrupt setting */

          lt1pa01_putreg8(priv, LT1PA01_PROX_INT_TL, g_prox_lthreshold);
          lt1pa01_putreg8(priv, LT1PA01_PROX_INT_TH, g_prox_hthreshold);
          val = LT1PA01_PORX_INT_PRST2 | LT1PA01_INT_CFG_ALS_OR_PROX;
          lt1pa01_putreg8(priv, LT1PA01_INTCONFIG, val);
          val = lt1pa01_getreg8(priv, LT1PA01_CONFIG1);
          val |= LT1PA01_INT_ALG_COMPARATOR;
          lt1pa01_putreg8(priv, LT1PA01_CONFIG1, val);

          /* PROX Enable */

          val = LT1PA01_PROX_EN | LT1PA01_PROX_SLP_100MS;
          lt1pa01_putreg8(priv, LT1PA01_CONFIG0, val);
        }
        break;

      case SNIOC_STOPPROXMEASUREMENT:
        {
          /* PROX Disable */

          lt1pa01_putreg8(priv, LT1PA01_CONFIG1, 0);
        }
        break;

      case SNIOC_GETINTSTATUS:
        {
          uint8_t intstatus = lt1pa01_getreg8(priv, LT1PA01_INTCONFIG);
          *(uint8_t *)(uintptr_t)arg = intstatus;
          sninfo("Get proximity IntStatus 0x%02x\n", intstatus);
        }
        break;
#endif

      default:
        {
          if (_SCUIOCVALID(cmd))
            {
#ifndef CONFIG_LT1PA01_PROXIMITY_INTERRUPT
              /* Redirect SCU commands */

              ret = seq_ioctl(priv->seq, priv->minor, cmd, arg);
#else
              snerr("Unregistered SCU sequencer cmd: %d\n", cmd);
              ret = - ENOTTY;
#endif
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
 * Name: lt1pa01_init
 *
 * Description:
 *   Initialize the LT1PA01 device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             LT1PA01
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lt1pa01_init(struct i2c_master_s *i2c, int port)
{
  struct lt1pa01_dev_s tmp;
  struct lt1pa01_dev_s *priv = &tmp;
  int ret;

  /* Setup temporary device structure for initialization */

  priv->i2c = i2c;
  priv->addr = LT1PA01_ADDR;
  priv->port = port;

  /* Check Device ID */

  ret = lt1pa01_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ret;
    }

  /* Software reset */

  lt1pa01_putreg8(priv, LT1PA01_CONFIG3, LT1PA01_SW_RESET);

  /* Power-up and Brown-out Reset */

  lt1pa01_putreg8(priv, LT1PA01_INTCONFIG, 0);

  return OK;
}

/****************************************************************************
 * Name: lt1pa01als_register
 *
 * Description:
 *   Register the LT1PA01 ambient light sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             LT1PA01
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lt1pa01als_register(const char *devpath, int minor,
                        struct i2c_master_s *i2c, int port)
{
  struct lt1pa01_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the LT1PA01 device structure */

  priv = (struct lt1pa01_dev_s *)
    kmm_malloc(sizeof(struct lt1pa01_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = LT1PA01_ADDR;
  priv->port = port;
  priv->seq = NULL;
  priv->minor = minor;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_lt1pa01alsfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: lt1pa01prox_register
 *
 * Description:
 *   Register the LT1PA01 proximity sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/proxim0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             LT1PA01
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lt1pa01prox_register(const char *devpath, int minor,
                         struct i2c_master_s *i2c, int port)
{
  struct lt1pa01_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the LT1PA01 device structure */

  priv = (struct lt1pa01_dev_s *)
    kmm_malloc(sizeof(struct lt1pa01_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = LT1PA01_ADDR;
  priv->port = port;
  priv->seq = NULL;
  priv->minor = minor;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_lt1pa01proxfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("LT1PA01 proximity sensor driver loaded successfully!\n");

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_LT1PA01_SCU */

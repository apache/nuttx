/****************************************************************************
 * boards/arm/cxd56xx/drivers/sensors/bm1422gmv_scu.c
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
#include <nuttx/sensors/bm1422gmv.h>
#include <nuttx/irq.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BM1422GMV_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BM1422GMV_SCU_DECI
#  define BM1422GMV_SEQ_TYPE SEQ_TYPE_DECI
#else
#  define BM1422GMV_SEQ_TYPE SEQ_TYPE_NORMAL
#endif

/* I2C Slave Address */

#ifdef CONFIG_BM1422GMV_SLAVE_ADDRESS_0F
#define BM1422GMV_ADDR              0x0F
#else
#define BM1422GMV_ADDR              0x0E
#endif

#define BM1422GMV_DEVID             0x41    /* Device ID */
#define BM1422GMV_BYTESPERSAMPLE    6
#define BM1422GMV_ELEMENTSIZE       2

/* BM1422GMV Registers */

#define BM1422GMV_WIA               0x0F
#define BM1422GMV_DATAX             0x10
#define BM1422GMV_CNTL1             0x1B
#define BM1422GMV_CNTL2             0x1C
#define BM1422GMV_CNTL3             0x1D
#define BM1422GMV_CNTL4             0x5C

/* Register CNTL1 */

#define BM1422GMV_CNTL1_PC1         (1 << 7)
#define BM1422GMV_CNTL1_OUT_BIT     (1 << 6)
#define BM1422GMV_CNTL1_RST_LV      (1 << 5)
#define BM1422GMV_CNTL1_ODR_10Hz    (0 << 3)
#define BM1422GMV_CNTL1_ODR_20Hz    (2 << 3)
#define BM1422GMV_CNTL1_ODR_100Hz   (1 << 3)
#define BM1422GMV_CNTL1_ODR_1000Hz  (3 << 3)
#define BM1422GMV_CNTL1_FS1         (1 << 1)

/* Register CNTL2 */

#define BM1422GMV_CNTL2_DREN        (1 << 3)
#define BM1422GMV_CNTL2_DRP         (1 << 2)

/* Register CNTL3 */

#define BM1422GMV_CNTL3_FORCE       (1 << 6)

/* Register CNTL4 */

#define BM1422GMV_CNTL4_VAL         0x0000

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Structure for bm1422gmv device */

struct bm1422gmv_dev_s
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

static int bm1422gmv_open(struct file *filep);
static int bm1422gmv_close(struct file *filep);
static ssize_t bm1422gmv_read(struct file *filep, char *buffer,
                              size_t buflen);
static ssize_t bm1422gmv_write(struct file *filep,
                               const char *buffer, size_t buflen);
static int bm1422gmv_ioctl(struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bm1422gmvfops =
{
  bm1422gmv_open,              /* open */
  bm1422gmv_close,             /* close */
  bm1422gmv_read,              /* read */
  bm1422gmv_write,             /* write */
  NULL,                        /* seek */
  bm1422gmv_ioctl,             /* ioctl */
};

/* Take XYZ data. */

static const uint16_t g_bm1422gmvinst[] =
{
  SCU_INST_SEND(BM1422GMV_DATAX),
  SCU_INST_RECV(BM1422GMV_BYTESPERSAMPLE) | SCU_INST_LAST,
};

/* Reference count */

static int g_refcnt = 0;

/* Sequencer instance */

static struct seq_s *g_seq = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm1422gmv_getreg8
 *
 * Description:
 *   Read from an 8-bit BM1422GMV register
 *
 ****************************************************************************/

static uint8_t bm1422gmv_getreg8(struct bm1422gmv_dev_s *priv,
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
 * Name: bm1422gmv_putreg8
 *
 * Description:
 *   Write to an 8-bit BM1422GMV register
 *
 ****************************************************************************/

static void bm1422gmv_putreg8(struct bm1422gmv_dev_s *priv,
                              uint8_t regaddr, uint8_t regval)
{
  uint16_t inst[2];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_SEND(regval) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 2, NULL, 0);
}

/****************************************************************************
 * Name: bm1422gmv_putreg16
 *
 * Description:
 *   Write to an 16-bit BM1422GMV register
 *
 ****************************************************************************/

static void bm1422gmv_putreg16(struct bm1422gmv_dev_s *priv,
                               uint8_t regaddr, uint16_t regval)
{
  uint16_t inst[3];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_SEND((uint8_t)(regval & 0xff));
  inst[2] = SCU_INST_SEND((uint8_t)(regval >> 8)) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 3, NULL, 0);
}

/****************************************************************************
 * Name: bm1422gmv_checkid
 *
 * Description:
 *   Read and verify the BM1422GMV chip ID
 *
 ****************************************************************************/

static int bm1422gmv_checkid(struct bm1422gmv_dev_s *priv)
{
  uint8_t devid;

  /* Read device ID */

  devid = bm1422gmv_getreg8(priv, BM1422GMV_WIA);

  if (devid != BM1422GMV_DEVID)
    {
      /* ID is not Correct */

      snerr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bm1422gmv_seqinit
 *
 * Description:
 *   Initialize SCU sequencer.
 *
 ****************************************************************************/

static int bm1422gmv_seqinit(struct bm1422gmv_dev_s *priv)
{
  DEBUGASSERT(g_seq == NULL);

  /* Open sequencer */

  g_seq = seq_open(BM1422GMV_SEQ_TYPE, SCU_BUS_I2C0);
  if (!g_seq)
    {
      return -ENOENT;
    }

  priv->seq = g_seq;

  seq_setaddress(priv->seq, priv->addr);

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq, g_bm1422gmvinst, itemsof(g_bm1422gmvinst));
  seq_setsample(priv->seq, BM1422GMV_BYTESPERSAMPLE, 0,
                BM1422GMV_ELEMENTSIZE, false);

  return OK;
}

/****************************************************************************
 * Name: bm1422gmv_open
 *
 * Description:
 *   This function is called whenever the BM1422GMV device is opened.
 *
 ****************************************************************************/

static int bm1422gmv_open(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct bm1422gmv_dev_s *priv = inode->i_private;
  uint8_t val;

  if (g_refcnt == 0)
    {
      int ret;

      ret = bm1422gmv_seqinit(priv);
      if (ret < 0)
        {
          return ret;
        }

      /* goto active mode */

      val = BM1422GMV_CNTL1_PC1 | BM1422GMV_CNTL1_OUT_BIT |
            BM1422GMV_CNTL1_ODR_100Hz;
      bm1422gmv_putreg8(priv, BM1422GMV_CNTL1, val);
      up_mdelay(1);

      /* release reset */

      bm1422gmv_putreg16(priv, BM1422GMV_CNTL4, BM1422GMV_CNTL4_VAL);

      /* start sampling */

      bm1422gmv_putreg8(priv, BM1422GMV_CNTL3, BM1422GMV_CNTL3_FORCE);
    }
  else
    {
      /* Set existing sequencer */

      priv->seq = g_seq;
    }

  g_refcnt++;

  return OK;
}

/****************************************************************************
 * Name: bm1422gmv_close
 *
 * Description:
 *   This routine is called when the BM1422GMV device is closed.
 *
 ****************************************************************************/

static int bm1422gmv_close(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct bm1422gmv_dev_s *priv = inode->i_private;

  g_refcnt--;

  seq_ioctl(priv->seq, priv->minor, SCUIOC_STOP, 0);

  if (g_refcnt == 0)
    {
      /* goto power-down mode */

      bm1422gmv_putreg8(priv, BM1422GMV_CNTL1, BM1422GMV_CNTL1_RST_LV);
      up_mdelay(1);

      seq_close(g_seq);
      g_seq = NULL;
    }
  else
    {
      seq_ioctl(priv->seq, priv->minor, SCUIOC_FREEFIFO, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: bm1422gmv_read
 ****************************************************************************/

static ssize_t bm1422gmv_read(struct file *filep, char *buffer,
                              size_t len)
{
  struct inode *inode = filep->f_inode;
  struct bm1422gmv_dev_s *priv = inode->i_private;

  len = len / BM1422GMV_BYTESPERSAMPLE * BM1422GMV_BYTESPERSAMPLE;
  len = seq_read(priv->seq, priv->minor, buffer, len);

  return len;
}

/****************************************************************************
 * Name: bm1422gmv_write
 ****************************************************************************/

static ssize_t bm1422gmv_write(struct file *filep,
                               const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bm1422gmv_ioctl
 ****************************************************************************/

static int bm1422gmv_ioctl(struct file *filep, int cmd,
                           unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct bm1422gmv_dev_s *priv = inode->i_private;
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm1422gmv_init
 *
 * Description:
 *   Initialize the BM1422GMV device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BM1422GMV
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bm1422gmv_init(struct i2c_master_s *i2c, int port)
{
  struct bm1422gmv_dev_s tmp;
  struct bm1422gmv_dev_s *priv = &tmp;
  int ret;

  /* Setup temporary device structure for initialization */

  priv->i2c = i2c;
  priv->addr = BM1422GMV_ADDR;
  priv->port = port;

  /* Check Device ID */

  ret = bm1422gmv_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bm1422gmv_register
 *
 * Description:
 *   Register the BM1422GMV character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mag0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BM1422GMV
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bm1422gmv_register(const char *devpath, int minor,
                       struct i2c_master_s *i2c, int port)
{
  struct bm1422gmv_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the BM1422GMV device structure */

  priv = (struct bm1422gmv_dev_s *)
    kmm_malloc(sizeof(struct bm1422gmv_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = BM1422GMV_ADDR;
  priv->port = port;
  priv->seq = NULL;
  priv->minor = minor;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_bm1422gmvfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BM1422GMV driver loaded successfully!\n");

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BM1422GMV_SCU */

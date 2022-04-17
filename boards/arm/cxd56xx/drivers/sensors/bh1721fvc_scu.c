/****************************************************************************
 * boards/arm/cxd56xx/drivers/sensors/bh1721fvc_scu.c
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
#include <nuttx/sensors/bh1721fvc.h>
#include <nuttx/irq.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BH1721FVC_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BH1721FVC_ADDR                  0x23    /* I2C Slave Address */
#define BH1721FVC_BYTESPERSAMPLE        2
#define BH1721FVC_ELEMENTSIZE           0

/* BH1721FVC Opecode */

#define BH1721FVC_POWERDOWN             0x00
#define BH1721FVC_POWERON               0x01
#define BH1721FVC_AUTORESOLUTION        0x10

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/**
 * @brief Structure for bh1721fvc device
 */

struct bh1721fvc_dev_s
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

static int bh1721fvc_open(struct file *filep);
static int bh1721fvc_close(struct file *filep);
static ssize_t bh1721fvc_read(struct file *filep,
                              char *buffer,
                              size_t buflen);
static ssize_t bh1721fvc_write(struct file *filep,
                               const char *buffer,
                               size_t buflen);
static int bh1721fvc_ioctl(struct file *filep,
                           int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bh1721fvcfops =
{
  bh1721fvc_open,              /* open */
  bh1721fvc_close,             /* close */
  bh1721fvc_read,              /* read */
  bh1721fvc_write,             /* write */
  NULL,                        /* seek */
  bh1721fvc_ioctl,             /* ioctl */
  NULL                         /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                       /* unlink */
#endif
};

/* Take ambient light data. */

static const uint16_t g_bh1721fvcinst[] =
{
  SCU_INST_RECV(BH1721FVC_BYTESPERSAMPLE) | SCU_INST_LAST,
};

/* Reference count */

static int g_refcnt = 0;

/* Sequencer instance */

static struct seq_s *g_seq = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1721fvc_writeopecode
 *
 * Description:
 *   Write to BH1721FVC opecode
 *
 ****************************************************************************/

static void bh1721fvc_writeopecode(struct bh1721fvc_dev_s *priv,
                                   uint8_t opecode)
{
  uint16_t inst = SCU_INST_SEND(opecode) | SCU_INST_LAST;

  /* Send opecode */

  scu_i2ctransfer(priv->port, priv->addr, &inst, 1, NULL, 0);
}

/****************************************************************************
 * Name: bh1721fvc_seqinit
 *
 * Description:
 *   Initialize SCU sequencer.
 *
 ****************************************************************************/

static int bh1721fvc_seqinit(struct bh1721fvc_dev_s *priv)
{
  DEBUGASSERT(g_seq == NULL);

  /* Open sequencer */

  g_seq = seq_open(SEQ_TYPE_NORMAL, SCU_BUS_I2C0);
  if (!g_seq)
    {
      return -ENOENT;
    }

  priv->seq = g_seq;

  seq_setaddress(priv->seq, priv->addr);

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq,
                     g_bh1721fvcinst,
                     itemsof(g_bh1721fvcinst));
  seq_setsample(priv->seq,
                BH1721FVC_BYTESPERSAMPLE,
                0,
                BH1721FVC_ELEMENTSIZE,
                false);

  return OK;
}

/****************************************************************************
 * Name: bh1721fvc_open
 *
 * Description:
 *   This function is called whenever the BH1721FVC device is opened.
 *
 ****************************************************************************/

static int bh1721fvc_open(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct bh1721fvc_dev_s *priv = inode->i_private;

  if (g_refcnt == 0)
    {
      int ret;

      ret = bh1721fvc_seqinit(priv);
      if (ret < 0)
        {
          return ret;
        }

      bh1721fvc_writeopecode(priv, BH1721FVC_POWERON);
      bh1721fvc_writeopecode(priv, BH1721FVC_AUTORESOLUTION);
    }

  g_refcnt++;

  return OK;
}

/****************************************************************************
 * Name: bh1721fvc_close
 *
 * Description:
 *   This routine is called when the BH1721FVC device is closed.
 *
 ****************************************************************************/

static int bh1721fvc_close(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct bh1721fvc_dev_s *priv = inode->i_private;

  g_refcnt--;

  seq_ioctl(priv->seq, priv->minor, SCUIOC_STOP, 0);

  if (g_refcnt == 0)
    {
      bh1721fvc_writeopecode(priv, BH1721FVC_POWERDOWN);

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
 * Name: bh1721fvc_read
 ****************************************************************************/

static ssize_t bh1721fvc_read(struct file *filep, char *buffer,
                              size_t len)
{
  struct inode *inode = filep->f_inode;
  struct bh1721fvc_dev_s *priv = inode->i_private;

  len = len / BH1721FVC_BYTESPERSAMPLE * BH1721FVC_BYTESPERSAMPLE;
  len = seq_read(priv->seq, priv->minor, buffer, len);

  return len;
}

/****************************************************************************
 * Name: bh1721fvc_write
 ****************************************************************************/

static ssize_t bh1721fvc_write(struct file *filep,
                               const char *buffer,
                               size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bh1721fvc_ioctl
 ****************************************************************************/

static int bh1721fvc_ioctl(struct file *filep, int cmd,
                           unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct bh1721fvc_dev_s *priv = inode->i_private;
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
 * Name: bh1721fvc_init
 *
 * Description:
 *   Initialize the BH1721FVC device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BH1721FVC
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1721fvc_init(struct i2c_master_s *i2c, int port)
{
  return OK;
}

/****************************************************************************
 * Name: bh1721fvc_register
 *
 * Description:
 *   Register the BH1721FVC ambient light sensor character device as
 *   'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BH1721FVC
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1721fvc_register(const char *devpath, int minor,
                       struct i2c_master_s *i2c, int port)
{
  struct bh1721fvc_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the BH1721FVC device structure */

  priv = (struct bh1721fvc_dev_s *)
    kmm_malloc(sizeof(struct bh1721fvc_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = BH1721FVC_ADDR;
  priv->port = port;
  priv->seq = NULL;
  priv->minor = minor;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_bh1721fvcfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BH1721FVC_SCU */

/****************************************************************************
 * boards/arm/cxd56xx/drivers/sensors/bm1383glv_scu.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
#include <stdio.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>
#include <arch/types.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bm1383glv.h>
#include <nuttx/irq.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BM1383GLV_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BM1383GLV_ADDR              0x5D    /* I2C Slave Address */

#define BM1383GLV_DEVID             0x31
#define BM1383AGLV_DEVID            0x32

    /* Device ID */

#define BM1383GLV_BYTESPERSAMPLE    3
#define BM1383GLV_ELEMENTSIZE       0

/* BM1383GLV Registers */

#define BM1383GLV_ID                0x10
#define BM1383GLV_POWER_DOWN        0x12
#define BM1383GLV_RESET             0x13
#define BM1383GLV_MODE_CONTROL      0x14
#define BM1383GLV_PRESSURE_MSB      0x1C
#define BM1383AGLV_PRESSURE_MSB     0x1A

/* Register POWER_DOWN */

#define BM1383GLV_POWER_DOWN_PWR_DOWN      (1 << 0)

/* Register RESET */

#define BM1383GLV_RESET_RSTB               (1 << 0)

/* Register MODE_CONTROL */

#define BM1383GLV_MODE_CONTROL_AVE_NUM64   (6 << 5)
#define BM1383GLV_MODE_CONTROL_T_AVE       (1 << 3)
#define BM1383GLV_MODE_CONTORL_MODE_200MS  (4 << 0)

#define BM1383AGLV_MODE_CONTROL_AVE_NUM64  (6 << 5)
#define BM1383AGLV_MODE_CONTROL_RESERVED   (1 << 3)
#define BM1383AGLV_MODE_CONTROL_CONTINUOUS (2 << 0)

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Structure for bm1383glv device */

struct bm1383glv_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  int port;                     /* I2C port */
  struct seq_s *seq;            /* Sequencer instance */
  int minor;                    /* Minor device number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int bm1383glv_open(FAR struct file *filep);
static int bm1383glv_close(FAR struct file *filep);
static ssize_t bm1383glv_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t bm1383glv_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int bm1383glv_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bm1383glvfops =
{
  bm1383glv_open,              /* open */
  bm1383glv_close,             /* close */
  bm1383glv_read,              /* read */
  bm1383glv_write,             /* write */
  0,                           /* seek */
  bm1383glv_ioctl,             /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                           /* poll */
#endif
  0                            /* unlink */
};

/* Device is not BM1383AGLV but BM1383GLV */

static uint8_t g_is_bm1383glv = 0;

/* Take press data for BM1383GLV */

static const uint16_t g_bm1383glvinst[] =
{
  SCU_INST_SEND(BM1383GLV_PRESSURE_MSB),
  SCU_INST_RECV(BM1383GLV_BYTESPERSAMPLE) | SCU_INST_LAST,
};

/* Take press data for BM1383AGLV */

static const uint16_t g_bm1383aglvinst[] =
{
  SCU_INST_SEND(BM1383AGLV_PRESSURE_MSB),
  SCU_INST_RECV(BM1383GLV_BYTESPERSAMPLE) | SCU_INST_LAST,
};

/* Reference count */

static int g_refcnt = 0;

/* Sequencer instance */

static struct seq_s *g_seq = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm1383glv_getreg8
 *
 * Description:
 *   Read from an 8-bit BM1383GLV register
 *
 ****************************************************************************/

static uint8_t bm1383glv_getreg8(FAR struct bm1383glv_dev_s *priv,
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
 * Name: bm1383glv_putreg8
 *
 * Description:
 *   Write to an 8-bit BM1383GLV register
 *
 ****************************************************************************/

static void bm1383glv_putreg8(FAR struct bm1383glv_dev_s *priv,
                              uint8_t regaddr, uint8_t regval)
{
  uint16_t inst[2];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_SEND(regval) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 2, NULL, 0);
}

/****************************************************************************
 * Name: bm1383glv_checkid
 *
 * Description:
 *   Read and verify the BM1383GLV chip ID
 *
 ****************************************************************************/

static int bm1383glv_checkid(FAR struct bm1383glv_dev_s *priv)
{
  uint8_t devid;

  /* Read device ID */

  devid = bm1383glv_getreg8(priv, BM1383GLV_ID);

  if ((devid != BM1383GLV_DEVID) && (devid != BM1383AGLV_DEVID))
    {
      /* ID is not Correct */

      snerr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  if (devid == BM1383GLV_DEVID)
    {
      /* Device is BM1383GLV, which remains for backward compatibility */

      g_is_bm1383glv = 1;
    }

  return OK;
}

/****************************************************************************
 * Name: bm1383glv_seqinit
 *
 * Description:
 *   Initialize SCU sequencer.
 *
 ****************************************************************************/

static int bm1383glv_seqinit(FAR struct bm1383glv_dev_s *priv)
{
  const uint16_t *inst;
  uint16_t nr;

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

  if (g_is_bm1383glv)
    {
      inst = g_bm1383glvinst;
      nr   = itemsof(g_bm1383glvinst);
    }
  else
    {
      inst = g_bm1383aglvinst;
      nr   = itemsof(g_bm1383aglvinst);
    }

  seq_setinstruction(priv->seq, inst, nr);
  seq_setsample(priv->seq, BM1383GLV_BYTESPERSAMPLE, 0,
                BM1383GLV_ELEMENTSIZE, false);

  return OK;
}

/****************************************************************************
 * Name: bm1383glv_open
 *
 * Description:
 *   This function is called whenever the BM1383GLV device is opened.
 *
 ****************************************************************************/

static int bm1383glv_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bm1383glv_dev_s *priv = inode->i_private;
  uint8_t val;

  if (g_refcnt == 0)
    {
      int ret;

      ret = bm1383glv_seqinit(priv);
      if (ret < 0)
        {
          return ret;
        }

      /* goto reset mode */

      bm1383glv_putreg8(priv, BM1383GLV_POWER_DOWN,
                        BM1383GLV_POWER_DOWN_PWR_DOWN);
      up_mdelay(1);

      /* goto stand-by mode */

      bm1383glv_putreg8(priv, BM1383GLV_RESET, BM1383GLV_RESET_RSTB);

      /* start sampling */

      if (g_is_bm1383glv)
        {
          val = BM1383GLV_MODE_CONTROL_AVE_NUM64 |
                BM1383GLV_MODE_CONTROL_T_AVE |
                BM1383GLV_MODE_CONTORL_MODE_200MS;
        }
      else
        {
          val = BM1383AGLV_MODE_CONTROL_AVE_NUM64 |
                BM1383AGLV_MODE_CONTROL_RESERVED |
                BM1383AGLV_MODE_CONTROL_CONTINUOUS;
        }

      bm1383glv_putreg8(priv, BM1383GLV_MODE_CONTROL, val);
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
 * Name: bm1383glv_close
 *
 * Description:
 *   This routine is called when the BM1383GLV device is closed.
 *
 ****************************************************************************/

static int bm1383glv_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bm1383glv_dev_s *priv = inode->i_private;

  g_refcnt--;

  seq_ioctl(priv->seq, priv->minor, SCUIOC_STOP, 0);

  if (g_refcnt == 0)
    {
      /* goto stand-by mode */

      bm1383glv_putreg8(priv, BM1383GLV_MODE_CONTROL, 0);

      /* goto reset mode */

      bm1383glv_putreg8(priv, BM1383GLV_RESET, 0);

      /* goto power-down mode */

      bm1383glv_putreg8(priv, BM1383GLV_POWER_DOWN, 0);

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
 * Name: bm1383glv_read
 ****************************************************************************/

static ssize_t bm1383glv_read(FAR struct file *filep, FAR char *buffer,
                              size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bm1383glv_dev_s *priv = inode->i_private;

  len = len / BM1383GLV_BYTESPERSAMPLE * BM1383GLV_BYTESPERSAMPLE;
  len = seq_read(priv->seq, priv->minor, buffer, len);

  return len;
}

/****************************************************************************
 * Name: bm1383glv_write
 ****************************************************************************/

static ssize_t bm1383glv_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bm1383glv_ioctl
 ****************************************************************************/

static int bm1383glv_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bm1383glv_dev_s *priv = inode->i_private;
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
 * Name: bm1383glv_init
 *
 * Description:
 *   Initialize the BM1383GLV device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BM1383GLV
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bm1383glv_init(FAR struct i2c_master_s *i2c, int port)
{
  FAR struct bm1383glv_dev_s tmp;
  FAR struct bm1383glv_dev_s *priv = &tmp;
  int ret;

  /* Setup temporary device structure for initialization */

  priv->i2c = i2c;
  priv->addr = BM1383GLV_ADDR;
  priv->port = port;

  /* Check Device ID */

  ret = bm1383glv_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bm1383glv_register
 *
 * Description:
 *   Register the BM1383GLV character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BM1383GLV
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bm1383glv_register(FAR const char *devpath, int minor,
                       FAR struct i2c_master_s *i2c, int port)
{
  FAR struct bm1383glv_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the BM1383GLV device structure */

  priv = (FAR struct bm1383glv_dev_s *)
    kmm_malloc(sizeof(struct bm1383glv_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = BM1383GLV_ADDR;
  priv->port = port;
  priv->seq = NULL;
  priv->minor = minor;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_bm1383glvfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BM1383GLV driver loaded successfully!\n");

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BM1383GLV_SCU */

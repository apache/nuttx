/****************************************************************************
 * boards/arm/cxd56xx/drivers/sensors/bh1745nuc_scu.c
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
#include <nuttx/sensors/bh1745nuc.h>
#include <nuttx/irq.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BH1745NUC_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BH1745NUC_ADDR              0x39    /* I2C Slave Address */
#define BH1745NUC_MANUFACTID        0xE0    /* Manufact ID */
#define BH1745NUC_PARTID            0x0B    /* Part ID */
#define BH1745NUC_BYTESPERSAMPLE    8
#define BH1745NUC_ELEMENTSIZE       0

/* BH1745NUC Registers */

#define BH1745NUC_SYSTEM_CONTROL    0x40
#define BH1745NUC_MODE_CONTROL1     0x41
#define BH1745NUC_MODE_CONTROL2     0x42
#define BH1745NUC_MODE_CONTROL3     0x44
#define BH1745NUC_RED_DATA_LSB      0x50
#define BH1745NUC_MANUFACTURER_ID   0x92

/* Register SYSTEM_CONTROL */

#define BH1745NUC_SYSTEM_CONTROL_SW_RESET      (1 << 7)
#define BH1745NUC_SYSTEM_CONTROL_INT_RESET     (1 << 6)

/* Register MODE_CONTROL1 */

#define BH1745NUC_MODE_CONTROL1_MEAS_TIME160MS (0x00)

/* Register MODE_CONTROL2 */

#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X1    (0)
#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X2    (1)
#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X16   (2)
#define BH1745NUC_MODE_CONTROL2_RGBC_EN        (1 << 4)

/* Register MODE_CONTROL3 */

#define BH1745NUC_MODE_CONTROL3_VAL            (0x02)

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Structure for bh1745nuc device */

struct bh1745nuc_dev_s
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

static int bh1745nuc_open(FAR struct file *filep);
static int bh1745nuc_close(FAR struct file *filep);
static ssize_t bh1745nuc_read(FAR struct file *filep,
                              FAR char *buffer,
                              size_t buflen);
static ssize_t bh1745nuc_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t buflen);
static int bh1745nuc_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bh1745nucfops =
{
  bh1745nuc_open,              /* open */
  bh1745nuc_close,             /* close */
  bh1745nuc_read,              /* read */
  bh1745nuc_write,             /* write */
  0,                           /* seek */
  bh1745nuc_ioctl,             /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                           /* poll */
#endif
  0                            /* unlink */
};

/* Take color data. */

static const uint16_t g_bh1745nucinst[] =
{
  SCU_INST_SEND(BH1745NUC_RED_DATA_LSB),
  SCU_INST_RECV(BH1745NUC_BYTESPERSAMPLE) | SCU_INST_LAST,
};

/* Reference count */

static int g_refcnt = 0;

/* Sequencer instance */

static struct seq_s *g_seq = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1745nuc_getreg8
 *
 * Description:
 *   Read from an 8-bit BH1745NUC register
 *
 ****************************************************************************/

static uint8_t bh1745nuc_getreg8(FAR struct bh1745nuc_dev_s *priv,
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
 * Name: bh1745nuc_putreg8
 *
 * Description:
 *   Write to an 8-bit BH1745NUC register
 *
 ****************************************************************************/

static void bh1745nuc_putreg8(FAR struct bh1745nuc_dev_s *priv,
                              uint8_t regaddr, uint8_t regval)
{
  uint16_t inst[2];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_SEND(regval) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 2, NULL, 0);
}

/****************************************************************************
 * Name: bh1745nuc_checkid
 *
 * Description:
 *   Read and verify the BH1745NUC chip ID
 *
 ****************************************************************************/

static int bh1745nuc_checkid(FAR struct bh1745nuc_dev_s *priv)
{
  uint8_t id;

  /* Read Manufact ID */

  id = bh1745nuc_getreg8(priv, BH1745NUC_MANUFACTURER_ID);

  if (id != BH1745NUC_MANUFACTID)
    {
      /* Manufact ID is not Correct */

      snerr("Wrong Manufact ID! %02x\n", id);
      return -ENODEV;
    }

  /* Read Part ID */

  id = bh1745nuc_getreg8(priv, BH1745NUC_SYSTEM_CONTROL);

  if ((id & 0x3f) != BH1745NUC_PARTID)
    {
      /* Part ID is not Correct */

      snerr("Wrong Part ID! %02x\n", id);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bh1745nuc_seqinit
 *
 * Description:
 *   Initialize SCU sequencer.
 *
 ****************************************************************************/

static int bh1745nuc_seqinit(FAR struct bh1745nuc_dev_s *priv)
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

  seq_setinstruction(priv->seq, g_bh1745nucinst,
                     itemsof(g_bh1745nucinst));
  seq_setsample(priv->seq,
                BH1745NUC_BYTESPERSAMPLE,
                0,
                BH1745NUC_ELEMENTSIZE,
                false);

  return OK;
}

/****************************************************************************
 * Name: bh1745nuc_open
 *
 * Description:
 *   This function is called whenever the BH1745NUC device is opened.
 *
 ****************************************************************************/

static int bh1745nuc_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bh1745nuc_dev_s *priv  = inode->i_private;
  uint8_t val;

  if (g_refcnt == 0)
    {
      int ret;

      ret = bh1745nuc_seqinit(priv);
      if (ret < 0)
        {
          return ret;
        }

      /* MODE_CONTROL1 */

      val = BH1745NUC_MODE_CONTROL1_MEAS_TIME160MS;
      bh1745nuc_putreg8(priv, BH1745NUC_MODE_CONTROL1, val);

      /* MODE_CONTROL2 */

      val = BH1745NUC_MODE_CONTROL2_RGBC_EN |
            BH1745NUC_MODE_CONTROL2_ADC_GAIN_X16;
      bh1745nuc_putreg8(priv, BH1745NUC_MODE_CONTROL2, val);

      /* MODE_CONTROL3 */

      val = BH1745NUC_MODE_CONTROL3_VAL;
      bh1745nuc_putreg8(priv, BH1745NUC_MODE_CONTROL3, val);
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
 * Name: bh1745nuc_close
 *
 * Description:
 *   This routine is called when the BH1745NUC device is closed.
 *
 ****************************************************************************/

static int bh1745nuc_close(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bh1745nuc_dev_s *priv  = inode->i_private;
  uint8_t val;

  g_refcnt--;

  seq_ioctl(priv->seq, priv->minor, SCUIOC_STOP, 0);

  if (g_refcnt == 0)
    {
      /* stop sampling */

      val = BH1745NUC_SYSTEM_CONTROL_SW_RESET |
            BH1745NUC_SYSTEM_CONTROL_INT_RESET;
      bh1745nuc_putreg8(priv, BH1745NUC_SYSTEM_CONTROL, val);

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
 * Name: bh1745nuc_read
 ****************************************************************************/

static ssize_t bh1745nuc_read(FAR struct file *filep, FAR char *buffer,
                              size_t len)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bh1745nuc_dev_s *priv  = inode->i_private;

  len = len / BH1745NUC_BYTESPERSAMPLE * BH1745NUC_BYTESPERSAMPLE;
  len = seq_read(priv->seq, priv->minor, buffer, len);

  return len;
}

/****************************************************************************
 * Name: bh1745nuc_write
 ****************************************************************************/

static ssize_t bh1745nuc_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bh1745nuc_ioctl
 ****************************************************************************/

static int bh1745nuc_ioctl(FAR struct file *filep,
                           int cmd,
                           unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bh1745nuc_dev_s *priv = inode->i_private;
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
 * Name: bh1745nuc_init
 *
 * Description:
 *   Initialize the BH1745NUC device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BH1745NUC
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1745nuc_init(FAR struct i2c_master_s *i2c, int port)
{
  FAR struct bh1745nuc_dev_s tmp;
  FAR struct bh1745nuc_dev_s *priv = &tmp;
  int ret;

  /* Setup temporary device structure for initialization */

  priv->i2c = i2c;
  priv->addr = BH1745NUC_ADDR;
  priv->port = port;

  /* Check Device ID */

  ret = bh1745nuc_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bh1745nuc_register
 *
 * Description:
 *   Register the BH1745NUC character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/color0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BH1745NUC
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1745nuc_register(FAR const char *devpath, int minor,
                       FAR struct i2c_master_s *i2c, int port)
{
  FAR struct bh1745nuc_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the BH1745NUC device structure */

  priv = (FAR struct bh1745nuc_dev_s *)
    kmm_malloc(sizeof(struct bh1745nuc_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = BH1745NUC_ADDR;
  priv->port = port;
  priv->seq = NULL;
  priv->minor = minor;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_bh1745nucfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BH1745NUC driver loaded successfully!\n");

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BH1745NUC_SCU */

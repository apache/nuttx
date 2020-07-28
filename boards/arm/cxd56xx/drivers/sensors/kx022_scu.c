/****************************************************************************
 * boards/arm/cxd56xx/drivers/sensors/kx022_scu.c
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
#include <nuttx/sensors/kx022.h>
#include <nuttx/irq.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_KX022_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_KX022_SCU_DECI
#  define KX022_SEQ_TYPE SEQ_TYPE_DECI
#else
#  define KX022_SEQ_TYPE SEQ_TYPE_NORMAL
#endif

#ifdef CONFIG_SENSORS_KX122
#  define KX022_ADDR            0x1F    /* I2C Slave Address */
#  define KX022_DEVID           0x1B    /* Device ID */
#else
#  define KX022_ADDR            0x1E    /* I2C Slave Address */
#  define KX022_DEVID           0x14    /* Device ID */
#endif
#define KX022_BYTESPERSAMPLE    6
#define KX022_ELEMENTSIZE       2

/* KX022 Registers */

#define KX022_XOUT_L            0x06
#define KX022_WHO_AM_I          0x0F
#define KX022_CNTL1             0x18
#define KX022_ODCNTL            0x1B

/* Register CNTL1 */

#define KX022_CNTL1_TPE         (1 << 0)
#define KX022_CNTL1_WUFE        (1 << 1)
#define KX022_CNTL1_TDTE        (1 << 2)
#define KX022_CNTL1_GSELMASK    (0x18)
#define KX022_CNTL1_GSEL_2G     (0x00)
#define KX022_CNTL1_GSEL_4G     (0x08)
#define KX022_CNTL1_GSEL_8G     (0x10)
#define KX022_CNTL1_DRDYE       (1 << 5)
#define KX022_CNTL1_RES         (1 << 6)
#define KX022_CNTL1_PC1         (1 << 7)

/* Register ODCNTL */

#define KX022_ODCNTL_OSA_50HZ   (2)
#define KX022_ODCNTL_LPRO       (1 << 6)
#define KX022_ODCNTL_IIR_BYPASS (1 << 7)

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Structure for kx022 device */

struct kx022_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t       addr;           /* I2C address */
  int           port;           /* I2C port */

  struct seq_s *seq;            /* Sequencer instance */
  int           fifoid;         /* FIFO ID */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int kx022_open(FAR struct file *filep);
static int kx022_close(FAR struct file *filep);
static ssize_t kx022_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t kx022_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int kx022_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_kx022fops =
{
  kx022_open,                  /* open */
  kx022_close,                 /* close */
  kx022_read,                  /* read */
  kx022_write,                 /* write */
  0,                           /* seek */
  kx022_ioctl,                 /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                           /* poll */
#endif
  0                            /* unlink */
};

/* Take XYZ data. */

static const uint16_t g_kx022inst[] =
{
  SCU_INST_SEND(KX022_XOUT_L),
  SCU_INST_RECV(KX022_BYTESPERSAMPLE) | SCU_INST_LAST,
};

/* Reference count */

static int g_refcnt = 0;

/* Sequencer instance */

static struct seq_s *g_seq = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kx022_getreg8
 *
 * Description:
 *   Read from an 8-bit KX022 register
 *
 ****************************************************************************/

static uint8_t kx022_getreg8(FAR struct kx022_dev_s *priv, uint8_t regaddr)
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
 * Name: kx022_putreg8
 *
 * Description:
 *   Write to an 8-bit KX022 register
 *
 ****************************************************************************/

static void kx022_putreg8(FAR struct kx022_dev_s *priv, uint8_t regaddr,
                          uint8_t regval)
{
  uint16_t inst[2];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_SEND(regval) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 2, NULL, 0);
}

/****************************************************************************
 * Name: kx022_checkid
 *
 * Description:
 *   Read and verify the KX022 chip ID
 *
 ****************************************************************************/

static int kx022_checkid(FAR struct kx022_dev_s *priv)
{
  uint8_t devid;

  /* Read device ID */

  devid = kx022_getreg8(priv, KX022_WHO_AM_I);

  if (devid != KX022_DEVID)
    {
      /* ID is not Correct */

      snerr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: kx022_initialize
 *
 * Description:
 *   Initialize and goto stand-by mode.
 *
 ****************************************************************************/

static void kx022_initialize(FAR struct kx022_dev_s *priv)
{
  uint8_t val;

  /* CNTL1 */

  val = KX022_CNTL1_RES | KX022_CNTL1_GSEL_2G;
  kx022_putreg8(priv, KX022_CNTL1, val);

  /* ODCNTL */

  val = KX022_ODCNTL_OSA_50HZ;
  kx022_putreg8(priv, KX022_ODCNTL, val);
}

static int kx022_seqinit(FAR struct kx022_dev_s *priv)
{
  DEBUGASSERT(g_seq == NULL);

  /* Open sequencer */

  g_seq = seq_open(KX022_SEQ_TYPE, SCU_BUS_I2C0);
  if (!g_seq)
    {
      return -ENOENT;
    }

  priv->seq = g_seq;

  seq_setaddress(priv->seq, priv->addr);

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq,
                     g_kx022inst,
                     itemsof(g_kx022inst));
  seq_setsample(priv->seq,
                KX022_BYTESPERSAMPLE,
                0,
                KX022_ELEMENTSIZE,
                false);

  return OK;
}

/****************************************************************************
 * Name: kx022_open
 *
 * Description:
 *   This function is called whenever the KX022 device is opened.
 *
 ****************************************************************************/

static int kx022_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct kx022_dev_s *priv = inode->i_private;
  uint8_t val;

  if (g_refcnt == 0)
    {
      int ret;

      ret = kx022_seqinit(priv);
      if (ret < 0)
        {
          return ret;
        }

      /* goto operating mode */

      val = kx022_getreg8(priv, KX022_CNTL1);
      val |= KX022_CNTL1_PC1;
      kx022_putreg8(priv, KX022_CNTL1, val);
      up_mdelay(1);
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
 * Name: kx022_close
 *
 * Description:
 *   This routine is called when the KX022 device is closed.
 *
 ****************************************************************************/

static int kx022_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct kx022_dev_s *priv = inode->i_private;
  uint8_t val;

  g_refcnt--;

  seq_ioctl(priv->seq, priv->fifoid, SCUIOC_STOP, 0);

  if (g_refcnt == 0)
    {
      /* goto stand-by mode */

      val = kx022_getreg8(priv, KX022_CNTL1);
      val &= ~KX022_CNTL1_PC1;
      kx022_putreg8(priv, KX022_CNTL1, val);
      up_mdelay(1);

      seq_close(g_seq);
      g_seq = NULL;
    }
  else
    {
      seq_ioctl(priv->seq, priv->fifoid, SCUIOC_FREEFIFO, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: kx022_read
 ****************************************************************************/

static ssize_t kx022_read(FAR struct file *filep, FAR char *buffer,
                          size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct kx022_dev_s *priv = inode->i_private;

  len = len / KX022_BYTESPERSAMPLE * KX022_BYTESPERSAMPLE;
  len = seq_read(priv->seq, priv->fifoid, buffer, len);

  return len;
}

/****************************************************************************
 * Name: kx022_write
 ****************************************************************************/

static ssize_t kx022_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: kx022_ioctl
 ****************************************************************************/

static int kx022_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct kx022_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      default:
        {
          if (_SCUIOCVALID(cmd))
            {
              /* Redirect SCU commands */

              ret = seq_ioctl(priv->seq, priv->fifoid, cmd, arg);
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
 * Name: kx022_init
 *
 * Description:
 *   Initialize the KX022 device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             KX022
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int kx022_init(FAR struct i2c_master_s *i2c, int port)
{
  FAR struct kx022_dev_s tmp;
  FAR struct kx022_dev_s *priv = &tmp;
  int ret;

  /* Setup temporary device structure for initialization */

  priv->i2c = i2c;
  priv->addr = KX022_ADDR;
  priv->port = port;

  /* Check Device ID */

  ret = kx022_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ret;
    }

  /* Initialize KX022 */

  kx022_initialize(priv);

  return OK;
}

/****************************************************************************
 * Name: kx022_register
 *
 * Description:
 *   Register the KX022 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel0"
 *   fifoid  - FIFO ID
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             KX022
 *   port    - I2C port (0 or 1)
 *   seq     - Sequencer instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int kx022_register(FAR const char *devpath, int minor,
                   FAR struct i2c_master_s *i2c, int port)
{
  FAR struct kx022_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the KX022 device structure */

  priv = (FAR struct kx022_dev_s *)kmm_malloc(sizeof(struct kx022_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = KX022_ADDR;
  priv->port = port;
  priv->seq = NULL;
  priv->fifoid = minor;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_kx022fops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("KX022 driver loaded successfully!\n");

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_KX022_SCU */

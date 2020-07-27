/****************************************************************************
 * boards/arm/cxd56xx/drivers/sensors/ak09912_scu.c
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
#include <nuttx/sensors/ak09912.h>
#include <nuttx/irq.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_AK09912_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_AK09912_SCU_DECI
#  define MAG_SEQ_TYPE SEQ_TYPE_DECI
#else
#  define MAG_SEQ_TYPE SEQ_TYPE_NORMAL
#endif

#define AK09912_ADDR         0x0C
#define AK09912_FREQ         400000
#define AK09912_DEVID        0x0448
#define AK09911_DEVID        0x0548

/* REGISTER: WIA
 * Who I am.
 */

#define AK09912_WIA1       0x00
#define AK09912_WIA2       0x01

/* REGISTER: CNTL2
 * Power mode
 */

#define POWER_MODE_ADDR      0x31

/* REGISTER: ASAX
 * Sensitivity values
 */

#define AK09912_ASAX       0x60

/* REGISTER: CNTL1
 * Enable or disable temperature measure or enable or disable Noise
 *  suppression filter.
 */

#define AK09912_CTRL1      0x30

/* REGISTER: HXL
 * The start address of data registers.
 */

#define AK09912_HXL        0x11

/* The parameter for compensating. */

#define AK09912_SENSITIVITY               (128)
#define AK09912_SENSITIVITY_DIV           (256)

/* Noise Suppression Filter */

#define AK09912_NSF_MASK                  (0x3 << 5)
#define AK09912_NSF_NONE                  (0x0 << 5)
#define AK09912_NSF_LOW                   (0x1 << 5)
#define AK09912_NSF_MIDDLE                (0x2 << 5)
#define AK09912_NSF_HIGH                  (0x3 << 5)

/* Power mode */

#define AKM_POWER_DOWN_MODE                0b0000
#define AKM_SINGL_MEAS_MODE                0b00001
#define AKM_CONT_MEAS_1                    0b00010
#define AKM_CONT_MEAS_2                    0b00100
#define AKM_CONT_MEAS_3                    0b00110
#define AKM_CONT_MEAS_4                    0b01000
#define AKM_EXT_TRIG_MODE                  0b01010
#define AKM_SELF_TEST_MODE                 0b10000
#define AKM_FUSE_ROM_MODE                  0b11111

/* REGISTER: ST1
 * DRDY: Data ready bit. 0: not ready, 1: ready
 * DOR: Data overrun. 0: Not overrun, 1: overrun
 */

#define AK09912_ST1 0x10

/* AK09912 is magnetometer */

#define AK09912_BYTESPERSAMPLE  6
#define AK09912_ELEMENTSIZE     2

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/**
 * @brief Structure for ak09912 device
 */

struct ak09912_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t       addr;           /* I2C address */
  int           freq;           /* Frequency <= 3.4MHz */
  int           port;           /* I2C port */

  struct seq_s *seq;
  int           id;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int ak09912_open(FAR struct file *filep);
static int ak09912_close(FAR struct file *filep);
static ssize_t ak09912_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t ak09912_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int ak09912_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ak09912fops =
{
  ak09912_open,                  /* open */
  ak09912_close,                 /* close */
  ak09912_read,                  /* read */
  ak09912_write,                 /* write */
  0,                             /* seek */
  ak09912_ioctl,                 /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                             /* poll */
#endif
  0                              /* unlink */
};

/* Take XYZ data, temperature and Status 2 register.
 * Status 2 register has a role as data reading end.
 */

static const uint16_t g_ak09912inst[] =
{
  SCU_INST_SEND(AK09912_HXL),
  SCU_INST_RECV(AK09912_BYTESPERSAMPLE + 2) | SCU_INST_LAST,
};

static struct ak09912_sensadj_s g_asa;

/* Sequencer instance */

static FAR struct seq_s *g_seq = NULL;

/* Reference count */

static int g_refcnt = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ak09912_getreg8
 *
 * Description:
 *   Read from an 8-bit AK09912 register
 *
 ****************************************************************************/

static uint8_t ak09912_getreg8(FAR struct ak09912_dev_s *priv,
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
 * Name: ak09912_putreg8
 *
 * Description:
 *   Write to an 8-bit AK09912 register
 *
 ****************************************************************************/

static void ak09912_putreg8(FAR struct ak09912_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  uint16_t inst[2];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_SEND(regval) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 2, NULL, 0);
}

/****************************************************************************
 * Name: ak09912_getreg
 *
 * Description:
 *   Read cnt bytes from a ak09912 register
 *
 ****************************************************************************/

static int ak09912_getreg(FAR struct ak09912_dev_s *priv, uint8_t regaddr,
                          uint8_t *buffer, uint32_t cnt)
{
  uint16_t inst[2];

  /* Send register to read and get the next byte */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_RECV(cnt) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 2, buffer, cnt);

  return OK;
}

/****************************************************************************
 * Name: ak09912_checkid
 *
 * Description:
 *   Read and verify the AK09911/AK09912 chip ID
 *
 ****************************************************************************/

static int ak09912_checkid(FAR struct ak09912_dev_s *priv)
{
  uint16_t devid = 0;

  /* Read device ID */

  devid = ak09912_getreg8(priv, AK09912_WIA1);
  devid += ak09912_getreg8(priv, AK09912_WIA2) << 8;
  sninfo("devid: 0x%04x\n", devid);

  if (devid != AK09911_DEVID && devid != AK09912_DEVID)
    {
      /* ID is not Correct */

      snerr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

static int ak09912_seqinit(FAR struct ak09912_dev_s *priv)
{
  DEBUGASSERT(g_seq == NULL);

  /* Open sequencer */

  g_seq = seq_open(MAG_SEQ_TYPE, SCU_BUS_I2C0);
  if (!g_seq)
    {
      return -ENOENT;
    }

  priv->seq = g_seq;

  seq_setaddress(priv->seq, priv->addr);

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq, g_ak09912inst, itemsof(g_ak09912inst));
  seq_setsample(priv->seq, AK09912_BYTESPERSAMPLE, 0,
                AK09912_ELEMENTSIZE, false);

  return OK;
}

/****************************************************************************
 * Name: ak09912_open
 *
 * Description:
 *   This function is called whenever the AK09912 device is opened.
 *
 ****************************************************************************/

static int ak09912_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct ak09912_dev_s *priv  = inode->i_private;

  if (g_refcnt == 0)
    {
      int ret;

      ret = ak09912_seqinit(priv);
      if (ret < 0)
        {
          return ret;
        }

      ak09912_putreg8(priv, POWER_MODE_ADDR, AKM_CONT_MEAS_4);
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
 * Name: ak09912_close
 *
 * Description:
 *   This routine is called when the AK09912 device is closed.
 *
 ****************************************************************************/

static int ak09912_close(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct ak09912_dev_s *priv  = inode->i_private;

  g_refcnt--;

  seq_ioctl(priv->seq, priv->id, SCUIOC_STOP, 0);

  if (g_refcnt == 0)
    {
      DEBUGASSERT(g_seq);

      ak09912_putreg8(priv, POWER_MODE_ADDR, AKM_POWER_DOWN_MODE);
      up_mdelay(1);

      seq_close(g_seq);
      g_seq = NULL;
    }
  else
    {
      seq_ioctl(priv->seq, priv->id, SCUIOC_FREEFIFO, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: ak09912_read
 ****************************************************************************/

static ssize_t ak09912_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct ak09912_dev_s *priv  = inode->i_private;

  len = len / AK09912_BYTESPERSAMPLE * AK09912_BYTESPERSAMPLE;
  len = seq_read(priv->seq, priv->id, buffer, len);

  return len;
}

/****************************************************************************
 * Name: ak09912_write
 ****************************************************************************/

static ssize_t ak09912_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ak09912_write
 ****************************************************************************/

static int ak09912_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ak09912_dev_s *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      /* Get sensitivity adjustment value
       * Arg: Pointer to struct ak09912_sensadj_s
       */

      case SNIOC_GETADJ:
        {
          struct ak09912_sensadj_s *user = (struct ak09912_sensadj_s *)
                                           (uintptr_t)arg;

          user->x = g_asa.x;
          user->y = g_asa.y;
          user->z = g_asa.z;
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

int ak09912_init(FAR struct i2c_master_s *i2c, int port)
{
  struct ak09912_dev_s tmp;
  struct ak09912_dev_s *priv = &tmp;
  uint8_t val;
  int ret;

  /* Setup temporary device structure for initialization */

  priv->i2c = i2c;
  priv->addr = AK09912_ADDR;
  priv->freq = AK09912_FREQ;
  priv->port = port;

  /* Check Device ID */

  ret = ak09912_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ret;
    }

  ak09912_putreg8(priv, POWER_MODE_ADDR, AKM_FUSE_ROM_MODE);
  up_mdelay(1);

  ak09912_getreg(priv, AK09912_ASAX, (uint8_t *)&g_asa,
                 sizeof(struct ak09912_sensadj_s));

  ak09912_putreg8(priv, POWER_MODE_ADDR, AKM_POWER_DOWN_MODE);
  up_mdelay(1);

  /* Set noise suppression filter to LOW */

  val = ak09912_getreg8(priv, AK09912_CTRL1);
  val = (val & ~AK09912_NSF_MASK) | AK09912_NSF_LOW;
  ak09912_putreg8(priv, AK09912_CTRL1, val);

  return OK;
}

/****************************************************************************
 * Name: ak09912_register
 *
 * Description:
 *   Register the AK09912 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mag"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             AK09912
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ak09912_register(FAR const char *devpath, int minor,
                     FAR struct i2c_master_s *i2c, int port)
{
  FAR struct ak09912_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the AK09912 device structure */

  priv = (FAR struct ak09912_dev_s *)
          kmm_malloc(sizeof(struct ak09912_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->id = minor;
  priv->addr = AK09912_ADDR;
  priv->freq = AK09912_FREQ;
  priv->port = port;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_ak09912fops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("AK09912 driver loaded successfully!\n");
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_AK09912_SCU */

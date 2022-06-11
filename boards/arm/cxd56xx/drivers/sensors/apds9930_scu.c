/****************************************************************************
 * boards/arm/cxd56xx/drivers/sensors/apds9930_scu.c
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
#include <nuttx/sensors/apds9930.h>
#include <nuttx/irq.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_APDS9930_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APDS9930_ADDR                  0x39    /* I2C Slave Address */
#define APDS9930_DEVICEID              0x39    /* Device ID */
#define APDS9930_ALS_BYTESPERSAMPLE    4
#define APDS9930_PS_BYTESPERSAMPLE     2
#define APDS9930_ELEMENTSIZE           0

/* APDS9930 Registers */

#define APDS9930_CMD_TYPE_REPEAT       0x80
#define APDS9930_CMD_TYPE_AUTOINC      0xA0
#define APDS9930_CMD_TYPE_PSINTCLR     0xE5
#define APDS9930_ENABLE                0x00
#define APDS9930_ATIME                 0x01
#define APDS9930_PTIME                 0x02
#define APDS9930_WTIME                 0x03
#define APDS9930_AILTL                 0x04
#define APDS9930_AILTH                 0x05
#define APDS9930_AIHTL                 0x06
#define APDS9930_AIHTH                 0x07
#define APDS9930_PILTL                 0x08
#define APDS9930_PILTH                 0x09
#define APDS9930_PIHTL                 0x0A
#define APDS9930_PIHTH                 0x0B
#define APDS9930_PERS                  0x0C
#define APDS9930_CONFIG                0x0D
#define APDS9930_PPULSE                0x0E
#define APDS9930_CONTROL               0x0F
#define APDS9930_ID                    0x12
#define APDS9930_STATUS                0x13
#define APDS9930_CH0DATAL              0x14
#define APDS9930_CH0DATAH              0x15
#define APDS9930_CH1DATAL              0x16
#define APDS9930_CH1DATAH              0x17
#define APDS9930_PDATAL                0x18
#define APDS9930_PDATAH                0x19
#define APDS9930_POFFSET               0x1E

/* Register ENABLE */

#define APDS9930_ENABLE_SAI            (1 << 6)
#define APDS9930_ENABLE_PIEN           (1 << 5)
#define APDS9930_ENABLE_AIEN           (1 << 4)
#define APDS9930_ENABLE_WEN            (1 << 3)
#define APDS9930_ENABLE_PEN            (1 << 2)
#define APDS9930_ENABLE_AEN            (1 << 1)
#define APDS9930_ENABLE_PON            (1 << 0)
#define APDS9930_ENABLE_STANDBY        0x00

/* Register ATIME */

#define APDS9930_ATIME_VAL             0xff

/* Register PTIME */

#define APDS9930_PTIME_VAL             0xff

/* Register WTIME */

#define APDS9930_WTIME_VAL             0xff

/* Register PPULSE */

#define APDS9930_PPULSE_VAL            0x01

/* Register CONTROL */

#define APDS9930_CONTROL_PDRIVE_100MA  (0 << 6)
#define APDS9930_CONTROL_PDIODE_CH1    (2 << 4)
#define APDS9930_CONTROL_PGAIN_X1      (0 << 2)
#define APDS9930_CONTROL_AGAIN_X1      (0 << 0)

/* PS threshold default value */

#define APDS9930_PS_HT_DEFAULT         0x0300
#define APDS9930_PS_LT_DEFAULT         0x0300

/* Register PERS */

#define APDS9930_PERS_PS_DEFAULT       2

#define SETENABLE_TYPE_PS              0
#define SETENABLE_TYPE_ALS             1

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/**
 * @brief Structure for apds9930 device
 */

struct apds9930_dev_s
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

static int apds9930_open_als(struct file *filep);
static int apds9930_open_ps(struct file *filep);
static int apds9930_close_als(struct file *filep);
static int apds9930_close_ps(struct file *filep);
static ssize_t apds9930_read_als(struct file *filep, char *buffer,
                                 size_t buflen);
static ssize_t apds9930_read_ps(struct file *filep, char *buffer,
                                size_t buflen);
static ssize_t apds9930_write(struct file *filep, const char *buffer,
                              size_t buflen);
static int apds9930_ioctl_als(struct file *filep, int cmd,
                              unsigned long arg);
static int apds9930_ioctl_ps(struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Ambient light sensor */

static const struct file_operations g_apds9930alsfops =
{
  apds9930_open_als,           /* open */
  apds9930_close_als,          /* close */
  apds9930_read_als,           /* read */
  apds9930_write,              /* write */
  NULL,                        /* seek */
  apds9930_ioctl_als,          /* ioctl */
  NULL                         /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                       /* unlink */
#endif
};

/* Proximity sensor */

static const struct file_operations g_apds9930psfops =
{
  apds9930_open_ps,            /* open */
  apds9930_close_ps,           /* close */
  apds9930_read_ps,            /* read */
  apds9930_write,              /* write */
  NULL,                        /* seek */
  apds9930_ioctl_ps,           /* ioctl */
  NULL                         /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                       /* unlink */
#endif
};

/* SCU instructions for pick ambient light sensing data. */

static const uint16_t g_apds9930alsinst[] =
{
  SCU_INST_SEND(APDS9930_CMD_TYPE_AUTOINC | APDS9930_CH0DATAL),
  SCU_INST_RECV(APDS9930_ALS_BYTESPERSAMPLE) | SCU_INST_LAST,
};

#ifndef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
/* SCU instructions for pick proximity sensing data. */

static const uint16_t g_apds9930psinst[] =
{
  SCU_INST_SEND(APDS9930_CMD_TYPE_AUTOINC | APDS9930_PDATAL),
  SCU_INST_RECV(APDS9930_PS_BYTESPERSAMPLE) | SCU_INST_LAST,
};
#endif

/* Reference count */

static int g_als_refcnt = 0;
#ifndef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
static int g_ps_refcnt = 0;
#endif

/* Sequencer instance */

static struct seq_s *g_als_seq = NULL;
#ifndef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
static struct seq_s *g_ps_seq = NULL;
#endif

#ifdef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
/* Proximity interrupt config */

static uint16_t g_ps_lthreshold = APDS9930_PS_LT_DEFAULT;
static uint16_t g_ps_hthreshold = APDS9930_PS_HT_DEFAULT;
static uint8_t g_ps_persistence = APDS9930_PERS_PS_DEFAULT << 4;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apds9930_getreg8
 *
 * Description:
 *   Read from an 8-bit APDS9930 register
 *
 ****************************************************************************/

static uint8_t apds9930_getreg8(struct apds9930_dev_s *priv,
                                uint8_t regaddr)
{
  uint8_t regval = 0;
  uint16_t inst[2];

  /* Send register to read and get the next byte */

  inst[0] = SCU_INST_SEND(APDS9930_CMD_TYPE_REPEAT | regaddr);
  inst[1] = SCU_INST_RECV(1) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 2, &regval, 1);

  return regval;
}

/****************************************************************************
 * Name: apds9930_putreg8
 *
 * Description:
 *   Write to an 8-bit APDS9930 register
 *
 ****************************************************************************/

static void apds9930_putreg8(struct apds9930_dev_s *priv,
                             uint8_t regaddr, uint8_t regval)
{
  uint16_t inst[2];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(APDS9930_CMD_TYPE_REPEAT | regaddr);
  inst[1] = SCU_INST_SEND(regval) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 2, NULL, 0);
}

#ifdef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
/****************************************************************************
 * Name: apds9930_getreg16
 *
 * Description:
 *   Read from an 16-bit APDS9930 register
 *
 ****************************************************************************/

static uint16_t apds9930_getreg16(struct apds9930_dev_s *priv,
                                  uint8_t regaddr)
{
  uint16_t regval = 0;
  uint16_t inst[2];

  /* Send register to read and get the next byte */

  inst[0] = SCU_INST_SEND(APDS9930_CMD_TYPE_AUTOINC | regaddr);
  inst[1] = SCU_INST_RECV(2) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port,
                  priv->addr,
                  inst,
                  2,
                 (uint8_t *)&regval,
                  2);

  return regval;
}

/****************************************************************************
 * Name: apds9930_putreg16
 *
 * Description:
 *   Write to an 16-bit APDS9930 register
 *
 ****************************************************************************/

static void apds9930_putreg16(struct apds9930_dev_s *priv,
                              uint8_t regaddr, uint16_t regval)
{
  uint16_t inst[3];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(APDS9930_CMD_TYPE_AUTOINC | regaddr);
  inst[1] = SCU_INST_SEND((uint8_t)(regval & 0xff));
  inst[2] = SCU_INST_SEND((uint8_t)(regval >> 8)) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 3, NULL, 0);
}

/****************************************************************************
 * Name: apds9930_intclr
 *
 * Description:
 *   APDS9930 Interrupt clear
 *
 ****************************************************************************/

static void apds9930_intclr(struct apds9930_dev_s *priv)
{
  uint16_t inst = SCU_INST_SEND(APDS9930_CMD_TYPE_PSINTCLR) | SCU_INST_LAST;

  /* Send command */

  scu_i2ctransfer(priv->port, priv->addr, &inst, 1, NULL, 0);
}
#endif

/****************************************************************************
 * Name: apds9930_checkid
 *
 * Description:
 *   Read and verify the APDS9930 chip ID
 *
 ****************************************************************************/

static int apds9930_checkid(struct apds9930_dev_s *priv)
{
  uint8_t id;

  /* Read Device ID */

  id = apds9930_getreg8(priv, APDS9930_ID);

  if (id != APDS9930_DEVICEID)
    {
      /* Device ID is not Correct */

      snerr("Wrong Device ID! %02x (Exp:%02x)\n", id, APDS9930_DEVICEID);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: apds9930_setenable
 *
 * Description:
 *   Set MODE_CONTROL register
 *
 ****************************************************************************/

static void apds9930_setenable(struct apds9930_dev_s *priv,
                               uint8_t type, bool enable)
{
  uint8_t val;
  uint8_t checkbit;
  uint8_t setbit;
  irqstate_t flags;

  if (type == SETENABLE_TYPE_PS)
    {
      checkbit = APDS9930_ENABLE_AEN;
      setbit = APDS9930_ENABLE_PEN;
    }
  else
    {
      checkbit = APDS9930_ENABLE_PEN;
      setbit = APDS9930_ENABLE_AEN;
    }

  flags = enter_critical_section();

  val = apds9930_getreg8(priv, APDS9930_ENABLE);

  if (val & checkbit)
    {
      if (enable)
        {
          val |= setbit;
#ifdef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
          if (type == SETENABLE_TYPE_PS)
            {
              val |= APDS9930_ENABLE_PIEN;
            }
#endif
        }
      else
        {
          val &= ~setbit;
#ifdef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
          if (type == SETENABLE_TYPE_PS)
            {
              val &= ~APDS9930_ENABLE_PIEN;
            }
#endif
        }
    }
  else
    {
      if (enable)
        {
          val = APDS9930_ENABLE_PON | APDS9930_ENABLE_WEN | setbit;
#ifdef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
          if (type == SETENABLE_TYPE_PS)
            {
              val |= APDS9930_ENABLE_PIEN;
            }
#endif
        }
      else
        {
          val = APDS9930_ENABLE_STANDBY;
        }
    }

  apds9930_putreg8(priv, APDS9930_ENABLE, val);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: apds9930als_seqinit
 *
 * Description:
 *   Initialize SCU sequencer.
 *
 ****************************************************************************/

static int apds9930als_seqinit(struct apds9930_dev_s *priv)
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
                     g_apds9930alsinst,
                     itemsof(g_apds9930alsinst));
  seq_setsample(priv->seq,
                APDS9930_ALS_BYTESPERSAMPLE,
                0,
                APDS9930_ELEMENTSIZE,
                false);

  return OK;
}

#ifndef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
/****************************************************************************
 * Name: apds9930ps_seqinit
 *
 * Description:
 *   Initialize SCU sequencer.
 *
 ****************************************************************************/

static int apds9930ps_seqinit(struct apds9930_dev_s *priv)
{
  DEBUGASSERT(g_ps_seq == NULL);

  /* Open sequencer */

  g_ps_seq = seq_open(SEQ_TYPE_NORMAL, SCU_BUS_I2C0);
  if (!g_ps_seq)
    {
      return -ENOENT;
    }

  priv->seq = g_ps_seq;

  seq_setaddress(priv->seq, priv->addr);

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq,
                     g_apds9930psinst,
                     itemsof(g_apds9930psinst));
  seq_setsample(priv->seq,
                APDS9930_PS_BYTESPERSAMPLE,
                0,
                APDS9930_ELEMENTSIZE,
                false);

  return OK;
}
#endif

/****************************************************************************
 * Name: apds9930_open_als
 *
 * Description:
 *   This function is called whenever the APDS9930 device is opened.
 *
 ****************************************************************************/

static int apds9930_open_als(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct apds9930_dev_s *priv = inode->i_private;

  if (g_als_refcnt == 0)
    {
      int ret;

      ret = apds9930als_seqinit(priv);
      if (ret < 0)
        {
          return ret;
        }

      apds9930_setenable(priv, SETENABLE_TYPE_ALS, true);
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
 * Name: apds9930_open_ps
 *
 * Description:
 *   This function is called whenever the APDS9930 device is opened.
 *
 ****************************************************************************/

static int apds9930_open_ps(struct file *filep)
{
#ifndef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
  struct inode *inode = filep->f_inode;
  struct apds9930_dev_s *priv = inode->i_private;

  if (g_ps_refcnt == 0)
    {
      int ret;

      ret = apds9930ps_seqinit(priv);
      if (ret < 0)
        {
          return ret;
        }

      apds9930_setenable(priv, SETENABLE_TYPE_PS, true);
    }
  else
    {
      /* Set existing sequencer */

      priv->seq = g_ps_seq;
    }

  g_ps_refcnt++;
#endif

  return OK;
}

/****************************************************************************
 * Name: apds9930_close_als
 *
 * Description:
 *   This routine is called when the APDS9930 device is closed.
 *
 ****************************************************************************/

static int apds9930_close_als(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct apds9930_dev_s *priv = inode->i_private;

  g_als_refcnt--;

  seq_ioctl(priv->seq, priv->minor, SCUIOC_STOP, 0);

  if (g_als_refcnt == 0)
    {
      apds9930_setenable(priv, SETENABLE_TYPE_ALS, false);

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
 * Name: apds9930_close_ps
 *
 * Description:
 *   This routine is called when the APDS9930 device is closed.
 *
 ****************************************************************************/

static int apds9930_close_ps(struct file *filep)
{
#ifndef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
  struct inode *inode = filep->f_inode;
  struct apds9930_dev_s *priv = inode->i_private;

  g_ps_refcnt--;

  seq_ioctl(priv->seq, priv->minor, SCUIOC_STOP, 0);

  if (g_ps_refcnt == 0)
    {
      apds9930_setenable(priv, SETENABLE_TYPE_PS, false);

      if (g_ps_seq)
        {
          seq_close(g_ps_seq);
          g_ps_seq = NULL;
        }
    }
  else
    {
      seq_ioctl(priv->seq, priv->minor, SCUIOC_FREEFIFO, 0);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: apds9930_read_als
 ****************************************************************************/

static ssize_t apds9930_read_als(struct file *filep, char *buffer,
                                 size_t len)
{
  struct inode *inode = filep->f_inode;
  struct apds9930_dev_s *priv = inode->i_private;

  len = len / APDS9930_ALS_BYTESPERSAMPLE * APDS9930_ALS_BYTESPERSAMPLE;
  len = seq_read(priv->seq, priv->minor, buffer, len);

  return len;
}

/****************************************************************************
 * Name: apds9930_read_ps
 ****************************************************************************/

static ssize_t apds9930_read_ps(struct file *filep, char *buffer,
                                size_t len)
{
  struct inode *inode = filep->f_inode;
  struct apds9930_dev_s *priv = inode->i_private;

  len = len / APDS9930_PS_BYTESPERSAMPLE * APDS9930_PS_BYTESPERSAMPLE;

#ifdef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
  if (len)
    {
      len = APDS9930_PS_BYTESPERSAMPLE;
      *(uint16_t *)buffer = apds9930_getreg16(priv, APDS9930_PDATAL);
    }
#else
  len = seq_read(priv->seq, priv->minor, buffer, len);
#endif

  return len;
}

/****************************************************************************
 * Name: apds9930_write
 ****************************************************************************/

static ssize_t apds9930_write(struct file *filep, const char *buffer,
                              size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: apds9930_ioctl_als
 ****************************************************************************/

static int apds9930_ioctl_als(struct file *filep, int cmd,
                              unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct apds9930_dev_s *priv = inode->i_private;
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
 * Name: apds9930_ioctl_ps
 ****************************************************************************/

static int apds9930_ioctl_ps(struct file *filep, int cmd,
                             unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct apds9930_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
#ifdef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
      case SNIOC_SETPSLTHRESHOLD:
        {
          g_ps_lthreshold = (uint16_t)arg;
        }
        break;

      case SNIOC_SETPSHTHRESHOLD:
        {
          g_ps_hthreshold = (uint16_t)arg;
        }
        break;

      case SNIOC_SETPSPERSISTENCE:
        {
          g_ps_persistence = ((uint8_t)arg & 0xf) << 4;
        }
        break;

      case SNIOC_STARTPSMEASUREMENT:
        {
          apds9930_putreg16(priv, APDS9930_PILTL, g_ps_lthreshold);
          apds9930_putreg16(priv, APDS9930_PIHTL, g_ps_hthreshold);
          apds9930_putreg8(priv, APDS9930_PERS, g_ps_persistence);
          apds9930_setenable(priv, SETENABLE_TYPE_PS, true);
        }
        break;

      case SNIOC_STOPPSMEASUREMENT:
        {
          apds9930_setenable(priv, SETENABLE_TYPE_PS, false);
        }
        break;

      case SNIOC_GETINTSTATUS:
        {
          uint8_t intstatus = apds9930_getreg8(priv, APDS9930_STATUS);
          *(uint8_t *)(uintptr_t)arg = intstatus;
          sninfo("Get proximity IntStatus 0x%02x\n", intstatus);
        }
        break;

      case SNIOC_CLEARPSINT:
        {
          apds9930_intclr(priv);
        }
        break;
#endif

      default:
        {
          if (_SCUIOCVALID(cmd))
            {
#ifndef CONFIG_SENSORS_APDS9930_PROXIMITY_INTERRUPT
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
 * Name: apds9930_init
 *
 * Description:
 *   Initialize the APDS9930 device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             APDS9930
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9930_init(struct i2c_master_s *i2c, int port)
{
  struct apds9930_dev_s tmp;
  struct apds9930_dev_s *priv = &tmp;
  int ret;
  uint8_t val;

  /* Setup temporary device structure for initialization */

  priv->i2c = i2c;
  priv->addr = APDS9930_ADDR;
  priv->port = port;

  /* Check Device ID */

  ret = apds9930_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ret;
    }

  /* Disable and power down */

  val = APDS9930_ENABLE_STANDBY;
  apds9930_putreg8(priv, APDS9930_ENABLE, val);

  /* ALS ADC time */

  val = APDS9930_ATIME_VAL;
  apds9930_putreg8(priv, APDS9930_ATIME, val);

  /* PS ADC time */

  val = APDS9930_PTIME_VAL;
  apds9930_putreg8(priv, APDS9930_PTIME, val);

  /* Wait time */

  val = APDS9930_WTIME_VAL;
  apds9930_putreg8(priv, APDS9930_WTIME, val);

  /* PS Pulse count */

  val = APDS9930_PPULSE_VAL;
  apds9930_putreg8(priv, APDS9930_PPULSE, val);

  /* Control */

  val = APDS9930_CONTROL_PDRIVE_100MA | APDS9930_CONTROL_PDIODE_CH1 |
        APDS9930_CONTROL_PGAIN_X1 | APDS9930_CONTROL_AGAIN_X1;
  apds9930_putreg8(priv, APDS9930_CONTROL, val);

  return OK;
}

/****************************************************************************
 * Name: apds9930als_register
 *
 * Description:
 *   Register the APDS9930 ambient light sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             APDS9930
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9930als_register(const char *devpath, int minor,
                         struct i2c_master_s *i2c, int port)
{
  struct apds9930_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the APDS9930 device structure */

  priv = (struct apds9930_dev_s *)
    kmm_malloc(sizeof(struct apds9930_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = APDS9930_ADDR;
  priv->port = port;
  priv->seq = NULL;
  priv->minor = minor;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_apds9930alsfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: apds9930ps_register
 *
 * Description:
 *   Register the APDS9930 proximity sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/proxim0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             APDS9930
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9930ps_register(const char *devpath, int minor,
                        struct i2c_master_s *i2c, int port)
{
  struct apds9930_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the APDS9930 device structure */

  priv = (struct apds9930_dev_s *)
    kmm_malloc(sizeof(struct apds9930_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = APDS9930_ADDR;
  priv->port = port;
  priv->seq = NULL;
  priv->minor = minor;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_apds9930psfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("APDS9930 proximity sensor driver loaded successfully!\n");

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_APDS9930_SCU */

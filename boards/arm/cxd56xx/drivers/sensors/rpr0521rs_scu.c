/****************************************************************************
 * boards/arm/cxd56xx/drivers/sensors/rpr0521rs_scu.c
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
#include <nuttx/sensors/rpr0521rs.h>
#include <nuttx/irq.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_RPR0521RS_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPR0521RS_ADDR                0x38    /* I2C Slave Address */
#define RPR0521RS_MANUFACTID          0xE0    /* Manufact ID */
#define RPR0521RS_PARTID              0x0A    /* Part ID */
#define RPR0521RS_ALS_BYTESPERSAMPLE  4
#define RPR0521RS_PS_BYTESPERSAMPLE   2
#define RPR0521RS_ELEMENTSIZE         0

/* RPR0521RS Registers */

#define RPR0521RS_SYSTEM_CONTROL      0x40
#define RPR0521RS_MODE_CONTROL        0x41
#define RPR0521RS_ALS_PS_CONTROL      0x42
#define RPR0521RS_PS_CONTROL          0x43
#define RPR0521RS_PS_DATA_LSB         0x44
#define RPR0521RS_ALS_DATA0_LSB       0x46
#define RPR0521RS_INTERRUPT           0x4A
#define RPR0521RS_PS_TH_LSB           0x4B
#define RPR0521RS_PS_TL_LSB           0x4D
#define RPR0521RS_MANUFACT_ID         0x92

/* Register SYSTEM_CONTROL */

#define RPR0521RS_SYSTEM_CONTROL_INT_RESET         (1 << 6)

/* Register MODE_CONTROL */

#define RPR0521RS_MODE_CONTROL_ALS_EN              (1 << 7)
#define RPR0521RS_MODE_CONTROL_PS_EN               (1 << 6)
#define RPR0521RS_MODE_CONTROL_MEASTIME_100_100MS  (6 << 0)
#define RPR0521RS_MODE_CONTROL_MEASTIME_STANDBY    (0 << 0)

/* Register ALS_PS_CONTROL */

#define RPR0521RS_ALS_PS_CONTROL_LED_CURRENT_100MA (2 << 0)
#define RPR0521RS_ALS_PS_CONTROL_DATA1_GAIN_X1     (0 << 2)
#define RPR0521RS_ALS_PS_CONTROL_DATA0_GAIN_X1     (0 << 4)

/* Register PS_CONTROL */

#define RPR0521RS_PS_CONTROL_PS_GAINX1             (0 << 4)
#define RPR0521RS_PS_CONTROL_PS_PERSISTENCE_2      (2 << 0)

/* Register INTERRUPT */

#define RPR0521RS_INTERRUPT_PS_INT_STATUS          (1 << 7)
#define RPR0521RS_INTERRUPT_ALS_INT_STATUS         (1 << 6)
#define RPR0521RS_INTERRUPT_INT_MODE_PS_TH         (0 << 4)
#define RPR0521RS_INTERRUPT_INT_MODE_PS_HYSTERESIS (1 << 4)
#define RPR0521RS_INTERRUPT_INT_MODE_PS_OUTOFRANGE (2 << 4)
#define RPR0521RS_INTERRUPT_INT_ASSERT_KEEP_ACTIVE (0 << 3)
#define RPR0521RS_INTERRUPT_INT_LATCH_DISABLE      (1 << 2)
#define RPR0521RS_INTERRUPT_INT_TRIG_PS            (1 << 0)
#define RPR0521RS_INTERRUPT_INT_TRIG_INACTIVE      (0 << 0)

/* PS_TH default value */

#define RPR0521RS_PS_TH_DEFAULT       0x0300

#define SETMODECONTROL_TYPE_PS        0
#define SETMODECONTROL_TYPE_ALS       1

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/**
 * @brief Structure for rpr0521rs device
 */

struct rpr0521rs_dev_s
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

static int rpr0521rs_open_als(FAR struct file *filep);
static int rpr0521rs_open_ps(FAR struct file *filep);
static int rpr0521rs_close_als(FAR struct file *filep);
static int rpr0521rs_close_ps(FAR struct file *filep);
static ssize_t rpr0521rs_read_als(FAR struct file *filep,
                                  FAR char *buffer,
                                  size_t buflen);
static ssize_t rpr0521rs_read_ps(FAR struct file *filep,
                                 FAR char *buffer,
                                 size_t buflen);
static ssize_t rpr0521rs_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t buflen);
static int rpr0521rs_ioctl_als(FAR struct file *filep,
                               int cmd,
                               unsigned long arg);
static int rpr0521rs_ioctl_ps(FAR struct file *filep,
                              int cmd,
                              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Ambient light sensor */

static const struct file_operations g_rpr0521rsalsfops =
{
  rpr0521rs_open_als,          /* open */
  rpr0521rs_close_als,         /* close */
  rpr0521rs_read_als,          /* read */
  rpr0521rs_write,             /* write */
  0,                           /* seek */
  rpr0521rs_ioctl_als,         /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                           /* poll */
#endif
  0                            /* unlink */
};

/* Proximity sensor */

static const struct file_operations g_rpr0521rspsfops =
{
  rpr0521rs_open_ps,           /* open */
  rpr0521rs_close_ps,          /* close */
  rpr0521rs_read_ps,           /* read */
  rpr0521rs_write,             /* write */
  0,                           /* seek */
  rpr0521rs_ioctl_ps,          /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                           /* poll */
#endif
  0                            /* unlink */
};

/* SCU instructions for pick ambient light sensing data. */

static const uint16_t g_rpr0521rsalsinst[] =
{
  SCU_INST_SEND(RPR0521RS_ALS_DATA0_LSB),
  SCU_INST_RECV(RPR0521RS_ALS_BYTESPERSAMPLE) | SCU_INST_LAST,
};

#ifndef CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
/* SCU instructions for pick proximity sensing data. */

static const uint16_t g_rpr0521rspsinst[] =
{
  SCU_INST_SEND(RPR0521RS_PS_DATA_LSB),
  SCU_INST_RECV(RPR0521RS_PS_BYTESPERSAMPLE) | SCU_INST_LAST,
};
#endif

/* Reference count */

static int g_als_refcnt = 0;
#ifndef CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
static int g_ps_refcnt = 0;
#endif

/* Sequencer instance */

static struct seq_s *g_als_seq = NULL;
#ifndef CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
static struct seq_s *g_ps_seq = NULL;
#endif

/* Proximity interrupt config */

static uint16_t g_ps_threshold = RPR0521RS_PS_TH_DEFAULT;
static uint8_t g_ps_persistence = RPR0521RS_PS_CONTROL_PS_PERSISTENCE_2;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpr0521rs_getreg8
 *
 * Description:
 *   Read from an 8-bit RPR0521RS register
 *
 ****************************************************************************/

static uint8_t rpr0521rs_getreg8(FAR struct rpr0521rs_dev_s *priv,
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
 * Name: rpr0521rs_putreg8
 *
 * Description:
 *   Write to an 8-bit RPR0521RS register
 *
 ****************************************************************************/

static void rpr0521rs_putreg8(FAR struct rpr0521rs_dev_s *priv,
                              uint8_t regaddr, uint8_t regval)
{
  uint16_t inst[2];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_SEND(regval) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 2, NULL, 0);
}

#ifdef CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
/****************************************************************************
 * Name: rpr0521rs_getreg16
 *
 * Description:
 *   Read from an 16-bit RPR0521RS register
 *
 ****************************************************************************/

static uint16_t rpr0521rs_getreg16(FAR struct rpr0521rs_dev_s *priv,
                                   uint8_t regaddr)
{
  uint16_t regval;
  uint16_t inst[2];

  /* Send register to read and get the next byte */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_RECV(2) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port,
                  priv->addr,
                  inst,
                  2,
                 (FAR uint8_t *)&regval,
                  2);

  return regval;
}

/****************************************************************************
 * Name: rpr0521rs_putreg16
 *
 * Description:
 *   Write to an 16-bit RPR0521RS register
 *
 ****************************************************************************/

static void rpr0521rs_putreg16(FAR struct rpr0521rs_dev_s *priv,
                               uint8_t regaddr, uint16_t regval)
{
  uint16_t inst[3];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_SEND((uint8_t)(regval & 0xff));
  inst[2] = SCU_INST_SEND((uint8_t)(regval >> 8)) | SCU_INST_LAST;

  scu_i2ctransfer(priv->port, priv->addr, inst, 3, NULL, 0);
}
#endif

/****************************************************************************
 * Name: rpr0521rs_checkid
 *
 * Description:
 *   Read and verify the RPR0521RS chip ID
 *
 ****************************************************************************/

static int rpr0521rs_checkid(FAR struct rpr0521rs_dev_s *priv)
{
  uint8_t id;

  /* Read Manufact ID */

  id = rpr0521rs_getreg8(priv, RPR0521RS_MANUFACT_ID);

  if (id != RPR0521RS_MANUFACTID)
    {
      /* Manufact ID is not Correct */

      snerr("Wrong Manufact ID! %02x\n", id);
      return -ENODEV;
    }

  /* Read Part ID */

  id = rpr0521rs_getreg8(priv, RPR0521RS_SYSTEM_CONTROL);

  if ((id & 0x3f) != RPR0521RS_PARTID)
    {
      /* Part ID is not Correct */

      snerr("Wrong Part ID! %02x\n", id);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: rpr0521rs_setmodecontrol
 *
 * Description:
 *   Set MODE_CONTROL register
 *
 ****************************************************************************/

static void rpr0521rs_setmodecontrol(FAR struct rpr0521rs_dev_s *priv,
                                     uint8_t type, bool enable)
{
  uint8_t val;
  uint8_t checkbit;
  uint8_t setbit;
  irqstate_t flags;

  if (type == SETMODECONTROL_TYPE_PS)
    {
      checkbit = RPR0521RS_MODE_CONTROL_ALS_EN;
      setbit = RPR0521RS_MODE_CONTROL_PS_EN;
    }
  else
    {
      checkbit = RPR0521RS_MODE_CONTROL_PS_EN;
      setbit = RPR0521RS_MODE_CONTROL_ALS_EN;
    }

  flags = enter_critical_section();

  val = rpr0521rs_getreg8(priv, RPR0521RS_MODE_CONTROL);

  if (val & checkbit)
    {
      if (enable)
        {
          val = setbit |
                checkbit |
                RPR0521RS_MODE_CONTROL_MEASTIME_100_100MS;
        }
      else
        {
          val = checkbit | RPR0521RS_MODE_CONTROL_MEASTIME_100_100MS;
        }
    }
  else
    {
      if (enable)
        {
          val = setbit | RPR0521RS_MODE_CONTROL_MEASTIME_100_100MS;
        }
      else
        {
          val = RPR0521RS_MODE_CONTROL_MEASTIME_STANDBY;
        }
    }

  rpr0521rs_putreg8(priv, RPR0521RS_MODE_CONTROL, val);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rpr0521rsals_seqinit
 *
 * Description:
 *   Initialize SCU sequencer.
 *
 ****************************************************************************/

static int rpr0521rsals_seqinit(FAR struct rpr0521rs_dev_s *priv)
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
                     g_rpr0521rsalsinst,
                     itemsof(g_rpr0521rsalsinst));
  seq_setsample(priv->seq,
                RPR0521RS_ALS_BYTESPERSAMPLE,
                0,
                RPR0521RS_ELEMENTSIZE,
                false);

  return OK;
}

#ifndef CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
/****************************************************************************
 * Name: rpr0521rsps_seqinit
 *
 * Description:
 *   Initialize SCU sequencer.
 *
 ****************************************************************************/

static int rpr0521rsps_seqinit(FAR struct rpr0521rs_dev_s *priv)
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
                     g_rpr0521rspsinst,
                     itemsof(g_rpr0521rspsinst));
  seq_setsample(priv->seq,
                RPR0521RS_PS_BYTESPERSAMPLE,
                0,
                RPR0521RS_ELEMENTSIZE,
                false);

  return OK;
}
#endif

/****************************************************************************
 * Name: rpr0521rs_open_als
 *
 * Description:
 *   This function is called whenever the RPR0521RS device is opened.
 *
 ****************************************************************************/

static int rpr0521rs_open_als(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rpr0521rs_dev_s *priv = inode->i_private;

  if (g_als_refcnt == 0)
    {
      int ret;

      ret = rpr0521rsals_seqinit(priv);
      if (ret < 0)
        {
          return ret;
        }

      rpr0521rs_setmodecontrol(priv, SETMODECONTROL_TYPE_ALS, true);
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
 * Name: rpr0521rs_open_ps
 *
 * Description:
 *   This function is called whenever the RPR0521RS device is opened.
 *
 ****************************************************************************/

static int rpr0521rs_open_ps(FAR struct file *filep)
{
#ifndef CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
  FAR struct inode *inode = filep->f_inode;
  FAR struct rpr0521rs_dev_s *priv = inode->i_private;

  if (g_ps_refcnt == 0)
    {
      int ret;

      ret = rpr0521rsps_seqinit(priv);
      if (ret < 0)
        {
          return ret;
        }

      rpr0521rs_setmodecontrol(priv, SETMODECONTROL_TYPE_PS, true);
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
 * Name: rpr0521rs_close_als
 *
 * Description:
 *   This routine is called when the RPR0521RS device is closed.
 *
 ****************************************************************************/

static int rpr0521rs_close_als(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rpr0521rs_dev_s *priv = inode->i_private;

  g_als_refcnt--;

  seq_ioctl(priv->seq, priv->minor, SCUIOC_STOP, 0);

  if (g_als_refcnt == 0)
    {
      rpr0521rs_setmodecontrol(priv, SETMODECONTROL_TYPE_ALS, false);

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
 * Name: rpr0521rs_close_ps
 *
 * Description:
 *   This routine is called when the RPR0521RS device is closed.
 *
 ****************************************************************************/

static int rpr0521rs_close_ps(FAR struct file *filep)
{
#ifndef CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
  FAR struct inode *inode = filep->f_inode;
  FAR struct rpr0521rs_dev_s *priv = inode->i_private;

  g_ps_refcnt--;

  seq_ioctl(priv->seq, priv->minor, SCUIOC_STOP, 0);

  if (g_ps_refcnt == 0)
    {
      rpr0521rs_setmodecontrol(priv, SETMODECONTROL_TYPE_PS, false);

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
 * Name: rpr0521rs_read_als
 ****************************************************************************/

static ssize_t rpr0521rs_read_als(FAR struct file *filep, FAR char *buffer,
                                  size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rpr0521rs_dev_s *priv = inode->i_private;

  len = len / RPR0521RS_ALS_BYTESPERSAMPLE * RPR0521RS_ALS_BYTESPERSAMPLE;
  len = seq_read(priv->seq, priv->minor, buffer, len);

  return len;
}

/****************************************************************************
 * Name: rpr0521rs_read_ps
 ****************************************************************************/

static ssize_t rpr0521rs_read_ps(FAR struct file *filep, FAR char *buffer,
                                 size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rpr0521rs_dev_s *priv = inode->i_private;

  len = len / RPR0521RS_PS_BYTESPERSAMPLE * RPR0521RS_PS_BYTESPERSAMPLE;

#ifdef CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
  if (len)
    {
      len = RPR0521RS_PS_BYTESPERSAMPLE;
      *(FAR uint16_t *)buffer = rpr0521rs_getreg16(priv,
                                                   RPR0521RS_PS_DATA_LSB);
    }
#else
  len = seq_read(priv->seq, priv->minor, buffer, len);
#endif

  return len;
}

/****************************************************************************
 * Name: rpr0521rs_write
 ****************************************************************************/

static ssize_t rpr0521rs_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: rpr0521rs_ioctl_als
 ****************************************************************************/

static int rpr0521rs_ioctl_als(FAR struct file *filep,
                               int cmd,
                               unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rpr0521rs_dev_s *priv = inode->i_private;
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
 * Name: rpr0521rs_ioctl_ps
 ****************************************************************************/

static int rpr0521rs_ioctl_ps(FAR struct file *filep,
                              int cmd,
                              unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rpr0521rs_dev_s *priv = inode->i_private;
  int ret = OK;
#ifdef CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
  uint8_t val;
#endif

  switch (cmd)
    {
      case SNIOC_SETTHRESHOLD:
        {
          g_ps_threshold = (uint16_t)arg;
        }
        break;

      case SNIOC_SETPERSISTENCE:
        {
          g_ps_persistence = (uint8_t)arg;
        }
        break;

      case SNIOC_STARTMEASUREMENT:
        {
#ifdef CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
          val = RPR0521RS_PS_CONTROL_PS_GAINX1 | (g_ps_persistence & 0x0f);
          rpr0521rs_putreg8(priv, RPR0521RS_PS_CONTROL, val);

          rpr0521rs_putreg16(priv, RPR0521RS_PS_TH_LSB,
                             g_ps_threshold & 0x0fff);

          val = RPR0521RS_INTERRUPT_INT_MODE_PS_TH |
                RPR0521RS_INTERRUPT_INT_ASSERT_KEEP_ACTIVE |
                RPR0521RS_INTERRUPT_INT_LATCH_DISABLE |
                RPR0521RS_INTERRUPT_INT_TRIG_PS;
          rpr0521rs_putreg8(priv, RPR0521RS_INTERRUPT, val);

          rpr0521rs_setmodecontrol(priv, SETMODECONTROL_TYPE_PS, true);
#endif
        }
        break;

      case SNIOC_STOPMEASUREMENT:
        {
#ifdef CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
          rpr0521rs_setmodecontrol(priv, SETMODECONTROL_TYPE_PS, false);

          val = RPR0521RS_INTERRUPT_INT_TRIG_INACTIVE;
          rpr0521rs_putreg8(priv, RPR0521RS_INTERRUPT, val);

          val = RPR0521RS_SYSTEM_CONTROL_INT_RESET;
          rpr0521rs_putreg8(priv, RPR0521RS_SYSTEM_CONTROL, val);
#endif
        }
        break;

      case SNIOC_GETINTSTATUS:
        {
          FAR uint8_t intstatus = rpr0521rs_getreg8(priv,
                                                    RPR0521RS_INTERRUPT);
          *(FAR uint8_t *)(uintptr_t)arg = intstatus;
          sninfo("Get proximity IntStatus 0x%02x\n", intstatus);
        }
        break;

      default:
        {
          if (_SCUIOCVALID(cmd))
            {
#ifndef CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
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
 * Name: rpr0521rs_init
 *
 * Description:
 *   Initialize the RPR0521RS device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             RPR0521RS
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rpr0521rs_init(FAR struct i2c_master_s *i2c, int port)
{
  FAR struct rpr0521rs_dev_s tmp;
  FAR struct rpr0521rs_dev_s *priv = &tmp;
  int ret;
  uint8_t val;

  /* Setup temporary device structure for initialization */

  priv->i2c = i2c;
  priv->addr = RPR0521RS_ADDR;
  priv->port = port;

  /* Check Device ID */

  ret = rpr0521rs_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ret;
    }

  /* ALS initialize */

  val = RPR0521RS_ALS_PS_CONTROL_DATA0_GAIN_X1 |
        RPR0521RS_ALS_PS_CONTROL_DATA1_GAIN_X1 |
        RPR0521RS_ALS_PS_CONTROL_LED_CURRENT_100MA;
  rpr0521rs_putreg8(priv, RPR0521RS_ALS_PS_CONTROL, val);

  /* PS initialize */

  val = RPR0521RS_PS_CONTROL_PS_GAINX1;
  rpr0521rs_putreg8(priv, RPR0521RS_PS_CONTROL, val);

  val = RPR0521RS_INTERRUPT_INT_TRIG_INACTIVE;
  rpr0521rs_putreg8(priv, RPR0521RS_INTERRUPT, val);

  return OK;
}

/****************************************************************************
 * Name: rpr0521rsals_register
 *
 * Description:
 *   Register the RPR0521RS ambient light sensor character device as
 *   'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             RPR0521RS
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rpr0521rsals_register(FAR const char *devpath, int minor,
                          FAR struct i2c_master_s *i2c, int port)
{
  FAR struct rpr0521rs_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the RPR0521RS device structure */

  priv = (FAR struct rpr0521rs_dev_s *)
    kmm_malloc(sizeof(struct rpr0521rs_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = RPR0521RS_ADDR;
  priv->port = port;
  priv->seq = NULL;
  priv->minor = minor;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_rpr0521rsalsfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: rpr0521rsps_register
 *
 * Description:
 *   Register the RPR0521RS proximity sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/proxim0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             RPR0521RS
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rpr0521rsps_register(FAR const char *devpath, int minor,
                         FAR struct i2c_master_s *i2c, int port)
{
  FAR struct rpr0521rs_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the RPR0521RS device structure */

  priv = (FAR struct rpr0521rs_dev_s *)
    kmm_malloc(sizeof(struct rpr0521rs_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = RPR0521RS_ADDR;
  priv->port = port;
  priv->seq = NULL;
  priv->minor = minor;

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, &g_rpr0521rspsfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("RPR0521RS proximity sensor driver loaded successfully!\n");

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_RPR0521RS && CONFIG_CXD56_SCU */

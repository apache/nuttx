/****************************************************************************
 * drivers/usbmisc/stusb4500.c
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

/* Definitions, names and concept inspiered bygit push
 * https://github.com/ardnew/STUSB4500/blob/master/LICENSE
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/compiler.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/usb/stusb4500.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_STUSB4500
  #define stusb4500_err(x, ...)        _err(x, ##__VA_ARGS__)
  #define stusb4500_info(x, ...)       _info(x, ##__VA_ARGS__)
#else
  #define stusb4500_err(x, ...)        uerr(x, ##__VA_ARGS__)
  #define stusb4500_info(x, ...)       uinfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_STUSB4500_I2C_FREQUENCY
  #define CONFIG_STUSB4500_I2C_FREQUENCY 400000
#endif

/* Other macros */

#define STUSB4500_BUFFER_SIZE 128
#define STUSB4500_I2C_RETRIES  10

/* Debug */

#ifdef CONFIG_DEBUG_STUSB4500

#endif

/* Registers */
#define  STUSB4500_REG_BCD_TYPEC_REV_LOW         0x06
#define  STUSB4500_REG_BCD_TYPEC_REV_HIGH        0x07
#define  STUSB4500_REG_BCD_USBPD_REV_LOW         0x08
#define  STUSB4500_REG_BCD_USBPD_REV_HIGH        0x09
#define  STUSB4500_REG_DEVICE_CAPAB_HIGH         0x0A
#define  STUSB4500_REG_ALERT_STATUS_1            0x0B
#define  STUSB4500_REG_ALERT_STATUS_1_MASK       0x0C
#define  STUSB4500_REG_PORT_STATUS_0             0x0D
#define  STUSB4500_REG_PORT_STATUS_1             0x0E
#define  STUSB4500_REG_TYPEC_MONITORING_STATUS_0 0x0F
#define  STUSB4500_REG_TYPEC_MONITORING_STATUS_1 0x10
#define  STUSB4500_REG_CC_STATUS                 0x11
#define  STUSB4500_REG_CC_HW_FAULT_STATUS_0      0x12
#define  STUSB4500_REG_CC_HW_FAULT_STATUS_1      0x13
#define  STUSB4500_REG_PD_TYPEC_STATUS           0x14
#define  STUSB4500_REG_TYPEC_STATUS              0x15
#define  STUSB4500_REG_PRT_STATUS                0x16
#define  STUSB4500_REG_PD_COMMAND_CTRL           0x1A
#define  STUSB4500_REG_MONITORING_CTRL_0         0x20
#define  STUSB4500_REG_MONITORING_CTRL_2         0x22
#define  STUSB4500_REG_RESET_CTRL                0x23
#define  STUSB4500_REG_VBUS_DISCHARGE_TIME_CTRL  0x25
#define  STUSB4500_REG_VBUS_DISCHARGE_CTRL       0x26
#define  STUSB4500_REG_VBUS_CTRL                 0x27
#define  STUSB4500_REG_PE_FSM                    0x29
#define  STUSB4500_REG_GPIO_SW_GPIO              0x2D
#define  STUSB4500_REG_DEVICE_ID                 0x2F
#define  STUSB4500_REG_RX_HEADER_LOW             0x31
#define  STUSB4500_REG_RX_HEADER_HIGH            0x32
#define  STUSB4500_REG_RX_DATA_OBJ1_0            0x33
#define  STUSB4500_REG_RX_DATA_OBJ1_1            0x34
#define  STUSB4500_REG_RX_DATA_OBJ1_2            0x35
#define  STUSB4500_REG_RX_DATA_OBJ1_3            0x36
#define  STUSB4500_REG_RX_DATA_OBJ2_0            0x37
#define  STUSB4500_REG_RX_DATA_OBJ2_1            0x38
#define  STUSB4500_REG_RX_DATA_OBJ2_2            0x39
#define  STUSB4500_REG_RX_DATA_OBJ2_3            0x3A
#define  STUSB4500_REG_RX_DATA_OBJ3_0            0x3B
#define  STUSB4500_REG_RX_DATA_OBJ3_1            0x3C
#define  STUSB4500_REG_RX_DATA_OBJ3_2            0x3D
#define  STUSB4500_REG_RX_DATA_OBJ3_3            0x3E
#define  STUSB4500_REG_RX_DATA_OBJ4_0            0x3F
#define  STUSB4500_REG_RX_DATA_OBJ4_1            0x40
#define  STUSB4500_REG_RX_DATA_OBJ4_2            0x41
#define  STUSB4500_REG_RX_DATA_OBJ4_3            0x42
#define  STUSB4500_REG_RX_DATA_OBJ5_0            0x43
#define  STUSB4500_REG_RX_DATA_OBJ5_1            0x44
#define  STUSB4500_REG_RX_DATA_OBJ5_2            0x45
#define  STUSB4500_REG_RX_DATA_OBJ5_3            0x46
#define  STUSB4500_REG_RX_DATA_OBJ6_0            0x47
#define  STUSB4500_REG_RX_DATA_OBJ6_1            0x48
#define  STUSB4500_REG_RX_DATA_OBJ6_2            0x49
#define  STUSB4500_REG_RX_DATA_OBJ6_3            0x4A
#define  STUSB4500_REG_RX_DATA_OBJ7_0            0x4B
#define  STUSB4500_REG_RX_DATA_OBJ7_1            0x4C
#define  STUSB4500_REG_RX_DATA_OBJ7_2            0x4D
#define  STUSB4500_REG_RX_DATA_OBJ7_3            0x4E
#define  STUSB4500_REG_TX_HEADER_LOW             0x51
#define  STUSB4500_REG_TX_HEADER_HIGH            0x52
#define  STUSB4500_REG_DPM_PDO_NUMB              0x70
#define  STUSB4500_REG_DPM_SNK_PDO1_0            0x85
#define  STUSB4500_REG_DPM_SNK_PDO1_1            0x86
#define  STUSB4500_REG_DPM_SNK_PDO1_2            0x87
#define  STUSB4500_REG_DPM_SNK_PDO1_3            0x88
#define  STUSB4500_REG_DPM_SNK_PDO2_0            0x89
#define  STUSB4500_REG_DPM_SNK_PDO2_1            0x8A
#define  STUSB4500_REG_DPM_SNK_PDO2_2            0x8B
#define  STUSB4500_REG_DPM_SNK_PDO2_3            0x8C
#define  STUSB4500_REG_DPM_SNK_PDO3_0            0x8D
#define  STUSB4500_REG_DPM_SNK_PDO3_1            0x8E
#define  STUSB4500_REG_DPM_SNK_PDO3_2            0x8F
#define  STUSB4500_REG_DPM_SNK_PDO3_3            0x90
#define  STUSB4500_REG_RDO_REG_STATUS_0          0x91
#define  STUSB4500_REG_RDO_REG_STATUS_1          0x92
#define  STUSB4500_REG_RDO_REG_STATUS_2          0x93
#define  STUSB4500_REG_RDO_REG_STATUS_3          0x94

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct stusb4500_dev_s
{
  FAR struct i2c_master_s *i2c;          /* I2C interface */
  uint8_t addr;                          /* I2C address */
  mutex_t devlock;                       /* Manages exclusive access */
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

#ifdef CONFIG_DEBUG_STUSB4500

#endif
static int stusb4500_open(FAR struct file *filep);
static int stusb4500_close(FAR struct file *filep);
static ssize_t stusb4500_read(FAR struct file *, FAR char *, size_t);
static ssize_t stusb4500_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int stusb4500_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_stusb4500ops =
{
  stusb4500_open,  /* open */
  stusb4500_close, /* close */
  stusb4500_read,  /* read */
  stusb4500_write, /* write */
  NULL,            /* seek */
  stusb4500_ioctl, /* ioctl */
  NULL,            /* mmap */
  NULL,            /* truncate */
  NULL             /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stusb4500_getreg
 *
 * Description:
 *   Read from an 8-bit STUSB4500 register
 *
 * Input Parameters:
 *   priv   - pointer to STUSB4500 Private Structure
 *   reg    - register to read
 *   rxbuf  - pointer to receive data buffer
 *   len    - receive data length
 *
 * Returned Value:
 *   Returns positive register value in case of success, otherwise ERROR
 *
 ****************************************************************************/

static int stusb4500_getreg(FAR struct stusb4500_dev_s *priv, uint8_t reg,
                            uint8_t *rxbuf, uint8_t len)
{
  int ret = -EIO;
  int retries;
  struct i2c_msg_s msg[2];

  DEBUGASSERT(priv);

  msg[0].frequency = CONFIG_STUSB4500_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_STUSB4500_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = rxbuf;
  msg[1].length    = len;

  /* Perform the transfer */

  for (retries = 0; retries < STUSB4500_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret >= 0)
        {
          char buffer[STUSB4500_BUFFER_SIZE];
          int offset = snprintf(buffer, STUSB4500_BUFFER_SIZE,
                                "reg:%02X value:%02X", reg, rxbuf[0]);

          for (int rxbytes = 1; rxbytes < len && offset <
               STUSB4500_BUFFER_SIZE; rxbytes++)
            {
              offset += snprintf(buffer + offset, STUSB4500_BUFFER_SIZE -
                                 offset, " %02X", rxbuf[rxbytes]);
            }

          snprintf(buffer + offset, STUSB4500_BUFFER_SIZE - offset, "\n");

          stusb4500_info("%s", buffer);
          ret = OK;
          break;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == STUSB4500_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              stusb4500_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  return ret;
}

/****************************************************************************
 * Name: stusb4500_putreg
 *
 * Description:
 *   Write a value to an 8-bit STUSB4500 register
 *
 * Input Parameters:
 *   priv    - pointer to STUSB4500 Private Structure
 *   regaddr - register to read
 *   regval  - value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int stusb4500_putreg(FAR struct stusb4500_dev_s *priv,
                            uint8_t regaddr, uint8_t regval)
{
  int ret = -EIO;
  int retries;
  struct i2c_msg_s msg;
  uint8_t txbuffer[2];

  /* Setup to the data to be transferred (register address and data). */

  txbuffer[0]   = regaddr;
  txbuffer[1]   = regval;

  /* Setup 8-bit STUSB4500 address write message */

  msg.frequency = CONFIG_STUSB4500_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = 2;

  /* Perform the transfer */

  for (retries = 0; retries < STUSB4500_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->i2c, &msg, 1);
      if (ret == OK)
        {
          stusb4500_info("reg:%02X, value:%02X\n", regaddr, regval);

          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == STUSB4500_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              stusb4500_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  stusb4500_err("ERROR: failed reg:%02X, value:%02X, error:%d\n",
                regaddr, regval, ret);
  return ret;
}

/****************************************************************************
 * Name: stusb4500_read_device_id
 *
 * Description:
 *   Read device ID.
 *
 ****************************************************************************/

static int stusb4500_read_device_id(FAR struct stusb4500_dev_s *priv,
                                    FAR uint8_t *dev_id)
{
  int ret = -EIO;

  ret = stusb4500_getreg(priv, STUSB4500_REG_DEVICE_ID, dev_id, 1);
  if (ret < 0)
    {
      stusb4500_err("ERROR: Failed to read device ID\n");
    }

  return ret;
}

/****************************************************************************
 * Name: stusb4500_open
 *
 * Description:
 *   This function is called whenever the STUSB4500 device is opened.
 *
 ****************************************************************************/

static int stusb4500_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct stusb4500_dev_s *priv = inode->i_private;
  uint8_t dev_id;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Probe device */

  ret = stusb4500_read_device_id(priv, &dev_id);
  if (ret < 0)
    {
      stusb4500_err("ERROR: No response at given address 0x%02X\n",
                    priv->addr);
      ret = -EFAULT;
    }
  else
    {
      stusb4500_info("device id: 0x%02X", dev_id);
    }

  /* Clear all interrupts by reading from  */

  uint8_t foo[10];
  stusb4500_getreg(priv, STUSB4500_REG_PORT_STATUS_0, &foo[0], 10);

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: stusb4500_close
 *
 * Description:
 *   This routine is called when the STUSB4500 device is closed.
 *
 ****************************************************************************/

static int stusb4500_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct stusb4500_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* add closing code here */

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: stusb4500_read
 *
 * Description:
 *   This routine is called when the STUSB4500 device is read.
 *
 ****************************************************************************/

static ssize_t stusb4500_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: stusb4500_write
 *
 * Description:
 *   This routine is called when the STUSB4500 device is written to.
 *
 ****************************************************************************/

static ssize_t stusb4500_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  ssize_t length = 0;

  return length;
}

/****************************************************************************
 * Name: stusb4500_ioctl
 *
 * Description:
 *   This routine is called when ioctl function call is performed for
 *   the STUSB4500 device.
 *
 ****************************************************************************/

static int stusb4500_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct stusb4500_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  stusb4500_info("cmd: 0x%02X, arg:%lu\n", cmd, arg);

  switch (cmd)
    {
    case USBCIOC_READ_DEVID:
    {
      ret = stusb4500_read_device_id(priv, (uint8_t *)arg);
      break;
    }

    case USBCIOC_READ_PD_STATUS:
    {
      STUSB_GEN1S_RDO_REG_STATUS_REGTYPEDEF *status =
        (STUSB_GEN1S_RDO_REG_STATUS_REGTYPEDEF *)arg;

      /* Read PD status */

      stusb4500_getreg(priv, STUSB4500_REG_RDO_REG_STATUS_0,
                       &status->bytes[0], 4);
      break;
    }

    case USBCIOC_SET_PWR:
    {
      USB_PD_SNK_PDO_TYPEDEF *pdo = (USB_PD_SNK_PDO_TYPEDEF *)arg;

      /* switch active PDO to slot 2 */

      stusb4500_putreg(priv, STUSB4500_REG_DPM_PDO_NUMB,
                       2U);

      /* Download negotiation contract */

      ret = stusb4500_putreg(priv, STUSB4500_REG_DPM_SNK_PDO2_0,
                             pdo->bytes[0]);
      ret = stusb4500_putreg(priv, STUSB4500_REG_DPM_SNK_PDO2_1,
                             pdo->bytes[1]);
      ret = stusb4500_putreg(priv, STUSB4500_REG_DPM_SNK_PDO2_2,
                             pdo->bytes[2]);
      ret = stusb4500_putreg(priv, STUSB4500_REG_DPM_SNK_PDO2_3,
                             pdo->bytes[3]);

      /* Trigger negotiation by soft reset */

      stusb4500_putreg(priv, STUSB4500_REG_TX_HEADER_LOW,
                       0x0d);
      stusb4500_putreg(priv, STUSB4500_REG_PD_COMMAND_CTRL,
                       0x26);
      break;
    }

    case USBCIOC_READ_PWR:
    {
      USB_PD_SNK_PDO_TYPEDEF *pdo = (USB_PD_SNK_PDO_TYPEDEF *)arg;

      /* Read current negotiation contract */

      stusb4500_getreg(priv, STUSB4500_REG_DPM_SNK_PDO2_0,
                       &pdo->bytes[0], 4);
      break;
    }

    default:
    {
      stusb4500_err("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stusb4500_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr)
{
  FAR struct stusb4500_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL);

  /* Initialize the STUSB4500 device structure */

  priv = (FAR struct stusb4500_dev_s *)
         kmm_zalloc(sizeof(struct stusb4500_dev_s));
  if (!priv)
    {
      stusb4500_err("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Initialize device structure mutex */

  nxmutex_init(&priv->devlock);

  priv->i2c         = i2c;
  priv->addr        = addr;

  /* Register the character driver */

  ret = register_driver(devpath, &g_stusb4500ops, 0666, priv);
  if (ret < 0)
    {
      stusb4500_err("ERROR: Failed to register driver: %d\n", ret);
      goto errout_with_priv;
    }

  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->devlock);
  kmm_free(priv);
  return ret;
}

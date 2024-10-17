/****************************************************************************
 * drivers/wireless/lpwan/rn2483/rn2483.c
 *
 * Contributed by Carleton University InSpace
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <nuttx/wireless/lpwan/rn2483.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Frequency for communication in Hz */

#ifndef CONFIG_LPWAN_RN2483_FREQ
#define CONFIG_LPWAN_RN2483_FREQ 433050000
#endif /* CONFIG_LPWAN_RN2483_FREQ */

/* Sync word */

#ifndef CONFIG_LPWAN_RN2483_SYNC
#define CONFIG_LPWAN_RN2483_SYNC 0x43
#endif /* CONFIG_LPWAN_RN2483_SYNC */

/* Preamble length */

#ifndef CONFIG_LPWAN_RN2483_PRLEN
#define CONFIG_LPWAN_RN2483_PRLEN 6
#endif /* CONFIG_LPWAN_RN2483_PRLEN */

/* Bandwidth in kHz */

#ifndef CONFIG_LPWAN_RN2483_BW
#define CONFIG_LPWAN_RN2483_BW 500
#endif /* CONFIG_LPWAN_RN2483_BW */

#if (CONFIG_LPWAN_RN2483_BW != 125 && CONFIG_LPWAN_RN2483_BW != 250 &&       \
     CONFIG_LPWAN_RN2483_BW != 500)
#error "RN2483 bandwidth can only be one of 125, 250 and 500kHz"
#endif /* Bandwidth value check */

/* Spread factor */

#ifndef CONFIG_LPWAN_RN2483_SPREAD
#define CONFIG_LPWAN_RN2483_SPREAD 7
#endif /* CONFIG_LPWAN_RN2483_SPREAD */

#if !(CONFIG_LPWAN_RN2483_SPREAD <= 12 && CONFIG_LPWAN_RN2483_SPREAD >= 7)
#error "RN2483 spread factor must be between 7 and 12, inclusive."
#endif /* Power value check */

/* Modulation in LoRA mode. If not LoRa, FSK is selected. */

#ifndef CONFIG_LPWAN_RN2483_LORA
#define CONFIG_LPWAN_RN2483_LORA 1
#endif /* CONFIG_LPWAN_RN2483_LORA */

/* Transmit output power. TODO: what unit? */

#ifndef CONFIG_LPWAN_RN2483_POWER
#define CONFIG_LPWAN_RN2483_POWER 15
#endif /* CONFIG_LPWAN_RN2483_POWER */

#if !(CONFIG_LPWAN_RN2483_POWER <= 15 && CONFIG_LPWAN_RN2483_POWER >= -3)
#error "RN2483 power must be between -3 and 15, inclusive."
#endif /* Power value check */

/* Coding rate */

#ifndef CONFIG_LPWAN_RN2483_CR
#define CONFIG_LPWAN_RN2483_CR 7
#endif /* CONFIG_LPWAN_RN2483_CR */

#if !(CONFIG_LPWAN_RN2483_CR <= 8 && CONFIG_LPWAN_RN2483_CR >= 5)
#error "RN2483 coding rate must be between 5 and 8, inclusive."
#endif /* Power value check */

/* IQ invert */

#ifndef CONFIG_LPWAN_RN2483_IQI
#define CONFIG_LPWAN_RN2483_IQI 0
#endif /* CONFIG_LPWAN_RN2483_IQI */

/* Cyclic redundancy check */

#ifndef CONFIG_LPWAN_RN2483_CRC
#define CONFIG_LPWAN_RN2483_CRC 1
#endif /* CONFIG_LPWAN_RN2483_CRC */

/****************************************************************************
 * Private
 ****************************************************************************/

/* Contains configuration parameters for RN2483. */

struct rn2483_config_s
{
  enum rn2483_cr_e cr;   /* Coding rate */
  enum rn2483_mod_e mod; /* Modulation, either FSK or LoRa */
  uint32_t freq;         /* TX/RX frequency in Hz */
  uint16_t bw;           /* Bandwidth in kHz */
  uint16_t prlen;        /* Preamble length */
  bool crc;              /* Cyclic redundancy check, true for enabled */
  bool iqi;              /* IQ invert, true for enabled */
  int8_t pwr;            /* Transmit output power */
  uint8_t sf;            /* Spread factor */
  uint8_t sync;          /* Synchronization word */
};

/* Represents the RN2483 device */

struct rn2483_dev_s
{
  FAR struct file uart; /* Underlying UART interface */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;                 /* True means driver is unlinked */
#endif                           /* CONFIG_DISABLE_PSEUDOFS_OPERATIONS */
  struct rn2483_config_s config; /* Radio parameters */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs; /* Number of open references. */
#endif           // CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  mutex_t devlock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

// TODO: add granularity for enabling/disabling TX/RX support separately

static int rn2483_open(FAR struct file *filep);
static int rn2483_close(FAR struct file *filep);
static ssize_t rn2483_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static ssize_t rn2483_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static int rn2483_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int rn2483_unlink(FAR struct inode *inode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_rn2483fops = {
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    .open = rn2483_open,
    .close = rn2483_close,
#else
    .open = NULL,
    .close = NULL,
#endif
    .read = rn2483_read,
    .write = rn2483_write,
    .seek = NULL,
    .ioctl = rn2483_ioctl,
    .mmap = NULL,
    .truncate = NULL,
    .poll = NULL,
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    .unlink = rn2483_unlink,
#endif
};

/* String representation of modulation */

static const char *MODULATIONS[] = {
    [RN2483_MOD_FSK] = "fsk",
    [RN2483_MOD_LORA] = "lora",
};

/* String representation of coding rates */

static const char *CODING_RATES[] = {
    [RN2483_CR_4_5] = "4/5",
    [RN2483_CR_4_6] = "4/6",
    [RN2483_CR_4_7] = "4/7",
    [RN2483_CR_4_8] = "4/8",
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rn2483_open
 *
 * Description:
 *   This function is called whenever the RN2483 device is opened.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rn2483_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rn2483_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  nxmutex_unlock(&priv->devlock);
  return 0;
}
#endif

/****************************************************************************
 * Name: rn2483_close
 *
 * Description:
 *   This routine is called when the RN2483 device is closed.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rn2483_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rn2483_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then free memory now.
   */

  if (priv->crefs <= 0 && priv->unlinked)
    {
      file_close(&priv->uart);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return 0;
    }

  nxmutex_unlock(&priv->devlock);
  return 0;
}
#endif

/****************************************************************************
 * Name: rn2483_read
 *
 * Description:
 *     Character driver interface to RN2483 for receiving data.
 *
 ****************************************************************************/

static ssize_t rn2483_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rn2483_dev_s *priv = inode->i_private;
  ssize_t length = 0;
  int err;

  /* If file position is non-zero, then we're at the end of file. */

  if (filep->f_pos > 0)
    {
      return 0;
    }

  /* Get exclusive access */

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked drivers. */

      nxmutex_unlock(&priv->devlock);
      return 0;
    }
#endif

  // TODO actually receive

  // For now just get the radio version info
  char command[] = "sys get ver\r\n";
  length = file_write(&priv->uart, command, sizeof(command));
  if (length < 0)
    {
      // TODO: log error
      nxmutex_unlock(&priv->devlock);
      return length;
    }

  // Read back the version string
  length = file_read(&priv->uart, buffer, buflen);

  filep->f_pos += length;
  nxmutex_unlock(&priv->devlock);
  return length;
}

/****************************************************************************
 * Name: rn2483_write
 *
 * Description:
 *     Write interface for transmitting over the RN2483.
 ****************************************************************************/

static ssize_t rn2483_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  // TODO actually transmit
  return -ENOSYS;
}

/****************************************************************************
 * Name: rn2483_ioctl
 ****************************************************************************/

static int rn2483_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rn2483_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked drivers. */

      nxmutex_unlock(&priv->devlock);
      return -ENODEV;
    }
#endif

  /* Handle command */
  switch (cmd)
    {
      // TODO: real commands
    default:
      err = -EINVAL;
      break;
    }

  nxmutex_unlock(&priv->devlock);
  return err;
}

/****************************************************************************
 * Name: rn2483_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rn2483_unlink(FAR struct inode *inode)
{
  FAR struct rn2483_dev_s *priv;
  int err;

  DEBUGASSERT(inode->i_private != NULL);
  priv = inode->i_private;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return 0;
    }

  /* No. Just mark the driver as unlinked and free the resources when
   * the last client closes their reference to the driver.
   */

  priv->unlinked = true;
  nxmutex_unlock(&priv->devlock);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rn2483_register
 *
 * Description:
 *   Register the RN2483 LoRa driver.
 *
 ****************************************************************************/

int rn2483_register(FAR const char *devpath, FAR const char *uartpath)
{

  FAR struct rn2483_dev_s *priv = NULL;
  int err = 0;

  DEBUGASSERT(uartpath != NULL);

  /* Initialize the device structure */

  priv = kmm_zalloc(sizeof(struct rn2483_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance.\n");
      return -ENOMEM;
    }

  /* Initialize mutex */

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      // TODO: what logging macro needs to be used
      // snerr("ERROR: Failed to register SHT4X driver: %d\n", err);
      kmm_free(priv);
      return err;
    }

  /* Get access to underlying UART interface */

  err = file_open(&priv->uart, uartpath, O_RDWR | O_CLOEXEC);
  if (err < 0)
    {
      // TODO log error
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return err;
    }

    /* Set configuration options. */

#if CONFIG_LPWAN_RN2483_CR == 5
  priv->config.cr = RN2483_CR_4_5;
#elif CONFIG_LPWAN_RN2483_CR == 6
  priv->config.cr = RN2483_CR_4_6;
#elif CONFIG_LPWAN_RN2483_CR == 7
  priv->config.cr = RN2483_CR_4_7;
#elif CONFIG_LPWAN_RN2483_CR == 8
  priv->config.cr = RN2483_CR_4_8;
#endif

#if CONFIG_LPWAN_RN2483_LORA
  priv->config.mod = RN2483_MOD_LORA;
#else
  priv->config.mod = RN2483_MOD_FSK;
#endif

  // TODO: error check this value
  priv->config.freq = CONFIG_LPWAN_RN2483_FREQ;

  priv->config.bw = CONFIG_LPWAN_RN2483_BW;
  priv->config.prlen = CONFIG_LPWAN_RN2483_PRLEN;
  priv->config.iqi = CONFIG_LPWAN_RN2483_IQI;
  priv->config.crc = CONFIG_LPWAN_RN2483_CRC;
  priv->config.pwr = CONFIG_LPWAN_RN2483_POWER;
  priv->config.sf = CONFIG_LPWAN_RN2483_SPREAD;
  priv->config.sync = CONFIG_LPWAN_RN2483_SYNC;

  /* Register the character driver */

  err = register_driver(devpath, &g_rn2483fops, 0666, priv);
  if (err < 0)
    {
      // TODO: what logging macro?
      // snerr("ERROR: Failed to register RN2483 driver: %d\n", err);
      file_close(&priv->uart);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
    }

  return err;
}

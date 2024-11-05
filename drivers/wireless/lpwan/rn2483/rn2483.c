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

#include <syslog.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <nuttx/wireless/lpwan/rn2483.h>
#include <nuttx/wireless/lpwan/sx127x.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Frequency for communication in Hz */

#ifndef CONFIG_LPWAN_RN2483_FREQ
#define CONFIG_LPWAN_RN2483_FREQ 433050000
#endif /* CONFIG_LPWAN_RN2483_FREQ */

#if (CONFIG_LPWAN_RN2483_FREQ < 433000000 ||                                           \
     (CONFIG_LPWAN_RN2483_FREQ > 434800000 && CONFIG_LPWAN_RN2483_FREQ < 863000000) || \
     CONFIG_LPWAN_RN2483_FREQ > 870000000)
#error "RN2483 frequency has to be between 433000000 and 434800000, or between 863000000 and 870000000, in Hz"
#endif /* Bandwidth value check */

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

#if (CONFIG_LPWAN_RN2483_BW != 125 && CONFIG_LPWAN_RN2483_BW != 250 && \
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

/*Represents the set commands for the RN2483 device*/
typedef enum
{
  RN2483_SET_CR,
  RN2483_SET_MOD,
  RN2483_SET_FREQ,
  RN2483_SET_BW,
  RN2483_SET_PRLEN,
  RN2483_SET_CRC,
  RN2483_SET_IQI,
  RN2483_SET_PWR,
  RN2483_SET_SF,
  RN2483_SET_SYNC,
} rn2483_set_params;

/*Represents the ioctl commands for the RN2483 device*/
typedef enum
{
  RN2483_IOCTL_SET_FREQ,

} rn2483_ioctl_cmd;

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
static int rn2483_radio_set_cmd(FAR struct rn2483_dev_s *priv, rn2483_set_params option, void *data);

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

/* String representation of set options */
static const char *RN2483_SET_OPTIONS[] = {
    [RN2483_SET_CR] = "cr",
    [RN2483_SET_MOD] = "mod",
    [RN2483_SET_FREQ] = "freq",
    [RN2483_SET_BW] = "bw",
    [RN2483_SET_PRLEN] = "prlen",
    [RN2483_SET_CRC] = "crc",
    [RN2483_SET_IQI] = "iqi",
    [RN2483_SET_PWR] = "pwr",
    [RN2483_SET_SF] = "sf",
    [RN2483_SET_SYNC] = "sync",
};

#define SUCCESS "SUCCESS"
#define FAILURE "FAILURE"

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
  syslog(LOG_INFO, "Enter read\n");

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
{ // // TODO actually transmit

  FAR struct inode *inode = filep->f_inode;
  FAR struct rn2483_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
  {
    return err;
  }

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

  case RN2483_IOCTL_SET_FREQ:
  {
    FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
    err = rn2483_radio_set_cmd(priv, RN2483_SET_FREQ, ptr);
    if (err < 0)
    {
      return err;
    }
    break;
  }

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
 * Name: rn2483_radio_set_cmd
 *
 * Description:
 *     Transmit set commands to radio board.
 ****************************************************************************/

static int rn2483_radio_set_cmd(FAR struct rn2483_dev_s *priv, rn2483_set_params cmd, void *data)
{
  syslog(LOG_INFO, "SET: %s", RN2483_SET_OPTIONS[cmd]);

  char write_buffer[50];
  ssize_t written;

  switch (cmd)
  {
  case RN2483_SET_CR:
  {
    written = snprintf(write_buffer, sizeof(write_buffer), "radio set cr %s\r\n", CODING_RATES[*(enum rn2483_cr_e *)data]);
    syslog(LOG_INFO, "%s", write_buffer);
    // syslog(LOG_INFO, "Write Length: %d\n", written);
    break;
  }
  case RN2483_SET_MOD:
  {
    written = snprintf(write_buffer, sizeof(write_buffer), "radio set mod %s\r\n", MODULATIONS[*(enum rn2483_mod_e *)data]);
    syslog(LOG_INFO, "%s", write_buffer);
    // syslog(LOG_INFO, "Write Length: %d\n", written);
    break;
  }
  case RN2483_SET_FREQ:
  {
    written = snprintf(write_buffer, sizeof(write_buffer), "radio set freq %lu\r\n", *(uint32_t *)data);
    syslog(LOG_INFO, "%s", write_buffer);
    // syslog(LOG_INFO, "Write Length: %d\n", written);
    break;
  }
  case RN2483_SET_BW:
  {
    written = snprintf(write_buffer, sizeof(write_buffer), "radio set bw %hu\r\n", *(uint16_t *)data);
    syslog(LOG_INFO, "%s", write_buffer);
    // syslog(LOG_INFO, "Write Length: %d\n", written);
    break;
  }
  case RN2483_SET_PRLEN:
  {
    written = snprintf(write_buffer, sizeof(write_buffer), "radio set prlen %hu\r\n", *(uint16_t *)data);
    syslog(LOG_INFO, "%s", write_buffer);
    // syslog(LOG_INFO, "Write Length: %d\n", written);
    break;
  }
  case RN2483_SET_CRC:
  {
    char *crc = (*(bool *)data) ? "on" : "off";
    written = snprintf(write_buffer, sizeof(write_buffer), "radio set crc %s\r\n", crc);
    syslog(LOG_INFO, "%s", write_buffer);
    // syslog(LOG_INFO, "Write Length: %d\n", written);
    break;
  }
  case RN2483_SET_IQI:
  {
    char *iqi = (*(bool *)data) ? "on" : "off";
    written = snprintf(write_buffer, sizeof(write_buffer), "radio set iqi %s\r\n", iqi);
    syslog(LOG_INFO, "%s", write_buffer);
    // syslog(LOG_INFO, "Write Length: %d\n", written);
    break;
  }
  case RN2483_SET_PWR:
  {
    written = snprintf(write_buffer, sizeof(write_buffer), "radio set pwr %d\r\n", *(int8_t *)data);
    syslog(LOG_INFO, "%s", write_buffer);
    // syslog(LOG_INFO, "Write Length: %d\n", written);
    break;
  }
  case RN2483_SET_SF:
  {
    written = snprintf(write_buffer, sizeof(write_buffer), "radio set sf sf%d\r\n", *(uint8_t *)data);
    syslog(LOG_INFO, "%s", write_buffer);
    // syslog(LOG_INFO, "Write Length: %d\n", written);
    break;
  }
  case RN2483_SET_SYNC:
  {
    written = snprintf(write_buffer, sizeof(write_buffer), "radio set sync %d\r\n", *(uint8_t *)data);
    syslog(LOG_INFO, "%s", write_buffer);
    // syslog(LOG_INFO, "Write Length: %d\n", written);
    break;
  }
  default:
  {
    // Not a valid set command
    syslog(LOG_INFO, "INVALID COMMAND GIVEN %d", cmd);
    return -EINVAL;
  }
  }

  ssize_t write_length = file_write(&priv->uart, write_buffer, written);
  if (write_length < 0)
  {
    return write_length;
  }

  // Read
  int buf_len = 50;
  char read_buffer[buf_len];
  size_t read = 0;
  ssize_t fread;
  char last = '\0';

  // Read data into buffer until \r\n terminating sequence reached
  while (read < buf_len)
  {
    fread = file_read(&priv->uart, &read_buffer[read], 1);
    if (last == '\r' && read_buffer[read] == '\n')
    {
      break;
    }
    last = read_buffer[read];

    if (fread < 0)
      return fread;

    read++;
  }

  // syslog(LOG_INFO, "%s", read_buffer);

  if (strstr(read_buffer, "ok") != NULL)
  {
    syslog(LOG_INFO, "%s\n\n", SUCCESS);
    return F_OK;
  }
  else if (strstr(read_buffer, "invalid_param") != NULL)
  {
    syslog(LOG_INFO, "%s\n\n", FAILURE);
    return -EINVAL;
  }
  else
  {
    syslog(LOG_INFO, "INVALID VALUE (Not 'ok' or 'invalid_param')\n\n", FAILURE);
    return -EINVAL;
  }
}

int rn2483_set_config(FAR struct rn2483_dev_s *priv)
{
  int err;

  /*  Set freq (Frequency in Hz)*/
  err = rn2483_radio_set_cmd(priv, RN2483_SET_FREQ, &priv->config.freq);
  if (err < 0)
  {
    return err;
  }

  /*  Set cr*/
  err = rn2483_radio_set_cmd(priv, RN2483_SET_CR, &priv->config.cr);
  if (err < 0)
  {
    return err;
  }

  /*  Set bw (Bandwidth in kHz)*/
  err = rn2483_radio_set_cmd(priv, RN2483_SET_BW, &priv->config.bw);
  if (err < 0)
  {
    return err;
  }

  /*  Set prlen (Preamble length)*/
  err = rn2483_radio_set_cmd(priv, RN2483_SET_PRLEN, &priv->config.prlen);
  if (err < 0)
  {
    return err;
  }

  /*  Set iqi (IQ invert, true for enabled)*/
  err = rn2483_radio_set_cmd(priv, RN2483_SET_IQI, &priv->config.iqi);
  if (err < 0)
  {
    return err;
  }

  /*  Set crc (Cyclic redundancy check, true for enabled)*/
  err = rn2483_radio_set_cmd(priv, RN2483_SET_CRC, &priv->config.crc);
  if (err < 0)
  {
    return err;
  }

  /*  Set pwr (Transmit output power)*/
  err = rn2483_radio_set_cmd(priv, RN2483_SET_PWR, &priv->config.pwr);
  if (err < 0)
  {
    return err;
  }

  /*  Set sf (Spread Factor)*/
  err = rn2483_radio_set_cmd(priv, RN2483_SET_SF, &priv->config.sf);
  if (err < 0)
  {
    return err;
  }

  /*  Set sync (Synchronization word)*/
  err = rn2483_radio_set_cmd(priv, RN2483_SET_SYNC, &priv->config.sync);
  if (err < 0)
  {
    return err;
  }

  /*  Set mod (Modulation: FSK or LoRa)*/
  err = rn2483_radio_set_cmd(priv, RN2483_SET_MOD, &priv->config.mod);
  if (err < 0)
  {
    return err;
  }

  return F_OK;
}

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
  syslog(LOG_INFO, "Enter Register\n\n");

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

  priv->config.freq = CONFIG_LPWAN_RN2483_FREQ;

  // Error check of frequency value (Not sure if this is what you want Matteo)
  assert((priv->config.freq >= 433000000 && priv->config.freq <= 434800000) || (priv->config.freq >= 863000000 && priv->config.freq <= 870000000));

  priv->config.bw = CONFIG_LPWAN_RN2483_BW;
  priv->config.prlen = CONFIG_LPWAN_RN2483_PRLEN;
  priv->config.iqi = CONFIG_LPWAN_RN2483_IQI;
  priv->config.crc = CONFIG_LPWAN_RN2483_CRC;
  priv->config.pwr = CONFIG_LPWAN_RN2483_POWER;
  priv->config.sf = CONFIG_LPWAN_RN2483_SPREAD;
  priv->config.sync = CONFIG_LPWAN_RN2483_SYNC;

  // Dummy read to get rid of version string
  int buf_len = 50;
  char read_buffer[buf_len];
  size_t read = 0;
  ssize_t fread;
  char last = '\0';

  while (read < buf_len)
  {
    fread = file_read(&priv->uart, &read_buffer[read], 1);
    if (last == '\r' && read_buffer[read] == '\n')
    {
      break;
    }
    last = read_buffer[read];

    if (fread < 0)
      return fread;
    read++;
  }

  // Set configuration parameters on the radio via commands
  rn2483_set_config(priv);

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
/****************************************************************************
 * drivers/wireless/lpwan/sx127x/sx127x.c
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
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <poll.h>
#include <debug.h>
#include <time.h>
#include <fcntl.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

#include <nuttx/wireless/lpwan/sx127x.h>
#include "sx127x.h"

/* TODO:
 *   - OOK communication (RX+TX) doesn't work yet
 *   - Channel Activity Detection (CAD) for LORA
 *   - frequency hopping for LORA and FSK/OOK
 *   - modulation shaping for FSK/OOK
 *   - support for long payload for FSK/OOK (len > FIFO size)
 *   - address filtering for FSK/OOK
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_SCHED_HPWORK)
#  error SX127X requires CONFIG_SCHED_HPWORK
#endif

/* Configuration ************************************************************/

/* Device name */

#define SX127X_DEV_NAME               "/dev/sx127x"

/* Payload fixlen default */

#define SX127X_RX_FIXLEN_DEFAULT      (0xff)

/* Calibration frequency */

#define SX127X_FREQ_CALIBRATION       (CONFIG_LPWAN_SX127X_RFFREQ_DEFAULT)

/* FSK default frequency deviation is 5kHz */

#define SX127X_FDEV_DEFAULT           (5000)

/* FSK/OOK bitrate default */

#define SX127X_FOM_BITRATE_DEFAULT    (4800)

/* FSK/OOK bandwidth default */

#define SX127X_FSKOOK_RXBW_DEFAULT    FSKOOK_BANDWIDTH_15P6KHZ
#define SX127X_FSKOOK_AFCBW_DEFAULT   FSKOOK_BANDWIDTH_20P8KHZ

/* Default LORA bandwidth */

#define SX127X_LRM_BW_DEFAULT         LORA_BANDWIDTH_7P8KHZ

/* Default SF for LORA */

#define SX127X_LRM_SF_DEFAULT         (7)

/* FSK/OOK RX/TX FIFO size (two separate FIFOs) */

#define SX127X_FOM_FIFO_LEN           (64)

/* LORA RX/TX FIFO size (one FIFO) */

#define SX127X_LRM_FIFO_LEN           (256)

/* LORA maximum payload length */

#define SX127X_LRM_PAYLOADMAX_DEFAULT (0xff)

/* FSK/OOK default shaping configuration */

#define SX127X_FSKOOK_SHAPING_DEFAULT SX127X_CMN_PARAMP_SHAPING_NONE

/* FSK/OOK default PARAMP configuration */

#define SX127X_FSKOOK_PARAMP_DEFAULT  SX127X_CMN_PARAMP_PARAMP_40us

/* Default code rate for LORA */

#define SX127X_LRM_CR_DEFAULT         LORA_CR_4d5

/* Default IDLE mode */

#define SX127X_IDLE_OPMODE            SX127X_OPMODE_STANDBY

/* Total size for local RX FIFO */

#define SX127X_RXFIFO_TOTAL_SIZE      (SX127X_RXFIFO_ITEM_SIZE*CONFIG_LPWAN_SX127X_RXFIFO_LEN)

/* Some assertions */

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
#  warning OOK support is not complete, RX+TX does not work yet!
#  if CONFIG_LPWAN_SX127X_RXFIFO_DATA_LEN > SX127X_FOM_FIFO_LEN
#    warning RX data length limited by chip RX FIFO size (FSK/OOK = 64, LORA = 256)
#  endif
#endif

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

/* SPI access mode */

typedef enum
{
  MODE_READ,
  MODE_WRITE
} sx127x_access_mode_t;

/* SX127X modulation specific ops */

struct sx127x_dev_s;
struct sx127x_priv_ops_s
{
  /* Initialize configuration for modulation */

  CODE void (*init)(FAR struct sx127x_dev_s *dev);

  /* Process IRQ 0 */

  CODE int (*isr0_process)(FAR struct sx127x_dev_s *dev);

  /* Operation mode initialization */

  CODE int (*opmode_init)(FAR struct sx127x_dev_s *dev, uint8_t opmode);

  /* Change operation mode */

  CODE int (*opmode_set)(FAR struct sx127x_dev_s *dev, uint8_t opmode);

  /* Set preamble length */

  CODE void (*preamble_set)(FAR struct sx127x_dev_s *dev, uint32_t len);

  /* Get preamble length */

  CODE int (*preamble_get)(FAR struct sx127x_dev_s *dev);

  /* Get current RSSI */

  CODE int16_t (*rssi_get)(FAR struct sx127x_dev_s *dev);

  /* Set sync word */

  CODE int (*syncword_set)(FAR struct sx127x_dev_s *dev, FAR uint8_t *sw,
                           uint8_t len);

  /* Get sync word */

  CODE void (*syncword_get)(FAR struct sx127x_dev_s *dev, FAR uint8_t *sw,
                            FAR uint8_t *len);

#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
  /* Send packet */

  CODE int (*send)(FAR struct sx127x_dev_s *dev, FAR const uint8_t *data,
              size_t datalen);
#endif
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  /* Dump registers for given modulation */

  CODE void (*dumpregs)(FAR struct sx127x_dev_s *dev);
#endif
};

#ifdef CONFIG_LPWAN_SX127X_FSKOOK

/* FSK/OOK private data */

struct sx127x_fskook_s
{
  uint32_t bitrate;             /* Bitrate */
  uint32_t fdev;                /* Frequency deviation */
  uint8_t  rx_bw;               /* RX bandwidth */
  uint8_t  afc_bw;              /* AFC bandwidth */
  uint8_t  addr_node;           /* Node address used in address filtering */
  uint8_t  addr_brdcast;        /* Broadcast address used int address filtering */
  bool     fixlen;              /* Fix length */
  bool     addr_fltr;           /* TODO: Address filtering */
  bool     seqon;               /* Sequencer enabled */
};
#endif

#ifdef CONFIG_LPWAN_SX127X_LORA
/* LORA private data */

struct sx127x_lora_s
{
  uint32_t freqhop;             /* Frequency hopping (not supported) */
  uint8_t  bw;                  /* LORA banwidth */
  uint8_t  sf;                  /* Spreading factor */
  uint8_t  cr;                  /* Coding rate */
  bool     implicthdr;          /* Implicit header mode ON */
  bool     invert_iq;           /* Invert I and Q signals */
};
#endif

/* SX127X private data */

struct sx127x_dev_s
{
  /* Reference to SPI bus device */

  FAR struct spi_dev_s *spi;

  /* Low-level MCU-specific support */

  FAR const struct sx127x_lower_s *lower;

  /* Operations specific for selected modulation scheme */

  struct sx127x_priv_ops_s ops;
  struct work_s irq0_work;        /* Interrupt DIO0 handling "bottom half" */

  uint32_t freq;                  /* RF carrier frequency */
  uint8_t  modulation;            /* Current modulation (LORA/FSK/OOK) */
  uint8_t  opmode;                /* Current operation mode */
  uint8_t  idle;                  /* IDLE opmode */
  bool     crcon;                 /* TX/RX CRC enable */
  bool     rx_cont;               /* RX in continuous mode (not supported) */
  bool     tx_cont;               /* TX in continuous mode (not supported) */

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
  struct sx127x_fskook_s fskook;  /* FSK/OOK modulation specific data */
#endif
#ifdef CONFIG_LPWAN_SX127X_LORA
  struct sx127x_lora_s   lora;    /* LORA modulation specific data */
#endif

#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
  sem_t    tx_sem;                /* Wait for availability of send data */
  uint32_t tx_timeout;            /* TX timeout (not supported) */
  int8_t   power;                 /* TX power */
  bool     pa_force;              /* Force PA BOOST pin select */
#endif
#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  uint32_t rx_timeout;            /* RX timeout (not supported) */
  uint16_t rx_fifo_len;           /* Number of bytes stored in fifo */
  uint16_t nxt_read;              /* Next read index */
  uint16_t nxt_write;             /* Next write index */

  /* Circular RX packet buffer */

  uint8_t  rx_buffer[SX127X_RXFIFO_TOTAL_SIZE];
  sem_t    rx_sem;                /* Wait for availability of received data */
  mutex_t  rx_buffer_lock;        /* Protect access to rx fifo */
#endif

  uint8_t nopens;                 /* Number of times the device has been opened */
  mutex_t dev_lock;               /* Ensures exclusive access to this structure */
  FAR struct pollfd *pfd;         /* Polled file descr  (or NULL if any) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level SPI helpres */

static void sx127x_lock(FAR struct spi_dev_s *spi);
static void sx127x_unlock(FAR struct spi_dev_s *spi);
static uint8_t sx127x_readregbyte(FAR struct sx127x_dev_s *dev, uint8_t reg);
static void sx127x_writeregbyte(FAR struct sx127x_dev_s *dev, uint8_t reg,
                                uint8_t value);
static uint8_t sx127x_modregbyte(FAR struct sx127x_dev_s *dev, uint8_t reg,
                                 uint8_t setbits, uint8_t clrbits);

/* LORA specific functions */

#ifdef CONFIG_LPWAN_SX127X_LORA
static void sx127x_lora_init(FAR struct sx127x_dev_s *dev);
static int16_t sx127x_lora_rssi_get(FAR struct sx127x_dev_s *dev);
static int16_t sx127x_lora_rssi_correct(FAR struct sx127x_dev_s *dev,
                                        uint32_t freq, int8_t snr,
                                        uint8_t regval);
static void sx127x_lora_preamble_set(FAR struct sx127x_dev_s *dev,
                                     uint32_t len);
static int sx127x_lora_preamble_get(FAR struct sx127x_dev_s *dev);
static int sx127x_lora_opmode_set(FAR struct sx127x_dev_s *dev,
                                  uint8_t opmode);
static int sx127x_lora_opmode_init(FAR struct sx127x_dev_s *dev,
                                   uint8_t opmode);
static int sx127x_lora_syncword_set(FAR struct sx127x_dev_s *dev,
                                    FAR uint8_t *sw, uint8_t len);
static void sx127x_lora_syncword_get(FAR struct sx127x_dev_s *dev,
                                     FAR uint8_t *sw, uint8_t *len);

#  ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
static int8_t sx127x_lora_snr_get(FAR struct sx127x_dev_s *dev);
static int16_t sx127x_lora_pckrssi_get(FAR struct sx127x_dev_s *dev,
                                       int8_t snr);
static size_t sx127x_lora_rxhandle(FAR struct sx127x_dev_s *dev);
#  endif
#  ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
static int sx127x_lora_send(FAR struct sx127x_dev_s *dev,
                            FAR const uint8_t *data, size_t datalen);
#  endif
#  ifdef CONFIG_DEBUG_WIRELESS_INFO
static void sx127x_lora_dumpregs(FAR struct sx127x_dev_s *dev);
#  endif
#endif

/* FSK/OOK specific functions */

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
static void sx127x_fskook_init(FAR struct sx127x_dev_s *dev);
static int sx127x_fskook_fdev_set(FAR struct sx127x_dev_s *dev,
                                  uint32_t freq);
static int16_t sx127x_fskook_rssi_get(FAR struct sx127x_dev_s *dev);
static int sx127x_fskook_bitrate_set(FAR struct sx127x_dev_s *dev,
                                     uint32_t bitrate);
static void sx127x_fskook_preamble_set(FAR struct sx127x_dev_s *dev,
                                       uint32_t len);
static int sx127x_fskook_preamble_get(FAR struct sx127x_dev_s *dev);
static int sx127x_fskook_opmode_init(FAR struct sx127x_dev_s *dev,
                                     uint8_t opmode);
static int sx127x_fskook_syncword_set(FAR struct sx127x_dev_s *dev,
                                      FAR uint8_t *sw, uint8_t len);
static void sx127x_fskook_syncword_get(FAR struct sx127x_dev_s *dev,
                                       FAR uint8_t *sw, FAR uint8_t *len);
#  ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
static size_t sx127x_fskook_rxhandle(FAR struct sx127x_dev_s *dev);
#  endif
#  ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
static int sx127x_fskook_send(FAR struct sx127x_dev_s *dev,
                              FAR const uint8_t *data, size_t datalen);
#  endif
#  ifdef CONFIG_DEBUG_WIRELESS_INFO
static void sx127x_fskook_dumpregs(FAR struct sx127x_dev_s *dev);
#  endif
#endif

/* Common for FSK/OOK and LORA */

static int sx127x_fskook_opmode_set(FAR struct sx127x_dev_s *dev,
                                    uint8_t opmode);
static int sx127x_init(FAR struct sx127x_dev_s *dev);
static int sx127x_deinit(FAR struct sx127x_dev_s *dev);
static int sx127x_unregister(FAR struct sx127x_dev_s *dev);
static inline int sx127x_attachirq0(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg);
static int sx127x_irq0handler(int irq, FAR void *context, FAR void *arg);

static int sx127x_modulation_set(FAR struct sx127x_dev_s *dev,
                                 uint8_t modulation);
static uint8_t sx127x_modulation_get(FAR struct sx127x_dev_s *dev);
static int16_t sx127x_rssi_get(FAR struct sx127x_dev_s *dev);
static int sx127x_frequency_set(FAR struct sx127x_dev_s *dev, uint32_t freq);
static uint32_t sx127x_frequency_get(FAR struct sx127x_dev_s *dev);
static int sx127x_power_set(FAR struct sx127x_dev_s *dev, int8_t power);
static int8_t sx127x_power_get(FAR struct sx127x_dev_s *dev);
static void sx127x_preamble_set(FAR struct sx127x_dev_s *dev, uint32_t len);
static int sx127x_preamble_get(FAR struct sx127x_dev_s *dev);
static int sx127x_opmode_set(FAR struct sx127x_dev_s *dev, uint8_t opmode);
static uint8_t sx127x_opmode_get(FAR struct sx127x_dev_s *dev);
static int sx127x_opmode_init(FAR struct sx127x_dev_s *dev, uint8_t opmode);
static int sx127x_syncword_set(FAR struct sx127x_dev_s *dev,
                               FAR uint8_t *sw, uint8_t len);
static void sx127x_syncword_get(FAR struct sx127x_dev_s *dev,
                                FAR uint8_t *sw, FAR uint8_t *len);
#ifdef CONFIG_DEBUG_WIRELESS_INFO
static void sx127x_dumpregs(FAR struct sx127x_dev_s *dev);
#else
#  define sx127x_dumpregs(x)
#endif

static bool sx127x_channel_scan(FAR struct sx127x_dev_s *dev,
                                FAR struct sx127x_chanscan_ioc_s *chanscan);
static uint32_t sx127x_random_get(FAR struct sx127x_dev_s *dev);

#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
static int sx127x_txfifo_write(FAR struct sx127x_dev_s *dev,
                               FAR const uint8_t *data, size_t datalen);
#endif
#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
static ssize_t sx127x_rxfifo_get(FAR struct sx127x_dev_s *dev,
                                 FAR uint8_t *buffer, size_t buflen);
static void sx127x_rxfifo_put(FAR struct sx127x_dev_s *dev,
                              FAR uint8_t *buffer, size_t buflen);
#endif

/* POSIX API */

static int sx127x_open(FAR struct file *filep);
static int sx127x_close(FAR struct file *filep);
static ssize_t sx127x_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t sx127x_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int sx127x_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int sx127x_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only one device is supported for now */

static struct sx127x_dev_s g_sx127x_devices[1];

/* File ops */

static const struct file_operations g_sx127x_fops =
{
  sx127x_open,    /* open */
  sx127x_close,   /* close */
  sx127x_read,    /* read */
  sx127x_write,   /* write */
  NULL,           /* seek */
  sx127x_ioctl,   /* ioctl */
  NULL,           /* mmap */
  NULL,           /* truncate */
  sx127x_poll     /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx127x_lock
 *
 * Description:
 *   Acquire exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static void sx127x_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, 1);
  SPI_SETBITS(spi, 8);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETFREQUENCY(spi, CONFIG_LPWAN_SX127X_SPIFREQ);
}

/****************************************************************************
 * Name: sx127x_unlock
 *
 * Description:
 *   Release exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static void sx127x_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, 0);
}

/****************************************************************************
 * Name: sx127x_select
 ****************************************************************************/

static inline void sx127x_select(FAR struct sx127x_dev_s *dev)
{
  SPI_SELECT(dev->spi, SPIDEV_LPWAN(0), true);
}

/****************************************************************************
 * Name: sx127x_deselect
 ****************************************************************************/

static inline void sx127x_deselect(FAR struct sx127x_dev_s *dev)
{
  SPI_SELECT(dev->spi, SPIDEV_LPWAN(0), false);
}

/****************************************************************************
 * Name: sx127x_access
 ****************************************************************************/

static uint8_t sx127x_access(FAR struct sx127x_dev_s *dev,
                             sx127x_access_mode_t mode, uint8_t cmd,
                             FAR uint8_t *buf, int length)
{
  uint8_t status = 0;

  /* Prepare SPI */

  sx127x_select(dev);

  /* Transfer */

  status = SPI_SEND(dev->spi, cmd);

  switch (mode)
    {
      case MODE_WRITE:
        {
          if (length > 0)
            {
              SPI_SNDBLOCK(dev->spi, buf, length);
            }

          break;
        }

      case MODE_READ:
        {
          SPI_RECVBLOCK(dev->spi, buf, length);
          break;
        }

      default:
        {
          wlerr("ERROR: unknown SPI access mode %d!\n", mode);
          break;
        }
    }

  sx127x_deselect(dev);

  return status;
}

/****************************************************************************
 * Name: sx127x_readreg
 *
 * Description:
 *   Read register from sx127x
 *
 ****************************************************************************/

static inline uint8_t sx127x_readreg(FAR struct sx127x_dev_s *dev,
                                     uint8_t reg, FAR uint8_t *value,
                                     int len)
{
  return sx127x_access(dev, MODE_READ, reg | SX127X_R_REGISTER, value, len);
}

/****************************************************************************
 * Name: sx127x_readregbyte
 *
 * Description:
 *   Read single byte value from a register of sx127x
 *
 ****************************************************************************/

static inline uint8_t sx127x_readregbyte(FAR struct sx127x_dev_s *dev,
                                         uint8_t reg)
{
  uint8_t val = 0;

  sx127x_readreg(dev, reg, &val, 1);

  return val;
}

/****************************************************************************
 * Name: sx127x_writereg
 *
 * Description:
 *   Write value to a register of sx127x
 *
 ****************************************************************************/

static inline int sx127x_writereg(FAR struct sx127x_dev_s *dev, uint8_t reg,
                                  FAR const uint8_t *value, int len)
{
  return sx127x_access(dev, MODE_WRITE, reg | SX127X_W_REGISTER,
                       (FAR uint8_t *)value, len);
}

/****************************************************************************
 * Name: sx127x_writeregbyte
 *
 * Description:
 *   Write single byte value to a register of sx127x
 *
 ****************************************************************************/

static inline void sx127x_writeregbyte(FAR struct sx127x_dev_s *dev,
                                       uint8_t reg, uint8_t value)
{
  sx127x_writereg(dev, reg, &value, 1);
}

/****************************************************************************
 * Name: sx127x_modreg
 *
 * Description:
 *  Modify register value of sx127x
 *
 ****************************************************************************/

static uint8_t sx127x_modregbyte(FAR struct sx127x_dev_s *dev, uint8_t reg,
                                 uint8_t setbits, uint8_t clrbits)
{
  uint8_t val = 0;

  sx127x_readreg(dev, reg, &val, 1);

  val &= ~clrbits;
  val |= setbits;

  sx127x_writereg(dev, reg, &val, 1);
  return val;
}

/****************************************************************************
 * Name: sx127x_attachirq0
 ****************************************************************************/

static inline int sx127x_attachirq0(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
  return dev->lower->irq0attach(isr, arg);
}

/****************************************************************************
 * Name: sx127x_attachirq1
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_DIO1
static inline int sx127x_attachirq1(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
  return dev->lower->irq1attach(isr, arg);
}
#endif

/****************************************************************************
 * Name: sx127x_attachirq2
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_DIO2
static inline int sx127x_attachirq2(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
  return dev->lower->irq2attach(isr, arg);
}
#endif

/****************************************************************************
 * Name: sx127x_attachirq3
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_DIO3
static inline int sx127x_attachirq3(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
  return dev->lower->irq3attach(isr, arg);
}
#endif

/****************************************************************************
 * Name: sx127x_attachirq4
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_DIO4
static inline int sx127x_attachirq4(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
  return dev->lower->irq4attach(isr, arg);
}
#endif

/****************************************************************************
 * Name: sx127x_attachirq5
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_DIO5
static inline int sx127x_attachirq5(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
  return dev->lower->irq5attach(isr, arg);
}
#endif

/****************************************************************************
 * Name: sx127x_reset
 *
 * Description:
 *   Reset radio
 *
 ****************************************************************************/

static void sx127x_reset(FAR struct sx127x_dev_s *dev)
{
  dev->lower->reset();
}

/****************************************************************************
 * Name: sx127x_open
 *
 * Description:
 *   This function is called whenever the SX127X device is opened.
 *
 ****************************************************************************/

static int sx127x_open(FAR struct file *filep)
{
  FAR struct sx127x_dev_s *dev   = NULL;
  FAR struct inode        *inode = NULL;
  int ret = 0;

  wlinfo("Opening sx127x dev\n");

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct sx127x_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&dev->dev_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if device is not already used */

  if (dev->nopens > 0)
    {
      ret = -EBUSY;
      goto errout;
    }

  /* Initialize device */

  ret = sx127x_init(dev);
  if (ret < 0)
    {
      wlerr("ERROR: failed to initialize sx127x\n");
      goto errout;
    }

  dev->nopens++;

errout:
  nxmutex_unlock(&dev->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: sx127x_close
 *
 * Description:
 *   This routine is called when the SX127X device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ****************************************************************************/

static int sx127x_close(FAR struct file *filep)
{
  FAR struct sx127x_dev_s *dev   = NULL;
  FAR struct inode        *inode = NULL;
  int ret = 0;

  wlinfo("Closing sx127x dev\n");
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct sx127x_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&dev->dev_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = sx127x_deinit(dev);
  if (ret < 0)
    {
      wlerr("ERROR: failed to deinit sx127x\n");
    }

  dev->nopens--;

  nxmutex_unlock(&dev->dev_lock);
  return OK;
}

/****************************************************************************
 * Name: sx127x_read
 *
 * Description:
 *   Standard driver read method
 *
 ****************************************************************************/

static ssize_t sx127x_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
#ifndef CONFIG_LPWAN_SX127X_RXSUPPORT
  return -ENOSYS;
#else
  FAR struct sx127x_dev_s *dev   = NULL;
  FAR struct inode        *inode = NULL;
  int ret = 0;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct sx127x_dev_s *)inode->i_private;

  ret = nxmutex_lock(&dev->dev_lock);
  if (ret < 0)
    {
      return ret;
    }

  if ((filep->f_oflags & O_NONBLOCK) != 0)
    {
      nxsem_trywait(&dev->rx_sem);
      ret = 0;
    }
  else
    {
      ret = nxsem_wait(&dev->rx_sem);
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Get RX data from fifo */

  ret = sx127x_rxfifo_get(dev, (FAR uint8_t *)buffer, buflen);

  nxmutex_unlock(&dev->dev_lock);
  return ret;
#endif
}

/****************************************************************************
 * Name: sx127x_write
 *
 * Description:
 *   Standard driver write method.
 *
 ****************************************************************************/

static ssize_t sx127x_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
#ifndef CONFIG_LPWAN_SX127X_TXSUPPORT
  return -ENOSYS;
#else
  FAR struct sx127x_dev_s *dev   = NULL;
  FAR struct inode        *inode = NULL;
  int ret = 0;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct sx127x_dev_s *)inode->i_private;

  ret = nxmutex_lock(&dev->dev_lock);
  if (ret < 0)
    {
      return ret;
    }

  wlinfo("buflen=%d\n", buflen);

  /* Change mode to STANDBY */

  sx127x_opmode_set(dev, SX127X_OPMODE_STANDBY);

  /* Initialize TX mode */

  ret = sx127x_opmode_init(dev, SX127X_OPMODE_TX);
  if (ret < 0)
    {
      /* Restore IDLE mode settings */

      sx127x_opmode_init(dev, dev->idle);

      wlerr("Failed to initialize TX mode!\n");

      ret = -EINVAL;
      goto errout;
    }

  /* Call modulation specific send */

  ret = dev->ops.send(dev, (FAR uint8_t *)buffer, buflen);

  /* Change mode to TX to start data transfer */

  sx127x_opmode_set(dev, SX127X_OPMODE_TX);

  /* Wait for TXDONE */

  nxsem_wait(&dev->tx_sem);

errout:
  /* Change mode to IDLE after transfer
   * NOTE: if sequencer for FSK/OOK is ON - this should be done automatically
   */

  sx127x_opmode_set(dev, dev->idle);
  nxmutex_unlock(&dev->dev_lock);

  return ret;
#endif
}

/****************************************************************************
 * Name: sx127x_ioctl
 *
 * Description:
 *   Standard driver ioctl method.
 *
 ****************************************************************************/

static int sx127x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct sx127x_dev_s *dev = NULL;
  FAR struct inode *inode      = NULL;
  int ret                      = 0;

  wlinfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct sx127x_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&dev->dev_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      /* Set radio frequency. Arg: Pointer to uint32_t frequency value in
       * Hz.
       */

      case WLIOC_SETRADIOFREQ:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          sx127x_frequency_set(dev, *ptr);
          break;
        }

      /* Get current radio frequency. arg: Pointer to uint32_t frequency
       * value in Hz.
       */

      case WLIOC_GETRADIOFREQ:
        {
          FAR int8_t *ptr = (FAR int8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_frequency_get(dev);
          break;
        }

      /* Set TX power. arg: Pointer to int8_t power value */

      case WLIOC_SETTXPOWER:
        {
          FAR int8_t *ptr = (FAR int8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          sx127x_power_set(dev, *ptr);
          break;
        }

      /* Get current TX power. arg: Pointer to int8_t power value */

      case WLIOC_GETTXPOWER:
        {
          FAR int8_t *ptr = (FAR int8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_power_get(dev);
          break;
        }

      /* Get RSSI */

      case SX127XIOC_RSSIGET:
        {
          FAR int16_t *ptr = (FAR int16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_rssi_get(dev);
          break;
        }

      /* Set modulation */

      case SX127XIOC_MODULATIONSET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          ret = sx127x_modulation_set(dev, *ptr);
          break;
        }

      /* Get modulation */

      case SX127XIOC_MODULATIONGET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_modulation_get(dev);
          break;
        }

      /* Operation mode set */

      case SX127XIOC_OPMODESET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          ret = sx127x_opmode_set(dev, *ptr);
          break;
        }

      /* Operation mode get */

      case SX127XIOC_OPMODEGET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_opmode_get(dev);
          break;
        }

      /* Channel scan */

      case SX127XIOC_CHANSCAN:
        {
          FAR struct sx127x_chanscan_ioc_s *ptr
              = (FAR struct sx127x_chanscan_ioc_s *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          sx127x_channel_scan(dev, ptr);
          break;
        }

      /* Preamble length set */

      case SX127XIOC_PREAMBLESET:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          sx127x_preamble_set(dev, *ptr);
          break;
        }

      /* Preamble length get */

      case SX127XIOC_PREAMBLEGET:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_preamble_get(dev);
          break;
        }

      /* SyncWord set */

      case SX127XIOC_SYNCWORDSET:
        {
          PANIC();
          sx127x_syncword_set(dev, NULL, 0);
          break;
        }

      /* SyncWord get */

      case SX127XIOC_SYNCWORDGET:
        {
          PANIC();
          sx127x_syncword_get(dev, NULL, 0);
          break;
        }

      /* Get random number based on RSSI */

      case SX127XIOC_RANDOMGET:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_random_get(dev);
          break;
        }

      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  nxmutex_unlock(&dev->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: sx127x_poll
 *
 * Description:
 *   Standard driver poll method.
 *
 ****************************************************************************/

static int sx127x_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
#ifndef CONFIG_LPWAN_SX127X_RXSUPPORT
  return -ENOSYS;
#else

  FAR struct sx127x_dev_s *dev   = NULL;
  FAR struct inode        *inode = NULL;
  int ret = 0;

  wlinfo("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct sx127x_dev_s *)inode->i_private;

  /* Exclusive access */

  ret = nxmutex_lock(&dev->dev_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* Check if we can accept this poll.
       * For now, only one thread can poll the device at any time
       * (shorter / simpler code)
       */

      if (dev->pfd)
        {
          ret = -EBUSY;
          goto errout;
        }

      dev->pfd = fds;

      /* Is there is already data in the fifo? then trigger POLLIN now -
       * don't wait for RX.
       */

      nxmutex_lock(&dev->rx_buffer_lock);
      if (dev->rx_fifo_len > 0)
        {
          /* Data available for input */

          poll_notify(&dev->pfd, 1, POLLIN);
        }

      nxmutex_unlock(&dev->rx_buffer_lock);
    }
  else /* Tear it down */
    {
      dev->pfd = NULL;
    }

errout:
  nxmutex_unlock(&dev->dev_lock);
  return ret;
#endif
}

/****************************************************************************
 * Name: sx127x_lora_isr0_process
 *
 * Description:
 *   Handle DIO0 interrupt for LORA radio
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_LORA
static int sx127x_lora_isr0_process(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  bool    data_valid = true;
#endif
  uint8_t irq        = 0;
  int     ret        = OK;

  /* Get IRQ */

  sx127x_lock(dev->spi);
  irq = sx127x_readregbyte(dev, SX127X_LRM_IRQ);
  sx127x_unlock(dev->spi);

  wlinfo("ISR0: IRQ = 0x%02x\n", irq);

  switch (dev->opmode)
    {
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
      /* TX DONE */

      case SX127X_OPMODE_TX:
        {
          /* Release TX sem */

          nxsem_post(&dev->tx_sem);

          /* Clear TX interrupt */

          irq = SX127X_LRM_IRQ_TXDONE;
          break;
        }
#endif /* CONFIG_LPWAN_SX127X_TXSUPPORT */

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
      /* RX DONE */

      case SX127X_OPMODE_RX:
      case SX127X_OPMODE_RXSINGLE:
        {
          /* REVISIT: Always check PAYLOADCRCERR even CRCONPAYLOAD not set */

          if ((irq & SX127X_LRM_IRQ_PAYLOADCRCERR) != 0)
            {
              data_valid = false;
            }

          if (data_valid)
            {
              ret = sx127x_lora_rxhandle(dev);
              if (ret > 0)
                {
                  if (dev->pfd)
                    {
                      /* Data available for input */

                      poll_notify(&dev->pfd, 1, POLLIN);
                    }

                  /* Wake-up any thread waiting in recv */

                  nxsem_post(&dev->rx_sem);
                }
            }
          else
            {
              /* RX Data invalid */

              wlinfo("Invalid LORA RX data!\n");
            }

          /* After receiving the data in RXSINGLE mode the chip goes into
           * STANBY mode
           */

          if (dev->opmode == SX127X_OPMODE_RXSINGLE)
            {
              dev->opmode = SX127X_OPMODE_STANDBY;
            }

          /* Clear RX interrupts  */

          irq = (SX127X_LRM_IRQ_RXDONE | SX127X_LRM_IRQ_PAYLOADCRCERR |
                 SX127X_LRM_IRQ_VALIDHDR);
          break;
        }
#endif /* CONFIG_LPWAN_SX127X_RXSUPPORT */

      /* Only LORA - CAD DONE */

      case SX127X_OPMODE_CAD:
        {
          /* TODO */

          wlerr("TODO: ISR0 in CAD mode not implemented yet!\n");

          /* Clear CAD interrupt */

          irq = SX127X_LRM_IRQ_CADDONE;
          break;
        }

      default:
        {
          wlwarn("WARNING: Interrupt not processed, opmode=%d\n",
                 dev->opmode);
          ret = -EINVAL;
          break;
        }
    }

  /* Clear interrupts */

  sx127x_lock(dev->spi);
  sx127x_writeregbyte(dev, SX127X_LRM_IRQ, irq);
  sx127x_unlock(dev->spi);

  return ret;
}
#endif /* CONFIG_LPWAN_SX127X_LORA */

/****************************************************************************
 * Name: sx127x_fskook_isr0_process
 *
 * Description:
 *   Handle DIO0 interrupt for FSK/OOK radio
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
static int sx127x_fskook_isr0_process(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  bool    data_valid = true;
#endif
  uint8_t irq1       = 0;
  uint8_t irq2       = 0;
  int     ret        = OK;

  /* Get IRQ1 and IRQ2 */

  sx127x_lock(dev->spi);
  irq1 = sx127x_readregbyte(dev, SX127X_FOM_IRQ1);
  irq2 = sx127x_readregbyte(dev, SX127X_FOM_IRQ2);
  sx127x_unlock(dev->spi);

  wlinfo("ISR0: IRQ1 = 0x%02x, IRQ2 = 0x%02x\n", irq1, irq2);

  switch (dev->opmode)
    {
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
      /* TX DONE */

      case SX127X_OPMODE_TX:
        {
          /* Release TX sem */

          nxsem_post(&dev->tx_sem);
          break;
        }
#endif /* CONFIG_LPWAN_SX127X_TXSUPPORT */

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
      /* RX DONE */

      case SX127X_OPMODE_RX:
        {
          /* RX data valid ? */

          if (dev->crcon == true && (irq2 & SX127X_FOM_IRQ2_CRCOK) == 0)
            {
              data_valid = false;
            }

          if (data_valid == true)
            {
              /* RX data valid */

              ret = sx127x_fskook_rxhandle(dev);
              if (ret > 0)
                {
                  if (dev->pfd)
                    {
                      /* Data available for input */

                      poll_notify(&dev->pfd, 1, POLLIN);
                    }

                  /* Wake-up any thread waiting in recv */

                  nxsem_post(&dev->rx_sem);
                }
            }
          else
            {
              /* RX Data invalid */

              wlinfo("Invalid FSK/OOK RX data!\n");
            }

          /* TODO: restart RX if continuous mode */

          break;
        }
#endif /* CONFIG_LPWAN_SX127X_RXSUPPORT */

      default:
        {
          wlwarn("WARNING: Interrupt not processed\n");
          ret = -EINVAL;
          break;
        }
    }

  /* REVISIT: clear interrupts */

  irq1 = (SX127X_FOM_IRQ1_RSSI | SX127X_FOM_IRQ1_PREAMBE |
          SX127X_FOM_IRQ1_SYNCADDRMATCH);
  irq2 = SX127X_FOM_IRQ2_FIFOOVR;

  sx127x_lock(dev->spi);
  sx127x_writeregbyte(dev, SX127X_FOM_IRQ1, irq1);
  sx127x_writeregbyte(dev, SX127X_FOM_IRQ2, irq2);
  sx127x_unlock(dev->spi);

  return ret;
}
#endif /* CONFIG_LPWAN_SX127X_FSKOOK */

/****************************************************************************
 * Name: sx127x_isr0_process
 *
 * Description:
 *   Handle DIO0 interrupt for LORA radio
 *
 ****************************************************************************/

static void sx127x_isr0_process(FAR void *arg)
{
  DEBUGASSERT(arg);

  FAR struct sx127x_dev_s *dev = (FAR struct sx127x_dev_s *)arg;
  int ret = OK;

  /* Return immediately if isr0_process is not initialized */

  if (dev->ops.isr0_process == NULL)
    {
      return;
    }

  /* isr0_process depends on the current modulation scheme */

  ret = dev->ops.isr0_process(dev);
  if (ret < 0)
    {
      wlerr("Failed to process ISR0 %d\n", ret);
    }
}

/****************************************************************************
 * Name: sx127x_irq0handler
 ****************************************************************************/

static int sx127x_irq0handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct sx127x_dev_s *dev = (FAR struct sx127x_dev_s *)arg;

  DEBUGASSERT(dev != NULL);

  DEBUGASSERT(work_available(&dev->irq0_work));

  return work_queue(HPWORK, &dev->irq0_work, sx127x_isr0_process, arg, 0);
}

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT

/****************************************************************************
 * Name: sx127x_fskook_rxhandle
 *
 * Description:
 *   Receive data from FIFO for FSK/OOK radio
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
static size_t sx127x_fskook_rxhandle(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  struct sx127x_read_hdr_s rxdata;
  uint8_t datalen = 0;
  size_t len      = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get data from chip fifo */

  if (dev->fskook.fixlen == true)
    {
      /* Fixed packet length hardcoded */

      datalen = SX127X_RX_FIXLEN_DEFAULT;
    }
  else
    {
      /* First byte is payload length */

      datalen = sx127x_readregbyte(dev, SX127X_CMN_FIFO);
    }

  /* Ignore packets with unsupported data length */

  if (datalen > SX127X_READ_DATA_MAX)
    {
      wlerr("Unsupported data length! %d > %d\n",
            datalen, SX127X_READ_DATA_MAX);
      sx127x_unlock(dev->spi);
      return 0;
    }

  /* Read payload and store */

  sx127x_readreg(dev, SX127X_CMN_FIFO, rxdata.data, datalen);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* No RX SNR data for FSK/OOK */

  rxdata.snr = 0;

  /* Store last RSSI */

  rxdata.rssi = sx127x_fskook_rssi_get(dev);

  /* Store packet length */

  rxdata.datalen = datalen;

  /* Total length */

  len = datalen + SX127X_READ_DATA_HEADER_LEN;

  /* Put data on local fifo */

  sx127x_rxfifo_put(dev, (FAR uint8_t *)&rxdata, len);

  /* Return total length */

  return len;
}
#endif /* CONFIG_LPWAN_SX127X_FSKOOK */

/****************************************************************************
 * Name: sx127x_lora_rxhandle
 *
 * Description:
 *   Receive data from FIFO for LORA radio
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_LORA
static size_t sx127x_lora_rxhandle(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  struct sx127x_read_hdr_s rxdata;
  size_t  len     = 0;
  uint8_t datalen = 0;
  uint8_t rx_ptr  = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get payload length */

  datalen = sx127x_readregbyte(dev, SX127X_LRM_RXBYTES);

  /* Ignore packets with unsupported data length */

  if (datalen > SX127X_READ_DATA_MAX)
    {
      wlerr("Unsupported data length! %d > %d\n",
            datalen, SX127X_READ_DATA_MAX);
      sx127x_unlock(dev->spi);
      return 0;
    }

  /* Get start address of last packet received */

  rx_ptr = sx127x_readregbyte(dev, SX127X_LRM_RXCURR);

  /* Set FIFO pointer */

  sx127x_writeregbyte(dev, SX127X_LRM_ADDRPTR, rx_ptr);

  /* Read payload */

  sx127x_readreg(dev, SX127X_CMN_FIFO, rxdata.data, datalen);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Store last RX SNR */

  rxdata.snr = sx127x_lora_snr_get(dev);

  /* Store last RX RSSI */

  rxdata.rssi = sx127x_lora_pckrssi_get(dev, rxdata.snr);

  /* Store packet length */

  rxdata.datalen = datalen;

  /* Total length */

  len = datalen + SX127X_READ_DATA_HEADER_LEN;

  /* Put data on local fifo */

  sx127x_rxfifo_put(dev, (FAR uint8_t *)&rxdata, len);

  /* Return total length */

  return len;
}
#endif /* CONFIG_LPWAN_SX127X_LORA */

/****************************************************************************
 * Name: sx127x_rxfifo_get
 *
 * Description:
 *   Get data from RX FIFO
 *
 ****************************************************************************/

static ssize_t sx127x_rxfifo_get(FAR struct sx127x_dev_s *dev,
                                 FAR uint8_t *buffer, size_t buflen)
{
  FAR struct sx127x_read_hdr_s *pkt = NULL;
  size_t i      = 0;
  size_t pktlen = 0;
  size_t ret    = 0;

  ret = nxmutex_lock(&dev->rx_buffer_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* No data on RX FIFO */

  if (dev->rx_fifo_len == 0)
    {
      pktlen = 0;
      goto no_data;
    }

  /* Get packet header */

  pkt = (struct sx127x_read_hdr_s *)
    (dev->rx_buffer + dev->nxt_read * SX127X_RXFIFO_ITEM_SIZE);

  /* Packet length is data length + header length */

  pktlen = pkt->datalen + SX127X_READ_DATA_HEADER_LEN;

  /* Get packet from FIFO */

  for (i = 0; i < pktlen && i < SX127X_RXFIFO_ITEM_SIZE; i += 1)
    {
      buffer[i] =
        dev->rx_buffer[dev->nxt_read * SX127X_RXFIFO_ITEM_SIZE + i];
    }

  dev->nxt_read = (dev->nxt_read + 1) % CONFIG_LPWAN_SX127X_RXFIFO_LEN;
  dev->rx_fifo_len--;

  ret = pktlen;

no_data:
  nxmutex_unlock(&dev->rx_buffer_lock);
  return ret;
}

/****************************************************************************
 * Name: sx127x_rxfifo_put
 *
 * Description:
 *   Put packet data on RX FIFO
 *
 ****************************************************************************/

static void sx127x_rxfifo_put(FAR struct sx127x_dev_s *dev,
                              FAR uint8_t *buffer, size_t buflen)
{
  size_t  i   = 0;
  int     ret = 0;

  ret = nxmutex_lock(&dev->rx_buffer_lock);
  if (ret < 0)
    {
      return;
    }

  dev->rx_fifo_len++;
  if (dev->rx_fifo_len > CONFIG_LPWAN_SX127X_RXFIFO_LEN)
    {
      dev->rx_fifo_len = CONFIG_LPWAN_SX127X_RXFIFO_LEN;
      dev->nxt_read = (dev->nxt_read + 1) % CONFIG_LPWAN_SX127X_RXFIFO_LEN;
    }

  /* Put packet on fifo */

  for (i = 0; i < (buflen + 1) && i < SX127X_RXFIFO_ITEM_SIZE; i += 1)
    {
      dev->rx_buffer[i + dev->nxt_write * SX127X_RXFIFO_ITEM_SIZE] =
        buffer[i];
    }

  dev->nxt_write = (dev->nxt_write + 1) % CONFIG_LPWAN_SX127X_RXFIFO_LEN;
  nxmutex_unlock(&dev->rx_buffer_lock);
}

#endif /* CONFIG_LPWAN_SX127X_RXSUPPORT */

#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT

/****************************************************************************
 * Name: sx127x_txfifo_write
 *
 * Description:
 *   Write data to the SX127X TX FIFO
 *
 ****************************************************************************/

static int sx127x_txfifo_write(FAR struct sx127x_dev_s *dev,
                               FAR const uint8_t *data, size_t datalen)
{
  /* NOTE: Do not lock SPI here, it should be already locked! */

  /* Write buffer to FIFO */

  sx127x_writereg(dev, SX127X_CMN_FIFO, data, datalen);
  return OK;
}

/****************************************************************************
 * Name: sx127x_fskook_send
 *
 * Description:
 *   Send data in FSK/OOK radio mode
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
static int sx127x_fskook_send(FAR struct sx127x_dev_s *dev,
                              FAR const uint8_t *data, size_t datalen)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  int ret = 0;

  /* Check payload length */

  if (datalen > SX127X_FOM_PAYLOADLEN_MAX)
    {
      wlerr("Not supported data len!\n");
      return -EINVAL;
    }

#if 1
  /* For now we don't support datalen > FIFO_LEN for FSK/OOK.
   * For fixlen = true,  datalen <= 64
   * For fixlen = false, datalen < 64 (we support this for now)
   */

  if (datalen > 63)
    {
      wlerr("Not supported data len!\n");
      return -EINVAL;
    }
#endif

  /* Lock SPI */

  sx127x_lock(dev->spi);

  if (dev->fskook.fixlen == true)
    {
      /* Write payload length register (only LSB for now) */

      sx127x_writeregbyte(dev, SX127X_FOM_PAYLOADLEN, datalen);
    }
  else
    {
      /* First byte is length */

      ret = sx127x_txfifo_write(dev, (FAR uint8_t *)&datalen, 1);
    }

  /* Write payload */

  sx127x_txfifo_write(dev, data, datalen);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
  return ret;
}
#endif /* CONFIG_LPWAN_SX127X_FSKOOK */

/****************************************************************************
 * Name: sx127x_lora_send
 *
 * Description:
 *   Send data in LORA radio mode
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_LORA
static int sx127x_lora_send(FAR struct sx127x_dev_s *dev,
                            FAR const uint8_t *data, size_t datalen)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  int ret = 0;

  /* Check payload length */

  if (datalen > SX127X_LRM_PAYLOADLEN_MAX)
    {
      wlerr("Not supported data len!\n");
      return -EINVAL;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Configure payload length */

  sx127x_writeregbyte(dev, SX127X_LRM_PAYLOADLEN, datalen);

  /* Write payload */

  sx127x_txfifo_write(dev, data, datalen);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
  return ret;
}
#endif /* CONFIG_LPWAN_SX127X_LORA */
#endif /* CONFIG_LPWAN_SX127X_TXSUPPORT */

/****************************************************************************
 * Name: sx127x_opmode_init
 *
 * Description:
 *   Initialize operation mode
 *
 ****************************************************************************/

static int sx127x_opmode_init(FAR struct sx127x_dev_s *dev, uint8_t opmode)
{
  int ret = OK;

  if (opmode == dev->opmode)
    {
      return OK;
    }

  /* Board-specific opmode configuration */

  ret = dev->lower->opmode_change(opmode);
  if (ret < 0)
    {
      wlerr("Board-specific opmode_change failed %d!\n", ret);
      return ret;
    }

  /* Initialize opmode */

  ret = dev->ops.opmode_init(dev, opmode);
  if (ret < 0)
    {
      wlerr("opmode_init failed %d!\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: sx127x_opmode_set
 *
 * Description:
 *   Set operation mode
 *
 ****************************************************************************/

static int sx127x_opmode_set(FAR struct sx127x_dev_s *dev, uint8_t opmode)
{
  int ret = OK;

  wlinfo("opmode_set %d->%d\n", dev->opmode, opmode);

  if (opmode == dev->opmode)
    {
      return ret;
    }

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  /* REVISIT: TX is initialized before data send,
   * but where we should initialize RX ?
   */

  if (opmode != SX127X_OPMODE_TX)
    {
      ret = sx127x_opmode_init(dev, opmode);
    }
#endif

  /* Change mode */

  dev->ops.opmode_set(dev, opmode);

  /* Update local variable */

  dev->opmode = opmode;
  return ret;
}

/****************************************************************************
 * Name: sx127x_opmode_get
 *
 * Description:
 *   Get current operation mode
 *
 ****************************************************************************/

static uint8_t sx127x_opmode_get(FAR struct sx127x_dev_s *dev)
{
  wlerr("TODO: sx127x_opmode_get not implemented yet\n");
  return 0;
}

/****************************************************************************
 * Name: sx127x_lora_opmode_init
 *
 * Description:
 *   Initialize operation mode for FSK/OOK.
 *   We need this even if FSK/OOK support is disabled
 *
 ****************************************************************************/

static int sx127x_fskook_opmode_init(FAR struct sx127x_dev_s *dev,
                                     uint8_t opmode)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  uint8_t dio0map = 0;
  uint8_t setbits = 0;
  uint8_t clrbits = 0;
  int     ret     = OK;

  sx127x_lock(dev->spi);

  /* Get mode specific configuration */

  switch (opmode)
    {
      case SX127X_OPMODE_SLEEP:
      case SX127X_OPMODE_STANDBY:
      case SX127X_OPMODE_FSRX:
      case SX127X_OPMODE_FSTX:
        {
          break;
        }

      case SX127X_OPMODE_TX:
        {
          /* Remap DIO0 to RXTX DONE */

          dio0map = SX127X_FOM_DIOMAP1_DIO0_RXTX;

          /* TX start condition on FIFO not empty */

          sx127x_writeregbyte(dev, SX127X_FOM_FIFOTHR,
                              SX127X_FOM_FIFOTHR_TXSTARTCOND);

          break;
        }

      case SX127X_OPMODE_RX:
        {
          /* Remap DIO0 to RXTX DONE */

          dio0map = SX127X_FOM_DIOMAP1_DIO0_RXTX;

          /* REVISIT: Configure RXCFG register:
           * - AGC auto ON
           * - AFC auto ON
           * - RX trigger on PreableDetect
           */

          setbits = (SX127X_FOM_RXCFG_AGCAUTOON | SX127X_FOM_RXCFG_AFCAUTOON
                     | SX127X_FOM_RXCFG_TRG_PREDET);

          sx127x_writeregbyte(dev, SX127X_FOM_RXCFG, setbits);

          break;
        }

      default:
        {
          wlerr("ERROR: invalid mode %d\n", opmode);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Configure DIO0 pin */

  setbits = dio0map;
  clrbits = SX127X_CMN_DIOMAP1_DIO0_MASK;
  sx127x_modregbyte(dev, SX127X_CMN_DIOMAP1, setbits, clrbits);

errout:
  sx127x_unlock(dev->spi);
  return ret;
}

/****************************************************************************
 * Name: sx127x_fskook_opmode_set
 *
 * Description:
 *   Set operation mode for FSK/OOK.
 *   We need this even if FSK/OOK support is disabled
 *
 ****************************************************************************/

static int sx127x_fskook_opmode_set(FAR struct sx127x_dev_s *dev,
                                    uint8_t opmode)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  uint8_t setbits = 0;
  uint8_t clrbits = 0;

  switch (opmode)
    {
      case SX127X_OPMODE_SLEEP:
      case SX127X_OPMODE_STANDBY:
      case SX127X_OPMODE_FSRX:
      case SX127X_OPMODE_FSTX:
      case SX127X_OPMODE_TX:
      case SX127X_OPMODE_RX:
        {
          /* Do nothing */

          break;
        }

      default:
        {
          wlerr("ERROR: invalid FSK/OOK mode %d\n", opmode);
          return -EINVAL;
        }
    }

  sx127x_lock(dev->spi);

  /* Update mode */

  setbits = ((opmode - 1) << SX127X_CMN_OPMODE_MODE_SHIFT);
  clrbits = SX127X_CMN_OPMODE_MODE_MASK;
  sx127x_modregbyte(dev, SX127X_CMN_OPMODE, setbits, clrbits);

  sx127x_unlock(dev->spi);
  return OK;
}

#ifdef CONFIG_LPWAN_SX127X_FSKOOK

/****************************************************************************
 * Name: sx127x_fskook_rxbw_set
 *
 * Description:
 *  Set RX BW for FSK/OOK
 *
 ****************************************************************************/

static int sx127x_fskook_rxbw_set(FAR struct sx127x_dev_s *dev,
                                  uint8_t rx_bw)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  if (rx_bw == dev->fskook.rx_bw)
    {
      return OK;
    }

  switch (rx_bw)
    {
      case FSKOOK_BANDWIDTH_2P6KHZ:
      case FSKOOK_BANDWIDTH_3P1KHZ:
      case FSKOOK_BANDWIDTH_3P9KHZ:
      case FSKOOK_BANDWIDTH_5P2KHZ:
      case FSKOOK_BANDWIDTH_6P3KHZ:
      case FSKOOK_BANDWIDTH_7P8KHZ:
      case FSKOOK_BANDWIDTH_10P4KHZ:
      case FSKOOK_BANDWIDTH_12P5KHZ:
      case FSKOOK_BANDWIDTH_15P6KHZ:
      case FSKOOK_BANDWIDTH_20P8KHZ:
      case FSKOOK_BANDWIDTH_25KHZ:
      case FSKOOK_BANDWIDTH_31P3KHZ:
      case FSKOOK_BANDWIDTH_41P7KHZ:
      case FSKOOK_BANDWIDTH_50KHZ:
      case FSKOOK_BANDWIDTH_62P5KHZ:
      case FSKOOK_BANDWIDTH_83P3KHZ:
      case FSKOOK_BANDWIDTH_100KHZ:
      case FSKOOK_BANDWIDTH_125KHZ:
      case FSKOOK_BANDWIDTH_166P7KHZ:
      case FSKOOK_BANDWIDTH_200KHZ:
      case FSKOOK_BANDWIDTH_250KHZ:
        {
          /* Lock SPI */

          sx127x_lock(dev->spi);

          /* Write register */

          sx127x_writeregbyte(dev, SX127X_FOM_RXBW, rx_bw);

          /* Unlock SPI */

          sx127x_unlock(dev->spi);

          break;
        }

      default:
        {
          wlerr("Unsupported bandwidth %d\n", rx_bw);
          return -EINVAL;
        }
    }

  /* Update local */

  dev->fskook.rx_bw = rx_bw;
  return OK;
}

/****************************************************************************
 * Name: sx127x_fskook_afcbw_set
 *
 * Description:
 *  Set AFC BW for FSK/OOK
 *
 ****************************************************************************/

static int sx127x_fskook_afcbw_set(FAR struct sx127x_dev_s *dev,
                                   uint8_t afc_bw)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  if (afc_bw == dev->fskook.afc_bw)
    {
      return OK;
    }

  switch (afc_bw)
    {
      case FSKOOK_BANDWIDTH_2P6KHZ:
      case FSKOOK_BANDWIDTH_3P1KHZ:
      case FSKOOK_BANDWIDTH_3P9KHZ:
      case FSKOOK_BANDWIDTH_5P2KHZ:
      case FSKOOK_BANDWIDTH_6P3KHZ:
      case FSKOOK_BANDWIDTH_7P8KHZ:
      case FSKOOK_BANDWIDTH_10P4KHZ:
      case FSKOOK_BANDWIDTH_12P5KHZ:
      case FSKOOK_BANDWIDTH_15P6KHZ:
      case FSKOOK_BANDWIDTH_20P8KHZ:
      case FSKOOK_BANDWIDTH_25KHZ:
      case FSKOOK_BANDWIDTH_31P3KHZ:
      case FSKOOK_BANDWIDTH_41P7KHZ:
      case FSKOOK_BANDWIDTH_50KHZ:
      case FSKOOK_BANDWIDTH_62P5KHZ:
      case FSKOOK_BANDWIDTH_83P3KHZ:
      case FSKOOK_BANDWIDTH_100KHZ:
      case FSKOOK_BANDWIDTH_125KHZ:
      case FSKOOK_BANDWIDTH_166P7KHZ:
      case FSKOOK_BANDWIDTH_200KHZ:
      case FSKOOK_BANDWIDTH_250KHZ:
        {
          /* Lock SPI */

          sx127x_lock(dev->spi);

          /* Write register */

          sx127x_writeregbyte(dev, SX127X_FOM_AFCBW, afc_bw);

          /* Unlock SPI */

          sx127x_unlock(dev->spi);

          break;
        }

      default:
        {
          wlerr("Unsupported bandwidth %d\n", afc_bw);
          return -EINVAL;
        }
    }

  /* Update local */

  dev->fskook.afc_bw = afc_bw;
  return OK;
}

/****************************************************************************
 * Name: sx127x_fskook_seq_start
 ****************************************************************************/

static void sx127x_fskook_seq_start(FAR struct sx127x_dev_s *dev, bool state)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  /* Lock SPI */

  sx127x_lock(dev->spi);

  if (state == true)
    {
      /* Start sequencer */

      sx127x_modregbyte(dev, SX127X_FOM_SEQCFG1,
                        SX127X_FOM_SEQCFG1_SEQSTART, 0);
    }
  else
    {
      /* Stop sequencer */

      sx127x_modregbyte(dev, SX127X_FOM_SEQCFG1,
                        SX127X_FOM_SEQCFG1_SEQSTOP, 0);
    }

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Store sequencer state */

  dev->fskook.seqon = state;
}

/****************************************************************************
 * Name: sx127x_fskook_seq_init
 *
 * Description:
 *   Initialize FSK/OOK sequencer.
 *   This can be used to automate transitions between operation modes and
 *   thus further reduce energy consumption.
 *
 ****************************************************************************/

static int sx127x_fskook_seq_init(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  uint8_t seq1 = 0;
  uint8_t seq2 = 0;

  /* Need sleep mode or standby mode */

  if (dev->opmode > SX127X_OPMODE_STANDBY)
    {
      sx127x_opmode_set(dev, SX127X_OPMODE_STANDBY);
    }

  /* Nothing here */

  seq1 = 0;
  seq2 = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Write registers */

  sx127x_writeregbyte(dev, SX127X_FOM_SEQCFG1, seq1);
  sx127x_writeregbyte(dev, SX127X_FOM_SEQCFG2, seq2);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
  return OK;
}

/****************************************************************************
 * Name: sx127x_fskook_syncword_get
 ****************************************************************************/

static void sx127x_fskook_syncword_get(FAR struct sx127x_dev_s *dev,
                                       FAR uint8_t *sw, FAR uint8_t *len)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  wlerr("sx127x_fskook_syncword_get not implemented yet\n");
}

/****************************************************************************
 * Name: sx127x_fskook_syncword_set
 *
 * Description:
 *   Set SyncWord for FSK/OOK
 *
 ****************************************************************************/

static int sx127x_fskook_syncword_set(FAR struct sx127x_dev_s *dev,
                                      FAR uint8_t *sw, uint8_t len)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  uint8_t setbits = 0;
  uint8_t clrbits = 0;
  uint8_t offset  = 0;
  int     i       = 0;

  if (len > SX127X_FOM_SYNCSIZE_MAX)
    {
      wlerr("Unsupported sync word length %d!", len);
      return -EINVAL;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  if (len == 0)
    {
      /* Disable sync word generation and detection */

      clrbits = (SX127X_FOM_SYNCCFG_SYNCSIZE_MASK |
                 SX127X_FOM_SYNCCFG_SYNCON);
      setbits = 0;

      sx127x_modregbyte(dev, SX127X_FOM_SYNCCFG, setbits, clrbits);
    }
  else
    {
      /* Configure sync word length */

      clrbits = SX127X_FOM_SYNCCFG_SYNCSIZE_MASK;
      setbits = (SX127X_FOM_SYNCCFG_SYNCON |
                 SX127X_FOM_SYNCCFG_SYNCSIZE(len - 1));

      sx127x_modregbyte(dev, SX127X_FOM_SYNCCFG, setbits, clrbits);

      /* Write sync words */

      for (i = 0; i < len; i += 1)
        {
          offset = SX127X_FOM_SYNCVAL1 + i;
          sx127x_writeregbyte(dev, offset, sw[i]);
        }
    }

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
  return OK;
}

/****************************************************************************
 * Name: sx127x_fskook_init
 *
 * Description:
 *   Initialization specific for FSK/OOK modulation
 *
 ****************************************************************************/

static void sx127x_fskook_init(FAR struct sx127x_dev_s *dev)
{
  uint8_t setbits = 0;
  uint8_t clrbits = 0;
  uint8_t syncword[] =
  {
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01
  };

  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  /* Set FDEV */

  sx127x_fskook_fdev_set(dev, SX127X_FDEV_DEFAULT);

  /* Set bitrate */

  sx127x_fskook_bitrate_set(dev, SX127X_FOM_BITRATE_DEFAULT);

  /* Configure sequencer
   * WARNING: sequencer is OFF for now!
   */

  sx127x_fskook_seq_init(dev);
  sx127x_fskook_seq_start(dev, false);

  /* Configure Sync Word
   * REVISIT: FSK communication doesn't work if syncword is disabled!
   */

  sx127x_fskook_syncword_set(dev, syncword, 8);

  /* Configure bandwidth */

  sx127x_fskook_rxbw_set(dev, SX127X_FSKOOK_RXBW_DEFAULT);
  sx127x_fskook_afcbw_set(dev, SX127X_FSKOOK_AFCBW_DEFAULT);

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Configure packet mode settings 1:
   *   - fixlen
   *   - RX/TX CRC
   */

  setbits  = 0;
  setbits |= dev->fskook.fixlen == true ?  0 : SX127X_FOM_PKTCFG1_PCKFORMAT;
  setbits |= dev->crcon == true ? SX127X_FOM_PKTCFG1_CRCON : 0;
  clrbits  = SX127X_FOM_PKTCFG1_PCKFORMAT | SX127X_FOM_PKTCFG1_CRCON;

  /* Write packet mode settings 1 */

  sx127x_modregbyte(dev, SX127X_FOM_PKTCFG1, setbits, clrbits);

  /* Configure packet mode settings 2:
   *   - packet mode on
   */

  setbits  = 0;
  setbits |= SX127X_FOM_PKTCFG2_DATAMODE;
  clrbits  = 0;

  /* Write packet mode settings 2 */

  sx127x_modregbyte(dev, SX127X_FOM_PKTCFG2, setbits, clrbits);

  /* Configure PARAMP register */

  setbits = (SX127X_FSKOOK_SHAPING_DEFAULT | SX127X_FSKOOK_PARAMP_DEFAULT);
  clrbits = (SX127X_CMN_PARAMP_PARAMP_MASK | SX127X_CMN_PARAMP_SHAPING_MASK);

  /* Write PARAMP register */

  sx127x_modregbyte(dev, SX127X_CMN_PARAMP, setbits, clrbits);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
}

/****************************************************************************
 * Name: sx127x_fskook_rssi_get
 *
 * Description:
 *   Get current RSSI for FSK/OOK modem
 *
 ****************************************************************************/

static int16_t sx127x_fskook_rssi_get(FAR struct sx127x_dev_s *dev)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get register value */

  regval = sx127x_readregbyte(dev, SX127X_FOM_RSSIVAL);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Return decoded RSSI value */

  return SX127X_FOM_RSSIVAL_GET(regval);
}

/****************************************************************************
 * Name: sx127x_fskook_fdev_set
 *
 * Description:
 *   Set frequency deviation
 *
 ****************************************************************************/

static int sx127x_fskook_fdev_set(FAR struct sx127x_dev_s *dev,
                                  uint32_t freq)
{
  uint32_t fdev = 0;

  /* Only for FSK modulation */

  if (dev->modulation != SX127X_MODULATION_FSK)
    {
      return -EINVAL;
    }

  if (freq == dev->fskook.fdev)
    {
      return OK;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get FDEV value */

  fdev = SX127X_FDEV_FROM_FREQ(freq);

  /* Write FDEV MSB */

  sx127x_writeregbyte(dev, SX127X_FOM_FDEVMSB, SX127X_FOM_FDEV_MSB(fdev));

  /* Write FDEV LSB */

  sx127x_writeregbyte(dev, SX127X_FOM_FDEVLSB, SX127X_FOM_FDEV_LSB(fdev));

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Update local variable */

  dev->fskook.fdev = freq;
  return OK;
}

/****************************************************************************
 * Name: sx127x_fskook_bitrate_set
 *
 * Description:
 *   Set bitrate for FSK/OOK modulation
 *
 ****************************************************************************/

static int sx127x_fskook_bitrate_set(FAR struct sx127x_dev_s *dev,
                                     uint32_t bitrate)
{
  uint32_t br = 0;

  if (bitrate == dev->fskook.bitrate)
    {
      return OK;
    }

  /* Get bitrate register value */

  br = SX127X_FXOSC / bitrate;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Set fractial part to 0 */

  sx127x_writeregbyte(dev, SX127X_FOM_BITRATEFRAC, 0);

  /* Write MSB */

  sx127x_writeregbyte(dev, SX127X_FOM_BITRATEMSB,
                      SX127X_FOM_BITRATE_MSB(br));

  /* Write LSB */

  sx127x_writeregbyte(dev, SX127X_FOM_BITRATELSB,
                      SX127X_FOM_BITRATE_LSB(br));

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Update local variable */

  dev->fskook.bitrate = bitrate;
  return OK;
}

/****************************************************************************
 * Name: sx127x_fskook_preamble_set
 *
 * Description:
 *   Set preamble for FSK/OOK modulation
 *
 ****************************************************************************/

static void sx127x_fskook_preamble_set(FAR struct sx127x_dev_s *dev,
                                       uint32_t len)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  if (len == 0)
    {
      /* Disable detector */

      regval = 0;
      sx127x_writeregbyte(dev, SX127X_FOM_PREDET, regval);
    }
  else
    {
      /* Configure preamble length */

      regval = SX127X_FOM_PRE_MSB(len);
      sx127x_writeregbyte(dev, SX127X_FOM_PREMSB, regval);
      regval = SX127X_FOM_PRE_LSB(len);
      sx127x_writeregbyte(dev, SX127X_FOM_PRELSB, regval);

      /* Configure preamble polarity to 0xAA */

      regval = SX127X_FOM_SYNCCFG_PREPOL;
      sx127x_modregbyte(dev, SX127X_FOM_SYNCCFG, regval, 0);

      /* Configure and enable preamble detector:
       *   - tolerance = 10
       *   - detector size = 2B
       */

      regval = (SX127X_FOM_PREDET_ON | SX127X_FOM_PREDET_SIZE_2B |
                  SX127X_FOM_PREDET_TOL(10));
      sx127x_writeregbyte(dev, SX127X_FOM_PREDET, regval);
    }

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
}

/****************************************************************************
 * Name: sx127x_fskook_preamble_get
 *
 * Description:
 *   Get current preamble configuration for FSK/OOK
 *
 ****************************************************************************/

static int sx127x_fskook_preamble_get(FAR struct sx127x_dev_s *dev)
{
  wlerr("sx127x_fskook_preamble_get\n");
  return 0;
}

#endif /* CONFIG_LPWAN_SX127X_FSKOOK */

#ifdef CONFIG_LPWAN_SX127X_LORA

/****************************************************************************
 * Name: sx127x_lora_opmode_init
 *
 * Description:
 *   Initialize operation mode for LORA
 *
 ****************************************************************************/

static int sx127x_lora_opmode_init(FAR struct sx127x_dev_s *dev,
                                   uint8_t opmode)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  uint8_t dio0map = 0;
  uint8_t setbits = 0;
  uint8_t clrbits = 0;
  int     ret     = OK;

  sx127x_lock(dev->spi);

  /* Get mode specific configuration */

  switch (opmode)
    {
      case SX127X_OPMODE_SLEEP:
      case SX127X_OPMODE_STANDBY:
      case SX127X_OPMODE_FSRX:
      case SX127X_OPMODE_FSTX:
        {
          break;
        }

      case SX127X_OPMODE_TX:
        {
          /* DIO0 is TX DONE */

          dio0map = SX127X_LRM_DIOMAP1_DIO0_TXDONE;

          /* Full buffer for TX */

          sx127x_writeregbyte(dev, SX127X_LRM_TXBASE, 0);

          /* Reset FIFO pointer */

          sx127x_writeregbyte(dev, SX127X_LRM_ADDRPTR, 0);
          break;
        }

      case SX127X_OPMODE_RX:
      case SX127X_OPMODE_RXSINGLE:
        {
          /* DIO0 is RX DONE */

          dio0map = SX127X_LRM_DIOMAP1_DIO0_RXDONE;

          /* Full buffer for RX */

          sx127x_writeregbyte(dev, SX127X_LRM_RXBASE, 0);

          /* Reset FIFO pointer */

          sx127x_writeregbyte(dev, SX127X_LRM_ADDRPTR, 0);
          break;
        }

      case SX127X_OPMODE_CAD:
        {
          /* DIO0 is CAD DONE */

          dio0map = SX127X_LRM_DIOMAP1_DIO0_CADDONE;
          break;
        }

      default:
        {
          wlerr("ERROR: invalid mode %d\n", opmode);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Configure DIO0 pin */

  setbits = dio0map;
  clrbits = SX127X_CMN_DIOMAP1_DIO0_MASK;
  sx127x_modregbyte(dev, SX127X_CMN_DIOMAP1, setbits, clrbits);

errout:
  sx127x_unlock(dev->spi);
  return ret;
}

/****************************************************************************
 * Name: sx127x_lora_opmode_set
 *
 * Description:
 *   Set operation mode for LORA
 *
 ****************************************************************************/

static int sx127x_lora_opmode_set(FAR struct sx127x_dev_s *dev,
                                  uint8_t opmode)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  int ret = OK;

  sx127x_lock(dev->spi);

  switch (opmode)
    {
      case SX127X_OPMODE_SLEEP:
      case SX127X_OPMODE_STANDBY:
      case SX127X_OPMODE_FSRX:
      case SX127X_OPMODE_FSTX:
      case SX127X_OPMODE_TX:
      case SX127X_OPMODE_RX:
      case SX127X_OPMODE_RXSINGLE:
      case SX127X_OPMODE_CAD:
        {
          /* Do nothing */

          break;
        }

      default:
        {
          wlerr("ERROR: invalid LORA mode %d\n", opmode);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Update mode */

  sx127x_modregbyte(dev, SX127X_CMN_OPMODE,
                    ((opmode - 1) << SX127X_CMN_OPMODE_MODE_SHIFT),
                    SX127X_CMN_OPMODE_MODE_MASK);

  /* Wait for mode ready. REVISIT: do we need this ? */

  nxsig_usleep(250);

errout:
  sx127x_unlock(dev->spi);
  return ret;
}

/****************************************************************************
 * Name: sx127x_lora_syncword_get
 ****************************************************************************/

static void sx127x_lora_syncword_get(FAR struct sx127x_dev_s *dev,
                                     FAR uint8_t *sw, FAR uint8_t *len)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  wlerr("sx127x_lora_syncword_get not implemented yet\n");
}

/****************************************************************************
 * Name: sx127x_lora_syncword_set
 *
 * Description:
 *   Set SyncWord for LORA
 *
 ****************************************************************************/

static int sx127x_lora_syncword_set(FAR struct sx127x_dev_s *dev,
                                    FAR uint8_t *sw, uint8_t len)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  if (len != 1)
    {
      wlerr("LORA support sync word with len = 1 but len = %d\n", len);
      return -EINVAL;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Write sync word */

  sx127x_writeregbyte(dev, SX127X_LRM_SYNCWORD, sw[0]);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
  return OK;
}

/****************************************************************************
 * Name: sx127x_lora_bw_set
 *
 * Description:
 *   Configure LORA bandwidth
 *
 ****************************************************************************/

static int sx127x_lora_bw_set(FAR struct sx127x_dev_s *dev, uint8_t bw)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  uint8_t clrbits = 0;
  uint8_t setbits = 0;

  if (bw == dev->lora.bw)
    {
      return OK;
    }

  switch (bw)
    {
      case LORA_BANDWIDTH_7P8KHZ:
      case LORA_BANDWIDTH_10P4KHZ:
      case LORA_BANDWIDTH_15P6KHZ:
      case LORA_BANDWIDTH_20P8KHZ:
      case LORA_BANDWIDTH_31P2KHZ:
      case LORA_BANDWIDTH_41P4KHZ:
      case LORA_BANDWIDTH_62P5KHZ:
      case LORA_BANDWIDTH_125KHZ:
      case LORA_BANDWIDTH_250KHZ:
        {
          /* Lock SPI */

          sx127x_lock(dev->spi);

          setbits = bw << SX127X_LRM_MDMCFG1_BW_SHIFT;
          clrbits = SX127X_LRM_MDMCFG1_BW_MASK;
          sx127x_modregbyte(dev, SX127X_LRM_MDMCFG1, setbits, clrbits);

          /* Unlock SPI */

          sx127x_unlock(dev->spi);
          break;
        }

      default:
        {
          wlerr("Unsupported bandwidth %d\n", bw);
          return -EINVAL;
        }
    }

  dev->lora.bw = bw;
  return OK;
}

/****************************************************************************
 * Name: sx127x_lora_cr_set
 *
 * Description:
 *   Configure LORA coding rate
 *
 ****************************************************************************/

static int sx127x_lora_cr_set(FAR struct sx127x_dev_s *dev, uint8_t cr)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  uint8_t clrbits = 0;
  uint8_t setbits = 0;

  if (cr == dev->lora.cr)
    {
      return OK;
    }

  switch (cr)
    {
      case LORA_CR_4d5:
      case LORA_CR_4d6:
      case LORA_CR_4d7:
      case LORA_CR_4d8:
        {
          /* Lock SPI */

          sx127x_lock(dev->spi);

          setbits = cr << SX127X_LRM_MDMCFG1_CDRATE_SHIFT;
          clrbits = SX127X_LRM_MDMCFG1_CDRATE_MASK;
          sx127x_modregbyte(dev, SX127X_LRM_MDMCFG1, setbits, clrbits);

          /* Unlock SPI */

          sx127x_unlock(dev->spi);
          break;
        }

      default:
        {
          wlerr("Unsupported code rate %d\n", cr);
          return -EINVAL;
        }
    }

  dev->lora.cr = cr;
  return OK;
}

/****************************************************************************
 * Name: sx127x_lora_sf_set
 *
 * Description:
 *   Configure LORA SF
 *
 ****************************************************************************/

static int sx127x_lora_sf_set(FAR struct sx127x_dev_s *dev, uint8_t sf)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  uint8_t dopt    = SX127X_LRM_DETECTOPT_DO_SF7SF12;
  uint8_t dthr    = SX127X_LRM_DETECTTHR_SF7SF12;
  uint8_t setbits = 0;
  uint8_t clrbits = 0;

  if (dev->lora.sf == sf)
    {
      return OK;
    }

  /* Special configuration required by SF6 (highest data rate transmission):
   *   - implicit header mode ON
   *   - Detection optimize for SF6
   *   - Detection threshold for SF6
   */

  if (dev->lora.sf == 6)
    {
      if (dev->lora.implicthdr == true)
        {
          wlerr("SF6 needs implicit header ON!\n");
          return -EINVAL;
        }

      dopt = SX127X_LRM_DETECTOPT_DO_SF6;
      dthr = SX127X_LRM_DETECTTHR_SF6;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Write spreading factor */

  clrbits = SX127X_LRM_MDMCFG2_SPRFACT_MASK;
  setbits = (sf << SX127X_LRM_MDMCFG2_SPRFACT_SHIFT);
  sx127x_modregbyte(dev, SX127X_LRM_MDMCFG2, setbits, clrbits);

  sx127x_writeregbyte(dev, SX127X_LRM_DETECTOPT, dopt);
  sx127x_writeregbyte(dev, SX127X_LRM_DETECTTHR, dthr);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Update local variable */

  dev->lora.sf = sf;
  return OK;
}

/****************************************************************************
 * Name: sx127x_lora_implicthdr_set
 *
 * Description:
 *  Enable/disable implicit header for LORA
 *
 ****************************************************************************/

static int sx127x_lora_implicthdr_set(FAR struct sx127x_dev_s *dev,
                                      bool enable)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  uint8_t setbits = 0;
  uint8_t clrbits = 0;

  if (dev->lora.sf == 6 && enable == false)
    {
      wlerr("SF=6 requires implicit header ON\n");
      return -EINVAL;
    }

  if (enable == dev->lora.implicthdr)
    {
      return OK;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Modify MDMCFG1 register */

  clrbits = 0;
  setbits = SX127X_LRM_MDMCFG1_IMPLHDRON;

  sx127x_modregbyte(dev, SX127X_LRM_MDMCFG1, setbits, clrbits);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Update local variable */

  dev->lora.implicthdr = enable;
  return OK;
}

/****************************************************************************
 * Name: sx127x_lora_init
 *
 * Description:
 *   Initialization specific for LORA modulation
 *
 ****************************************************************************/

static void sx127x_lora_init(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  uint8_t setbits = 0;
  uint8_t clrbits = 0;

  /* Configure sync word for LORA modulation */

  setbits = SX127X_LRM_SYNCWORD_DEFAULT;
  sx127x_lora_syncword_set(dev, &setbits, 1);

  /* Configure bandwidth */

  sx127x_lora_bw_set(dev, SX127X_LRM_BW_DEFAULT);

  /* Configure coding rate */

  sx127x_lora_cr_set(dev, SX127X_LRM_CR_DEFAULT);

  /* TODO: Configure frequency hopping */

  /* sx127x_lora_fhop_set(dev,) */

  /* Configure spreading factor */

  sx127x_lora_sf_set(dev, SX127X_LRM_SF_DEFAULT);

  /* Configure LORA header */

  sx127x_lora_implicthdr_set(dev, CONFIG_LPWAN_SX127X_LORA_IMPHEADER);

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Configure maximum payload */

  sx127x_writeregbyte(dev, SX127X_LRM_PAYLOADMAX,
                      SX127X_LRM_PAYLOADMAX_DEFAULT);

  /* Modem PHY config 2:
   *   - RXCRCON
   *     NOTE: this works differently for implicit header and explicit header
   *   - packet mode
   */

  setbits = (dev->crcon == true ? SX127X_LRM_MDMCFG2_RXCRCON : 0);
  clrbits = (SX127X_LRM_MDMCFG2_TXCONT | SX127X_LRM_MDMCFG2_RXCRCON);
  sx127x_modregbyte(dev, SX127X_LRM_MDMCFG2, setbits, clrbits);

  /* Invert I and Q signals if configured */

  setbits = (dev->lora.invert_iq == true ? SX127X_LRM_INVERTIQ_IIQ : 0);
  clrbits = SX127X_LRM_INVERTIQ_IIQ;
  sx127x_modregbyte(dev, SX127X_LRM_INVERTIQ, setbits, clrbits);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
}

/****************************************************************************
 * Name: sx127x_lora_rssi_correct
 *
 * Description:
 *   Correct RSSI for LORA radio according to datasheet
 *
 ****************************************************************************/

static int16_t sx127x_lora_rssi_correct(FAR struct sx127x_dev_s *dev,
                                        uint32_t freq, int8_t snr,
                                        uint8_t regval)
{
  int16_t offset = 0;
  int16_t ret    = 0;

  /* Ignore SNR if >= 0 */

  if (snr >= 0)
    {
      snr = 0;
    }

  /* RSSI offset depends on RF frequency */

  offset = (freq > SX127X_HFBAND_THR ?
            SX127X_LRM_RSSIVAL_HF_OFFSET : SX127X_LRM_RSSIVAL_LF_OFFSET);

  /* Get corrected RSSI value */

  ret = regval + offset + snr;

  return ret;
}

/****************************************************************************
 * Name: sx127x_lora_snr_get
 *
 * Description:
 *   Get estimation of SNR on last packet received for LORA modem
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
static int8_t sx127x_lora_snr_get(FAR struct sx127x_dev_s *dev)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get register value */

  regval = sx127x_readregbyte(dev, SX127X_LRM_PKTSNR);

  /* Get SNR */

  regval = regval / 4;

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Return corrected RSSI */

  return (int8_t)regval;
}

/****************************************************************************
 * Name: sx127x_lora_pckrssi_get
 *
 * Description:
 *   Get RSSI of the last received LORA packet
 *
 ****************************************************************************/

static int16_t sx127x_lora_pckrssi_get(FAR struct sx127x_dev_s *dev,
                                       int8_t snr)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get register value */

  regval = sx127x_readregbyte(dev, SX127X_LRM_PKTRSSI);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Return corrected RSSI */

  return sx127x_lora_rssi_correct(dev, dev->freq, snr, regval);
}
#endif

/****************************************************************************
 * Name: sx127x_lora_rssi_get
 *
 * Description:
 *   Get current RSSI for LORA modem
 *
 ****************************************************************************/

static int16_t sx127x_lora_rssi_get(FAR struct sx127x_dev_s *dev)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get register value */

  regval = sx127x_readregbyte(dev, SX127X_LRM_RSSIVAL);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Return corrected RSSI */

  return sx127x_lora_rssi_correct(dev, dev->freq, 0, regval);
}

/****************************************************************************
 * Name: sx127x_lora_preamble_set
 *
 * Description:
 *   Set preamble for LORA modulation
 *
 ****************************************************************************/

static void sx127x_lora_preamble_set(FAR struct sx127x_dev_s *dev,
                                     uint32_t len)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Configure preamble len */

  regval = SX127X_LRM_PRE_MSB(len);
  sx127x_writeregbyte(dev, SX127X_LRM_PREMSB, regval);
  regval = SX127X_LRM_PRE_LSB(len);
  sx127x_writeregbyte(dev, SX127X_LRM_PRELSB, regval);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
}

/****************************************************************************
 * Name: sx127x_lora_preamble_get
 *
 * Description:
 *
 ****************************************************************************/

static int sx127x_lora_preamble_get(FAR struct sx127x_dev_s *dev)
{
  wlerr("sx127x_lora_preamble_get\n");
  return 0;
}

#endif /* CONFIG_LPWAN_SX127X_LORA */

/****************************************************************************
 * Name: sx127x_syncword_get
 ****************************************************************************/

static void sx127x_syncword_get(FAR struct sx127x_dev_s *dev,
                                FAR uint8_t *sw, FAR uint8_t *len)
{
  dev->ops.syncword_get(dev, sw, len);
}

/****************************************************************************
 * Name: sx127x_syncword_set
 ****************************************************************************/

static int sx127x_syncword_set(FAR struct sx127x_dev_s *dev,
                               FAR uint8_t *sw, uint8_t len)
{
  return dev->ops.syncword_set(dev, sw, len);
}

/****************************************************************************
 * Name: sx127x_modulation_get
 *
 * Description:
 *   Get current radio modulation
 *
 ****************************************************************************/

static uint8_t sx127x_modulation_get(FAR struct sx127x_dev_s *dev)
{
  uint8_t regval = 0;
  uint8_t ret    = 0;

  /* Get OPMODE register */

  regval = sx127x_readregbyte(dev, SX127X_CMN_OPMODE);

  if (regval & SX127X_CMN_OPMODE_LRMODE)
    {
      /* LORA modulation */

      ret = SX127X_MODULATION_LORA;
    }
  else
    {
      /* FSK or OOK modulation */

      ret = (regval & SX127X_CMN_OPMODE_MODTYPE_FSK ?
             SX127X_MODULATION_FSK : SX127X_MODULATION_OOK);
    }

  return ret;
}

/****************************************************************************
 * Name: sx127x_ops_set
 ****************************************************************************/

static void sx127x_ops_set(FAR struct sx127x_dev_s *dev, uint8_t modulation)
{
  if (modulation <= SX127X_MODULATION_OOK)
    {
      /* NOTE: we need opmode_init and opmode_set for FSK/OOK even if
       * support for these modulations is disabled!
       */

      dev->ops.opmode_init  = sx127x_fskook_opmode_init;
      dev->ops.opmode_set   = sx127x_fskook_opmode_set;
#ifdef CONFIG_LPWAN_SX127X_FSKOOK
      dev->ops.init         = sx127x_fskook_init;
      dev->ops.isr0_process = sx127x_fskook_isr0_process;
      dev->ops.preamble_set = sx127x_fskook_preamble_set;
      dev->ops.preamble_get = sx127x_fskook_preamble_get;
      dev->ops.rssi_get     = sx127x_fskook_rssi_get;
      dev->ops.syncword_set = sx127x_fskook_syncword_set;
      dev->ops.syncword_get = sx127x_fskook_syncword_get;
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
      dev->ops.send         = sx127x_fskook_send;
#endif
#ifdef CONFIG_DEBUG_WIRELESS_INFO
      dev->ops.dumpregs     = sx127x_fskook_dumpregs;
#endif
#endif /* CONFIG_LPWAN_SX127X_FSKOOK */
    }

#ifdef CONFIG_LPWAN_SX127X_LORA
  if (modulation == SX127X_MODULATION_LORA)
    {
      dev->ops.init         = sx127x_lora_init;
      dev->ops.isr0_process = sx127x_lora_isr0_process;
      dev->ops.opmode_init  = sx127x_lora_opmode_init;
      dev->ops.opmode_set   = sx127x_lora_opmode_set;
      dev->ops.preamble_set = sx127x_lora_preamble_set;
      dev->ops.preamble_get = sx127x_lora_preamble_get;
      dev->ops.rssi_get     = sx127x_lora_rssi_get;
      dev->ops.syncword_set = sx127x_lora_syncword_set;
      dev->ops.syncword_get = sx127x_lora_syncword_get;
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
      dev->ops.send         = sx127x_lora_send;
#endif
#ifdef CONFIG_DEBUG_WIRELESS_INFO
      dev->ops.dumpregs     = sx127x_lora_dumpregs;
#endif
    }
#endif /* CONFIG_LPWAN_SX127X_LORA */
}

/****************************************************************************
 * Name: sx127x_modulation_init
 ****************************************************************************/

static void sx127x_modulation_init(FAR struct sx127x_dev_s *dev)
{
  dev->ops.init(dev);

  /* Configure preamble */

  sx127x_preamble_set(dev, CONFIG_LPWAN_SX127X_PREAMBLE_DEFAULT);

  /* Dump registers after initial configuration */

  sx127x_dumpregs(dev);
}

/****************************************************************************
 * Name: sx127x_modulation_set
 *
 * Description:
 *   Set radio modulation and configure
 *
 ****************************************************************************/

static int sx127x_modulation_set(FAR struct sx127x_dev_s *dev,
                                 uint8_t modulation)
{
  uint8_t setbits = 0;
  uint8_t clrbits = 0;

  wlinfo("modulation_set %d->%d\n", dev->modulation, modulation);

  if (modulation == dev->modulation)
    {
      return OK;
    }

  /* Modulation can be only changed in SLEEP mode */

  sx127x_opmode_set(dev, SX127X_OPMODE_SLEEP);

  /* Change modulation */

  switch (modulation)
    {
#ifdef CONFIG_LPWAN_SX127X_FSKOOK
      case SX127X_MODULATION_FSK:
        {
          clrbits = (SX127X_CMN_OPMODE_LRMODE |
                     SX127X_CMN_OPMODE_MODTYPE_MASK);
          setbits = SX127X_CMN_OPMODE_MODTYPE_FSK;

          break;
        }

      case SX127X_MODULATION_OOK:
        {
          clrbits = (SX127X_CMN_OPMODE_LRMODE |
                     SX127X_CMN_OPMODE_MODTYPE_MASK);
          setbits = SX127X_CMN_OPMODE_MODTYPE_OOK;

          break;
        }
#endif /* CONFIG_LPWAN_SX127X_FSKOOK */

#ifdef CONFIG_LPWAN_SX127X_LORA
      case SX127X_MODULATION_LORA:
        {
          clrbits = SX127X_CMN_OPMODE_MODTYPE_MASK;
          setbits = SX127X_CMN_OPMODE_LRMODE;

          break;
        }
#endif /* CONFIG_LPWAN_SX127X_LORA */

      default:
        {
          wlerr("ERROR: Unsupported modulation type %d\n", modulation);
          return -EINVAL;
        }
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Modify register */

  sx127x_modregbyte(dev, SX127X_CMN_OPMODE, setbits, clrbits);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Initialization specific for modulation and initialize private ops */

  sx127x_ops_set(dev, modulation);

  /* Update local variable */

  dev->modulation = modulation;

  /* Initial configuration */

  sx127x_modulation_init(dev);
  return OK;
}

/****************************************************************************
 * Name: sx127x_fskook_rssi_get
 ****************************************************************************/

static int16_t sx127x_rssi_get(FAR struct sx127x_dev_s *dev)
{
  return dev->ops.rssi_get(dev);
}

/****************************************************************************
 * Name: sx127x_channel_scan
 *
 * Description:
 *
 ****************************************************************************/

static bool sx127x_channel_scan(FAR struct sx127x_dev_s *dev,
                                FAR struct sx127x_chanscan_ioc_s *chanscan)
{
  struct timespec tstart;
  struct timespec tnow;
  bool    ret  = true;
  int16_t rssi = 0;
  int16_t max = 0;
  int16_t min = 0;

  /* Set frequency */

  sx127x_frequency_set(dev, chanscan->freq);

  /* Set mode to RX */

  sx127x_opmode_set(dev, SX127X_OPMODE_RX);

  /* Get start time */

  clock_systime_timespec(&tstart);

  /* Initialize min/max */

  max = INT16_MIN;
  min = INT16_MAX;

  do
    {
      /* Get time now */

      clock_systime_timespec(&tnow);

      /* Check RSSI */

      rssi = dev->ops.rssi_get(dev);

      /* Store maximum/minimum value */

      if (rssi > max)
        {
          max = rssi;
        }
      else if (rssi < min)
        {
          min = rssi;
        }

      if (rssi > chanscan->rssi_thr)
        {
          ret = false;
          break;
        }

      /* Wait some time */

      nxsig_usleep(1000);
    }
  while (tstart.tv_sec + chanscan->stime > tnow.tv_sec);

  /* Set mode to STANDBY */

  sx127x_opmode_set(dev, SX127X_OPMODE_STANDBY);

  /* Store limit values */

  chanscan->rssi_max = max;
  chanscan->rssi_min = min;

  /* Store return value in struct */

  chanscan->free = ret;
  return ret;
}

/****************************************************************************
 * Name: sx127x_random_get
 *
 * Description:
 *
 ****************************************************************************/

static uint32_t sx127x_random_get(FAR struct sx127x_dev_s *dev)
{
  wlerr("sx127x_random_get not implemented yet\n");
  return 0;
}

/****************************************************************************
 * Name: sx127x_frequency_get
 *
 * Description:
 *   Get RF carrier frequency
 *
 ****************************************************************************/

static uint32_t sx127x_frequency_get(FAR struct sx127x_dev_s *dev)
{
  wlerr("sx127x_frequency_get\n");
  return 0;
}

/****************************************************************************
 * Name: sx127x_frequency_set
 *
 * Description:
 *   Set RF carrier frequency for LORA and FSK/OOK modulation
 *
 ****************************************************************************/

static int sx127x_frequency_set(FAR struct sx127x_dev_s *dev, uint32_t freq)
{
  uint32_t frf = 0;
  int      ret = OK;

  wlinfo("frequency %" PRId32 "->%" PRId32 "\n", dev->freq, freq);

  if (freq == dev->freq)
    {
      return OK;
    }

  /* REVISIT: needs sleep/standby mode ? */

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get FRF value */

  frf = SX127X_FRF_FROM_FREQ(freq);

  /* Write FRF MSB */

  sx127x_writeregbyte(dev, SX127X_CMN_FRFMSB, SX127X_CMN_FRF_MSB(frf));

  /* Write FRF MID */

  sx127x_writeregbyte(dev, SX127X_CMN_FRFMID, SX127X_CMN_FRF_MID(frf));

  /* Write FRF LSB */

  sx127x_writeregbyte(dev, SX127X_CMN_FRFLSB, SX127X_CMN_FRF_LSB(frf));

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Update local variable */

  dev->freq = freq;

  /* Call board-specific LF/HF configuration */

  ret = dev->lower->freq_select(freq);
  if (ret < 0)
    {
      wlerr("Board-specific freq_select failed %d!\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sx127x_power_set
 ****************************************************************************/

static int sx127x_power_set(FAR struct sx127x_dev_s *dev, int8_t power)
{
#ifndef CONFIG_LPWAN_SX127X_TXSUPPORT
  return -ENOSYS;
#else
  bool pa_select  = false;
  bool pa_dac     = false;
  uint8_t setbits = 0;
  uint8_t clrbits = 0;
  int ret         = OK;

  if (dev->power == power)
    {
      return OK;
    }

  /* PA BOOST configuration */

  if (power > SX127X_PASELECT_POWER || dev->pa_force == true)
    {
      pa_select = true;
    }

  /* High power PA BOOST */

  if (power >= 20)
    {
      pa_dac = true;
    }

  /* Saturate power output */

  if (pa_select == true)
    {
      if (pa_dac == true)
        {
          if (power < 5)
            {
              power = 5;
            }
          else if (power > 20)
            {
              power = 20;
            }
        }
      else
        {
          if (power < 2)
            {
              power = 2;
            }
          else if (power > 17)
            {
              power = 17;
            }
        }
    }
  else
    {
      if (power < -1)
        {
          power = -1;
        }
      else if (power > 14)
        {
          power = 14;
        }
    }

  wlinfo("power %d->%d, pa=%d, dac=%d\n",
         dev->power, power, pa_select, pa_dac);

  sx127x_lock(dev->spi);

  if (pa_select == true)
    {
      if (pa_dac == true)
        {
          /* Enable high power on PA_BOOST */

          sx127x_writeregbyte(dev, SX127X_CMN_PADAC, SX127X_CMN_PADAC_BOOST);

          /* Configure output power */

          setbits = (power - 5) << SX127X_CMN_PACFG_OUTPOWER_SHIFT;
          clrbits = SX127X_CMN_PACFG_OUTPOWER_MASK;

          sx127x_modregbyte(dev, SX127X_CMN_PACFG, setbits, clrbits);
        }
      else
        {
          /* Disable high power on PA_BOOST */

          sx127x_writeregbyte(dev, SX127X_CMN_PADAC,
                              SX127X_CMN_PADAC_DEFAULT);

          /* Configure output power */

          setbits = (power - 2) << SX127X_CMN_PACFG_OUTPOWER_SHIFT;
          clrbits = SX127X_CMN_PACFG_OUTPOWER_MASK;

          sx127x_modregbyte(dev, SX127X_CMN_PACFG, setbits, clrbits);
        }

      /* Enable PA BOOST output */

      sx127x_modregbyte(dev, SX127X_CMN_PACFG, SX127X_CMN_PACFG_PASELECT, 0);
    }
  else
    {
      /* Configure output power and max power to 13.8 dBm */

      setbits = ((power + 1) << SX127X_CMN_PACFG_OUTPOWER_SHIFT);
      setbits |= (5 << SX127X_CMN_PACFG_MAXPOWER_SHIFT);
      clrbits = (SX127X_CMN_PACFG_OUTPOWER_MASK |
                 SX127X_CMN_PACFG_MAXPOWER_SHIFT);

      sx127x_modregbyte(dev, SX127X_CMN_PACFG, setbits, clrbits);

      /* Enable RFO output */

      sx127x_modregbyte(dev, SX127X_CMN_PACFG, 0, SX127X_CMN_PACFG_PASELECT);
    }

  sx127x_unlock(dev->spi);

  /* Call board-specific logic */

  ret = dev->lower->pa_select(pa_select);
  if (ret < 0)
    {
      wlerr("Board-specific pa_select failed %d!\n", ret);
    }

  /* Update local variable */

  dev->power = power;
  return ret;
#endif
}

/****************************************************************************
 * Name: sx127x_power_get
 ****************************************************************************/

static int8_t sx127x_power_get(FAR struct sx127x_dev_s *dev)
{
#ifndef CONFIG_LPWAN_SX127X_TXSUPPORT
  return 0;
#else
  return dev->power;
#endif
}

/****************************************************************************
 * Name: sx127x_preamble_set
 ****************************************************************************/

static void sx127x_preamble_set(FAR struct sx127x_dev_s *dev, uint32_t len)
{
  dev->ops.preamble_set(dev, len);
}

/****************************************************************************
 * Name: sx127x_preamble_get
 ****************************************************************************/

static int sx127x_preamble_get(FAR struct sx127x_dev_s *dev)
{
  return dev->ops.preamble_get(dev);
}

/****************************************************************************
 * Name: sx127x_version_get
 *
 * Description:
 *   Get chip version
 *
 ****************************************************************************/

static uint8_t sx127x_version_get(FAR struct sx127x_dev_s *dev)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get version */

  regval = sx127x_readregbyte(dev, SX127X_CMN_VERSION);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  return regval;
}

/****************************************************************************
 * Name: sx127x_calibration
 *
 * Description:
 *   Calibrate radio for given frequency
 *
 ****************************************************************************/

static int sx127x_calibration(FAR struct sx127x_dev_s *dev, uint32_t freq)
{
  uint8_t regval = 0;
  int     ret    = OK;

  /* NOTE: The automatic calibration at POR and Reset is only valid at
   * 434 MHz.
   */

  wlinfo("SX127X calibration for %" PRId32 "\n", freq);

  /* Calibration is supported only in FSK/OOK mode */

  ret = sx127x_modulation_set(dev, SX127X_MODULATION_FSK);
  if (ret < 0)
    {
      wlerr("ERROR: can't change modulation to FSK\n");
      return ret;
    }

  /* We need standby mode ? */

  ret = sx127x_opmode_set(dev, SX127X_OPMODE_STANDBY);

  /* Set calibration frequency */

  sx127x_frequency_set(dev, freq);

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Start calibration */

  sx127x_modregbyte(dev, SX127X_FOM_IMAGECAL,
                    SX127X_FOM_IMAGECAL_IMGCALSTART, 0);

  /* Wait for calibration done */

  do
    {
      /* Wait 10ms */

      nxsig_usleep(10000);

      /* Get register */

      regval = sx127x_readregbyte(dev, SX127X_FOM_IMAGECAL);
    }
  while (regval & SX127X_FOM_IMAGECAL_IMGCALRUN);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  wlinfo("Calibration done\n");
  return ret;
}

/****************************************************************************
 * Name: sx127x_init
 *
 * Description:
 *   Initialize SX127X chip
 *
 ****************************************************************************/

static int sx127x_init(FAR struct sx127x_dev_s *dev)
{
  int     ret    = OK;
  uint8_t regval = 0;

  wlinfo("Init sx127x dev\n");

  /* Reset radio */

  sx127x_reset(dev);

  /* Get initial modem state */

  regval = sx127x_readregbyte(dev, SX127X_CMN_OPMODE);

  dev->opmode = (((regval >> SX127X_CMN_OPMODE_MODE_SHIFT) &
                  SX127X_CMN_OPMODE_MODE_MASK) + 1);

  /* After reset - FSK mode */

  dev->modulation = SX127X_MODULATION_FSK;

  /* Initialize modem ops */

  sx127x_ops_set(dev, dev->modulation);

  wlinfo("Init state: modulation=%d, opmode=%d\n",
         dev->modulation, dev->opmode);

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
  /* Initialize FSK/OOK modem only if support is enabled */

  sx127x_modulation_init(dev);
#endif

  /* Get chip version */

  regval = sx127x_version_get(dev);
  if (regval == 0x00)
    {
      /* Probably sth wrong with communication */

      wlerr("ERROR: failed to get chip version!\n");
      return -ENODATA;
    }

  wlinfo("SX127X version = 0x%02x\n", regval);

  /* Calibration */

  ret = sx127x_calibration(dev, SX127X_FREQ_CALIBRATION);
  if (ret < 0)
    {
      wlerr("ERROR: calibration failed\n");
    }

  /* Set default modulation */

  sx127x_modulation_set(dev, CONFIG_LPWAN_SX127X_MODULATION_DEFAULT);

  /* Enter SLEEP mode */

  sx127x_opmode_set(dev, SX127X_OPMODE_SLEEP);

  /* Set default channel frequency - common for FSK/OOK and LORA */

  ret = sx127x_frequency_set(dev, CONFIG_LPWAN_SX127X_RFFREQ_DEFAULT);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure RF output power - common for FSK/OOK and LORA */

  ret = sx127x_power_set(dev, CONFIG_LPWAN_SX127X_TXPOWER_DEFAULT);
  if (ret < 0)
    {
      return ret;
    }

  wlinfo("Init sx127x dev - DONE\n");
  return ret;
}

/****************************************************************************
 * Name: sx127x_deinit
 *
 * Description:
 *   Deinitialize SX127X chip
 *
 ****************************************************************************/

static int sx127x_deinit(FAR struct sx127x_dev_s *dev)
{
  wlinfo("Deinit sx127x dev\n");

  /* Enter SLEEP mode */

  sx127x_opmode_set(dev, SX127X_OPMODE_SLEEP);

  /* Reset radio */

  sx127x_reset(dev);
  return OK;
}

#ifdef CONFIG_DEBUG_WIRELESS_INFO

/****************************************************************************
 * Name: sx127x_lora_dumpregs
 *
 * Description:
 *   Dump registers for LORA modem
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_LORA
static void sx127x_lora_dumpregs(FAR struct sx127x_dev_s *dev)
{
  sx127x_lock(dev->spi);
  wlinfo("LORA dump:\n");
  wlinfo("FIFO:         %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_FIFO));
  wlinfo("OPMODE:       %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_OPMODE));
  wlinfo("FRFMSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_FRFMSB));
  wlinfo("FRFMID:       %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_FRFMID));
  wlinfo("FRFLSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_FRFLSB));
  wlinfo("PACFG:        %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_PACFG));
  wlinfo("PARAMP:       %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_PARAMP));
  wlinfo("OCP:          %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_OCP));
  wlinfo("LNA:          %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_LNA));
  wlinfo("ADDRPTR:      %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_ADDRPTR));
  wlinfo("TXBASE:       %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_TXBASE));
  wlinfo("RXBASE:       %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_RXBASE));
  wlinfo("RXCURR:       %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_RXCURR));
  wlinfo("IRQMASK:      %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_IRQMASK));
  wlinfo("IRQ:          %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_IRQ));
  wlinfo("RXBYTES:      %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_RXBYTES));
  wlinfo("RXHDRMSB:     %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_RXHDRMSB));
  wlinfo("RXHDRLSB:     %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_RXHDRLSB));
  wlinfo("RXPKTMSB:     %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_RXPKTMSB));
  wlinfo("RXPKTLSB:     %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_RXPKTLSB));
  wlinfo("MODSTAT:      %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_MODSTAT));
  wlinfo("PKTSNR:       %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_PKTSNR));
  wlinfo("PKTRSSI:      %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_PKTRSSI));
  wlinfo("RSSI:         %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_RSSIVAL));
  wlinfo("HOPCHAN:      %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_HOPCHAN));
  wlinfo("MDMCFG1:      %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_MDMCFG1));
  wlinfo("MDMCFG2:      %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_MDMCFG2));
  wlinfo("RXTIMEOUTLSB: %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_RXTIMEOUTLSB));
  wlinfo("PREMSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_PREMSB));
  wlinfo("PRELSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_PRELSB));
  wlinfo("PAYLOADLEN:   %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_PAYLOADLEN));
  wlinfo("PAYLOADMAX:   %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_PAYLOADMAX));
  wlinfo("HOPPER:       %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_HOPPER));
  wlinfo("RXFIFOADDR:   %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_RXFIFOADDR));
  wlinfo("MODEMCFG3:    %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_MODEMCFG3));
  wlinfo("FEIMSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_FEIMSB));
  wlinfo("FEIMID:       %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_FEIMID));
  wlinfo("FEILSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_FEILSB));
  wlinfo("RSSIWIDEBAND: %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_RSSIWIDEBAND));
  wlinfo("DETECTOPT:    %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_DETECTOPT));
  wlinfo("INVERTIQ:     %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_INVERTIQ));
  wlinfo("DETECTTHR:    %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_DETECTTHR));
  wlinfo("SYNCWORD:     %02x\n",
         sx127x_readregbyte(dev, SX127X_LRM_SYNCWORD));
  wlinfo("DIOMAP1:      %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_DIOMAP1));
  wlinfo("DIOMAP2:      %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_DIOMAP2));
  wlinfo("VERSION:      %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_VERSION));
  wlinfo("TCXO:         %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_TCXO));
  wlinfo("PADAC:        %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_PADAC));
  wlinfo("FTEMP:        %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_FTEMP));
  wlinfo("AGCREF:       %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_AGCREF));
  wlinfo("AGCTHR1:      %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_AGCTHR1));
  wlinfo("AGCTHR2:      %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_AGCTHR2));
  wlinfo("AGCTHR3:      %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_AGCTHR3));
  wlinfo("PLL:          %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_PLL));
  sx127x_unlock(dev->spi);
}
#endif /* CONFIG_LPWAN_SX127X_LORA */

/****************************************************************************
 * Name: sx127x_fskook_dumpregs
 *
 * Description:
 *   Dump registers for FSK/OOK modem
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
static void sx127x_fskook_dumpregs(FAR struct sx127x_dev_s *dev)
{
  sx127x_lock(dev->spi);
  wlinfo("FSK/OOK dump:\n");
  wlinfo("FIFO:         %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_FIFO));
  wlinfo("OPMODE:       %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_OPMODE));
  wlinfo("FRFMSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_FRFMSB));
  wlinfo("FRFMID:       %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_FRFMID));
  wlinfo("FRFLSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_FRFLSB));
  wlinfo("PACFG:        %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_PACFG));
  wlinfo("PARAMP:       %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_PARAMP));
  wlinfo("OCP:          %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_OCP));
  wlinfo("LNA:          %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_LNA));
  wlinfo("BITRATEMSB:   %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_BITRATEMSB));
  wlinfo("BITRATELSM:   %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_BITRATELSB));
  wlinfo("FDEVMSB:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_FDEVMSB));
  wlinfo("FDEVLSB:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_FDEVLSB));
  wlinfo("RXCFG:        %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_RXCFG));
  wlinfo("RSSICFG:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_RSSICFG));
  wlinfo("RSSICOLL:     %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_RSSICOLL));
  wlinfo("RSSITHR:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_RSSITHR));
  wlinfo("RSSIVAL:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_RSSIVAL));
  wlinfo("RXBW:         %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_RXBW));
  wlinfo("AFCBW:        %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_AFCBW));
  wlinfo("OOKPEAK:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_OOKPEAK));
  wlinfo("OOKFIX:       %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_OOKFIX));
  wlinfo("AFCFEI:       %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_AFCFEI));
  wlinfo("AFCMSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_AFCMSB));
  wlinfo("AFCLSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_AFCLSB));
  wlinfo("FEIMSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_FEIMSB));
  wlinfo("FEILSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_FEILSB));
  wlinfo("PREDET:       %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_PREDET));
  wlinfo("RXTIMEOUT1:   %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_RXTIMEOUT1));
  wlinfo("RXTIMEOUT2:   %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_RXTIMEOUT1));
  wlinfo("RXTIMEOUT3:   %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_RXTIMEOUT1));
  wlinfo("RXDELAY:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_RXDELAY));
  wlinfo("OSC:          %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_OSC));
  wlinfo("PREMSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_PREMSB));
  wlinfo("PRELSB:       %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_PRELSB));
  wlinfo("SYNCCFG:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_SYNCCFG));
  wlinfo("SYNCVAL1:     %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_SYNCVAL1));
  wlinfo("SYNCVAL2:     %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_SYNCVAL2));
  wlinfo("SYNCVAL3:     %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_SYNCVAL3));
  wlinfo("SYNCVAL4:     %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_SYNCVAL4));
  wlinfo("SYNCVAL5:     %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_SYNCVAL5));
  wlinfo("PKTCFG1:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_PKTCFG1));
  wlinfo("PKTCFG2:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_PKTCFG2));
  wlinfo("PAYLOADLEN:   %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_PAYLOADLEN));
  wlinfo("NODEADDR:     %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_NODEADDR));
  wlinfo("BROADCAST:    %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_BROADCAST));
  wlinfo("FIFOTHR:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_FIFOTHR));
  wlinfo("SEQCFG1:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_SEQCFG1));
  wlinfo("SEQCFG2:      %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_SEQCFG2));
  wlinfo("TIMRES:       %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_TIMRES));
  wlinfo("TIMER1COEF:   %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_TIMER1COEF));
  wlinfo("TIMER2COEF:   %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_TIMER2COEF));
  wlinfo("IMAGECAL:     %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_IMAGECAL));
  wlinfo("TEMP:         %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_TEMP));
  wlinfo("LOWBAT:       %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_LOWBAT));
  wlinfo("IRQ1:         %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_IRQ1));
  wlinfo("IRQ2:         %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_IRQ2));
  wlinfo("PLLHOP:       %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_PLLHOP));
  wlinfo("BITRATEFRAC:  %02x\n",
         sx127x_readregbyte(dev, SX127X_FOM_BITRATEFRAC));
  wlinfo("DIOMAP1:      %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_DIOMAP1));
  wlinfo("DIOMAP2:      %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_DIOMAP2));
  wlinfo("VERSION:      %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_VERSION));
  wlinfo("TCXO:         %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_TCXO));
  wlinfo("PADAC:        %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_PADAC));
  wlinfo("FTEMP:        %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_FTEMP));
  wlinfo("AGCREF:       %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_AGCREF));
  wlinfo("AGCTHR1:      %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_AGCTHR1));
  wlinfo("AGCTHR2:      %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_AGCTHR2));
  wlinfo("AGCTHR3:      %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_AGCTHR3));
  wlinfo("PLL:          %02x\n",
         sx127x_readregbyte(dev, SX127X_CMN_PLL));
  sx127x_unlock(dev->spi);
}
#endif

/****************************************************************************
 * Name: sx127x_dumpregs
 *
 * Description:
 *   Dump registers according to current modulation
 *
 ****************************************************************************/

static void sx127x_dumpregs(FAR struct sx127x_dev_s *dev)
{
  switch (dev->modulation)
    {
#ifdef CONFIG_LPWAN_SX127X_LORA
      case SX127X_MODULATION_LORA:
        {
          sx127x_lora_dumpregs(dev);
          break;
        }
#endif

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
      case SX127X_MODULATION_FSK:
      case SX127X_MODULATION_OOK:
        {
          sx127x_fskook_dumpregs(dev);
          break;
        }
#endif

      default:
        {
          wlinfo("Unknown SX127X modulation\n");
          break;
        }
    }
}
#endif /* CONFIG_DEBUG_WIRELESS_INFO */

/****************************************************************************
 * Name: sx127x_unregister
 *
 * Description:
 *   Unregister SX127X device
 *
 ****************************************************************************/

static int sx127x_unregister(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  /* Release IRQ */

  sx127x_attachirq0(dev, NULL, NULL);

  /* Destroy mutex & semaphores */

  nxmutex_destroy(&dev->dev_lock);
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
  nxsem_destroy(&dev->tx_sem);
#endif
#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  nxsem_destroy(&dev->rx_sem);
  nxmutex_destroy(&dev->rx_buffer_lock);
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx127x_register
 *
 * Description:
 *   Register sx127x driver
 *
 ****************************************************************************/

int sx127x_register(FAR struct spi_dev_s *spi,
                    FAR const struct sx127x_lower_s *lower)
{
  FAR struct sx127x_dev_s *dev = NULL;
  int ret = OK;

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(lower->reset != NULL);
  DEBUGASSERT(lower->irq0attach != NULL);
  DEBUGASSERT(lower->opmode_change != NULL);
  DEBUGASSERT(lower->freq_select != NULL);
  DEBUGASSERT(lower->pa_select != NULL);

  /* Only one sx127x device supported for now */

  dev = &g_sx127x_devices[0];

  /* Reset data */

  memset(dev, 0, sizeof(struct sx127x_dev_s));

  /* Attach the interface, lower driver */

  dev->spi   = spi;
  dev->lower = lower;

  /* Initlaize configuration */

  dev->idle           = SX127X_IDLE_OPMODE;
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
  dev->pa_force       = lower->pa_force;
#endif
  dev->crcon          = CONFIG_LPWAN_SX127X_CRCON;
#ifdef CONFIG_LPWAN_SX127X_FSKOOK
  dev->fskook.fixlen  = false;
#endif
#ifdef CONFIG_LPWAN_SX127X_LORA
  dev->lora.invert_iq = false;
#endif

  /* Polled file decr */

  dev->pfd = NULL;

  /* Initialize mutex & sem */

  nxmutex_init(&dev->dev_lock);
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
  nxsem_init(&dev->tx_sem, 0, 0);
#endif
#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  nxsem_init(&dev->rx_sem, 0, 0);
  nxmutex_init(&dev->rx_buffer_lock);
#endif

  /* Attach irq0 - TXDONE/RXDONE/CADDONE */

  sx127x_attachirq0(dev, sx127x_irq0handler, dev);

  /* TODO: support for irq1-5 */

  wlinfo("Registering " SX127X_DEV_NAME "\n");

  ret = register_driver(SX127X_DEV_NAME, &g_sx127x_fops, 0666, dev);
  if (ret < 0)
    {
      wlerr("ERROR: register_driver() failed: %d\n", ret);
      sx127x_unregister(dev);
    }

  return ret;
}

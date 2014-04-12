/****************************************************************************
 * drivers/wireless/nrf24l01/nrf24l01.c
 *
 *   Copyright (C) 2013 Laurent Latil. All rights reserved.
 *   Authors: Laurent Latil <laurent@latil.nom.fr>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

/* Features:
 *   - Fixed length and dynamically sized payloads  (1 - 32 bytes)
 *   - Management of the 6 receiver pipes
 *   - Configuration of each pipe: address, packet length, auto-acknowledge, etc.
 *   - Use a FIFO buffer to store the received packets
 *
 * Todo:
 *   - Add support for payloads in ACK packets  (?)
 *   - Add compatibility with nRF24L01 (not +)  hardware  (?)
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#ifdef CONFIG_WL_NRF24L01_RXSUPPORT
#  include <nuttx/wqueue.h>
#endif

#include <nuttx/wireless/nrf24l01.h>
#include "nrf24l01.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_WL_NRF24L01_DFLT_ADDR_WIDTH
#  define CONFIG_WL_NRF24L01_DFLT_ADDR_WIDTH 5
#endif

#ifndef CONFIG_WL_NRF24L01_RXFIFO_LEN
#  define CONFIG_WL_NRF24L01_RXFIFO_LEN 128
#endif

#ifdef CONFIG_WL_NRF24L01_CHECK_PARAMS
#  define CHECK_ARGS(cond) do { if (!(cond)) return -EINVAL; } while (0)
#else
#  define CHECK_ARGS(cond)
#endif

/* Default SPI bus frequency (in Hz) */
#define NRF24L01_SPIFREQ   9000000 /* Can go up to 10 Mbs according to datasheet */

/* power-down -> standby transition timing (in us).  Note: this value is probably larger than required. */
#define NRF24L01_TPD2STBY_DELAY  4500

#define FIFO_PKTLEN_MASK  0x1F   /* 5 ls bits used to store packet length */
#define FIFO_PKTLEN_SHIFT 0
#define FIFO_PIPENO_MASK  0xE0   /* 3 ms bits used to store pipe # */
#define FIFO_PIPENO_SHIFT 4

#define FIFO_PKTLEN(dev) (((dev->rx_fifo[dev->nxt_read] & FIFO_PKTLEN_MASK) >> FIFO_PKTLEN_SHIFT) + 1)
#define FIFO_PIPENO(dev) (((dev->rx_fifo[dev->nxt_read] & FIFO_PIPENO_MASK) >> FIFO_PIPENO_SHIFT))
#define FIFO_HEADER(pktlen,pipeno) ((pktlen - 1) | (pipeno << FIFO_PIPENO_SHIFT))

#define DEV_NAME   "/dev/nrf24l01"

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

typedef enum
{
  MODE_READ,
  MODE_WRITE
} nrf24l01_access_mode_t;

#define FL_AA_ENABLED  (1 << 0)

struct nrf24l01_dev_s
{
  FAR struct spi_dev_s *spi;            /* Reference to SPI bus device */
  FAR struct nrf24l01_config_s *config; /* Board specific GPIO functions */

  nrf24l01_state_t state;       /* Current state of the nRF24L01 */

  uint8_t en_aa;            /* Cache EN_AA register value */
  uint8_t en_pipes;         /* Cache EN_RXADDR register value */
  bool ce_enabled;          /* Cache the value of CE pin */
  uint8_t lastxmitcount;    /* Retransmit count of the last succeeded AA transmission */
  uint8_t addrlen;          /* Address width (3-5) */
  uint8_t pipedatalen[NRF24L01_PIPE_COUNT];

  uint8_t pipe0addr[NRF24L01_MAX_ADDR_LEN];  /* Configured address on pipe 0 */

  uint8_t last_recvpipeno;
  sem_t sem_tx;

#ifdef CONFIG_WL_NRF24L01_RXSUPPORT
  uint8_t *rx_fifo;         /* Circular RX buffer.  [pipe# / pkt_len] [packet data...] */
  uint16_t fifo_len;        /* Number of bytes stored in fifo */
  uint16_t nxt_read;        /* Next read index */
  uint16_t nxt_write;       /* Next write index */
  sem_t sem_fifo;           /* Protect access to rx fifo */
  sem_t sem_rx;             /* Wait for availability of received data */

  struct work_s irq_work;   /* Interrupt handling "bottom half" */
#endif

  uint8_t nopens;           /* Number of times the device has been opened */
  sem_t devsem;             /* Ensures exclusive access to this structure */
#ifndef CONFIG_DISABLE_POLL
  FAR struct pollfd *pfd;   /* Polled file descr  (or NULL if any) */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Low-level SPI helpers */

#ifdef CONFIG_SPI_OWNBUS
static inline void nrf24l01_configspi(FAR struct spi_dev_s *spi);
#  define nrf24l01_lock(spi)
#  define nrf24l01_unlock(spi)
#else
#  define nrf24l01_configspi(spi);
static void nrf24l01_lock(FAR struct spi_dev_s *spi);
static void nrf24l01_unlock(FAR struct spi_dev_s *spi);
#endif

static uint8_t nrf24l01_access(FAR struct nrf24l01_dev_s *dev,
    nrf24l01_access_mode_t mode, uint8_t cmd, uint8_t *buf, int length);

static uint8_t nrf24l01_flush_rx(FAR struct nrf24l01_dev_s *dev);

static uint8_t nrf24l01_flush_tx(FAR struct nrf24l01_dev_s *dev);

/* Read register from nrf24 */

static uint8_t nrf24l01_readreg(FAR struct nrf24l01_dev_s *dev, uint8_t reg,
    uint8_t *value, int len);

/* Read single byte value from a register of nrf24 */

static uint8_t nrf24l01_readregbyte(FAR struct nrf24l01_dev_s *dev,
    uint8_t reg);

static void nrf24l01_writeregbyte(FAR struct nrf24l01_dev_s *dev, uint8_t reg,
    uint8_t value);

static uint8_t nrf24l01_setregbit(FAR struct nrf24l01_dev_s *dev, uint8_t reg,
    uint8_t value, bool set);

static void nrf24l01_tostate(FAR struct nrf24l01_dev_s *dev, nrf24l01_state_t state);

static int nrf24l01_irqhandler(FAR int irq, FAR void *context);

static inline int nrf24l01_attachirq(FAR struct nrf24l01_dev_s *dev, xcpt_t isr);

static int dosend(FAR struct nrf24l01_dev_s *dev, FAR const uint8_t *data, size_t datalen);

static int nrf24l01_unregister(FAR struct nrf24l01_dev_s *dev);

#ifdef CONFIG_WL_NRF24L01_RXSUPPORT

void fifoput(struct nrf24l01_dev_s *dev, uint8_t pipeno, uint8_t *buffer, uint8_t buflen);

uint8_t fifoget(struct nrf24l01_dev_s *dev, uint8_t *buffer, uint8_t buflen, uint8_t *pipeno);

static void nrf24l01_worker(FAR void *arg);

#endif

/* POSIX API */

static int nrf24l01_open(FAR struct file *filep);

static int nrf24l01_close(FAR struct file *filep);

static ssize_t nrf24l01_read(FAR struct file *filep, FAR char *buffer, size_t buflen);

static ssize_t nrf24l01_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);

static int nrf24l01_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int nrf24l01_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct nrf24l01_dev_s *g_nrf24l01dev;

static const struct file_operations nrf24l01_fops =
{
  .open = nrf24l01_open,    /* open */
  .close = nrf24l01_close,  /* close */
  .read = nrf24l01_read,    /* read */
  .write = nrf24l01_write,  /* write */
  .seek = NULL,             /* seek */
  .ioctl = nrf24l01_ioctl,  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  .poll = nrf24l01_poll     /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static void nrf24l01_lock(FAR struct spi_dev_s *spi)
{
  /* Lock the SPI bus because there are multiple devices competing for the
   * SPI bus
   */

  (void)SPI_LOCK(spi, true);

  /* We have the lock.  Now make sure that the SPI bus is configured for the
   * NRF24L01 (it might have gotten configured for a different device while
   * unlocked)
   */

  SPI_SELECT(spi, SPIDEV_WIRELESS, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, NRF24L01_SPIFREQ);
  SPI_SELECT(spi, SPIDEV_WIRELESS, false);
}
#endif

/****************************************************************************
 * Function: nrf24l01_unlock
 *
 * Description:
 *   If we are sharing the SPI bus with other devices (CONFIG_SPI_OWNBUS
 *   undefined) then we need to un-lock the SPI bus for each transfer,
 *   possibly losing the current configuration.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static void nrf24l01_unlock(FAR struct spi_dev_s *spi)
{
  /* Relinquish the SPI bus. */

  (void)SPI_LOCK(spi, false);
}
#endif

/****************************************************************************
 * Function: nrf24l01_configspi
 *
 * Description:
 *   Configure the SPI for use with the NRF24L01.  This function should be
 *   called once during touchscreen initialization to configure the SPI
 *   bus.  Note that if CONFIG_SPI_OWNBUS is not defined, then this function
 *   does nothing.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void nrf24l01_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the NRF24L01 module.
   * As we own the SPI bus this method is called just once.
   */

  SPI_SELECT(spi, SPIDEV_WIRELESS, true);  // Useful ?
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, NRF24L01_SPIFREQ);
  SPI_SELECT(spi, SPIDEV_WIRELESS, false);
}
#endif

static inline void nrf24l01_select(struct nrf24l01_dev_s * dev)
{
  SPI_SELECT(dev->spi, SPIDEV_WIRELESS, true);
}

static inline void nrf24l01_deselect(struct nrf24l01_dev_s * dev)
{
  SPI_SELECT(dev->spi, SPIDEV_WIRELESS, false);
}

static uint8_t nrf24l01_access(FAR struct nrf24l01_dev_s *dev,
    nrf24l01_access_mode_t mode, uint8_t cmd, FAR uint8_t *buf, int length)
{
  uint8_t status;

  /* Prepare SPI */

  nrf24l01_select(dev);

  /* Transfer */

  status = SPI_SEND(dev->spi, cmd);

  switch (mode)
    {
    case MODE_WRITE:
      if (length > 0)
        {
          SPI_SNDBLOCK(dev->spi, buf, length);
        }
      break;

    case MODE_READ:
      SPI_RECVBLOCK(dev->spi, buf, length);
      break;
    }

  nrf24l01_deselect(dev);
  return status;
}

static inline uint8_t nrf24l01_flush_rx(struct nrf24l01_dev_s *dev)
{
  return nrf24l01_access(dev, MODE_WRITE, NRF24L01_FLUSH_RX, NULL, 0);
}

static inline uint8_t nrf24l01_flush_tx(struct nrf24l01_dev_s *dev)
{
  return nrf24l01_access(dev, MODE_WRITE, NRF24L01_FLUSH_TX, NULL, 0);
}

/* Read register from nrf24l01 */

static inline uint8_t nrf24l01_readreg(struct nrf24l01_dev_s *dev, uint8_t reg,
    uint8_t *value, int len)
{
  return nrf24l01_access(dev, MODE_READ, reg | NRF24L01_R_REGISTER, value, len);
}

/* Read single byte value from a register of nrf24l01 */

static inline uint8_t nrf24l01_readregbyte(struct nrf24l01_dev_s *dev,
    uint8_t reg)
{
  uint8_t val;
  nrf24l01_readreg(dev, reg, &val, 1);
  return val;
}

/* Write value to a register of nrf24l01 */

static inline int nrf24l01_writereg(FAR struct nrf24l01_dev_s *dev, uint8_t reg,
    FAR const uint8_t *value, int len)
{
  return nrf24l01_access(dev, MODE_WRITE, reg | NRF24L01_W_REGISTER, (FAR uint8_t *)value, len);
}

/* Write single byte value to a register of nrf24l01 */

static inline void nrf24l01_writeregbyte(struct nrf24l01_dev_s *dev, uint8_t reg,
    uint8_t value)
{
  nrf24l01_writereg(dev, reg, &value, 1);
}

static uint8_t nrf24l01_setregbit(struct nrf24l01_dev_s *dev, uint8_t reg,
    uint8_t value, bool set)
{
  uint8_t val;

  nrf24l01_readreg(dev, reg, &val, 1);
  if (set)
    {
      val |= value;
    }
  else
    {
      val &= ~value;
    }

  nrf24l01_writereg(dev, reg, &val, 1);
  return val;
}

#ifdef CONFIG_WL_NRF24L01_RXSUPPORT

/* RX fifo mgt */

void fifoput(struct nrf24l01_dev_s *dev, uint8_t pipeno, uint8_t *buffer, uint8_t buflen)
{
  sem_wait(&dev->sem_fifo);
  while (dev->fifo_len + buflen + 1 > CONFIG_WL_NRF24L01_RXFIFO_LEN)
    {
      /* TODO: Set fifo overrun flag ! */

      int skiplen = FIFO_PKTLEN(dev) + 1;

      dev->nxt_read = (dev->nxt_read + skiplen) % CONFIG_WL_NRF24L01_RXFIFO_LEN;
      dev->fifo_len -= skiplen;
    }

  dev->rx_fifo[dev->nxt_write] = FIFO_HEADER(buflen, pipeno);
  dev->nxt_write = (dev->nxt_write + 1) % CONFIG_WL_NRF24L01_RXFIFO_LEN;

  /* Adjust fifo bytes count */

  dev->fifo_len += (buflen + 1);
  while (buflen--)
    {
      dev->rx_fifo[dev->nxt_write] = *(buffer++);
      dev->nxt_write = (dev->nxt_write + 1) % CONFIG_WL_NRF24L01_RXFIFO_LEN;
    }

  sem_post(&dev->sem_fifo);
}

uint8_t fifoget(struct nrf24l01_dev_s *dev, uint8_t *buffer, uint8_t buflen, uint8_t *pipeno)
{
  uint8_t pktlen;
  uint8_t i;

  sem_wait(&dev->sem_fifo);

  ASSERT(dev->fifo_len > 0);

  pktlen = FIFO_PKTLEN(dev);
  if (NULL != pipeno)
    {
      *pipeno = FIFO_PIPENO(dev);
    }

  dev->nxt_read = (dev->nxt_read + 1) % CONFIG_WL_NRF24L01_RXFIFO_LEN;

  for (i = 0; i < pktlen && i < buflen; i++)
    {
      *(buffer++) = dev->rx_fifo[dev->nxt_read];
      dev->nxt_read = (dev->nxt_read + 1) % CONFIG_WL_NRF24L01_RXFIFO_LEN;
    }

  if (i < pktlen)
    {
      dev->nxt_read = (dev->nxt_read + pktlen - i) % CONFIG_WL_NRF24L01_RXFIFO_LEN;
    }

  /* Adjust fifo bytes count */

  dev->fifo_len -= (pktlen + 1);

  sem_post(&dev->sem_fifo);
  return pktlen;
}

#endif

static int nrf24l01_irqhandler(int irq, FAR void *context)
{
  FAR struct nrf24l01_dev_s *dev = g_nrf24l01dev;

  wllvdbg("*IRQ*");

#ifdef CONFIG_WL_NRF24L01_RXSUPPORT

  /* If RX is enabled we delegate the actual work to bottom-half handler */

  work_queue(HPWORK, &g_nrf24l01dev->irq_work, nrf24l01_worker, dev, 0);
#else

  /* Otherwise we simply wake up the send function */

  sem_post(&dev->sem_tx);  /* Wake up the send function */
#endif

  return OK;
}

/* Configure IRQ pin  (falling edge) */

static inline int nrf24l01_attachirq(FAR struct nrf24l01_dev_s *dev, xcpt_t isr)
{
  return dev->config->irqattach(isr);
}

static inline bool nrf24l01_chipenable(FAR struct nrf24l01_dev_s *dev, bool enable)
{
  if (dev->ce_enabled != enable)
    {
      dev->config->chipenable(enable);
      dev->ce_enabled = enable;
      return !enable;
    }
  else
    {
      return enable;
    }
}

#ifdef CONFIG_WL_NRF24L01_RXSUPPORT

static void nrf24l01_worker(FAR void *arg)
{
  FAR struct nrf24l01_dev_s *dev = (FAR struct nrf24l01_dev_s *) arg;
  uint8_t status;
  uint8_t fifo_status;

  nrf24l01_lock(dev->spi);

  status = nrf24l01_readregbyte(dev, NRF24L01_STATUS);

  if (status & NRF24L01_RX_DR)
    {
      /* put CE low */

      bool ce = nrf24l01_chipenable(dev, false);

      wdbg("RX_DR is set!\n");

      /* Read and store all received payloads */

      do
        {
          uint8_t pipeno;
          uint8_t pktlen;
          uint8_t buf[NRF24L01_MAX_PAYLOAD_LEN];

          /* For each packet:
           *   - Get pipe #
           *   - Get payload length  (either static or dynamic)
           *   - Read payload content
           */

          pipeno = (status & NRF24L01_RX_P_NO_MASK) >> NRF24L01_RX_P_NO_SHIFT;

          pktlen = dev->pipedatalen[pipeno];
          if (NRF24L01_DYN_LENGTH == pktlen)
            {
              /* If dynamic length payload need to use R_RX_PL_WID command to get actual length */

              nrf24l01_access(dev, MODE_READ, NRF24L01_R_RX_PL_WID, &pktlen, 1);
            }

          /* Get payload content */

          nrf24l01_access(dev, MODE_READ, NRF24L01_R_RX_PAYLOAD, buf, pktlen);

          fifoput(dev, pipeno, buf, pktlen);
          sem_post(&dev->sem_rx);  /* Wake-up any thread waiting in recv */

          status = nrf24l01_readreg(dev, NRF24L01_FIFO_STATUS, &fifo_status, 1);

          wdbg("FIFO_STATUS=%02x\n", fifo_status);
          wdbg("STATUS=%02x\n", status);
        }
      while (!(fifo_status | NRF24L01_RX_EMPTY));

      /* Clear interrupt sources */

      nrf24l01_writeregbyte(dev, NRF24L01_STATUS, NRF24L01_RX_DR);

      /* Restore CE */

      nrf24l01_chipenable(dev, ce);

#ifndef CONFIG_DISABLE_POLL
      if (dev->pfd)
        {
          dev->pfd->revents |= POLLIN;  /* Data available for input */

          wvdbg("Wake up polled fd");
          sem_post(dev->pfd->sem);
        }
#endif
    }

  if (status & (NRF24L01_TX_DS | NRF24L01_MAX_RT))
    {
      /* The actual work is done in the send function */

      sem_post(&dev->sem_tx);
    }

  if (dev->state == ST_RX)
    {
      /* re-enable CE   (to go back to RX mode state) */

      nrf24l01_chipenable(dev, true);
    }
  nrf24l01_unlock(dev->spi);
}

#endif

static void nrf24l01_tostate(struct nrf24l01_dev_s *dev, nrf24l01_state_t state)
{
  nrf24l01_state_t oldstate = dev->state;

  if (oldstate == state)
    {
      return;
    }

  if (oldstate == ST_POWER_DOWN)
    {
      /* Leaving power down   (note: new state cannot be power down here) */

      nrf24l01_setregbit(dev, NRF24L01_CONFIG, NRF24L01_PWR_UP, true);
      usleep(NRF24L01_TPD2STBY_DELAY);
    }

  /* Entering new state */

  switch(state)
    {
    case ST_UNKNOWN:
      /* Power down the module here... */
    case ST_POWER_DOWN:
      nrf24l01_chipenable(dev, false);
      nrf24l01_setregbit(dev, NRF24L01_CONFIG, NRF24L01_PWR_UP, false);
      break;

    case ST_STANDBY:
      nrf24l01_chipenable(dev, false);
      nrf24l01_setregbit(dev, NRF24L01_CONFIG, NRF24L01_PRIM_RX, false);
      break;

#ifdef CONFIG_WL_NRF24L01_RXSUPPORT
    case ST_RX:
      nrf24l01_setregbit(dev, NRF24L01_CONFIG, NRF24L01_PRIM_RX, true);
      nrf24l01_chipenable(dev, true);
      break;
#endif
    }

  dev->state = state;
}

static int dosend(FAR struct nrf24l01_dev_s *dev, FAR const uint8_t *data, size_t datalen)
{
  uint8_t status;
  uint8_t obsvalue;
  int result;

  /* Store the current lifecycle state in order to restore it after transmit done */

  nrf24l01_state_t prevstate = dev->state;

  nrf24l01_tostate(dev, ST_STANDBY);

  /* Write payload */

  nrf24l01_access(dev, MODE_WRITE, NRF24L01_W_TX_PAYLOAD, (FAR uint8_t *)data, datalen);

  /* Enable CE to start transmission */

  nrf24l01_chipenable(dev, true);

  /* Free the SPI bus during the IRQ wait */

  nrf24l01_unlock(dev->spi);

  /* Wait for IRQ  (TX_DS or MAX_RT) */

  while (sem_wait(&dev->sem_tx) != 0)
    {
      /* Note that we really need to wait here, as the interrupt source
       * (either TX_DS in case of success, or MAX_RT for failure) needs to be cleared.
       */

      DEBUGASSERT(errno == EINTR);
    }

  /* Re-acquire the SPI bus */

  nrf24l01_lock(dev->spi);

  status = nrf24l01_readreg(dev, NRF24L01_OBSERVE_TX, &obsvalue, 1);
  if (status & NRF24L01_TX_DS)
    {
      /* transmit OK */

      result = OK;
      dev->lastxmitcount = (obsvalue & NRF24L01_ARC_CNT_MASK)
          >> NRF24L01_ARC_CNT_SHIFT;

      wvdbg("Transmission OK (lastxmitcount=%d)\n", dev->lastxmitcount);
    }
  else if (status & NRF24L01_MAX_RT)
    {
      wvdbg("MAX_RT!\n", dev->lastxmitcount);
      result = -ECOMM;
      dev->lastxmitcount = NRF24L01_XMIT_MAXRT;

      /* If no ACK packet is received the payload remains in TX fifo.  We need to flush it. */

      nrf24l01_flush_tx(dev);
    }
  else
    {
      /* Unexpected... */

      wdbg("No TX_DS nor MAX_RT bit set in STATUS reg!\n");
      result = -EIO;
    }

  /* Clear interrupt sources */

  nrf24l01_writeregbyte(dev, NRF24L01_STATUS, NRF24L01_TX_DS | NRF24L01_MAX_RT);

  /* Restore state */

  nrf24l01_tostate(dev, prevstate);
  return result;
}

/* POSIX API */

static int nrf24l01_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct nrf24l01_dev_s *dev;
  int result;

  wvdbg("Opening nRF24L01 dev\n");

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct nrf24l01_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  if (sem_wait(&dev->devsem) < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Check if device is not already used */

  if (dev->nopens > 0)
    {
      result = -EBUSY;
      goto errout;
    }

  result = nrf24l01_init(dev);
  if (!result)
    {
      dev->nopens++;
    }

errout:
  sem_post(&dev->devsem);
  return result;
}

static int nrf24l01_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct nrf24l01_dev_s *dev;

  wvdbg("Closing nRF24L01 dev\n");
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev  = (FAR struct nrf24l01_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  if (sem_wait(&dev->devsem) < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  nrf24l01_changestate(dev, ST_POWER_DOWN);
  dev->nopens--;

  sem_post(&dev->devsem);
  return OK;
}

static ssize_t nrf24l01_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
#ifndef CONFIG_WL_NRF24L01_RXSUPPORT
  return -ENOSYS;
#else
  FAR struct nrf24l01_dev_s *dev;
  FAR struct inode *inode;
  int result;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct nrf24l01_dev_s *)inode->i_private;

  if (sem_wait(&dev->devsem) < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }
  result = nrf24l01_recv(dev, (uint8_t *)buffer, buflen, &dev->last_recvpipeno);

  sem_post(&dev->devsem);
  return result;
#endif
}

static ssize_t nrf24l01_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  FAR struct nrf24l01_dev_s *dev;
  FAR struct inode *inode;
  int result;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct nrf24l01_dev_s *)inode->i_private;

  if (sem_wait(&dev->devsem) < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  result = nrf24l01_send(dev, (const uint8_t *)buffer, buflen);

  sem_post(&dev->devsem);
  return result;
}

static int nrf24l01_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct nrf24l01_dev_s *dev;
  int result = OK;

  wvdbg("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev  = (FAR struct nrf24l01_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  if (sem_wait(&dev->devsem) < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      case WLIOC_SETRADIOFREQ:  /* Set radio frequency. Arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          nrf24l01_setradiofreq(dev, *ptr);
        }
        break;

      case WLIOC_GETRADIOFREQ:  /* Get current radio frequency. arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          *ptr = nrf24l01_getradiofreq(dev);
        }
        break;

      case NRF24L01IOC_SETTXADDR:  /* Set current TX addr. arg: Pointer to uint8_t array defining the address */
        {
          FAR const uint8_t *addr = (FAR const uint8_t *)(arg);
          DEBUGASSERT(addr != NULL);
          nrf24l01_settxaddr(dev, addr);
        }
        break;

      case NRF24L01IOC_GETTXADDR:  /* Get current TX addr. arg: Pointer to uint8_t array defining the address */
        {
          FAR uint8_t *addr = (FAR uint8_t *)(arg);
          DEBUGASSERT(addr != NULL);
          nrf24l01_gettxaddr(dev, addr);
        }
        break;

      case WLIOC_SETTXPOWER:  /* Set current radio frequency. arg: Pointer to int32_t, output power */
        {
          FAR int32_t *ptr = (FAR int32_t *)(arg);
          DEBUGASSERT(ptr != NULL);
          nrf24l01_settxpower(dev, *ptr);
        }
        break;

      case WLIOC_GETTXPOWER:  /* Get current radio frequency. arg: Pointer to int32_t, output power */
        {
          FAR int32_t *ptr = (FAR int32_t *)(arg);
          DEBUGASSERT(ptr != NULL);
          *ptr = nrf24l01_gettxpower(dev);
        }
        break;

      case NRF24L01IOC_SETRETRCFG:  /* Set retransmit params. arg: Pointer to nrf24l01_retrcfg_t */
        {
          FAR nrf24l01_retrcfg_t *ptr = (FAR nrf24l01_retrcfg_t *)(arg);
          DEBUGASSERT(ptr != NULL);
          nrf24l01_setretransmit(dev, ptr->delay, ptr->count);
        }
        break;

      case NRF24L01IOC_GETRETRCFG:  /* Get retransmit params. arg: Pointer to nrf24l01_retrcfg_t */
        result = -ENOSYS;  /* TODO !*/
        break;

      case NRF24L01IOC_SETPIPESCFG:
        {
          int i;
          FAR nrf24l01_pipecfg_t **cfg_array = (FAR nrf24l01_pipecfg_t **)(arg);

          DEBUGASSERT(cfg_array != NULL);
          for (i = 0; i < NRF24L01_PIPE_COUNT; i++)
            {
              if (cfg_array[i])
                {
                  nrf24l01_setpipeconfig(dev, i, cfg_array[i]);
                }
            }
        }
        break;

      case NRF24L01IOC_GETPIPESCFG:
        {
          int i;
          FAR nrf24l01_pipecfg_t **cfg_array = (FAR nrf24l01_pipecfg_t **)(arg);

          DEBUGASSERT(cfg_array != NULL);
          for (i = 0; i < NRF24L01_PIPE_COUNT; i++)
            {
              if (cfg_array[i])
                {
                  nrf24l01_getpipeconfig(dev, i, cfg_array[i]);
                }
            }
        }
        break;

      case NRF24L01IOC_SETPIPESENABLED:
        {
          int i;
          uint8_t en_pipes;

          FAR uint8_t *en_pipesp = (FAR uint8_t *)(arg);

          DEBUGASSERT(en_pipesp != NULL);
          en_pipes = *en_pipesp;
          for (i = 0; i < NRF24L01_PIPE_COUNT; i++)
            {
              if ((dev->en_pipes & (1 << i)) != (en_pipes & (1 << i)))
                {
                  nrf24l01_enablepipe(dev, i, en_pipes & (1 << i));
                }
            }
        }
        break;

      case NRF24L01IOC_GETPIPESENABLED:
        {
           FAR uint8_t *en_pipesp = (FAR uint8_t *)(arg);

           DEBUGASSERT(en_pipesp != NULL);
           *en_pipesp = dev->en_pipes;
           break;
        }

      case NRF24L01IOC_SETDATARATE:
        {
           FAR nrf24l01_datarate_t *drp = (FAR nrf24l01_datarate_t *)(arg);
           DEBUGASSERT(drp != NULL);

           nrf24l01_setdatarate(dev, *drp);
           break;
        }

      case NRF24L01IOC_GETDATARATE:
        result = -ENOSYS;  /* TODO !*/
        break;

      case NRF24L01IOC_SETADDRWIDTH:
        {
           FAR uint32_t *widthp = (FAR uint32_t *)(arg);
           DEBUGASSERT(widthp != NULL);

           nrf24l01_setaddrwidth(dev, *widthp);
           break;
        }

      case NRF24L01IOC_GETADDRWIDTH:
        {
           FAR int *widthp = (FAR int *)(arg);
           DEBUGASSERT(widthp != NULL);

           *widthp = (int)dev->addrlen;
           break;
        }

      case NRF24L01IOC_SETSTATE:
        {
           FAR nrf24l01_state_t *statep = (FAR nrf24l01_state_t *)(arg);
           DEBUGASSERT(statep != NULL);

           nrf24l01_changestate(dev, *statep);
           break;
        }

      case NRF24L01IOC_GETSTATE:
        {
           FAR nrf24l01_state_t *statep = (FAR nrf24l01_state_t *)(arg);
           DEBUGASSERT(statep != NULL);

           *statep = dev->state;
           break;
        }

      case NRF24L01IOC_GETLASTXMITCOUNT:
        {
           FAR uint32_t *xmitcntp = (FAR uint32_t *)(arg);
           DEBUGASSERT(xmitcntp != NULL);

           *xmitcntp = dev->lastxmitcount;
           break;
        }

      case NRF24L01IOC_GETLASTPIPENO:
        {
          FAR uint32_t *lastpipep = (FAR uint32_t *)(arg);
          DEBUGASSERT(lastpipep != NULL);

          *lastpipep = dev->last_recvpipeno;
          break;
        }

      default:
        result = -ENOTTY;
        break;
    }

  sem_post(&dev->devsem);
  return result;
}

#ifndef CONFIG_DISABLE_POLL

static int nrf24l01_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
#ifndef CONFIG_WL_NRF24L01_RXSUPPORT
  /* Polling is currently implemented for data input only */
  return -ENOSYS;
#else

  FAR struct inode *inode;
  FAR struct nrf24l01_dev_s *dev;
  int result = OK;

  wvdbg("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev  = (FAR struct nrf24l01_dev_s *)inode->i_private;

  /* Exclusive access */

  if (sem_wait(&dev->devsem) < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          result = -EDEADLK;
          goto errout;
        }

      /* Check if we can accept this poll.
       * For now, only one thread can poll the device at any time (shorter / simpler code)
       */

      if (dev->pfd)
        {
          result = -EBUSY;
          goto errout;
        }

      dev->pfd = fds;
    }
  else  /* Tear it down */
    {
      dev->pfd = NULL;
    }

errout:
  sem_post(&dev->devsem);
  return result;
#endif
}

#endif

static int nrf24l01_unregister(FAR struct nrf24l01_dev_s *dev)
{
  CHECK_ARGS(dev);

  /* Release IRQ */

  nrf24l01_attachirq(dev, NULL);

  g_nrf24l01dev = NULL;

  /* Free memory */
#ifdef CONFIG_WL_NRF24L01_RXSUPPORT
  kfree(dev->rx_fifo);
#endif
  kfree(dev);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nrf24l01_register(FAR struct spi_dev_s *spi, FAR struct nrf24l01_config_s *cfg)
{
  FAR struct nrf24l01_dev_s *dev;
  int result = OK;

#ifdef CONFIG_WL_NRF24L01_RXSUPPORT
  uint8_t *rx_fifo;
#endif

  ASSERT((spi != NULL) & (cfg != NULL));

  if ((dev = kmalloc(sizeof(struct nrf24l01_dev_s))) == NULL)
    {
      return -ENOMEM;
    }

  dev->spi = spi;
  dev->config = cfg;

  dev->state = ST_UNKNOWN;
  dev->en_aa = 0;
  dev->ce_enabled = false;

  sem_init(&(dev->devsem), 0, 1);
  dev->nopens = 0;

#ifndef CONFIG_DISABLE_POLL
  dev->pfd = NULL;
#endif

  sem_init(&(dev->sem_tx), 0, 0);

#ifdef CONFIG_WL_NRF24L01_RXSUPPORT
  if ((rx_fifo = kmalloc(CONFIG_WL_NRF24L01_RXFIFO_LEN)) == NULL)
    {
      kfree(dev);
      return -ENOMEM;
    }

  dev->rx_fifo = rx_fifo;
  dev->nxt_read = 0;
  dev->nxt_write = 0;
  dev->fifo_len = 0;

  sem_init(&(dev->sem_fifo), 0, 1);
  sem_init(&(dev->sem_rx), 0, 0);
#endif

  /* Set the global reference */

  g_nrf24l01dev = dev;

  /* Configure IRQ pin  (falling edge) */

  nrf24l01_attachirq(dev, nrf24l01_irqhandler);

  /* Register the device as an input device */

  ivdbg("Registering " DEV_NAME "\n");

  result = register_driver(DEV_NAME, &nrf24l01_fops, 0666, dev);
  if (result < 0)
    {
      wdbg("register_driver() failed: %d\n", result);
      nrf24l01_unregister(dev);
    }

  return result;
}

FAR struct nrf24l01_dev_s * nrf24l01_getinstance(void)
{
  return g_nrf24l01dev;
}

/* (re)set the device in a default initial state */

int nrf24l01_init(FAR struct nrf24l01_dev_s *dev)
{
  int result = OK;
  uint8_t features;

  CHECK_ARGS(dev);
  nrf24l01_lock(dev->spi);

  /* Configure the SPI parameters now  (if we own the bus) */

  nrf24l01_configspi(dev->spi);

  /* Enable features. */

  nrf24l01_writeregbyte(dev, NRF24L01_FEATURE, NRF24L01_EN_DPL);
  features = nrf24l01_readregbyte(dev, NRF24L01_FEATURE);
  if (0 == features)
    {
      /* The ACTIVATE instruction is not documented in the nRF24L01+ docs.
       * However it is referenced / described by many sources on Internet,
       *
       * Is it for nRF24L01  (not +) hardware ?
       */

      uint8_t v = 0x73;
      nrf24l01_access(dev, MODE_WRITE, NRF24L01_ACTIVATE, &v, 1);

      features = nrf24l01_readregbyte(dev, NRF24L01_FEATURE);
      if (0 == features)
        {
          /* If FEATURES reg is still unset here, consider there is no actual hardware */

          result = -ENODEV;
          goto out;
        }
    }

  /* Set initial state */

  nrf24l01_tostate(dev, ST_POWER_DOWN);

  /* Disable all pipes */

  dev->en_pipes = 0;
  nrf24l01_writeregbyte(dev, NRF24L01_EN_RXADDR, 0);

  /* Set addr width to default   */

  dev->addrlen = CONFIG_WL_NRF24L01_DFLT_ADDR_WIDTH;
  nrf24l01_writeregbyte(dev, NRF24L01_SETUP_AW, CONFIG_WL_NRF24L01_DFLT_ADDR_WIDTH - 2);

  /* Get pipe #0 addr */

  nrf24l01_readreg(dev, NRF24L01_RX_ADDR_P0, dev->pipe0addr, dev->addrlen);

  dev->en_aa = nrf24l01_readregbyte(dev, NRF24L01_EN_AA);

  /* Flush HW fifo */

  nrf24l01_flush_rx(dev);
  nrf24l01_flush_tx(dev);

  /* Clear interrupt sources (useful ?) */

  nrf24l01_writeregbyte(dev, NRF24L01_STATUS, NRF24L01_RX_DR|NRF24L01_TX_DS|NRF24L01_MAX_RT);

out:
  nrf24l01_unlock(dev->spi);
  return result;
}

int nrf24l01_setpipeconfig(FAR struct nrf24l01_dev_s *dev, unsigned int pipeno,
    FAR const nrf24l01_pipecfg_t *pipecfg)
{
  bool dynlength;
  bool en_aa;

  CHECK_ARGS(dev && pipecfg && pipeno < NRF24L01_PIPE_COUNT);

  dynlength = (pipecfg->payload_length == NRF24L01_DYN_LENGTH);

  /* Need to enable AA to enable dynamic length payload */

  en_aa = dynlength || pipecfg->en_aa;

  nrf24l01_lock(dev->spi);

  /* Set addr  */

  int addrlen = (pipeno <= 1) ? dev->addrlen : 1;   /* Pipe 0 & 1 are the only ones to have a full length address */
  nrf24l01_writereg(dev, NRF24L01_RX_ADDR_P0 + pipeno, pipecfg->rx_addr, addrlen);

  /* Auto ack */

  if (en_aa)
    {
      dev->en_aa |= 1 << pipeno;
    }
  else
    {
      dev->en_aa &= ~(1 << pipeno);
    }

  nrf24l01_setregbit(dev, NRF24L01_EN_AA, 1 << pipeno, en_aa);

  /* Payload config */

  nrf24l01_setregbit(dev, NRF24L01_DYNPD, 1 << pipeno, dynlength);
  if (!dynlength)
    {
      nrf24l01_writeregbyte(dev, NRF24L01_RX_PW_P0 + pipeno, pipecfg->payload_length);
    }
  nrf24l01_unlock(dev->spi);

  dev->pipedatalen[pipeno] = pipecfg->payload_length;
  return OK;
}

int nrf24l01_getpipeconfig(FAR struct nrf24l01_dev_s *dev, unsigned int pipeno,
    FAR nrf24l01_pipecfg_t *pipecfg)
{
  bool dynlength;

  CHECK_ARGS(dev && pipecfg && pipeno < NRF24L01_PIPE_COUNT);

  nrf24l01_lock(dev->spi);

  /* Get pipe address  */

  int addrlen = (pipeno <= 1) ? dev->addrlen : 1;   /* Pipe 0 & 1 are the only ones to have a full length address */
  nrf24l01_readreg(dev, NRF24L01_RX_ADDR_P0 + pipeno, pipecfg->rx_addr, addrlen);

  /* Auto ack */

  pipecfg->en_aa = ((nrf24l01_readregbyte(dev, NRF24L01_EN_AA) & (1 << pipeno)) != 0);

  /* Payload config */

  dynlength = ((nrf24l01_readregbyte(dev, NRF24L01_DYNPD) & (1 << pipeno)) != 0);

  if (dynlength)
    {
      pipecfg->payload_length = NRF24L01_DYN_LENGTH;
    }
  else
    {
      pipecfg->payload_length = nrf24l01_readregbyte(dev, NRF24L01_RX_PW_P0 + pipeno);
    }

  nrf24l01_unlock(dev->spi);

  return OK;
}

int nrf24l01_enablepipe(FAR struct nrf24l01_dev_s *dev, unsigned int pipeno, bool enable)
{
  CHECK_ARGS(dev && pipeno < NRF24L01_PIPE_COUNT);

  uint8_t rxaddrval;
  uint8_t pipemask = 1 << pipeno;

  nrf24l01_lock(dev->spi);

  /* Enable pipe on nRF24L01 */

  rxaddrval = nrf24l01_readregbyte(dev, NRF24L01_EN_RXADDR);

  if (enable)
    {
      rxaddrval |= pipemask;
    }
  else
    {
      rxaddrval &= ~pipemask;
    }

  nrf24l01_writeregbyte(dev, NRF24L01_EN_RXADDR, rxaddrval);
  nrf24l01_unlock(dev->spi);

  /* Update cached value */

  dev->en_pipes = rxaddrval;

  return OK;
}

int nrf24l01_settxaddr(FAR struct nrf24l01_dev_s *dev, FAR const uint8_t *txaddr)
{
  CHECK_ARGS(dev && txaddr);

  nrf24l01_lock(dev->spi);

  nrf24l01_writereg(dev, NRF24L01_TX_ADDR, txaddr, dev->addrlen);
  nrf24l01_unlock(dev->spi);
  return OK;
}

int nrf24l01_gettxaddr(FAR struct nrf24l01_dev_s *dev, FAR uint8_t *txaddr)
{
  CHECK_ARGS(dev && txaddr);

  nrf24l01_lock(dev->spi);

  nrf24l01_readreg(dev, NRF24L01_TX_ADDR, txaddr, dev->addrlen);
  nrf24l01_unlock(dev->spi);
  return OK;
}

int nrf24l01_setretransmit(FAR struct nrf24l01_dev_s *dev, nrf24l01_retransmit_delay_t retrdelay, uint8_t retrcount)
{
  uint8_t val;

  CHECK_ARGS(dev && retrcount <= NRF24L01_MAX_XMIT_RETR);

  val = (retrdelay << NRF24L01_ARD_SHIFT) | (retrcount << NRF24L01_ARC_SHIFT);

  nrf24l01_lock(dev->spi);

  nrf24l01_writeregbyte(dev, NRF24L01_SETUP_RETR, val);
  nrf24l01_unlock(dev->spi);
  return OK;
}

int nrf24l01_settxpower(FAR struct nrf24l01_dev_s *dev, int outpower)
{
  uint8_t value;
  uint8_t hwpow;

  /**  RF_PWR value  <->  Output power in dBm
   *
   * '00' – -18dBm
   * '01' – -12dBm
   * '10' – -6dBm
   * '11' – 0dBm
   */

  switch(outpower)
    {
    case 0:
      hwpow = 3 << NRF24L01_RF_PWR_SHIFT;
      break;

    case -6:
      hwpow = 2 << NRF24L01_RF_PWR_SHIFT;
      break;

    case -12:
      hwpow = 1 << NRF24L01_RF_PWR_SHIFT;
      break;

    case -18:
      hwpow = 0;
      break;

    default:
      return -EINVAL;
  }

  nrf24l01_lock(dev->spi);

  value = nrf24l01_readregbyte(dev, NRF24L01_RF_SETUP);

  value &= ~(NRF24L01_RF_PWR_MASK);
  value |= hwpow;

  nrf24l01_writeregbyte(dev, NRF24L01_RF_SETUP, value);
  nrf24l01_unlock(dev->spi);
  return OK;
}

int nrf24l01_gettxpower(FAR struct nrf24l01_dev_s *dev)
{
  uint8_t value;
  int powers[] = { -18, -12, -6, 0};

  nrf24l01_lock(dev->spi);

  value = nrf24l01_readregbyte(dev, NRF24L01_RF_SETUP);
  nrf24l01_unlock(dev->spi);

  value = (value & NRF24L01_RF_PWR_MASK) >> NRF24L01_RF_PWR_SHIFT;
  return powers[value];
}

int nrf24l01_setdatarate(FAR struct nrf24l01_dev_s *dev, nrf24l01_datarate_t datarate)
{
  uint8_t value;

  nrf24l01_lock(dev->spi);

  value = nrf24l01_readregbyte(dev, NRF24L01_RF_SETUP);
  value &= ~(NRF24L01_RF_DR_HIGH | NRF24L01_RF_DR_LOW);

  switch (datarate)
    {
      case RATE_1Mbps:
        break;

      case RATE_2Mbps:
        value |= NRF24L01_RF_DR_HIGH;
        break;

      case RATE_250kbps:
        value |= NRF24L01_RF_DR_LOW;
        break;
    }

  nrf24l01_writeregbyte(dev, NRF24L01_RF_SETUP, value);
  nrf24l01_unlock(dev->spi);
  return OK;
}

int nrf24l01_setradiofreq(FAR struct nrf24l01_dev_s *dev, uint32_t freq)
{
  uint8_t value;

  CHECK_ARGS(dev && freq >= NRF24L01_MIN_FREQ && freq <= NRF24L01_MAX_FREQ);

  value = NRF24L01_MIN_FREQ - freq;
  nrf24l01_lock(dev->spi);
  nrf24l01_writeregbyte(dev, NRF24L01_RF_CH, value);
  nrf24l01_unlock(dev->spi);
  return OK;
}

uint32_t nrf24l01_getradiofreq(FAR struct nrf24l01_dev_s *dev)
{
  int rffreq;

  CHECK_ARGS(dev);

  nrf24l01_lock(dev->spi);
  rffreq = (int)nrf24l01_readregbyte(dev, NRF24L01_RF_CH);
  nrf24l01_unlock(dev->spi);

  return rffreq + NRF24L01_MIN_FREQ;
}

int nrf24l01_setaddrwidth(FAR struct nrf24l01_dev_s *dev, uint32_t width)
{
  CHECK_ARGS(dev && width <= NRF24L01_MAX_ADDR_LEN && width >= NRF24L01_MIN_ADDR_LEN);

  nrf24l01_lock(dev->spi);
  nrf24l01_writeregbyte(dev, NRF24L01_SETUP_AW, width-2);
  nrf24l01_unlock(dev->spi);
  dev->addrlen = width;
  return OK;
}

int nrf24l01_changestate(FAR struct nrf24l01_dev_s *dev, nrf24l01_state_t state)
{
  nrf24l01_lock(dev->spi);
  nrf24l01_tostate(dev, state);
  nrf24l01_unlock(dev->spi);
  return OK;
}

int nrf24l01_send(FAR struct nrf24l01_dev_s *dev, FAR const uint8_t *data, size_t datalen)
{
  int result;

  CHECK_ARGS(dev && data && datalen <= NRF24L01_MAX_PAYLOAD_LEN);

  nrf24l01_lock(dev->spi);

  result = dosend(dev, data, datalen);

  nrf24l01_unlock(dev->spi);
  return result;
}

int nrf24l01_sendto(FAR struct nrf24l01_dev_s *dev, FAR const uint8_t *data,
    size_t datalen, FAR const uint8_t *destaddr)
{
  bool pipeaddrchg = false;
  int result;

  nrf24l01_lock(dev->spi);

  /* If AA is enabled (pipe 0 is active and its AA flag is set) and the dest
   * addr is not the current pipe 0 addr we need to change pipe 0 addr in
   * order to receive the ACK packet.
   */

  if ((dev->en_aa & 1) && (memcmp(destaddr, dev->pipe0addr, dev->addrlen)))
    {
      wdbg("Change pipe #0 addr to dest addr\n");
      nrf24l01_writereg(dev, NRF24L01_RX_ADDR_P0, destaddr, NRF24L01_MAX_ADDR_LEN);
      pipeaddrchg = true;
    }

  result = dosend(dev, data, datalen);

  if (pipeaddrchg)
    {
      /* Restore pipe #0 addr */

      nrf24l01_writereg(dev, NRF24L01_RX_ADDR_P0, dev->pipe0addr, NRF24L01_MAX_ADDR_LEN);
      wdbg("Pipe #0 default addr restored\n");
    }

  nrf24l01_unlock(dev->spi);
  return result;
}

int nrf24l01_lastxmitcount(FAR struct nrf24l01_dev_s *dev)
{
  return dev->lastxmitcount;
}

#ifdef CONFIG_WL_NRF24L01_RXSUPPORT

ssize_t nrf24l01_recv(struct nrf24l01_dev_s *dev, uint8_t *buffer,
    size_t buflen, uint8_t *recvpipe)
{
  if (sem_wait(&dev->sem_rx) != 0)
    {
       /* This should only happen if the wait was canceled by an signal */

       DEBUGASSERT(errno == EINTR);
       return -EINTR;
    }

  return fifoget(dev, buffer, buflen, recvpipe);
}

#endif

#ifdef NRF24L01_DEBUG

static void binarycvt(char *deststr, const uint8_t *srcbin, size_t srclen)
{
  int i = 0;
  while (i < srclen)
    {
      sprintf(deststr + i*2, "%02x", srcbin[i]);
      ++i;
    }

  *(deststr + i*2) = '\0';
}

void nrf24l01_dumpregs(struct nrf24l01_dev_s *dev)
{
  uint8_t addr[NRF24L01_MAX_ADDR_LEN];
  char addrstr[NRF24L01_MAX_ADDR_LEN * 2 +1];

  syslog("CONFIG:    %02x\n", nrf24l01_readregbyte(dev, NRF24L01_CONFIG));
  syslog("EN_AA:     %02x\n", nrf24l01_readregbyte(dev, NRF24L01_EN_AA));
  syslog("EN_RXADDR: %02x\n", nrf24l01_readregbyte(dev, NRF24L01_EN_RXADDR));
  syslog("SETUP_AW:  %02x\n", nrf24l01_readregbyte(dev, NRF24L01_SETUP_AW));

  syslog("SETUP_RETR:%02x\n", nrf24l01_readregbyte(dev, NRF24L01_SETUP_RETR));
  syslog("RF_CH:     %02x\n", nrf24l01_readregbyte(dev, NRF24L01_RF_CH));
  syslog("RF_SETUP:  %02x\n", nrf24l01_readregbyte(dev, NRF24L01_RF_SETUP));
  syslog("STATUS:    %02x\n", nrf24l01_readregbyte(dev, NRF24L01_STATUS));
  syslog("OBS_TX:    %02x\n", nrf24l01_readregbyte(dev, NRF24L01_OBSERVE_TX));

  nrf24l01_readreg(dev, NRF24L01_TX_ADDR, addr, dev->addrlen);
  binarycvt(addrstr, addr, dev->addrlen);
  syslog("TX_ADDR:   %s\n", addrstr);

  syslog("CD:        %02x\n", nrf24l01_readregbyte(dev, NRF24L01_CD));
  syslog("RX_PW_P0:  %02x\n", nrf24l01_readregbyte(dev, NRF24L01_RX_PW_P0));
  syslog("RX_PW_P1:  %02x\n", nrf24l01_readregbyte(dev, NRF24L01_RX_PW_P1));
  syslog("RX_PW_P2:  %02x\n", nrf24l01_readregbyte(dev, NRF24L01_RX_PW_P2));
  syslog("RX_PW_P3:  %02x\n", nrf24l01_readregbyte(dev, NRF24L01_RX_PW_P3));
  syslog("RX_PW_P4:  %02x\n", nrf24l01_readregbyte(dev, NRF24L01_RX_PW_P4));
  syslog("RX_PW_P5:  %02x\n", nrf24l01_readregbyte(dev, NRF24L01_RX_PW_P5));

  syslog("FIFO_STAT: %02x\n", nrf24l01_readregbyte(dev, NRF24L01_FIFO_STATUS));
  syslog("DYNPD:     %02x\n", nrf24l01_readregbyte(dev, NRF24L01_DYNPD));
  syslog("FEATURE:   %02x\n", nrf24l01_readregbyte(dev, NRF24L01_FEATURE));
}

#ifdef CONFIG_WL_NRF24L01_RXSUPPORT
void nrf24l01_dumprxfifo(struct nrf24l01_dev_s *dev)
{
  syslog("bytes count: %d\n", dev->fifo_len);
  syslog("next read:   %d,  next write: %d\n", dev->nxt_read, dev-> nxt_write);
}
#endif

#endif

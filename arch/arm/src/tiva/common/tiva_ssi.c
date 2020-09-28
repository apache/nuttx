/****************************************************************************
 * arch/arm/src/tiva/common/tiva_ssi.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "tiva_enablepwr.h"
#include "tiva_enableclks.h"
#include "tiva_gpio.h"
#include "tiva_ssi.h"
#include "hardware/tiva_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_SPI enables debug output from this file */

#ifdef CONFIG_DEBUG_SPI
#  define ssi_dumpgpio(m) tiva_dumpgpio(SDCCS_GPIO, m)
#else
#  define ssi_dumpgpio(m)
#endif

/* How many SSI modules does this chip support? The LM3S6918 supports 2 SSI
 * modules, the LM3S6965 and LM3S8962 support 1 module (others may support
 * more than 2-- in such case, the following must be expanded).
 */

#if TIVA_NSSI < 1
#  undef CONFIG_TIVA_SSI0
#  undef CONFIG_TIVA_SSI1
#  undef CONFIG_TIVA_SSI2
#  undef CONFIG_TIVA_SSI3
#elif TIVA_NSSI < 2
#  undef CONFIG_TIVA_SSI1
#  undef CONFIG_TIVA_SSI2
#  undef CONFIG_TIVA_SSI3
#elif TIVA_NSSI < 3
#  undef CONFIG_TIVA_SSI2
#  undef CONFIG_TIVA_SSI3
#elif TIVA_NSSI < 4
#  undef CONFIG_TIVA_SSI3
#endif

/* Which SSI modules have been enabled? */

#ifdef CONFIG_TIVA_SSI0
#  define SSI0_NDX       0               /* Index to SSI0 in g_ssidev[] */
#  define __SSI1_NDX     1               /* Next available index */
#else
#  define __SSI1_NDX     0               /* Next available index */
#endif

#ifdef CONFIG_TIVA_SSI1
#  define SSI1_NDX       __SSI1_NDX       /* Index to SSI1 in g_ssidev[] */
#  define __SSI2_NDX     (__SSI1_NDX + 1) /* Next available index */
#else
#  define __SSI2_NDX     __SSI1_NDX       /* Next available index */
#endif

#ifdef CONFIG_TIVA_SSI2
#  define SSI2_NDX       __SSI2_NDX       /* Index to SSI2 in g_ssidev[] */
#  define __SSI3_NDX     (__SSI2_NDX + 1) /* Next available index */
#else
#  define __SSI3_NDX      __SSI2_NDX      /* Next available index */
#endif

#ifdef CONFIG_TIVA_SSI3
#  define SSI3_NDX       __SSI3_NDX       /* Index to SSI3 in g_ssidev[] */
#  define NSSI_ENABLED   (__SSI3_NDX + 1) /* Number of SSI peripheral senabled */
#else
#  define NSSI_ENABLED   __SSI3_NDX       /* Number of SSI peripheral senabled */
#endif

/* Compile the rest of the file only if at least one SSI interface has been
 * enabled.
 */

#if NSSI_ENABLED > 0

/* Some special definitions if there is exactly one interface enabled */

#if NSSI_ENABLED < 2
#  if defined(CONFIG_TIVA_SSI0)
#    define SSI_BASE TIVA_SSI0_BASE
#    define SSI_IRQ  TIVA_IRQ_SSI0
#  elif defined(CONFIG_TIVA_SSI1)
#    define SSI_BASE TIVA_SSI1_BASE
#    define SSI_IRQ  TIVA_IRQ_SSI1
#  elif defined(CONFIG_TIVA_SSI2)
#    define SSI_BASE TIVA_SSI2_BASE
#    define SSI_IRQ  TIVA_IRQ_SSI2
#  elif defined(CONFIG_TIVA_SSI3)
#    define SSI_BASE TIVA_SSI3_BASE
#    define SSI_IRQ  TIVA_IRQ_SSI3
#  else
#    error Help me... I am confused
#  endif
#endif

/* The number of (16-bit) words that will fit in the Tx FIFO */

#define TIVA_TXFIFO_WORDS 8

/* Configuration settings */

#ifndef CONFIG_SSI_TXLIMIT
#  define CONFIG_SSI_TXLIMIT (TIVA_TXFIFO_WORDS/2)
#endif

#if CONFIG_SSI_TXLIMIT < 1 || CONFIG_SSI_TXLIMIT > TIVA_TXFIFO_WORDS
#  error "Invalid range for CONFIG_SSI_TXLIMIT"
#endif

#if CONFIG_SSI_TXLIMIT && CONFIG_SSI_TXLIMIT < (TIVA_TXFIFO_WORDS/2)
#  error "CONFIG_SSI_TXLIMIT must be at least half the TX FIFO size"
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct tiva_ssidev_s
{
  const struct spi_ops_s *ops;  /* Common SPI operations */
#ifndef CONFIG_SSI_POLLWAIT
  sem_t  xfrsem;                /* Wait for transfer to complete */
#endif

  /* These following are the source and destination buffers of the transfer.
   * they are retained in this structure so that they will be accessible
   * from an interrupt handler.  The actual type of the buffer is uint8_t if
   * nbits <=8 and uint16_t if nbits >8.
   */

  void  *txbuffer;              /* Source buffer */
  void  *rxbuffer;              /* Destination buffer */

  /* These are functions pointers that are configured to perform the
   * appropriate transfer for the particular kind of exchange that is
   * occurring.  Different functions may be selected depending on (1)
   * if the tx or txbuffer is NULL and depending on the number of bits
   * per word.
   */

  void  (*txword)(struct tiva_ssidev_s *priv);
  void  (*rxword)(struct tiva_ssidev_s *priv);

#if NSSI_ENABLED > 1
  uint32_t base;                /* SSI register base address */
#endif

  int      ntxwords;            /* Number of words left to transfer on the Tx FIFO */
  int      nrxwords;            /* Number of words received on the Rx FIFO */
  int      nwords;              /* Number of words to be exchanged */
  uint8_t  nbits;               /* Current number of bits per word */

#if !defined(CONFIG_SSI_POLLWAIT) && NSSI_ENABLED > 1
  uint8_t  irq;                 /* SSI IRQ number */
#endif

  /* Enforce mutual exclusion and remember some configuration settings to
   * reduce the overhead of constant SPI re-configuration.
   */

  sem_t    exclsem;             /* For exclusive access to the SSI bus */
  uint32_t frequency;           /* Current desired SCLK frequency */
  uint32_t actual;              /* Current actual SCLK frequency */
  uint8_t  mode;                /* Current mode 0,1,2,3 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SSI register access */

static inline uint32_t ssi_getreg(struct tiva_ssidev_s *priv,
                                  unsigned int offset);
static inline void ssi_putreg(struct tiva_ssidev_s *priv,
                              unsigned int offset, uint32_t value);

/* Misc helpers */

static uint32_t ssi_disable(struct tiva_ssidev_s *priv);
static void ssi_enable(struct tiva_ssidev_s *priv, uint32_t enable);

#ifndef CONFIG_SSI_POLLWAIT
static int ssi_semtake(sem_t *sem);
#define ssi_semgive(s) nxsem_post(s);
#endif

/* SSI data transfer */

static void ssi_txnull(struct tiva_ssidev_s *priv);
static void ssi_txuint16(struct tiva_ssidev_s *priv);
static void ssi_txuint8(struct tiva_ssidev_s *priv);
static void ssi_rxnull(struct tiva_ssidev_s *priv);
static void ssi_rxuint16(struct tiva_ssidev_s *priv);
static void ssi_rxuint8(struct tiva_ssidev_s *priv);
static inline bool ssi_txfifofull(struct tiva_ssidev_s *priv);
static inline bool ssi_rxfifoempty(struct tiva_ssidev_s *priv);
#if CONFIG_SSI_TXLIMIT == 1 && defined(CONFIG_SSI_POLLWAIT)
static inline int ssi_performtx(struct tiva_ssidev_s *priv);
#else
static int  ssi_performtx(struct tiva_ssidev_s *priv);
#endif
static inline void ssi_performrx(struct tiva_ssidev_s *priv);
static int  ssi_transfer(struct tiva_ssidev_s *priv, const void *txbuffer,
                         void *rxbuffer, unsigned int nwords);

/* Interrupt handling */

#ifndef CONFIG_SSI_POLLWAIT
static inline struct tiva_ssidev_s *ssi_mapirq(int irq);
static int  ssi_interrupt(int irq, void *context, FAR void *arg);
#endif

/* SPI methods */

static int  ssi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t ssi_setfrequencyinternal(struct tiva_ssidev_s *priv,
              uint32_t frequency);
static uint32_t ssi_setfrequency(FAR struct spi_dev_s *dev,
              uint32_t frequency);
static void ssi_setmodeinternal(struct tiva_ssidev_s *priv,
              enum spi_mode_e mode);
static void ssi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void ssi_setbitsinternal(struct tiva_ssidev_s *priv, int nbits);
static void ssi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint32_t ssi_send(FAR struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static void ssi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords);
#else
static void ssi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
              size_t nwords);
static void ssi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
              size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Common SSI operations */

static const struct spi_ops_s g_spiops =
{
  .lock         = ssi_lock,
  .select       = tiva_ssiselect,    /* Provided externally by board logic */
  .setfrequency = ssi_setfrequency,
  .setmode      = ssi_setmode,
  .setbits      = ssi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = 0,                 /* Not supported */
#endif
  .status       = tiva_ssistatus,    /* Provided externally by board logic */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = tiva_ssicmddata,
#endif
  .send         = ssi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = ssi_exchange,
#else
  .sndblock     = ssi_sndblock,
  .recvblock    = ssi_recvblock,
#endif
};

/* This supports is up to two SSI buses/ports */

static struct tiva_ssidev_s g_ssidev[] =
{
#ifdef CONFIG_TIVA_SSI0
  {
    .ops  = &g_spiops,
#if NSSI_ENABLED > 1
    .base = TIVA_SSI0_BASE,
#endif
#if !defined(CONFIG_SSI_POLLWAIT) && NSSI_ENABLED > 1
    .irq  = TIVA_IRQ_SSI0,
#endif
  },
#endif
#ifdef CONFIG_TIVA_SSI1
  {
    .ops  = &g_spiops,
#if NSSI_ENABLED > 1
    .base = TIVA_SSI1_BASE,
#endif
#if !defined(CONFIG_SSI_POLLWAIT) && NSSI_ENABLED > 1
    .irq  = TIVA_IRQ_SSI1,
#endif
  },
#endif
#ifdef CONFIG_TIVA_SSI2
  {
    .ops  = &g_spiops,
#if NSSI_ENABLED > 1
    .base = TIVA_SSI2_BASE,
#endif
#if !defined(CONFIG_SSI_POLLWAIT) && NSSI_ENABLED > 1
    .irq  = TIVA_IRQ_SSI2,
#endif
  },
#endif
#ifdef CONFIG_TIVA_SSI3
  {
    .ops  = &g_spiops,
#if NSSI_ENABLED > 1
    .base = TIVA_SSI3_BASE,
#endif
#if !defined(CONFIG_SSI_POLLWAIT) && NSSI_ENABLED > 1
    .irq  = TIVA_IRQ_SSI3,
#endif
  },
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssi_getreg
 *
 * Description:
 *   Read the SSI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SSI register from the register base address
 *
 * Returned Value:
 *   Value of the register at this offset
 *
 ****************************************************************************/

static inline uint32_t ssi_getreg(struct tiva_ssidev_s *priv,
                                  unsigned int offset)
{
#if NSSI_ENABLED > 1
  return getreg32(priv->base + offset);
#else
  return getreg32(SSI_BASE + offset);
#endif
}

/****************************************************************************
 * Name: ssi_putreg
 *
 * Description:
 *   Write the value to the SSI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SSI register from the register base address
 *   value  - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ssi_putreg(struct tiva_ssidev_s *priv,
                              unsigned int offset, uint32_t value)
{
#if NSSI_ENABLED > 1
  putreg32(value, priv->base + offset);
#else
  putreg32(value, SSI_BASE + offset);
#endif
}

/****************************************************************************
 * Name: ssi_disable
 *
 * Description:
 *   Disable SSI operation.  NOTE: The SSI must be disabled before any
 *   control registers can be re-programmed.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   State of the SSI before the SSE was disabled
 *
 * Assumption:
 *   Caller holds a lock on the SPI bus
 *
 ****************************************************************************/

static uint32_t ssi_disable(struct tiva_ssidev_s *priv)
{
  uint32_t retval;
  uint32_t regval;

  retval = ssi_getreg(priv, TIVA_SSI_CR1_OFFSET);
  regval = (retval & ~SSI_CR1_SSE);
  ssi_putreg(priv, TIVA_SSI_CR1_OFFSET, regval);
  spiinfo("CR1: %08x\n", regval);
  return retval;
}

/****************************************************************************
 * Name: ssi_enable
 *
 * Description:
 *   Restore the SSI operational state
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   enable - The previous operational state
 *
 * Returned Value:
 *
 * Assumption:
 *   Caller holds a lock on the SPI bus
 *
 ****************************************************************************/

static void ssi_enable(struct tiva_ssidev_s *priv, uint32_t enable)
{
  uint32_t regval = ssi_getreg(priv, TIVA_SSI_CR1_OFFSET);
  regval &= ~SSI_CR1_SSE;
  regval  |= (enable & SSI_CR1_SSE);
  ssi_putreg(priv, TIVA_SSI_CR1_OFFSET, regval);
  spiinfo("CR1: %08x\n", regval);
}

/****************************************************************************
 * Name: ssi_semtake
 *
 * Description:
 *   Wait for a semaphore (handling interruption by signals);
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   enable - The previous operational state
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifndef CONFIG_SSI_POLLWAIT
static int ssi_semtake(sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}
#endif

/****************************************************************************
 * Name: ssi_txnull, ssi_txuint16, and ssi_txuint8
 *
 * Description:
 *   Transfer all ones, a uint8_t, or uint16_t to Tx FIFO and update the
 *   txbuffer pointer appropriately.  The selected function dependes on (1)
 *   if there is a source txbuffer provided, and (2) if the number of bits
 *   per word is <=8 or >8.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ssi_txnull(struct tiva_ssidev_s *priv)
{
  spiinfo("TX: ->0xffff\n");
  ssi_putreg(priv, TIVA_SSI_DR_OFFSET, 0xffff);
}

static void ssi_txuint16(struct tiva_ssidev_s *priv)
{
  uint16_t *ptr    = (uint16_t *)priv->txbuffer;
  spiinfo("TX: %p->%04x\n", ptr, *ptr);
  ssi_putreg(priv, TIVA_SSI_DR_OFFSET, (uint32_t)(*ptr++));
  priv->txbuffer = (void *)ptr;
}

static void ssi_txuint8(struct tiva_ssidev_s *priv)
{
  uint8_t *ptr   = (uint8_t *)priv->txbuffer;
  spiinfo("TX: %p->%02x\n", ptr, *ptr);
  ssi_putreg(priv, TIVA_SSI_DR_OFFSET, (uint32_t)(*ptr++));
  priv->txbuffer = (void *)ptr;
}

/****************************************************************************
 * Name: ssi_rxnull, ssi_rxuint16, and ssi_rxuint8
 *
 * Description:
 *   Discard input, save a uint8_t, or or save a uint16_t from Tx FIFO in the
 *   user rxvbuffer and update the rxbuffer pointer appropriately.  The
 *   selected function dependes on (1) if there is a destination rxbuffer
 *   provided, and (2) if the number of bits per word is <=8 or >8.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ssi_rxnull(struct tiva_ssidev_s *priv)
{
#ifdef CONFIG_DEBUG_SPI_INFO
  uint32_t regval  = ssi_getreg(priv, TIVA_SSI_DR_OFFSET);
  spiinfo("RX: discard %04x\n", regval);
#else
  ssi_getreg(priv, TIVA_SSI_DR_OFFSET);
#endif
}

static void ssi_rxuint16(struct tiva_ssidev_s *priv)
{
  uint16_t *ptr    = (uint16_t *)priv->rxbuffer;
  *ptr           = (uint16_t)ssi_getreg(priv, TIVA_SSI_DR_OFFSET);
  spiinfo("RX: %p<-%04x\n", ptr, *ptr);
  priv->rxbuffer = (void *)(++ptr);
}

static void ssi_rxuint8(struct tiva_ssidev_s *priv)
{
  uint8_t *ptr   = (uint8_t *)priv->rxbuffer;
  *ptr           = (uint8_t)ssi_getreg(priv, TIVA_SSI_DR_OFFSET);
  spiinfo("RX: %p<-%02x\n", ptr, *ptr);
  priv->rxbuffer = (void *)(++ptr);
}

/****************************************************************************
 * Name: ssi_txfifofull
 *
 * Description:
 *   Return true if the Tx FIFO is full
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   true: Not full
 *
 ****************************************************************************/

static inline bool ssi_txfifofull(struct tiva_ssidev_s *priv)
{
  return (ssi_getreg(priv, TIVA_SSI_SR_OFFSET) & SSI_SR_TNF) == 0;
}

/****************************************************************************
 * Name: ssi_rxfifoempty
 *
 * Description:
 *   Return true if the Rx FIFO is empty
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   true: Not empty
 *
 ****************************************************************************/

static inline bool ssi_rxfifoempty(struct tiva_ssidev_s *priv)
{
  return (ssi_getreg(priv, TIVA_SSI_SR_OFFSET) & SSI_SR_RNE) == 0;
}

/****************************************************************************
 * Name: ssi_performtx
 *
 * Description:
 *   If the Tx FIFO is empty, then transfer as many words as we can to
 *   the FIFO.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   The number of words written to the Tx FIFO (a value from 0 to 8,
 *   inclusive).
 *
 ****************************************************************************/

#if CONFIG_SSI_TXLIMIT == 1 && defined(CONFIG_SSI_POLLWAIT)
static inline int ssi_performtx(struct tiva_ssidev_s *priv)
{
  /* Check if the Tx FIFO is full and more data to transfer */

  if (!ssi_txfifofull(priv) && priv->ntxwords > 0)
    {
      /* Transfer one word to the Tx FIFO */

      priv->txword(priv);
      priv->ntxwords--;
      return 1;
    }

  return 0;
}

#else /* CONFIG_SSI_TXLIMIT == 1 CONFIG_SSI_POLLWAIT */

static int ssi_performtx(struct tiva_ssidev_s *priv)
{
#ifndef CONFIG_SSI_POLLWAIT
  uint32_t regval;
#endif
  int ntxd = 0;  /* Number of words written to Tx FIFO */

  /* Check if the Tx FIFO is full */

  if (!ssi_txfifofull(priv))
    {
      /* Not full.. Check if all of the Tx words have been sent */

      if (priv->ntxwords > 0)
        {
          /* No.. Transfer more words until either the Tx FIFO is full or
           * until all of the user provided data has been sent.
           */
#ifdef CONFIG_SSI_TXLIMIT
          /* Further limit the number of words that we put into the Tx
           * FIFO to CONFIG_SSI_TXLIMIT.  Otherwise, we could
           * overrun the Rx FIFO on a very fast SSI bus.
           */

          for (; ntxd < priv->ntxwords && ntxd < CONFIG_SSI_TXLIMIT &&
               !ssi_txfifofull(priv); ntxd++)
#else
          for (; ntxd < priv->ntxwords && !ssi_txfifofull(priv); ntxd++)
#endif
            {
               priv->txword(priv);
            }

          /* Update the count of words to be transferred */

          priv->ntxwords -= ntxd;
        }

      /* Check again... Now have all of the Tx words been sent? */

#ifndef CONFIG_SSI_POLLWAIT
      regval = ssi_getreg(priv, TIVA_SSI_IM_OFFSET);
      if (priv->ntxwords > 0)
        {
          /* No.. Enable the Tx FIFO interrupt.  This interrupt occurs
           * when the Tx FIFO is 1/2 full or less.
           */

#ifdef CONFIG_DEBUG_FEATURES
          regval |= (SSI_IM_TX | SSI_RIS_ROR);
#else
          regval |= SSI_IM_TX;
#endif
        }
      else
        {
          /* Yes.. Disable the Tx FIFO interrupt.  The final stages of
           * the transfer will be driven by Rx FIFO interrupts.
           */

          regval &= ~(SSI_IM_TX | SSI_RIS_ROR);
        }

      ssi_putreg(priv, TIVA_SSI_IM_OFFSET, regval);
#endif /* CONFIG_SSI_POLLWAIT */
    }

  return ntxd;
}

#endif /* CONFIG_SSI_TXLIMIT == 1 CONFIG_SSI_POLLWAIT */

/****************************************************************************
 * Name: ssi_performrx
 *
 * Description:
 *   Transfer as many bytes as possible from the Rx FIFO to the user Rx
 *   buffer (if one was provided).
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ssi_performrx(struct tiva_ssidev_s *priv)
{
#ifndef CONFIG_SSI_POLLWAIT
  uint32_t regval;
#endif

  /* Loop while data is available in the Rx FIFO */

  while (!ssi_rxfifoempty(priv))
    {
      /* Have all of the requested words been transferred from the Rx FIFO? */

      if (priv->nrxwords < priv->nwords)
        {
          /* No.. Read more data from Rx FIFO */

          priv->rxword(priv);
          priv->nrxwords++;
        }
    }

  /* The Rx FIFO is now empty.  While there is Tx data to be sent, the
   * transfer will be driven by Tx FIFO interrupts.  The final part
   * of the transfer is driven by Rx FIFO interrupts only.
   */

#ifndef CONFIG_SSI_POLLWAIT
  regval = ssi_getreg(priv, TIVA_SSI_IM_OFFSET);
  if (priv->ntxwords == 0 && priv->nrxwords < priv->nwords)
    {
      /* There are no more outgoing words to send, but there are
       * additional incoming words expected (I would think that this
       * a real corner case, be we will handle it with an extra
       * interrupt, probably an Rx timeout).
       */

#ifdef CONFIG_DEBUG_FEATURES
      regval |= (SSI_IM_RX | SSI_IM_RT | SSI_IM_ROR);
#else
      regval |= (SSI_IM_RX | SSI_IM_RT);
#endif
    }
  else
    {
      /* No.. there are either more Tx words to send or all Rx words
       * have received.  Disable Rx FIFO interrupts.
       */

      regval &= ~(SSI_IM_RX | SSI_IM_RT);
    }

  ssi_putreg(priv, TIVA_SSI_IM_OFFSET, regval);
#endif /* CONFIG_SSI_POLLWAIT */
}

/****************************************************************************
 * Name: ssi_transfer
 *
 * Description:
 *   Exchange a block data with the SPI device
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   txbuffer - The buffer of data to send to the device (may be NULL).
 *   rxbuffer - The buffer to receive data from the device (may be NULL).
 *   nwords   - The total number of words to be exchanged.  If the interface
 *              uses <= 8 bits per word, then this is the number of
 *              uint8_t's; if the interface uses >8 bits per word, then this
 *              is the number of uint16_t's
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 * Assumption:
 *   Caller holds a lock on the SPI bus
 *
 ****************************************************************************/

static int ssi_transfer(struct tiva_ssidev_s *priv, const void *txbuffer,
                        void *rxbuffer, unsigned int nwords)
{
#ifndef CONFIG_SSI_POLLWAIT
  irqstate_t flags;
#endif
  int ntxd;

  spiinfo("txbuffer: %p rxbuffer: %p nwords: %d\n",
         txbuffer, rxbuffer, nwords);

  /* Set up to perform the transfer */

  priv->txbuffer     = (uint8_t *)txbuffer; /* Source buffer */
  priv->rxbuffer     = (uint8_t *)rxbuffer; /* Destination buffer */
  priv->ntxwords     = nwords;              /* Number of words left to send */
  priv->nrxwords     = 0;                   /* Number of words received */
  priv->nwords       = nwords;              /* Total number of exchanges */

  /* Set up the low-level data transfer function pointers */

  if (priv->nbits > 8)
    {
      priv->txword = ssi_txuint16;
      priv->rxword = ssi_rxuint16;
    }
  else
    {
      priv->txword = ssi_txuint8;
      priv->rxword = ssi_rxuint8;
    }

  if (!txbuffer)
    {
      priv->txword = ssi_txnull;
    }

  if (!rxbuffer)
    {
      priv->rxword = ssi_rxnull;
    }

  /* Prime the Tx FIFO to start the sequence (saves one interrupt).
   * At this point, all SSI interrupts should be disabled, but the
   * operation of ssi_performtx() will set up the interrupts
   * approapriately (if nwords > TxFIFO size).
   */

#ifndef CONFIG_SSI_POLLWAIT
  flags = enter_critical_section();
  spiinfo("ntxwords: %d nrxwords: %d nwords: %d SR: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, TIVA_SSI_SR_OFFSET));

  ntxd  = ssi_performtx(priv);
  UNUSED(ntxd);

  /* For the case where nwords < Tx FIFO size, ssi_performrx will
   * configure interrupts correctly for the final phase of the
   * the transfer.
   */

  ssi_performrx(priv);

  spiinfo("ntxwords: %d nrxwords: %d nwords: %d SR: %08x IM: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, TIVA_SSI_SR_OFFSET),
          ssi_getreg(priv, TIVA_SSI_IM_OFFSET));

  /* Wait for the transfer to complete.  Since there is no handshake
   * with SPI, the following should complete even if there are problems
   * with the transfer, so it should be safe with no timeout.
   */

  spiinfo("Waiting for transfer complete\n");
  leave_critical_section(flags);
  do
    {
      ret = ssi_semtake(&priv->xfrsem);
    }
  while (priv->nrxwords < priv->nwords && ret >= 0);

  spiinfo("Transfer complete\n");

#else
  /* Perform the transfer using polling logic.  This will totally
   * dominate the CPU until the transfer is complete.  Only recommended
   * if (1) your SPI is very fast, and (2) if you only use very short
   * transfers.
   */

  do
    {
      /* Handle outgoing Tx FIFO transfers */

      ntxd = ssi_performtx(priv);
      UNUSED(ntxd);

      /* Handle incoming Rx FIFO transfers */

      ssi_performrx(priv);

      /* If there are other threads at this same priority level,
       * the following may help:
       */

      sched_yield();
    }
  while (priv->nrxwords < priv->nwords);
#endif

  return OK;
}

/****************************************************************************
 * Name: ssi_mapirq
 *
 * Description:
 *   Map an IRQ number into the appropriate SSI device
 *
 * Input Parameters:
 *   irq   - The IRQ number to be mapped
 *
 * Returned Value:
 *   On success, a reference to the private data structgure for this IRQ.
 *   NULL on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_SSI_POLLWAIT
static inline struct tiva_ssidev_s *ssi_mapirq(int irq)
{
  switch (irq)
    {
#ifdef CONFIG_TIVA_SSI0
      case TIVA_IRQ_SSI0:
        return &g_ssidev[SSI0_NDX];
#endif
#ifdef CONFIG_TIVA_SSI1
      case TIVA_IRQ_SSI1:
        return &g_ssidev[SSI1_NDX];
#endif
#ifdef CONFIG_TIVA_SSI2
      case TIVA_IRQ_SSI2:
        return &g_ssidev[SSI2_NDX];
#endif
#ifdef CONFIG_TIVA_SSI3
      case TIVA_IRQ_SSI3:
        return &g_ssidev[SSI3_NDX];
#endif
      default:
        return NULL;
    }
}
#endif

/****************************************************************************
 * Name: ssi_interrupt
 *
 * Description:
 *   Exchange a block data with the SSI device
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   txbuffer - The buffer of data to send to the device (may be NULL).
 *   rxbuffer - The buffer to receive data from the device (may be NULL).
 *   nwords   - The total number of words to be exchanged.  If the interface
 *              uses <= 8 bits per word, then this is the number of
 *              uint8_t's; if the interface uses >8 bits per word, then this
 *              is the number of uint16_t's
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SSI_POLLWAIT
static int ssi_interrupt(int irq, void *context, FAR void *arg)
{
  struct tiva_ssidev_s *priv = ssi_mapirq(irq);
  uint32_t regval;

  DEBUGASSERT(priv != NULL);

  /* Clear pending interrupts */

  regval = ssi_getreg(priv, TIVA_SSI_RIS_OFFSET);
  ssi_putreg(priv, TIVA_SSI_ICR_OFFSET, regval);

  /* Check for Rx FIFO overruns */

#ifdef CONFIG_DEBUG_SPI_ERROR
  if ((regval & SSI_RIS_ROR) != 0)
    {
      spierr("ERROR: Rx FIFO Overrun!\n");
    }
#endif

  spiinfo("ntxwords: %d nrxwords: %d nwords: %d SR: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, TIVA_SSI_SR_OFFSET));

  /* Handle outgoing Tx FIFO transfers */

  ssi_performtx(priv);

  /* Handle incoming Rx FIFO transfers */

  ssi_performrx(priv);

  spiinfo("ntxwords: %d nrxwords: %d nwords: %d SR: %08x IM: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, TIVA_SSI_SR_OFFSET),
          ssi_getreg(priv, TIVA_SSI_IM_OFFSET));

  /* Check if the transfer is complete */

  if (priv->nrxwords >= priv->nwords)
    {
      /* Yes.. Disable all SSI interrupt sources */

      ssi_putreg(priv, TIVA_SSI_IM_OFFSET, 0);

      /* Wake up the waiting thread */

      spiinfo("Transfer complete\n");
      ssi_semgive(&priv->xfrsem);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: ssi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int ssi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct tiva_ssidev_s *priv = (FAR struct tiva_ssidev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&priv->exclsem);
    }
  else
    {
      ret = nxsem_post(&priv->exclsem);
    }

  return ret;
}

/****************************************************************************
 * Name: ssi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 * Assumption:
 *   Caller holds a lock on the SPI bus
 *
 ****************************************************************************/

static uint32_t ssi_setfrequencyinternal(struct tiva_ssidev_s *priv,
                                         uint32_t frequency)
{
  uint32_t maxdvsr;
  uint32_t cpsdvsr;
  uint32_t regval;
  uint32_t scr;
  uint32_t actual;

  spiinfo("frequency: %d\n", frequency);
  DEBUGASSERT(frequency);

  /* Has the frequency changed? */

  if (frequency != priv->frequency)
    {
      /* "The serial bit rate is derived by dividing down the input clock
       *  (FSysClk). The clock is first divided by an even prescale value
       *  CPSDVSR from 2 to 254, which is programmed in the SSI Clock
       *  Prescale (SSI_CPSR) register ... The clock is further divided by
       *  a value from 1 to 256, which is 1 + SCR, where SCR is the value
       *  programmed in the SSI Control0 (SSICR0) register ...
       *
       * "The frequency of the output clock SSIClk is defined by:
       *
       *    "SSIClk = FSysClk / (CPSDVSR * (1 + SCR))
       *
       * "Note: Although the SSIClk transmit clock can theoretically be
       *  25 MHz, the module may not be able to operate at that speed. For
       *  master mode, the system clock must be at least two times faster
       *  than the SSIClk.  For slave mode, the system clock must be at
       *  least 12 times faster than the SSIClk."
       */

      if (frequency > SYSCLK_FREQUENCY / 2)
        {
          frequency = SYSCLK_FREQUENCY / 2;
        }

      /* Find optimal values for CPSDVSR and SCR.  This loop is inefficient,
       * but should not have to execute many times.
       *
       * EXAMPLE 1: SYSCLK_FREQUENCY=50,000,0000 and frequency=400,000.
       *
       *   maxcvsr = 125
       *   1. cpsdvsr = 2, scr = 61 -> DONE
       *
       *   This would correspond to an actual frequency of:
       *   50,000,000 / (2 * (62)) = 403,226
       *
       * EXAMPLE 2: SYSCLK_FREQUENCY=50,000,0000 and frequency=25,000,000.
       *
       *   maxcvsr = 2
       *   1. cpsdvsr = 2, scr = 0 -> DONE
       *
       *   This would correspond to an actual frequency of:
       *   50,000,000 / (2 * (1)) = 25,000,000
       */

      maxdvsr = SYSCLK_FREQUENCY / frequency;
      cpsdvsr = 0;
      do
        {
          cpsdvsr += 2;
          scr = (maxdvsr / cpsdvsr) - 1;
        }
      while (scr > 255);

      /* Set CPDVSR */

      DEBUGASSERT(cpsdvsr < 255);
      ssi_putreg(priv, TIVA_SSI_CPSR_OFFSET, cpsdvsr);

      /* Set SCR */

      regval = ssi_getreg(priv, TIVA_SSI_CR0_OFFSET);
      regval &= ~SSI_CR0_SCR_MASK;
      regval |= (scr << SSI_CR0_SCR_SHIFT);
      ssi_putreg(priv, TIVA_SSI_CR0_OFFSET, regval);
      spiinfo("CR0: %08x CPSR: %08x\n", regval, cpsdvsr);

      /* Calculate the actual frequency */

      actual = SYSCLK_FREQUENCY / (cpsdvsr * (scr + 1));

      /* Save the frequency selection so that subsequent reconfigurations
       * will be faster.
       */

      priv->frequency = frequency;
      priv->actual    = actual;
    }

  return priv->actual;
}

static uint32_t ssi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  struct tiva_ssidev_s *priv = (struct tiva_ssidev_s *)dev;
  uint32_t enable;
  uint32_t actual;

  /* NOTE that the SSI must be disabled when setting any configuration
   * registers.
   */

  enable = ssi_disable(priv);
  actual = ssi_setfrequencyinternal(priv, frequency);
  ssi_enable(priv, enable);
  return actual;
}

/****************************************************************************
 * Name: ssi_setmode
 *
 * Description:
 *   Set the SPI mode. Optional.  See enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 * Assumption:
 *   Caller holds a lock on the SPI bus
 *
 ****************************************************************************/

static void ssi_setmodeinternal(struct tiva_ssidev_s *priv,
                                enum spi_mode_e mode)
{
  uint32_t modebits;
  uint32_t regval;

  spiinfo("mode: %d\n", mode);
  DEBUGASSERT(priv);

  /* Has the number of bits per word changed? */

  if (mode != priv->mode)
    {
      /* Select the CTL register bits based on the selected mode */

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0 CHPHA=0 */
          modebits = 0;
          break;

        case SPIDEV_MODE1: /* CPOL=0 CHPHA=1 */
          modebits = SSI_CR0_SPH;
          break;

        case SPIDEV_MODE2: /* CPOL=1 CHPHA=0 */
          modebits = SSI_CR0_SPO;
         break;

        case SPIDEV_MODE3: /* CPOL=1 CHPHA=1 */
          modebits = SSI_CR0_SPH | SSI_CR0_SPO;
          break;

        default:
          return;
        }

      /* Then set the selected mode: Freescale SPI format, mode0-3 */

      regval  = ssi_getreg(priv, TIVA_SSI_CR0_OFFSET);
      regval &= ~(SSI_CR0_FRF_MASK | SSI_CR0_SPH | SSI_CR0_SPO);
      regval |= modebits;
      ssi_putreg(priv, TIVA_SSI_CR0_OFFSET, regval);
      spiinfo("CR0: %08x\n", regval);

      /* Save the mode so that subsequent re-configuratins will be faster */

      priv->mode = mode;
    }
}

static void ssi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct tiva_ssidev_s *priv = (struct tiva_ssidev_s *)dev;
  uint32_t enable;

  /* NOTE that the SSI must be disabled when setting any configuration
   * registers.
   */

  enable = ssi_disable(priv);
  ssi_setmodeinternal(priv, mode);
  ssi_enable(priv, enable);
}

/****************************************************************************
 * Name: ssi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 * Assumption:
 *   Caller holds a lock on the SPI bus
 *
 ****************************************************************************/

static void ssi_setbitsinternal(struct tiva_ssidev_s *priv, int nbits)
{
  uint32_t regval;

  spiinfo("nbits: %d\n", nbits);
  DEBUGASSERT(priv);
  if (nbits != priv->nbits && nbits >= 4 && nbits <= 16)
    {
      regval  = ssi_getreg(priv, TIVA_SSI_CR0_OFFSET);
      regval &= ~SSI_CR0_DSS_MASK;
      regval |= ((nbits - 1) << SSI_CR0_DSS_SHIFT);
      ssi_putreg(priv, TIVA_SSI_CR0_OFFSET, regval);
      spiinfo("CR0: %08x\n", regval);

      priv->nbits = nbits;
    }
}

static void ssi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  struct tiva_ssidev_s *priv = (struct tiva_ssidev_s *)dev;
  uint32_t enable;

  /* NOTE that the SSI must be disabled when setting any configuration
   * registers.
   */

  enable = ssi_disable(priv);
  ssi_setbitsinternal(priv, nbits);
  ssi_enable(priv, enable);
}

/****************************************************************************
 * Name: ssi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t ssi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  struct tiva_ssidev_s *priv = (struct tiva_ssidev_s *)dev;
  uint32_t response = 0;

  ssi_transfer(priv, &wd, &response, 1);
  return response;
}

/****************************************************************************
 * Name: SPI_EXCHANGE
 *
 * Description:
 *   Exahange a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   buffer   - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
static void ssi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  struct tiva_ssidev_s *priv = (struct tiva_ssidev_s *)dev;
  ssi_transfer(priv, txbuffer, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: ssi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void ssi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
                         size_t nwords)
{
  struct tiva_ssidev_s *priv = (struct tiva_ssidev_s *)dev;
  ssi_transfer(priv, buffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: ssi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface.  If nbits <= 8,
 *            the data is packed into uint8_t's; if nbits >8, the data is
 *            packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void ssi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                          size_t nwords)
{
  struct tiva_ssidev_s *priv = (struct tiva_ssidev_s *)dev;
  ssi_transfer(priv, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_ssibus_initialize
 *
 * Description:
 *   Initialize common parts the selected SPI port.  Initialization of
 *   chip select GPIOs must have been performed by board specific logic
 *   prior to calling this function.  Specifically:  GPIOs should have
 *   been configured for output, and all chip selects disabled.
 *
 *   One GPIO, SS (PB2 on the eZ8F091) is reserved as a chip select.
 *   However, if multiple devices on on the bus, then multiple chip selects
 *   will be required.  Therefore, all GPIO chip management is deferred to
 *   board-specific logic.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SSI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *tiva_ssibus_initialize(int port)
{
  struct tiva_ssidev_s *priv;
  irqstate_t flags;

  spiinfo("port: %d\n", port);

  /* Set up for the selected port */

  flags = enter_critical_section();
  switch (port)
    {
#ifdef CONFIG_TIVA_SSI0
    case 0:

      /* Select SSI0 */

      priv = &g_ssidev[SSI0_NDX];

      /* Enable power and clocking to the SSI0 peripheral.
       *
       * - Enable Power (TM4C129 family only):  Applies power (only) to the
       *   SSI0 peripheral.  This is not an essential step since enabling
       *   clocking will also apply power.  The only significance is that
       *   the SSI0 state will be retained if the SSI0 clocking is
       *   subsequently disabled.
       * - Enable Clocking (All families):  Applies both power and clocking
       *   to the SSI0 peripheral, bringing it a fully functional state.
       */

      tiva_ssi0_enablepwr();
      tiva_ssi0_enableclk();

      /* Configure SSI0 GPIOs (NOTE that SSI0Fss is not initialized here,
       * the logic in this file makes no assumptions about chip select).
       */

      tiva_configgpio(GPIO_SSI0_CLK);  /* PA2: SSI0 clock (SSI0Clk) */
      tiva_configgpio(GPIO_SSI0_RX);   /* PA4: SSI0 receive (SSI0Rx) */
      tiva_configgpio(GPIO_SSI0_TX);   /* PA5: SSI0 transmit (SSI0Tx) */
      break;
#endif /* CONFIG_TIVA_SSI0 */

#ifdef CONFIG_TIVA_SSI1
    case 1:

      /* Select SSI1 */

      priv = &g_ssidev[SSI1_NDX];

      /* Enable power and clocking to the SSI1 peripheral.
       *
       * - Enable Power (TM4C129 family only):  Applies power (only) to the
       *   SSI1 peripheral.  This is not an essential step since enabling
       *   clocking will also apply power.  The only significance is that
       *   the SSI1 state will be retained if the SSI1 clocking is
       *   subsequently disabled.
       * - Enable Clocking (All families):  Applies both power and clocking
       *   to the SSI1 peripheral, bringing it a fully functional state.
       */

      tiva_ssi1_enablepwr();
      tiva_ssi1_enableclk();

      /* Configure SSI1 GPIOs (except for SSI1Fss) */

      tiva_configgpio(GPIO_SSI1_CLK);  /* PE0: SSI1 clock (SSI1Clk) */
      tiva_configgpio(GPIO_SSI1_RX);   /* PE2: SSI1 receive (SSI1Rx) */
      tiva_configgpio(GPIO_SSI1_TX);   /* PE3: SSI1 transmit (SSI1Tx) */
      break;
#endif /* CONFIG_TIVA_SSI1 */

#ifdef CONFIG_TIVA_SSI2
    case 2:

      /* Select SSI2 */

      priv = &g_ssidev[SSI2_NDX];

      /* Enable power and clocking to the SSI2 peripheral.
       *
       * - Enable Power (TM4C129 family only):  Applies power (only) to the
       *   SSI2 peripheral.  This is not an essential step since enabling
       *   clocking will also apply power.  The only significance is that
       *   the SSI2 state will be retained if the SSI2 clocking is
       *   subsequently disabled.
       * - Enable Clocking (All families):  Applies both power and clocking
       *   to the SSI2 peripheral, bringing it a fully functional state.
       */

      tiva_ssi2_enablepwr();
      tiva_ssi2_enableclk();

      /* Configure SSI2 GPIOs (except for SSI2Fss) */

      tiva_configgpio(GPIO_SSI2_CLK);  /* PE0: SSI2 clock (SSI2Clk) */
      tiva_configgpio(GPIO_SSI2_RX);   /* PE2: SSI2 receive (SSI2Rx) */
      tiva_configgpio(GPIO_SSI2_TX);   /* PE3: SSI2 transmit (SSI2Tx) */
      break;
#endif /* CONFIG_TIVA_SSI2 */

#ifdef CONFIG_TIVA_SSI3
    case 3:

      /* Select SSI3 */

      priv = &g_ssidev[SSI3_NDX];

      /* Enable power and clocking to the SSI3 peripheral.
       *
       * - Enable Power (TM4C129 family only):  Applies power (only) to the
       *   SSI3 peripheral.  This is not an essential step since enabling
       *   clocking will also apply power.  The only significance is that
       *   the SSI3 state will be retained if the SSI3 clocking is
       *   subsequently disabled.
       * - Enable Clocking (All families):  Applies both power and clocking
       *   to the SSI3 peripheral, bringing it a fully functional state.
       */

      tiva_ssi3_enablepwr();
      tiva_ssi3_enableclk();

      /* Configure SSI3 GPIOs (except for SSI3Fss) */

      tiva_configgpio(GPIO_SSI3_CLK);  /* PE0: SSI3 clock (SSI3Clk) */
      tiva_configgpio(GPIO_SSI3_RX);   /* PE2: SSI3 receive (SSI3Rx) */
      tiva_configgpio(GPIO_SSI3_TX);   /* PE3: SSI3 transmit (SSI3Tx) */
      break;
#endif /* CONFIG_TIVA_SSI3 */

    default:
      leave_critical_section(flags);
      return NULL;
    }

  /* Initialize the state structure */

#ifndef CONFIG_SSI_POLLWAIT
  /* The xfrsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->xfrsem, 0, 0);
  nxsem_set_protocol(&priv->xfrsem, SEM_PRIO_NONE);
#endif
  nxsem_init(&priv->exclsem, 0, 1);

  /* Set all CR1 fields to reset state.  This will be master mode. */

  ssi_putreg(priv, TIVA_SSI_CR1_OFFSET, 0);

  /* Set all CR0 fields to the reset state. This will also select Freescale
   * SPI mode.
   */

  ssi_putreg(priv, TIVA_SSI_CR0_OFFSET, 0);

  /* Set the initial mode to mode 0.  The application may override
   * this initial setting using the setmode() method.
   */

  ssi_setmodeinternal(priv, SPIDEV_MODE0);

  /* Set the initial data width to 8-bits.  The application may
   * override this initial setting using the setbits() method.
   */

  ssi_setbitsinternal(priv, 8);

  /* Pick some initialize clock frequency. 400,000Hz is the startup
   * MMC/SD frequency used for card detection.  The application may
   * override this setting using the setfrequency() method.
   */

  ssi_setfrequencyinternal(priv, 400000);

  /* Disable all SSI interrupt sources.  They will be enabled only
   * while there is an SSI transfer in progress.
   */

  ssi_putreg(priv, TIVA_SSI_IM_OFFSET, 0);

  /* Attach the interrupt */

#ifndef CONFIG_SSI_POLLWAIT
#if NSSI_ENABLED > 1
  irq_attach(priv->irq, (xcpt_t)ssi_interrupt, NULL);
#else
  irq_attach(SSI_IRQ, (xcpt_t)ssi_interrupt, NULL);
#endif
#endif /* CONFIG_SSI_POLLWAIT */

  /* Enable the SSI for operation */

  ssi_enable(priv, SSI_CR1_SSE);

  /* Enable SSI interrupts (They are still disabled at the source). */

#ifndef CONFIG_SSI_POLLWAIT
#if NSSI_ENABLED > 1
  up_enable_irq(priv->irq);
#else
  up_enable_irq(SSI_IRQ);
#endif
#endif /* CONFIG_SSI_POLLWAIT */

  leave_critical_section(flags);
  return (FAR struct spi_dev_s *)priv;
}

#endif /* NSSI_ENABLED > 0 */

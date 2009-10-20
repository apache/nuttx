/****************************************************************************
 * arch/arm/src/lm32/lm3s_spi.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "lm3s_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG with
 * CONFIG_DEBUG_VERBOSE too)
 */

#undef SSI_DEBUG  /* Define to enable debug */

#ifdef SSI_DEBUG
#  define ssidbg  lldbg
#  define ssivdbg llvdbg
#else
#  define ssidbg(x...)
#  define ssivdbg(x...)
#endif

/* How many SSI modules does this chip support? The LM3S6918 supports 2 SSI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */

#if LM3S_NSSI == 0
#  undef CONFIG_SSI0_DISABLE
#  define CONFIG_SSI0_DISABLE 1
#  undef CONFIG_SSI1_DISABLE
#  define CONFIG_SSI1_DISABLE 1
#elif LM3S_NSSI == 1
#  undef CONFIG_SSI1_DISABLE
#  define CONFIG_SSI1_DISABLE 1
#endif

/* Which SSI modules have been enabled? */

#ifndef CONFIG_SSI0_DISABLE
#  define SSI0_NDX 0             /* Index to SSI0 in g_ssidev[] */
#  ifndef CONFIG_SSI1_DISABLE
#   define SSI1_NDX 1            /* Index to SSI1 in g_ssidev[] */
#   define NSSI_ENABLED 2        /* Two SSI interfaces: SSI0 & SSI1 */
#  else
#   define NSSI_ENABLED 1        /* One SSI interface: SSI0 */
#   define SSI_BASE          LM3S_SSI0_BASE
#   define SSI_IRQ           LM3S_IRQ_SSI0
#  endif
#else
#  ifndef CONFIG_SSI1_DISABLE
#   define SSI1_NDX 0            /* Index to SSI1 in g_ssidev[] */
#   define NSSI_ENABLED 1        /* One SSI interface: SSI1 */
#   define SSI_BASE          LM3S_SSI1_BASE
#   define SSI_IRQ           LM3S_IRQ_SSI1
#  else
#   define NSSI_ENABLED 0        /* No SSI interfaces */
#  endif
#endif

/* Compile the rest of the file only if at least one SSI interface has been
 * enabled.
 */

#if NSSI_ENABLED > 0

/* The number of (16-bit) words that will fit in the Tx FIFO */

#define LM3S_TXFIFO_WORDS 8

/* Configuration settings */

#ifndef CONFIG_SSI_TXLIMIT
#  define CONFIG_SSI_TXLIMIT (LM3S_TXFIFO_WORDS/2)
#endif

#if CONFIG_SSI_TXLIMIT < 1 || CONFIG_SSI_TXLIMIT > LM3S_TXFIFO_WORDS
#  error "Invalid range for CONFIG_SSI_TXLIMIT"
#endif

#if CONFIG_SSI_TXLIMIT && CONFIG_SSI_TXLIMIT < (LM3S_TXFIFO_WORDS/2)
#  error "CONFIG_SSI_TXLIMIT must be at least half the TX FIFO size"
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct lm3s_ssidev_s
{
  const struct spi_ops_s *ops;  /* Common SPI operations */
#ifndef CONFIG_SSI_POLLWAIT
  sem_t  xfrsem;                /* Wait for transfer to complete */
#endif
  sem_t  exclsem;               /* For exclusive access to the SSI bus */

  /* These following are the source and destination buffers of the transfer.
   * they are retained in this structure so that they will be accessible
   * from an interrupt handler.  The actual type of the buffer is ubyte is
   * nbits <=8 and uint16 is nbits >8.
   */

  void  *txbuffer;              /* Source buffer */
  void  *rxbuffer;              /* Destination buffer */

  /* These are functions pointers that are configured to perform the
   * appropriate transfer for the particular kind of exchange that is
   * occurring.  Differnt functions may be selected depending on (1)
   * if the tx or txbuffer is NULL and depending on the number of bits
   * per word.
   */

  void  (*txword)(struct lm3s_ssidev_s *priv);
  void  (*rxword)(struct lm3s_ssidev_s *priv);

#if NSSI_ENABLED > 1
  uint32 base;                  /* SSI register base address */
#endif
  uint32 frequency;             /* Current desired SCLK frequency */
  uint32 actual;                /* Current actual SCLK frequency */

  int    ntxwords;              /* Number of words left to transfer on the Tx FIFO */
  int    nrxwords;              /* Number of words received on the Rx FIFO */
  int    nwords;                /* Number of words to be exchanged */

  ubyte  mode;                  /* Current mode */
  ubyte  nbits;                 /* Current number of bits per word */
#if !defined(CONFIG_SSI_POLLWAIT) && NSSI_ENABLED > 1
  ubyte  irq;                   /* SSI IRQ number */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SSI register access */

static inline uint32 ssi_getreg(struct lm3s_ssidev_s *priv, unsigned int offset);
static inline void ssi_putreg(struct lm3s_ssidev_s *priv, unsigned int offset, uint32 value);

/* Misc helpers */

static uint32 ssi_disable(struct lm3s_ssidev_s *priv);
static void ssi_enable(struct lm3s_ssidev_s *priv, uint32 enable);
static void ssi_semtake(sem_t *sem);
#define ssi_semgive(s) sem_post(s);

/* SSI data transfer */

static void   ssi_txnull(struct lm3s_ssidev_s *priv);
static void   ssi_txuint16(struct lm3s_ssidev_s *priv);
static void   ssi_txubyte(struct lm3s_ssidev_s *priv);
static void   ssi_rxnull(struct lm3s_ssidev_s *priv);
static void   ssi_rxuint16(struct lm3s_ssidev_s *priv);
static void   ssi_rxubyte(struct lm3s_ssidev_s *priv);
static inline boolean ssi_txfifofull(struct lm3s_ssidev_s *priv);
static inline boolean ssi_rxfifoempty(struct lm3s_ssidev_s *priv);
#if CONFIG_SSI_TXLIMIT == 1 && defined(CONFIG_SSI_POLLWAIT)
static inline int ssi_performtx(struct lm3s_ssidev_s *priv);
#else
static int    ssi_performtx(struct lm3s_ssidev_s *priv);
#endif
static inline void ssi_performrx(struct lm3s_ssidev_s *priv);
static int    ssi_transfer(struct lm3s_ssidev_s *priv, const void *txbuffer,
                           void *rxbuffer, unsigned int nwords);

/* Interrupt handling */

#ifndef CONFIG_SSI_POLLWAIT
static inline struct lm3s_ssidev_s *ssi_mapirq(int irq);
static int    ssi_interrupt(int irq, void *context);
#endif

/* SPI methods */

static void   ssi_setfrequencyinternal(struct lm3s_ssidev_s *priv, uint32 frequency);
static uint32 ssi_setfrequency(FAR struct spi_dev_s *dev, uint32 frequency);
static void   ssi_setmodeinternal(struct lm3s_ssidev_s *priv, enum spi_mode_e mode);
static void   ssi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void   ssi_setbitsinternal(struct lm3s_ssidev_s *priv, int nbits);
static void   ssi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16 ssi_send(FAR struct spi_dev_s *dev, uint16 wd);
#ifdef CONFIG_SPI_EXCHANGE
static void   ssi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                           FAR void *rxbuffer, size_t nwords);
#else
static void   ssi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords);
static void   ssi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Common SSI operations */

static const struct spi_ops_s g_spiops =
{
  .lock         = 0,                 /* Not yet implemented */
  .select       = lm3s_spiselect,    /* Provided externally by board logic */
  .setfrequency = ssi_setfrequency,
  .setmode      = ssi_setmode,
  .setbits      = ssi_setbits,
  .status       = lm3s_spistatus,    /* Provided externally by board logic */
  .send         = ssi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = ssi_exchange,
#else
  .sndblock     = ssi_sndblock,
  .recvblock    = ssi_recvblock,
#endif
};

/* This supports is up to two SSI busses/ports */

static struct lm3s_ssidev_s g_ssidev[] =
{
#ifndef CONFIG_SSI0_DISABLE
  {
    .ops  = &g_spiops,
#if NSSI_ENABLED > 1
    .base = LM3S_SSI0_BASE,
#endif
#if !defined(CONFIG_SSI_POLLWAIT) && NSSI_ENABLED > 1
    .irq  = LM3S_IRQ_SSI0,
#endif
  },
#endif
#ifndef CONFIG_SSI1_DISABLE
  {
    .ops  = &g_spiops,
#if NSSI_ENABLED > 1
    .base = LM3S_SSI1_BASE,
#endif
#if !defined(CONFIG_SSI_POLLWAIT) && NSSI_ENABLED > 1
    .irq  = LM3S_IRQ_SSI1,
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

static inline uint32 ssi_getreg(struct lm3s_ssidev_s *priv, unsigned int offset)
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

static inline void ssi_putreg(struct lm3s_ssidev_s *priv, unsigned int offset, uint32 value)
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
 *   Disable SSI operation.  NOTE: The SSI must be disabled before any control
 *   registers can be re-programmed.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   State of the SSI before the SSE was disabled
 *
 ****************************************************************************/

static uint32 ssi_disable(struct lm3s_ssidev_s *priv)
{
  uint32 retval;
  uint32 regval;

  retval = ssi_getreg(priv, LM3S_SSI_CR1_OFFSET);
  regval = (retval & ~SSI_CR1_SSE);
  ssi_putreg(priv, LM3S_SSI_CR1_OFFSET, regval);
  ssivdbg("CR1: %08x\n", regval);
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
 ****************************************************************************/

static void ssi_enable(struct lm3s_ssidev_s *priv, uint32 enable)
{
  uint32 regval = ssi_getreg(priv, LM3S_SSI_CR1_OFFSET);
  regval &= ~SSI_CR1_SSE;
  regval  |= (enable & SSI_CR1_SSE);
  ssi_putreg(priv, LM3S_SSI_CR1_OFFSET, regval);
  ssivdbg("CR1: %08x\n", regval);
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

static void ssi_semtake(sem_t *sem)
{
  int ret;
  do
    {
      ret = sem_wait(sem);
    }
  while (ret < 0 && errno == EINTR);
  DEBUGASSERT(ret == 0);
}

/****************************************************************************
 * Name: ssi_txnull, ssi_txuint16, and ssi_txubyte
 *
 * Description:
 *   Transfer all ones, a ubyte, or uint16 to Tx FIFO and update the txbuffer
 *   pointer appropriately.  The selected function dependes on (1) if there
 *   is a source txbuffer provided, and (2) if the number of bits per
 *   word is <=8 or >8.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ssi_txnull(struct lm3s_ssidev_s *priv)
{
  ssivdbg("TX: ->0xffff\n");
  ssi_putreg(priv, LM3S_SSI_DR_OFFSET, 0xffff);
}

static void ssi_txuint16(struct lm3s_ssidev_s *priv)
{
  uint16 *ptr    = (uint16*)priv->txbuffer;
  ssivdbg("TX: %p->%04x\n", ptr, *ptr);
  ssi_putreg(priv, LM3S_SSI_DR_OFFSET, (uint32)(*ptr++));
  priv->txbuffer = (void*)ptr;
}

static void ssi_txubyte(struct lm3s_ssidev_s *priv)
{
  ubyte *ptr     = (ubyte*)priv->txbuffer;
  ssivdbg("TX: %p->%02x\n", ptr, *ptr);
  ssi_putreg(priv, LM3S_SSI_DR_OFFSET, (uint32)(*ptr++));
  priv->txbuffer = (void*)ptr;
}

/****************************************************************************
 * Name: ssi_rxnull, ssi_rxuint16, and ssi_rxubyte
 *
 * Description:
 *   Discard input, save a ubyte, or or save a uint16 from Tx FIFO in the
 *   user rxvbuffer and update the rxbuffer pointer appropriately.  The
 *   selected function dependes on (1) if there is a desination rxbuffer
 *   provided, and (2) if the number of bits per word is <=8 or >8.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ssi_rxnull(struct lm3s_ssidev_s *priv)
{
#if defined(SSI_DEBUG) && defined(CONFIG_DEBUG_VERBOSE)
  uint32 regval  = ssi_getreg(priv, LM3S_SSI_DR_OFFSET);
  ssivdbg("RX: discard %04x\n", regval);
#else
  (void)ssi_getreg(priv, LM3S_SSI_DR_OFFSET);
#endif
}

static void ssi_rxuint16(struct lm3s_ssidev_s *priv)
{
  uint16 *ptr    = (uint16*)priv->rxbuffer;
  *ptr           = (uint16)ssi_getreg(priv, LM3S_SSI_DR_OFFSET);
  ssivdbg("RX: %p<-%04x\n", ptr, *ptr);
  priv->rxbuffer = (void*)(++ptr);
}

static void ssi_rxubyte(struct lm3s_ssidev_s *priv)
{
  ubyte *ptr     = (ubyte*)priv->rxbuffer;
  *ptr           = (ubyte)ssi_getreg(priv, LM3S_SSI_DR_OFFSET);
  ssivdbg("RX: %p<-%02x\n", ptr, *ptr);
  priv->rxbuffer = (void*)(++ptr);
}

/****************************************************************************
 * Name: ssi_txfifofull
 *
 * Description:
 *   Return TRUE if the Tx FIFO is full
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   TRUE: Not full
 *
 ****************************************************************************/

static inline boolean ssi_txfifofull(struct lm3s_ssidev_s *priv)
{
  return (ssi_getreg(priv, LM3S_SSI_SR_OFFSET) & SSI_SR_TNF) == 0;
}

/****************************************************************************
 * Name: ssi_rxfifoempty
 *
 * Description:
 *   Return TRUE if the Rx FIFO is empty
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   TRUE: Not empty
 *
 ****************************************************************************/

static inline boolean ssi_rxfifoempty(struct lm3s_ssidev_s *priv)
{
  return (ssi_getreg(priv, LM3S_SSI_SR_OFFSET) & SSI_SR_RNE) == 0;
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
static inline int ssi_performtx(struct lm3s_ssidev_s *priv)
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

static int ssi_performtx(struct lm3s_ssidev_s *priv)
{
#ifndef CONFIG_SSI_POLLWAIT
  uint32 regval;
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
          for (; ntxd < priv->ntxwords && ntxd < CONFIG_SSI_TXLIMIT && !ssi_txfifofull(priv); ntxd++)
#else
          for (; ntxd < priv->ntxwords && !ssi_txfifofull(priv); ntxd++)
#endif
            {
               priv->txword(priv);
            }

          /* Update the count of words to to transferred */

          priv->ntxwords -= ntxd;
        }

      /* Check again... Now have all of the Tx words been sent? */

#ifndef CONFIG_SSI_POLLWAIT
      regval = ssi_getreg(priv, LM3S_SSI_IM_OFFSET);
      if (priv->ntxwords > 0)
        {
          /* No.. Enable the Tx FIFO interrupt.  This interrupt occurs
           * when the Tx FIFO is 1/2 full or less.
           */

#ifdef CONFIG_DEBUG
          regval |= (SSI_IM_TX|SSI_RIS_ROR);
#else
          regval |= SSI_IM_TX;
#endif
        }
      else
        {
          /* Yes.. Disable the Tx FIFO interrupt.  The final stages of
           * the transfer will be driven by Rx FIFO interrupts.
           */

          regval &= ~(SSI_IM_TX|SSI_RIS_ROR);
        }
      ssi_putreg(priv, LM3S_SSI_IM_OFFSET, regval);
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

static inline void ssi_performrx(struct lm3s_ssidev_s *priv)
{
#ifndef CONFIG_SSI_POLLWAIT
  uint32 regval;
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
  regval = ssi_getreg(priv, LM3S_SSI_IM_OFFSET);
  if (priv->ntxwords == 0 && priv->nrxwords < priv->nwords)
    {
       /* There are no more outgoing words to send, but there are
        * additional incoming words expected (I would think that this
        * a real corner case, be we will handle it with an extra 
        * interrupt, probably an Rx timeout).
        */

#ifdef CONFIG_DEBUG
      regval |= (SSI_IM_RX|SSI_IM_RT|SSI_IM_ROR);
#else
      regval |= (SSI_IM_RX|SSI_IM_RT);
#endif
    }
  else
    {
      /* No.. there are either more Tx words to send or all Rx words
       * have received.  Disable Rx FIFO interrupts.
       */

      regval &= ~(SSI_IM_RX|SSI_IM_RT);
    }
  ssi_putreg(priv, LM3S_SSI_IM_OFFSET, regval);
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
 *              uses <= 8 bits per word, then this is the number of ubytes;
 *              if the interface uses >8 bits per word, then this is the
 *              number of uint16's
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

static int ssi_transfer(struct lm3s_ssidev_s *priv, const void *txbuffer,
                        void *rxbuffer, unsigned int nwords)
{
#ifndef CONFIG_SSI_POLLWAIT
  irqstate_t flags;
#endif
  int ntxd;

  ssidbg("txbuffer: %p rxbuffer: %p nwords: %d\n", txbuffer, rxbuffer, nwords);
  ssi_semtake(&priv->exclsem);

  /* Set up to perform the transfer */

  priv->txbuffer     = (ubyte*)txbuffer; /* Source buffer */
  priv->rxbuffer     = (ubyte*)rxbuffer; /* Destination buffer */
  priv->ntxwords     = nwords;           /* Number of words left to send */
  priv->nrxwords     = 0;                /* Number of words received */
  priv->nwords       = nwords;           /* Total number of exchanges */

  /* Set up the low-level data transfer function pointers */

  if (priv->nbits > 8)
    {
      priv->txword = ssi_txuint16;
      priv->rxword = ssi_rxuint16;
    }
  else
    {
      priv->txword = ssi_txubyte;
      priv->rxword = ssi_rxubyte;
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
  flags = irqsave();
  ssivdbg("ntxwords: %d nrxwords: %d nwords: %d SR: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, LM3S_SSI_SR_OFFSET));

  ntxd  = ssi_performtx(priv);

  /* For the case where nwords < Tx FIFO size, ssi_performrx will
   * configure interrupts correctly for the final phase of the
   * the transfer.
   */

  ssi_performrx(priv);

  ssivdbg("ntxwords: %d nrxwords: %d nwords: %d SR: %08x IM: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, LM3S_SSI_SR_OFFSET),
          ssi_getreg(priv, LM3S_SSI_IM_OFFSET));

  /* Wait for the transfer to complete.  Since there is no handshake
   * with SPI, the following should complete even if there are problems
   * with the transfer, so it should be safe with no timeout.
   */

  ssivdbg("Waiting for transfer complete\n");
  irqrestore(flags);
  do
    {
      ssi_semtake(&priv->xfrsem);
    }
  while (priv->nrxwords < priv->nwords);
  ssidbg("Transfer complete\n");

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

      /* Handle incoming Rx FIFO transfers */

      ssi_performrx(priv);

      /* If there are other threads at this same priority level,
       * the following may help:
       */

      sched_yield();
    }
  while (priv->nrxwords < priv->nwords);
#endif
  ssi_semgive(&priv->exclsem);
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
static inline struct lm3s_ssidev_s *ssi_mapirq(int irq)
{
  switch (irq)
    {
#ifndef CONFIG_SSI0_DISABLE
      case LM3S_IRQ_SSI0:
        return &g_ssidev[SSI0_NDX];
#endif
#ifndef CONFIG_SSI1_DISABLE
      case LM3S_IRQ_SSI1:
        return &g_ssidev[SSI1_NDX];
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
 *              uses <= 8 bits per word, then this is the number of ubytes;
 *              if the interface uses >8 bits per word, then this is the
 *              number of uint16's
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SSI_POLLWAIT
static int ssi_interrupt(int irq, void *context)
{
  struct lm3s_ssidev_s *priv = ssi_mapirq(irq);
  uint32 regval;
  int ntxd;

  DEBUGASSERT(priv != NULL);

  /* Clear pending interrupts */

  regval = ssi_getreg(priv, LM3S_SSI_RIS_OFFSET);
  ssi_putreg(priv, LM3S_SSI_ICR_OFFSET, regval);

  /* Check for Rx FIFO overruns */

#ifdef CONFIG_DEBUG
  if ((regval & SSI_RIS_ROR) != 0)
    {
      lldbg("Rx FIFO Overrun!\n");
    }
#endif

  ssivdbg("ntxwords: %d nrxwords: %d nwords: %d SR: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, LM3S_SSI_SR_OFFSET));

  /* Handle outgoing Tx FIFO transfers */

  ntxd = ssi_performtx(priv);

  /* Handle incoming Rx FIFO transfers */

  ssi_performrx(priv);

  ssivdbg("ntxwords: %d nrxwords: %d nwords: %d SR: %08x IM: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, LM3S_SSI_SR_OFFSET),
          ssi_getreg(priv, LM3S_SSI_IM_OFFSET));

  /* Check if the transfer is complete */

  if (priv->nrxwords >= priv->nwords)
    {
      /* Yes.. Disable all SSI interrupt sources */

      ssi_putreg(priv, LM3S_SSI_IM_OFFSET, 0);

      /* Wake up the waiting thread */

      ssidbg("Transfer complete\n");
      ssi_semgive(&priv->xfrsem);
    }
  return OK;
}
#endif

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
 ****************************************************************************/

static void ssi_setfrequencyinternal(struct lm3s_ssidev_s *priv, uint32 frequency)
{
  uint32 maxdvsr;
  uint32 cpsdvsr;
  uint32 regval;
  uint32 scr;

  ssidbg("frequency: %d\n", frequency);
  if (priv && frequency != priv->frequency)
    {
      /* "The serial bit rate is derived by dividing down the input clock
       *  (FSysClk). The clock is first divided by an even prescale value
       *  CPSDVSR from 2 to 254, which is programmed in the SSI Clock Prescale
       *  (SSI_CPSR) register ... The clock is further divided by a value
       *  from 1 to 256, which is 1 + SCR, where SCR is the value programmed 
       *  i n the SSI Control0 (SSICR0) register ...
       *
       * "The frequency of the output clock SSIClk is defined by:
       *
       *    "SSIClk = FSysClk / (CPSDVSR * (1 + SCR))
       *
       * "Note: Although the SSIClk transmit clock can theoretically be 25 MHz,
       *  the module may not be able to operate at that speed. For master mode,
       *  the system clock must be at least two times faster than the SSIClk.
       *  For slave mode, the system clock must be at least 12 times faster
       *  than the SSIClk."
       */

      if (frequency > SYSCLK_FREQUENCY/2)
        {
          frequency = SYSCLK_FREQUENCY/2;
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
      ssi_putreg(priv, LM3S_SSI_CPSR_OFFSET, cpsdvsr);

      /* Set SCR */

      regval = ssi_getreg(priv, LM3S_SSI_CR0_OFFSET);
      regval &= ~SSI_CR0_SCR_MASK;
      regval |= (scr << SSI_CR0_SCR_SHIFT);
      ssi_putreg(priv, LM3S_SSI_CR0_OFFSET, regval);
      ssivdbg("CR0: %08x CPSR: %08x\n", regval, cpsdvsr);

      /* Calcluate the actual frequency */

      priv->actual = SYSCLK_FREQUENCY / (cpsdvsr * (scr + 1));
    }
}

static uint32 ssi_setfrequency(FAR struct spi_dev_s *dev, uint32 frequency)
{
  struct lm3s_ssidev_s *priv = (struct lm3s_ssidev_s *)dev;
  uint32 enable;

  /* NOTE that the SSI must be disabled when setting any configuration registers. */

  ssi_semtake(&priv->exclsem);
  enable = ssi_disable(priv);
  ssi_setfrequencyinternal(priv, frequency);
  ssi_enable(priv, enable);
  ssi_semgive(&priv->exclsem);
  return priv->actual;
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
 ****************************************************************************/

static void ssi_setmodeinternal(struct lm3s_ssidev_s *priv, enum spi_mode_e mode)
{
  uint32 modebits;
  uint32 regval;

  ssidbg("mode: %d\n", mode);
  if (priv && mode != priv->mode)
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
          modebits = SSI_CR0_SPH|SSI_CR0_SPO;
          break;

        default:
          return;
        }

      /* Then set the selected mode */

      regval  = ssi_getreg(priv, LM3S_SSI_CR0_OFFSET);
      regval &= ~SSI_CR0_FRF_MASK;
      regval |= modebits;
      ssi_putreg(priv, LM3S_SSI_CR0_OFFSET, regval);
      ssivdbg("CR0: %08x\n", regval);
    }
}

static void ssi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct lm3s_ssidev_s *priv = (struct lm3s_ssidev_s *)dev;
  uint32 enable;

  /* NOTE that the SSI must be disabled when setting any configuration registers. */

  ssi_semtake(&priv->exclsem);
  enable = ssi_disable(priv);
  ssi_setmodeinternal(priv, mode);
  ssi_enable(priv, enable);
  ssi_semgive(&priv->exclsem);
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
 ****************************************************************************/

static void ssi_setbitsinternal(struct lm3s_ssidev_s *priv, int nbits)
{
  uint32 regval;

  ssidbg("nbits: %d\n", nbits);
  if (priv && nbits != priv->nbits && nbits >=4 && nbits <= 16)
    {
      regval  = ssi_getreg(priv, LM3S_SSI_CR0_OFFSET);
      regval &= ~SSI_CR0_DSS_MASK;
      regval |= ((nbits - 1) << SSI_CR0_DSS_SHIFT);
      ssi_putreg(priv, LM3S_SSI_CR0_OFFSET, regval);
      ssivdbg("CR0: %08x\n", regval);

      priv->nbits = nbits;
    }
}

static void ssi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  struct lm3s_ssidev_s *priv = (struct lm3s_ssidev_s *)dev;
  uint32 enable;

  /* NOTE that the SSI must be disabled when setting any configuration registers. */

  ssi_semtake(&priv->exclsem);
  enable = ssi_disable(priv);
  ssi_setbitsinternal(priv, nbits);
  ssi_enable(priv, enable);
  ssi_semgive(&priv->exclsem);
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

static uint16 ssi_send(FAR struct spi_dev_s *dev, uint16 wd)
{
  struct lm3s_ssidev_s *priv = (struct lm3s_ssidev_s*)dev;
  uint16 response = 0;

  (void)ssi_transfer(priv, &wd, &response, 1);
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
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into ubytes; if nbits >8, the data is packed into uint16's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
static void ssi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  struct lm3s_ssidev_s *priv = (struct lm3s_ssidev_s *)dev;
  (void)ssi_transfer(priv, txbuffer, rxbuffer, nwords);
}
#endif

/*************************************************************************
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
 *            packed into ubytes; if nbits >8, the data is packed into uint16's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void ssi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords)
{
  struct lm3s_ssidev_s *priv = (struct lm3s_ssidev_s *)dev;
  (void)ssi_transfer(priv, buffer, NULL, nwords);
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
 *   buffer - A pointer to the buffer in which to recieve data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into ubytes; if nbits >8, the data is packed into uint16's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void ssi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords)
{
  struct lm3s_ssidev_s *priv = (struct lm3s_ssidev_s *)dev;
  (void)ssi_transfer(priv, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize common parts the selected SPI port.  Initialization of
 *   chip select GPIOs must have been performed by board specific logic
 *   prior to calling this function.  Specifically:  GPIOs should have
 *   been configured for output, and all chip selects disabled.
 *
 *   One GPIO, SS (PB2 on the eZ8F091) is reserved as a chip select.  However,
 *   If multiple devices on on the bus, then multiple chip selects will be
 *   required.  Theregore, all GPIO chip management is deferred to board-
 *   specific logic.
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SSI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  struct lm3s_ssidev_s *priv;
  irqstate_t flags;
  ubyte regval;

  ssidbg("port: %d\n", port);
 
  /* Set up for the selected port */

  flags = irqsave();
  switch (port)
    {
#ifndef CONFIG_SSI0_DISABLE
    case 0:
      /* Select SSI0 */

      priv = &g_ssidev[SSI0_NDX];

      /* Enable the SSI0 peripheral */

      regval = getreg32(LM3S_SYSCON_RCGC1);
      regval |= SYSCON_RCGC1_SSI0;
      putreg32(regval, LM3S_SYSCON_RCGC1);
      ssivdbg("RCGC1: %08x\n", regval);

      /* Configure SSI0 GPIOs (NOTE that SS is not initialized here, the
       * logic in this file makes no assumptions about chip select)
       */

      lm3s_configgpio(GPIO_SSI0_CLK);  /* PA2: SSI0 clock (SSI0Clk) */
   /* lm3s_configgpio(GPIO_SSI0_FSS);     PA3: SSI0 frame (SSI0Fss) */
      lm3s_configgpio(GPIO_SSI0_RX);   /* PA4: SSI0 receive (SSI0Rx) */
      lm3s_configgpio(GPIO_SSI0_TX);   /* PA5: SSI0 transmit (SSI0Tx) */
      break;
#endif /* CONFIG_SSI0_DISABLE */

#ifndef CONFIG_SSI1_DISABLE
    case 1:
      /* Select SSI0 */

      priv = &g_ssidev[SSI1_NDX];

      /* Enable the SSI1 peripheral */

      regval = getreg32(LM3S_SYSCON_RCGC1);
      regval |= SYSCON_RCGC1_SSI1;
      putreg32(regval, LM3S_SYSCON_RCGC1);
      ssivdbg("RCGC1: %08x\n", regval);

      /* Configure SSI1 GPIOs */

      lm3s_configgpio(GPIO_SSI1_CLK);  /* PE0: SSI1 clock (SSI1Clk) */
   /* lm3s_configgpio(GPIO_SSI1_FSS);     PE1: SSI1 frame (SSI1Fss) */
      lm3s_configgpio(GPIO_SSI1_RX);   /* PE2: SSI1 receive (SSI1Rx) */
      lm3s_configgpio(GPIO_SSI1_TX);   /* PE3: SSI1 transmit (SSI1Tx) */
      break;
#endif /* CONFIG_SSI1_DISABLE */

    default:
      irqrestore(flags);
      return NULL;
    }

  /* Initialize the state structure */

#ifndef CONFIG_SSI_POLLWAIT
  sem_init(&priv->xfrsem, 0, 0);
#endif
  sem_init(&priv->exclsem, 0, 1);

  /* Set all CR1 fields to reset state.  This will be master mode. */

  ssi_putreg(priv, LM3S_SSI_CR1_OFFSET, 0);

  /* Set all CR0 fields to the reset state. This will also select Freescale SPI mode. */

  ssi_putreg(priv, LM3S_SSI_CR0_OFFSET, 0);

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

  ssi_putreg(priv, LM3S_SSI_IM_OFFSET, 0);

  /* Attach the interrupt */

#ifndef CONFIG_SSI_POLLWAIT
#if NSSI_ENABLED > 1
  irq_attach(priv->irq, (xcpt_t)ssi_interrupt);
#else
  irq_attach(SSI_IRQ, (xcpt_t)ssi_interrupt);
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

  irqrestore(flags);
  return (FAR struct spi_dev_s *)priv;
}

#endif /* NSSI_ENABLED > 0 */

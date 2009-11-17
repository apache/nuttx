/****************************************************************************
 * arch/arm/src/stm32/stm32_sdio.c
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
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "chip.h"
#include "up_arch.h"
#include "stm32_internal.h"
#include "stm32_dma.h"
#include "stm32_sdio.h"

#if CONFIG_STM32_SDIO

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if defined(CONFIG_SDIO_DMA) && !defined(CONFIG_STM32_DMA2)
#  warning "CONFIG_SDIO_DMA support requires CONFIG_STM32_DMA2"
#  undef CONFIG_SDIO_DMA
#endif

#ifndef CONFIG_SDIO_PRI
#  define CONFIG_SDIO_PRI        DMA_CCR_PRIMED
#endif

#ifndef CONFIG_SDIO_DMAPRIO
#  define CONFIG_SDIO_DMAPRIO    DMA_CCR_PRIMED
#endif

/* Friendly CLKCR bit re-definitions ****************************************/

#define SDIO_CLKCR_RISINGEDGE    (0)
#define SDIO_CLKCR_FALLINGEDGE   SDIO_CLKCR_NEGEDGE

/* HCLK=72MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(178+2)=400 KHz */
  
#define SDIO_INIT_CLKDIV         (178 << SDIO_CLKCR_CLKDIV_SHIFT)
#define STM32_CLCKCR_INIT \
  (SDIO_INIT_CLKDIV|SDIO_CLKCR_RISINGEDGE|SDIO_CLKCR_WIDBUS_D1)

/* HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(1+2)=24 MHz */

#define SDIO_TRANSFER_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT) 
#define STM32_CLCKCR_TRANSFER    (SDIO_TRANSFER_CLKDIV|SDIO_CLKCR_RISINGEDGE|\
                                  SDIO_CLKCR_WIDBUS_D1)
#define STM32_CLKCR_WIDETRANSFER (SDIO_TRANSFER_CLKDIV|SDIO_CLKCR_RISINGEDGE|\
                                  SDIO_CLKCR_WIDBUS_D4)

/* Timing */

#define SDIO_CMDTIMEOUT          (100000)
#define SDIO_LONGTIMEOUT         (0x7fffffff)

/* Big DTIMER setting */

#define SDIO_DTIMER_DATATIMEOUT  (0x000fffff)

/* DMA CCR register settings */

#define SDIO_RXDMA16_CONFIG      (CONFIG_SDIO_DMAPRIO|DMA_CCR_MSIZE_16BITS|\
                                  DMA_CCR_PSIZE_16BITS|DMA_CCR_MINC)
#define SDIO_TXDMA16_CONFIG      (CONFIG_SDIO_DMAPRIO|DMA_CCR_MSIZE_16BITS|\
                                  DMA_CCR_PSIZE_16BITS|DMA_CCR_MINC|DMA_CCR_DIR)

/* FIFO sizes */

#define SDIO_HALFFIFO_WORDS      (8)
#define SDIO_HALFFIFO_BYTES      (8*4)

/* Data transfer interrupt mask bits */

#define SDIO_RECV_MASK     (SDIO_MASK_DCRCFAILIE|SDIO_MASK_DTIMEOUTIE|\
                            SDIO_MASK_DATAENDIE|SDIO_MASK_RXOVERRIE|\
                            SDIO_MASK_RXFIFOHFIE|SDIO_MASK_STBITERRIE)
#define SDIO_SEND_MASK     (SDIO_MASK_DCRCFAILIE|SDIO_MASK_DTIMEOUTIE|\
                            SDIO_MASK_DATAENDIE|SDIO_MASK_TXUNDERRIE|\
                            SDIO_MASK_TXFIFOHEIE|SDIO_MASK_STBITERRIE)
#define SDIO_DMARECV_MASK  (SDIO_MASK_DCRCFAILIE|SDIO_MASK_DTIMEOUTIE|\
                            SDIO_MASK_DATAENDIE|SDIO_MASK_RXOVERRIE|\
                            SDIO_MASK_STBITERRIE)
#define SDIO_DMASEND_MASK  (SDIO_MASK_DCRCFAILIE|SDIO_MASK_DTIMEOUTIE|\
                            SDIO_MASK_DATAENDIE|SDIO_MASK_TXUNDERRIE|\
                            SDIO_MASK_STBITERRIE)

/* Event waiting interrupt mask bits */

#define SDIO_CMDDONE_STA   (SDIO_STA_CMDSENT)
#define SDIO_CRCRESP_STA   (SDIO_STA_CTIMEOUT|SDIO_STA_CCRCFAIL|SDIO_STA_CMDREND)
#define SDIO_RESPDONE_STA  (SDIO_STA_CTIMEOUT|SDIO_STA_CMDREND)
#define SDIO_XFRDONE_STA   (0)

#define SDIO_CMDDONE_MASK  (SDIO_MASK_CMDSENTIE)
#define SDIO_CRCRESP_MASK  (SDIO_MASK_CCRCFAILIE|SDIO_MASK_CTIMEOUTIE|\
                            SDIO_MASK_CMDRENDIE)
#define SDIO_RESPDONE_MASK (SDIO_MASK_CTIMEOUTIE|SDIO_MASK_CMDRENDIE)
#define SDIO_XFRDONE_MASK  (0)

#define SDIO_CMDDONE_ICR   (SDIO_ICR_CMDSENTC)
#define SDIO_CRCRESP_ICR   (SDIO_ICR_CTIMEOUTC|SDIO_ICR_CCRCFAILC|SDIO_ICR_CMDRENDC)
#define SDIO_RESPDONE_ICR  (SDIO_ICR_CTIMEOUTC|SDIO_ICR_CMDRENDC)
#define SDIO_XFRDONE_ICR   (0)

#define SDIO_WAITALL_ICR   (SDIO_ICR_CMDSENTC|SDIO_ICR_CTIMEOUTC|\
                            SDIO_ICR_CCRCFAILC|SDIO_ICR_CMDRENDC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the STM32 SDIO interface */

struct stm32_dev_s
{
  struct sdio_dev_s  dev;        /* Standard, base SDIO interface */
  
  /* STM32-specific extensions */

  sem_t              waitsem;    /* Implements event waiting */
  sdio_eventset_t    waitevents; /* Set of events to be waited for */
  uint32             waitmask;   /* Interrupt enables for event waiting */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  sdio_eventset_t    cbevents;   /* Set of events to be cause callbacks */
  sdio_mediachange_t callback;   /* Registered callback function */
  void              *cbarg;      /* Registered callback argument */

  /* Interrupt mode data transfer support */

  uint32            *buffer;     /* Address of current R/W buffer */
  size_t             remaining;  /* Number of bytes remaining in the transfer */
  int                result;     /* Result of the transfer */
  uint32             xfrmask;    /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  boolean            widebus;    /* Required for DMA support */
#ifdef CONFIG_SDIO_DMA
  boolean            dmamode;    /* TRUE: DMA mode transfer */
  DMA_HANDLE         dma;        /* Handle for DMA channel */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static void   stm32_takesem(struct stm32_dev_s *priv);
#define       stm32_givesem(priv) (sem_post(&priv->waitsem))
static inline void stm32_setclkcr(uint32 clkcr);
static void   stm32_configwaitints(struct stm32_dev_s *priv, uint32 waitmask,
                ubyte waitevents, ubyte wkupevents);
static void   stm32_configxfrints(struct stm32_dev_s *priv, uint32 xfrmask);
static void   stm32_setpwrctrl(uint32 pwrctrl);
static inline uint32 stm32_getpwrctrl(void);
static inline void stm32_clkenable(void);
static inline void stm32_clkdisable(void);

/* DMA Helpers **************************************************************/

#ifdef CONFIG_SDIO_DMA
static void  stm32_dmacallback(DMA_HANDLE handle, ubyte isr, void *arg);
#endif

/* Data Transfer Helpers ****************************************************/

static ubyte stm32_log2(uint16 value);
static void  stm32_dataconfig(uint32 timeout, uint32 dlen, uint32 dctrl);
static void  stm32_datadisable(void);
static void  stm32_sendfifo(struct stm32_dev_s *priv);
static void  stm32_recvfifo(struct stm32_dev_s *priv);
static void  stm32_endwait(struct stm32_dev_s *priv, uint32 eventset);
static void  stm32_endtransfer(struct stm32_dev_s *priv, int result);

/* Interrupt Handling *******************************************************/

static int    stm32_interrupt(int irq, void *context);

/* SDIO interface methods ***************************************************/

/* Initialization/setup */

static void  stm32_reset(FAR struct sdio_dev_s *dev);
static ubyte stm32_status(FAR struct sdio_dev_s *dev);
static void  stm32_widebus(FAR struct sdio_dev_s *dev, boolean enable);
static void  stm32_clock(FAR struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int   stm32_attach(FAR struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static void  stm32_sendcmd(FAR struct sdio_dev_s *dev, uint32 cmd,
               uint32 arg);
static int   stm32_recvsetup(FAR struct sdio_dev_s *dev, FAR ubyte *buffer,
               size_t nbytes);
static int   stm32_sendsetup(FAR struct sdio_dev_s *dev,
               FAR const ubyte *buffer, uint32 nbytes);

static int   stm32_waitresponse(FAR struct sdio_dev_s *dev, uint32 cmd);
static int   stm32_recvshortcrc(FAR struct sdio_dev_s *dev, uint32 cmd,
               uint32 *rshort);
static int   stm32_recvlong(FAR struct sdio_dev_s *dev, uint32 cmd,
               uint32 rlong[4]);
static int   stm32_recvshort(FAR struct sdio_dev_s *dev, uint32 cmd,
               uint32 *rshort);
static int   stm32_recvnotimpl(FAR struct sdio_dev_s *dev, uint32 cmd,
               uint32 *rnotimpl);

/* EVENT handler */

static void  stm32_waitenable(FAR struct sdio_dev_s *dev,
               sdio_eventset_t eventset);
static ubyte stm32_eventwait(FAR struct sdio_dev_s *dev, uint32 timeout);
static ubyte stm32_events(FAR struct sdio_dev_s *dev);
static void  stm32_callbackenable(FAR struct sdio_dev_s *dev,
               sdio_eventset_t eventset);
static int   stm32_registercallback(FAR struct sdio_dev_s *dev,
               sdio_mediachange_t callback, void *arg);

/* DMA */

#ifdef CONFIG_SDIO_DMA
static boolean stm32_dmasupported(FAR struct sdio_dev_s *dev);
static int   stm32_dmarecvsetup(FAR struct sdio_dev_s *dev,
               FAR ubyte *buffer, size_t buflen);
static int   stm32_dmasendsetup(FAR struct sdio_dev_s *dev,
               FAR const ubyte *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void  stm32_callback(void *arg);
static void  stm32_default(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct stm32_dev_s g_sdiodev =
{
  .dev =
  {
    .reset            = stm32_reset,
    .status           = stm32_status,
    .widebus          = stm32_widebus,
    .clock            = stm32_clock,
    .attach           = stm32_attach,
    .sendcmd          = stm32_sendcmd,
    .recvsetup        = stm32_recvsetup,
    .sendsetup        = stm32_sendsetup,
    .waitresponse     = stm32_waitresponse,
    .recvR1           = stm32_recvshortcrc,
    .recvR2           = stm32_recvlong,
    .recvR3           = stm32_recvshort,
    .recvR4           = stm32_recvnotimpl,
    .recvR5           = stm32_recvnotimpl,
    .recvR6           = stm32_recvshortcrc,
    .recvR7           = stm32_recvshort,
    .waitenable       = stm32_waitenable,
    .eventwait        = stm32_eventwait,
    .events           = stm32_events,
    .callbackenable   = stm32_callbackenable,
    .registercallback = stm32_registercallback,
#ifdef CONFIG_SDIO_DMA
    .dmasupported     = stm32_dmasupported,
    .dmarecvsetup     = stm32_dmarecvsetup,
    .dmasendsetup     = stm32_dmasendsetup,
#endif
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Low-level Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wakeups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   dev - Instance of the SDIO device driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_takesem(struct stm32_dev_s *priv)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&priv->waitsem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: stm32_setclkcr
 *
 * Description:
 *   Modify oft-changed bits in the CLKCR register.  Only the following bit-
 *   fields are changed:
 *
 *   CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, and HWFC_EN
 *
 * Input Parameters:
 *   clkcr - A new CLKCR setting for the above mentions bits (other bits
 *           are ignored.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_setclkcr(uint32 clkcr)
{
  uint32 regval = getreg32(STM32_SDIO_CLKCR);
    
  /* Clear CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, HWFC_EN bits */

  regval &= ~(SDIO_CLKCR_CLKDIV_MASK|SDIO_CLKCR_PWRSAV|SDIO_CLKCR_BYPASS|
              SDIO_CLKCR_WIDBUS_MASK|SDIO_CLKCR_NEGEDGE|SDIO_CLKCR_HWFC_EN);

  /* Replace with user provided settings */

  clkcr  &=  (SDIO_CLKCR_CLKDIV_MASK|SDIO_CLKCR_PWRSAV|SDIO_CLKCR_BYPASS|
              SDIO_CLKCR_WIDBUS_MASK|SDIO_CLKCR_NEGEDGE|SDIO_CLKCR_HWFC_EN);
  regval |=  clkcr;
  putreg32(regval, STM32_SDIO_CLKCR);
}

/****************************************************************************
 * Name: stm32_configwaitints
 *
 * Description:
 *   Enable/disable SDIO interrupts needed to suport the wait function
 *
 * Input Parameters:
 *   priv      - A reference to the SDIO device state structure
 *   waitmask  - The set of bits in the SDIO MASK register to set
 *   waitevent - Waited for events
 *   wkupevent - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_configwaitints(struct stm32_dev_s *priv, uint32 waitmask,
                                 ubyte waitevents, ubyte wkupevent)
{
  irqstate_t flags;
  flags = irqsave();
  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = waitmask;
  putreg32(priv->xfrmask | priv->waitmask, STM32_SDIO_MASK);
  irqrestore(flags);
}

/****************************************************************************
 * Name: stm32_configxfrints
 *
 * Description:
 *   Enable SDIO interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the SDIO device state structure
 *   xfrmask - The set of bits in the SDIO MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_configxfrints(struct stm32_dev_s *priv, uint32 xfrmask)
{
  irqstate_t flags;
  flags = irqsave();
  priv->xfrmask = xfrmask;
  putreg32(priv->xfrmask | priv->waitmask, STM32_SDIO_MASK);
  irqrestore(flags);
}

/****************************************************************************
 * Name: stm32_setpwrctrl
 *
 * Description:
 *   Change the PWRCTRL field of the SDIO POWER register to turn the SDIO
 *   ON or OFF
 *
 * Input Parameters:
 *   clkcr - A new PWRCTRL setting
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_setpwrctrl(uint32 pwrctrl)
{
  uint32 regval;

  regval  = getreg32(STM32_SDIO_POWER);
  regval &= ~SDIO_POWER_PWRCTRL_MASK;
  regval |= pwrctrl;
  putreg32(regval, STM32_SDIO_POWER);
}

/****************************************************************************
 * Name: stm32_getpwrctrl
 *
 * Description:
 *   Return the current value of the  the PWRCTRL field of the SDIO POWER
 *   register.  This function can be used to see the the SDIO is power ON
 *   or OFF
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current value of the  the PWRCTRL field of the SDIO POWER register.
 *
 ****************************************************************************/

static inline uint32 stm32_getpwrctrl(void)
{
  return getreg32(STM32_SDIO_POWER) & SDIO_POWER_PWRCTRL_MASK;
}

static inline void stm32_clkenable(void)
{
  putreg32(1, SDIO_CLKCR_CLKEN_BB);
}

static inline void stm32_clkdisable(void)
{
  putreg32(0, SDIO_CLKCR_CLKEN_BB);
}

/****************************************************************************
 * DMA Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dmacallback
 *
 * Description:
 *   Called when SDIO DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static void stm32_dmacallback(DMA_HANDLE handle, ubyte isr, void *arg)
{
  /* FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)arg; */

  /* We don't really do anything at the completion of DMA.  The termination
   * of the transfer is driven by the SDIO interrupts.
   */
}
#endif

/****************************************************************************
 * Data Transfer Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_log2
 *
 * Description:
 *   Take (approximate) log base 2 of the provided number (Only works if the
 *   provided number is a power of 2).
 *
 ****************************************************************************/

static ubyte stm32_log2(uint16 value)
{
  ubyte log2 = 0;

  /* 0000 0000 0000 0001 -> return 0,
   * 0000 0000 0000 001x -> return 1,
   * 0000 0000 0000 01xx -> return 2,
   * 0000 0000 0000 1xxx -> return 3,
   * ...
   * 1xxx xxxx xxxx xxxx -> return 15,
   */

  DEBUGASSERT(value > 0);
  while (value != 1)
  {
    value >>= 1;
    log2++;
  }
  return log2;
}

/****************************************************************************
 * Name: stm32_dataconfig
 *
 * Description:
 *   Configure the SDIO data path for the next data transfer
 *
 ****************************************************************************/

static void stm32_dataconfig(uint32 timeout, uint32 dlen, uint32 dctrl)
{
  uint32 regval = 0;

  /* Enable data path */

  putreg32(timeout, STM32_SDIO_DTIMER); /* Set DTIMER */
  putreg32(dlen,    STM32_SDIO_DLEN);   /* Set DLEN */

  /* Configure DCTRL DTDIR, DTMODE, and DBLOCKSIZE fields and set the DTEN
   * field
   */

  regval  =  getreg32(STM32_SDIO_DCTRL);
  regval &= ~(SDIO_DCTRL_DTDIR|SDIO_DCTRL_DTMODE|SDIO_DCTRL_DBLOCKSIZE_MASK);
  dctrl  &=  (SDIO_DCTRL_DTDIR|SDIO_DCTRL_DTMODE|SDIO_DCTRL_DBLOCKSIZE_MASK);
  regval |=  (dctrl|SDIO_DCTRL_DTEN);
  putreg32(regval, STM32_SDIO_DCTRL);
}

/****************************************************************************
 * Name: stm32_datadisable
 *
 * Description:
 *   Disable the the SDIO data path setup by stm32_dataconfig() and
 *   disable DMA.
 *
 ****************************************************************************/

static void stm32_datadisable(void)
{
  uint32 regval;

  /* Disable the data path */

  putreg32(SDIO_DTIMER_DATATIMEOUT, STM32_SDIO_DTIMER); /* Reset DTIMER */
  putreg32(0,              STM32_SDIO_DLEN);   /* Reset DLEN */

  /* Reset DCTRL DTEN, DTDIR, DTMODE, DMAEN, and DBLOCKSIZE fields */

  regval  = getreg32(STM32_SDIO_DCTRL);
  regval &= ~(SDIO_DCTRL_DTEN|SDIO_DCTRL_DTDIR|SDIO_DCTRL_DTMODE|
              SDIO_DCTRL_DMAEN|SDIO_DCTRL_DBLOCKSIZE_MASK);
  putreg32(regval, STM32_SDIO_DCTRL);
}

/****************************************************************************
 * Name: stm32_sendfifo
 *
 * Description:
 *   Send SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_sendfifo(struct stm32_dev_s *priv)
{
  union
  {
    uint32 w;
    ubyte  b[2];
  } data;

  /* Loop while there is more data to be sent and the RX FIFO is not full */

  while (priv->remaining > 0 &&
         (getreg32(STM32_SDIO_STA) & SDIO_STA_TXFIFOF) == 0)
    {
      /* Is there a full word remaining in the user buffer? */

      if (priv->remaining >= sizeof(uint32))
        {
          /* Yes, transfer the word to the TX FIFO */

          data.w           = *priv->buffer++;
          priv->remaining -= sizeof(uint32);
        }
      else
        {
           /* No.. transfer just the bytes remaining in the user buffer,
            * padding with zero as necessary to extend to a full word.
            */

           ubyte *ptr = (ubyte *)priv->remaining;
           int i;

           data.w = 0;
           for (i = 0; i < priv->remaining; i++)
             {
                data.b[i] = *ptr++;
             }
 
           /* Now the transfer is finished */

           priv->remaining = 0;
         }

       /* Put the word in the FIFO */

       putreg32(data.w, STM32_SDIO_FIFO);
    }
}

/****************************************************************************
 * Name: stm32_recvfifo
 *
 * Description:
 *   Receive SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_recvfifo(struct stm32_dev_s *priv)
{
  union
  {
    uint32 w;
    ubyte  b[2];
  } data;

  /* Loop while there is space to store the data and there is more
   * data available in the RX FIFO.
   */

  while (priv->remaining > 0 &&
         (getreg32(STM32_SDIO_STA) & SDIO_STA_RXDAVL) == 0)
    {
      /* Read the next word from the RX FIFO */

      data.w = getreg32(STM32_SDIO_FIFO);
      if (priv->remaining >= sizeof(uint32))
        {
          /* Transfer the whole word to the user buffer */

          *priv->buffer++  = data.w;
          priv->remaining -= sizeof(uint32);
        }
      else
        {
          /* Transfer any trailing fractional word */

          ubyte *ptr = (ubyte*)priv->buffer;
          int i;

          for (i = 0; i < priv->remaining; i++)
            {
               *ptr++ = data.b[i];
            }

          /* Now the transfer is finished */

          priv->remaining = 0;
        }
    }
}

/****************************************************************************
 * Name: stm32_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv   - An instance of the SDIO device interface
 *   result - The result status of the transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_endwait(struct stm32_dev_s *priv, uint32 eventset)
{
  /* Yes.. Disable event-related interrupts */

  stm32_configwaitints(priv, 0, 0, eventset);

  /* Wake up the waiting thread */

  stm32_givesem(priv);
}

/****************************************************************************
 * Name: stm32_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status
 *
 * Input Parameters:
 *   priv   - An instance of the SDIO device interface
 *   result - The result status of the transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_endtransfer(struct stm32_dev_s *priv, int result)
{
  /* Disable all transfer related interrupts */

  stm32_configxfrints(priv, 0);

  /* Mark the transfer finished with the provided status */

  priv->remaining = 0;
  priv->result    = result;

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TRANSFERDONE) != 0)
    {
      /* Yes.. wake up any waiting threads */

      stm32_endwait(priv, SDIOWAIT_TRANSFERDONE);
    }
}

/****************************************************************************
 * Interrrupt Handling
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_interrupt
 *
 * Description:
 *   SDIO interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int stm32_interrupt(int irq, void *context)
{
  struct stm32_dev_s *priv = &g_sdiodev;
  uint32 enabled;
  uint32 pending;

  /* Loop while there are pending interrupts.  Check the SDIO status
   * register.  Mask out all bits that don't correspond to enabled
   * interrupts.  (This depends on the fact that bits are ordered
   * the same in both the STA and MASK register).  If there are non-zero
   * bits remaining, then we have work to do here.
   */

  while ((enabled = getreg32(STM32_SDIO_STA) & getreg32(STM32_SDIO_MASK)) != 0)
    {
      /* Handle in progress, interrupt driven data transfers ****************/

      pending  = enabled & priv->xfrmask;
      if (pending != 0)
        {
#ifdef CONFIG_SDIO_DMA
          if (!priv->dmamode)
#endif
           {
             /* Is the RX FIFO half full or more?  Is so then we must be
              * processing a receive transaction.
             */

             if ((pending & SDIO_STA_RXFIFOHF) != 0)
               {
                 /* Receive data from the RX FIFO */

                 stm32_recvfifo(priv);
               }

             /* Otherwise, Is the transmit FIFO half empty or less?  If so we must
              * be processing a send transaction.  NOTE:  We can't be processing
              * both!
              */

             else if ((pending & SDIO_STA_TXFIFOHE) != 0)
               {
                 /* Send data via the TX FIFO */

                 stm32_sendfifo(priv);
               }
           }

          /* Handle data end events */

          if ((pending & SDIO_STA_DATAEND) != 0)
            {
              /* Handle any data remaining the RX FIFO.  If the RX FIFO is
               * less than half full at the end of the transfer, then no
               * half-full interrupt will be received.
               */

#ifdef CONFIG_SDIO_DMA
              if (!priv->dmamode)
#endif
                {
                  /* Receive data from the RX FIFO */

                  stm32_recvfifo(priv);
                }

              /* Then terminate the transfer */

              putreg32(SDIO_ICR_DATAENDC, STM32_SDIO_ICR);
              stm32_endtransfer(priv, OK);
            }

          /* Handler data block send/receive CRC failure */

          else if ((pending & SDIO_STA_DCRCFAIL) != 0)
            {
              /* Terminate the transfer with an error */

              putreg32(SDIO_ICR_DCRCFAILC, STM32_SDIO_ICR);
              stm32_endtransfer(priv, -EIO);
            }

          /* Handle data timeout error */

          else if ((pending & SDIO_STA_DTIMEOUT) != 0)
            {
              /* Terminate the transfer with an error */

              putreg32(SDIO_ICR_DTIMEOUTC, STM32_SDIO_ICR);
              stm32_endtransfer(priv, -ETIMEDOUT);
            }

          /* Handle RX FIFO overrun error */

          else if ((pending & SDIO_STA_RXOVERR) != 0)
            {
              /* Terminate the transfer with an error */

              putreg32(SDIO_ICR_RXOVERRC, STM32_SDIO_ICR);
              stm32_endtransfer(priv, -EOVERFLOW);
            }

          /* Handle TX FIFO underrun error */

          else if ((pending & SDIO_STA_TXUNDERR) != 0)
            {
              /* Terminate the transfer with an error */

              putreg32(SDIO_ICR_TXUNDERRC, STM32_SDIO_ICR);
              stm32_endtransfer(priv, -EOVERFLOW);
            }

          /* Handle start bit error */

          else if ((pending & SDIO_STA_STBITERR) != 0)
            {
              /* Terminate the transfer with an error */

              putreg32(SDIO_ICR_STBITERRC, STM32_SDIO_ICR);
              stm32_endtransfer(priv, -EIO);
            }
        }

      /* Handle wait events *************************************************/

      pending  = enabled & priv->waitmask;
      if (pending != 0)
        {
          /* Is this a response completion event? */

          if ((pending & SDIO_CRCRESP_STA) != 0)
            {
              /* Yes.. Is their a thread waiting for response done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  putreg32(SDIO_CRCRESP_ICR, STM32_SDIO_ICR);
                  stm32_endwait(priv, SDIOWAIT_RESPONSEDONE);
                }
            }

          /* Is this a command completion event? */

          if ((pending & SDIO_CMDDONE_STA) != 0)
            {
              /* Yes.. Is their a thread waiting for command done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  putreg32(SDIO_CMDDONE_ICR, STM32_SDIO_ICR);
                  stm32_endwait(priv, SDIOWAIT_CMDDONE);
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * SDIO Interface Methods
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_reset
 *
 * Description:
 *   Reset the SDIO controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_reset(FAR struct sdio_dev_s *dev)
{
}

/****************************************************************************
 * Name: stm32_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see stm32_status_* defines)
 *
 ****************************************************************************/

static ubyte stm32_status(FAR struct sdio_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: SDIO_WIDEBUS
 *
 * Description:
 *   Called after change in Bus width has been selected (via ACMD6).  Most
 *   controllers will need to perform some special operations to work
 *   correctly in the new bus mode.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   wide - TRUE: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_widebus(FAR struct sdio_dev_s *dev, boolean wide)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  priv->widebus = wide;
}

/****************************************************************************
 * Name: stm32_clock
 *
 * Description:
 *   Enable/disable SDIO clocking
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
}

/****************************************************************************
 * Name: stm32_attach
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

static int stm32_attach(FAR struct sdio_dev_s *dev)
{
  int ret;

  /* Attach the SDIO interrupt handler */

  ret = irq_attach(STM32_IRQ_SDIO, stm32_interrupt);
  if (ret == OK)
    {

      /* Disable all interrupts at the SDIO controller and clear static
       * interrupt flags
       */

      putreg32(SDIO_MASK_RESET,      STM32_SDIO_MASK);
      putreg32(SDIO_ICR_STATICFLAGS, STM32_SDIO_ICR);

      /* Enable SDIO interrupts at the NVIC.  They can now be enabled at
       * the SDIO controller as needed.
       */

      up_enable_irq(STM32_IRQ_SDIO);

      /* Set the interrrupt priority */

      up_prioritize_irq(STM32_IRQ_SDIO, CONFIG_SDIO_PRI);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_sendcmd
 *
 * Description:
 *   Send the SDIO command
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command to send (32-bits, encoded)
 *   arg  - 32-bit argument required with some commands
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_sendcmd(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 arg)
{
  uint32 regval;
  uint32 cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;

  /* Set the SDIO Argument value */

  putreg32(arg, STM32_SDIO_ARG);

  /* Clear CMDINDEX, WAITRESP, WAITINT, WAITPEND, and CPSMEN bits */

  regval = getreg32(STM32_SDIO_CMD);
  regval &= ~(SDIO_CMD_CMDINDEX_MASK|SDIO_CMD_WAITRESP_MASK|
              SDIO_CMD_WAITINT|SDIO_CMD_WAITPEND|SDIO_CMD_CPSMEN);

  /* Set WAITRESP bits */

  switch ((cmd & MMCSD_RESPONSE_MASK) >> MMCSD_RESPONSE_SHIFT)
    {
    case MMCSD_NO_RESPONSE:
      regval |= SDIO_CMD_NORESPONSE;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= SDIO_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R2_RESPONSE:
      regval |= SDIO_CMD_LONGRESPONSE;
      break;
    }

  /* Set CPSMEN and the command index */

  cmdidx  = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= cmdidx | SDIO_CMD_CPSMEN;
  
  /* Write the SDIO CMD */

  putreg32(regval, STM32_SDIO_CMD);
}

/****************************************************************************
 * Name: stm32_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data trasfer from the card in non-DMA
 *   (interrupt driven mode).  This method will do whatever controller setup
 *   is necessary.  This would be called for SD memory just BEFORE sending
 *   CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally, SDIO_WAITEVENT
 *   will be called to receive the indication that the transfer is complete.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer in which to receive the data
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int stm32_recvsetup(FAR struct sdio_dev_s *dev, FAR ubyte *buffer,
                           size_t nbytes)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32 dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  stm32_datadisable();

  /* Save the destination buffer information for use by the interrupt handler */

  priv->buffer    = (uint32*)buffer;
  priv->remaining = nbytes;
  priv->result    = -EBUSY;
#ifdef CONFIG_SDIO_DMA
  priv->dmamode   = FALSE;
#endif

  /* Then set up the SDIO data path */

  dblocksize = stm32_log2(nbytes) << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
  stm32_dataconfig(SDIO_DTIMER_DATATIMEOUT, nbytes, dblocksize|SDIO_DCTRL_DTDIR);

  /* And enable interrupts */

  stm32_configxfrints(priv, SDIO_RECV_MASK);
  return OK;
}

/****************************************************************************
 * Name: stm32_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data trasfer from the card.  This method
 *   will do whatever controller setup is necessary.  This would be called
 *   for SD memory just AFTER sending CMD24 (WRITE_BLOCK), CMD25
 *   (WRITE_MULTIPLE_BLOCK), ... and before SDIO_SENDDATA is called.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer containing the data to send
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int stm32_sendsetup(FAR struct sdio_dev_s *dev, FAR const ubyte *buffer,
                           size_t nbytes)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32 dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  stm32_datadisable();

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32*)buffer;
  priv->remaining = nbytes;
  priv->result    = -EBUSY;
#ifdef CONFIG_SDIO_DMA
  priv->dmamode   = FALSE;
#endif

  /* Then set up the SDIO data path */

  dblocksize = stm32_log2(nbytes) << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
  stm32_dataconfig(SDIO_DTIMER_DATATIMEOUT, nbytes, dblocksize);

  /* Enable TX interrrupts */

  stm32_configxfrints(priv, SDIO_SEND_MASK);
  return OK;
}

/****************************************************************************
 * Name: stm32_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int stm32_waitresponse(FAR struct sdio_dev_s *dev, uint32 cmd)
{
  sint32 timeout;
  uint32 events;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      events  = SDIO_CMDDONE_STA;
      timeout = SDIO_CMDTIMEOUT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R6_RESPONSE:
      events  = SDIO_CRCRESP_STA;
      timeout = SDIO_LONGTIMEOUT;
      break;

    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
      return -ENOSYS;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      events  = SDIO_RESPDONE_STA;
      timeout = SDIO_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the response (or timeout) */

  while ((getreg32(STM32_SDIO_STA) & events) == 0)
    {
      if (--timeout <= 0)
        {
          fdbg("ERROR: Timeout cmd=%04x\n", cmd);
          return -ETIMEDOUT;
        }
    }

  /* Clear all the static flags */

  putreg32(SDIO_ICR_STATICFLAGS, STM32_SDIO_ICR);
  return OK;
}

/****************************************************************************
 * Name: stm32_recvRx
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   Rx - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a faiure to obtain the requested reponse (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intacta and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int stm32_recvshortcrc(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 *rshort)
{
#ifdef CONFIG_DEBUG
  uint32 respcmd;
#endif
  uint32 regval;

  /* R1  Command response (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Command index (0-63)
   *     39:8      bit31  - bit0   32-bit card status
   *     7:1       bit6   - bit0   CRC7
   *     0         1               End bit
   *
   * R1b Identical to R1 with the additional busy signaling via the data
   *     line.
   *
   * R6  Published RCA Response (48-bit, SD card only)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Command index (0-63)
   *     39:8      bit31  - bit0   32-bit Argument Field, consisting of:
   *                               [31:16] New published RCA of card
   *                               [15:0]  Card status bits {23,22,19,12:0}
   *     7:1       bit6   - bit0   CRC7
   *     0         1               End bit
   */


#ifdef CONFIG_DEBUG
  if (!rshort)
    {
      fdbg("ERROR: rshort=NULL\n");
      return -EINVAL;
    }

  /* Check that this is the correct response to this command */

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1B_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK)  != MMCSD_R6_RESPONSE)
    {
      fdbg("ERROR: Wrong response CMD=%08x\n", cmd);
      return -EINVAL;
    }
#endif

  /* Verify that the response is available */
 
  regval = getreg32(STM32_SDIO_STA);
  if ((regval & SDIO_STA_CTIMEOUT) != 0)
    {
      fdbg("ERROR: Command timeout: %08x\n", regval);
      putreg32(SDIO_ICR_CTIMEOUTC, STM32_SDIO_ICR);
      return -ETIMEDOUT;
    }
  else if ((regval & SDIO_STA_CCRCFAIL) != 0)
    {
      fdbg("ERROR: CRC failuret: %08x\n", regval);
      putreg32(SDIO_ICR_CCRCFAILC, STM32_SDIO_ICR);
      return -EIO;
    }
  else if ((regval & SDIO_STA_CMDREND) == 0)
    {
      fdbg("ERROR: Status is not yet available: %08x\n", regval);
      return -EBUSY;
    }

  /* Check response received is of desired command */

#ifdef CONFIG_DEBUG
  respcmd = getreg32(STM32_SDIO_RESPCMD);
  if ((ubyte)(respcmd & SDIO_RESPCMD_MASK) != (cmd & MMCSD_CMDIDX_MASK))
    {
      fdbg("ERROR: RESCMD=%02x CMD=%08x\n", respcmd, cmd);
      return -EINVAL;
    }
#endif

  /* Return the R1 response */

  putreg32(SDIO_ICR_STATICFLAGS, STM32_SDIO_ICR);
  *rshort = getreg32(STM32_SDIO_RESP1);
  return OK;
}

static int stm32_recvlong(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 rlong[4])
{
  uint32 regval;

 /* R2  CID, CSD register (136-bit)
  *     135       0               Start bit
  *     134       0               Transmission bit (0=from card)
  *     133:128   bit5   - bit0   Reserved
  *     127:1     bit127 - bit1   127-bit CID or CSD register
  *                               (including internal CRC)
  *     0         1               End bit
  */

#ifdef CONFIG_DEBUG
  /* Check that R1 is the correct response to this command */

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R2_RESPONSE)
    {
      fdbg("ERROR: Wrong response CMD=%08x\n", cmd);
      return -EINVAL;
    }
#endif

  /* Verify that the response is available */
 
  regval = getreg32(STM32_SDIO_STA);
  if (regval & SDIO_STA_CTIMEOUT)
    {
      putreg32(SDIO_ICR_CTIMEOUTC, STM32_SDIO_ICR);
      return -ETIMEDOUT;
    }
  else if (regval & SDIO_STA_CCRCFAIL)
    {
      putreg32(SDIO_ICR_CCRCFAILC, STM32_SDIO_ICR);
      return -EIO;
    }
  else if ((regval & SDIO_STA_CMDREND) == 0)
    {
      fdbg("ERROR: Status is not yet available: %08x\n", regval);
      return -EBUSY;
    }

  /* Return the long response */

  putreg32(SDIO_ICR_STATICFLAGS, STM32_SDIO_ICR);
  if (rlong)
    {
      rlong[0] = getreg32(STM32_SDIO_RESP1);
      rlong[1] = getreg32(STM32_SDIO_RESP2);
      rlong[2] = getreg32(STM32_SDIO_RESP3);
      rlong[3] = getreg32(STM32_SDIO_RESP4);
    }
  return OK;
}

static int stm32_recvshort(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 *rshort)
{
  uint32 regval;

 /* R3  OCR (48-bit)
  *     47        0               Start bit
  *     46        0               Transmission bit (0=from card)
  *     45:40     bit5   - bit0   Reserved
  *     39:8      bit31  - bit0   32-bit OCR register
  *     7:1       bit6   - bit0   Reserved
  *     0         1               End bit
  */

  /* Check that this is the correct response to this command */

#ifdef CONFIG_DEBUG
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      fdbg("ERROR: Wrong response CMD=%08x\n", cmd);
      return -EINVAL;
    }
#endif

  regval = getreg32(STM32_SDIO_STA);
  if (regval & SDIO_STA_CTIMEOUT)
    {
       putreg32(SDIO_ICR_CTIMEOUTC, STM32_SDIO_ICR);
       return -ETIMEDOUT;
    }
  else if ((regval & SDIO_STA_CMDREND) == 0)
    {
      fdbg("ERROR: Status is not yet available: %08x\n", regval);
      return -EBUSY;
    }

 /* Return the short response */

  putreg32(SDIO_ICR_STATICFLAGS, STM32_SDIO_ICR);
  if (rshort)
    {
      *rshort = getreg32(STM32_SDIO_RESP1);
    }
  return OK;
}

/* MMC responses not supported */

static int stm32_recvnotimpl(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 *rnotimpl)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling stm32_waitevent.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
 *   setup and the subsequent events.
 *
 *   The enabled events persist until either (1) SDIO_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) SDIO_EVENTWAIT
 *   returns.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_waitenable(FAR struct sdio_dev_s *dev,
                             sdio_eventset_t eventset)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s*)dev;
  uint32 waitmask;
 
  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  stm32_configwaitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitmask = 0;
  if ((eventset & SDIOWAIT_CMDDONE) != 0)
    {
      waitmask |= SDIO_CMDDONE_MASK;
    }

  if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
    {
      waitmask |= SDIO_CRCRESP_MASK;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
      waitmask |= SDIO_XFRDONE_MASK;
    }

  /* Enable event-related interrupts */

  putreg32(SDIO_WAITALL_ICR, STM32_SDIO_ICR);
  stm32_configwaitints(priv, waitmask, eventset, 0);
}

/****************************************************************************
 * Name: stm32_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when stm32_eventwait
 *   returns.  SDIO_WAITEVENTS must be called again before stm32_eventwait
 *   can be used again.
 *
 * Input Parameters:
 *   dev     - An instance of the SDIO device interface
 *   timeout - Maximum time in milliseconds to wait.  Zero means no timeout.
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  If no events the
 *   returned event set is zero, then the wait was terminated by the timeout.
 *   All events are cleared disabled after the wait concludes.
 *
 ****************************************************************************/

static ubyte stm32_eventwait(FAR struct sdio_dev_s *dev, uint32 timeout)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s*)dev;
  ubyte wkupevent = 0;

  DEBUGASSERT(priv->waitevents != 0);

  /* Loop until the event (or the timeout occurs). Race conditions are avoided
   * by calling stm32_waitenable prior to triggering the logic that will cause
   * the wait to terminate.  Under certain race conditions, the waited-for
   * may have already occurred before this function was called!
   */
#warning "Timeout logic not implemented"
  for (;;)
    {
      /* Wait for an event in event set to occur.  If this the event has already
       * occurred, then the semaphore will already have been incremented and
       * there will be no wait.
       */

      stm32_takesem(priv);

      /* Check if the event has occurred. */

      wkupevent = (ubyte)(priv->wkupevent & priv->waitevents);
      if (wkupevent != 0)
        {
          /* Yes... break out of the loop with wkupevent non-zero */

          break;
        }
    }

  /* Disable event-related interrupts */

  stm32_configwaitints(priv, 0, 0, 0);
  return wkupevent;
}

/****************************************************************************
 * Name: stm32_events
 *
 * Description:
 *   Return the current event set.  This supports polling for SDIO (vs.
 *   waiting). Only enabled events need be reported.
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   Event set containing the current events (All pending events are cleared
 *   after reading).
 *
 ****************************************************************************/

static ubyte stm32_events(FAR struct sdio_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: stm32_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in stm32_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this methos.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOMEDIA_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_callbackenable(FAR struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s*)dev;
  DEBUGASSERT(priv != NULL);
  priv->cbevents = eventset;
  stm32_callback(priv);
}

/****************************************************************************
 * Name: stm32_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDIO_CALLBACKENABLE
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The funtion to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

static int stm32_registercallback(FAR struct sdio_dev_s *dev,
                                  sdio_mediachange_t callback, void *arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s*)dev;

  /* Disable callbacks and register this callback and is argument */

  DEBUGASSERT(priv != NULL);
  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: stm32_dmasupported
 *
 * Description:
 *   Return TRUE if the hardware can support DMA
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   TRUE if DMA is supported.
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static boolean stm32_dmasupported(FAR struct sdio_dev_s *dev)
{
  return TRUE;
}
#endif

/****************************************************************************
 * Name: stm32_dmarecvsetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int stm32_dmarecvsetup(FAR struct sdio_dev_s *dev, FAR ubyte *buffer,
                              size_t buflen)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32 dblocksize;
  int ret = -EINVAL;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  stm32_datadisable();

  /* Wide bus operation is required for DMA */

  if (priv->widebus)
    {
      /* Save the destination buffer information for use by the interrupt handler */

      priv->buffer    = (uint32*)buffer;
      priv->remaining = buflen;
      priv->result    = -EBUSY;
      priv->dmamode   = TRUE;

      /* Then set up the SDIO data path */

      dblocksize = stm32_log2(buflen) << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
      stm32_dataconfig(SDIO_DTIMER_DATATIMEOUT, buflen, dblocksize|SDIO_DCTRL_DTDIR);

      /* Configure the RX DMA */

      stm32_configxfrints(priv, SDIO_DMARECV_MASK);

      putreg32(1, SDIO_DCTRL_DMAEN_BB);
      stm32_dmasetup(priv->dma, STM32_SDIO_FIFO, (uint32)buffer,
                     (buflen + 3) >> 2, SDIO_RXDMA16_CONFIG);
 
     /* Start the DMA */

      stm32_dmastart(priv->dma, stm32_dmacallback, priv, FALSE);
      ret = OK;
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: stm32_dmasendsetup
 *
 * Description:
 *   Setup to perform a write DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For write transfers, this may mean
 *   flushing the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA into
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int stm32_dmasendsetup(FAR struct sdio_dev_s *dev,
                               FAR const ubyte *buffer, size_t buflen)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32 dblocksize;
  int ret = -EINVAL;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  stm32_datadisable();

  /* Wide bus operation is required for DMA */

  if (priv->widebus)
    {
      /* Save the source buffer information for use by the interrupt handler */

      priv->buffer    = (uint32*)buffer;
      priv->remaining = buflen;
      priv->result    = -EBUSY;
      priv->dmamode   = TRUE;

      /* Then set up the SDIO data path */

      dblocksize = stm32_log2(buflen) << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
      stm32_dataconfig(SDIO_DTIMER_DATATIMEOUT, buflen, dblocksize);

      /* Enable TX interrrupts */

      stm32_configxfrints(priv, SDIO_DMASEND_MASK);

      /* Configure the TX DMA */

      stm32_dmasetup(priv->dma, STM32_SDIO_FIFO, (uint32)buffer,
                     (buflen + 3) >> 2, SDIO_TXDMA16_CONFIG);
      putreg32(1, SDIO_DCTRL_DMAEN_BB);

      /* Start the DMA */

      stm32_dmastart(priv->dma, stm32_dmacallback, priv, FALSE);
      ret = OK;
    }
  return ret;
}
#endif

/****************************************************************************
 * Initialization/uninitialization/reset
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_callback
 *
 * Description:
 *   Perform callback.
 *
 * Assumptions:
 *   This function does not execute in the context of an interrupt handler.
 *   It may be invoked on any user thread or scheduled on the work thread
 *   from an interrupt handler.
 *
 ****************************************************************************/

static void stm32_callback(void *arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s*)arg;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  if (priv->callback)
    {
      /* Yes.. Check for enabled callback events */

      if ((stm32_status(&priv->dev) & SDIO_STATUS_PRESENT) != 0)
        {
          /* Media is present.  Is the media inserted event enabled? */

          if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
           {
             /* No... return without performing the callback */

              return;
            }
        }
      else
        {
          /* Media is not present.  Is the media eject event enabled? */

          if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
            {
              /* No... return without performing the callback */

              return;
            }
        }

      /* Perform the callback, disabling further callbacks.  Of course, the
       * the callback can (and probably should) re-enable callbacks.
       */

      priv->cbevents = 0;
      priv->callback(priv->cbarg);
      
    }
}

/****************************************************************************
 * Name: stm32_default
 *
 * Description:
 *   Restore SDIO registers to their default, reset values
 *
 ****************************************************************************/

static void stm32_default(void)
{
  putreg32(SDIO_POWER_RESET,  STM32_SDIO_POWER);
  putreg32(SDIO_CLKCR_RESET,  STM32_SDIO_CLKCR);
  putreg32(SDIO_ARG_RESET,    STM32_SDIO_ARG);
  putreg32(SDIO_CMD_RESET,    STM32_SDIO_CMD);
  putreg32(SDIO_DTIMER_RESET, STM32_SDIO_DTIMER);
  putreg32(SDIO_DLEN_RESET,   STM32_SDIO_DLEN);
  putreg32(SDIO_DCTRL_RESET,  STM32_SDIO_DCTRL);
  putreg32(SDIO_ICR_RESET,    STM32_SDIO_ICR);
  putreg32(SDIO_MASK_RESET,   STM32_SDIO_MASK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SDIO interface structure.  NULL is returned on failures.
 *
 ****************************************************************************/

FAR struct sdio_dev_s *sdio_initialize(int slotno)
{
  /* There is only one, slot */

  struct stm32_dev_s *priv = &g_sdiodev;

  /* Initialize the SDIO slot structure */

  sem_init(&priv->waitsem, 0, 0);

  /* Allocate a DMA channel */

#ifdef CONFIG_SDIO_DMA
  priv->dma = stm32_dmachannel(DMACHAN_SDIO);
#endif

  /* Configure GPIOs for 4-bit, wide-bus operation (the chip is capable of
   * 8-bit wide bus operation but D4-D7 are not configured).
   */

  stm32_configgpio(GPIO_SDIO_D0);
  stm32_configgpio(GPIO_SDIO_D1);
  stm32_configgpio(GPIO_SDIO_D2);
  stm32_configgpio(GPIO_SDIO_D3);
  stm32_configgpio(GPIO_SDIO_CK);
  stm32_configgpio(GPIO_SDIO_CMD);

  /* Put SDIO registers in their default, reset state */

  stm32_default();

  /* Configure the SDIO peripheral */

  stm32_setclkcr(STM32_CLCKCR_INIT);
  stm32_setpwrctrl(SDIO_POWER_PWRCTRL_ON);
  stm32_clkenable();

  return &g_sdiodev.dev;
}

#endif /* CONFIG_STM32_SDIO */

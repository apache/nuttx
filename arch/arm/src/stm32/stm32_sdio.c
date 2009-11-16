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

#include <debug.h>
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

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

#ifndef CONFIG_SDIO_DMAPRIO
#  define CONFIG_SDIO_DMAPRIO DMA_CCR_PRIMED
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
#define STM32_CLCKCR_TRANSFER \
  (SDIO_TRANSFER_CLKDIV|SDIO_CLKCR_RISINGEDGE|SDIO_CLKCR_WIDBUS_D1)
#define STM32_CLKCR_WIDETRANSFER \
  (SDIO_TRANSFER_CLKDIV|SDIO_CLKCR_RISINGEDGE|SDIO_CLKCR_WIDBUS_D4)

/* Timing */

#define SDIO_CMDTIMEOUT  100000
#define SDIO_LONGTIMEOUT 0x7fffffff

/* DMA CCR register settings */

#define SDIO_RXDMA16_CONFIG   (CONFIG_SDIO_DMAPRIO|DMA_CCR_MSIZE_16BITS|\
                               DMA_CCR_PSIZE_16BITS|DMA_CCR_MINC)
#define SDIO_TXDMA16_CONFIG   (CONFIG_SDIO_DMAPRIO|DMA_CCR_MSIZE_16BITS|\
                               DMA_CCR_PSIZE_16BITS|DMA_CCR_MINC|DMA_CCR_DIR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the STM32 SDIO interface */

struct stm32_dev_s
{
  struct sdio_dev_s     dev;        /* Standard, base MMC/SD interface */
  
  /* STM32-specific extensions */

  sem_t                 eventsem;   /* Implements event waiting */
  sdio_event_t          waitevents; /* Set of events to be waited for */
  volatile sdio_event_t wkupevents; /* Set of events that caused the wakeup */
  sdio_event_t          cbevents;   /* Set of events to be cause callbacks */
  sdio_mediachange_t    callback;   /* Registered callback function */

  /* DMA support */

#ifdef CONFIG_SDIO_DMA
  DMA_HANDLE            dma;        /* Handle for DMA channel */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static void   stm32_takesem(struct stm32_dev_s *priv);
#define       stm32_givesem(priv) (sem_post(&priv->waitsem))
static inline void stm32_setclkcr(uint32 clkcr);
static inline void stm32_enableint(uint32 bitset);
static inline void stm32_disableint(uint32 bitset);
static void   stm32_setpwrctrl(uint32 pwrctrl);
static inline uint32 stm32_getpwrctrl(void);
static inline void stm32_clkenable(void)
static inline void stm32_clkdisable(void)

/* DMA Helpers **************************************************************/

#ifdef CONFIG_SDIO_DMA
static void  stm32_dmacallback(DMA_HANDLE handle, ubyte isr, void *arg);
#endif

/* Data Transfer Helpers ****************************************************/

static ubyte stm32_log2(uint16 value);
static void  stm32_dataconfig(uint32 timeout, uint32 dlen, uint32 dctrl);
static void  stm32_datadisable(void);

/* SDIO interface methods ***************************************************/

/* Initialization/setup */

static void  stm32_reset(FAR struct sdio_dev_s *dev);
static ubyte stm32_status(FAR struct sdio_dev_s *dev);
static void  stm32_widebus(FAR struct sdio_dev_s *dev, boolean enable);
static void  stm32_clock(FAR struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int   stm32_attach(FAR struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static void  stm32_sendcmd(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 arg);
static int   stm32_sendsetup(FAR struct sdio_dev_s *dev, uint32 nbytes);
static int   stm32_senddata(FAR struct sdio_dev_s *dev,
               FAR const ubyte *buffer);

static int   stm32_waitresponseFAR struct sdio_dev_s *dev, uint32 cmd);
static int   stm32_recvshortcrc(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 *rshort);
static int   stm32_recvlong(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 rlong[4]);
static int   stm32_recvshort(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 *rshort);
static int   stm32_recvnotimpl(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 *rnotimpl);
static int   stm32_recvsetup(FAR struct sdio_dev_s *dev, uint32 nbytes);
static int   stm32_recvdata(FAR struct sdio_dev_s *dev, FAR ubyte *buffer);

/* EVENT handler */

static void  stm32_waitenable(FAR struct sdio_dev_s *dev, sdio_event_t eventset,
               boolean enable);
static ubyte stm32_eventwait(FAR struct sdio_dev_s *dev, uint32 timeout);
static ubyte stm32_events(FAR struct sdio_dev_s *dev);
static void  stm32_callbackenable(FAR struct sdio_dev_s *dev, sdio_event_t eventset);
static int   stm32_registercallback(FAR struct sdio_dev_s *dev,
               sdio_mediachange_t callback, void *arg)

/* DMA */

#ifdef CONFIG_SDIO_DMA
static boolean stm32_dmasupported(FAR struct sdio_dev_s *dev);
static int   stm32_dmareadsetup(FAR struct sdio_dev_s *dev,
               FAR ubyte *buffer, size_t buflen);
static int   stm32_dmawritesetup(FAR struct sdio_dev_s *dev,
               FAR const ubyte *buffer, size_t buflen);
static int   stm32_dmastart(FAR struct sdio_dev_s *dev);
static int   stm32_dmastatus(FAR struct sdio_dev_s *dev,
               size_t *remaining);
#endif

/* Initialization/uninitialization/reset ************************************/

static void  stm32_callback(void *arg);
static void  stm32_default(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct stm32_dev_s g_mmcsd =
{
  .dev =
  {
    .reset            = stm32_reset,
    .status           = stm32_status,
    .widebus          = stm32_widebus,
    .clock            = stm32_clock,
    .attach           = stm32_attach,
    .sendcmd          = stm32_sendcmd,
    .sendsetup        = stm32_sendsetup,
    .senddata         = stm32_senddata,
    .waitresponse     = stm32_waitresponse,
    .recvR1           = stm32_recvshortcrc,
    .recvR2           = stm32_recvlong,
    .recvR3           = stm32_recvshort,
    .recvR4           = stm32_recvnotimpl,
    .recvR5           = stm32_recvnotimpl,
    .recvR6           = stm32_recvshortcrc,
    .recvR7           = stm32_recvshort,
    .recvsetup        = stm32_recvsetup,
    .recvdata         = stm32_recvdata,
    .waitenable       = stm32_waitenable,
    .eventwait        = stm32_eventwait,
    .events           = stm32_events,
    .callbackenable   = stm32_callbackenable,
    .registercallback = stm32_registercallback,
#ifdef CONFIG_SDIO_DMA
    .dmasupported     = stm32_dmasupported,
    .dmareadsetup     = stm32_dmareadsetup,
    .dmawritesetup    = stm32_dmawritesetup,
    .dmastart         = stm32_dmastart,
    .dmastatus        = stm32_dmastatus,
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
 * Name: stm32_enableint
 *
 * Description:
 *   Enable SDIO interrupts
 *
 * Input Parameters:
 *   bitset - The set of bits in the SDIO MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_enableint(uint32 bitset)
{
  uint32 regval;
  regval  = getreg32(STM32_SDIO_MASK);
  regval |= bitset;
  putreg32(regval, STM32_SDIO_MASK);
}

/****************************************************************************
 * Name: stm32_disableint
 *
 * Description:
 *   Disable SDIO interrupts
 *
 * Input Parameters:
 *   bitset - The set of bits in the SDIO MASK register to clear
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_disableint(uint32 bitset)
{
  uint32 regval;
  regval  = getreg32(STM32_SDIO_MASK);
  regval &= ~bitset;
  putreg32(regval, STM32_SDIO_MASK);
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
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)arg;

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TRANSFERDONE) != 0)
    {
      /* Yes, wake up the waiting thread */

      priv->wkupevent = SDIOWAIT_TRANSFERDONE;
      sdio_semgive(priv);
    }
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
  regval |=  (dctrl|DIO_DCTRL_DTEN);
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

  putreg32(SD_DATATIMEOUT, STM32_SDIO_DTIMER); /* Reset DTIMER */
  putreg32(0,              STM32_SDIO_DLEN);   /* Reset DLEN */

  /* Reset DCTRL DTEN, DTDIR, DTMODE, DMAEN, and DBLOCKSIZE fields */

  regval  = getreg32(STM32_SDIO_DCTRL);
  regval &= ~(SDIO_DCTRL_DTEN|SDIO_DCTRL_DTDIR|SDIO_DCTRL_DTMODE|
              SDIO_DCTRL_DMAEN|SDIO_DCTRL_DBLOCKSIZE_MASK);
  putreg32(regval, STM32_SDIO_DCTRL);
}

/****************************************************************************
 * SDIO Interface Methods
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_reset
 *
 * Description:
 *   Reset the MMC/SD controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
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
 *   Get MMC/SD status.
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
 *   dev  - An instance of the MMC/SD device interface
 *   wide - TRUE: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_widebus(FAR struct sdio_dev_s *dev, boolean wide)
{
  if (wide)
    {
      priv->mode = MMCSDMODE_DMA;
    }
}

/****************************************************************************
 * Name: stm32_clock
 *
 * Description:
 *   Enable/disable MMC/SD clocking
 *
 * Input Parameters:
 *   dev  - An instance of the MMC/SD device interface
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
 *   dev    - An instance of the MMC/SD device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

static int stm32_attach(FAR struct sdio_dev_s *dev)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_sendcmd
 *
 * Description:
 *   Send the MMC/SD command
 *
 * Input Parameters:
 *   dev  - An instance of the MMC/SD device interface
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
 * Name: stm32_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data trasfer from the card.  This method
 *   will do whatever controller setup is necessary.  This would be called
 *   for SD memory just AFTER sending CMD24 (WRITE_BLOCK), CMD25
 *   (WRITE_MULTIPLE_BLOCK), ... and before SDIO_SENDDATA is called.
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int stm32_sendsetup(FAR struct sdio_dev_s *dev, uint32 nbytes)
{
  uint32 dctrl = stm32_log2(nbytes) << SDIO_DCTRL_DBLOCKSIZE_SHIFT);
  stm32_dataconfig(SD_DATATIMEOUT, nbytes, dctrl);
  return OK;
}

/****************************************************************************
 * Name: stm32_senddata
 *
 * Description:
 *   Send more MMC/SD data
 *
 * Input Parameters:
 *   dev  - An instance of the MMC/SD device interface
 *   data - Data to be sent
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int stm32_senddata(FAR struct sdio_dev_s *dev, FAR const ubyte *buffer)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.
 *
 * Input Parameters:
 *   dev  - An instance of the MMC/SD device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int stm32_waitresponseFAR struct sdio_dev_s *dev, uint32 cmd)
{
  sint32 timeout = SDIO_LONGTIMEOUT;
  uint32 events;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      timeout = SDIO_CMDTIMEOUT;
      events  = SDIO_STA_CMDSENT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R6_RESPONSE:
      events  = SDIO_STA_CTIMEOUT|SDIO_STA_CCRCFAIL|SDIO_STA_CMDREND;
      break;

    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
      return -ENOSYS;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      events  = SDIO_STA_CTIMEOUT|SDIO_STA_CMDREND;
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
 *   Receive response to MMC/SD command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
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
       cmd & MMCSD_RESPONSE_MASK  |= MMCSD_R6_RESPONSE)
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
       cmd & MMCSD_RESPONSE_MASK  |= MMCSD_R7_RESPONSE)
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
 * Name: stm32_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data trasfer from the card.  This method
 *   will do whatever controller setup is necessary.  This would be called
 *   for SD memory just BEFORE sending CMD13 (SEND_STATUS), CMD17
 *   (READ_SINGLE_BLOCK), CMD18 (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), ...
 *   and before SDIO_RECVDATA is called.
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int stm32_recvsetup(FAR struct sdio_dev_s *dev, uint32 nbytes)
{
  uint32 dctrl = (stm32_log2(nbytes) << SDIO_DCTRL_DBLOCKSIZE_SHIFT)) | SDIO_DCTRL_DTDIR;
  stm32_dataconfig(SD_DATATIMEOUT, nbytes, dctrl);
  return OK;
}

/****************************************************************************
 * Name: stm32_recvdata
 *
 * Description:
 *   Receive data from MMC/SD
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *   buffer - Buffer in which to receive the data
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int stm32_recvdata(FAR struct sdio_dev_s *dev, FAR ubyte *buffer)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_waitenable
 *
 * Description:
 *   Enable/disable of a set of MMC/SD wait events.  This is part of the
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
 *   dev      - An instance of the MMC/SD device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_waitenable(FAR struct sdio_dev_s *dev, sdio_event_t eventset)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s*)dev;

  /* This odd sequence avoids race conditions */

  DEBUGASSERT(priv != NULL);
  priv->waitevents = 0;
  priv->wkupevents = 0;
  priv->waitevents = eventset;
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
 *   dev     - An instance of the MMC/SD device interface
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
  ubyte wkupevents = 0;

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

      wkupevents = (ubyte)(priv->wkupevents & priv->waitevents)
      if (wkupevents != 0)
        {
          /* Yes... break out of the loop with wkupevents non-zero */

          break;
        }
    }

  /* Clear all enabled wait events before returning */

  priv->waitevents = 0;
  priv->wkupevents = 0;
  return wkupevents;
}

/****************************************************************************
 * Name: stm32_events
 *
 * Description:
 *   Return the current event set.  This supports polling for MMC/SD (vs.
 *   waiting). Only enabled events need be reported.
 *
 * Input Parameters:
 *   dev - An instance of the MMC/SD device interface
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
 *   Enable/disable of a set of MMC/SD callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in stm32_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this methos.
 *
 * Input Parameters:
 *   dev      - An instance of the MMC/SD device interface
 *   eventset - A bitset of events to enable or disable (see SDIOMEDIA_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_callbackenable(FAR struct sdio_dev_s *dev, sdio_event_t eventset)
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
 *   dev - An instance of the MMC/SD device interface
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
 * Name: stm32_dmareadsetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int  tm32_dmareadsetup(FAR struct sdio_dev_s *dev, FAR ubyte *buffer, size_t buflen)
{
  /* Configure the RX DMA */

  stm32_dmasetup(priv->dma, STM32_SDIO_FIFO, (uint32)buffer,
                 (buflen + 3) >> 2, SDIO_RXDMA16_CONFIG);
}
#endif

/****************************************************************************
 * Name: stm32_dmawritesetup
 *
 * Description:
 *   Setup to perform a write DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For write transfers, this may mean
 *   flushing the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *   buffer - The memory to DMA into
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int stm32_dmawritesetup(FAR struct sdio_dev_s *dev,
                               FAR const ubyte *buffer, size_t buflen)
{
  /* Configure the RX DMA */

  stm32_dmasetup(priv->dma, STM32_SDIO_FIFO, (uint32)buffer,
                 (buflen + 3) >> 2, SDIO_TXDMA16_CONFIG);
}
#endif

/****************************************************************************
 * Name: stm32_dmastart
 *
 * Description:
 *   Start the DMA
 *
 * Input Parameters:
 *   dev - An instance of the MMC/SD device interface
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int stm32_dmastart(FAR struct sdio_dev_s *dev)
{
   stm32_dmastart(priv->dma, sdio_dmacallback, priv, FALSE);
}
#endif

/****************************************************************************
 * Name: stm32_dmastatus
 *
 * Description:
 *   Return the number of bytes remaining in the DMA transfer
 *
 * Input Parameters:
 *   dev       - An instance of the MMC/SD device interface
 *   remaining - A pointer to location in which to return the number of bytes
 *               remaining in the transfer.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int stm32_dmastatus(FAR struct sdio_dev_s *dev, size_t *remaining)
{
#ifdef CONFIG_DEBUG
  if (remaining)
    {
      *remaining = getreg32(STM32_SDIO_DCOUNT);
      return OK;
    }
  return -EINVAL;
#else
  *remaining = getreg32(STM32_SDIO_DCOUNT);
  return OK;
#endif
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

FAR sdio_dev_s *mmcsd_slotinitialize(int slotno)
{
  /* There is only one, slot */

  struct stm32_dev_s *priv = &g_sdiodev;

  /* Initialize the SDIO slot structure */

  sem_init(&priv->eventsem, 0, 0);

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
  stm32_clkenable(ENABLE);

  return &g_sdiodev.dev;
}

#endif /* CONFIG_STM32_SDIO */

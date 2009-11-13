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

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the STM32 SDIO interface */

struct stm32_dev_s
{
  struct sdio_dev_s dev; /* Standard, base MMC/SD interface */
  
  /* STM32-specific extensions */

  ubyte type;            /* Card type (see MMCSD_CARDTYPE_ definitions) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static inline void stm32_setclkcr(uint32 clkcr);
static inline void stm32_enableint(uint32 bitset);
static inline void stm32_disableint(uint32 bitset);
static void   stm32_setpwrctrl(uint32 pwrctrl);
static inline uint32 stm32_getpwrctrl(void);
static inline void stm32_clkenable(void)
static inline void stm32_clkdisable(void)

/* DMA Helpers **************************************************************/

static inline void stm32_dmaenable(void);

/* Data Transfer Helpers ****************************************************/

static void  stm32_dataconfig(uint32 timeout, uint32 dlen, uint32 dctrl);
static void  stm32_datadisable(void);

/* SDIO interface methods ***************************************************/

/* Initialization/setup */

static void  stm32_reset(FAR struct sdio_dev_s *dev);
static ubyte stm32_status(FAR struct sdio_dev_s *dev);
static void  stm32_widebus(FAR struct sdio_dev_s *dev, boolean enable);
static void  stm32_clock(FAR struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int   stm32_setblocklen(FAR struct sdio_dev_s *dev, int blocklen,
              int nblocks);
static int   stm32_attach(FAR struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static void  stm32_sendcmd(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 arg);
static int   stm32_senddata(FAR struct sdio_dev_s *dev,
               FAR const ubyte *buffer);

static int   stm32_recvshortcrc(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 *rshort);
static int   stm32_recvlong(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 rlong[4]);
static int   stm32_recvshort(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 *rshort);
static int   stm32_recvnotimpl(FAR struct sdio_dev_s *dev, uint32 cmd, uint32 *rnotimpl);
static int   stm32_recvdata(FAR struct sdio_dev_s *dev, FAR ubyte *buffer);

/* EVENT handler */

static void  stm32_eventenable(FAR struct sdio_dev_s *dev, ubyte eventset,
               boolean enable);
static ubyte stm32_eventwait(FAR struct sdio_dev_s *dev, uint32 timeout);
static ubyte stm32_events(FAR struct sdio_dev_s *dev);

/* DMA */

#ifdef CONFIG_SDIO_DMA
static boolean stm32_dmasupported(FAR struct sdio_dev_s *dev);
#ifdef CONFIG_DATA_CACHE
static void  stm32_coherent(FAR struct sdio_dev_s *dev, FAR void *addr,
               size_t len, boolean write);
#endif
static int   stm32_dmareadsetup(FAR struct sdio_dev_s *dev,
               FAR ubyte *buffer);
static int   stm32_dmawritesetup(FAR struct sdio_dev_s *dev,
               FAR const ubyte *buffer);
static int   stm32_dmaenable(FAR struct sdio_dev_s *dev);
static int   stm32_dmastart(FAR struct sdio_dev_s *dev);
static int   stm32_dmastop(FAR struct sdio_dev_s *dev);
static int   stm32_dmastatus(FAR struct sdio_dev_s *dev,
               size_t *remaining);
#endif

/* Initialization/uninitialization/reset ************************************/

static void  stm32_default(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct stm32_dev_s g_mmcsd =
{
  .dev =
  {
    .reset         = stm32_reset,
    .status        = stm32_status,
    .widebus       = stm32_widebus,
    .clock         = stm32_clock,
    .setblocklen   = stm32_setblocklen,
    .attach        = stm32_attach,
    .sendcmd       = stm32_sendcmd,
    .senddata      = stm32_senddata,
    .recvR1        = stm32_recvshortcrc,
    .recvR2        = stm32_recvlong,
    .recvR3        = stm32_recvshort,
    .recvR4        = stm32_recvnotimpl,
    .recvR5        = stm32_recvnotimpl,
    .recvR6        = stm32_recvshortcrc,
    .recvR7        = stm32_recvshort,
    .recvdata      = stm32_recvdata,
    .eventenable   = stm32_eventenable,
    .eventwait     = stm32_eventwait,
    .events        = stm32_events,
#ifdef CONFIG_SDIO_DMA
    .dmasupported  = stm32_dmasupported,
#ifdef CONFIG_DATA_CACHE
    .coherent      = stm32_coherent,
#endif
    .dmareadsetup  = stm32_dmareadsetup,
    .dmawritesetup = stm32_dmawritesetup,
    .dmaenable     = stm32_dmaenable,
    .dmastart      = stm32_dmastart,
    .dmastop       = stm32_dmastop,
    .dmastatus     = stm32_dmastatus,
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
static inline void stm32_dmaenable(void)
{
  putreg32(1, SDIO_DCTRL_DMAEN_BB);
}

/****************************************************************************
 * Data Transfer Helpers
 ****************************************************************************/
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
 * Name: stm32_setblocklen
 *
 * Description:
 *   Set the MMC/SD block length and block count
 *
 * Input Parameters:
 *   dev      - An instance of the MMC/SD device interface
 *   blocklen - The block length
 *   nblocks  - The block count
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 ****************************************************************************/

static int stm32_setblocklen(FAR struct sdio_dev_s *dev, int blocklen, int nblocks)
{
  return -ENOSYS;
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
 *   Number of bytes sent on succes; a negated errno on failure
 *
 ****************************************************************************/

static int stm32_senddata(FAR struct sdio_dev_s *dev, FAR const ubyte *buffer)
{
  return -ENOSYS;
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
 *   Number of bytes sent on succes; a negated errno on failure
 *
 ****************************************************************************/

static int stm32_recvdata(FAR struct sdio_dev_s *dev, FAR ubyte *buffer)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_eventenable
 *
 * Description:
 *   Enable/disable notification of a set of MMC/SD events
 *
 * Input Parameters:
 *   dev      - An instance of the MMC/SD device interface
 *   eventset - A bitset of events to enable or disable (see MMCSDEVENT_*
 *              definitions
 *   enable   - TRUE: enable event; FALSE: disable events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_eventenable(FAR struct sdio_dev_s *dev, ubyte eventset,
                              boolean enable)
{
}

/****************************************************************************
 * Name: stm32_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout)
 *
 * Input Parameters:
 *   dev     - An instance of the MMC/SD device interface
 *   timeout - Maximum time in milliseconds to wait.  Zero means no timeout.
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  If no events the
 *   returned event set is zero, then the wait was terminated by the timeout.
 *
 ****************************************************************************/

static ubyte stm32_eventwait(FAR struct sdio_dev_s *dev, uint32 timeout)
{
  return 0;
}

/****************************************************************************
 * Name: stm32_events
 *
 * Description:
 *   Return the current event set.  This supports polling for MMC/SD (vs.
 *   waiting).
 *
 * Input Parameters:
 *   dev     - An instance of the MMC/SD device interface
 *
 * Returned Value:
 *   Event set containing the current events (cleared after reading).
 *
 ****************************************************************************/

static ubyte stm32_events(FAR struct sdio_dev_s *dev)
{
  return 0;
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
 * Name: stm32_coherent
 *
 * Description:
 *   If the processor supports a data cache, then this method will make sure
 *   that the contents of the DMA memory and the data cache are coherent in
 *   preparation for the DMA transfer.  For write transfers, this may mean
 *   flushing the data cache, for read transfers this may mean invalidating
 *   the data cache.
 *
 * Input Parameters:
 *   dev   - An instance of the MMC/SD device interface
 *   addr  - The beginning address of the DMA
 *   len   - The length of the DMA
 *   write - TRUE: A write DMA will be performed; FALSE: a read DMA will be
 *           performed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SDIO_DMA) && defined(CONFIG_DATA_CACHE)
static void stm32_coherent(FAR struct sdio_dev_s *dev, FAR void *addr,
                           size_t len, boolean write)
{
}
#endif

/****************************************************************************
 * Name: stm32_dmareadsetup
 *
 * Description:
 *   Setup to perform a read DMA
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *   buffer - The memory to DMA from
 *
 * Returned Value:
 *   OK on succes; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int  tm32_dmareadsetup(FAR struct sdio_dev_s *dev, FAR ubyte *buffer)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: stm32_dmawritesetup
 *
 * Description:
 *   Setup to perform a write DMA
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *   buffer - The memory to DMA into
 *
 * Returned Value:
 *   OK on succes; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int stm32_dmawritesetup(FAR struct sdio_dev_s *dev,
                               FAR const ubyte *buffer)
{
  return -ENOSYS;
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
 *   OK on succes; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int stm32_dmastart(FAR struct sdio_dev_s *dev)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: stm32_dmastop
 *
 * Description:
 *   Stop the DMA
 *
 * Input Parameters:
 *   dev - An instance of the MMC/SD device interface
 *
 * Returned Value:
 *   OK on succes; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int stm32_dmastop(FAR struct sdio_dev_s *dev)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: stm32_dmastatus
 *
 * Description:
 *   Returnt the number of bytes remaining in the DMA transfer
 *
 * Input Parameters:
 *   dev       - An instance of the MMC/SD device interface
 *   remaining - A pointer to location in which to return the number of bytes
 *               remaining in the transfer.
 *
 * Returned Value:
 *   OK on succes; a negated errno on failure
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
 * Name: mmcsd_slotinitialize
 *
 * Description:
 *   Initialize one slot for operation using the MMC/SD interface
 *
 * Input Parameters:
 *   minor - The MMC/SD minor device number.  The MMC/SD device will be
 *     registered as /dev/mmcsdN where N is the minor number
 *   slotno - The slot number to use.  This is only meaningful for architectures
 *     that support multiple MMC/SD slots.  This value must be in the range
 *     {0, ..., CONFIG_MMCSD_NSLOTS}.
 *   dev - And instance of an MMC/SD interface.  The MMC/SD hardware should
 *     be initialized and ready to use.
 *
 ****************************************************************************/

int mmcsd_slotinitialize(int minor, int slotno, FAR struct sdio_dev_s *dev)
{
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

  return -ENOSYS;
}

#endif /* CONFIG_STM32_SDIO */

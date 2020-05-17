/****************************************************************************
 * arch/arm/src/sama5/sam_nand.c
 *
 *   Copyright (C) 2013, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2011, 2012, Atmel Corporation
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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
#include <nuttx/mtd/nand_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/nand_raw.h>
#include <nuttx/mtd/nand_model.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_arch.h"

#include "sam_memories.h"
#include "sam_dmac.h"
#include "sam_pmecc.h"
#include "sam_nand.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_CE
#  define ENABLE_CE(priv)  board_nand_ce(priv->cs, true)
#  define DISABLE_CE(priv) board_nand_ce(priv->cs, false)
#else
#  define ENABLE_CE(priv)
#  define DISABLE_CE(priv)
#endif

/* Nand flash chip status codes */

#define STATUS_ERROR         (1 << 0)
#define STATUS_READY         (1 << 6)

/* NFC ALE CLE command parameter */

#define HSMC_ALE_COL_EN      (1 << 0)
#define HSMC_ALE_ROW_EN      (1 << 1)
#define HSMC_CLE_WRITE_EN    (1 << 2)
#define HSMC_CLE_DATA_EN     (1 << 3)
#define HSMC_CLE_VCMD2_EN    (1 << 4)

/* Number of tries for erasing or writing block */

#define NAND_ERASE_NRETRIES  2
#define NAND_WRITE_NRETRIES  2

/* DMA Configuration */

#define NFCSRAM_DMA_FLAGS \
   DMACH_FLAG_FIFOCFG_LARGEST | \
   (DMACH_FLAG_PERIPHPID_MAX | DMACH_FLAG_PERIPHAHB_AHB_IF0 | \
   DMACH_FLAG_PERIPHWIDTH_32BITS | DMACH_FLAG_PERIPHINCREMENT | \
   DMACH_FLAG_PERIPHCHUNKSIZE_1 | \
   DMACH_FLAG_MEMPID_MAX | DMACH_FLAG_MEMAHB_AHB_IF0 | \
   DMACH_FLAG_MEMWIDTH_32BITS | DMACH_FLAG_MEMINCREMENT | \
   DMACH_FLAG_MEMCHUNKSIZE_1 | DMACH_FLAG_MEMBURST_4)

#define NAND_DMA_FLAGS8 \
   DMACH_FLAG_FIFOCFG_LARGEST | \
   (DMACH_FLAG_PERIPHPID_MAX | DMACH_FLAG_PERIPHAHB_AHB_IF0 | \
   DMACH_FLAG_PERIPHWIDTH_8BITS | DMACH_FLAG_PERIPHCHUNKSIZE_1 | \
   DMACH_FLAG_MEMPID_MAX | DMACH_FLAG_MEMAHB_AHB_IF0 | \
   DMACH_FLAG_MEMWIDTH_8BITS | DMACH_FLAG_MEMINCREMENT | \
   DMACH_FLAG_MEMCHUNKSIZE_1 | DMACH_FLAG_MEMBURST_4)

#define NAND_DMA_FLAGS16 \
   DMACH_FLAG_FIFOCFG_LARGEST | \
   (DMACH_FLAG_PERIPHPID_MAX | DMACH_FLAG_PERIPHAHB_AHB_IF0 | \
   DMACH_FLAG_PERIPHWIDTH_16BITS | DMACH_FLAG_PERIPHCHUNKSIZE_1 | \
   DMACH_FLAG_MEMPID_MAX | DMACH_FLAG_MEMAHB_AHB_IF0 | \
   DMACH_FLAG_MEMWIDTH_16BITS | DMACH_FLAG_MEMINCREMENT | \
   DMACH_FLAG_MEMCHUNKSIZE_1 | DMACH_FLAG_MEMBURST_4)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level HSMC Helpers */

#if NAND_NBANKS > 1
int             nand_lock(void);
void            nand_unlock(void);
#else
#  define       nand_lock() (0)
#  define       nand_unlock()
#endif

#ifdef CONFIG_SAMA5_NAND_DUMP
#  define nand_dump(m,b,s) lib_dumpbuffer(m,b,s)
#else
#  define nand_dump(m,b,s)
#endif

static void     nand_wait_ready(struct sam_nandcs_s *priv);
static void     nand_nfc_cmdsend(struct sam_nandcs_s *priv, uint32_t cmd,
                  uint32_t acycle, uint32_t cycle0);
static int      nand_operation_complete(struct sam_nandcs_s *priv);
static int      nand_translate_address(struct sam_nandcs_s *priv,
                  uint16_t coladdr, uint32_t rowaddr, uint32_t *acycle0,
                  uint32_t *acycle1234, bool rowonly);
static uint32_t nand_get_acycle(int ncycles);
static void     nand_nfc_cleale(struct sam_nandcs_s *priv,
                   uint8_t mode, uint32_t cmd1, uint32_t cmd2,
                   uint32_t coladdr, uint32_t rowaddr);

/* Interrupt Handling */

static void     nand_wait_cmddone(struct sam_nandcs_s *priv);
static void     nand_setup_cmddone(struct sam_nandcs_s *priv);
static void     nand_wait_xfrdone(struct sam_nandcs_s *priv);
static void     nand_setup_xfrdone(struct sam_nandcs_s *priv);
static void     nand_wait_rbedge(struct sam_nandcs_s *priv);
static void     nand_setup_rbedge(struct sam_nandcs_s *priv);
#if 0 /* Not used */
static void     nand_wait_nfcbusy(struct sam_nandcs_s *priv);
#endif
static uint32_t nand_nfc_poll(void);
#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
static int      hsmc_interrupt(int irq, void *context, FAR void *arg);
#endif

/* DMA Helpers */

#ifdef CONFIG_SAMA5_NAND_DMA
#ifdef CONFIG_SAMA5_NAND_DMADEBUG
static void     nand_dma_sampleinit(struct sam_nandcs_s *priv);
#  define       nand_dma_sample(p,i) sam_dmasample((p)->dma, &(p)->dmaregs[i])
static void     nand_dma_sampledone(struct sam_nandcs_s *priv, int result);

#else
#  define       nand_dma_sampleinit(p)
#  define       nand_dma_sample(p,i)
#  define       nand_dma_sampledone(p,r)

#endif

static int      nand_wait_dma(struct sam_nandcs_s *priv);
static void     nand_dmacallback(DMA_HANDLE handle, void *arg, int result);
static int      nand_dma_read(struct sam_nandcs_s *priv,
                  uintptr_t vsrc, uintptr_t vdest, size_t nbytes,
                  uint32_t dmaflags);
static int      nand_dma_write(struct sam_nandcs_s *priv,
                  uintptr_t vsrc, uintptr_t vdest, size_t nbytes,
                  uint32_t dmaflags);
#endif

/* Raw Data Transfer Helpers */

static int      nand_nfcsram_read(struct sam_nandcs_s *priv,
                  uint8_t *buffer, uint16_t buflen, uint16_t offset);
#ifdef CONFIG_SAMA5_HAVE_PMECC
static int      nand_read(struct sam_nandcs_s *priv, uint8_t *buffer,
                  uint16_t buflen);
#endif

#ifdef CONFIG_SAMA5_HAVE_PMECC
static int      nand_read_pmecc(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, void *data);
#endif

static int      nand_nfcsram_write(struct sam_nandcs_s *priv,
                  uint8_t *buffer, uint16_t buflen, uint16_t offset);
static int      nand_write(struct sam_nandcs_s *priv, uint8_t *buffer,
                  uint16_t buflen, uint16_t offset);

/* NAND Access Helpers */

static int      nand_readpage_noecc(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, void *data, void *spare);

#ifdef CONFIG_SAMA5_HAVE_PMECC
static int      nand_readpage_pmecc(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, void *data);
#endif

static int      nand_writepage_noecc(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, const void *data, const void *spare);

#ifdef CONFIG_SAMA5_HAVE_PMECC
static int      nand_writepage_pmecc(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, const void *data);
#endif

/* MTD driver methods */

static int      nand_eraseblock(struct nand_raw_s *raw, off_t block);
static int      nand_rawread(struct nand_raw_s *raw, off_t block,
                  unsigned int page, void *data, void *spare);
static int      nand_rawwrite(struct nand_raw_s *raw, off_t block,
                  unsigned int page, const void *data, const void *spare);

#ifdef CONFIG_MTD_NAND_HWECC
static int      nand_readpage(struct nand_raw_s *raw, off_t block,
                  unsigned int page, void *data, void *spare);
static int      nand_writepage(struct nand_raw_s *raw, off_t block,
                  unsigned int page, const void *data, const void *spare);
#endif

/* Initialization */

static void     nand_reset(struct sam_nandcs_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These pre-allocated structures hold the state of the MTD driver for NAND
 * on CS0..3 as configured.
 */

#ifdef CONFIG_SAMA5_EBICS0_NAND
static struct sam_nandcs_s g_cs0nand;
#endif
#ifdef CONFIG_SAMA5_EBICS1_NAND
static struct sam_nandcs_s g_cs1nand;
#endif
#ifdef CONFIG_SAMA5_EBICS2_NAND
static struct sam_nandcs_s g_cs2nand;
#endif
#ifdef CONFIG_SAMA5_EBICS3_NAND
static struct sam_nandcs_s g_cs3nand;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* NAND global state */

struct sam_nand_s g_nand;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nand_lock
 *
 * Description:
 *   Get exclusive access to PMECC hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *  Normally success (OK) is returned, but the error -ECANCELED may be
 *  return in the event that task has been canceled.
 *
 ****************************************************************************/

#if NAND_NBANKS > 1
static int nand_lock(void)
{
  return nxsem_wait_uninterruptible(&g_nand.exclsem);
}
#endif

/****************************************************************************
 * Name: nand_unlock
 *
 * Description:
 *   Relinquish exclusive access to PMECC hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if NAND_NBANKS > 1
static void nand_unlock(void)
{
  nxsem_post(&g_nand.exclsem);
}
#endif

/****************************************************************************
 * Name: nand_wait_ready
 *
 * Description:
 *   Waiting for the completion of a page program, erase and random read
 *   completion.
 *
 * Input Parameters:
 *   priv  Pointer to a sam_nandcs_s instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nand_wait_ready(struct sam_nandcs_s *priv)
{
#ifdef SAMA5_NAND_READYBUSY
  while (board_nand_busy(priv->cs));
#endif
  nand_nfc_cleale(priv, 0, COMMAND_STATUS, 0, 0, 0);
  while ((READ_DATA8(&priv->raw) & STATUS_READY) == 0);
}

/****************************************************************************
 * Name: nand_nfc_cmdsend
 *
 * Description:
 *   Use the HOST NAND FLASH controller to send a command to the NFC.
 *
 * Input Parameters:
 *   priv - Lower-half, private NAND FLASH device state
 *   cmd    - command to send
 *   acycle - address cycle when command access id decoded
 *   cycle0 - address at first cycle
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nand_nfc_cmdsend(struct sam_nandcs_s *priv, uint32_t cmd,
                             uint32_t acycle, uint32_t cycle0)
{
  uintptr_t cmdaddr;

  /* Wait until host controller is not busy. */

  while ((nand_getreg(NFCCMD_BASE + NFCADDR_CMD_NFCCMD) & 0x08000000) != 0);
  nand_setup_cmddone(priv);

  /* Send the command plus the ADDR_CYCLE */

  cmdaddr = NFCCMD_BASE + cmd;
  nand_putreg(SAM_HSMC_ADDR, cycle0);
  nand_putreg(cmdaddr, acycle);

  /* Wait for the command transfer to complete */

  nand_wait_cmddone(priv);
}

/****************************************************************************
 * Name: nand_operation_complete
 *
 * Description:
 *   Check if a program or erase operation completed successfully
 *
 * Input Parameters:
 *   priv - Lower-half, private NAND FLASH device state
 *
 * Returned Value:
 *   OK on success, a negated errnor value on failure
 *
 ****************************************************************************/

static int nand_operation_complete(struct sam_nandcs_s *priv)
{
  uint8_t status;

  nand_nfc_cleale(priv, 0, COMMAND_STATUS, 0, 0, 0);
  status = READ_DATA8(&priv->raw);

  /* On successful completion, the NAND will be READY with no ERROR conditions */

  if ((status & STATUS_ERROR) != 0)
    {
      return -EIO;
    }
  else if ((status & STATUS_READY) == 0)
    {
      return -EBUSY;
    }

  return OK;
}

/****************************************************************************
 * Name: nand_translate_address
 *
 * Description:
 *   Translates the given column and row address into first and other (1-4)
 *   address cycles. The resulting values are stored in the provided
 *   variables if they are not null.
 *
 * Input Parameters:
 *   priv       - Lower-half, private NAND FLASH device state
 *   coladdr    - Column address to translate.
 *   rowaddr    - Row address to translate.
 *   acycle0    - First address cycle
 *   acycle1234 - Four address cycles.
 *   rowonly    - True:Only ROW address is used.
 *
 * Returned Value:
 *   Number of address cycles converted.
 *
 ****************************************************************************/

static int nand_translate_address(struct sam_nandcs_s *priv,
                                  uint16_t coladdr, uint32_t rowaddr,
                                  uint32_t *acycle0, uint32_t *acycle1234,
                                  bool rowonly)
{
  uint16_t maxsize;
  uint32_t maxpage;
  uint32_t accum0;
  uint32_t accum1234;
  uint8_t bytes[8];
  int  ncycles;
  int  ndx;
  int  pos;

  /* Setup */

  maxsize   = nandmodel_getpagesize(&priv->raw.model) +
              nandmodel_getsparesize(&priv->raw.model) - 1;
  maxpage   = nandmodel_getdevpagesize(&priv->raw.model) - 1;
  ncycles   = 0;
  accum0    = 0;
  accum1234 = 0;

  /* Check the data bus width of the NAND FLASH */

  if (nandmodel_getbuswidth(&priv->raw.model) == 16)
    {
      /* Use word vs. bytes addressing */

      coladdr >>= 1;
    }

  /* Convert column address */

  if (!rowonly)
    {
      /* Send single column address byte for small block devices, or two
       * column address bytes for large block devices
       */

      while (maxsize > 2)
        {
          bytes[ncycles++] = coladdr & 0xff;
          maxsize >>= 8;
          coladdr >>= 8;
        }
    }

  /* Convert row address */

  while (maxpage > 0)
    {
      bytes[ncycles++] = rowaddr & 0xff;
      maxpage >>= 8;
      rowaddr >>= 8;
    }

  /* Build acycle0 and acycle1234 */

  ndx = 0;

  /* If more than 4 cycles, acycle0 is used */

  if (ncycles > 4)
    {
      for (pos = 0; ndx < ncycles - 4; ndx++)
        {
          accum0 += bytes[ndx] << pos;
          pos += 8;
        }
    }

  /* acycle1234 */

  for (pos = 0; ndx < ncycles; ndx++)
    {
      accum1234 += bytes[ndx] << pos;
      pos += 8;
    }

  /* Store values */

  if (acycle0)
    {
      *acycle0 = accum0;
    }

  if (acycle1234)
    {
      *acycle1234 = accum1234;
    }

  return ncycles;
}

/****************************************************************************
 * Name: nand_get_acycle
 *
 * Description:
 *   Map the number of address cycles the bit setting for the NFC command
 *
 * Input Parameters:
 *   ncycles    - Number of address cycles
 *
 * Returned Value:
 *   NFC command value
 *
 ****************************************************************************/

static uint32_t nand_get_acycle(int ncycles)
{
  switch (ncycles)
    {
    case 1:
      return NFCADDR_CMD_ACYCLE_ONE;

    case 2:
      return NFCADDR_CMD_ACYCLE_TWO;

    case 3:
      return NFCADDR_CMD_ACYCLE_THREE;

    case 4:
      return NFCADDR_CMD_ACYCLE_FOUR;

    case 5:
      return NFCADDR_CMD_ACYCLE_FIVE;
    }

  return 0;
}

/****************************************************************************
 * Name: nand_nfc_cleale
 *
 * Description:
 *   Sends NAND CLE/ALE command.
 *
 * Input Parameters:
 *   priv    - Pointer to a sam_nandcs_s instance.
 *   mode    - SMC ALE CLE mode parameter.
 *   cmd1    - First command to be sent.
 *   cmd2    - Second command to be sent.
 *   coladdr - Column address.
 *   rowaddr - Row address.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nand_nfc_cleale(struct sam_nandcs_s *priv, uint8_t mode,
                            uint32_t cmd1, uint32_t cmd2,
                            uint32_t coladdr, uint32_t rowaddr)
{
  uint32_t cmd;
  uint32_t regval;
  uint32_t rw;
  uint32_t acycle;
  uint32_t acycle0 = 0;
  uint32_t acycle1234 = 0;
  int ncycles;

  if ((mode & HSMC_CLE_WRITE_EN) != 0)
    {
      rw = NFCADDR_CMD_NFCWR;
    }
  else
    {
      rw = NFCADDR_CMD_NFCRD;
    }

  if ((mode & HSMC_CLE_DATA_EN) != 0)
    {
      regval = NFCADDR_CMD_DATAEN;
    }
  else
    {
      regval = NFCADDR_CMD_DATADIS;
    }

  if (((mode & HSMC_ALE_COL_EN) != 0) || ((mode & HSMC_ALE_ROW_EN) != 0))
    {
      bool rowonly = ((mode & HSMC_ALE_COL_EN) == 0);
      ncycles = nand_translate_address(priv, coladdr, rowaddr,
                                       &acycle0, &acycle1234, rowonly);
      acycle  = nand_get_acycle(ncycles);
    }
  else
    {
      acycle  = NFCADDR_CMD_ACYCLE_NONE;
    }

  cmd = (rw | regval | NFCADDR_CMD_CSID(priv->cs) | acycle |
         (((mode & HSMC_CLE_VCMD2_EN) == HSMC_CLE_VCMD2_EN) ?
         NFCADDR_CMD_VCMD2 : 0) |
         (cmd1 << NFCADDR_CMD_CMD1_SHIFT) |
         (cmd2 << NFCADDR_CMD_CMD2_SHIFT));

  nand_nfc_cmdsend(priv, cmd, acycle1234, acycle0);
}

/****************************************************************************
 * Name: nand_wait_cmddone
 *
 * Description:
 *   Wait for NFC command done
 *
 * Input Parameters:
 *   priv - CS state structure instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nand_wait_cmddone(struct sam_nandcs_s *priv)
{
#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
  irqstate_t flags;

  /* Wait for the CMDDONE interrupt to occur */

  flags = enter_critical_section();
  do
    {
      nxsem_wait_uninterruptible(&g_nand.waitsem);
    }
  while (!g_nand.cmddone);

  /* CMDDONE received */

  g_nand.cmddone = false;
  leave_critical_section(flags);

#else
  /* Poll for the CMDDONE event (latching other events as necessary) */

  do
    {
      nand_nfc_poll();
    }
  while (!g_nand.cmddone);
#endif
}

/****************************************************************************
 * Name: nand_setup_cmddone
 *
 * Description:
 *   Setup to wait for CMDDONE event
 *
 * Input Parameters:
 *   priv - CS state structure instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nand_setup_cmddone(struct sam_nandcs_s *priv)
{
#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
  irqstate_t flags;

  /* Clear all pending interrupts.  This must be done with interrupts
   * enabled or we could lose interrupts.
   */

  nand_getreg(SAM_HSMC_SR);
  flags = enter_critical_section();

  /* Mark CMDDONE not received */

  g_nand.cmddone = false;

  /* Enable the CMDDONE interrupt */

  nand_putreg(SAM_HSMC_IER, HSMC_NFCINT_CMDDONE);
  leave_critical_section(flags);
#else
  /* Just sample and clear any pending NFC status, then clear CMDDONE status */

  nand_nfc_poll();
  g_nand.cmddone = false;
#endif
}

/****************************************************************************
 * Name: nand_wait_xfrdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 * Input Parameters:
 *   priv - CS state structure instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nand_wait_xfrdone(struct sam_nandcs_s *priv)
{
#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
  irqstate_t flags;

  /* Wait for the XFRDONE interrupt to occur */

  flags = enter_critical_section();
  do
    {
      nxsem_wait_uninterruptible(&g_nand.waitsem);
    }
  while (!g_nand.xfrdone);

  /* XFRDONE received */

  g_nand.xfrdone = false;
  leave_critical_section(flags);

#else
  /* Poll for the XFRDONE event (latching other events as necessary) */

  do
    {
      nand_nfc_poll();
    }
  while (!g_nand.xfrdone);
#endif
}

/****************************************************************************
 * Name: nand_setup_xfrdone
 *
 * Description:
 *   Setup to wait for XFDONE event
 *
 * Input Parameters:
 *   priv - CS state structure instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nand_setup_xfrdone(struct sam_nandcs_s *priv)
{
#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
  irqstate_t flags;

  /* Clear all pending interrupts.  This must be done with interrupts
   * enabled or we could lose interrupts.
   */

  nand_getreg(SAM_HSMC_SR);
  flags = enter_critical_section();

  /* Mark XFRDONE not received */

  g_nand.xfrdone = false;

  /* Enable the XFRDONE interrupt */

  nand_putreg(SAM_HSMC_IER, HSMC_NFCINT_XFRDONE);
  leave_critical_section(flags);
#else
  /* Just sample and clear any pending NFC status, then clear XFRDONE status */

  nand_nfc_poll();
  g_nand.xfrdone = false;
#endif
}

/****************************************************************************
 * Name: nand_wait_rbedge
 *
 * Description:
 *   Wait for read/busy edge detection
 *
 * Input Parameters:
 *   priv - CS state structure instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nand_wait_rbedge(struct sam_nandcs_s *priv)
{
#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
  irqstate_t flags;

  /* Wait for the RBEDGE0 interrupt to occur */

  flags = enter_critical_section();
  do
    {
      nxsem_wait_uninterruptible(&g_nand.waitsem);
    }
  while (!g_nand.rbedge);

  /* RBEDGE0 received */

  g_nand.rbedge = false;
  leave_critical_section(flags);

#else
  /* Poll for the RBEDGE0 event (latching other events as necessary) */

  do
    {
      nand_nfc_poll();
    }
  while (!g_nand.rbedge);
#endif
}

/****************************************************************************
 * Name: nand_setup_rbedge
 *
 * Description:
 *   Setup to wait for RBEDGE0 event
 *
 * Input Parameters:
 *   priv - CS state structure instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nand_setup_rbedge(struct sam_nandcs_s *priv)
{
#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
  irqstate_t flags;

  /* Clear all pending interrupts.  This must be done with interrupts
   * enabled or we could lose interrupts.
   */

  nand_getreg(SAM_HSMC_SR);
  flags = enter_critical_section();

  /* Mark RBEDGE0 not received */

  g_nand.rbedge = false;

  /* Enable the RBEDGE0 interrupt */

  nand_putreg(SAM_HSMC_IER, HSMC_NFCINT_RBEDGE0);
  leave_critical_section(flags);
#else
  /* Just sample and clear any pending NFC status, then clear RBEDGE0 status */

  nand_nfc_poll();
  g_nand.rbedge = false;
#endif
}

/****************************************************************************
 * Name: nand_wait_nfcbusy
 *
 * Description:
 *   Wait for NFC not busy
 *
 * Input Parameters:
 *   priv - CS state structure instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if 0 /* Not used */
static void nand_wait_nfcbusy(struct sam_nandcs_s *priv)
{
  uint32_t sr;

  /* Poll for the NFC not busy state (latching other events as necessary) */

  do
    {
      sr = nand_nfc_poll();
    }
  while ((sr & HSMC_SR_NFCBUSY) != 0);
}
#endif

/****************************************************************************
 * Name: nand_nfc_poll
 *
 * Description:
 *   Sample, latch, and return NFC status.  Some pending status is cleared.
 *   This latching capability function is needed to prevent loss of pending
 *   status when sampling the HSMC_SR register.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Current HSMC_SR register value;
 *
 ****************************************************************************/

static uint32_t nand_nfc_poll(void)
{
  uint32_t sr;
#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
  irqstate_t flags;

  /* Disable interrupts while we sample NFS status as this may be done from
   * the interrupt level as well.
   */

  flags = enter_critical_section();
#endif

  /* Read the current HSMC status, clearing most pending conditions */

  sr = nand_getreg(SAM_HSMC_SR);

  /* When set to one, this XFRDONE indicates that the NFC has terminated
   * the data transfer. This flag is reset after the status read.
   */

  if ((sr & HSMC_NFCINT_XFRDONE) != 0)
    {
      /* Set the latching XFRDONE status */

      g_nand.xfrdone = true;
    }

  /* When set to one, the CMDDONE flag indicates that the NFC has terminated
   * the Command. This flag is reset after the status read.
   */

  if ((sr & HSMC_NFCINT_CMDDONE) != 0)
    {
      /* Set the latching CMDDONE status */

      g_nand.cmddone = true;
    }

  /* If set to one, the RBEDGE0 flag indicates that an edge has been
   * detected on the Ready/Busy Line x. Depending on the EDGE CTRL field
   * located in the SMC_CFG register, only rising or falling edge is
   * detected. This flag is reset after the status read.
   */

  if ((sr & HSMC_NFCINT_RBEDGE0) != 0)
    {
      /* Set the latching RBEDGE0 status */

      g_nand.rbedge = true;
    }

#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
  leave_critical_section(flags);
#endif
  return sr;
}

/****************************************************************************
 * Name: hsmc_interrupt
 *
 * Description:
 *   HSMC interrupt handler
 *
 * Input Parameters:
 *   Standard interrupt arguments
 *
 * Returned Value:
 *   Always returns OK
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
static int hsmc_interrupt(int irq, void *context, FAR void *arg)
{
  uint32_t sr      = nand_nfc_poll();
  uint32_t imr     = nand_getreg(SAM_HSMC_IMR);
  uint32_t pending = sr & imr;

#ifndef CONFIG_SAMA5_NAND_REGDEBUG
  finfo("sr=%08x imr=%08x\n", sr, imr);
#endif

  /* When set to one, this XFRDONE indicates that the NFC has terminated
   * the data transfer. This flag is reset after the status read.
   */

  if ((g_nand.xfrdone && (imr & HSMC_NFCINT_XFRDONE) != 0)
    {
      /* Post the XFRDONE event */

      nxsem_post(&g_nand.waitsem);

      /* Disable further XFRDONE interrupts */

      nand_putreg(SAM_HSMC_IDR, HSMC_NFCINT_XFRDONE);
    }

  /* When set to one, the CMDDONE flag indicates that the NFC has terminated
   * the Command. This flag is reset after the status read.
   */

  if (g_nand.xfrdone && (imr & HSMC_NFCINT_CMDDONE) != 0)
    {
      /* Post the CMDDONE event */

      nxsem_post(&g_nand.waitsem);

      /* Disable further CMDDONE interrupts */

      nand_putreg(SAM_HSMC_IDR, HSMC_NFCINT_CMDDONE);
    }

  /* If set to one, the RBEDGE0 flag indicates that an edge has been
   * detected on the Ready/Busy Line x. Depending on the EDGE CTRL field
   * located in the SMC_CFG register, only rising or falling edge is
   * detected. This flag is reset after the status read.
   */

  if (g_nand.rbedge && (imr & HSMC_NFCINT_RBEDGE0) != 0)
    {
      /* Post the RBEDGE0 event */

      nxsem_post(&g_nand.waitsem);

      /* Disable further RBEDGE0 interrupts */

      nand_putreg(SAM_HSMC_IDR, HSMC_NFCINT_RBEDGE0);
    }

  return OK;
}
#endif /* CONFIG_SAMA5_NAND_HSMCINTERRUPTS */

/****************************************************************************
 * Name: nand_dma_sampleinit
 *
 * Description:
 *   Initialize sampling of DMA registers (if CONFIG_SAMA5_NAND_DMADEBUG)
 *
 * Input Parameters:
 *   priv - Lower-half, private NAND FLASH device state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_DMADEBUG
static void nand_dma_sampleinit(struct sam_nandcs_s *priv)
{
  /* Put contents of register samples into a known state */

  memset(priv->dmaregs, 0xff, DMA_NSAMPLES * sizeof(struct sam_dmaregs_s));

  /* Then get the initial samples */

  sam_dmasample(priv->dma, &priv->dmaregs[DMA_INITIAL]);
}
#endif

/****************************************************************************
 * Name: nand_dma_sampledone
 *
 * Description:
 *   Dump sampled RX DMA registers
 *
 * Input Parameters:
 *   priv - Lower-half, private NAND FLASH device state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_DMADEBUG
static void nand_dma_sampledone(struct sam_nandcs_s *priv, int result)
{
  finfo("result: %d\n", result);

  /* Sample the final registers */

  sam_dmasample(priv->dma, &priv->dmaregs[DMA_END_TRANSFER]);

  /* Then dump the sampled DMA registers */

  /* Initial register values */

  sam_dmadump(priv->dma, &priv->dmaregs[DMA_INITIAL],
              "Initial Registers");

  /* Register values after DMA setup */

  sam_dmadump(priv->dma, &priv->dmaregs[DMA_AFTER_SETUP],
              "After DMA Setup");

  /* Register values after DMA start */

  sam_dmadump(priv->dma, &priv->dmaregs[DMA_AFTER_START],
              "After DMA Start");

  /* Register values at the time of the TX and RX DMA callbacks
   * -OR- DMA timeout.
   *
   * If the DMA timedout, then there will not be any RX DMA
   * callback samples.  There is probably no TX DMA callback
   * samples either, but we don't know for sure.
   */

#if 0 /* No timeout */
  if (result == -ETIMEDOUT || result == -EINTR)
    {
      sam_dmadump(priv->dma, &priv->dmaregs[DMA_TIMEOUT],
                  "At DMA timeout");
    }
  else
#endif
    {
      sam_dmadump(priv->dma, &priv->dmaregs[DMA_CALLBACK],
                  "At DMA callback");
    }

  sam_dmadump(priv->dma, &priv->dmaregs[DMA_END_TRANSFER],
              "At End-of-Transfer");
}
#endif

/****************************************************************************
 * Name: nand_wait_dma
 *
 * Description:
 *   Wait for the completion of a DMA transfer
 *
 * Input Parameters:
 *   Wait for read/busy edge detection
 *
 * Returned Value:
 *   The result of the DMA.  OK on success; a negated ernno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_DMA
static int nand_wait_dma(struct sam_nandcs_s *priv)
{
  while (!priv->dmadone)
    {
      nxsem_wait_uninterruptible(&priv->waitsem);
    }

  finfo("Awakened: result=%d\n", priv->result);
  priv->dmadone = false;
  return priv->result;
}
#endif

/****************************************************************************
 * Name: sam_adc_dmacallback
 *
 * Description:
 *   Called when one NAND DMA sequence completes.  This function just wakes
 *   the waiting NAND driver logic.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_DMA
static void nand_dmacallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)arg;

  DEBUGASSERT(priv);
  nand_dma_sample(priv, DMA_CALLBACK);

  /* Wake up the thread that is waiting for the DMA result */

  priv->result  = result;
  priv->dmadone = true;
  nxsem_post(&priv->waitsem);
}
#endif

/****************************************************************************
 * Name: nand_dma_read
 *
 * Description:
 *   Transfer data to NAND from the provided buffer via DMA.
 *
 * Input Parameters:
 *   priv     - Lower-half, private NAND FLASH device state
 *   vsrc     - NAND data destination address.
 *   vdest    - Buffer where data read from NAND will be returned.
 *   nbytes   - The number of bytes to transfer
 *   dmaflags - Describes the DMA configuration
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_DMA
static int nand_dma_read(struct sam_nandcs_s *priv,
                         uintptr_t vsrc, uintptr_t vdest, size_t nbytes,
                         uint32_t dmaflags)
{
  uint32_t psrc;
  uint32_t pdest;
  int ret;

  DEBUGASSERT(priv->dma);

  finfo("vsrc=%08x vdest=%08x nbytes=%d\n",
        (int)vsrc, (int)vdest, (int)nbytes);

  /* Initialize sampling */

  nand_dma_sampleinit(priv);

  /* Invalidate the destination memory buffer before performing the DMA (so
   * that nothing gets flushed later, corrupting the DMA transfer, and so
   * that memory will be re-cached after the DMA completes).
   */

  up_invalidate_dcache(vdest, vdest + nbytes);

  /* DMA will need physical addresses. */

  psrc  = sam_physregaddr(vsrc);   /* Source is NAND */
  pdest = sam_physramaddr(vdest);  /* Destination is normal memory */

  /* Configure the DMA:  8- vs 16-bit, NFC SRAM or NAND */

  sam_dmaconfig(priv->dma, dmaflags);

  /* Setup the Memory-to-Memory DMA.  The semantics of the DMA module are
   * awkward here.  We will treat the NAND (src) as the peripheral source
   * and memory as the destination.  Internally, the DMA module will realize
   * that this is a memory to memory transfer and should do the right thing.
   */

  ret = sam_dmarxsetup(priv->dma, psrc, pdest, nbytes);
  if (ret < 0)
    {
      ferr("ERROR: sam_dmarxsetup failed: %d\n", ret);
      return ret;
    }

  nand_dma_sample(priv, DMA_AFTER_SETUP);

  /* Start the DMA */

  priv->dmadone = false;
  priv->result  = -EBUSY;

  sam_dmastart(priv->dma, nand_dmacallback, priv);
  nand_dma_sample(priv, DMA_AFTER_START);

  /* Wait for the DMA to complete */

  ret = nand_wait_dma(priv);
  if (ret < 0)
    {
      ferr("ERROR: DMA failed: %d\n", ret);
    }

  nand_dma_sample(priv, DMA_END_TRANSFER);
  nand_dma_sampledone(priv, ret);
  return ret;
}
#endif

/****************************************************************************
 * Name: nand_dma_write
 *
 * Description:
 *   Transfer data to NAND from the provided buffer via DMA.
 *
 * Input Parameters:
 *   priv     - Lower-half, private NAND FLASH device state
 *   vsrc     - Buffer that provides the data for the write
 *   vdest    - NAND data destination address
 *   nbytes   - The number of bytes to transfer
 *   dmaflags - Describes the DMA configuration
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_DMA
static int nand_dma_write(struct sam_nandcs_s *priv,
                          uintptr_t vsrc, uintptr_t vdest, size_t nbytes,
                          uint32_t dmaflags)
{
  uint32_t psrc;
  uint32_t pdest;
  int ret;

  DEBUGASSERT(priv->dma);

  /* Initialize sampling */

  nand_dma_sampleinit(priv);

  /* Clean the D-Cache associated with the source data buffer so that all of
   * the data to be transferred lies in physical memory
   */

  up_clean_dcache(vsrc, vsrc + nbytes);

  /* DMA will need physical addresses. */

  psrc  = sam_physramaddr(vsrc);   /* Source is normal memory */
  pdest = sam_physregaddr(vdest);  /* Destination is NAND (or NAND host SRAM) */

  /* Configure the DMA:  8- vs 16-bit, NFC SRAM or NAND */

  sam_dmaconfig(priv->dma, dmaflags);

  /* Setup the Memory-to-Memory DMA.  The semantics of the DMA module are
   * awkward here.  We will treat the NAND (dest) as the peripheral
   * destination and memory as the source.  Internally, the DMA module will
   * realize that this is a memory to memory transfer and should do the
   * right thing.
   */

  ret = sam_dmatxsetup(priv->dma, pdest, psrc, nbytes);
  if (ret < 0)
    {
      ferr("ERROR: sam_dmatxsetup failed: %d\n", ret);
      return ret;
    }

  nand_dma_sample(priv, DMA_AFTER_SETUP);

  /* Start the DMA */

  priv->dmadone = false;
  priv->result  = -EBUSY;

  sam_dmastart(priv->dma, nand_dmacallback, priv);
  nand_dma_sample(priv, DMA_AFTER_START);

  /* Wait for the DMA to complete */

  ret = nand_wait_dma(priv);
  if (ret < 0)
    {
      ferr("ERROR: DMA failed: %d\n", ret);
    }

  nand_dma_sample(priv, DMA_END_TRANSFER);
  nand_dma_sampledone(priv, ret);
  return ret;
}
#endif

/****************************************************************************
 * Name: nand_nfcsram_read
 *
 * Description:
 *   Read data from NAND using the NFC SRAM
 *
 * Input Parameters:
 *   priv     - Lower-half, private NAND FLASH device state
 *   buffer   - Buffer that provides the data for the write
 *   buflen   - The amount of data to read into the buffer
 *   offset   - If reading from NFC SRAM, this is the offset into
 *              the SRAM.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nand_nfcsram_read(struct sam_nandcs_s *priv, uint8_t *buffer,
                             uint16_t buflen, uint16_t offset)
{
  uintptr_t src;
  int remaining;
  int ret;

  finfo("buffer=%p buflen=%d\n", buffer, buflen);

  /* Get the offset data source address */

  src = NFCSRAM_BASE + (uintptr_t)offset;

#ifdef CONFIG_SAMA5_NAND_DMA
  /* Then perform the transfer via memory-to-memory DMA or not, depending
   * on if we have a DMA channel assigned and if the transfer is
   * sufficiently large.  Small DMAs (e.g., for spare data) are not performed
   * because the DMA context switch can take more time that the DMA itself.
   */

  if (priv->dma && buflen > CONFIG_SAMA5_NAND_DMA_THRESHOLD)
    {
      DEBUGASSERT(((uintptr_t)buffer & 3) == 0 && ((uintptr_t)src & 3) == 0);

      /* Transfer using DMA */

      ret = nand_dma_read(priv, src, (uintptr_t)buffer, buflen,
                          NFCSRAM_DMA_FLAGS);
    }
  else
#endif

  /* Transfer without DMA */

    {
      uint8_t *src8 = (uint8_t *)src;
      uint8_t *dest8 = buffer;

      for (remaining = buflen; remaining > 0; remaining--)
        {
          *dest8++ = *src8++;
        }

      ret = OK;
    }

  nand_dump("NFS SRAM Read", buffer, buflen);
  return ret;
}

/****************************************************************************
 * Name: nand_read
 *
 * Description:
 *   Read data directly from the NAND data address.  Currently this only
 *   used by the PMECC logic which I could not get working if I read from
 *   NFC SRAM.
 *
 * Input Parameters:
 *   priv     - Lower-half, private NAND FLASH device state
 *   nfcsram  - True: Use NFC Host SRAM
 *   buffer   - Buffer that provides the data for the write
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_HAVE_PMECC
static int nand_read(struct sam_nandcs_s *priv, uint8_t *buffer,
                     uint16_t buflen)
{
  int remaining;
  int buswidth;
  int ret;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Get the buswidth */

  buswidth = nandmodel_getbuswidth(&priv->raw.model);

#ifdef CONFIG_SAMA5_NAND_DMA
  /* Then perform the transfer via memory-to-memory DMA or not, depending
   * on if we have a DMA channel assigned and if the transfer is
   * sufficiently large.  Small DMAs (e.g., for spare data) are not performed
   * because the DMA context switch can take more time that the DMA itself.
   */

  if (priv->dma && buflen > CONFIG_SAMA5_NAND_DMA_THRESHOLD)
    {
      /* Select NFC DATA DMA */

      uint32_t dmaflags =
        (buswidth == 16 ? NAND_DMA_FLAGS16 : NAND_DMA_FLAGS8);

      /* Transfer using DMA */

      ret = nand_dma_read(priv, priv->raw.dataaddr, (uintptr_t)buffer,
                          buflen, dmaflags);
    }
  else
#endif

  /* Transfer without DMA */

    {
      /* Check the data bus width of the NAND FLASH */

      remaining = buflen;
      if (buswidth == 16)
        {
          volatile uint16_t *src16  =
            (volatile uint16_t *)priv->raw.dataaddr;
          uint16_t *dest16 = (uint16_t *)buffer;

          DEBUGASSERT(((uintptr_t)buffer & 1) == 0);

          for (; remaining > 1; remaining -= sizeof(uint16_t))
            {
              *dest16++ = *src16;
            }
        }
      else
        {
          volatile uint8_t *src8  = (volatile uint8_t *)priv->raw.dataaddr;
          uint8_t *dest8 = (uint8_t *)buffer;

          for (; remaining > 0; remaining--)
            {
              *dest8++ = *src8;
            }
        }

      ret = OK;
    }

  nand_dump("NAND Read", buffer, buflen);
  return ret;
}
#endif

/****************************************************************************
 * Name: nand_read_pmecc
 *
 * Description:
 *   Reads the data area of a page of a NAND FLASH into the provided buffer.
 *
 * Input Parameters:
 *   priv   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_HAVE_PMECC
static int nand_read_pmecc(struct sam_nandcs_s *priv, off_t block,
                           unsigned int page, void *data)
{
  uint32_t rowaddr;
  uint32_t regval;
  uint16_t pagesize;
  uint16_t sparesize;
  int ret;

  finfo("block=%d page=%d data=%p\n", (int)block, page, data);
  DEBUGASSERT(priv && data);

  /* Get page and spare sizes */

  pagesize  = nandmodel_getpagesize(&priv->raw.model);
  sparesize = nandmodel_getsparesize(&priv->raw.model);

  /* Convert the page size to something understood by the hardware */

  switch (pagesize)
    {
    case 512:
      regval = HSMC_CFG_PAGESIZE_512;
      break;

    case 1024:
      regval = HSMC_CFG_PAGESIZE_1024;
      break;

    case 2048:
      regval = HSMC_CFG_PAGESIZE_2048;
      break;

    case 4096:
      regval = HSMC_CFG_PAGESIZE_4096;
      break;

    case 8192:
      regval = HSMC_CFG_PAGESIZE_8192;
      break;

    default:
      ferr("ERROR:  Unsupported page size: %d\n", pagesize);
      return -EINVAL;
    }

  /* Configure the SMC */

  regval |= (HSMC_CFG_RSPARE | HSMC_CFG_RBEDGE | HSMC_CFG_DTOCYC(15) |
             HSMC_CFG_DTOMUL_1048576 |
             HSMC_CFG_NFCSPARESIZE((sparesize - 1) >> 2));
  nand_putreg(SAM_HSMC_CFG, regval);

  /* Calculate actual address of the page */

  rowaddr = block * nandmodel_pagesperblock(&priv->raw.model) + page;

  /* Reset and enable the PMECC */

  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_RST);
  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_ENABLE);

  regval = nand_getreg(SAM_HSMC_PMECCFG);
  if ((regval & HSMC_PMECCFG_SPAREEN_MASK) == HSMC_PMECCFG_SPARE_DISABLE)
    {
      regval |= HSMC_PMECCFG_AUTO_ENABLE;
    }

  nand_putreg(SAM_HSMC_PMECCFG, regval);

  /* Start the data phase and perform the transfer */

  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_DATA);

#if 0 /* Don't use NFC SRAM */
  nand_nfc_cleale(priv,
                  HSMC_ALE_COL_EN | HSMC_ALE_ROW_EN | HSMC_CLE_VCMD2_EN |
                  HSMC_CLE_DATA_EN,
                  COMMAND_READ_1, COMMAND_READ_2, 0, rowaddr);
#else
  nand_setup_rbedge(priv);
  nand_nfc_cleale(priv,
                  HSMC_ALE_COL_EN | HSMC_ALE_ROW_EN | HSMC_CLE_VCMD2_EN,
                  COMMAND_READ_1, COMMAND_READ_2, 0, rowaddr);
  nand_wait_rbedge(priv);
#endif

  /* Reset the PMECC module */

  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_RST);

  /* Start a Data Phase */

  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_DATA);

  /* Read the data area into the caller provided buffer (pagesize bytes).
   * NOTE: NFC SRAM is not used.  In that case, the wait for PMECC not
   * busy below would hang.
   */

#if 0 /* Don't use NFC SRAM */
  ret = nand_nfcsram_read(priv, (uint8_t *)data, pagesize, 0);
#else
  ret = nand_read(priv, (uint8_t *)data, pagesize);
#endif
  if (ret < 0)
    {
      ferr("ERROR: nand_read for data region failed: %d\n", ret);
      return ret;
    }

  /* Now read the spare area into priv->raw.spare (sparesize bytes). */

#if 0 /* Don't use NFC SRAM */
  ret = nand_nfcsram_read(priv, priv->raw.spare, sparesize, pagesize);
#else
  ret = nand_read(priv, priv->raw.spare, sparesize);
#endif
  if (ret < 0)
    {
      ferr("ERROR: nand_read for spare region failed: %d\n", ret);
      return ret;
    }

  /* Wait until the kernel of the PMECC is not busy */

  while ((nand_getreg(SAM_HSMC_PMECCSR) & HSMC_PMECCSR_BUSY) != 0);
  return OK;
}
#endif

/****************************************************************************
 * Name: nand_nfcsram_write
 *
 * Description:
 *   Write data to NAND using NAND NFC SRAM
 *
 * Input Parameters:
 *   priv     - Lower-half, private NAND FLASH device state
 *   buffer   - Buffer that provides the data for the write
 *   offset   - Data offset in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nand_nfcsram_write(struct sam_nandcs_s *priv, uint8_t *buffer,
                              uint16_t buflen, uint16_t offset)
{
  uintptr_t dest;
  int ret;

  finfo("buffer=%p buflen=%d offset=%d\n", buffer, buflen, offset);
  nand_dump("NFC SRAM Write", buffer, buflen);

  /* Apply the offset to the destination address */

  dest = NFCSRAM_BASE + offset;

#ifdef CONFIG_SAMA5_NAND_DMA
  /* Then perform the transfer via memory-to-memory DMA or not, depending
   * on if we have a DMA channel assigned and if the transfer is
   * sufficiently large.  Small DMAs (e.g., for spare data) are not performed
   * because the DMA context switch can take more time that the DMA itself.
   */

  if (priv->dma && buflen > CONFIG_SAMA5_NAND_DMA_THRESHOLD)
    {
      DEBUGASSERT(((uintptr_t)buffer & 3) == 0 &&
                  ((uintptr_t)dest & 3) == 0);

      /* Transfer using DMA */

      ret = nand_dma_write(priv, (uintptr_t)buffer, dest, buflen,
                           NFCSRAM_DMA_FLAGS);
    }
  else
#endif

  /* Transfer without DMA */

    {
      uint8_t *dest8 = (uint8_t *)dest;

      for (; buflen > 0; buflen--)
        {
          *dest8++ = *buffer++;
        }

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: nand_write
 *
 * Description:
 *   Write data to NAND using the NAND data address.
 *
 * Input Parameters:
 *   priv     - Lower-half, private NAND FLASH device state
 *   buffer   - Buffer that provides the data for the write
 *   offset   - Data offset in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nand_write(struct sam_nandcs_s *priv, uint8_t *buffer,
                      uint16_t buflen, uint16_t offset)
{
  uintptr_t dest;
  int buswidth;
  int ret;

  finfo("buffer=%p buflen=%d offset=%d\n", buffer, buflen, offset);
  nand_dump("NAND Write", buffer, buflen);

  /* Apply the offset to the destination address */

  dest =  priv->raw.dataaddr + offset;

  /* Get the buswidth */

  buswidth = nandmodel_getbuswidth(&priv->raw.model);

#ifdef CONFIG_SAMA5_NAND_DMA
  /* Then perform the transfer via memory-to-memory DMA or not, depending
   * on if we have a DMA channel assigned and if the transfer is
   * sufficiently large.  Small DMAs (e.g., for spare data) are not performed
   * because the DMA context switch can take more time that the DMA itself.
   */

  if (priv->dma && buflen > CONFIG_SAMA5_NAND_DMA_THRESHOLD)
    {
      /* Select NFC DATA DMA */

      uint32_t dmaflags =
        (buswidth == 16 ? NAND_DMA_FLAGS16 : NAND_DMA_FLAGS8);

      /* Transfer using DMA */

      ret = nand_dma_write(priv, (uintptr_t)buffer, dest, buflen, dmaflags);
    }
  else
#endif

  /* Transfer without DMA */

    {
      /* Check the data bus width of the NAND FLASH */

      if (buswidth == 16)
        {
          volatile uint16_t *dest16  = (volatile uint16_t *)dest;
          const uint16_t *src16 = (const uint16_t *)buffer;

          DEBUGASSERT(((uintptr_t)buffer & 1) == 0);

          for (; buflen > 1; buflen -=  sizeof(uint16_t))
            {
              *dest16 = *src16++;
            }
        }
      else
        {
          volatile uint8_t *dest8  = (volatile uint8_t *)dest;

          for (; buflen > 0; buflen--)
            {
              *dest8 = *buffer++;
            }
        }

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: nand_readpage_noecc
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.  The raw NAND contents are returned with no ECC
 *   corrections.
 *
 * Input Parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   spare - Buffer where the spare area will be stored.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_readpage_noecc(struct sam_nandcs_s *priv, off_t block,
             unsigned int page, void *data, void *spare)
{
  uint32_t regval;
  uint16_t pagesize;
  uint16_t sparesize;
  off_t rowaddr;
  off_t coladdr;
  int ret;

  finfo("block=%d page=%d data=%p spare=%p\n",
        (int)block, page, data, spare);
  DEBUGASSERT(priv && (data || spare));

  /* Get page and spare sizes */

  pagesize  = nandmodel_getpagesize(&priv->raw.model);
  sparesize = nandmodel_getsparesize(&priv->raw.model);

  /* Convert the page size to something understood by the hardware */

  switch (pagesize)
    {
    case 512:
      regval = HSMC_CFG_PAGESIZE_512;
      break;

    case 1024:
      regval = HSMC_CFG_PAGESIZE_1024;
      break;

    case 2048:
      regval = HSMC_CFG_PAGESIZE_2048;
      break;

    case 4096:
      regval = HSMC_CFG_PAGESIZE_4096;
      break;

    case 8192:
      regval = HSMC_CFG_PAGESIZE_8192;
      break;

    default:
      ferr("ERROR:  Unsupported page size: %d\n", pagesize);
      return -EINVAL;
    }

  /* Configure the SMC */

  regval |= HSMC_CFG_RBEDGE | HSMC_CFG_DTOCYC(15) |
            HSMC_CFG_DTOMUL_1048576 |
            HSMC_CFG_NFCSPARESIZE((sparesize - 1) >> 2);
  nand_putreg(SAM_HSMC_CFG, regval);

  /* Calculate actual address of the page */

  rowaddr = block * nandmodel_pagesperblock(&priv->raw.model) + page;
  coladdr = data ? 0 : pagesize;

  /* Initialize the NFC */

  nand_setup_xfrdone(priv);
  nand_nfc_cleale(priv,
                  HSMC_ALE_COL_EN | HSMC_ALE_ROW_EN | HSMC_CLE_VCMD2_EN |
                  HSMC_CLE_DATA_EN,
                  COMMAND_READ_1, COMMAND_READ_2, coladdr, rowaddr);
  nand_wait_xfrdone(priv);

  /* Read data area if so requested */

  if (data)
    {
      ret = nand_nfcsram_read(priv, (uint8_t *)data, pagesize, 0);
      if (ret < 0)
        {
          ferr("ERROR: nand_nfcsram_read for data region failed: %d\n", ret);
          return ret;
        }
    }

  /* Read the spare area if so requested.  If there is no data, then the
   * spare data will appear at offset 0; If there is data, thenthe spare data
   * will appear following the data at offset pagesize.
   */

  if (spare)
    {
      uint16_t offset = data ? pagesize : 0;
      ret = nand_nfcsram_read(priv, (uint8_t *)spare, sparesize, offset);
      if (ret < 0)
        {
          ferr("ERROR: nand_nfcsram_read for spare region failed: %d\n",
               ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nand_readpage_pmecc
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.  PMECC is used
 *
 * Input Parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_HAVE_PMECC
static int nand_readpage_pmecc(struct sam_nandcs_s *priv, off_t block,
                               unsigned int page, void *data)
{
  uint32_t regval;
  uint16_t sparesize;
  int ret;
  int i;

  finfo("block=%d page=%d data=%p\n", (int)block, page, data);
  DEBUGASSERT(priv && data);

  /* Make sure that we have exclusive access to the PMECC and that the PMECC
   * is properly configured for this CS.
   */

  ret = pmecc_lock();
  if (ret < 0)
    {
      return ret;
    }

  ret = pmecc_configure(priv, false);
  if (ret < 0)
    {
      ferr("ERROR: pmecc_configure failed: %d\n", ret);
      goto errout;
    }

  /* Read page data into the user data buffer and spared data
   * into the priv->raw.spare buffer.
   */

  ret = nand_read_pmecc(priv, block, page, data);
  if (ret < 0)
    {
      ferr("ERROR: Block %d page %d Failed to read page\n",
           block, page, ret);
      goto errout;
    }

  /* Check if any sector is corrupted */

  regval = nand_getreg(SAM_HSMC_PMECCISR);
  if (regval)
    {
      /* Bad sectors.  Check if this is because spare area has been erased */

      /* First, re-read the spare area.  REVISIT:  Is this necessary? */

      ret = nand_readpage_noecc(priv, block, page, NULL, priv->raw.spare);
      if (ret < 0)
        {
          ferr("ERROR: Block %d page %d Failed to re-read spare area: %d\n",
               block, page, ret);
          goto errout;
        }

      /* Then check if all bytes are in the erased state */

      sparesize = nandmodel_getsparesize(&priv->raw.model);
      for (i = 0 ; i < sparesize; i++)
        {
          if (priv->raw.spare[i] != 0xff)
            {
              break;
            }
        }

      /* Has the spare area has been erased? */

      if (i >= sparesize)
        {
          /* Yes.. clear sector errors */

          finfo("Block=%d page=%d has been erased: %08x\n",
               block, page, regval);
          regval = 0;
        }
      else
        {
          ferr("ERROR: block=%d page=%d Corrupted sectors: %08x\n",
               block, page, regval);
        }
    }

  /* Bit correction will be done directly in destination buffer. */

  ret = pmecc_correction(regval, (uintptr_t)data);
  if (ret < 0)
    {
      ferr("ERROR: block=%d page=%d Unrecoverable data error: %d\n",
           block, page, ret);
    }

  /* Disable auto mode */

errout:
  regval  = nand_getreg(SAM_HSMC_PMECCFG);
  regval &= ~HSMC_PMECCFG_AUTO_MASK;
  nand_putreg(SAM_HSMC_PMECCFG, regval);

  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_DISABLE);
  pmecc_unlock();
  return ret;
}
#endif /* CONFIG_SAMA5_HAVE_PMECC */

/****************************************************************************
 * Name: nand_writepage_noecc
 *
 * Description:
 *   Writes the data and/or the spare area of a page on a NAND FLASH chip.
 *   No ECC calculations are performed.
 *
 * Input Parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writing
 *   spare - Buffer containing the spare data to be written.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_writepage_noecc(struct sam_nandcs_s *priv, off_t block,
             unsigned int page, const void *data, const void *spare)
{
  uint32_t regval;
  uint16_t pagesize;
  uint16_t sparesize;
  off_t rowaddr;
  int ret = OK;

  finfo("block=%d page=%d data=%p spare=%p\n",
        (int)block, page, data, spare);

  /* Get page and spare sizes */

  pagesize  = nandmodel_getpagesize(&priv->raw.model);
  sparesize = nandmodel_getsparesize(&priv->raw.model);

  /* Convert the page size to something understood by the hardware */

  switch (pagesize)
    {
    case 512:
      regval = HSMC_CFG_PAGESIZE_512;
      break;

    case 1024:
      regval = HSMC_CFG_PAGESIZE_1024;
      break;

    case 2048:
      regval = HSMC_CFG_PAGESIZE_2048;
      break;

    case 4096:
      regval = HSMC_CFG_PAGESIZE_4096;
      break;

    case 8192:
      regval = HSMC_CFG_PAGESIZE_8192;
      break;

    default:
      ferr("ERROR:  Unsupported page size: %d\n", pagesize);
      return -EINVAL;
    }

  /* Configure the SMC */

  regval |= HSMC_CFG_RBEDGE | HSMC_CFG_DTOCYC(15) |
            HSMC_CFG_DTOMUL_1048576 |
            HSMC_CFG_NFCSPARESIZE((sparesize - 1) >> 2);

  if (spare)
    {
      /* Write spare area */

      regval |= HSMC_CFG_WSPARE;
    }

  nand_putreg(SAM_HSMC_CFG, regval);

  /* Calculate physical address of the page */

  rowaddr = block * nandmodel_pagesperblock(&priv->raw.model) + page;

  /* Write the data and, if present, the spare bytes. */

  if (data)
    {
      ret = nand_nfcsram_write(priv, (uint8_t *)data, pagesize, 0);
      if (ret < 0)
        {
          ferr("ERROR: nand_nfcsram_write for data region failed: %d\n",
               ret);
          return ret;
        }

      if (spare)
        {
          ret = nand_nfcsram_write(priv, (uint8_t *)spare, sparesize,
                                   pagesize);
          if (ret < 0)
            {
              ferr("ERROR: nand_nfcsram_write for data region failed: %d\n",
                   ret);
              return ret;
            }
        }
    }

  /* Write data area if needed */

  if (data)
    {
      /* Start a Data Phase */

      nand_setup_xfrdone(priv);
      nand_nfc_cleale(priv,
                      HSMC_CLE_WRITE_EN | HSMC_ALE_COL_EN |
                      HSMC_ALE_ROW_EN | HSMC_CLE_DATA_EN,
                      COMMAND_WRITE_1, 0, 0, rowaddr);
      nand_wait_xfrdone(priv);

      nand_setup_rbedge(priv);
      nand_nfc_cleale(priv, HSMC_CLE_WRITE_EN, COMMAND_WRITE_2, 0, 0, 0);
      nand_wait_rbedge(priv);

      /* Check if the transfer completed successfully */

      ret = nand_operation_complete(priv);
      if (ret < 0)
        {
          ferr("ERROR: Failed writing data area: %d\n", ret);
        }
    }

  /* Write spare area alone if needed */

  else if (spare)
    {
      nand_nfc_cleale(priv,
                      HSMC_CLE_WRITE_EN | HSMC_ALE_COL_EN | HSMC_ALE_ROW_EN,
                      COMMAND_WRITE_1, 0,  pagesize, rowaddr);

      ret = nand_write(priv, (uint8_t *)spare, sparesize, 0);
      if (ret < 0)
        {
          ferr("ERROR: nand_write for spare region failed: %d\n", ret);
          ret = -EPERM;
        }

      nand_nfc_cleale(priv, HSMC_CLE_WRITE_EN, COMMAND_WRITE_2, 0, 0, 0);
      nand_wait_ready(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: nand_writepage_pmecc
 *
 * Description:
 *   Writes the data area of a NAND FLASH page, The PMECC module generates
 *   redundancy at encoding time.  When a NAND write page operation is
 *   performed.  The redundancy is appended to the page and written in the
 *   spare area.
 *
 * Input Parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writing
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_HAVE_PMECC
static int nand_writepage_pmecc(struct sam_nandcs_s *priv, off_t block,
                                unsigned int page, const void *data)
{
  uint32_t regval;
  volatile uint8_t *pmecc;
  uint8_t *ecc;
  unsigned int pagesize;
  unsigned int rowaddr;
  unsigned int eccsaddr;
  unsigned int eccpersector;
  unsigned int sectersperpage;
  unsigned int eccsize;
  unsigned int sector;
  unsigned int i;
  int ret;

  finfo("block=%d page=%d data=%p\n", (int)block, page, data);
  DEBUGASSERT(priv && data);

  /* Make sure that we have exclusive access to the PMECC and that the PMECC
   * is properly configured for this CS.
   */

  ret = pmecc_lock();
  if (ret < 0)
    {
      return ret;
    }

  ret = pmecc_configure(priv, false);
  if (ret < 0)
    {
      ferr("ERROR: pmecc_configure failed: %d\n", ret);
      goto errout;
    }

  /* Calculate the start page address */

  regval   = nand_getreg(SAM_HSMC_PMECCSADDR);
  pagesize = nandmodel_getpagesize(&priv->raw.model);
  eccsaddr = pagesize + regval;

  /* Calculate physical address of the page */

  rowaddr = block * nandmodel_pagesperblock(&priv->raw.model) + page;
  finfo("pagesize=%d eccsaddr=%d rowaddr=%d\n", pagesize, eccsaddr, rowaddr);

#if 1 /* Use NFC SRAM */
  /* Write the data area to NFC SRAM */

  ret = nand_nfcsram_write(priv, (uint8_t *)data, pagesize, 0);
  if (ret < 0)
    {
      ferr("ERROR: Block %d page %d nand_nfcsram_write for data region "
           "failed: %d\n",
           block, page, ret);
      goto errout;
    }
#endif

  /* Get the encoded number of sectors per page */

  switch (pmecc_get_pagesize())
    {
    case HSMC_PMECCFG_PAGESIZE_1SEC:
      sectersperpage = 1;
      break;

    case HSMC_PMECCFG_PAGESIZE_2SEC:
      sectersperpage = 2;
      break;

    case HSMC_PMECCFG_PAGESIZE_4SEC:
      sectersperpage = 4;
      break;

    case HSMC_PMECCFG_PAGESIZE_8SEC:
      sectersperpage = 8;
      break;

    default:
      sectersperpage = 1;
      break;
    }

  /* Reset and enable the PMECC */

  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_RST);
  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_ENABLE);

  /* Start a data phase */

  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_DATA);

  regval  = nand_getreg(SAM_HSMC_PMECCFG);
  regval |= HSMC_PMECCFG_NANDWR_WRITE;
  nand_putreg(SAM_HSMC_PMECCFG, regval);

#if 1 /* Use NFC SRAM */
  /* Setup the NFC and wait for the transfer to complete */

  nand_setup_xfrdone(priv);
  nand_nfc_cleale(priv,
                  HSMC_CLE_WRITE_EN | HSMC_ALE_COL_EN |
                  HSMC_ALE_ROW_EN | HSMC_CLE_DATA_EN,
                  COMMAND_WRITE_1, 0, 0, rowaddr);
  nand_wait_xfrdone(priv);
#else
  /* Setup the for the data transfer */

  nand_nfc_cleale(priv,
                  HSMC_CLE_WRITE_EN | HSMC_ALE_COL_EN | HSMC_ALE_ROW_EN,
                  COMMAND_WRITE_1, 0, 0, rowaddr);

  /* Transfer the data via the NAND */

  ret = nand_write(priv, (uint8_t *)data, pagesize, 0);
  if (ret < 0)
    {
      ferr("ERROR: Block %d page %d nand_write for data region failed: %d\n",
           block, page, ret);
      goto errout;
    }
#endif

  /* Set up for the ECC transfer */

  nand_nfc_cleale(priv, HSMC_CLE_WRITE_EN | HSMC_ALE_COL_EN,
                  COMMAND_RANDOM_IN, 0, eccsaddr, 0);

  /* Wait until the kernel of the PMECC is not busy */

  while ((nand_getreg(SAM_HSMC_PMECCSR) & HSMC_PMECCSR_BUSY) != 0);

  /* Get the ECC values from the PMECC */

  eccpersector = (pmecc_get_eccsize()) / sectersperpage;
  eccsize      = sectersperpage * eccpersector;

  finfo("sectersperpage=%d eccpersector=%d eccsize=%d\n",
        sectersperpage, eccpersector, eccsize);

#ifdef CONFIG_SAMA5_PMECC_TRIMPAGE
  if (nand_trrimffs(priv) && page >= nand_get_trimpage(priv))
    {
      /* Comments in the Atmel sample say that this behavior was found to
       * fix both UBI and JFFS2 images written to cleanly erased NAND
       * partitions
       */

      memset(g_nand.ecctab, 0xff, eccsize);
    }
  else
#endif
    {
      /* Read ECC registers for each sector in the page */

      ecc = g_nand.ecctab;
      for (sector = 0; sector < sectersperpage; sector++)
        {
          pmecc = (volatile uint8_t *)SAM_HSMC_PMECC_BASE(sector);

          /* Read all EEC registers for this page */

          for (i = 0; i < eccpersector; i++)
            {
              *ecc++ = *pmecc++;
            }
        }
    }

  /* Write the ECC to NAND */

  ret = nand_write(priv, (uint8_t *)g_nand.ecctab, eccsize, 0);
  if (ret < 0)
    {
      ferr("ERROR: Block %d page %d nand_write for spare region "
           "failed: %d\n",
           block, page, ret);
      goto errout;
    }

  nand_nfc_cleale(priv, HSMC_CLE_WRITE_EN, COMMAND_WRITE_2, 0, 0, 0);
  nand_wait_ready(priv);

  /* Check for success */

  ret = nand_operation_complete(priv);
  if (ret < 0)
    {
      ferr("ERROR: Block %d page %d Failed writing data area: %d\n",
           block, page, ret);
    }

  /* Disable the PMECC */

errout:
  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_DISABLE);
  pmecc_unlock();
  return ret;
}
#endif /* CONFIG_SAMA5_HAVE_PMECC */

/****************************************************************************
 * Name: nand_eraseblock
 *
 * Description:
 *   Erases the specified block of the device.
 *
 * Input Parameters:
 *   raw    - Lower-half, raw NAND FLASH interface
 *   block  - Number of the physical block to erase.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static inline int nand_tryeraseblock(struct sam_nandcs_s *priv, off_t block)
{
  uint32_t rowaddr;
  int ret;

  /* Calculate address used for erase */

  rowaddr = block * nandmodel_pagesperblock(&priv->raw.model);

  /* Configure the NFC for the block erase */

  nand_nfc_cleale(priv, HSMC_CLE_VCMD2_EN | HSMC_ALE_ROW_EN,
                  COMMAND_ERASE_1, COMMAND_ERASE_2, 0, rowaddr);

  /* Wait for the erase operation to complete */

  nand_wait_ready(priv);

  ret = nand_operation_complete(priv);
  if (ret < 0)
    {
      ferr("ERROR: Block %d Could not erase: %d\n", block, ret);
    }

  return ret;
}

static int nand_eraseblock(struct nand_raw_s *raw, off_t block)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  int retries = NAND_ERASE_NRETRIES;
  int ret;

  DEBUGASSERT(priv);

  finfo("block=%d\n", (int)block);

  /* Get exclusive access to the HSMC hardware.
   * REVISIT:  The scope of this exclusivity is just NAND.
   */

  ret = nand_lock();
  if (ret < 0)
    {
      return ret;
    }

  /* Try up to NAND_ERASE_NRETRIES times to erase the FLASH */

  while (retries > 0)
    {
      ret = nand_tryeraseblock(priv, block);
      if (ret == OK)
        {
          nand_unlock();
          return OK;
        }

      retries--;
    }

  ferr("ERROR: Block %d Failed to erase after %d tries\n",
       (int)block, NAND_ERASE_NRETRIES);

  nand_unlock();
  return -EAGAIN;
}

/****************************************************************************
 * Name: nand_rawread
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.  This is a raw read of the flash contents.
 *
 * Input Parameters:
 *   raw   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   spare - Buffer where the spare area will be stored.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_rawread(struct nand_raw_s *raw, off_t block,
                        unsigned int page, void *data, void *spare)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  int ret;

  DEBUGASSERT(raw);

  /* Get exclusvie access to the HSMC hardware.
   * REVISIT:  The scope of this exclusivity is just NAND.
   */

  ret = nand_lock();
  if (ret >= 0)
    {
      ret = nand_readpage_noecc(priv, block, page, data, spare);
      nand_unlock();
    }

  return ret;
}

/****************************************************************************
 * Name: nand_rawwrite
 *
 * Description:
 *   Writes the data and/or the spare area of a page on a NAND FLASH chip.
 *   This is a raw write of the flash contents.
 *
 * Input Parameters:
 *   raw   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writing
 *   spare - Buffer containing the spare data to be written.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_rawwrite(struct nand_raw_s *raw, off_t block,
                         unsigned int page, const void *data,
                         const void *spare)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  int ret;

  DEBUGASSERT(raw);

  /* Get exclusvie access to the HSMC hardware.
   * REVISIT:  The scope of this exclusivity is just NAND.
   */

  ret = nand_lock();
  if (ret >= 0)
    {
      ret = nand_writepage_noecc(priv, block, page, data, spare);
      nand_unlock();
    }

  return ret;
}

/****************************************************************************
 * Name: nand_readpage
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.  Hardware ECC checking will be performed if so
 *   configured.
 *
 * Input Parameters:
 *   raw   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   spare - Buffer where the spare area will be stored.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_HWECC
static int nand_readpage(struct nand_raw_s *raw, off_t block,
                         unsigned int page, void *data, void *spare)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  int ret;

  DEBUGASSERT(raw);

  /* Get exclusvie access to the HSMC hardware.
   * REVISIT:  The scope of this exclusivity is just NAND.
   */

  ret = nand_lock();
  if (ret < 0)
    {
      return ret;
    }

  /* Read the page */

  DEBUGASSERT(raw->ecctype != NANDECC_SWECC);
  switch (raw->ecctype)
    {
    case NANDECC_NONE:
    case NANDECC_CHIPECC:
      ret = nand_readpage_noecc(priv, block, page, data, spare);
      break;

#ifdef CONFIG_SAMA5_HAVE_PMECC
    case NANDECC_PMECC:
      DEBUGASSERT(!spare);
      ret = nand_readpage_pmecc(priv, block, page, data);
      break;
#endif

    case NANDECC_SWECC:
    default:
      ret = -EINVAL;
      break;
    }

  nand_unlock();
  return ret;
}
#endif

/****************************************************************************
 * Name: nand_writepage
 *
 * Description:
 *   Writes the data and/or the spare area of a page on a NAND FLASH chip.
 *   Hardware ECC checking will be performed if so configured.
 *
 * Input Parameters:
 *   raw   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writing
 *   spare - Buffer containing the spare data to be written.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_HWECC
static int nand_writepage(struct nand_raw_s *raw, off_t block,
                          unsigned int page, const void *data,
                          const void *spare)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  int ret;

  DEBUGASSERT(raw);

  /* Get exclusvie access to the HSMC hardware.
   * REVISIT:  The scope of this exclusivity is just NAND.
   */

  ret = nand_lock();
  if (ret < 0)
    {
      return ret;
    }

  /* Write the page */

  DEBUGASSERT(raw->ecctype != NANDECC_SWECC);
  switch (raw->ecctype)
    {
    case NANDECC_NONE:
    case NANDECC_CHIPECC:
      ret = nand_writepage_noecc(priv, block, page, data, spare);
      break;

#ifdef CONFIG_SAMA5_HAVE_PMECC
    case NANDECC_PMECC:
      DEBUGASSERT(!spare);
      ret = nand_writepage_pmecc(priv, block, page, data);
      break;
#endif

    case NANDECC_SWECC:
    default:
      ret = -EINVAL;
      break;
    }

  nand_unlock();
  return ret;
}
#endif

/****************************************************************************
 * Name: nand_reset
 *
 * Description:
 *   Resets a NAND FLASH device
 *
 * Input Parameters:
 *   priv - Lower-half, private NAND FLASH device state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nand_reset(struct sam_nandcs_s *priv)
{
  finfo("Resetting\n");
  nand_nfc_cleale(priv, 0, COMMAND_RESET, 0, 0, 0);
  nand_wait_ready(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_nand_initialize
 *
 * Description:
 *   Create and initialize an raw NAND device instance.  This driver
 *   implements the RAW NAND interface:  No software ECC or sparing is
 *   performed here.  Those necessary NAND features are provided by common,
 *   higher level NAND MTD layers found in drivers/mtd.
 *
 * Input Parameters:
 *   cs - Chip select number (in the event that multiple NAND devices
 *        are connected on-board).
 *
 * Returned Value:
 *   On success a non-NULL pointer to an MTD device structure is returned;
 *   NULL is returned on a failure.
 *
 ****************************************************************************/

struct mtd_dev_s *sam_nand_initialize(int cs)
{
  struct sam_nandcs_s *priv;
  struct mtd_dev_s *mtd;
  uintptr_t cmdaddr;
  uintptr_t addraddr;
  uintptr_t dataaddr;
  uint8_t ecctype;
  int ret;

  finfo("CS%d\n", cs);

  /* Select the device structure (In SAMA5D3, NAND is only supported on CS3). */

#ifdef CONFIG_SAMA5_EBICS0_NAND
  if (cs == HSMC_CS0)
    {
      /* Refer to the pre-allocated NAND device structure */

      priv = &g_cs0nand;

      /* Set up the NAND addresses.  These must be provided in the board.h
       * header file.
       */

      cmdaddr  = BOARD_EBICS0_NAND_CMDADDR;
      addraddr = BOARD_EBICS0_NAND_ADDRADDR;
      dataaddr = BOARD_EBICS0_NAND_DATAADDR;

      /* Pass on the configured ECC type */

      ecctype = SAMA5_EBICS0_ECCTYPE;
    }
  else
#endif
#ifdef CONFIG_SAMA5_EBICS1_NAND
  if (cs == HSMC_CS1)
    {
      /* Refer to the pre-allocated NAND device structure */

      priv = &g_cs1nand;

      /* Set up the NAND addresses.  These must be provided in the board.h
       * header file.
       */

      cmdaddr  = BOARD_EBICS1_NAND_CMDADDR;
      addraddr = BOARD_EBICS1_NAND_ADDRADDR;
      dataaddr = BOARD_EBICS1_NAND_DATAADDR;

      /* Pass on the configured ECC type */

      ecctype = SAMA5_EBICS1_ECCTYPE;
    }
  else
#endif
#ifdef CONFIG_SAMA5_EBICS2_NAND
  if (cs == HSMC_CS2)
    {
      /* Refer to the pre-allocated NAND device structure */

      priv = &g_cs2nand;

      /* Set up the NAND addresses.  These must be provided in the board.h
       * header file.
       */

      cmdaddr  = BOARD_EBICS2_NAND_CMDADDR;
      addraddr = BOARD_EBICS2_NAND_ADDRADDR;
      dataaddr = BOARD_EBICS2_NAND_DATAADDR;

      /* Pass on the configured ECC type */

      ecctype = SAMA5_EBICS2_ECCTYPE;
    }
  else
#endif
#ifdef CONFIG_SAMA5_EBICS3_NAND
  if (cs == HSMC_CS3)
    {
      /* Refer to the pre-allocated NAND device structure */

      priv = &g_cs3nand;

      /* Set up the NAND addresses.  These must be provided in the board.h
       * header file.
       */

      cmdaddr  = BOARD_EBICS3_NAND_CMDADDR;
      addraddr = BOARD_EBICS3_NAND_ADDRADDR;
      dataaddr = BOARD_EBICS3_NAND_DATAADDR;

      /* Pass on the configured ECC type */

      ecctype = SAMA5_EBICS3_ECCTYPE;
    }
  else
#endif
    {
      ferr("ERROR: CS%d unsupported or invalid\n", cs);
      return NULL;
    }

  /* Initialize the device structure */

  memset(priv, 0, sizeof(struct sam_nandcs_s));
  priv->raw.cmdaddr    = cmdaddr;
  priv->raw.addraddr   = addraddr;
  priv->raw.dataaddr   = dataaddr;
  priv->raw.ecctype    = ecctype;
  priv->raw.eraseblock = nand_eraseblock;
  priv->raw.rawread    = nand_rawread;
  priv->raw.rawwrite   = nand_rawwrite;
#ifdef CONFIG_MTD_NAND_HWECC
  priv->raw.readpage   = nand_readpage;
  priv->raw.writepage  = nand_writepage;
#endif
  priv->cs             = cs;

#ifdef CONFIG_SAMA5_NAND_DMA
  /* The waitsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->waitsem, 0, 0);
  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);
#endif

  /* Perform one-time, global NFC/PMECC initialization */

  if (!g_nand.initialized)
    {
      /* Initialize the global nand state structure */

#if NAND_NBANKS > 1
      nxsem_init(&g_nand.exclsem, 0, 1);
#endif

#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
      /* The waitsem semaphore is used for signaling and, hence, should not
       * have priority inheritance enabled.
       */

      nxsem_init(&g_nand.waitsem, 0, 0);
      nxsem_set_protocol(&g_nand.waitsem, SEM_PRIO_NONE);
#endif

      /* Enable the NAND FLASH Controller (The NFC is always used) */

      nand_putreg(SAM_HSMC_CTRL, HSMC_CTRL_NFCEN);

#ifdef CONFIG_SAMA5_HAVE_PMECC
      /* Perform one-time initialization of the PMECC */

      pmecc_initialize();

#else
      /* Disable the PMECC if it is not being used */

      nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_RST);
      nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_DISABLE);
      nand_putreg(SAM_HSMC_PMECCFG, 0);
#endif

#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
      /* Attach the CAN interrupt handler */

      ret = irq_attach(SAM_IRQ_HSMC, hsmc_interrupt, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to attach HSMC IRQ (%d)", SAM_IRQ_HSMC);
          return NULL;
        }
#endif

      /* Disable all interrupts at the HSMC */

      nand_putreg(SAM_HSMC_IDR, HSMC_NFCINT_ALL);

#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
      /* Enable the HSMC interrupts at the interrupt controller */

      up_enable_irq(SAM_IRQ_HSMC);
      g_nand.initialized = true;
#endif
    }

  /* Initialize the NAND hardware for this CS */

  /* Perform board-specific SMC initialization for this CS.  This should
   * include:
   *
   *   1. Enabling of clocking to the HSMC
   *   2. Configuration of timing for the HSMC NAND CS
   *   3. Configuration of PIO pins
   *
   * Other than enabling the HSMC, these are all things that the board-
   * cognizant logic is best prepared to handle.
   */

  ret = board_nandflash_config(cs);
  if (ret < 0)
    {
      ferr("ERROR: board_nandflash_config failed for CS%d: %d\n",
           cs, ret);
      return NULL;
    }

  /* Reset the NAND FLASH part */

  nand_reset(priv);

  /* Probe the NAND part.  On success, an MTD interface that wraps
   * our raw NAND interface is returned.
   */

  mtd = nand_initialize(&priv->raw);
  if (!mtd)
    {
      ferr("ERROR: CS%d nand_initialize failed %d\n", cs);
      return NULL;
    }

#ifdef CONFIG_SAMA5_NAND_DMA
  /* Allocate a DMA channel for NAND transfers.  The channels will be
   * configured as needed on-the-fly.  NOTE that no failure is declared
   * if we fail to allocate DMA channel; in that case, only non-DMA
   * transfers will be performed.
   */

  priv->dma = sam_dmachannel(NAND_DMAC, 0);
  if (!priv->dma)
    {
      ferr("ERROR: Failed to allocate the DMA channel for CS%d\n", cs);
    }
#endif

  /* Return the MTD wrapper interface as the MTD device */

  return mtd;
}

/****************************************************************************
 * Name: nand_checkreg
 *
 * Description:
 *   Check if the current HSMC register access is a duplicate of the
 *   preceding.
 *
 * Input Parameters:
 *   regval   - The value to be written
 *   regaddr - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_REGDEBUG
bool nand_checkreg(bool wr, uintptr_t regaddr, uint32_t regval)
{
  if (wr      == g_nand.wr &&      /* Same kind of access? */
      regval  == g_nand.regval &&  /* Same regval? */
      regaddr == g_nand.regadddr)  /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      g_nand.ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (g_nand.ntimes > 0)
        {
          /* Yes... show how many times we did it */

          finfo("...[Repeats %d times]...\n", g_nand.ntimes);
        }

      /* Save information about the new access */

      g_nand.wr       = wr;
      g_nand.regval   = regval;
      g_nand.regadddr = regaddr;
      g_nand.ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

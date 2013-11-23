/****************************************************************************
 * arch/arm/src/sama5/sam_nand.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatibile license that requires this
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
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/nand_raw.h>
#include <nuttx/mtd/nand_model.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "cache.h"

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

#define SMC_ALE_COL_EN       (1 << 0)
#define SMC_ALE_ROW_EN       (1 << 1)
#define SMC_CLE_WRITE_EN     (1 << 2)
#define SMC_CLE_DATA_EN      (1 << 3)
#define SMC_CLE_VCMD2_EN     (1 << 4)

/* Number of tries for erasing or writing block */

#define NAND_ERASE_NRETRIES  2
#define NAND_WRITE_NRETRIES  2

/* DMA Configuration */

#define DMA_FLAGS8 \
   DMACH_FLAG_FIFOCFG_LARGEST | \
   (((0x3f) << DMACH_FLAG_PERIPHPID_SHIFT) | DMACH_FLAG_PERIPHAHB_AHB_IF0 | \
   DMACH_FLAG_PERIPHWIDTH_8BITS | DMACH_FLAG_PERIPHINCREMENT | \
   DMACH_FLAG_PERIPHCHUNKSIZE_1 | \
   ((0x3f) << DMACH_FLAG_MEMPID_SHIFT) | DMACH_FLAG_MEMAHB_AHB_IF0 | \
   DMACH_FLAG_MEMWIDTH_8BITS | DMACH_FLAG_MEMINCREMENT | \
   DMACH_FLAG_MEMCHUNKSIZE_1)

#define DMA_FLAGS16 \
   DMACH_FLAG_FIFOCFG_LARGEST | \
   (((0x3f) << DMACH_FLAG_PERIPHPID_SHIFT) | DMACH_FLAG_PERIPHAHB_AHB_IF0 | \
   DMACH_FLAG_PERIPHWIDTH_16BITS | DMACH_FLAG_PERIPHINCREMENT | \
   DMACH_FLAG_PERIPHCHUNKSIZE_1 | \
   ((0x3f) << DMACH_FLAG_MEMPID_SHIFT) | DMACH_FLAG_MEMAHB_AHB_IF0 | \
   DMACH_FLAG_MEMWIDTH_16BITS | DMACH_FLAG_MEMINCREMENT | \
   DMACH_FLAG_MEMCHUNKSIZE_1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Low-level HSMC Helpers */

static void     nand_wait_ready(struct sam_nandcs_s *priv);
static void     nand_cmdsend(struct sam_nandcs_s *priv, uint32_t cmd,
                  uint32_t acycle, uint32_t cycle0);
static bool     nand_operation_complete(struct sam_nandcs_s *priv);
static void     nand_coladdr_write(struct sam_nandcs_s *priv,
                  uint16_t coladdr);
static void     nand_rowaddr_write(struct sam_nandcs_s *priv,
                  uint32_t rowaddr);
static int      nand_translate_address(struct sam_nandcs_s *priv,
                  uint16_t coladdr, uint32_t rowaddr, uint32_t *acycle0,
                  uint32_t *acycle1234, bool rowonly);
static uint32_t nand_get_acycle(int ncycles);
static void     nand_nfc_configure(struct sam_nandcs_s *priv,
                   uint8_t mode, uint32_t cmd1, uint32_t cmd2,
                   uint32_t coladdr, uint32_t rowaddr);

/* Interrupt Handling */

static void     nand_wait_cmddone(struct sam_nandcs_s *priv);
static void     nand_wait_xfrdone(struct sam_nandcs_s *priv);
static void     nand_wait_rbedge(struct sam_nandcs_s *priv);

/* DMA Helpers */

static int      nand_wait_dma(struct sam_nandcs_s *priv);
static void     nand_dmacallback(DMA_HANDLE handle, void *arg, int result);

/* Raw Data Transfer Helpers */

static int      nand_nfcsram_read(uintptr_t src, uint8_t *dest,
                  size_t buflen);
static int      nand_smc_read8(uintptr_t src, uint8_t *dest, size_t buflen);
static int      nand_smc_read16(uintptr_t src, uint8_t *dest,
                  size_t buflen);
static int      nand_read(struct sam_nandcs_s *priv, bool nfcsram,
                  uint8_t *buffer, size_t buflen);

static int      nand_nfcsram_write(const uint8_t *src, uintptr_t dest,
                  size_t buflen);
static int      nand_smc_write8(const uint8_t *src, uintptr_t dest,
                  size_t buflen);
static int      nand_smc_write16(const uint8_t *src, uintptr_t dest,
                  size_t buflen);
static int      nand_write(struct sam_nandcs_s *priv, bool nfcsram,
                  uint8_t *buffer, size_t buflen, off_t offset);

/* NAND Access Helpers */

static int      nand_readpage_noecc(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, void *data, void *spare);

#ifdef NAND_HAVE_PMECC
static int      nand_readpage_pmecc(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, void *data);
#endif

static int      nand_writepage_noecc(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, const void *data, const void *spare);

#ifdef NAND_HAVE_PMECC
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
 * Name: nand_wait_ready
 *
 * Description:
 *   Waiting for the completion of a page program, erase and random read
 *   completion.
 *
 * Input parameters:
 *   priv  Pointer to a sam_nandcs_s instance.
 *
 * Returned value.
 *   None
 *
 ****************************************************************************/

static void nand_wait_ready(struct sam_nandcs_s *priv)
{
#ifdef SAMA5_NAND_READYBUSY
  while (board_nand_busy(priv->cs));
#endif
  nand_nfc_configure(priv, 0, COMMAND_STATUS, 0, 0, 0);
  while ((READ_DATA8(&priv->raw) & STATUS_READY) == 0);
}

/****************************************************************************
 * Name: nand_cmdsend
 *
 * Description:
 *   Use the HOST nandflash controller to send a command to the NFC.
 *
 * Input parameters:
 *   priv - Lower-half, private NAND FLASH device state
 *   cmd    - command to send
 *   acycle - address cycle when command access id decoded
 *   cycle0 - address at first cycle
 *
 * Returned value.
 *   None
 *
 ****************************************************************************/

static void nand_cmdsend(struct sam_nandcs_s *priv, uint32_t cmd,
                         uint32_t acycle, uint32_t cycle0)
{
  uintptr_t cmdaddr;

  /* Wait until host controller is not busy. */

  while ((nand_getreg(NFCCMD_BASE + NFCADDR_CMD_NFCCMD) & 0x8000000) != 0);
  priv->cmddone = false;

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
 * Input parameters:
 *   priv - Lower-half, private NAND FLASH device state
 *
 * Returned value.
 *   None
 *
 ****************************************************************************/

static bool nand_operation_complete(struct sam_nandcs_s *priv)
{
  uint8_t status;

  nand_nfc_configure(priv, 0, COMMAND_STATUS, 0, 0, 0);
  status = READ_DATA8(&priv->raw);

  if (((status & STATUS_READY) == 0) || ((status & STATUS_ERROR) != 0))
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: nand_coladdr_write
 *
 * Description:
 *   Send a column address to the NAND FLASH chip.
 *
 * Input parameters:
 *   priv - Lower-half, private NAND FLASH device state
 *
 * Returned value.
 *   None
 *
 ****************************************************************************/

static void nand_coladdr_write(struct sam_nandcs_s *priv, uint16_t coladdr)
{
  uint16_t pagesize = nandmodel_getpagesize(&priv->raw.model);

  /* Check the data bus width of the NAND FLASH */

  if (nandmodel_getbuswidth(&priv->raw.model) == 16)
    {
      /* Use word vs byte addressing */

      coladdr >>= 1;
    }

  /* Send single column address byte for small block devices, or two column
   * address bytes for large block devices
   */

  while (pagesize > 2)
    {
      if (nandmodel_getbuswidth(&priv->raw.model) == 16)
        {
          WRITE_ADDRESS16(&priv->raw, coladdr & 0xff);
        }
      else
        {
          WRITE_ADDRESS8(&priv->raw, coladdr & 0xff);
        }

      pagesize >>= 8;
      coladdr >>= 8;
    }
}

/****************************************************************************
 * Name: nand_rowaddr_write
 *
 * Description:
 *   Send a row address to the NAND FLASH chip.
 *
 * Input parameters:
 *   priv - Lower-half, private NAND FLASH device state
 *
 * Returned value.
 *   None
 *
 ****************************************************************************/

static void nand_rowaddr_write(struct sam_nandcs_s *priv, uint32_t rowaddr)
{
  uint32_t npages = nandmodel_getdevpagesize(&priv->raw.model);

  while (npages > 0)
    {
      if (nandmodel_getbuswidth(&priv->raw.model) == 16)
        {
          WRITE_ADDRESS16(&priv->raw, rowaddr & 0xff);
        }
      else
        {
          WRITE_ADDRESS8(&priv->raw, rowaddr & 0xff);
        }

      npages >>= 8;
      rowaddr >>= 8;
    }
}

/****************************************************************************
 * Name: nand_translate_address
 *
 * Description:
 *   Translates the given column and row address into first and other (1-4)
 *   address cycles. The resulting values are stored in the provided
 *   variables if they are not null.
 *
 * Input parameters:
 *   priv       - Lower-half, private NAND FLASH device state
 *   coladdr    - Column address to translate.
 *   rowaddr    - Row address to translate.
 *   acycle0    - First address cycle
 *   acycle1234 - Four address cycles.
 *   rowonly    - True:Only ROW address is used.
 *
 * Returned value.
 *   Number of address cycles converted.
 *
 ****************************************************************************/

static int nand_translate_address(struct sam_nandcs_s *priv,
                                  uint16_t coladdr, uint32_t rowaddr,
                                  uint32_t *acycle0, uint32_t *acycle1234,
                                  bool rowonly)
{
  uint16_t maxsize;
  uint32_t page;
  uint32_t accum0;
  uint32_t accum1234;
  uint8_t bytes[8];
  int  ncycles;
  int  ndx;
  int  pos;

  /* Setup */

  maxsize   = nandmodel_getpagesize(&priv->raw.model) +
              nandmodel_getsparesize(&priv->raw.model) - 1;
  page      = nandmodel_getdevpagesize(&priv->raw.model) - 1;
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

  while (page > 0)
    {
      bytes[ncycles++] = rowaddr & 0xff;
      page >>= 8;
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
 * Input parameters:
 *   ncycles    - Number of address cycles
 *
 * Returned value.
 *   NFC command value
 *
 ****************************************************************************/

static uint32_t nand_get_acycle(int ncycles)
{
  switch(ncycles)
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
 * Name: nand_nfc_configure
 *
 * Description:
 *   Sets NFC configuration.
 *
 * Input parameters:
 *   priv    - Pointer to a sam_nandcs_s instance.
 *   mode    - SMC ALE CLE mode parameter.
 *   cmd1    - First command to be sent.
 *   cmd2    - Second command to be sent.
 *   coladdr - Column address.
 *   rowaddr - Row address.
 *
 * Returned value.
 *   None
 *
 ****************************************************************************/

static void nand_nfc_configure(struct sam_nandcs_s *priv, uint8_t mode,
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

  if ((mode & SMC_CLE_WRITE_EN) != 0)
    {
      rw = NFCADDR_CMD_NFCWR;
    }
  else
    {
      rw = NFCADDR_CMD_NFCRD;
    }

  if ((mode & SMC_CLE_DATA_EN) != 0)
    {
      regval = NFCADDR_CMD_DATAEN;
    }
  else
    {
      regval = NFCADDR_CMD_DATADIS;
    }

  if (((mode & SMC_ALE_COL_EN) == SMC_ALE_COL_EN) ||
      ((mode & SMC_ALE_ROW_EN) == SMC_ALE_ROW_EN))
    {
      bool rowonly = (((mode & SMC_ALE_COL_EN) == 0) &&
                      ((mode & SMC_ALE_ROW_EN) == SMC_ALE_ROW_EN));
      nand_translate_address(priv, coladdr, rowaddr, &acycle0, &acycle1234, rowonly);
      acycle = nand_get_acycle(ncycles);
    }
  else
    {
      acycle = NFCADDR_CMD_ACYCLE_NONE;
    }

  cmd = (rw | regval | NFCADDR_CMD_CSID_3 | acycle |
         (((mode & SMC_CLE_VCMD2_EN) == SMC_CLE_VCMD2_EN) ? NFCADDR_CMD_VCMD2 : 0) |
         (cmd1 << NFCADDR_CMD_CMD1_SHIFT) | (cmd2 << NFCADDR_CMD_CMD2_SHIFT));

  nand_cmdsend(priv, cmd, acycle1234, acycle0);
}

/****************************************************************************
 * Name: nand_wait_cmddone
 *
 * Description:
 *   Wait for NFC command done
 *
 * Input parameters:
 *   None
 *
 * Returned value.
 *   None
 *
 ****************************************************************************/

static void nand_wait_cmddone(struct sam_nandcs_s *priv)
{
  int ret;

  while (!priv->cmddone)
    {
      ret = sem_wait(&priv->waitsem);
      if (ret < 0)
        {
          DEBUGASSERT(errno == EINTR);
        }
    }

  priv->cmddone = false;
}

/****************************************************************************
 * Name: nand_wait_xfrdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 * Input parameters:
 *   None
 *
 * Returned value.
 *   None
 *
 ****************************************************************************/

static void nand_wait_xfrdone(struct sam_nandcs_s *priv)
{
  int ret;

  while (!priv->xfrdone)
    {
      ret = sem_wait(&priv->waitsem);
      if (ret < 0)
        {
          DEBUGASSERT(errno == EINTR);
        }
    }

  priv->xfrdone = false;
}

/****************************************************************************
 * Name: nand_wait_rbedge
 *
 * Description:
 *   Wait for read/busy edge detection
 *
 * Input parameters:
 *   None
 *
 * Returned value.
 *   None
 *
 ****************************************************************************/

static void nand_wait_rbedge(struct sam_nandcs_s *priv)
{
  int ret;

  while (!priv->rbedge)
    {
      ret = sem_wait(&priv->waitsem);
      if (ret < 0)
        {
          DEBUGASSERT(errno == EINTR);
        }
    }

  priv->rbedge = false;
}

/****************************************************************************
 * Name: nand_wait_dma
 *
 * Description:
 *   Wait for the completion of a DMA transfer
 *
 * Input parameters:
 *   Wait for read/busy edge detection
 *
 * Returned value.
 *   The result of the DMA.  OK on success; a negated ernno value on failure.
 *
 ****************************************************************************/

static int nand_wait_dma(struct sam_nandcs_s *priv)
{
  int ret;

  while (!priv->dmadone)
    {
      ret = sem_wait(&priv->waitsem);
      if (ret < 0)
        {
          DEBUGASSERT(errno == EINTR);
        }
    }

  priv->dmadone = false;
  return priv->result;
}

/****************************************************************************
 * Name: sam_adc_dmacallback
 *
 * Description:
 *   Called when one NAND DMA sequence completes.  This function just wakes
 *   the the waiting NAND driver logic.
 *
 ****************************************************************************/

static void nand_dmacallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)arg;

  DEBUGASSERT(priv);

  /* Wake up the thread that is waiting for the DMA result */

  priv->result  = result;
  priv->dmadone = true;
  sem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: nand_dma_read
 *
 * Description:
 *   Transfer data to NAND from the provided buffer via DMA.
 *
 * Input Parameters:
 *   priv   - Lower-half, private NAND FLASH device state
 *   vsrc   - NAND data destination address.
 *   vdest  - Buffer where data read from NAND will be returned.
 *   nbytes - The number of bytes to transfer
 *
 * Returned Value
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nand_dma_read(struct sam_nandcs_s *priv,
                          uintptr_t vsrc, uintptr_t vdest, size_t nbytes)
{
  uint32_t psrc;
  uint32_t pdest;
  int ret;

  DEBUGASSERT(priv->dma);

  /* Invalidate the destination memory buffer before performing the DMA (so
   * that nothing gets flushed later, corrupting the DMA transfer, and so
   * that memory will be re-cached after the DMA completes).
   */

  cp15_invalidate_dcache(vdest, vdest + nbytes);

  /* DMA will need physical addresses. */

  psrc  = sam_physregaddr(vsrc);   /* Source is NAND */
  pdest = sam_physramaddr(vdest);  /* Destination is normal memory */

  /* Setup the Memory-to-Memory DMA.  The semantics of the DMA module are
   * awkward here.  We will treat the NAND (src) as the peripheral source
   * and memory as the destination.  Internally, the DMA module will realize
   * that this is a memory to memory transfer and should do the right thing.
   */

  ret = sam_dmarxsetup(priv->dma, psrc, pdest, nbytes);
  if (ret < 0)
    {
      fdbg("ERROR: sam_dmarxsetup failed: %d\n", ret);
      return ret;
    }

  /* Start the DMA */

  priv->dmadone = false;
  priv->result  = -EBUSY;

  sam_dmastart(priv->dma, nand_dmacallback, priv);

  /* Wait for the DMA to complete */

  ret = nand_wait_dma(priv);
  if (ret < 0)
    {
      fdbg("ERROR: DMA failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: nand_dma_write
 *
 * Description:
 *   Transfer data to NAND from the provided buffer via DMA.
 *
 * Input Parameters:
 *   priv   - Lower-half, private NAND FLASH device state
 *   vsrc   - Buffer that provides the data for the write
 *   vdest  - NAND data destination address
 *   nbytes - The number of bytes to transfer
 *
 * Returned Value
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nand_dma_write(struct sam_nandcs_s *priv,
                          uintptr_t vsrc, uintptr_t vdest, size_t nbytes)
{
  uint32_t psrc;
  uint32_t pdest;
  int ret;

  DEBUGASSERT(priv->dma);

  /* Clean the D-Cache associated with the source data buffer so that all of
   * the data to be transferred lies in physical memory
   */

  cp15_clean_dcache(vsrc, vsrc + nbytes);

  /* DMA will need physical addresses. */

  psrc  = sam_physramaddr(vsrc);   /* Source is normal memory */
  pdest = sam_physregaddr(vdest);  /* Destination is NAND (or NAND host SRAM) */

  /* Setup the Memory-to-Memory DMA.  The semantics of the DMA module are
   * awkward here.  We will treat the NAND (dest) as the peripheral destination
   * and memory as the source.  Internally, the DMA module will realize taht
   * this is a memory to memory transfer and should do the right thing.
   */

  ret = sam_dmatxsetup(priv->dma, pdest, psrc, nbytes);
  if (ret < 0)
    {
      fdbg("ERROR: sam_dmatxsetup failed: %d\n", ret);
      return ret;
    }

  /* Start the DMA */

  priv->dmadone = false;
  priv->result  = -EBUSY;

  sam_dmastart(priv->dma, nand_dmacallback, priv);

  /* Wait for the DMA to complete */

  ret = nand_wait_dma(priv);
  if (ret < 0)
    {
      fdbg("ERROR: DMA failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: nand_nfcsram_read
 *
 * Description:
 *   Read data from NAND using the NFC SRAM (without DMA)
 *
 * Input Parameters:
 *   src    - NAND data source address
 *   dest   - Buffer that will receive the data from the read
 *   buflen - The number of bytes to transfer
 *
 * Returned Value
 *   OK always
 *
 ****************************************************************************/

static int nand_nfcsram_read(uintptr_t src, uint8_t *dest, size_t buflen)
{
  uint8_t *src8 = (uint8_t *)src;

  for (; buflen > 0; buflen--)
    {
      *dest++ = *src8++;
    }

  return OK;
}

/****************************************************************************
 * Name: nand_smc_read8
 *
 * Description:
 *   Read 8-bit data from NAND using the NAND data address (without DMA)
 *
 * Input Parameters:
 *   src    - NAND data source address
 *   dest   - Buffer that will receive the data from the read
 *   buflen - The number of bytes to transfer
 *
 * Returned Value
 *   OK always
 *
 ****************************************************************************/

static int nand_smc_read8(uintptr_t src, uint8_t *dest, size_t buflen)
{
  volatile uint8_t *src8  = (volatile uint8_t *)src;

  for (; buflen > 0; buflen--)
    {
      *dest++ = *src8;
    }

  return OK;
}

/****************************************************************************
 * Name: nand_smc_read16
 *
 * Description:
 *   Read 16-bit data from NAND using the NAND data address (without DMA)
 *
 * Input Parameters:
 *   src    - NAND data source address
 *   dest   - Buffer that will receive the data from the read
 *   buflen - The number of bytes to transfer
 *
 * Returned Value
 *   OK always
 *
 ****************************************************************************/

static int nand_smc_read16(uintptr_t src, uint8_t *dest, size_t buflen)
{
  volatile uint16_t *src16  = (volatile uint16_t *)src;
  uint16_t *dest16 = (uint16_t *)dest;

  DEBUGASSERT(((uintptr_t)dest & 1) == 0);

  for (; buflen > 1; buflen -= sizeof(uint16_t))
    {
      *dest16++ = *src16;
    }

  return OK;
}

/****************************************************************************
 * Name: nand_write
 *
 * Description:
 *   Read data from NAND using the appropriate method
 *
 * Input Parameters:
 *   priv     - Lower-half, private NAND FLASH device state
 *   nfcsram  - True: Use NFC Host SRAM
 *   buffer   - Buffer that provides the data for the write
 *
 * Returned Value
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nand_read(struct sam_nandcs_s *priv, bool nfcsram,
                      uint8_t *buffer, size_t buflen)
{
  uintptr_t src;
  int buswidth;

  /* Pick the data destination:  The NFC SRAM or the NAND data address */

  if (nfcsram)
    {
      src = NFCSRAM_BASE;
    }
  else
    {
      src = priv->raw.dataaddr;
    }

  /* Then perform the transfer via DMA or not, depending on if we have
   * a DMA channel assigned.
   */

  if (priv->dma)
    {
      /* Transfer using DMA */

      return nand_dma_read(priv, src, (uintptr_t)buffer, buflen);
    }

  /* Transfer without DMA */

  else if (nfcsram)
    {
      return nand_nfcsram_read(src, buffer, buflen);
    }
  else
    {
      /* Check the data bus width of the NAND FLASH */

      buswidth = nandmodel_getbuswidth(&priv->raw.model);
      if (buswidth == 16)
        {
          return nand_smc_read16(src, buffer, buflen);
        }
      else
        {
          return nand_smc_read8(src, buffer, buflen);
        }
    }
}

/****************************************************************************
 * Name: nand_nfcsram_write
 *
 * Description:
 *   Write data to NAND using the NFC SRAM (without DMA)
 *
 * Input Parameters:
 *   src    - Buffer that provides the data for the write
 *   dest   - NAND data destination address
 *   buflen - The number of bytes to transfer
 *
 * Returned Value
 *   OK always
 *
 ****************************************************************************/

static int nand_nfcsram_write(const uint8_t *src, uintptr_t dest, size_t buflen)
{
  uint8_t *dest8 = (uint8_t *)dest;

  for (; buflen > 0; buflen--)
    {
      *dest8++ = *src++;
    }

  return OK;
}

/****************************************************************************
 * Name: nand_smc_write8
 *
 * Description:
 *   Write 8-bit wide data to NAND using the NAND data address (without DMA)
 *
 * Input Parameters:
 *   src    - Buffer that provides the data for the write
 *   dest   - NAND data destination address
 *   buflen - The number of bytes to transfer
 *
 * Returned Value
 *   OK always
 *
 ****************************************************************************/

static int nand_smc_write8(const uint8_t *src, uintptr_t dest, size_t buflen)
{
  volatile uint8_t *dest8  = (volatile uint8_t *)dest;

  for (; buflen > 0; buflen--)
    {
      *dest8 = *src++;
    }

  return OK;
}

/****************************************************************************
 * Name: nand_smc_write16
 *
 * Description:
 *   Write 16-bit wide data to NAND using the NAND data address (without DMA)
 *
 * Input Parameters:
 *   src    - Buffer that provides the data for the write
 *   dest   - NAND data destination address
 *   buflen - The number of bytes to transfer
 *
 * Returned Value
 *   OK always
 *
 ****************************************************************************/

static int nand_smc_write16(const uint8_t *src, uintptr_t dest, size_t buflen)
{
  volatile uint16_t *dest16  = (volatile uint16_t *)dest;
  const uint16_t *src16 = (const uint16_t *)src;

  DEBUGASSERT(((uintptr_t)src & 1) == 0);

  for (; buflen > 1; buflen -=  sizeof(uint16_t))
    {
      *dest16 = *src16++;
    }

  return OK;
}

/****************************************************************************
 * Name: nand_write
 *
 * Description:
 *   Write data to NAND using the appropriate method
 *
 * Input Parameters:
 *   priv     - Lower-half, private NAND FLASH device state
 *   nfcsram  - True: Use NFC Host SRAM
 *   buffer   - Buffer that provides the data for the write
 *   offset   - Data offset in bytes
 *
 * Returned Value
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nand_write(struct sam_nandcs_s *priv, bool nfcsram,
                      uint8_t *buffer, size_t buflen, off_t offset)
{
  uintptr_t dest;
  int buswidth;

  /* Pick the data source:  The NFC SRAM or the NAND data address */

  if (nfcsram)
    {
      dest = NFCSRAM_BASE;
    }
  else
    {
      dest = priv->raw.dataaddr;
    }

  /* Apply the offset to the source address */

  dest += offset;

  /* Then perform the transfer via DMA or not, depending on if we have
   * a DMA channel assigned.
   */

  if (priv->dma)
    {
      /* Transfer using DMA */

      return nand_dma_write(priv, (uintptr_t)buffer, dest, buflen);
    }

  /* Transfer without DMA */

  else if (nfcsram)
    {
      return nand_nfcsram_write(buffer, dest, buflen);
    }
  else
    {
      /* Check the data bus width of the NAND FLASH */

      buswidth = nandmodel_getbuswidth(&priv->raw.model);
      if (buswidth == 16)
        {
          return nand_smc_write16(buffer, dest, buflen);
        }
      else
        {
          return nand_smc_write8(buffer, dest, buflen);
        }
    }
}

/****************************************************************************
 * Name: nand_readpage_noecc
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.  The raw NAND contents are returned with no ECC
 *   corrections.
 *
 * Input parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   spare - Buffer where the spare area will be stored.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_readpage_noecc(struct sam_nandcs_s *priv, off_t block,
             unsigned int page, void *data, void *spare)
{
  uint32_t regval;
  uint16_t pagesize;
  uint16_t sparesize;
  off_t rawaddr;
  off_t coladdr;

  fvdbg("Block %d Page %d\n", (int)block, page);
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
      fdbg("ERROR:  Unsupported page size: %d\n", pagesize);
      return -EINVAL;
    }
  /* Configure the SMC */

  regval |= (HSMC_CFG_RBEDGE | HSMC_CFG_DTOCYC(15) | HSMC_CFG_DTOMUL_1048576 |
             HSMC_CFG_NFCSPARESIZE((sparesize-1) >> 2));

  /* Calculate actual address of the page */

  rawaddr = block * nandmodel_pagesperblock(&priv->raw.model) + page;
  coladdr = data ? 0 : pagesize;

  /* Initialize the NFC */

  priv->xfrdone = false;
  nand_nfc_configure(priv,
                     SMC_ALE_COL_EN | SMC_ALE_ROW_EN | SMC_CLE_VCMD2_EN | SMC_CLE_DATA_EN,
                     COMMAND_READ_1, COMMAND_READ_2, coladdr, rawaddr);
  nand_wait_xfrdone(priv);

  /* Read data area if so requested */

  if (data)
    {
      nand_read(priv, true, (uint8_t *)data, pagesize);
    }

  /* Read the spare are is so requrest */

  if (spare)
    {
      nand_read(priv, true, (uint8_t *)spare, sparesize);
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
 * Input parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef NAND_HAVE_PMECC
static int nand_readpage_pmecc(struct sam_nandcs_s *priv, off_t block,
             unsigned int page, void *data)
{
#warning Missing logic
  return -ENOSYS;
}
#endif /* NAND_HAVE_PMECC */

/****************************************************************************
 * Name: nand_writepage_noecc
 *
 * Description:
 *   Writes the data and/or the spare area of a page on a NAND FLASH chip.
 *   No ECC calculations are performed.
 *
 * Input parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writting
 *   spare - Buffer conatining the spare data to be written.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_writepage_noecc(struct sam_nandcs_s *priv, off_t block,
             unsigned int page, const void *data, const void *spare)
{
  uint32_t regval;
  uint16_t pagesize;
  uint16_t sparesize;
  off_t rawaddr;
  int ret = OK;

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
      fdbg("ERROR:  Unsupported page size: %d\n", pagesize);
      return -EINVAL;
    }

  /* Configure the SMC */

  regval |= (HSMC_CFG_RBEDGE | HSMC_CFG_DTOCYC(15) | HSMC_CFG_DTOMUL_1048576 |
             HSMC_CFG_NFCSPARESIZE((sparesize-1) >> 2));

  if (spare)
    {
      /* Write spare area */

      regval |= HSMC_CFG_WSPARE;
    }

  nand_putreg(SAM_HSMC_CFG, regval);

  /* Calculate physical address of the page */

  rawaddr = block * nandmodel_pagesperblock(&priv->raw.model) + page;
  fvdbg("Block %d Page %d\n", block, page);

  /* Handle the case where we use NFC SRAM */

  if (data)
    {
      nand_write(priv, true, (uint8_t *)data, pagesize, 0);

      if (spare)
        {
          nand_write(priv, true, (uint8_t *)spare, sparesize, pagesize);
        }
    }

  /* Write data area if needed */

  if (data)
    {
      /* Start a Data Phase */

      priv->xfrdone = false;
      nand_nfc_configure(priv,
                         SMC_CLE_WRITE_EN | SMC_ALE_COL_EN |SMC_ALE_ROW_EN |SMC_CLE_DATA_EN,
                         COMMAND_WRITE_1, 0, 0, rawaddr);
      nand_wait_xfrdone(priv);

      nand_nfc_configure(priv, SMC_CLE_WRITE_EN, COMMAND_WRITE_2, 0, 0, 0);
      nand_wait_rbedge(priv);

      if (!nand_operation_complete(priv))
        {
          fdbg("ERROR: Failed writing data area\n");
          ret = -EPERM;
        }
    }

  /* Write spare area alone if needed */

  else if (spare)
    {
      nand_nfc_configure(priv,
                         SMC_CLE_WRITE_EN|SMC_ALE_COL_EN|SMC_ALE_ROW_EN,
                         COMMAND_WRITE_1,0,  pagesize, rawaddr);

      ret = nand_write(priv, false, (uint8_t *)spare, sparesize, 0);
      if (ret < 0)
        {
          fdbg("ERROR: Failed writing data area\n");
          ret = -EPERM;
        }

      nand_nfc_configure(priv, SMC_CLE_WRITE_EN, COMMAND_WRITE_2,0, 0, 0);
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
 * Input parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writting
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef NAND_HAVE_PMECC
static int nand_writepage_pmecc(struct sam_nandcs_s *priv, off_t block,
             unsigned int page, const void *data)
{
  int ret = -ENOSYS;

  /* Perform write operation */
# warning Missing logic

  /* Disable the PMECC */

  pmecc_disable();
  return ret;
}
#endif /* NAND_HAVE_PMECC */

/****************************************************************************
 * Name: nand_eraseblock
 *
 * Description:
 *   Erases the specified block of the device.
 *
 * Input parameters:
 *   raw    - Lower-half, raw NAND FLASH interface
 *   block  - Number of the physical block to erase.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static inline int nand_tryeraseblock(struct sam_nandcs_s *priv, off_t block)
{
  uint32_t rawaddr;
  int ret = OK;

  /* Calculate address used for erase */

  rawaddr = block * nandmodel_pagesperblock(&priv->raw.model);

  /* Configure the NFC for the block erase */

  nand_nfc_configure(priv, (SMC_CLE_VCMD2_EN | SMC_ALE_ROW_EN),
                     COMMAND_ERASE_1, COMMAND_ERASE_2, 0, rawaddr);

  /* Wait for the erase operation to complete */

  nand_wait_ready(priv);
  if (!nand_operation_complete(priv))
    {
      fdbg("ERROR: Could not erase block %d\n", block);
      ret = -ENOEXEC;
    }

  return ret;
}

static int nand_eraseblock(struct nand_raw_s *raw, off_t block)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  int retries = NAND_ERASE_NRETRIES;
  int ret = OK;

  DEBUGASSERT(priv);

  fvdbg("Block %d\n", (int)block);

  while (retries > 0)
    {
      ret = nand_tryeraseblock(priv, block);
      if (ret == OK)
        {
          return OK;
        }

      retries--;
    }

  fdbg("ERROR: Failed to erase %d after %d tries\n",
       (int)block, NAND_ERASE_NRETRIES);

  return -EAGAIN;
}

/****************************************************************************
 * Name: nand_rawread
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.  This is a raw read of the flash contents.
 *
 * Input parameters:
 *   raw   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   spare - Buffer where the spare area will be stored.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_rawread(struct nand_raw_s *raw, off_t block,
                        unsigned int page, void *data, void *spare)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  DEBUGASSERT(raw);

  return nand_readpage_noecc(priv, block, page, data, spare);
}

/****************************************************************************
 * Name: nand_rawwrite
 *
 * Description:
 *   Writes the data and/or the spare area of a page on a NAND FLASH chip.
 *   This is a raw write of the flash contents.
 *
 * Input parameters:
 *   raw   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writting
 *   spare - Buffer containing the spare data to be written.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_rawwrite(struct nand_raw_s *raw, off_t block,
                         unsigned int page, const void *data,
                         const void *spare)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  DEBUGASSERT(raw);

  return nand_writepage_noecc(priv, block, page, data, spare);
}

/****************************************************************************
 * Name: nand_readpage
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.  Hardware ECC checking will be performed if so
 *   configured.
 *
 * Input parameters:
 *   raw   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   spare - Buffer where the spare area will be stored.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_HWECC
static int nand_readpage(struct nand_raw_s *raw, off_t block,
                         unsigned int page, void *data, void *spare)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  DEBUGASSERT(raw);

#ifndef CONFIG_MTD_NAND_BLOCKCHECK
  return nand_readpage_noecc(priv, block, page, data, spare);
#else
  DEBUGASSERT(raw->ecctype != NANDECC_SWECC);
  switch (raw->ecctype)
    {
    case NANDECC_NONE:
    case NANDECC_CHIPECC:
      return nand_readpage_noecc(priv, block, page, data, spare);

#ifdef NAND_HAVE_PMECC
    case NANDECC_PMECC:
      DEBUGASSERT(!spare);
      return nand_readpage_pmecc(priv, block, page, data);
#endif

    case NANDECC_SWECC:
    default:
      return -EINVAL;
    }
#endif
}
#endif

/****************************************************************************
 * Name: nand_writepage
 *
 * Description:
 *   Writes the data and/or the spare area of a page on a NAND FLASH chip.
 *   Hardware ECC checking will be performed if so configured.
 *
 * Input parameters:
 *   raw   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writting
 *   spare - Buffer conatining the spare data to be written.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_HWECC
static int nand_writepage(struct nand_raw_s *raw, off_t block,
                          unsigned int page, const void *data,
                          const void *spare)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  DEBUGASSERT(raw);

#ifndef CONFIG_MTD_NAND_BLOCKCHECK
  return nand_writepage_noecc(priv, block, page, data, spare);
#else
  DEBUGASSERT(raw->ecctype != NANDECC_SWECC);
  switch (raw->ecctype)
    {
    case NANDECC_NONE:
    case NANDECC_CHIPECC:
      return nand_writepage_noecc(priv, block, page, data, spare);

#ifdef NAND_HAVE_PMECC
    case NANDECC_PMECC:
      DEBUGASSERT(!spare);
      return nand_writepage_pmecc(priv, block, page, data);
#endif

    case NANDECC_SWECC:
    default:
      return -EINVAL;
    }
#endif
}
#endif

/****************************************************************************
 * Name: nand_reset
 *
 * Description:
 *   Resets a NAND FLASH device
 *
 * Input parameters:
 *   priv - Lower-half, private NAND FLASH device state
 *
 * Returned value.
 *   None
 *
 ****************************************************************************/

static void nand_reset(struct sam_nandcs_s *priv)
{
  fvdbg("Resetting\n");
  nand_nfc_configure(priv, 0, COMMAND_RESET, 0, 0, 0);
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
 * Input parameters:
 *   cs - Chip select number (in the event that multiple NAND devices
 *        are connected on-board).
 *
 * Returned value.
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

  fvdbg("CS%d\n", cs);

  /* Select the device structure */

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
      fdbg("ERROR: CS%d unsupported or invalid\n", cs);
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

  sem_init(&priv->waitsem, 0, 0);

  /* Initialize the NAND hardware */
  /* Perform board-specific SMC intialization for this CS */

  ret = board_nandflash_config(cs);
  if (ret < 0)
    {
      fdbg("ERROR: board_nandflash_config failed for CS%d: %d\n",
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
      fdbg("ERROR: CS%d nand_initialize failed %d\n", cs);
      return NULL;
    }

  /* Allocate a DMA channel for NAND transfers
   * REVISIT:  Need DMA channel setup.
   */

  if (nandmodel_getbuswidth(&priv->raw.model) == 16)
    {
      priv->dma = sam_dmachannel(1, DMA_FLAGS16);
    }
  else
    {
      priv->dma = sam_dmachannel(1, DMA_FLAGS8);
    }

  if (!priv->dma)
    {
      fdbg("ERROR: Failed to allocate the DMA channel for CS%d\n", cs);
    }

  /* Enable the NAND FLASH controller */

  if (!g_nand.initialized)
    {
      /* Enable the NAND FLASH Controller (The NFC is always used) */

      nand_putreg(SAM_HSMC_CTRL, HSMC_CTRL_NFCEN);

#ifdef NAND_HAVE_PMECC
      /* Perform one-time initialization of the PMECC */

      pmecc_initialize();

#else
      /* Disable the PMECC if it is not being used */

      nand_putreg(SAM_SMC_PMECCTRL, SMC_PMECCTRL_RST);
      nand_putreg(SAM_SMC_PMECCTRL, SMC_PMECCTRL_DISABLE);
      nand_putreg(SAM_SMC_PMECCFG, 0);
#endif

      g_nand.initialized = true;
    }

  /* Return the MTD wrapper interface as the MTD device */

  return mtd;
}

/****************************************************************************
 * Name: nand_checkreg
 *
 * Description:
 *   Check if the current HSMC register access is a duplicate of the preceding.
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

          lldbg("...[Repeats %d times]...\n", g_nand.ntimes);
        }

      /* Save information about the new access */

      g_nand.wr   = wr;
      g_nand.regval  = regval;
      g_nand.regadddr = regaddr;
      g_nand.ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif


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

#define STATUS_ERROR       (1 << 0)
#define STATUS_READY       (1 << 6)

/*
 * NFC ALE CLE command paramter
 */
#define SMC_ALE_COL_EN     (1 << 0)
#define SMC_ALE_ROW_EN     (1 << 1)
#define SMC_CLE_WRITE_EN   (1 << 2)
#define SMC_CLE_DATA_EN    (1 << 3)
#define SMC_CLE_VCMD2_EN   (1 << 4)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Low-level HSMC Helpers */

static void     nand_nfccmd_waitdone(void);
static void     nand_nfccmd_send(uint32_t cmd, uint32_t acycle,
                  uint32_t cycle0);
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

/* NAND Access Helpers */

static int      nand_readpage_noecc(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, void *data, void *spare);

#ifdef NAND_HAVE_HSIAO
static int      nand_readpage_hsiao(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, void *data, void *spare);
#endif

#ifdef NAND_HAVE_PMECC
static int      nand_readpage_pmecc(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, void *data);
#endif

static int      nand_writepage_noecc(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, const void *data, const void *spare);

#ifdef NAND_HAVE_HSIAO
static int      nand_writepage_hsiao(struct sam_nandcs_s *priv, off_t block,
                  unsigned int page, const void *data, const void *spare);
#endif

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

/* NAND regiser debug state */

#ifdef CONFIG_SAMA5_NAND_REGDEBUG
struct sam_nanddbg_s g_nanddbg;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nand_nfccmd_waitdone
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

static void nand_nfccmd_waitdone(void)
{
#warning Missing logic
}

/****************************************************************************
 * Name: nand_nfccmd_waitdone
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

static void nand_waitready(struct sam_nandcs_s *priv)
{
#ifdef SAMA5_NAND_READYBUSY
  while (board_nand_busy(priv->cs));
#endif
  nand_nfc_configure(priv, 0, COMMAND_STATUS, 0, 0, 0);
  while ((READ_DATA8(&priv->raw) & STATUS_READY) == 0);
}

/****************************************************************************
 * Name: nand_nfccmd_send
 *
 * Description:
 *   Use the HOST nandflash controller to send a command to the NFC.
 *
 * Input parameters:
 *   cmd    - command to send
 *   acycle - address cycle when command access id decoded
 *   cycle0 - address at first cycle
 *
 * Returned value.
 *   None
 *
 ****************************************************************************/

static void nand_nfccmd_send(uint32_t cmd, uint32_t acycle, uint32_t cycle0)
{
  uintptr_t cmdaddr;

  /* Wait until host controller is not busy. */

  while ((nand_getreg(NFCCMD_BASE + NFCADDR_CMD_NFCCMD) & 0x8000000) != 0);

  /* Send the command plus the ADDR_CYCLE */

  cmdaddr = NFCCMD_BASE + cmd;
  nand_putreg(SAM_HSMC_ADDR, cycle0);
  nand_putreg(cmdaddr, acycle);

  /* Wait for the command transfer to complete */

  nand_nfccmd_waitdone();
}

/****************************************************************************
 * Name: nand_coladdr_write
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

  /* Issue CLE and ALE through EBI */

  if (!nand_nfc_enabled(priv))
    {
      WRITE_COMMAND8(&priv->raw, cmd1);
      if ((mode & SMC_ALE_COL_EN) == SMC_ALE_COL_EN)
        {
          nand_coladdr_write(priv, coladdr);
        }

      if ((mode & SMC_ALE_ROW_EN)== SMC_ALE_ROW_EN)
        {
          nand_rowaddr_write(priv, rowaddr);
        }

      if ((mode & SMC_CLE_VCMD2_EN) == SMC_CLE_VCMD2_EN)
        {
          /* When set, the CMD2 field is issued after the address cycle */

          WRITE_COMMAND8(&priv->raw, cmd2);
        }
    }

  /* Issue CLE and ALE using NFC */

  else
    {
      if ((mode & SMC_CLE_WRITE_EN) == SMC_CLE_WRITE_EN)
        {
          rw = NFCADDR_CMD_NFCWR;
        }
      else
        {
          rw = NFCADDR_CMD_NFCRD;
        }

      if (((mode & SMC_CLE_DATA_EN) == SMC_CLE_DATA_EN) &&
          nand_nfcsram_enabled(priv))
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

      nand_nfccmd_send( cmd, acycle1234, acycle0);
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
#warning Missing logic
  return -ENOSYS;
}

/****************************************************************************
 * Name: nand_readpage_hsiao
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.  HSIAO ECC is used
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

#ifdef NAND_HAVE_HSIAO
static int nand_readpage_hsiao(struct sam_nandcs_s *priv, off_t block,
             unsigned int page, void *data, void *spare)
{
#warning Missing logic
  return -ENOSYS;
}
#endif /* NAND_HAVE_HSIAO */

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
#warning Missing logic
  return -ENOSYS;
}

/****************************************************************************
 * Name: nand_writepage_hsaio
 *
 * Description:
 *   Writes the data and/or the spare area of a page on a NAND FLASH chip.
 *   HSIAO ECC calculations are performed.
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

#ifdef NAND_HAVE_HSIAO
static int nand_writepage_hsiao(struct sam_nandcs_s *priv, off_t block,
             unsigned int page, const void *data, const void *spare)
{
  int ret;

  /* Disable the PMECC */

  pmecc_disable();

  /* Perform write operation */
# warning Missing logic

  return ret;
}
#endif /* NAND_HAVE_HSIAO */

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
  int ret;

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

static int nand_eraseblock(struct nand_raw_s *raw, off_t block)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  uint32_t rawaddr;
  int ret = OK;

  DEBUGASSERT(raw);

  fvdbg("Block %d\n", (int)block);

  /* Calculate address used for erase */

  rawaddr = block * nandmodel_pagesperblock(&raw->model);

  /* Configure the NFC for the block erase */

  nand_nfc_configure(priv, (SMC_CLE_VCMD2_EN | SMC_ALE_ROW_EN),
                     COMMAND_ERASE_1, COMMAND_ERASE_2, 0, rawaddr);

  /* Wait for the erase operation to complete */

  nand_waitready(priv);
  if (!nand_operation_complete(priv))
    {
      fdbg("ERROR: Could not erase block %d\n", block);
      ret = -ENOEXEC;
    }

  return ret;
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

#ifdef NAND_HAVE_HSIAO
    case NANDECC_HSIAO:
      return nand_readpage_hsiao(priv, block, page, data, spare);
#endif

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

#ifdef NAND_HAVE_HSIAO
    case NANDECC_HSIAO:
      return nand_writepage_hsiao(priv, block, page, data, spare);
#endif

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
  priv->raw.eraseblock = nand_eraseblock;
  priv->raw.rawread    = nand_rawread;
  priv->raw.rawwrite   = nand_rawwrite;
#ifdef CONFIG_MTD_NAND_HWECC
  priv->raw.readpage   = nand_readpage;
  priv->raw.writepage  = nand_writepage;
#endif
  priv->cs             = cs;

  /* Initialize the NAND hardware */
  /* Perform board-specific SMC intialization for this CS */

  ret = board_nandflash_config(cs);
  if (ret < 0)
    {
      fdbg("ERROR: board_nandflash_config failed for CS%d: %d\n",
           cs, ret);
      return NULL;
    }

  /* Probe the NAND part.  On success, an MTD interface that wraps
   * our raw NAND interface is returned.
   */

  mtd = nand_initialize(&priv->raw);
  if (!mtd)
    {
      fdbg("ERROR: CS%d nand_initialize failed %d\n", cs);
      return NULL;
    }

#warning Missing logic

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
  if (wr      == g_nanddbg.wr &&      /* Same kind of access? */
      regval  == g_nanddbg.regval &&  /* Same regval? */
      regaddr == g_nanddbg.regadddr)  /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      g_nanddbg.ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (g_nanddbg.ntimes > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", g_nanddbg.ntimes);
        }

      /* Save information about the new access */

      g_nanddbg.wr   = wr;
      g_nanddbg.regval  = regval;
      g_nanddbg.regadddr = regaddr;
      g_nanddbg.ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif


/****************************************************************************
 * include/nuttx/mtd/mtd_onfi.c
 *
 * ONFI Support.  The Open NAND Flash Interface (ONFI) is an industry
 * Workgroup made up of more than 100 companies that build, design-in, or
 * enable NAND Flash memory. This file provides definitions for standardized
 * ONFI NAND interfaces.
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This ONFI logic was based largely on Atmel sample code with modifications
 * for better integration with NuttX.  The Atmel sample code has a BSD
 * compatibile license that requires this copyright notice:
 *
 *   Copyright (c) 2010, Atmel Corporation
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

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mtd/nand_model.h>
#include <nuttx/mtd/onfi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NAND status bit mask */

#define STATUS_BIT_0             0x01
#define STATUS_BIT_1             0x02
#define STATUS_BIT_5             0x20
#define STATUS_BIT_6             0x40

#define NAND_MFR_MICRON          0x2c

/* Nand flash commands */

#define NAND_CMD_RESET           0xff
#define NAND_CMD_READ0           0x00
#define NAND_CMD_READID          0x90
#define NAND_CMD_STATUS          0x70
#define NAND_CMD_READ_PARAM_PAGE 0xec
#define NAND_CMD_SET_FEATURE     0xef

#define EBICSA_EBI_DBPDC         (1 << 9)
#define EBICSA_NAND_D0_ON_D16    (1 << 24)

 /* Misc. definitions */
 
#define MAX_READ_STATUS_COUNT    100000 /* Read status timeout */
#define ONFI_PARAM_TABLE_SIZE    116    /* Not all 256 bytes are useful */

/* NAND access macros */

#define WRITE_NAND_COMMAND(d,a,c) \
  do { \
    *(volatile uint8_t *)((uintptr_t)(a) | (uintptr_t)(c)) = (uint8_t)(d); \
  } while (0)

#define WRITE_NAND_ADDRESS(d,a,b) \
  do { \
    *(volatile uint8_t *)((uintptr_t)(a) | (uintptr_t)(b)) = (uint8_t)(d); \
  } while (0)

#define READ_NAND(a) \
  ((*(volatile uint8_t *)(uint32_t)a))

#define WRITE_NAND(d,a) \
  do { \
    *(volatile uint8_t *)((uintptr_t)a) = (uint8_t)d; \
  } while(0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Describes memory organization block information in ONFI parameter page*/

struct onfi_pgparam_s
{   
  uint8_t manufacturer;   /* JEDEC manufacturer ID */
  uint8_t buswidth;       /* Bus width */
  uint8_t luns;           /* Number of logical units */
  uint8_t eccsize;        /* Number of bits of ECC correction */
  uint8_t model;          /* Device model */
  uint16_t sparesize;     /* Number of spare bytes per page */
  uint16_t pagesperblock; /* Number of pages per block */
  uint16_t blocksperlun;  /* Number of blocks per logical unit (LUN) */
  uint32_t pagesize;      /* Number of data bytes per page */
  uintptr_t cmdaddr;      /* Base address for NAND commands */
  uintptr_t addraddr;     /* Base address for NAND addresses */
  uintptr_t dataaddr;     /* NAND data address */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: onfi_readstatus
 *
 * Description:
 *   This function Reads the status register of the NAND device by issuing a
 *   0x70 command.
 *
 * Input Parameters:
 *   cmdaddr  - NAND command address base
 *   dataaddr - NAND data address
 *
 * Returned Value:
 *   OK        : The function completed operation successfully
 *  -EIO       : The function dif not complete operation successfully
 *  -ETIMEDOUT : A time out occurred before the operation completed
 *
 ****************************************************************************/

static int onfi_readstatus(uintptr_t cmdaddr, uintptr_t dataaddr)
{
  uint32_t timeout;
  uint8_t status;

  /* Issue command */

  WRITE_NAND_COMMAND(NAND_CMD_STATUS, dataaddr, cmdaddr);
  timeout = 0;

  while (timeout < MAX_READ_STATUS_COUNT)
    {
      /* Read status byte */

      status = READ_NAND(dataaddr);

      /* Check status. If status bit 6 = 1 device is ready */

      if ((status & STATUS_BIT_6) == STATUS_BIT_6)
        {
          /* If status bit 0 = 0 the last operation was successful */

          if ((status & STATUS_BIT_0) == 0)
            {
              return OK;
            }
          else
            {
              return -EIO;
            }
        }

      timeout++;
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: onfi_have_embeddedecc
 *
 * Description:
 *   This function check if the Nandflash has an embedded ECC controller.
 *
 * Input Parameters:
 *   handle - An ONFI handle previously created by onfi_create().
 *
 * Returned Value:
 *   True  - Internal ECC supported
 *   False - Internal ECC not supported.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_EMBEDDEDECC
bool onfi_have_embeddedecc(FAR struct onfi_pgparam_s *onfi)
{
  /* Check if the Nandflash has an embedded ECC controller.  Known memories
   * with this feature:
   *
   * - Manufacturer ID = 0x2c (Micron)
   * - Number of bits ECC = 0x04 (4-bit ECC means process 34nm)
   * - device size = 1Gb or 2Gb or 4Gb (Number of data bytes per page x
   *   Number of pages per block x Number of blocks per unit)
   */

  return ((onfi->manufacturer & NAND_MFR_MICRON) == NAND_MFR_MICRON &&
           onfi->eccsize == 4 &&
          (onfi->model == '1' || onfi->model == '2' || onfi->model == '4'));
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: onfi_compatible
 *
 * Description:
 *   This function read an the ONFI signature at address of 20h to detect
 *   if the device is ONFI compatiable.
 *
 * Input Parameters:
 *   cmdaddr  - NAND command address base
 *   addraddr - NAND address address base
 *   dataaddr - NAND data address
 *
 * Returned Value:
 *   True if ONFI compatible
 *
 ****************************************************************************/

bool onfi_compatible(uintptr_t cmdaddr, uintptr_t addraddr,
                     uintptr_t dataaddr)
{
  uint8_t parmtab[ONFI_PARAM_TABLE_SIZE];

  /* Check if the Nandflash is ONFI compliant */

  WRITE_NAND_COMMAND(NAND_CMD_READID, dataaddr, cmdaddr);
  WRITE_NAND_ADDRESS(0x20, dataaddr, addraddr);

  parmtab[0] = READ_NAND(dataaddr);
  parmtab[1] = READ_NAND(dataaddr);
  parmtab[2] = READ_NAND(dataaddr);
  parmtab[3] = READ_NAND(dataaddr);

  return
   (parmtab[0] == 'O' && parmtab[1] == 'N' &&
    parmtab[2] == 'F' && parmtab[3] == 'I');
}

/****************************************************************************
 * Name: onfi_create
 *
 * Description:
 *   If the addresses refer to a compatible ONFI device, then create the
 *   ONFI handle that can be used to interact with the device.
 *
 * Input Parameters:
 *   cmdaddr  - NAND command address base
 *   addraddr - NAND address address base
 *   dataaddr - NAND data address
 *
 * Returned Value:
 *   On success, a non-NULL ONFI handle is returned.  This handle must be
 *   freed by calling onfi_destroy with it is no longer needed.
 *   NULL is returned on any failure.  Failures include such things as
 *   memory allocation failures, ONFI incompatibility, timeouts, etc.
 *
 ****************************************************************************/

ONFI_HANDLE *onfi_create(uintptr_t cmdaddr, uintptr_t addraddr,
                         uintptr_t dataaddr)
{
  FAR struct onfi_pgparam_s *onfi;
  uint8_t parmtab[ONFI_PARAM_TABLE_SIZE];
  int i;

  fvdbg("cmdaddr=%08x addraddr=%08x dataaddr=%08x\n",
        (int)cmdaddr, (int)addraddr, (int)dataaddr);

  if (onfi_compatible(cmdaddr, addraddr, dataaddr))
    {
      /* Allocate the ONFI structure */

      onfi = (FAR struct onfi_pgparam_s *)kzalloc(sizeof(struct onfi_pgparam_s));
      if (!onfi)
        {
          fdbg("ERROR: Failed to allocate ONFI structure\n");
          return (ONFI_HANDLE)NULL;
        }

      /* Save the NAND base addresses */
 
      onfi->cmdaddr  = cmdaddr;
      onfi->addraddr = addraddr;
      onfi->dataaddr = dataaddr;

      /* Initialize the ONFI parameter table */

      memset(parmtab, 0xff, ONFI_PARAM_TABLE_SIZE);

      /* Perform Read Parameter Page command */

      WRITE_NAND_COMMAND(NAND_CMD_READ_PARAM_PAGE, dataaddr, cmdaddr);
      WRITE_NAND_ADDRESS(0x0, dataaddr, addraddr);

      /* Wait NF ready */

      onfi_readstatus(cmdaddr, dataaddr);

      /* Re-enable data output mode required after Read Status command */

      WRITE_NAND_COMMAND(NAND_CMD_READ0, dataaddr, cmdaddr);

      /* Read the parameter table */

      for (i = 0; i < ONFI_PARAM_TABLE_SIZE; i++)
        {
          parmtab[i] = READ_NAND(dataaddr);
        }

      for (i = 0; i < ONFI_PARAM_TABLE_SIZE; i++)
        {
          if (parmtab[i] != 0xff)
            {
              break;
            }
        }

      if (i == ONFI_PARAM_TABLE_SIZE)
        {
          kfree(onfi);
          return (ONFI_HANDLE)NULL;
        }
        
      /* JEDEC manufacturer ID */

      onfi->manufacturer = *(uint8_t *)(parmtab + 64);
      fvdbg("ONFI manufacturer %x \n\r", onfi->manufacturer);

      /* Bus width */

      onfi->buswidth = (*(uint8_t *)(parmtab + 6)) & 0x01;

      /* Get number of data bytes per page (bytes 80-83 in the param table) */

      onfi->pagesize =  *(uint32_t *)(void*)(parmtab + 80);
      fvdbg("ONFI pagesize %x \n\r", (unsigned int)onfi->pagesize);

      /* Get number of spare bytes per page (bytes 84-85 in the param table) */

      onfi->sparesize =  *(uint16_t *)(void*)(parmtab + 84);
      fvdbg("ONFI sparesize %x \n\r", (unsigned int)onfi->sparesize);

      /* Number of pages per block. */

      onfi->pagesperblock = *(uint32_t *)(void*)(parmtab + 92);

      /* Number of blocks per logical unit (LUN). */

      onfi->blocksperlun = *(uint32_t *)(void*)(parmtab + 96);

      /* Number of logical units. */

      onfi->luns = *(uint8_t *)(parmtab + 100);

      /* Number of bits of ECC correction */

      onfi->eccsize = *(uint8_t *)(parmtab + 112);
      fvdbg("ONFI eccsize %x \n\r", onfi->eccsize);

      /* Device model */

      onfi->model= *(uint8_t *)(parmtab + 49);

      fvdbg("Returning %p\n", onfi);
      return (ONFI_HANDLE)onfi;
    }

  return (ONFI_HANDLE)NULL;
}

/****************************************************************************
 * Name: onfi_destroy
 *
 * Description:
 *   Free resources allocated on onfi_create() when the ONFI handle was
 *   created.  Upon return, the ONFI handle is no longer valid and should not
 *   be used further.
 *
 * Input Parameters:
 *   handle - An ONFI handle previously created by onfi_create().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void onfi_destroy(ONFI_HANDLE handle)
{
  DEBUGASSERT(handle);
  kfree(handle);
}

/****************************************************************************
 * Name: onfi_embeddedecc
 *
 * Description:
 *   Enable or disable the NAND's embedded ECC controller.
 *
 * Input Parameters:
 *   handle - An ONFI handle previously created by onfi_create().
 *   enable - True: enabled the embedded ECC function; False: disable it
 *
 * Returned Value:
 *   True  - Internal ECC enabled or disabled successfully
 *   False - Internal ECC not supported.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_EMBEDDEDECC
bool onfi_embeddedecc(ONFI_HANDLE handle, bool enable)
{
  FAR struct onfi_pgparam_s *onfi = (FAR struct onfi_pgparam_s *)handle;
  DEBUGASSERT(onfi);

  /* Does the NAND supported the embedded ECC function? */

  if (onfi_have_embeddedecc(onfi))
    {
      /* Yes... enable or disable it */
      /* Perform common setup */

      WRITE_NAND_COMMAND(NAND_CMD_SET_FEATURE, onfi->dataaddr, onfi->cmdaddr);
      WRITE_NAND_ADDRESS(0x90, onfi->dataaddr, onfi->addraddr);

      if (enable)
        {
          /* Activate the internal ECC controller */

          WRITE_NAND(0x08, onfi->dataaddr);
          WRITE_NAND(0x00, onfi->dataaddr);
          WRITE_NAND(0x00, onfi->dataaddr);
          WRITE_NAND(0x00, onfi->dataaddr);
          setSmcOpEccType(SMC_ECC_INTERNAL);
        }
      else
        {
          /* De-activate the internal ECC controller */

          WRITE_NAND(0x00, onfi->dataaddr);
          WRITE_NAND(0x00, onfi->dataaddr);
          WRITE_NAND(0x00, onfi->dataaddr);
          WRITE_NAND(0x00, onfi->dataaddr);
        }

      return true;
    }

  return false;
}
#endif

/****************************************************************************
 * Name: onfi_ebidetect
 *
 * Description:
 *   Detect Nand connection on EBI
 *
 * Input Parameters:
 *   cmdaddr  - NAND command address base
 *   addraddr - NAND address address base
 *   dataaddr - NAND data address
 *
 * Returned Value:
 *   True if the chip is detected; false otherwise.
 *
 ****************************************************************************/

bool onfi_ebidetect(uintptr_t cmdaddr, uintptr_t addraddr,
                    uintptr_t dataaddr)
{
  uint32_t timer;
  uint8_t rc;
  bool found = 0;
  uint8_t ids[4];
  uint8_t i;

  fvdbg("cmdaddr=%08x addraddr=%08x dataaddr=%08x\n",
        (int)cmdaddr, (int)addraddr, (int)dataaddr);

  /* Send Reset command */

  WRITE_NAND_COMMAND(NAND_CMD_RESET, dataaddr, cmdaddr);

  /* If a Nandflash is connected, it should answer to a read status command */

  for (timer = 0; timer < 60; timer++)
    {
      rc = onfi_readstatus(cmdaddr, dataaddr);
      if (rc == OK)
        {
          WRITE_NAND_COMMAND(NAND_CMD_READID, dataaddr, cmdaddr);
          WRITE_NAND_ADDRESS(0, dataaddr, addraddr);

          ids[0] = READ_NAND(dataaddr);
          ids[1] = READ_NAND(dataaddr);
          ids[2] = READ_NAND(dataaddr);
          ids[3] = READ_NAND(dataaddr);

          for (i = 0; i< NAND_NMODELS ; i++)
            {
              if (g_nandmodels[i].devid == ids[1])
                {
                  found = true;
                  break;
                }
            }

          break;
        }
    }

  if (!found)
    {
      if (onfi_compatible(cmdaddr, addraddr, dataaddr))
        {
          /* Report true if it is an ONFI device that is not in device
           * list (perhaps it is a new device that is ONFI campatible
           */

          found = true;
       }
    }

  return found;
}

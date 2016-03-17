/****************************************************************************
 * include/nuttx/qspi/qspi.h
 *
 *   Copyright(C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_QSPI_QSPI_H
#define __INCLUDE_NUTTX_QSPI_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Access macros ************************************************************/

/****************************************************************************
 * Name: QSPI_LOCK
 *
 * Description:
 *   On QSPI busses where there are multiple devices, it will be necessary to
 *   lock QSPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the QSPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the QSPI is properly
 *   configured for the device.  If the QSPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock qspi bus, false: unlock QSPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define QSPI_LOCK(d,l) (d)->ops->lock(d,l)

/****************************************************************************
 * Name: QSPI_SETFREQUENCY
 *
 * Description:
 *   Set the QSPI frequency. Required.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The QSPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

#define QSPI_SETFREQUENCY(d,f) ((d)->ops->setfrequency(d,f))

/****************************************************************************
 * Name: QSPI_SETMODE
 *
 * Description:
 *   Set the QSPI mode. Optional.  See enum qspi_mode_e for mode definitions.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The QSPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define QSPI_SETMODE(d,m) (d)->ops->setmode(d,m)

/****************************************************************************
 * Name: QSPI_SETBITS
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests.
 *           If value is greater > 0 then it implies MSB first
 *           If value is below < 0, then it implies LSB first with -nbits
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define QSPI_SETBITS(d,b) (d)->ops->setbits(d,b)

/****************************************************************************
 * Name: QSPI_COMMAND
 *
 * Description:
 *   Perform one QSPI command transfer
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   cmdinfo - Describes the command transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

#define QSPI_COMMAND(d,c) (d)->ops->command(d,c)

/* QSPI Command Transfer Flags */

#define QSPICMD_ADDRESS       (1 << 0)  /* Bit 0: Enable address transfer */
#define QSPICMD_READDATA      (1 << 1)  /* Bit 1: Enable read data transfer */
#define QSPICMD_WRITEDATA     (1 << 2)  /* Bit 2: Enable write data transfer */

#define QSPICMD_ISADDRESS(f)  (((f) & QSPICMD_ADDRESS) != 0)
#define QSPICMD_ISDATA(f)     (((f) & (QSPICMD_READDATA | QSPICMD_WRITEDATA)) != 0)
#define QSPICMD_ISREAD(f)     (((f) & QSPICMD_READDATA) != 0)
#define QSPICMD_ISWRITE(f)    (((f) & QSPICMD_WRITEDATA) != 0)

/****************************************************************************
 * Name: QSPI_MEMORY
 *
 * Description:
 *   Perform one QSPI memory transfer
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   meminfo - Describes the memory transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

#define QSPI_MEMORY(d,m) (d)->ops->memory(d,m)

/* QSPI Memory Transfer Flags */

#define QSPIMEM_READ          (0)       /* Bit 2: 0=Memory read data transfer */
#define QSPIMEM_WRITE         (1 << 2)  /* Bit 2: 1=Memory write data transfer */
#define QSPIMEM_DUALIO        (1 << 3)  /* Bit 3: Use Dual I/O (READ only) */
#define QSPIMEM_QUADIO        (1 << 4)  /* Bit 4: Use Quad I/O (READ only) */
#define QSPIMEM_SCRAMBLE      (1 << 5)  /* Bit 5: Scramble data */
#define QSPIMEM_RANDOM        (1 << 6)  /* Bit 6: Use random key in scrambler */

#define QSPIMEM_ISREAD(f)     (((f) & QSPIMEM_WRITE) == 0)
#define QSPIMEM_ISWRITE(f)    (((f) & QSPIMEM_WRITE) != 0)
#define QSPIMEM_ISDUALIO(f)   (((f) & QSPIMEM_DUALIO) != 0)
#define QSPIMEM_ISQUADIO(f)   (((f) & QSPIMEM_QUADIO) != 0)
#define QSPIMEM_ISSCRAMBLE(f) (((f) & QSPIMEM_SCRAMBLE) != 0)

#define QSPIMEM_ISRANDOM(f) \
  (((f) & (QSPIMEM_SCRAMBLE|QSPIMEM_RANDOM)) == \
          (QSPIMEM_SCRAMBLE|QSPIMEM_RANDOM))

/****************************************************************************
 * Name: QSPI_ALLOC
 *
 * Description:
 *   Allocate a buffer suitable for DMA data transfer
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buflen - Buffer length to allocate in bytes
 *
 * Returned Value:
 *   Address of tha allocated memory on success; NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

#define QSPI_ALLOC(d,b) (d)->ops->alloc(d,b)

/****************************************************************************
 * Name: QSPI_FREE
 *
 * Description:
 *   Free memory returned by QSPI_ALLOC
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - Buffer previously allocated via QSPI_ALLOC
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#define QSPI_FREE(d,b) (d)->ops->free(d,b)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Certain QSPI devices may required different clocking modes */

enum qspi_mode_e
{
  QSPIDEV_MODE0 = 0,     /* CPOL=0 CHPHA=0 */
  QSPIDEV_MODE1,         /* CPOL=0 CHPHA=1 */
  QSPIDEV_MODE2,         /* CPOL=1 CHPHA=0 */
  QSPIDEV_MODE3          /* CPOL=1 CHPHA=1 */
};

/* This structure describes one command transfer */

struct qspi_cmdinfo_s
{
  uint8_t   flags;       /* See QSPICMD_* definitions */
  uint8_t   addrlen;     /* Address length in bytes (if QSPICMD_ADDRESS) */
  uint16_t  cmd;         /* Command */
  uint16_t  buflen;      /* Data buffer length in bytes (if QSPICMD_DATA) */
  uint32_t  addr;        /* Address (if QSPICMD_ADDRESS) */
  FAR void *buffer;      /* Data buffer (if QSPICMD_DATA) */
};

/* This structure describes one memory transfer */

struct qspi_meminfo_s
{
  uint8_t   flags;       /* See QSPMEM_* definitions */
  uint8_t   addrlen;     /* Address length in bytes */
  uint8_t   dummies;     /* Number of dummy read cycles (READ only) */
  uint16_t  buflen;      /* Data buffer length in bytes */
  uint16_t  cmd;         /* Memory access command */
  uint32_t  addr;        /* Memory Address */
  uint32_t  key;         /* Scrambler key */
  FAR void *buffer;      /* Data buffer */
};

/* The QSPI vtable */

struct qspi_dev_s;
struct qspi_ops_s
{
  CODE int       (*lock)(FAR struct qspi_dev_s *dev, bool lock);
  CODE uint32_t  (*setfrequency)(FAR struct qspi_dev_s *dev,
                    uint32_t frequency);
  CODE void      (*setmode)(FAR struct qspi_dev_s *dev,
                    enum qspi_mode_e mode);
  CODE void      (*setbits)(FAR struct qspi_dev_s *dev, int nbits);
  CODE int       (*command)(FAR struct qspi_dev_s *dev,
                    FAR struct qspi_cmdinfo_s *cmdinfo);
  CODE int       (*memory)(FAR struct qspi_dev_s *dev,
                    FAR struct qspi_meminfo_s *meminfo);
  CODE FAR void *(*alloc)(FAR struct qspi_dev_s *dev, size_t buflen);
  CODE void      (*free)(FAR struct qspi_dev_s *dev, FAR void *buffer);
};

/* QSPI private data.  This structure only defines the initial fields of the
 * structure visible to the QSPI client.  The specific implementation may
 * add additional, device specific fields
 */

struct qspi_dev_s
{
  FAR const struct qspi_ops_s *ops;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_QSPI_QSPI_H */

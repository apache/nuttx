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
 *   xfrinfo - Describes the command transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

#define QSPI_COMMAND(d,x) (d)->ops->command(d,x)

/* QSPI Transfer Flags */

#define QSPIXFR_ADDRESS      (1 << 0)  /* Enable address transfer */
#define QSPIXFR_READDATA     (1 << 1)  /* Enable read data transfer */
#define QSPIXFR_WRITEDATA    (1 << 2)  /* Enable write data transfer */

#define QSPIXFR_ISADDRESS(f) (((f) & QSPIXFR_ADDRESS) != 0)
#define QSPIXFR_ISDATA(f)    (((f) & (QSPIXFR_READDATA | QSPIXFR_WRITEDATA)) != 0)
#define QSPIXFR_ISREAD(f)    (((f) & QSPIXFR_READDATA) != 0)
#define QSPIXFR_ISWRITE(f)   (((f) & QSPIXFR_WRITEDATA) != 0)

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

/* This structure describes one transfer */

struct qspi_xfrinfo_s
{
  uint8_t  flags;        /* See QSPIXFR_* definitions */
  uint8_t  addrlen;      /* Address length in bytes (if QSPIXFR_ADDRESS) */
  uint16_t cmd;          /* Command */
  uint16_t buflen;       /* Data buffer length in bytes (if QSPIXFR_DATA) */
  uint32_t addr;         /* Address (if QSPIXFR_ADDRESS) */
  FAR void *buffer;      /* Data buffer (if QSPIXFR_DATA) */
};

/* The QSPI vtable */

struct qspi_dev_s;
struct qspi_ops_s
{
  CODE int      (*lock)(FAR struct qspi_dev_s *dev, bool lock);
  CODE uint32_t (*setfrequency)(FAR struct qspi_dev_s *dev, uint32_t frequency);
  CODE void     (*setmode)(FAR struct qspi_dev_s *dev, enum qspi_mode_e mode);
  CODE void     (*setbits)(FAR struct qspi_dev_s *dev, int nbits);
  CODE int      (*command)(FAR struct qspi_dev_s *dev,
                   FAR struct qspi_xfrinfo_s *xfrinfo);
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

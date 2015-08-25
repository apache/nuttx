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
 *   Send a command to the QSPI device
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   cmd - The command to send.  the size of the data is determined by the
 *         number of bits selected for the QSPI interface.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

#define QSPI_COMMAND(d,c) ((d)->ops->command(d,(uint16_t)c))

/****************************************************************************
 * Name: QSPI_COMMAND_WRITE
 *
 * Description:
 *   Send a command then send a block of data.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   cmd    - The command to send.  the size of the data is determined by
 *            the number of bits selected for the QSPI interface.
 *   buffer - A pointer to the buffer of data to be sent
 *   buflen - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the QSPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

#define QSPI_COMMAND_WRITE(d,c,b,l) ((d)->ops->command_write(d,c,b,l))

/****************************************************************************
 * Name: QSPI_COMMAND_READ
 *
 * Description:
 *   Receive a block of data from QSPI. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   cmd    - The command to send.  the size of the data is determined by
 *            the number of bits selected for the QSPI interface.
 *   buffer - A pointer to the buffer in which to receive data
 *   buflen - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-
 *            per-word selected for the QSPI interface.  If nbits <= 8, the
 *            data is packed into uint8_t's; if nbits >8, the data is packed
 *            into uint16_t's
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

#define QSPI_COMMAND_READ(d,c,b,l) ((d)->ops->command_read(d,c,b,l))

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

/* The QSPI vtable */

struct qspi_dev_s;
struct qspi_ops_s
{
  CODE int      (*lock)(FAR struct qspi_dev_s *dev, bool lock);
  CODE uint32_t (*setfrequency)(FAR struct qspi_dev_s *dev, uint32_t frequency);
  CODE void     (*setmode)(FAR struct qspi_dev_s *dev, enum qspi_mode_e mode);
  CODE void     (*setbits)(FAR struct qspi_dev_s *dev, int nbits);
  CODE int      (*command)(FAR struct qspi_dev_s *dev, uint16_t command);
  CODE int      (*command_write)(FAR struct qspi_dev_s *dev, uint16_t cmd,
                  FAR const void *buffer, size_t buflen);
  CODE int      (*command_read)(FAR struct qspi_dev_s *dev, uint16_t cmd,
                  FAR void *buffer, size_t buflen);
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

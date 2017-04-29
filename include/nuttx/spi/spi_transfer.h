/****************************************************************************
 * include/nuttx/spi/spi_transfer.h
 *
 *   Copyright(C) 2015-2016 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_SPI_TRANSFER_H
#define __INCLUDE_NUTTX_SPI_TRANSFER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

#ifdef CONFIG_SPI_EXCHANGE

/* SPI Character Driver IOCTL Commands **************************************/
/* The SPI driver is intended to support application testing of the SPI bus.
 * The SPI driver simply provides a user-accessible wrapper around the
 * OS internal spi_transfer() function.  The following IOCTL commands to
 * supported by the SPI driver to perform SPI transfers
 */

/* Command:      SPIIOC_TRANSFER
 * Description:  Perform a sequence of SPI transfers
 * Argument:     A reference to an instance of struct spi_sequence_s.
 * Dependencies: CONFIG_SPI_DRIVER
 */

#define SPIIOC_TRANSFER _SPIIOC(0x0001)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes one SPI transaction as handled by spi_transfer() */

struct spi_trans_s
{
  /* SPI attributes for unique to this transaction */

  bool deselect;            /* De-select after transfer */
#ifdef CONFIG_SPI_CMDDATA
  bool cmd;                 /* true=command; false=data */
#endif
#ifdef CONFIG_SPI_HWFEATURES
  spi_hwfeatures_t hwfeat;  /* H/W features to enable on this transfer */
#endif
  useconds_t delay;         /* Microsecond delay after transfer */

  /* These describe the single data transfer */

  size_t nwords;            /* Number of words in transfer */
  FAR const void *txbuffer; /* Source buffer for TX transfer */
  FAR void *rxbuffer;       /* Sink buffer for RX transfer */
};

/* This describes a sequence of SPI transactions as handled by spi_transfer.
 *
 * Example usage:
 *   struct spi_trans_s mytrans[5];
 *   struct spi_sequence_s myseq;
 *   ...
 *   myseq.ntrans = 5;
 *   myseq.trans  = mytrans;
 *   ...
 *   int ret = spi_transfer(spi, myseq);
 *   ...
 */

struct spi_sequence_s
{
  /* Properties that are fixed throughout the transfer */

  uint32_t dev;                /* See enum spi_devtype_e */
  uint8_t mode;                /* See enum spi_mode_e */
  uint8_t nbits;               /* Number of bits */
  uint8_t ntrans;              /* Number of transactions */
  uint32_t frequency;          /* SPI frequency (Hz) */
#ifdef CONFIG_SPI_CS_DELAY_CONTROL
  uint32_t a;                  /* Arguments to setdelay() */
  uint32_t b;
  uint32_t c;
#endif

  /* A pointer to the list of transfers to be be performed. */

  FAR struct spi_trans_s *trans;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_transfer
 *
 * Description:
 *   This is a helper function that can be used to encapsulate and manage
 *   a sequence of SPI transfers.  The SPI bus will be locked and the
 *   SPI device selected for the duration of the transfers.
 *
 * Input Parameters:
 *   spi - An instance of the SPI device to use for the transfer
 *   seq - Describes the sequence of transfers.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int spi_transfer(FAR struct spi_dev_s *spi, FAR struct spi_sequence_s *seq);

/****************************************************************************
 * Name: spi_register
 *
 * Description:
 *   Create and register the SPI character driver.
 *
 *   The SPI character driver is a simple character driver that supports SPI
 *   transfers.  The intent of this driver is to support SPI testing.  It is
 *   not suitable for use in any real driver application.
 *
 * Input Parameters:
 *   spi - An instance of the lower half SPI driver
 *   bus - The SPI bus number.  This will be used as the SPI device minor
 *     number.  The SPI character device will be registered as /dev/spiN
 *     where N is the minor number
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_DRIVER
int spi_register(FAR struct spi_dev_s *spi, int bus);
#endif

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_SPI_EXCHANGE */
#endif /* __INCLUDE_NUTTX_SPI_TRANSFER_H */

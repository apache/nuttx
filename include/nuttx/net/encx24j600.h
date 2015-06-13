/****************************************************************************
 * include/nuttx/net/encx24j600.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_ENCX24J600_H
#define __INCLUDE_NUTTX_NET_ENCX24J600_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ENCX24J600 Configuration Settings:
 *
 * CONFIG_ENCX24J600 - Enabled ENCX24J600 support
 * CONFIG_ENCX24J600_SPIMODE - Controls the SPI mode
 * CONFIG_ENCX24J600_FREQUENCY - Define to use a different bus frequency
 * CONFIG_ENCX24J600_NINTERFACES - Specifies the number of physical ENCX24J600
 *   devices that will be supported.
 * CONFIG_ENCX24J600_STATS - Collect network statistics
 * CONFIG_ENCX24J600_HALFDUPPLEX - Default is full duplex
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure returns driver statistics (if enabled) */

#ifdef CONFIG_ENCX24J600_STATS
struct enc_stats_s
{
  uint8_t  maxpktcnt;         /* Max. number of buffered RX packets */
  uint32_t txrequests;        /* Number of TX packets queued */
  uint32_t txifs;             /* TXIF completion events */
  uint32_t txabrts;           /* TXIF completions with ESTAT.TXABRT */
  uint32_t txerifs;           /* TXERIF error events */
  uint32_t txtimeouts;        /* S/W detected TX timeouts */
  uint32_t pktifs;            /* PKTIF RX completion events */
  uint32_t rxnotok;           /* PKTIF without RXSTAT_OK */
  uint32_t rxpktlen;          /* PKTIF with bad pktlen */
  uint32_t rxerifs;           /* RXERIF error evernts */
};
#endif

/* The ENCX24J600 normal provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanixm for controlling
 * the ENCX24J600 GPIO interrupt.
 *
 * The ENC32J60 interrupt is an active low, *level* interrupt. "When an
 * interrupt occurs, the interrupt flag is set. If the interrupt is enabled
 * in the EIE register and the INTIE global interrupt enable bit is set, the
 * INT pin will be driven low"
 *
 * "When an enabled interrupt occurs, the interrupt pin will remain low until
 * all flags which are causing the interrupt are cleared or masked off
 * (enable bit is cleared) by the host controller."  However, the interrupt
 * will behave like a falling edge interrupt because "After an interrupt
 * occurs, the host controller [clears] the global enable bit for the
 * interrupt pin before servicing the interrupt. Clearing the enable bit
 * will cause the interrupt pin to return to the non-asserted state (high).
 * Doing so will prevent the host controller from missing a falling edge
 * should another interrupt occur while the immediate interrupt is being
 * serviced."
 */

struct enc_lower_s
{
  int  (*attach)(FAR const struct enc_lower_s *lower, xcpt_t handler);
  void (*enable)(FAR const struct enc_lower_s *lower);
  void (*disable)(FAR const struct enc_lower_s *lower);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Function: enc_initialize
 *
 * Description:
 *   Initialize the Ethernet driver.  The ENCX24J600 device is assumed to be
 *   in the post-reset state upon entry to this function.
 *
 * Parameters:
 *   spi   - A reference to the platform's SPI driver for the ENCX24J600
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., ENCX24J600 GPIO interrupts).
 *   devno - If more than one ENCX24J600 is supported, then this is the
 *           zero based number that identifies the ENCX24J600;
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

struct spi_dev_s; /* see nuttx/spi/spi.h */
int enc_initialize(FAR struct spi_dev_s *spi,
                   FAR const struct enc_lower_s *lower, unsigned int devno);

/****************************************************************************
 * Function: enc_stats
 *
 * Description:
 *   Return accumulated ENCX24J600 statistics.  Statistics are cleared after
 *   being returned.
 *
 * Parameters:
 *   devno - If more than one ENCX24J600 is supported, then this is the
 *           zero based number that identifies the ENCX24J600;
 *   stats - The user-provided location to return the statistics.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_ENCX24J600_STATS
int enc_stats(unsigned int devno, struct enc_stats_s *stats);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_ENCX24J600_H */

/****************************************************************************
 * drivers/usbdev/sp.c
 *
 *   Copyright(C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * This logic emulates the Prolific PL2303 serial/USB converter
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

#ifndef __NUTTX_SPI_H
#define __NUTTX_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* SPI status bits -- Some dedicated for SPI MMC support and may have not
 * relationship to SPI other than needed by the SPI MMC interface
 */

#define SPI_STATUS_PRESENT     0x01 /* MMC card present */
#define SPI_STATUS_WRPROTECTED 0x02 /* MMC card write protected */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct spi_ops_s
{
  void   (*select)(boolean select);
  uint32 (*setclockfrequency)(uint32 frequency);
  ubyte  (*status)(void);
  ubyte  (*sndbyte)(ubyte ch);
  ubyte  (*waitready)(void);
  void   (*sndblock)(ubyte *data, int datlen);
  void   (*recvblock)(ubyte *data, int datlen);
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN void up_spinitialize(int port);

/****************************************************************************
 * Name: up_spigetvtable
 *
 * Description:
 *   Return the vtable for the selected SPI port
 *
 ****************************************************************************/

EXTERN FAR const struct spi_ops_s *up_spigetvtable(int port);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __NUTTX_SPI_H */

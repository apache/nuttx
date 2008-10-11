/****************************************************************************
 * drivers/usbdev/spi.h
 *
 *   Copyright(C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/* Access macros */

/****************************************************************************
 * Name:SPI_SELECT
 *
 * Description:
 *   Enable/disable the SPI chip select
 *
 * Input Parameters:
 *   select: TRUE: chip selected, FALSE: chip de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SPI_SELECT(s,b) ((s)->select(b))

/****************************************************************************
 * Name: SPI_SETFREQUENCY
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   frequency: The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

#define SPI_SETFREQUENCY(s,f) ((s)->setfrequency(f))

/****************************************************************************
 * Name: SPI_STATUS
 *
 * Description:
 *   Get SPI/MMC status
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines
 *
 ****************************************************************************/

#define SPI_STATUS(s) ((s)->status())

/****************************************************************************
 * Name: SPI_SNDBYTE
 *
 * Description:
 *   Send one byte on SPI
 *
 * Input Parameters:
 *   ch - the byte to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SPI_SNDBYTE(s,ch) ((s)->sndbyte(ch))

/****************************************************************************
 * Name: SPI_WAITREADY
 *
 * Description:
 *   Wait for SPI to be ready
 *
 * Input Parameters: None
 *
 * Returned Value:
 *   OK if no error occured; a negated errno otherwise.
 *
 ****************************************************************************/

#define SPI_WAITREADY(s) ((s)->waitready())

/****************************************************************************
 * Name: SPI_SNDBLOCK
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   data - A pointer to the buffer of data to be sent
 *   datlen - the length of data to send from the buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SPI_SNDBLOCK(s,d,l) ((s)->sndblock(d,l))

/****************************************************************************
 * Name: SPI_RECVBLOCK
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   data - A pointer to the buffer in which to recieve data
 *   datlen - the length of data that can be received in the buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SPI_RECVBLOCK(s,d,l) ((s)->recvblock(d,l))

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
  uint32 (*setfrequency)(uint32 frequency);
  ubyte  (*status)(void);
  void   (*sndbyte)(ubyte ch);
  int    (*waitready)(void);
  void   (*sndblock)(FAR const ubyte *data, int datlen);
  void   (*recvblock)(FAR ubyte *data, int datlen);
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

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid vtable pointer on succcess; a NULL on failure
 *
 ****************************************************************************/

EXTERN FAR const struct spi_ops_s *up_spiinitialize(int port);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __NUTTX_SPI_H */

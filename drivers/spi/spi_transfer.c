/****************************************************************************
 * drivers/spi/spi_transfer.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#ifdef CONFIG_SPI_EXCHANGE

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

int spi_transfer(FAR struct spi_dev_s *spi, FAR struct spi_sequence_s *seq)
{
  FAR struct spi_trans_s *trans;
  int ret = OK;
  int i;

  DEBUGASSERT(spi != NULL && seq != NULL && seq->trans != NULL);

  /* Get exclusive access to the SPI bus */

  SPI_LOCK(spi, true);

  /* Establish the fixed SPI attributes for all transfers in the sequence */

  SPI_SETFREQUENCY(spi, seq->frequency);

#ifdef CONFIG_SPI_CS_DELAY_CONTROL
  ret = SPI_SETDELAY(spi, seq->a, seq->b, seq->c);
  if (ret < 0)
    {
      spierr("ERROR: SPI_SETDELAY failed: %d\n", ret);
      SPI_LOCK(spi, false);
      return ret;
    }
#endif

  SPI_SETMODE(spi, seq->mode);
  SPI_SETBITS(spi, seq->nbits);

  /* Select the SPI device in preparation for the transfer.
   * REVISIT: This is redundant.
   */

  SPI_SELECT(spi, seq->dev, true);

  /* Then perform each transfer is the sequence */

  for (i = 0, trans = seq->trans; i < (int)seq->ntrans; i++, trans++)
    {
      /* Establish the fixed SPI attributes for unique to this transaction */

#ifdef CONFIG_SPI_HWFEATURES
      ret = SPI_HWFEATURES(spi, trans->hwfeat);
      if (ret < 0)
        {
          spierr("ERROR: SPI_HWFEATURES failed: %d\n", ret);
          break;
        }
#endif

#ifdef CONFIG_SPI_CMDDATA
      ret = SPI_CMDDATA(spi, seq->dev, trans->cmd);
      if (ret < 0)
        {
          spierr("ERROR: SPI_CMDDATA failed: %d\n", ret);
          break;
        }
#endif

      /* [Re-]select the SPI device in preparation for the transfer */

      SPI_SELECT(spi, seq->dev, true);

      /* Perform the transfer */

      SPI_EXCHANGE(spi, trans->txbuffer, trans->rxbuffer, trans->nwords);

      /* Possibly de-select the SPI device after the transfer */

      if (trans->deselect)
        {
          SPI_SELECT(spi, seq->dev, false);
        }

      /* Perform any requested inter-transfer delay */

      if (trans->delay > 0)
        {
          nxsig_usleep(trans->delay);
        }
    }

  SPI_SELECT(spi, seq->dev, false);
  SPI_LOCK(spi, false);
  return ret;
}

#endif /* CONFIG_SPI_EXCHANGE */

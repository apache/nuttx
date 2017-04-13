/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_sdpcm.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
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
#include <nuttx/compiler.h>

#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <stddef.h>

#include "bcmf_sdio.h"
#include "bcmf_core.h"
#include "bcmf_sdpcm.h"

/* SDA_FRAMECTRL */
#define SFC_RF_TERM  (1 << 0)  /* Read Frame Terminate */
#define SFC_WF_TERM  (1 << 1)  /* Write Frame Terminate */
#define SFC_CRC4WOOS (1 << 2)  /* CRC error for write out of sync */
#define SFC_ABORTALL (1 << 3)  /* Abort all in-progress frames */

/* tosbmailbox bits corresponding to intstatus bits */
#define SMB_NAK   (1 << 0)  /* Frame NAK */
#define SMB_INT_ACK (1 << 1)  /* Host Interrupt ACK */
#define SMB_USE_OOB (1 << 2)  /* Use OOB Wakeup */
#define SMB_DEV_INT (1 << 3)  /* Miscellaneous Interrupt */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bcmf_sdpcm_rxfail(FAR struct bcmf_dev_s *priv, bool retry);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int bcmf_sdpcm_rxfail(FAR struct bcmf_dev_s *priv, bool retry)
{
  /* issue abort command for F2 through F0 */

  bcmf_write_reg(priv, 0, SDIO_CCCR_IOABORT, 2);

  bcmf_write_reg(priv, 1, SBSDIO_FUNC1_FRAMECTRL, SFC_RF_TERM);

  /* TODO Wait until the packet has been flushed (device/FIFO stable) */

  /* Send NAK to retry to read frame */
  if (retry)
    {
      bcmf_write_sbregb(priv,
                  CORE_BUS_REG(priv->get_core_base_address(SDIOD_CORE_ID),
                  tosbmailbox), SMB_NAK);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bcmf_sdpcm_process_header(FAR struct bcmf_dev_s *priv,
                              struct bcmf_sdpcm_header *header)
{
  uint16_t len;

  len = header->frametag[0];

  if (header->data_offset < sizeof(struct bcmf_sdpcm_header) ||
      header->data_offset > len)
    {
      _err("Invalid data offset\n");
      bcmf_sdpcm_rxfail(priv, false);
      return -ENXIO;
    }

  /* Update tx credits */

  _info("update credit %x %x %x\n", header->credit,
                                    priv->tx_seq, priv->max_seq);

  if (header->credit - priv->tx_seq > 0x40)
    {
      _err("seq %d: max tx seq number error\n", priv->tx_seq);
      priv->max_seq = priv->tx_seq + 2;
    }
  else
    {
      priv->max_seq = header->credit;
    }

  return OK;
}

// FIXME remove
uint8_t tmp_buffer[512];
int bcmf_sdpcm_readframe(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint16_t len, checksum;
  struct bcmf_sdpcm_header *header = (struct bcmf_sdpcm_header*)tmp_buffer;

  /* Read header */

  ret = bcmf_transfer_bytes(priv, false, 2, 0, (uint8_t*)header, 4);
  if (ret != OK)
    {
      _info("failread size\n");
      ret = -EIO;
      goto exit_abort;
    }

  len = header->frametag[0];
  checksum = header->frametag[1];

  /* All zero means no more to read */

  if (!(len | checksum))
    {
      return -ENODATA;
    }

  if (((~len & 0xffff) ^ checksum) || len < sizeof(struct bcmf_sdpcm_header))
    {
      _err("Invalid header checksum or len %x %x\n", len, checksum);
      ret = -EIO;
      goto exit_abort;
    }

  // FIXME define for size
  if (len > 512)
    {
      _err("Frame is too large, cancel %d\n", len);
      ret = -ENOMEM;
      goto exit_abort;
    }

  /* Read remaining frame data */

  ret = bcmf_transfer_bytes(priv, false, 2, 0, (uint8_t*)header+4, len - 4);
  if (ret != OK)
    {
      ret = -EIO;
      goto exit_abort;
    }

  /* Process and validate header */

  ret = bcmf_sdpcm_process_header(priv, header);
  if (ret != OK)
    {
      _err("Error while processing header %d\n", ret);
    }

  return ret;

exit_abort:
  bcmf_sdpcm_rxfail(priv, false);
  return ret;
}

int bcmf_sdpcm_iovar_data_get(FAR struct bcmf_dev_s *priv, char *name,
                              void *data, unsigned int len)
{
  // TODO implement
  return -EINVAL;
}

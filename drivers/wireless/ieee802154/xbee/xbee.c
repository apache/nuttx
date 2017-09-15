/****************************************************************************
 * drivers/wireless/xbee/drivers/xbee.c
 *
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author:  Anthony Merlino <anthony@vergeaero.com>
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
 *
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

#include <assert.h>
#include <debug.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <semaphore.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mm/iob.h>

#include "xbee.h"
#include "xbee_mac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  xbee_interrupt(int irq, FAR void *context, FAR void *arg);
static void xbee_attnworker(FAR void *arg);
static bool xbee_validate_apiframe(uint8_t frametype, uint16_t framelen);
static bool xbee_verify_checksum(FAR const struct iob_s *iob);
static void xbee_process_apiframes(FAR struct xbee_priv_s *priv,
              FAR struct iob_s *iob);
static void xbee_process_txstatus(FAR struct xbee_priv_s *priv, uint8_t frameid,
              uint8_t status);
static void xbee_process_rxframe(FAR struct xbee_priv_s *priv,
              FAR struct iob_s *frame,
              enum ieee802154_addrmode_e addrmode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int xbee_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  priv->attn_latched = true;

  /* In complex environments, we cannot do SPI transfers from the interrupt
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  if (work_available(&priv->attnwork))
    {
      return work_queue(HPWORK, &priv->attnwork, xbee_attnworker, (FAR void *)priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: xbee_attnworker
 *
 * Description:
 *   Perform interrupt handling (Attention) logic outside of the interrupt handler
 *   (on the work queue thread).
 *
 * Parameters:
 *   arg     - The reference to the driver structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void xbee_attnworker(FAR void *arg)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)arg;
  FAR struct iob_s *iobhead = NULL;
  FAR struct iob_s *iob = NULL;
  FAR struct iob_s *previob = NULL;
  uint16_t rxframelen = 0;

  DEBUGASSERT(priv);
  DEBUGASSERT(priv->spi);

  /* NOTE: There is a helpful side-effect to trying to get the SPI Lock here
  * even when there is a write going on. That is, if the SPI write are on a
  * thread with lower priority, trying to get the lock here should boost the
  * priority of that thread, helping move along the low-level driver work
  * that really should be happening in a high priority way anyway.
  */

  SPI_LOCK(priv->spi, 1);
  SPI_SETBITS(priv->spi, 8);
  SPI_SETMODE(priv->spi, SPIDEV_MODE0);
  SPI_SETFREQUENCY(priv->spi, CONFIG_IEEE802154_XBEE_FREQUENCY);

  /* Assert CS */

  SPI_SELECT(priv->spi, SPIDEV_IEEE802154(0), true);

  /* Check to make sure all the data hasn't already been clocked in and
   * we just need to process it.
   */

  if (priv->attn_latched && !priv->lower->poll(priv->lower))
    {
      priv->attn_latched = false;
    }

  /* Allocate an IOB for the incoming data. */

  iob             = iob_alloc(false);
  iob->io_flink   = NULL;
  iob->io_len     = 0;
  iob->io_offset  = 0;
  iob->io_pktlen  = 0;

  /* Keep a reference to the first IOB.  If we need to allocate more than
   * one to hold each API frame, then we will still have this reference to
   * the head of the list
   */

  iobhead = iob;

  if (priv->attn_latched)
    {
      while (priv->lower->poll(priv->lower))
        {
          DEBUGASSERT(iob->io_len <= CONFIG_IOB_BUFSIZE);

          SPI_RECVBLOCK(priv->spi, &iob->io_data[iob->io_len], 1);

          switch (iob->io_len)
            {
              case XBEE_APIFRAMEINDEX_STARTBYTE:
                {
                  if (iob->io_data[iob->io_len] == XBEE_STARTBYTE)
                    {
                      iob->io_len++;
                    }
                }
                break;
              case XBEE_APIFRAMEINDEX_LENGTHMSB:
                {
                  rxframelen = iob->io_data[iob->io_len++] << 8;
                }
                break;
              case XBEE_APIFRAMEINDEX_LENGTHLSB:
                {
                  rxframelen |= iob->io_data[iob->io_len++];
                  rxframelen += XBEE_APIFRAME_OVERHEAD;
                }
                break;
              case XBEE_APIFRAMEINDEX_TYPE:
                {
                  /* Check that the length and frame type make sense together */

                  if (!xbee_validate_apiframe(iob->io_data[iob->io_len],
                                              rxframelen - XBEE_APIFRAME_OVERHEAD))
                    {
                      wlwarn("invalid length on incoming API frame. Dropping!\n");
                      iob->io_len = 0;
                    }
                  else
                    {
                      iob->io_len++;
                    }
                }
                break;
              default:
                {
                  if (iob->io_len == rxframelen - 1)
                    {
                      iob->io_len++;
                      if (xbee_verify_checksum(iob))
                        {
                          /* This API frame is complete. Allocate a new IOB
                           * and link it to the existing one. When we are all
                           * finished we will pass this IOB list along for
                           * processing.
                           */

                          iob->io_flink = iob_alloc(false);
                          iob = iob->io_flink;

                          iob->io_flink  = NULL;
                          iob->io_len    = 0;
                          iob->io_offset = 0;
                          iob->io_pktlen = 0;
                        }
                      else
                        {
                          wlwarn("invalid checksum on incoming API frame. Dropping!\n");
                          iob->io_len = 0;
                        }
                    }
                  else
                    {
                      iob->io_len++;
                    }
                }
                break;
            }
        }

      priv->attn_latched = false;
    }

  /* The last IOB in the list (or the only one) may be able to be freed since
   * it may not have any valid data. If it contains some data, but not a whole
   * API frame, something is wrong, so we just warn the user and drop the
   * data.  If the data was valid, the ATTN line should have stayed asserted
   * until all the data was clocked in. So if we don't have a full frame,
   * we can only drop it.
   */

  if (iob->io_len < XBEE_APIFRAME_OVERHEAD || iob->io_len != rxframelen)
    {
      if (iobhead == iob)
        {
          iobhead = NULL;
        }
      else
        {
          previob = iobhead;
          while (previob->io_flink != iob)
            {
              previob = previob->io_flink;
            }
          previob->io_flink = NULL;
        }

      if (iob->io_len > 0)
        {
          wlwarn("Partial API frame clocked in. Dropping!\n");
        }

      iob_free(iob);
    }

  if (iobhead != NULL)
    {
      if (priv->rx_apiframes == NULL)
        {
          priv->rx_apiframes = iobhead;
        }
      else
        {
          iob = priv->rx_apiframes;
          while (iob->io_flink != NULL)
            {
              iob = iob->io_flink;
            }

           iob->io_flink = iobhead;
        }
    }

  /* Before unlocking the SPI bus, we "detach" the IOB list from the private
   * struct and keep a copy. When the SPI bus becomes free, more data can
   * be clocked in from an SPI write. By detaching the IOB list, we can process
   * the incoming data without holding up the SPI bus
   */

  iobhead = priv->rx_apiframes;
  priv->rx_apiframes = NULL;

  /* De-assert CS */

  SPI_SELECT(priv->spi, SPIDEV_IEEE802154(0), false);

  /* Relinquish control of the SPI Bus */

  SPI_LOCK(priv->spi, 0);

  if (iobhead != NULL)
    {
      xbee_process_apiframes(priv, iobhead);
    }
}

/****************************************************************************
 * Name: xbee_validate_apiframe
 *
 * Description:
 *   Verifies that the API frame type is known and that the length makes
 *   sense for that frame type.
 *
 * Parameters:
 *    frame - pointer to the frame data
 *    datalen - The size of the data section of the frame.  This is the value
 *              included as the second and third byte of the frame.
 *
 * Returns:
 *   true  - Frame type is known and length is logical
 *   false - Frame type is unknown or length is invalid for frame type
 *
 ****************************************************************************/

static bool xbee_validate_apiframe(uint8_t frametype, uint16_t datalen)
{
  switch (frametype)
    {
      case XBEE_APIFRAME_MODEMSTATUS:
        {
          if (datalen != 2)
            {
              return false;
            }
        }
        break;
      case XBEE_APIFRAME_ATRESPONSE:
        {
          return true;
        }
        break;
      case XBEE_APIFRAME_TXSTATUS:
        {
          if (datalen != 3)
            {
              return false;
            }
        }
        break;
      case XBEE_APIFRAME_RX_EADDR:
        {
          if (datalen < 14)
            {
              return false;
            }
        }
        break;
      case XBEE_APIFRAME_RX_SADDR:
        {
          if (datalen < 8)
            {
              return false;
            }
        }
        break;
      default:
        {
          return false;
        }
        break;
    }

  return true;
}

/****************************************************************************
 * Name: xbee_verify_checksum
 *
 * Description:
 *   Verifies API frame checksum.
 *
 * Parameters:
 *    frame - pointer to the frame data
 *    framelen - size of the overall frame. NOT the data length field
 *
 * Returns:
 *   true  - Checksum is valid
 *   false - Checksum is invalid
 *
 ****************************************************************************/

static bool xbee_verify_checksum(FAR const struct iob_s *iob)
{
  int i;
  uint8_t checksum = 0;

  DEBUGASSERT(iob->io_len > XBEE_APIFRAME_OVERHEAD);

  /* Skip the start byte and frame length, but include the checksum */

  for (i = 3; i < iob->io_len; i++)
  {
    checksum += iob->io_data[i];
  }

  if (checksum != 0xFF)
    {
      wlwarn("Invalid checksum\n");
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: xbee_process_apiframes
 *
 * Description:
 *   Processes a list of complete API frames.
 *
 * Assumptions:
 *   Frame has already been validated using frame type and frame length and
 *   the checksum has also been verified.
 *
 ****************************************************************************/

static void xbee_process_apiframes(FAR struct xbee_priv_s *priv,
                                   FAR struct iob_s *framelist)
{
  FAR struct ieee802154_notif_s *notif;
  FAR struct iob_s *frame;
  FAR struct iob_s *nextframe;
  FAR char *command;

  DEBUGASSERT(framelist != NULL);

  frame = framelist;

  /* At the end of each iteration, frame is set to the next frame in the list
   * and the IOB is freed. If the IOB is not supposed to be freed, per the
   * API frame type, the logic must update the frame to the next frame in the
   * list and use continue to skip freeing the IOB.
   */

  while (frame)
    {
      /* Skip over start byte and length */

      frame->io_offset += XBEE_APIFRAMEINDEX_TYPE;

      switch (frame->io_data[frame->io_offset++])
        {
          case XBEE_APIFRAME_MODEMSTATUS:
            {
              wlinfo("Modem Status: %d\n", frame->io_data[frame->io_offset++]);
            }
            break;
          case XBEE_APIFRAME_ATRESPONSE:
            {
              frame->io_offset++; /* Skip over frame index */

              command = (FAR char *)&frame->io_data[frame->io_offset];
              frame->io_offset += 2;

              wlinfo("AT Repsonse Recevied: %.*s\n", 2, command);

              /* Make sure the command status is OK=0 */

              if (frame->io_data[frame->io_offset])
                {
                  wlwarn("AT Command Error: %d\n",
                         frame->io_data[frame->io_offset]);
                }
              else
                {
                  frame->io_offset++;

                  if (memcmp(command, "ID", 2) == 0)
                    {
                      priv->addr.panid[1] = frame->io_data[frame->io_offset++];
                      priv->addr.panid[0] = frame->io_data[frame->io_offset++];

                      xbee_notify_respwaiter(priv, XBEE_RESP_AT_NETWORKID);
                    }
                  else if (memcmp(command, "SH", 2) == 0)
                    {
                      priv->addr.eaddr[7] = frame->io_data[frame->io_offset++];
                      priv->addr.eaddr[6] = frame->io_data[frame->io_offset++];
                      priv->addr.eaddr[5] = frame->io_data[frame->io_offset++];
                      priv->addr.eaddr[4] = frame->io_data[frame->io_offset++];

                      xbee_notify_respwaiter(priv, XBEE_RESP_AT_SERIALHIGH);
                    }
                  else if (memcmp(command, "SL", 2) == 0)
                    {
                      priv->addr.eaddr[3] = frame->io_data[frame->io_offset++];
                      priv->addr.eaddr[2] = frame->io_data[frame->io_offset++];
                      priv->addr.eaddr[1] = frame->io_data[frame->io_offset++];
                      priv->addr.eaddr[0] = frame->io_data[frame->io_offset++];

                      xbee_notify_respwaiter(priv, XBEE_RESP_AT_SERIALLOW);
                    }
                  else if (memcmp(command, "MY", 2) == 0)
                    {
                      priv->addr.saddr[1] = frame->io_data[frame->io_offset++];
                      priv->addr.saddr[0] = frame->io_data[frame->io_offset++];

                      xbee_notify_respwaiter(priv, XBEE_RESP_AT_SOURCEADDR);
                    }
                  else if (memcmp(command, "CH", 2) == 0)
                    {
                      priv->chan = frame->io_data[frame->io_offset++];
                      xbee_notify_respwaiter(priv, XBEE_RESP_AT_CHAN);
                    }
                  else if (memcmp(command, "VR", 2) == 0)
                    {
                      priv->firmwareversion = frame->io_data[frame->io_offset++] << 8;
                      priv->firmwareversion |= frame->io_data[frame->io_offset++];

                      xbee_notify_respwaiter(priv, XBEE_RESP_AT_FIRMWAREVERSION);
                    }
                  else if (memcmp(command, "AI", 2) == 0)
                    {
                      wlinfo("Association Indication: %d\n",
                             frame->io_data[frame->io_offset]);

                      /* 0xFF = No assocication status determined yet. */

                      if (frame->io_data[frame->io_offset] != 0xFF &&
                          frame->io_data[frame->io_offset] != 0x13)
                        {
                          wd_cancel(priv->assocwd);

                          xbee_lock(priv,  false);
                          xbee_notif_alloc(priv, &notif, false);
                          xbee_unlock(priv);

                          notif->notiftype = IEEE802154_NOTIFY_CONF_ASSOC;

                          if (frame->io_data[frame->io_offset] == 0)
                            {
                              notif->u.assocconf.status = IEEE802154_STATUS_SUCCESS;
                            }
                          else
                            {
                              notif->u.assocconf.status = IEEE802154_STATUS_FAILURE;
                            }

                          xbee_notify(priv, notif);
                        }
                    }
                  else if (memcmp(command, "A1", 2) == 0)
                    {
                      wlinfo("Endpoint Association: %d\n",
                             frame->io_data[frame->io_offset]);
                    }
                  else if (memcmp(command, "A2", 2) == 0)
                    {
                      wlinfo("Coordinator Association: %d\n",
                             frame->io_data[frame->io_offset]);
                    }
                  else if (memcmp(command, "CE", 2) == 0)
                    {
                      wlinfo("Coordinator Enable: %d\n",
                             frame->io_data[frame->io_offset]);
                    }
                  else if (memcmp(command, "SP", 2) == 0)
                    {
                      wlinfo("Sleep Period: %dsec\n",
                             frame->io_data[frame->io_offset]/100);
                    }
                  else
                    {
                      wlwarn("Unhandled AT Response: %.*s\n", 2, command);
                    }
                }
            }
            break;
          case XBEE_APIFRAME_TXSTATUS:
            {
              xbee_process_txstatus(priv, frame->io_data[frame->io_offset],
                                    frame->io_data[frame->io_offset + 1]);
            }
            break;
          case XBEE_APIFRAME_RX_EADDR:
            {
              nextframe = frame->io_flink;
              xbee_process_rxframe(priv, frame, IEEE802154_ADDRMODE_EXTENDED);
              frame = nextframe;

              /* xbee_process_rxframe takes care of freeing the IOB or passing
               * it along to the next highest layer */

              continue;
            }
            break;
          case XBEE_APIFRAME_RX_SADDR:
            {
              nextframe = frame->io_flink;
              xbee_process_rxframe(priv, frame, IEEE802154_ADDRMODE_SHORT);
              frame = nextframe;

              /* xbee_process_rxframe takes care of freeing the IOB or passing
               * it along to the next highest layer */

              continue;
            }
            break;
          default:
            {
              /* This really should never happen since xbee_validateframe should
               * have caught it.
               */

              wlwarn("Unknown frame type: %d\n", frame[XBEE_APIFRAMEINDEX_TYPE]);
            }
            break;
        }

      nextframe = frame->io_flink;
      iob_free(frame);
      frame = nextframe;
    }
}

/****************************************************************************
 * Name: xbee_process_rxframe
 *
 * Description:
 *   Process an incoming RX frame.
 *
 ****************************************************************************/

static void xbee_process_rxframe(FAR struct xbee_priv_s *priv,
                                 FAR struct iob_s *frame,
                                 enum ieee802154_addrmode_e addrmode)
{
  FAR struct ieee802154_data_ind_s *dataind;
  FAR struct xbee_maccb_s *cb;
  int ret;

  xbee_lock(priv, false);
  xbee_dataind_alloc(priv, &dataind, false);
  xbee_unlock(priv);

  dataind->frame = frame;

  /* The XBee does not give us information about how the device was addressed.
   * It only indicates the source mode. Therefore, we use the src address mode
   * as the destination address mode, unless the short address is set to
   * IEEE802154_SADDR_BCAST or IEEE802154_SADDR_UNSPEC
   */

  memcpy(&dataind->dest, &priv->addr, sizeof(struct ieee802154_addr_s));

  if (addrmode == IEEE802154_ADDRMODE_EXTENDED)
    {
      dataind->dest.mode = IEEE802154_ADDRMODE_EXTENDED;
      dataind->src.mode = IEEE802154_ADDRMODE_EXTENDED;
      dataind->src.eaddr[7] = frame->io_data[frame->io_offset++];
      dataind->src.eaddr[6] = frame->io_data[frame->io_offset++];
      dataind->src.eaddr[5] = frame->io_data[frame->io_offset++];
      dataind->src.eaddr[4] = frame->io_data[frame->io_offset++];
      dataind->src.eaddr[3] = frame->io_data[frame->io_offset++];
      dataind->src.eaddr[2] = frame->io_data[frame->io_offset++];
      dataind->src.eaddr[1] = frame->io_data[frame->io_offset++];
      dataind->src.eaddr[0] = frame->io_data[frame->io_offset++];
    }
  else
    {
      if (priv->addr.saddr == IEEE802154_SADDR_BCAST ||
          priv->addr.saddr == IEEE802154_SADDR_UNSPEC)
        {
          dataind->dest.mode = IEEE802154_ADDRMODE_EXTENDED;
        }
      else
        {
          dataind->dest.mode = IEEE802154_ADDRMODE_SHORT;
        }

      dataind->src.mode = IEEE802154_ADDRMODE_SHORT;
      dataind->src.saddr[1] = frame->io_data[frame->io_offset++];
      dataind->src.saddr[0] = frame->io_data[frame->io_offset++];
    }

  dataind->rssi = frame->io_data[frame->io_offset++];

  frame->io_offset++; /* Skip options byte */

  frame->io_len--; /* Remove the checksum */

  /* If there are registered MCPS callback receivers registered,
   * then forward the frame in priority order.  If there are no
   * registered receivers or if none of the receivers accept the
   * data frame then drop the frame.
   */

  for (cb = priv->cb; cb != NULL; cb = cb->flink)
    {
      /* Does this MAC client want frames? */

      if (cb->rxframe != NULL)
        {
          /* Yes.. Offer this frame to the receiver */

          ret = cb->rxframe(cb, dataind);
          if (ret >= 0)
            {
              /* The receiver accepted and disposed of the frame and
               * its metadata.  We are done.
               */

              return;
            }
        }
    }

  xbee_dataind_free((XBEEHANDLE)priv, dataind);
  iob_free(frame);
}

/****************************************************************************
 * Name: xbee_process_txstatus
 *
 * Description:
 *   Process an incoming TX status message. This searches the list of pending
 *   tx requests and notifies the
 *
 ****************************************************************************/

static void xbee_process_txstatus(FAR struct xbee_priv_s *priv, uint8_t frameid,
                                  uint8_t status)
{
  FAR struct ieee802154_notif_s *notif;

  xbee_lock(priv, false);
  xbee_notif_alloc(priv, &notif, false);
  xbee_unlock(priv);

  notif->notiftype = IEEE802154_NOTIFY_CONF_DATA;

  switch (status)
    {
      case 0x00:
        notif->u.dataconf.status = IEEE802154_STATUS_SUCCESS;
        break;
      case 0x01:
      case 0x21:
        notif->u.dataconf.status = IEEE802154_STATUS_NO_ACK;
        break;
      case 0x02:
        notif->u.dataconf.status = IEEE802154_STATUS_CHANNEL_ACCESS_FAILURE;
        break;
      default:
        notif->u.dataconf.status = IEEE802154_STATUS_FAILURE;
        break;
    }

  wlinfo("TX done. Frame ID: %d Status: 0x%02X\n", frameid, status);

  xbee_notify(priv, notif);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_init
 *
 * Description:
 *   Initialize an XBee driver.  The XBee device is assumed to be
 *   in the post-reset state upon entry to this function.
 *
 * Parameters:
 *   spi   - A reference to the platform's SPI driver for the XBee
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., XBee GPIO interrupts).
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

XBEEHANDLE xbee_init(FAR struct spi_dev_s *spi,
                     FAR const struct xbee_lower_s *lower)
{
  FAR struct xbee_priv_s *priv;

  /* Allocate object */

  priv = (FAR struct xbee_priv_s *) kmm_zalloc(sizeof(struct xbee_priv_s));
  if (priv == NULL)
    {
      wlinfo("Failed allocation xbee_priv_s structure\n");
      return NULL;
    }

  /* Attach irq */

  if (lower->attach(lower, xbee_interrupt, priv) != OK)
    {
      wlinfo("Failed to attach IRQ with XBee lower half\n");
      kmm_free(priv);
      return NULL;
    }

  /* Allow exclusive access to the struct */

  sem_init(&priv->exclsem, 0, 1);

  /* Initialize the data indication and notifcation allocation pools */

  xbee_notifpool_init(priv);
  xbee_dataindpool_init(priv);

  sq_init(&priv->waiter_queue);

  priv->assocwd = wd_create();

  priv->lower = lower;
  priv->spi   = spi;

  priv->frameid = 0; /* Frame ID should never be 0, but it is incremented
                      * in xbee_next_frameid before being used so it will be 1 */

  /* Reset the XBee */

  priv->lower->reset(priv->lower);

  /* Enable interrupts */

  priv->lower->enable(priv->lower, true);

  /* Trigger a dummy query without waiting to tell the XBee to operate in SPI
   * mode. By default the XBee uses the UART interface. It switches automatically
   * when a valid SPI frame is received.
   */

  xbee_at_query(priv, "VR");

  return (XBEEHANDLE)priv;
}

/****************************************************************************
 * Name: xbee_send_apiframe
 *
 * Description:
 *   Write an api frame over SPI
 *
 ****************************************************************************/

void xbee_send_apiframe(FAR struct xbee_priv_s *priv,
                        FAR const uint8_t *frame, uint16_t framelen)
{
  FAR struct iob_s *iob;
  FAR struct iob_s *previob;
  FAR struct iob_s *iobhead;
  uint16_t rxframelen = 0;
  int i;

  /* Get access to SPI bus, set relevant settings */

  SPI_LOCK(priv->spi, 1);
  SPI_SETBITS(priv->spi, 8);
  SPI_SETMODE(priv->spi, SPIDEV_MODE0);
  SPI_SETFREQUENCY(priv->spi, CONFIG_IEEE802154_XBEE_FREQUENCY);

  /* Assert CS */

  SPI_SELECT(priv->spi, SPIDEV_IEEE802154(0), true);

  /* Allocate an IOB for the incoming data. The XBee supports full-duplex
   * SPI communication. This means that the MISO data can become valid at any
   * time. This requires us to process incoming MISO data to see if it is valid.
   */

  iob             = iob_alloc(false);
  iob->io_flink   = NULL;
  iob->io_len     = 0;
  iob->io_offset  = 0;
  iob->io_pktlen  = 0;

  /* Keep a reference to the first IOB.  If we need to allocate more than
   * one to hold each API frame, then we will still have this reference to the
   * head of the list
   */

  iobhead = iob;

  i = 0;
  while (i < framelen || priv->lower->poll(priv->lower))
    {
      if (i < framelen)
        {
          iob->io_data[iob->io_len] = SPI_SEND(priv->spi, frame[i++]);
        }
      else
        {
          SPI_RECVBLOCK(priv->spi, &iob->io_data[iob->io_len], 1);
        }

      /* attn_latched should be set true immediately from the interrupt. Any
       * data prior to that can be completely ignored.
       */

      if (priv->attn_latched)
        {
          DEBUGASSERT(iob->io_len <= CONFIG_IOB_BUFSIZE);

          switch (iob->io_len)
            {
              case XBEE_APIFRAMEINDEX_STARTBYTE:
                {
                  if (iob->io_data[iob->io_len] == XBEE_STARTBYTE)
                    {
                      iob->io_len++;
                    }
                }
                break;
              case XBEE_APIFRAMEINDEX_LENGTHMSB:
                {
                  rxframelen = iob->io_data[iob->io_len++] << 8;
                }
                break;
              case XBEE_APIFRAMEINDEX_LENGTHLSB:
                {
                  rxframelen |= iob->io_data[iob->io_len++];
                  rxframelen += XBEE_APIFRAME_OVERHEAD;
                }
                break;
              case XBEE_APIFRAMEINDEX_TYPE:
                {
                  /* Check that the length and frame type make sense together */

                  if (!xbee_validate_apiframe(iob->io_data[iob->io_len],
                                              rxframelen - XBEE_APIFRAME_OVERHEAD))
                    {
                      wlwarn("invalid length on incoming API frame. Dropping!\n");
                      iob->io_len = 0;
                    }
                  else
                    {
                      iob->io_len++;
                    }
                }
                break;
              default:
                {
                  if (iob->io_len == rxframelen - 1)
                    {
                      iob->io_len++;
                      if (xbee_verify_checksum(iob))
                        {
                          /* This API frame is complete. Allocate a new IOB
                           * and link it to the existing one. When we are all
                           * finished we will pass this IOB list along for
                           * processing.
                           */

                          iob->io_flink = iob_alloc(false);
                          iob = iob->io_flink;

                          iob->io_flink  = NULL;
                          iob->io_len    = 0;
                          iob->io_offset = 0;
                          iob->io_pktlen = 0;
                        }
                      else
                        {
                          wlwarn("invalid checksum on incoming API frame. Dropping!\n");
                          iob->io_len = 0;
                        }
                    }
                  else
                    {
                      iob->io_len++;
                    }
                }
                break;
            }
        }
    }

  /* The last IOB in the list (or the only one) may be able to be freed since
   * it may not have any valid data. If it contains some data, but not a whole
   * API frame, something is wrong, so we just warn the user and drop the
   * data.  If the data was valid, the ATTN line should have stayed asserted
   * until all the data was clocked in. So if we don't have a full frame,
   * we can only drop it.
   */

  if (iob->io_len < XBEE_APIFRAME_OVERHEAD || iob->io_len != rxframelen)
    {
      if (iobhead == iob)
        {
          iobhead = NULL;
        }
      else
        {
          previob = iobhead;
          while (previob->io_flink != iob)
            {
              previob = previob->io_flink;
            }
          previob->io_flink = NULL;
        }

      if (iob->io_len > 0)
        {
          wlwarn("Partial API frame clocked in. Dropping!\n");
        }

      iob_free(iob);
    }

  if (iobhead != NULL)
    {
      if (priv->rx_apiframes == NULL)
        {
          priv->rx_apiframes = iobhead;
        }
      else
        {
          iob = priv->rx_apiframes;
          while (iob->io_flink != NULL)
            {
              iob = iob->io_flink;
            }

           iob->io_flink = iobhead;
        }
    }

  /* De-assert CS */

  SPI_SELECT(priv->spi, SPIDEV_IEEE802154(0), false);

  /* Relinquish control of the SPI Bus */

  SPI_LOCK(priv->spi,0);
}

/****************************************************************************
 * Name: xbee_at_query
 *
 * Description:
 *   Helper function to query a AT Command value.
 *
 ****************************************************************************/

void xbee_at_query(FAR struct xbee_priv_s *priv, FAR const char *atcommand)
{
  uint8_t frame[8];

  frame[0] = XBEE_STARTBYTE;
  frame[1] = 0;
  frame[2] = 4;
  frame[3] = XBEE_APIFRAME_ATCOMMMAND;
  frame[4] = 1;
  frame[5] = *atcommand;
  frame[6] = *(atcommand + 1);

  xbee_insert_checksum(frame, 8);

  xbee_send_apiframe(priv, frame, 8);
}

/****************************************************************************
 * Name: xbee_query_firmwareversion
 *
 * Description:
 *   Sends API frame with AT command request in order to get the firmware version
 *   from the device.
 *
 ****************************************************************************/

void xbee_query_firmwareversion(FAR struct xbee_priv_s *priv)
{
  struct xbee_respwaiter_s respwaiter;

  respwaiter.resp_id = XBEE_RESP_AT_FIRMWAREVERSION;
  sem_init(&respwaiter.sem, 0, 0);
  sem_setprotocol(&respwaiter.sem, SEM_PRIO_NONE);

  xbee_register_respwaiter(priv, &respwaiter);
  xbee_at_query(priv, "VR");

  sem_wait(&respwaiter.sem);

  xbee_unregister_respwaiter(priv, &respwaiter);

  sem_destroy(&respwaiter.sem);
}

/****************************************************************************
 * Name: xbee_query_panid
 *
 * Description:
 *   Sends API frame with AT command request in order to get the PAN ID
 *   (Network ID) from the device.
 *
 ****************************************************************************/

void xbee_query_panid(FAR struct xbee_priv_s *priv)
{
  struct xbee_respwaiter_s respwaiter;

  respwaiter.resp_id = XBEE_RESP_AT_NETWORKID;
  sem_init(&respwaiter.sem, 0, 0);
  sem_setprotocol(&respwaiter.sem, SEM_PRIO_NONE);

  xbee_register_respwaiter(priv, &respwaiter);
  xbee_at_query(priv, "ID");

  sem_wait(&respwaiter.sem);

  xbee_unregister_respwaiter(priv, &respwaiter);

  sem_destroy(&respwaiter.sem);
}

/****************************************************************************
 * Name: xbee_query_eaddr
 *
 * Description:
 *   Sends API frame with AT command request in order to get the IEEE 802.15.4
 *   Extended Address. (Serial Number) from the device.
 *
 ****************************************************************************/

void xbee_query_eaddr(FAR struct xbee_priv_s *priv)
{
  struct xbee_respwaiter_s respwaiter;

  respwaiter.resp_id = XBEE_RESP_AT_SERIALHIGH;
  sem_init(&respwaiter.sem, 0, 0);
  sem_setprotocol(&respwaiter.sem, SEM_PRIO_NONE);

  xbee_register_respwaiter(priv, &respwaiter);
  xbee_at_query(priv, "SH");

  sem_wait(&respwaiter.sem);

  respwaiter.resp_id = XBEE_RESP_AT_SERIALLOW;
  xbee_at_query(priv, "SL");

  sem_wait(&respwaiter.sem);

  xbee_unregister_respwaiter(priv, &respwaiter);
  sem_destroy(&respwaiter.sem);
}

/****************************************************************************
 * Name: xbee_query_saddr
 *
 * Description:
 *   Sends API frame with AT command request in order to get the
 *   Short Address. (Source Address (MY)) from the device.
 *
 ****************************************************************************/

void xbee_query_saddr(FAR struct xbee_priv_s *priv)
{
  struct xbee_respwaiter_s respwaiter;

  respwaiter.resp_id = XBEE_RESP_AT_SOURCEADDR;
  sem_init(&respwaiter.sem, 0, 0);
  sem_setprotocol(&respwaiter.sem, SEM_PRIO_NONE);

  xbee_register_respwaiter(priv, &respwaiter);
  xbee_at_query(priv, "MY");

  sem_wait(&respwaiter.sem);

  xbee_unregister_respwaiter(priv, &respwaiter);

  sem_destroy(&respwaiter.sem);
}

/****************************************************************************
 * Name: xbee_query_chan
 *
 * Description:
 *   Sends API frame with AT command request in order to get the RF Channel
 *   (Operating Channel) from the device.
 *
 ****************************************************************************/

void xbee_query_chan(FAR struct xbee_priv_s *priv)
{
  struct xbee_respwaiter_s respwaiter;

  respwaiter.resp_id = XBEE_RESP_AT_CHAN;
  sem_init(&respwaiter.sem, 0, 0);
  sem_setprotocol(&respwaiter.sem, SEM_PRIO_NONE);

  xbee_register_respwaiter(priv, &respwaiter);
  xbee_at_query(priv, "CH");

  sem_wait(&respwaiter.sem);

  xbee_unregister_respwaiter(priv, &respwaiter);

  sem_destroy(&respwaiter.sem);
}

/****************************************************************************
 * Name: xbee_set_panid
 *
 * Description:
 *   Sends API frame with AT command request in order to set the PAN ID
 *   (Network ID) of the device.
 *
 ****************************************************************************/

void xbee_set_panid(FAR struct xbee_priv_s *priv, FAR const uint8_t *panid)
{
  uint8_t frame[10];

  IEEE802154_PANIDCOPY(priv->addr.panid, panid);

  frame[0] = XBEE_STARTBYTE;
  frame[1] = 0;
  frame[2] = 6;
  frame[3] = XBEE_APIFRAME_ATCOMMMAND;
  frame[4] = 0;
  frame[5] = 'I';
  frame[6] = 'D';
  frame[7] = *(panid + 1);
  frame[8] = *(panid);

  xbee_insert_checksum(frame, 10);

  xbee_send_apiframe(priv, frame, 10);
}

/****************************************************************************
 * Name: xbee_set_saddr
 *
 * Description:
 *   Sends API frame with AT command request in order to set the Short Address
 *   (Source Address (MY)) of the device
 *
 ****************************************************************************/

void xbee_set_saddr(FAR struct xbee_priv_s *priv, FAR const uint8_t *saddr)
{
  uint8_t frame[10];

  IEEE802154_SADDRCOPY(priv->addr.saddr, saddr);

  frame[0]  = XBEE_STARTBYTE;
  frame[1]  = 0;
  frame[2]  = 6;
  frame[3]  = XBEE_APIFRAME_ATCOMMMAND;
  frame[4]  = 0;
  frame[5]  = 'M';
  frame[6]  = 'Y';
  frame[7]  = *(saddr + 1);
  frame[8] = *(saddr);

  xbee_insert_checksum(frame, 10);

  xbee_send_apiframe(priv, frame, 10);
}

/****************************************************************************
 * Name: xbee_set_chan
 *
 * Description:
 *   Sends API frame with AT command request in order to set the RF channel
 *   (Operatin Channel) of the device.
 *
 ****************************************************************************/

void xbee_set_chan(FAR struct xbee_priv_s *priv, uint8_t chan)
{
  uint8_t frame[9];

  priv->chan = chan;

  frame[0] = XBEE_STARTBYTE;
  frame[1] = 0;
  frame[2] = 5;
  frame[3] = XBEE_APIFRAME_ATCOMMMAND;
  frame[4] = 0;
  frame[5] = 'C';
  frame[6] = 'H';
  frame[7]  = chan;

  xbee_insert_checksum(frame, 9);

  xbee_send_apiframe(priv, frame, 9);
}

/****************************************************************************
 * Name: xbee_set_epassocflags
 *
 * Description:
 *   Set flags in 'A1' command register to determine how endpoint behaves
 *   with regards to association.
 *
 ****************************************************************************/

void xbee_set_epassocflags(FAR struct xbee_priv_s *priv, uint8_t flags)
{
  uint8_t frame[9];

  frame[0] = XBEE_STARTBYTE;
  frame[1] = 0;
  frame[2] = 5;
  frame[3] = XBEE_APIFRAME_ATCOMMMAND;
  frame[4] = 0;
  frame[5] = 'A';
  frame[6] = '1';
  frame[7] = flags;

  xbee_insert_checksum(frame, 9);

  xbee_send_apiframe(priv, frame, 9);
}

/****************************************************************************
 * Name: xbee_set_coordassocflags
 *
 * Description:
 *   Set flags in 'A2' command register to determine how coordinator behaves
 *   with regards to association.
 *
 ****************************************************************************/

void xbee_set_coordassocflags(FAR struct xbee_priv_s *priv, uint8_t flags)
{
  uint8_t frame[9];

  frame[0] = XBEE_STARTBYTE;
  frame[1] = 0;
  frame[2] = 5;
  frame[3] = XBEE_APIFRAME_ATCOMMMAND;
  frame[4] = 0;
  frame[5] = 'A';
  frame[6] = '2';
  frame[7] = flags;

  xbee_insert_checksum(frame, 9);

  xbee_send_apiframe(priv, frame, 9);
}

/****************************************************************************
 * Name: xbee_set_sleepperiod
 *
 * Description:
 *   Set Cyclic Sleep Period using 'SP' AT command.
 *
 ****************************************************************************/

void xbee_set_sleepperiod(FAR struct xbee_priv_s *priv, uint16_t period)
{
  uint8_t frame[10];

  frame[0] = XBEE_STARTBYTE;
  frame[1] = 0;
  frame[2] = 6;
  frame[3] = XBEE_APIFRAME_ATCOMMMAND;
  frame[4] = 0;
  frame[5] = 'S';
  frame[6] = 'P';
  frame[7] = period >> 8;
  frame[8] = period;

  xbee_insert_checksum(frame, 10);

  xbee_send_apiframe(priv, frame, 10);
}

/****************************************************************************
 * Name: xbee_enable_coord
 *
 * Description:
 *   Enables/Disables coordinator mode using 'CE' command
 *
 ****************************************************************************/

void xbee_enable_coord(FAR struct xbee_priv_s *priv, bool enable)
{
  uint8_t frame[9];

  frame[0] = XBEE_STARTBYTE;
  frame[1] = 0;
  frame[2] = 5;
  frame[3] = XBEE_APIFRAME_ATCOMMMAND;
  frame[4] = 0;
  frame[5] = 'C';
  frame[6] = 'E';
  frame[7] = enable;

  xbee_insert_checksum(frame, 9);

  xbee_send_apiframe(priv, frame, 9);
}

/****************************************************************************
 * Name: xbee_regdump
 *
 * Description:
 *   Perform a series of queries updating struct and printing settings to SYSLOG.
 *
 ****************************************************************************/

void xbee_regdump(FAR struct xbee_priv_s *priv)
{
  xbee_query_firmwareversion(priv);

  wlinfo("XBee Firmware Version: %04x\n", priv->firmwareversion);

  xbee_at_query(priv, "CE");
  xbee_at_query(priv, "A1");
  xbee_at_query(priv, "A2");
  xbee_at_query(priv, "SP");
}

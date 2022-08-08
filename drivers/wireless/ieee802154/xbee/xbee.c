/****************************************************************************
 * drivers/wireless/ieee802154/xbee/xbee.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>

#include "xbee.h"
#include "xbee_mac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XBEE_ATQUERY_TIMEOUT MSEC2TICK(100)

#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
#define XBEE_LOCKUP_QUERYTIME MSEC2TICK(500)
#define XBEE_LOCKUP_QUERYATTEMPTS 20
#endif

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
static void xbee_process_txstatus(FAR struct xbee_priv_s *priv,
              uint8_t frameid, uint8_t status);
static void xbee_process_rxframe(FAR struct xbee_priv_s *priv,
              FAR struct iob_s *frame,
              enum ieee802154_addrmode_e addrmode);
static void xbee_notify(FAR struct xbee_priv_s *priv,
                FAR struct ieee802154_primitive_s *primitive);
static void xbee_notify_worker(FAR void *arg);
static void xbee_atquery_timeout(wdparm_t arg);

#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
static void xbee_lockupcheck_timeout(wdparm_t arg);
static void xbee_lockupcheck_worker(FAR void *arg);
static void xbee_backup_worker(FAR void *arg);
static void xbee_lockupcheck_reschedule(FAR struct xbee_priv_s *priv);
#endif

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
 * Input Parameters:
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
      return work_queue(HPWORK, &priv->attnwork, xbee_attnworker,
                        (FAR void *)priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: xbee_attnworker
 *
 * Description:
 *   Perform interrupt handling (Attention) logic outside of the interrupt
 *   handler (on the work queue thread).
 *
 * Input Parameters:
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

  /* NOTE: There is a helpful side-effect to trying to get the SPI Lock here
   * even when there is a write going on. That is, if the SPI write are on a
   * thread with lower priority, trying to get the lock here should boost
   * the priority of that thread, helping move along the low-level driver
   * work that really should be happening in a high priority way anyway.
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

  if (priv->attn_latched)
    {
      while (priv->lower->poll(priv->lower) && iob != NULL)
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
                  /* Check that the length and frame type make sense
                   * together.
                   */

                  if (!xbee_validate_apiframe(iob->io_data[iob->io_len],
                                              rxframelen -
                                              XBEE_APIFRAME_OVERHEAD))
                    {
                      wlwarn("invalid length on incoming API frame. "
                             "Dropping!\n");
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

                          iob->io_flink = iob_tryalloc(false);

                          iob = iob->io_flink;
                          if (iob != NULL)
                            {
                              iob->io_flink  = NULL;
                              iob->io_len    = 0;
                              iob->io_offset = 0;
                              iob->io_pktlen = 0;
                            }
                       }
                      else
                        {
                          wlwarn("invalid checksum on incoming API frame. "
                                 "Dropping!\n");
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

      if (!priv->lower->poll(priv->lower))
        {
          priv->attn_latched = false;
        }
    }

  /* The last IOB in the list (or the only one) may be able to be freed
   * since it may not have any valid data. If it contains some data, but not
   * a whole API frame, something is wrong, so we just warn the user and
   * drop the data.  If the data was valid, the ATTN line should have stayed
   * asserted until all the data was clocked in. So if we don't have a full
   * frame, we can only drop it.
   */

  if (iob != NULL)
    {
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
   * be clocked in from an SPI write. By detaching the IOB list, we can
   * process the incoming data without holding up the SPI bus
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

  /* If an interrupt occurred while the worker was running, it was not
   * scheduled since there is a good chance this function has already handled
   * it as part of the previous ATTN assertion. Therefore, if the ATTN
   * line is asserted again reschedule the work.
   */

  if (priv->attn_latched)
    {
      work_queue(HPWORK, &priv->attnwork, xbee_attnworker,
                 (FAR void *)priv, 0);
    }
}

/****************************************************************************
 * Name: xbee_validate_apiframe
 *
 * Description:
 *   Verifies that the API frame type is known and that the length makes
 *   sense for that frame type.
 *
 * Input Parameters:
 *    frame - pointer to the frame data
 *    datalen - The size of the data section of the frame.  This is the value
 *              included as the second and third byte of the frame.
 *
 * Returned Value:
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
 * Input Parameters:
 *    frame - pointer to the frame data
 *    framelen - size of the overall frame. NOT the data length field
 *
 * Returned Value:
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

  if (checksum != 0xff)
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
  FAR struct ieee802154_primitive_s *primitive;
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
#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
      /* Any time we receive an API frame from the XBee we reschedule our
       * lockup timeout. Receiving an API frame is an indication that the
       * XBee is not locked up.
       */

      xbee_lockupcheck_reschedule(priv);
#endif

      /* Skip over start byte and length */

      frame->io_offset += XBEE_APIFRAMEINDEX_TYPE;

      switch (frame->io_data[frame->io_offset++])
        {
          case XBEE_APIFRAME_MODEMSTATUS:
            {
#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
              /* If the Modem Status indicates that the coordinator has just
               * formed a new network, we know that the channel and PAN ID
               * are locked in.  In case we need to reset the radio due to a
               * lockup, we tell the XBee to write the parameters to non-
               * volatile memory so that upon reset, we can resume operation.
               */

              if (frame->io_data[frame->io_offset] == 0x06)
                {
                  if (work_available(&priv->backupwork))
                    {
                      work_queue(LPWORK, &priv->backupwork,
                                 xbee_backup_worker, (FAR void *)priv, 0);
                    }
                }
#endif

              wlinfo("Modem Status: %d\n",
                     frame->io_data[frame->io_offset++]);
            }
            break;

          case XBEE_APIFRAME_ATRESPONSE:
            {
              frame->io_offset++; /* Skip over frame index */

              command = (FAR char *)&frame->io_data[frame->io_offset];
              frame->io_offset += 2;

              wlinfo("AT Response Received: %.*s\n", 2, command);

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
                      priv->addr.panid[1] =
                        frame->io_data[frame->io_offset++];
                      priv->addr.panid[0] =
                        frame->io_data[frame->io_offset++];
                    }
                  else if (memcmp(command, "SH", 2) == 0)
                    {
                      priv->addr.eaddr[7] =
                        frame->io_data[frame->io_offset++];
                      priv->addr.eaddr[6] =
                        frame->io_data[frame->io_offset++];
                      priv->addr.eaddr[5] =
                        frame->io_data[frame->io_offset++];
                      priv->addr.eaddr[4] =
                        frame->io_data[frame->io_offset++];
                    }
                  else if (memcmp(command, "SL", 2) == 0)
                    {
                      priv->addr.eaddr[3] =
                        frame->io_data[frame->io_offset++];
                      priv->addr.eaddr[2] =
                        frame->io_data[frame->io_offset++];
                      priv->addr.eaddr[1] =
                        frame->io_data[frame->io_offset++];
                      priv->addr.eaddr[0] =
                        frame->io_data[frame->io_offset++];
                    }
                  else if (memcmp(command, "MY", 2) == 0)
                    {
                      priv->addr.saddr[1] =
                        frame->io_data[frame->io_offset++];
                      priv->addr.saddr[0] =
                        frame->io_data[frame->io_offset++];
                    }
                  else if (memcmp(command, "CH", 2) == 0)
                    {
                      priv->chan = frame->io_data[frame->io_offset++];
                    }
                  else if (memcmp(command, "VR", 2) == 0)
                    {
                      priv->firmwareversion  =
                        frame->io_data[frame->io_offset++] << 8;
                      priv->firmwareversion |=
                        frame->io_data[frame->io_offset++];
                    }
                  else if (memcmp(command, "AI", 2) == 0)
                    {
                      wlinfo("Association Indication: %d\n",
                             frame->io_data[frame->io_offset]);

                      /* 0xff = No association status determined yet. */

                      if (frame->io_data[frame->io_offset] != 0xff &&
                          frame->io_data[frame->io_offset] != 0x13)
                        {
                          wd_cancel(&priv->assocwd);
                          priv->associating = false;

                          primitive = ieee802154_primitive_allocate();
                          primitive->type = IEEE802154_PRIMITIVE_CONF_ASSOC;

                          if (frame->io_data[frame->io_offset] == 0)
                            {
                              primitive->u.assocconf.status =
                                IEEE802154_STATUS_SUCCESS;
#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
                              /* Upon successful association, we know that
                               * the channel and PAN ID give us a valid
                               * connection.  In case we need to reset the
                               * radio due to a lockup, we tell the XBee to
                               * write the parameters to non-volatile memory
                               * so that upon reset, we can resume operation.
                               */

                              if (work_available(&priv->backupwork))
                                {
                                  work_queue(LPWORK, &priv->backupwork,
                                             xbee_backup_worker,
                                             (FAR void *)priv, 0);
                                }
#endif
                            }
                          else
                            {
                              primitive->u.assocconf.status =
                                IEEE802154_STATUS_FAILURE;
                            }

                          xbee_notify(priv, primitive);
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
                             frame->io_data[frame->io_offset] / 100);
                    }
                  else if (memcmp(command, "RR", 2) == 0)
                    {
                      wlinfo("XBee Retries: %d\n",
                             frame->io_data[frame->io_offset]);
                    }
                  else if (memcmp(command, "MM", 2) == 0)
                    {
                      wlinfo("Mac Mode: %d\n",
                             frame->io_data[frame->io_offset]);
                    }
                  else if (memcmp(command, "PL", 2) == 0)
                    {
                      wlinfo("Power Level: %d\n",
                             frame->io_data[frame->io_offset]);
                      priv->pwrlvl = frame->io_data[frame->io_offset++];
                    }
                  else if (memcmp(command, "PM", 2) == 0)
                    {
                      wlinfo("Boost Mode (Power Mode): %d\n",
                             frame->io_data[frame->io_offset]);
                      priv->boostmode = frame->io_data[frame->io_offset++];
                    }
                  else if (memcmp(command, "WR", 2) == 0)
                    {
                      wlinfo("Write Complete: %d\n",
                             frame->io_data[frame->io_offset]);
                    }
                  else
                    {
                      wlwarn("Unhandled AT Response: %.*s\n", 2, command);
                    }

                  /* If this is the command we are querying for */

                  if ((priv->querycmd[0] == *command) &&
                      (priv->querycmd[1] == *(command + 1)))
                    {
                      wd_cancel(&priv->atquery_wd);
                      priv->querydone = true;
                      nxsem_post(&priv->atresp_sem);
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
              frame->io_flink = NULL;
              xbee_process_rxframe(priv, frame,
                                   IEEE802154_ADDRMODE_EXTENDED);
              frame = nextframe;

              /* xbee_process_rxframe takes care of freeing the IOB or
               * passing it along to the next highest layer.
               */

              continue;
            }
            break;

          case XBEE_APIFRAME_RX_SADDR:
            {
              nextframe = frame->io_flink;
              frame->io_flink = NULL;
              xbee_process_rxframe(priv, frame, IEEE802154_ADDRMODE_SHORT);
              frame = nextframe;

              /* xbee_process_rxframe takes care of freeing the IOB or
               * passing it along to the next highest layer.
               */

              continue;
            }
            break;

          default:
            {
              /* This really should never happen since xbee_validateframe
               * should have caught it.
               */

              wlwarn("Unknown frame type: %p\n",
                     &frame[XBEE_APIFRAMEINDEX_TYPE]);
            }
            break;
        }

      /* IOB free logic assumes that a valid io_flink entry in the IOB that
       * is being freed indicates that the IOB is a part of an IOB chain.
       * Since that is not the case here and we are just using the io_flink
       * field as a way of managing a list of independent frames, we set the
       * io_flink to NULL prior to freeing it.
       */

      nextframe = frame->io_flink;
      frame->io_flink = NULL;
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
  FAR struct ieee802154_primitive_s *primitive;
  FAR struct ieee802154_data_ind_s *dataind;

  DEBUGASSERT(frame != NULL);

  primitive = ieee802154_primitive_allocate();
  dataind = &primitive->u.dataind;

  primitive->type = IEEE802154_PRIMITIVE_IND_DATA;

  dataind->frame = frame;

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
      dataind->src.mode = IEEE802154_ADDRMODE_SHORT;
      dataind->src.saddr[1] = frame->io_data[frame->io_offset++];
      dataind->src.saddr[0] = frame->io_data[frame->io_offset++];
    }

  /* The XBee does not give us information about how the device was
   * addressed.  It only indicates the source mode. Therefore, we make the
   * assumption that if the saddr is set, we we're addressed using the
   * saddr, otherwise we must have been addressed using the eaddr.
   */

  memcpy(&dataind->dest, &priv->addr, sizeof(struct ieee802154_addr_s));

  if (IEEE802154_SADDRCMP(priv->addr.saddr, &IEEE802154_SADDR_BCAST) ||
      IEEE802154_SADDRCMP(priv->addr.saddr, &IEEE802154_SADDR_UNSPEC))
    {
      dataind->dest.mode = IEEE802154_ADDRMODE_EXTENDED;
    }
  else
    {
      dataind->dest.mode = IEEE802154_ADDRMODE_SHORT;
    }

  dataind->rssi = frame->io_data[frame->io_offset++];

  frame->io_offset++; /* Skip options byte */

  frame->io_len--; /* Remove the checksum */

  xbee_notify(priv, primitive);

  wlinfo("Received frame. RSSI: -%d dbm\n", dataind->rssi);
}

/****************************************************************************
 * Name: xbee_process_txstatus
 *
 * Description:
 *   Process an incoming TX status message. This searches the list of pending
 *   tx requests and notifies the
 *
 ****************************************************************************/

static void xbee_process_txstatus(FAR struct xbee_priv_s *priv,
                                  uint8_t frameid, uint8_t status)
{
  FAR struct ieee802154_primitive_s *primitive;

  primitive = ieee802154_primitive_allocate();

  primitive->type = IEEE802154_PRIMITIVE_CONF_DATA;

  switch (status)
    {
      case 0x00:
        primitive->u.dataconf.status =
          IEEE802154_STATUS_SUCCESS;
        break;

      case 0x01:
      case 0x21:
        primitive->u.dataconf.status =
          IEEE802154_STATUS_NO_ACK;
        break;

      case 0x02:
        primitive->u.dataconf.status =
          IEEE802154_STATUS_CHANNEL_ACCESS_FAILURE;
        break;

      default:
        primitive->u.dataconf.status =
          IEEE802154_STATUS_FAILURE;
        break;
    }

  if (status != IEEE802154_STATUS_SUCCESS)
    {
      wlwarn("TX done. Frame ID: %d Status: 0x%02x\n", frameid, status);
    }
  else
    {
      wlinfo("TX done. Frame ID: %d Status: 0x%02x\n", frameid, status);
    }

  xbee_notify(priv, primitive);

  /* If this is the frame we are currently waiting on, cancel the timeout,
   * and notify the waiting thread that the transmit is done
   */

  if (priv->frameid == frameid)
    {
      wd_cancel(&priv->reqdata_wd);
      priv->txdone = true;
      nxsem_post(&priv->txdone_sem);
    }
}

/****************************************************************************
 * Name: xbee_notify
 *
 * Description:
 *   Queue the primitive in the queue and queue work on the LPWORK
 *   queue if is not already scheduled.
 *
 * Assumptions:
 *    Called with the MAC locked
 *
 ****************************************************************************/

static void xbee_notify(FAR struct xbee_priv_s *priv,
                        FAR struct ieee802154_primitive_s *primitive)
{
  while (nxsem_wait(&priv->primitive_sem) < 0);

  sq_addlast((FAR sq_entry_t *)primitive, &priv->primitive_queue);
  nxsem_post(&priv->primitive_sem);

  if (work_available(&priv->notifwork))
    {
      work_queue(LPWORK, &priv->notifwork, xbee_notify_worker,
                 (FAR void *)priv, 0);
    }
}

/****************************************************************************
 * Name: xbee_notify_worker
 *
 * Description:
 *    Pop each primitive off the queue and call the registered
 *    callbacks.  There is special logic for handling ieee802154_data_ind_s.
 *
 ****************************************************************************/

static void xbee_notify_worker(FAR void *arg)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)arg;
  FAR struct xbee_maccb_s *cb;
  FAR struct ieee802154_primitive_s *primitive;
  int ret;

  DEBUGASSERT(priv != NULL);

  while (nxsem_wait(&priv->primitive_sem) < 0);
  primitive =
    (FAR struct ieee802154_primitive_s *)sq_remfirst(&priv->primitive_queue);
  nxsem_post(&priv->primitive_sem);

  while (primitive != NULL)
    {
      /* Data indications are a special case since the frame can only be
       * passed to one place. The return value of the notify call is used to
       * accept or reject the primitive. In the case of the data indication,
       * there can only be one accept. Callbacks are stored in order of
       * there receiver priority ordered when the callbacks are bound in
       * mac802154_bind().
       */

      if (primitive->type == IEEE802154_PRIMITIVE_IND_DATA)
        {
          bool dispose = true;

          primitive->nclients = 1;

          for (cb = priv->cb; cb != NULL; cb = cb->flink)
            {
              if (cb->notify != NULL)
                {
                  ret = cb->notify(cb, primitive);
                  if (ret >= 0)
                    {
                      /* The receiver accepted and disposed of the frame and
                       * it's meta-data.  We are done.
                       */

                      dispose = false;
                      break;
                    }
                }
            }

          if (dispose)
            {
              iob_free(primitive->u.dataind.frame);
              ieee802154_primitive_free(primitive);
            }
        }
      else
        {
          /* Set the number of clients count so that the primitive resources
           * will be preserved until all clients are finished with it.
           */

          primitive->nclients = priv->nclients;

          /* Try to notify every registered MAC client */

          for (cb = priv->cb; cb != NULL; cb = cb->flink)
            {
              if (cb->notify != NULL)
                {
                  ret = cb->notify(cb, primitive);
                  if (ret < 0)
                    {
                      ieee802154_primitive_free(primitive);
                    }
                }
              else
                {
                  ieee802154_primitive_free(primitive);
                }
            }
        }

      /* Get the next primitive then loop */

      while (nxsem_wait(&priv->primitive_sem) < 0);

      primitive = (FAR struct ieee802154_primitive_s *)
                  sq_remfirst(&priv->primitive_queue);
      nxsem_post(&priv->primitive_sem);
    }
}

/****************************************************************************
 * Name: xbee_atquery_timeout
 *
 * Description:
 *   This function runs when an AT Query has timed out waiting for a response
 *   from the XBee module. This really should never happen, but if it does,
 *   handle it gracefully by retrying the query.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void xbee_atquery_timeout(wdparm_t arg)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  wlwarn("AT Query timeout\n");

  /* Wake the pending query thread so it can retry */

  nxsem_post(&priv->atresp_sem);
}

#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
/****************************************************************************
 * Name: xbee_lockupcheck_timeout
 *
 * Description:
 *   This function runs when a query to the XBee has not been issued for
 *   awhile.  We query periodically in an effort to detect if the XBee has
 *   locked up.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void xbee_lockupcheck_timeout(wdparm_t arg)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  if (work_available(&priv->lockupwork))
    {
      work_queue(LPWORK, &priv->lockupwork, xbee_lockupcheck_worker,
                 (FAR void *)priv, 0);
    }
}

/****************************************************************************
 * Name: xbee_lockupcheck_worker
 *
 * Description:
 *    Perform an AT query to make sure the XBee is still responsive. If we've
 *    gotten here, it means we haven't talked to the XBee in a while. This
 *    is to workaround an issue where the XBee locks up. In this condition,
 *    most of the time, querying it kicks it out of the state. In some
 *    conditions though, the XBee locks up to the point where it needs to be
 *    reset.  This will be handled inside xbee_atquery; if we don't get a
 *    response we will reset the XBee.
 *
 ****************************************************************************/

static void xbee_lockupcheck_worker(FAR void *arg)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)arg;
  DEBUGASSERT(priv != NULL);
  wlinfo("Issuing query to detect lockup\n");
  xbee_query_chan(priv);
}

/****************************************************************************
 * Name: xbee_backup_worker
 *
 * Description:
 *    In the case that we need to reset the XBee to bring it out of a locked
 *    up state, we need to be able to restore it's previous state seamlessly
 *    to resume operation. In order to achieve this, we backup the
 *    parameters using the "WR" AT command. We do this at strategic points
 *    as we don't know what type of memory technology is being used and
 *    writing too often may reduce the lifetime of the device. The two key
 *    points chosen are upon association for endpoint nodes, and upon
 *    creating a new network for coordinator nodes.  These conditions
 *    indicate that the node is actively communicating on the network and
 *    that the channel and pan ID are now set for the network.
 *
 ****************************************************************************/

static void xbee_backup_worker(FAR void *arg)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)arg;
  DEBUGASSERT(priv != NULL);
  xbee_save_params(priv);
}

static void xbee_lockupcheck_reschedule(FAR struct xbee_priv_s *priv)
{
  wd_cancel(&priv->lockup_wd);

  /* Kickoff the watchdog timer that will query the XBee periodically (if
   * naturally occurring queries do not occur). We query periodically to
   * make sure the XBee is still responsive. If during any query, the XBee
   * does not respond after multiple attempts, we restart the XBee to get
   * it back in a working state
   */

  wd_start(&priv->lockup_wd, XBEE_LOCKUP_QUERYTIME,
           xbee_lockupcheck_timeout, (wdparm_t)priv);
}

#endif

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
 * Input Parameters:
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

  priv->lower = lower;
  priv->spi   = spi;

  nxsem_init(&priv->primitive_sem, 0, 1);
  nxsem_init(&priv->atquery_sem, 0, 1);
  nxsem_init(&priv->tx_sem, 0, 1);
  nxsem_init(&priv->txdone_sem, 0, 0);
  nxsem_set_protocol(&priv->txdone_sem, SEM_PRIO_NONE);

  ieee802154_primitivepool_initialize();

  sq_init(&priv->primitive_queue);
  priv->frameid = 0; /* Frame ID should never be 0, but it is incremented
                      * in xbee_next_frameid before being used so it will be 1 */
  priv->querycmd[0] = 0;
  priv->querycmd[1] = 0;

  /* Initialize the saddr */

  IEEE802154_SADDRCOPY(priv->addr.saddr, &IEEE802154_SADDR_UNSPEC);

  /* Reset the XBee radio */

  priv->lower->reset(priv->lower);

  /* Enable interrupts */

  priv->lower->enable(priv->lower, true);

  /* Trigger a query to tell the XBee to operate in SPI mode. By default the
   * XBee uses the UART interface. It switches automatically when a valid
   * SPI frame is received.
   *
   * Use this opportunity to pull the extended address
   */

  xbee_query_eaddr(priv);

#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
  xbee_lockupcheck_reschedule(priv);
#endif

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
   * time. This requires us to process incoming MISO data to see if it is
   * valid.
   *
   * If we can't allocate an IOB, then we have to just drop the incoming
   * data.
   */

  iob             = iob_tryalloc(false);
  iob->io_flink   = NULL;
  iob->io_len     = 0;
  iob->io_offset  = 0;
  iob->io_pktlen  = 0;

  /* Keep a reference to the first IOB.  If we need to allocate more than
   * one to hold each API frame, then we will still have this reference to
   * the head of the list
   */

  iobhead = iob;

  i = 0;
  while (i < framelen || (priv->lower->poll(priv->lower) && iob != NULL))
    {
      if (i < framelen)
        {
          if (iob)
            {
              iob->io_data[iob->io_len] = SPI_SEND(priv->spi, frame[i++]);
            }
          else
            {
              SPI_SEND(priv->spi, frame[i++]);
            }
        }
      else
        {
          SPI_RECVBLOCK(priv->spi, &iob->io_data[iob->io_len], 1);
        }

      /* attn_latched should be set true immediately from the interrupt. Any
       * data prior to that can be completely ignored.
       */

      if (priv->attn_latched && iob != NULL)
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
                  /* Check that the length and frame type make sense
                   * together.
                   */

                  if (!xbee_validate_apiframe(iob->io_data[iob->io_len],
                                              rxframelen -
                                              XBEE_APIFRAME_OVERHEAD))
                    {
                      wlwarn("invalid length on incoming API frame. "
                             "Dropping!\n");
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

                          iob->io_flink = iob_tryalloc(false);
                          iob = iob->io_flink;

                          if (iob != NULL)
                            {
                              iob->io_flink  = NULL;
                              iob->io_len    = 0;
                              iob->io_offset = 0;
                              iob->io_pktlen = 0;
                            }
                       }
                      else
                        {
                          wlwarn("invalid checksum on incoming API frame. "
                                 "Dropping!\n");
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

  /* The last IOB in the list (or the only one) may be able to be freed
   * since it may not have any valid data. If it contains some data, but not
   * a whole API frame, something is wrong, so we just warn the user and
   * drop the data.  If the data was valid, the ATTN line should have stayed
   * asserted until all the data was clocked in. So if we don't have a full
   * frame, we can only drop it.
   */

  if (iob != NULL)
    {
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

  SPI_LOCK(priv->spi, 0);
}

/****************************************************************************
 * Name: xbee_atquery
 *
 * Description:
 *   Sends AT Query and waits for response from device
 *
 ****************************************************************************/

int xbee_atquery(FAR struct xbee_priv_s *priv, FAR const char *atcommand)
{
  int ret;
#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
  int retries = XBEE_LOCKUP_QUERYATTEMPTS;
#endif

  /* Only allow one query at a time */

  ret = nxsem_wait(&priv->atquery_sem);
  if (ret < 0)
    {
      return ret;
    }

  priv->querydone = false;

  /* We reinitialize this every time, in case something gets out of phase
   * with the timeout and the received response.
   */

  nxsem_init(&priv->atresp_sem, 0, 0);
  nxsem_set_protocol(&priv->atresp_sem, SEM_PRIO_NONE);

  do
    {
      /* There is a note in the XBee datasheet: Once you issue a WR command,
       * do not send any additional characters to the device until after
       * you receive the OK response.
       *
       * If we are issuing a WR command, don't set a timeout. We will have
       * to rely on the XBee getting back to us reliably.
       */

      if (memcmp(atcommand, "WR", 2) != 0)
        {
          /* Setup a timeout */

          wd_start(&priv->atquery_wd, XBEE_ATQUERY_TIMEOUT,
                   xbee_atquery_timeout, (wdparm_t)priv);
        }

      /* Send the query */

      priv->querycmd[0] = *atcommand;
      priv->querycmd[1] = *(atcommand + 1);
      xbee_send_atquery(priv, atcommand);

      /* Wait for the response to be received */

      ret = nxsem_wait(&priv->atresp_sem);
      if (ret < 0)
        {
          wd_cancel(&priv->atquery_wd);
          priv->querycmd[0] = 0;
          priv->querycmd[1] = 0;
          nxsem_post(&priv->atquery_sem);
          return ret;
        }

#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
      if (--retries == 0 && !priv->querydone)
        {
          wlerr("XBee not responding. Resetting.\n");
          priv->lower->reset(priv->lower);
          retries = XBEE_LOCKUP_QUERYATTEMPTS;
        }
#endif
    }
  while (!priv->querydone);

  nxsem_post(&priv->atquery_sem);

  return OK;
}

/****************************************************************************
 * Name: xbee_send_atquery
 *
 * Description:
 *   Helper function to send the AT query to the XBee
 *
 ****************************************************************************/

void xbee_send_atquery(FAR struct xbee_priv_s *priv,
                       FAR const char *atcommand)
{
  uint8_t frame[8];

  wlinfo("AT Query: %c%c\n", *atcommand, *(atcommand + 1));

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
 *   Sends API frame with AT command request in order to set the Short
 *   Address (Source Address (MY)) of the device
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
 * Name: xbee_set_powerlevel
 *
 * Description:
 *   Sends API frame with AT command request in order to set the RF power
 *   level of the device.
 *
 ****************************************************************************/

void xbee_set_powerlevel(FAR struct xbee_priv_s *priv, uint8_t level)
{
  uint8_t frame[9];

  priv->pwrlvl = level;

  frame[0] = XBEE_STARTBYTE;
  frame[1] = 0;
  frame[2] = 5;
  frame[3] = XBEE_APIFRAME_ATCOMMMAND;
  frame[4] = 0;
  frame[5] = 'P';
  frame[6] = 'L';
  frame[7]  = level;

  xbee_insert_checksum(frame, 9);

  xbee_send_apiframe(priv, frame, 9);
}

/****************************************************************************
 * Name: xbee_set_boostmode
 *
 * Description:
 *   Sends API frame with AT command request in order to set the Boost mode
 *   setting of the device. NOTE: XBee Pro always enables boost mode and
 *   does not support this command
 *
 ****************************************************************************/

void xbee_set_boostmode(FAR struct xbee_priv_s *priv, uint8_t mode)
{
  uint8_t frame[9];

  if (mode > 1) return;

  priv->boostmode = mode;

  frame[0] = XBEE_STARTBYTE;
  frame[1] = 0;
  frame[2] = 5;
  frame[3] = XBEE_APIFRAME_ATCOMMMAND;
  frame[4] = 0;
  frame[5] = 'P';
  frame[6] = 'M';
  frame[7]  = mode;

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
 *   Perform a series of queries updating struct and printing settings to
 *   SYSLOG.
 *
 ****************************************************************************/

void xbee_regdump(FAR struct xbee_priv_s *priv)
{
  xbee_query_firmwareversion(priv);

  wlinfo("XBee Firmware Version: %04x\n", priv->firmwareversion);

  xbee_send_atquery(priv, "CE");
  xbee_send_atquery(priv, "A1");
  xbee_send_atquery(priv, "A2");
  xbee_send_atquery(priv, "AI");
  xbee_send_atquery(priv, "SP");
  xbee_send_atquery(priv, "RR");
  xbee_send_atquery(priv, "MM");
  xbee_send_atquery(priv, "PL");
  xbee_send_atquery(priv, "PM");
}

/****************************************************************************
 * drivers/wireless/ieee802154/xbee/xbee.h
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

#ifndef __DRIVERS_WIRELESS_IEEE802154_XBEE_H
#define __DRIVERS_WIRELESS_IEEE802154_XBEE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/wqueue.h>
#include <nuttx/spi/spi.h>

#include <nuttx/wireless/ieee802154/xbee.h>

#include "xbee_notif.h"
#include "xbee_dataind.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration *************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#  error High priority work queue required in this driver
#endif

#ifndef CONFIG_IEEE802154_XBEE_FREQUENCY
#  define CONFIG_IEEE802154_XBEE_FREQUENCY 2000000
#endif

#ifndef CONFIG_SPI_EXCHANGE
#  error CONFIG_SPI_EXCHANGE required for this driver
#endif

#if !defined(CONFIG_XBEE_NNOTIF) || CONFIG_XBEE_NNOTIF <= 0
#  undef CONFIG_XBEE_NNOTIF
#  define CONFIG_XBEE_NNOTIF 6
#endif

#if !defined(CONFIG_XBEE_NDATAIND) || CONFIG_XBEE_NDATAIND <= 0
#  undef CONFIG_XBEE_NDATAIND
#  define CONFIG_XBEE_NDATAIND 8
#endif

#define XBEE_APIFRAME_MODEMSTATUS       0x8A
#define XBEE_APIFRAME_ATCOMMMAND        0x08
#define XBEE_APIFRAME_ATCOMMMANDQUEUED  0x09
#define XBEE_APIFRAME_ATRESPONSE        0x88
#define XBEE_APIFRAME_REMOTEREQUEST     0x17
#define XBEE_APIFRAME_REMOTERESPONSE    0x97
#define XBEE_APIFRAME_TXREQ_EADDR       0x00
#define XBEE_APIFRAME_TXREQ_SADDR       0x01
#define XBEE_APIFRAME_TXSTATUS          0x89
#define XBEE_APIFRAME_RX_EADDR          0x80
#define XBEE_APIFRAME_RX_SADDR          0x81
#define XBEE_APIFRAME_RXIO_EADDR        0x82
#define XBEE_APIFRAME_RXIO_SADDR        0x83

#define XBEE_EPASSOCFLAGS_PANID_REASSIGN    1
#define XBEE_EPASSOCFLAGS_CHAN_REASSIGN     2
#define XBEE_EPASSOCFLAGS_AUTOASSOC         4
#define XBEE_EPASSOCFLAGS_POLLONWAKE        8

#define XBEE_COORDASSOCFLAGS_PANID_REASSIGN 1
#define XBEE_COORDASSOCFLAGS_CHAN_REASSIGN  2
#define XBEE_COORDASSOCFLAGS_ALLOWASSOC     4

/* Size of read buffer active for all of the transaction. i.e. must be big enough
 * to handle full transmit and receive.
 */

#define XBEE_RXBUF_SIZE                 256

#define XBEE_STARTBYTE                  0x7E

#define XBEE_APIFRAME_OVERHEAD          4

#define XBEE_APIFRAMEINDEX_STARTBYTE    0
#define XBEE_APIFRAMEINDEX_LENGTHMSB    1
#define XBEE_APIFRAMEINDEX_LENGTHLSB    2
#define XBEE_APIFRAMEINDEX_TYPE         3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Enumeration of Modem Status values */

enum xbee_modemstatus_e
{
  XBEE_MODEMSTATUS_HARDRESET = 0,
  XBEE_MODEMSTATUS_WATCHDOGRESET,
  XBEE_MODEMSTATUS_ASSOCIATED,
  XBEE_MODEMSTATUS_NONETWORK,
  XBEE_MODEMSTATUS_COORD,
  XBEE_MODEMSTATUS_VOLTAGETOOHIGH,
};

enum xbee_response_e
{
  XBEE_RESP_MODEMSTATUS,
  XBEE_RESP_AT_FIRMWAREVERSION,
  XBEE_RESP_AT_HARDWAREVERSION,
  XBEE_RESP_AT_NETWORKID,
  XBEE_RESP_AT_SERIALHIGH,
  XBEE_RESP_AT_SERIALLOW,
  XBEE_RESP_AT_SOURCEADDR,
  XBEE_RESP_AT_CHAN,

  /* Skip some allow for new AT commands */
};

struct xbee_respwaiter_s
{
  FAR struct xbee_respwaiter_s *flink;
  sem_t sem;
  enum xbee_response_e resp_id;
};

/* An XBee device instance */

struct xbee_priv_s
{
  /* Low-level MCU-specific support */

  FAR const struct xbee_lower_s *lower;
  FAR struct spi_dev_s *spi;      /* Saved SPI interface instance */

  FAR struct xbee_maccb_s   *cb;  /* Head of a list of XBee MAC callbacks */

  FAR struct iob_s *rx_apiframes; /* List of incoming API frames to process */

  struct work_s attnwork;         /* For deferring interrupt work to work queue */
  sem_t         exclsem;          /* Exclusive access to this struct */

  WDOG_ID assocwd;                /* Association watchdog */
  struct work_s assocwork;        /* For polling for association status */

  volatile bool attn_latched;     /* Latched state of ATTN */

  sq_queue_t waiter_queue;        /* List of response waiters */

  sq_queue_t tx_queue;            /* List of pending TX requests */
  uint8_t frameid;                /* For differentiating AT request/response */

  uint16_t firmwareversion;

  /************* Fields related to addressing and coordinator *****************/

  /* Holds all address information (Extended, Short, and PAN ID) for the MAC. */

  struct ieee802154_addr_s addr;

  struct ieee802154_pandesc_s pandesc;

  /******************* Fields related to notifications ************************/

  /* Pre-allocated notifications to be passed to the registered callback.  These
   * need to be freed by the application using xbee_xxxxnotif_free when
   * the callee layer is finished with it's use.
   */

  FAR struct xbee_notif_s *notif_free;
  struct xbee_notif_s notif_pool[CONFIG_XBEE_NNOTIF];
  sem_t notif_sem;
  uint8_t nclients;

  /******************* Fields related to data indications *********************/

  /* Pre-allocated notifications to be passed to the registered callback.  These
   * need to be freed by the application using xbee_dataind_free when
   * the callee layer is finished with it's use.
   */

  FAR struct xbee_dataind_s *dataind_free;
  struct xbee_dataind_s dataind_pool[CONFIG_XBEE_NDATAIND];
  sem_t dataind_sem;

  /****************** Uncategorized MAC PIB attributes ***********************/

  /* What type of device is this node acting as */

  enum ieee802154_devmode_e devmode : 2;

  /****************** PHY attributes ***********************/

  uint8_t chan;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#define xbee_givesem(s) sem_post(s)

static inline int xbee_takesem(sem_t *sem, bool allowinterrupt)
{
  int ret;
  do
    {
      /* Take a count from the semaphore, possibly waiting */

      ret = sem_wait(sem);
      if (ret < 0)
        {
          /* EINTR is the only error that we expect */

          DEBUGASSERT(get_errno() == EINTR);

          if (allowinterrupt)
            {
              return -EINTR;
            }
        }
    }
  while (ret != OK);

  return OK;
}

#ifdef CONFIG_XBEE_LOCK_VERBOSE
#define xbee_unlock(dev) \
  xbee_givesem(&dev->exclsem); \
  wlinfo("MAC unlocked\n");
#else
#define xbee_unlock(dev) \
  xbee_givesem(&dev->exclsem);
#endif

#define xbee_lock(dev, allowinterrupt) \
  xbee_lockpriv(dev, allowinterrupt, __FUNCTION__)

static inline int xbee_lockpriv(FAR struct xbee_priv_s *dev,
                    bool allowinterrupt, FAR const char *funcname)
{
  int ret;

#ifdef CONFIG_XBEE_LOCK_VERBOSE
  wlinfo("Locking MAC: %s\n", funcname);
#endif
  ret = xbee_takesem(&dev->exclsem, allowinterrupt);
  if (ret < 0)
    {
      wlwarn("Failed to lock MAC\n");
    }
  else
    {
#ifdef CONFIG_XBEE_LOCK_VERBOSE
      wlinfo("MAC locked\n");
#endif
    }

  return ret;
}

/****************************************************************************
 * Name: xbee_register_respwaiter
 *
 * Description:
 *   Register a respone waiter
 *
 ****************************************************************************/

static inline void xbee_register_respwaiter(FAR struct xbee_priv_s *priv,
                                            FAR struct xbee_respwaiter_s *waiter)
{
  sq_addlast((sq_entry_t *)waiter, &priv->waiter_queue);
}

/****************************************************************************
 * Name: xbee_unregister_respwaiter
 *
 * Description:
 *   Unregister a respone waiter
 *
 ****************************************************************************/

static inline void xbee_unregister_respwaiter(FAR struct xbee_priv_s *priv,
                                              FAR struct xbee_respwaiter_s *waiter)
{
  sq_rem((sq_entry_t *)waiter, &priv->waiter_queue);
}

/****************************************************************************
 * Name: xbee_notify_respwaiter
 *
 * Description:
 *   Check to see if there are any respwaiters waiting for this response type.
 *   If so, signal them.
 *
 ****************************************************************************/

static inline void xbee_notify_respwaiter(FAR struct xbee_priv_s *priv,
                                          enum xbee_response_e resp_id)
{
  FAR struct xbee_respwaiter_s *waiter;

  waiter = (FAR struct xbee_respwaiter_s *)sq_peek(&priv->waiter_queue);

  while (waiter != NULL)
    {
      if (waiter->resp_id == resp_id)
        {
          sem_post(&waiter->sem);
        }

      waiter = (FAR struct xbee_respwaiter_s *)sq_next((FAR sq_entry_t *)waiter);
    }
}

/****************************************************************************
 * Name: xbee_next_frameid
 *
 * Description:
 *   Increment the frame id.  This is used to coordinate TX requests with subsequent
 *   TX status frames received by the XBee device. We must skip value 0 since
 *   that value is to tell the XBee not to provide a status response.
 *
 ****************************************************************************/

static inline uint8_t xbee_next_frameid(FAR struct xbee_priv_s *priv)
{
  priv->frameid++;
  if (priv->frameid == 0)
    {
      priv->frameid = 1;
    }

  return priv->frameid;
}

/****************************************************************************
 * Name: xbee_insert_checksum
 *
 * Description:
 *   Insert checksum into outbound API frame.
 *
 * Parameters:
 *    frame - pointer to the frame data
 *    framelen - size of the overall frame. NOT the data length field
 *
 ****************************************************************************/

static inline void xbee_insert_checksum(FAR uint8_t *frame, uint16_t framelen)
{
  int i;
  uint8_t checksum = 0;

  DEBUGASSERT(framelen > XBEE_APIFRAME_OVERHEAD);

  /* Skip the start byte and frame length */

  for (i = 3; i < framelen - 1; i++)
  {
    checksum += frame[i];
  }

  frame[framelen - 1] = 0xFF - checksum;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_send_apiframe
 *
 * Description:
 *   Write an api frame over SPI
 *
 ****************************************************************************/

void xbee_send_apiframe(FAR struct xbee_priv_s *priv,
                        FAR const uint8_t *frame, uint16_t framelen);

/****************************************************************************
 * Name: xbee_at_query
 *
 * Description:
 *   Helper function to query a AT Command value.
 *
 ****************************************************************************/

void xbee_at_query(FAR struct xbee_priv_s *priv, FAR const char *atcommand);

/****************************************************************************
 * Name: xbee_query_firmwareversion
 *
 * Description:
 *   Sends API frame with AT command request in order to get the firmware version
 *   from the device.
 *
 ****************************************************************************/

void xbee_query_firmwareversion(FAR struct xbee_priv_s *priv);

/****************************************************************************
 * Name: xbee_query_panid
 *
 * Description:
 *   Sends API frame with AT command request in order to get the PAN ID
 *   (Network ID) from the device.
 *
 ****************************************************************************/

void xbee_query_panid(FAR struct xbee_priv_s *priv);

/****************************************************************************
 * Name: xbee_query_eaddr
 *
 * Description:
 *   Sends API frame with AT command request in order to get the IEEE 802.15.4
 *   Extended Address. (Serial Number) from the device.
 *
 ****************************************************************************/

void xbee_query_eaddr(FAR struct xbee_priv_s *priv);

/****************************************************************************
 * Name: xbee_query_saddr
 *
 * Description:
 *   Sends API frame with AT command request in order to get the
 *   Short Address. (Source Address (MY)) from the device.
 *
 ****************************************************************************/

void xbee_query_saddr(FAR struct xbee_priv_s *priv);

/****************************************************************************
 * Name: xbee_query_chan
 *
 * Description:
 *   Sends API frame with AT command request in order to get the RF Channel
 *   (Operating Channel) from the device.
 *
 ****************************************************************************/

void xbee_query_chan(FAR struct xbee_priv_s *priv);

/****************************************************************************
 * Name: xbee_query_assoc
 *
 * Description:
 *   Sends API frame with AT command request in order to get the association
 *   status (Association Indication) of the device
 *
 ****************************************************************************/

void xbee_query_assoc(FAR struct xbee_priv_s *priv);

/****************************************************************************
 * Name: xbee_set_panid
 *
 * Description:
 *   Sends API frame with AT command request in order to set the PAN ID
 *   (Network ID) of the device.
 *
 ****************************************************************************/

void xbee_set_panid(FAR struct xbee_priv_s *priv, FAR const uint8_t *panid);

/****************************************************************************
 * Name: xbee_set_saddr
 *
 * Description:
 *   Sends API frame with AT command request in order to set the Short Address
 *   (Source Address (MY)) of the device
 *
 ****************************************************************************/

void xbee_set_saddr(FAR struct xbee_priv_s *priv, FAR const uint8_t *saddr);

/****************************************************************************
 * Name: xbee_set_chan
 *
 * Description:
 *   Sends API frame with AT command request in order to set the RF channel
 *   (Operatin Channel) of the device.
 *
 ****************************************************************************/

void xbee_set_chan(FAR struct xbee_priv_s *priv, uint8_t chan);

/****************************************************************************
 * Name: xbee_set_epassocflags
 *
 * Description:
 *   Set flags in 'A1' command register to determine how endpoint behaves
 *   with regards to association.
 *
 ****************************************************************************/

void xbee_set_epassocflags(FAR struct xbee_priv_s *priv, uint8_t flags);

/****************************************************************************
 * Name: xbee_set_coordassocflags
 *
 * Description:
 *   Set flags in 'AT' command register to determine how coordinator behaves
 *   with regards to association.
 *
 ****************************************************************************/

void xbee_set_coordassocflags(FAR struct xbee_priv_s *priv, uint8_t flags);

/****************************************************************************
 * Name: xbee_set_sleepperiod
 *
 * Description:
 *   Set Cyclic Sleep Period using 'SP' AT command.
 *
 ****************************************************************************/

void xbee_set_sleepperiod(FAR struct xbee_priv_s *priv, uint16_t period);

/****************************************************************************
 * Name: xbee_enable_coord
 *
 * Description:
 *   Enables/Disables coordinator mode using 'CE' command
 *
 ****************************************************************************/

void xbee_enable_coord(FAR struct xbee_priv_s *priv, bool enable);

/****************************************************************************
 * Name: xbee_regdump
 *
 * Description:
 *   Perform a series of queries updating struct and printing settings to SYSLOG.
 *
 ****************************************************************************/

void xbee_regdump(FAR struct xbee_priv_s *priv);

#endif /* __DRIVERS_WIRELESS_IEEE802154_XBEE_H */

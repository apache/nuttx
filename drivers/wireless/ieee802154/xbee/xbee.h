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

#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/xbee.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if !defined(CONFIG_SCHED_HPWORK) || !defined(CONFIG_SCHED_LPWORK)
#  error Both Low and High priority work queues are required for this driver
#endif

#ifndef CONFIG_IEEE802154_XBEE_FREQUENCY
#  define CONFIG_IEEE802154_XBEE_FREQUENCY 2000000
#endif

#ifndef CONFIG_SPI_EXCHANGE
#  error CONFIG_SPI_EXCHANGE required for this driver
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

/* Size of read buffer active for all of the transaction. i.e. must be big
 * enough to handle full transmit and receive.
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

  /* Fields related to interface with next layer */

  FAR struct xbee_maccb_s *cb;    /* Head of a list of XBee MAC callbacks */
  uint8_t nclients;               /* Number of registered callbacks */
  FAR struct iob_s *rx_apiframes; /* List of incoming API frames to process */
  struct work_s notifwork;        /* For deferring notifications to LPWORK queue */
  struct work_s attnwork;         /* For deferring interrupt work to work queue */
  volatile bool attn_latched;     /* Latched state of ATTN */
  sem_t primitive_sem;            /* Exclusive access to the primitive queue */
  sq_queue_t primitive_queue;     /* Queue of primitives to pass via notify()
                                   * callback to registered receivers */
  struct wdog_s assocwd;          /* Association watchdog */
  struct work_s assocwork;        /* For polling for association status */
  bool associating;               /* Are we currently associating */
  sem_t atquery_sem;              /* Only allow one AT query at a time */
  sem_t atresp_sem;               /* For signaling pending AT response received */
  char querycmd[2];               /* Stores the pending AT Query command */
  bool querydone;                 /* Used to tell waiting thread query is done */
  struct wdog_s atquery_wd;       /* Support AT Query timeout and retry */
  struct wdog_s reqdata_wd;       /* Support send timeout and retry */
  uint8_t frameid;                /* For differentiating AT request/response */
  sem_t tx_sem;                   /* Support a single pending transmit */
  sem_t txdone_sem;               /* For signalling tx is completed */
  bool txdone;
#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
  struct wdog_s lockup_wd;        /* Watchdog to protect for XBee lockup */
  struct work_s lockupwork;       /* For deferring lockup query check to LPWORK queue */
  struct work_s backupwork;       /* For deferring backing up parameters to LPWORK queue */
#endif

  /******************* Fields related to Xbee radio *************************/

  uint16_t firmwareversion;

  /************* Fields related to addressing and coordinator ***************/

  /* Holds all address information(Extended, Short, and PAN ID) for the MAC */

  struct ieee802154_addr_s addr;
  struct ieee802154_pandesc_s pandesc;

  /****************** Uncategorized MAC PIB attributes **********************/

  /* What type of device is this node acting as */

  enum ieee802154_devmode_e devmode : 2;

  /****************** PHY attributes ****************************************/

  uint8_t chan;
  uint8_t pwrlvl;
  bool boostmode;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_next_frameid
 *
 * Description:
 *   Increment the frame id.  This is used to coordinate TX requests with
 *   subsequent TX status frames received by the XBee device. We must skip
 *   value 0 since that value is to tell the XBee not to provide a status
 *   response.
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
 * Input Parameters:
 *    frame - pointer to the frame data
 *    framelen - size of the overall frame. NOT the data length field
 *
 ****************************************************************************/

static inline void xbee_insert_checksum(FAR uint8_t *frame,
                                        uint16_t framelen)
{
  int i;
  uint8_t checksum = 0;

  DEBUGASSERT(framelen > XBEE_APIFRAME_OVERHEAD);

  /* Skip the start byte and frame length */

  for (i = 3; i < framelen - 1; i++)
  {
    checksum += frame[i];
  }

  frame[framelen - 1] = 0xff - checksum;
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
 * Name: xbee_atquery
 *
 * Description:
 *   Sends AT Query and waits for response from device
 *
 ****************************************************************************/

int xbee_atquery(FAR struct xbee_priv_s *priv, FAR const char *atcommand);

/****************************************************************************
 * Name: xbee_send_atquery
 *
 * Description:
 *   Helper function to send the AT query to the XBee
 *
 ****************************************************************************/

void xbee_send_atquery(FAR struct xbee_priv_s *priv,
                       FAR const char *atcommand);

/****************************************************************************
 * Name: xbee_query_firmwareversion
 *
 * Description:
 *   Sends API frame with AT command request in order to get the firmware
 *   version from the device.
 *
 ****************************************************************************/

#define xbee_query_firmwareversion(priv) xbee_atquery(priv, "VR")

/****************************************************************************
 * Name: xbee_query_panid
 *
 * Description:
 *   Sends API frame with AT command request in order to get the PAN ID
 *   (Network ID) from the device.
 *
 ****************************************************************************/

#define xbee_query_panid(priv) xbee_atquery(priv, "ID")

/****************************************************************************
 * Name: xbee_query_eaddr
 *
 * Description:
 *   Sends API frame with AT command request in order to get the IEEE
 *   802.15.4 Extended Address. (Serial Number) from the device.
 *
 ****************************************************************************/

#define xbee_query_eaddr(priv) xbee_atquery(priv, "SH"); \
                               xbee_atquery(priv, "SL")

/****************************************************************************
 * Name: xbee_query_saddr
 *
 * Description:
 *   Sends API frame with AT command request in order to get the
 *   Short Address. (Source Address (MY)) from the device.
 *
 ****************************************************************************/

#define xbee_query_saddr(priv) xbee_atquery(priv, "MY")

/****************************************************************************
 * Name: xbee_query_chan
 *
 * Description:
 *   Sends API frame with AT command request in order to get the RF Channel
 *   (Operating Channel) from the device.
 *
 ****************************************************************************/

#define xbee_query_chan(priv) xbee_atquery(priv, "CH")

/****************************************************************************
 * Name: xbee_query_powerlevel
 *
 * Description:
 *   Sends API frame with AT command request in order to get the RF Power
 *   Level from the device.
 *
 ****************************************************************************/

#define xbee_query_powerlevel(priv) xbee_atquery(priv, "PL")

/****************************************************************************
 * Name: xbee_query_powermode
 *
 * Description:
 *   Sends API frame with AT command request in order to get the RF Power
 *   Mode from the device.
 *
 ****************************************************************************/

#define xbee_query_powermode(priv) xbee_atquery(priv, "PM")

/****************************************************************************
 * Name: xbee_query_assoc
 *
 * Description:
 *   Sends API frame with AT command request in order to get the association
 *   status (Association Indication) of the device
 *
 ****************************************************************************/

#define xbee_query_assoc(priv) xbee_atquery(priv, "AI")

/****************************************************************************
 * Name: xbee_save_params
 *
 * Description:
 *   Sends API frame with AT command request to write current parameters to
 *   non-volatile memory so that they are used after next reset.
 *
 ****************************************************************************/

#define xbee_save_params(priv) xbee_atquery(priv, "WR")

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
 *   Sends API frame with AT command request in order to set the Short
 *   Address (Source Address (MY)) of the device
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
 * Name: xbee_set_powerlevel
 *
 * Description:
 *   Sends API frame with AT command request in order to set the RF power
 *   level of the device.
 *
 ****************************************************************************/

void xbee_set_powerlevel(FAR struct xbee_priv_s *priv, uint8_t level);

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
 *   Perform a series of queries updating struct and printing settings.
 *
 ****************************************************************************/

void xbee_regdump(FAR struct xbee_priv_s *priv);

#endif /* __DRIVERS_WIRELESS_IEEE802154_XBEE_H */

/****************************************************************************
 * arch/arm/src/nrf91/nrf91_modem_sock.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/debug.h>
#include <errno.h>
#include <syslog.h>
#include <string.h>
#include <stdio.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>

#include <nuttx/wqueue.h>
#include <nuttx/arch.h>
#include <nuttx/net/usrsock.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/signal.h>

#include <nuttx/wireless/lte/lte_ioctl.h>
#include <nuttx/wireless/lte/lte.h>

#undef s6_addr
#include "nrf_socket.h"
#include "nrf_modem_at.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_USRSOCK_CUSTOM
#  error CONFIG_NET_USRSOCK_CUSTOM must be set
#endif

#ifndef CONFIG_SCHED_LPWORK
#  error CONFIG_SCHED_LPWORK must be set
#endif

#define NRF91_USRSOCK_BUFSIZE (4096)

#define MAX_CTRL_SOCKET_COUNT (1)
#define MAX_SOCKET_COUNT      (NRF_MODEM_MAX_SOCKET_COUNT + \
                               MAX_CTRL_SOCKET_COUNT)
#define CTRL_SOCKET_FIRST     (NRF_MODEM_MAX_SOCKET_COUNT)

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct nrf91_sock_s
{
  bool in_use;
  bool recvpending;
};

struct nrf91_usrsock_s
{
  uint8_t             in[NRF91_USRSOCK_BUFSIZE];
  uint8_t             out[NRF91_USRSOCK_BUFSIZE];
  struct work_s       pollwork;
  struct nrf_pollfd   pollfd;
  struct nrf91_sock_s sock[MAX_SOCKET_COUNT];
};

typedef int (*usrsock_handler_t)(struct nrf91_usrsock_s *usrsock,
                                 const void *data, size_t len);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nrf91_usrsock_socket_handler(struct nrf91_usrsock_s *usrsock,
                                        const void *data, size_t len);
static int nrf91_usrsock_close_handler(struct nrf91_usrsock_s *usrsock,
                                       const void *data, size_t len);
static int nrf91_usrsock_connect_handler(struct nrf91_usrsock_s *usrsock,
                                         const void *data, size_t len);
static int nrf91_usrsock_sendto_handler(struct nrf91_usrsock_s *usrsock,
                                        const void *data, size_t len);
static int nrf91_usrsock_recvfrom_handler(struct nrf91_usrsock_s *usrsock,
                                          const void *data, size_t len);
static int nrf91_usrsock_setsockopt_handler(struct nrf91_usrsock_s *usrsock,
                                            const void *data, size_t len);
static int nrf91_usrsock_getsockopt_handler(struct nrf91_usrsock_s *usrsock,
                                            const void *data, size_t len);
static int nrf91_usrsock_getsockname_handler(struct nrf91_usrsock_s *usrsock,
                                             const void *data, size_t len);
static int nrf91_usrsock_getpeername_handler(struct nrf91_usrsock_s *usrsock,
                                             const void *data, size_t len);
static int nrf91_usrsock_bind_handler(struct nrf91_usrsock_s *usrsock,
                                      const void *data, size_t len);
static int nrf91_usrsock_listen_handler(struct nrf91_usrsock_s *usrsock,
                                        const void *data, size_t len);
static int nrf91_usrsock_accept_handler(struct nrf91_usrsock_s *usrsock,
                                        const void *data, size_t len);
static int nrf91_usrsock_ioctl_handler(struct nrf91_usrsock_s *usrsock,
                                       const void *data, size_t len);
static int nrf91_usrsock_shutdown_handler(struct nrf91_usrsock_s *usrsock,
                                          const void *data, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nrf91_usrsock_s g_usrsock;

static const usrsock_handler_t g_usrsock_handler[] =
{
  [USRSOCK_REQUEST_SOCKET]      = nrf91_usrsock_socket_handler,
  [USRSOCK_REQUEST_CLOSE]       = nrf91_usrsock_close_handler,
  [USRSOCK_REQUEST_CONNECT]     = nrf91_usrsock_connect_handler,
  [USRSOCK_REQUEST_SENDTO]      = nrf91_usrsock_sendto_handler,
  [USRSOCK_REQUEST_RECVFROM]    = nrf91_usrsock_recvfrom_handler,
  [USRSOCK_REQUEST_SETSOCKOPT]  = nrf91_usrsock_setsockopt_handler,
  [USRSOCK_REQUEST_GETSOCKOPT]  = nrf91_usrsock_getsockopt_handler,
  [USRSOCK_REQUEST_GETSOCKNAME] = nrf91_usrsock_getsockname_handler,
  [USRSOCK_REQUEST_GETPEERNAME] = nrf91_usrsock_getpeername_handler,
  [USRSOCK_REQUEST_BIND]        = nrf91_usrsock_bind_handler,
  [USRSOCK_REQUEST_LISTEN]      = nrf91_usrsock_listen_handler,
  [USRSOCK_REQUEST_ACCEPT]      = nrf91_usrsock_accept_handler,
  [USRSOCK_REQUEST_IOCTL]       = nrf91_usrsock_ioctl_handler,
  [USRSOCK_REQUEST_SHUTDOWN]    = nrf91_usrsock_shutdown_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf2nx_family
 ****************************************************************************/

static int nrf2nx_family(int nrf_family)
{
  int nx_family = 0;

  if (nrf_family == NRF_AF_UNSPEC)
    {
      nx_family = AF_UNSPEC;
    }
  else if (nrf_family == NRF_AF_INET)
    {
      nx_family = AF_INET;
    }
  else if (nrf_family == NRF_AF_INET6)
    {
      nx_family = AF_INET6;
    }
  else if (nrf_family == NRF_AF_PACKET)
    {
      nx_family = AF_PACKET;
    }

  return nx_family;
}

/****************************************************************************
 * Name: nx2nrf_family
 ****************************************************************************/

static int nx2nrf_family(int nx_family)
{
  int nrf_family = 0;

  if (nx_family == AF_UNSPEC)
    {
      nrf_family = NRF_AF_UNSPEC;
    }
  else if (nx_family == AF_INET)
    {
      nrf_family = NRF_AF_INET;
    }
  else if (nx_family == AF_INET6)
    {
      nrf_family = NRF_AF_INET6;
    }
  else if (nx_family == AF_PACKET)
    {
      nrf_family = NRF_AF_PACKET;
    }
  else
    {
      nerr("unsupported nrf family %d\n", nx_family);
    }

  return nrf_family;
}

/****************************************************************************
 * Name: nx2nrf_type
 ****************************************************************************/

static int nx2nrf_type(int nx_type)
{
  int nrf_type = 0;

  if (nx_type == SOCK_STREAM)
    {
      nrf_type = NRF_SOCK_STREAM;
    }
  else if (nx_type == SOCK_DGRAM)
    {
      nrf_type = NRF_SOCK_DGRAM;
    }
  else if (nx_type == SOCK_RAW)
    {
      nrf_type = NRF_SOCK_RAW;
    }
  else
    {
      nerr("unsupported nrf type %d\n", nx_type);
    }

  return nrf_type;
}

/****************************************************************************
 * Name: nx2nrf_protocol
 ****************************************************************************/

static int nx2nrf_protocol(int nx_protocol)
{
  int nrf_protocol = 0;

  if (nx_protocol == IPPROTO_IP)
    {
      nrf_protocol = NRF_IPPROTO_IP;
    }
  else if (nx_protocol == IPPROTO_TCP)
    {
      nrf_protocol = NRF_IPPROTO_TCP;
    }
  else if (nx_protocol == IPPROTO_UDP)
    {
      nrf_protocol = NRF_IPPROTO_UDP;
    }
  else if (nx_protocol == IPPROTO_IPV6)
    {
      nrf_protocol = NRF_IPPROTO_IPV6;
    }
  else if (nx_protocol == IPPROTO_RAW)
    {
      nrf_protocol = NRF_IPPROTO_RAW;
    }
  else
    {
      nerr("unsupported nrf protocol %d\n", nx_protocol);
    }

  return nrf_protocol;
}

/****************************************************************************
 * Name: nx2nrf_sockaddr
 ****************************************************************************/

static void nx2nrf_sockaddr(struct sockaddr *nxaddress,
                            struct nrf_sockaddr *address)
{
  address->sa_family = nx2nrf_family(nxaddress->sa_family);

  if (address->sa_family == NRF_AF_INET)
    {
      struct sockaddr_in     *tmp1 = (struct sockaddr_in *)nxaddress;
      struct nrf_sockaddr_in *tmp2 = (struct nrf_sockaddr_in *)address;

      tmp2->sin_port        = tmp1->sin_port;
      tmp2->sin_addr.s_addr = tmp1->sin_addr.s_addr;
      tmp2->sin_len = sizeof(struct nrf_sockaddr_in);
    }
  else if (address->sa_family == NRF_AF_INET6)
    {
      struct sockaddr_in6     *tmp1 = (struct sockaddr_in6 *)nxaddress;
      struct nrf_sockaddr_in6 *tmp2 = (struct nrf_sockaddr_in6 *)address;

      tmp2->sin6_port     = tmp1->sin6_port;
      memcpy(&tmp2->sin6_addr, &tmp1->sin6_addr, sizeof(struct in6_addr));
      tmp2->sin6_scope_id = tmp1->sin6_scope_id;
      tmp2->sin6_len = sizeof(struct nrf_sockaddr_in6);
    }
  else
    {
      nerr("unsupported nrf sa_family %d\n", address->sa_family);
      address->sa_family = 0;
    }
}

/****************************************************************************
 * Name: nrf2nx_sockaddr
 ****************************************************************************/

static void nrf2nx_sockaddr(struct nrf_sockaddr *address,
                            struct sockaddr *nxaddress)
{
  nxaddress->sa_family = nrf2nx_family(address->sa_family);

  if (nxaddress->sa_family == AF_INET)
    {
      struct nrf_sockaddr_in *tmp1 = (struct nrf_sockaddr_in *)address;
      struct sockaddr_in     *tmp2 = (struct sockaddr_in *)nxaddress;

      tmp2->sin_port        = tmp1->sin_port;
      tmp2->sin_addr.s_addr = tmp1->sin_addr.s_addr;
    }
  else if (nxaddress->sa_family == AF_INET6)
    {
      struct nrf_sockaddr_in6 *tmp1 = (struct nrf_sockaddr_in6 *)address;
      struct sockaddr_in6     *tmp2 = (struct sockaddr_in6 *)nxaddress;

      tmp2->sin6_port        = tmp1->sin6_port;
      tmp2->sin6_flowinfo    = 0;
      memcpy(&tmp2->sin6_addr, &tmp1->sin6_addr, sizeof(struct in6_addr));
      tmp2->sin6_scope_id = tmp1->sin6_scope_id;
    }
  else
    {
      nerr("unsupported nx sa_family %d\n", nxaddress->sa_family);
      nxaddress->sa_family = 0;
    }
}

/****************************************************************************
 * Name: nrf_usrsock_shutdown
 ****************************************************************************/

static int nrf_usrsock_shutdown(int sockfd, int how)
{
  /* TODO */

  return 0;
}

/****************************************************************************
 * Name: nrf91_usrsock_send
 ****************************************************************************/

static int nrf91_usrsock_send(struct nrf91_usrsock_s *usrsock,
                              const void *buf, size_t len)
{
  return usrsock_response(buf, len, NULL);
}

/****************************************************************************
 * Name: nrf91_usrsock_send_ack
 ****************************************************************************/

static int nrf91_usrsock_send_ack(struct nrf91_usrsock_s *usrsock,
                                  uint32_t xid, int32_t result)
{
  struct usrsock_message_req_ack_s ack;

  ack.head.msgid = USRSOCK_MESSAGE_RESPONSE_ACK;
  ack.head.flags = (result == -EINPROGRESS);
  ack.head.events = 0;

  ack.xid    = xid;
  ack.result = result;

  return nrf91_usrsock_send(usrsock, &ack, sizeof(ack));
}

/****************************************************************************
 * Name: nrf91_usrsock_send_dack
 ****************************************************************************/

static int nrf91_usrsock_send_dack(struct nrf91_usrsock_s *usrsock,
                                   struct usrsock_message_datareq_ack_s *ack,
                                   uint32_t xid, int32_t result,
                                   uint16_t valuelen,
                                   uint16_t valuelen_nontrunc)
{
  ack->reqack.head.msgid = USRSOCK_MESSAGE_RESPONSE_DATA_ACK;
  ack->reqack.head.flags = 0;
  ack->reqack.head.events = 0;

  ack->reqack.xid    = xid;
  ack->reqack.result = result;

  if (result < 0)
    {
      result             = 0;
      valuelen           = 0;
      valuelen_nontrunc  = 0;
    }
  else if (valuelen > valuelen_nontrunc)
    {
      valuelen           = valuelen_nontrunc;
    }

  ack->valuelen          = valuelen;
  ack->valuelen_nontrunc = valuelen_nontrunc;

  return nrf91_usrsock_send(usrsock, ack, sizeof(*ack) + valuelen + result);
}

/****************************************************************************
 * Name: nrf91_usrsock_send_event
 ****************************************************************************/

static int nrf91_usrsock_send_event(struct nrf91_usrsock_s *usrsock,
                                    int16_t usockid, uint16_t events)
{
  struct usrsock_message_socket_event_s event;

  event.head.msgid  = USRSOCK_MESSAGE_SOCKET_EVENT;
  event.head.flags  = USRSOCK_MESSAGE_FLAG_EVENT;
  event.head.events = events;

  event.usockid = usockid;

  return nrf91_usrsock_send(usrsock, &event, sizeof(event));
}

/****************************************************************************
 * Name: nrf91_usrsock_event_callback
 ****************************************************************************/

static int nrf91_usrsock_event_callback(int16_t usockid, uint16_t events)
{
  return nrf91_usrsock_send_event(&g_usrsock, usockid, events);
}

/****************************************************************************
 * Name: nrf91_modem_getver
 ****************************************************************************/

static int nrf91_modem_getver(lte_version_t *version)
{
  char buffer1[LTE_VER_FIRMWARE_LEN];
  char buffer2[LTE_VER_NP_PACKAGE_LEN];
  int  ret = OK;

  memset(version, 0, sizeof(*version));
  ret = nrf_modem_at_scanf("AT%HWVERSION", "%%HWVERSION: %31s %31s",
                           buffer1, buffer2);
  if (ret == 2)
    {
      strncpy(version->bb_product, buffer1, LTE_VER_BB_PRODUCT_LEN);
      strncpy(version->np_package, buffer2, LTE_VER_NP_PACKAGE_LEN);
    }

  ret = nrf_modem_at_scanf("AT+CGMR", "%31s", buffer1);
  if (ret > 0)
    {
      strncpy(version->fw_version, buffer1, LTE_VER_FIRMWARE_LEN);
      ret = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf91_modem_actpdn
 *
 * Description:
 *   Activate the default packet data network: optionally (re)configure the
 *   APN, attach to the network (AT+CFUN=1), wait for registration and read
 *   the assigned IP address.
 *
 ****************************************************************************/

static int nrf91_modem_actpdn(FAR lte_apn_setting_t *apn,
                              FAR lte_pdn_t *pdn)
{
  FAR const char *pdptype = "IP";
  char resp[64];
  FAR char *p;
  FAR char *q;
  size_t n;
  int stat = 0;
  int tmp = 0;
  int elapsed = 0;
  int ret;

  switch (apn->ip_type)
    {
      case LTE_IPTYPE_V6:
        pdptype = "IPV6";
        break;
      case LTE_IPTYPE_V4V6:
        pdptype = "IPV4V6";
        break;
      default:
        pdptype = "IP";
        break;
    }

  /* (Re)configure the default PDN context when an APN is provided.  Toggle
   * the radio off first so the new context is used on (re)attach.
   */

  if (apn->apn != NULL && apn->apn[0] != '\0')
    {
      nrf_modem_at_printf("AT+CFUN=4");

      ret = nrf_modem_at_printf("AT+CGDCONT=0,\"%s\",\"%s\"",
                                pdptype, apn->apn);
      if (ret < 0)
        {
          nerr("AT+CGDCONT failed %d\n", ret);
          return ret;
        }

      if (apn->auth_type != LTE_APN_AUTHTYPE_NONE &&
          apn->user_name != NULL && apn->password != NULL)
        {
          ret = nrf_modem_at_printf("AT+CGAUTH=0,%u,\"%s\",\"%s\"",
                                    (unsigned)apn->auth_type,
                                    apn->user_name, apn->password);
          if (ret < 0)
            {
              nerr("AT+CGAUTH failed %d\n", ret);
              return ret;
            }
        }
    }

  /* Attach to the network */

  ret = nrf_modem_at_printf("AT+CFUN=1");
  if (ret < 0)
    {
      nerr("AT+CFUN=1 failed %d\n", ret);
      return ret;
    }

  /* Wait for network registration (1 = home, 5 = roaming) */

  while (elapsed < 120)
    {
      ret = nrf_modem_at_scanf("AT+CEREG?", "+CEREG: %d,%d", &tmp, &stat);
      if (ret >= 2 && (stat == 1 || stat == 5))
        {
          break;
        }

      nxsig_usleep(1000 * 1000);
      elapsed++;
    }

  if (stat != 1 && stat != 5)
    {
      nerr("LTE registration timed out, CEREG stat=%d\n", stat);
      return -ETIMEDOUT;
    }

  /* Fill the PDN info and read the assigned IP address from
   * "+CGPADDR: 0,\"a.b.c.d\"".
   */

  memset(pdn, 0, sizeof(*pdn));
  pdn->session_id = 0;
  pdn->active     = LTE_PDN_ACTIVE;
  pdn->apn_type   = apn->apn_type;
  pdn->ipaddr_num = 0;

  ret = nrf_modem_at_cmd(resp, sizeof(resp), "AT+CGPADDR=0");
  if (ret == 0)
    {
      p = strchr(resp, '"');
      if (p != NULL)
        {
          p++;
          q = strchr(p, '"');
          if (q != NULL)
            {
              n = (size_t)(q - p);
              if (n >= LTE_IPADDR_MAX_LEN)
                {
                  n = LTE_IPADDR_MAX_LEN - 1;
                }

              pdn->address[0].ip_type = apn->ip_type;
              memcpy(pdn->address[0].address, p, n);
              pdn->address[0].address[n] = '\0';
              pdn->ipaddr_num = 1;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nrf91_modem_bits
 *
 * Description:
 *   Format the low 'nbits' of 'value' as a binary string (MSB first), as
 *   used by the 3GPP timer fields of AT+CPSMS and AT+CEDRXS.
 *
 * Input Parameters:
 *   value - Value to format.
 *   nbits - Number of low-order bits to emit.
 *   out   - Output buffer (must hold at least nbits + 1 bytes).
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void nrf91_modem_bits(uint8_t value, int nbits, FAR char *out)
{
  int i;

  for (i = 0; i < nbits; i++)
    {
      out[i] = (value & (1 << (nbits - 1 - i))) ? '1' : '0';
    }

  out[nbits] = '\0';
}

/****************************************************************************
 * Name: nrf91_modem_setpsm
 *
 * Description:
 *   Apply PSM settings (LTE_CMDID_SETPSM) by translating the LAPI
 *   lte_psm_setting_t into the AT+CPSMS command.
 *
 * Input Parameters:
 *   psm - PSM settings to apply.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf91_modem_setpsm(FAR lte_psm_setting_t *psm)
{
  /* LAPI timer-unit enums -> 3GPP encoded unit bits (TS 24.008 GPRS timers).
   * T3412 extended (periodic TAU) and T3324 (active time).
   */

  static const uint8_t t3412[] =
    {
      3, 4, 5, 0, 1, 2, 6, 7  /* 2s,30s,1m,10m,1h,10h,320h,deact */
    };

  static const uint8_t t3324[] =
    {
      0, 1, 2, 7              /* 2s,1m,6m,deact */
    };

  char tau[9];
  char act[9];
  uint8_t b;

  if (!psm->enable)
    {
      return nrf_modem_at_printf("AT+CPSMS=0");
    }

  b = (uint8_t)((t3412[psm->ext_periodic_tau_time.unit & 0x7] << 5) |
                (psm->ext_periodic_tau_time.time_val & 0x1f));
  nrf91_modem_bits(b, 8, tau);

  b = (uint8_t)((t3324[psm->req_active_time.unit & 0x3] << 5) |
                (psm->req_active_time.time_val & 0x1f));
  nrf91_modem_bits(b, 8, act);

  return nrf_modem_at_printf("AT+CPSMS=1,,,\"%s\",\"%s\"", tau, act);
}

/****************************************************************************
 * Name: nrf91_modem_setedrx
 *
 * Description:
 *   Apply eDRX settings (LTE_CMDID_SETEDRX) by translating the LAPI
 *   lte_edrx_setting_t into the AT+CEDRXS command.
 *
 * Input Parameters:
 *   edrx - eDRX settings to apply.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf91_modem_setedrx(FAR lte_edrx_setting_t *edrx)
{
  char value[5];
  int  acttype;

  if (!edrx->enable || edrx->act_type == LTE_EDRX_ACTTYPE_NOTUSE)
    {
      /* Disable eDRX and discard any stored value */

      return nrf_modem_at_printf("AT+CEDRXS=3");
    }

  /* AT+CEDRXS access technology: 4 = E-UTRAN WB-S1 (LTE-M),
   * 5 = E-UTRAN NB-S1 (NB-IoT). The LAPI eDRX cycle enum matches the 4-bit
   * 3GPP requested-eDRX value directly.
   */

  acttype = (edrx->act_type == LTE_EDRX_ACTTYPE_NBS1) ? 5 : 4;
  nrf91_modem_bits((uint8_t)(edrx->edrx_cycle & 0xf), 4, value);

  return nrf_modem_at_printf("AT+CEDRXS=2,%d,\"%s\"", acttype, value);
}

/****************************************************************************
 * Name: nrf91_ioctl_ltecmd
 ****************************************************************************/

static int nrf91_ioctl_ltecmd(int fd, int cmd, unsigned long arg)
{
  struct lte_ioctl_data_s  *ltecmd = (struct lte_ioctl_data_s *)(arg);
  int                     **result = (int **)ltecmd->outparam;
  int                       ret    = OK;

  switch (ltecmd->cmdid)
    {
      /* TODO: support for Nordic-specific functional modes 20-44 */

      case LTE_CMDID_POWERON:
        {
          ret = nrf_modem_at_printf("AT+CFUN=1");
          break;
        }

      case LTE_CMDID_POWEROFF:
        {
          ret = nrf_modem_at_printf("AT+CFUN=0");
          break;
        }

      case LTE_CMDID_RADIOON:
        {
          /* The radio is enabled by AT+CFUN=1 at power-on / PDN
           * activation, so there is nothing extra to do here.
           */

          ret = OK;
          break;
        }

      case LTE_CMDID_RADIOOFF:
        {
          ret = nrf_modem_at_printf("AT+CFUN=4");
          break;
        }

      case LTE_CMDID_ACTPDN:
        {
          lte_apn_setting_t **apn =
            (lte_apn_setting_t **)(ltecmd->inparam);
          lte_pdn_t **pdn =
            (lte_pdn_t **)(ltecmd->outparam + 1);
          ret = nrf91_modem_actpdn(*apn, *pdn);
          break;
        }

      case LTE_CMDID_GETVER:
        {
          lte_version_t **version =
            (lte_version_t **)(ltecmd->outparam + 1);
          ret = nrf91_modem_getver(*version);
          break;
        }

      case LTE_CMDID_GETIMEI:
        {
          char **imei = (char **)(ltecmd->outparam + 1);
          size_t **len = (size_t **)(ltecmd->outparam + 2);
          char buffer[32];
          char *ptr;
          ret = nrf_modem_at_cmd(buffer, 32, "AT+CGSN=1");
          ptr = strchr(buffer, '"') + 1;
          ptr[LTE_IMEI_LEN - 1] = 0;
          strncpy(*imei, ptr, **len);
          break;
        }

      case LTE_CMDID_GETQUAL:
      {
        lte_quality_t **quality =
          (lte_quality_t **)(ltecmd->outparam + 1);
        bool valid = false;
        int cresult;
        int tmp;
        int rsrp;
        int rsrq;
        int snr;

        (*quality)->rsrp = 0;
        (*quality)->rsrq = 0;
        (*quality)->rssi = 0;
        (*quality)->sinr = 0;

        /* RSRP/RSRQ via AT+CESQ - available whenever the modem is camped. */

        ret = nrf_modem_at_scanf("AT+CESQ",
                                 "+CESQ: %d,%d,%d,%d,%d,%d",
                                 &tmp, &tmp, &tmp, &tmp,
                                 &rsrq, &rsrp);
        if (ret > 0)
          {
            (*quality)->rsrq = (rsrq / 2) - 19;
            (*quality)->rsrp = rsrp - 140;
            valid = true;
          }
        else
          {
            nerr("AT+CESQ failed %d\n", ret);
          }

        /* SNR via AT%CONEVAL - the only SNR source on this modem (AT+CSQ is
         * not answered and carries no SNR). Reported SNR is the raw
         * value - 24.
         */

        ret = nrf_modem_at_scanf("AT%CONEVAL",
                                 "%%CONEVAL: %d,%d,%d,%d,%d,%d",
                                 &cresult, &tmp, &tmp, &tmp, &tmp, &snr);
        if (ret >= 6 && cresult == 0)
          {
            (*quality)->sinr = snr - 24;
          }

        (*quality)->valid = valid;
        ret = OK;
        break;
      }

      case LTE_CMDID_GETCELL:
      {
        lte_cellinfo_t **cell =
          (lte_cellinfo_t **)(ltecmd->outparam + 1);
        int band = 0;

        (*cell)->valid = false;
        (*cell)->phycell_id = 0;
        (*cell)->earfcn = 0;
        (*cell)->option = 0;
        (*cell)->nr_neighbor = 0;

        /* The modem AT set does not expose the raw EARFCN in a form the
         * scanf parser here can extract from AT%XMONITOR (quoted fields), so
         * report the serving band number via AT%XCBAND in the earfcn field.
         */

        ret = nrf_modem_at_scanf("AT%XCBAND", "%%XCBAND: %d", &band);
        if (ret > 0)
          {
            (*cell)->earfcn = (uint32_t)band;
            (*cell)->valid = true;
          }
        else
          {
            nerr("AT%%XCBAND failed %d\n", ret);
          }

        ret = OK;
        break;
      }

      case LTE_CMDID_SETPSM:
        {
          lte_psm_setting_t **psm =
            (lte_psm_setting_t **)(ltecmd->inparam);
          ret = nrf91_modem_setpsm(*psm);
          break;
        }

      case LTE_CMDID_SETEDRX:
        {
          lte_edrx_setting_t **edrx =
            (lte_edrx_setting_t **)(ltecmd->inparam);
          ret = nrf91_modem_setedrx(*edrx);
          break;
        }

      /* TODO: commands from include/nuttx/wireless/lte/lte.h */

      default:
        {
          nerr("unsupported cmd = %" PRId32, ltecmd->cmdid);
          break;
        }
    }

  if (result)
    {
      **result = ret;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf91_usrsock_ioctl
 ****************************************************************************/

static int nrf91_usrsock_ioctl(int fd, int cmd, unsigned long arg)
{
  int ret = OK;

  if (fd < NRF_MODEM_MAX_SOCKET_COUNT)
    {
      nerr("ioctl not supported for socket %d", fd);
      ret = -EACCES;
      goto errout;
    }

  switch (cmd)
    {
      case SIOCLTECMD:
        {
          ret = nrf91_ioctl_ltecmd(fd, cmd, arg);
          break;
        }

      default:
        {
          ret = -ENOTTY;
          nerr("ioctl unsupported cmd %d", cmd);
          break;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf91_usrsock_getsockopt_handler
 ****************************************************************************/

int nrf91_usrsock_setsockopt(int sockfd, int level, int optname,
                             const void *optval, unsigned int optlen)
{
  int ret = OK;

  ret = nrf_setsockopt(sockfd, level, optname, optval,
                        (nrf_socklen_t)optlen);
  if (ret < 0)
    {
      nerr("nrf_setsockopt failed %d", errno);
      ret = -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf91_usrsock_getsockopt_handler
 ****************************************************************************/

int nrf91_usrsock_getsockopt(int sockfd, int level, int optname,
                             void *optval, unsigned int *optlen)
{
  int ret = OK;

  ret = nrf_getsockopt(sockfd, level, optname, optval,
                       (nrf_socklen_t *)optlen);
  if (ret < 0)
    {
      nerr("nrf_getsockopt failed %d", errno);
      ret = -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf91_usrsock_poll_work
 ****************************************************************************/

static void nrf91_usrsock_poll_work(void *arg)
{
  struct nrf_pollfd *pollfd = arg;
  uint16_t           events = 0;
  int                ret    = OK;

  if (pollfd->revents & NRF_POLLIN)
    {
      events |= USRSOCK_EVENT_RECVFROM_AVAIL;
    }

  if (pollfd->revents & NRF_POLLERR)
    {
      events |= USRSOCK_EVENT_REMOTE_CLOSED;
    }

  if (pollfd->revents & NRF_POLLHUP)
    {
      events |= USRSOCK_EVENT_REMOTE_CLOSED;
    }

  if (events)
    {
      /* REVISIT: postpone REMOTE_CLOSED event if there are data to read */

      if (events & USRSOCK_EVENT_REMOTE_CLOSED)
        {
          while (g_usrsock.sock[pollfd->fd].recvpending == true)
            {
              nxsched_usleep(100);
            }
        }

      ret = nrf91_usrsock_event_callback(pollfd->fd, events);
      if (ret < 0)
        {
          nerr("usrsock_event_callback failed %d", ret);
        }
    }
}

/****************************************************************************
 * Name: nrf91_usrsock_poll_cb
 *
 *   The callback is invoked in an interrupt service routine.
 *
 ****************************************************************************/

static void nrf91_usrsock_poll_cb(struct nrf_pollfd *pollfd)
{
  memcpy(&g_usrsock.pollfd, pollfd, sizeof(struct nrf_pollfd));

  if (work_available(&g_usrsock.pollwork))
    {
      work_queue(LPWORK, &g_usrsock.pollwork, nrf91_usrsock_poll_work,
                 &g_usrsock.pollfd, 0);
    }
}

/****************************************************************************
 * Name: nrf91_usrsock_socket_handler
 ****************************************************************************/

static int nrf91_usrsock_socket_handler(struct nrf91_usrsock_s *usrsock,
                                        const void *data, size_t len)
{
  const struct usrsock_request_socket_s *req = data;
  struct nrf_modem_pollcb                cb;
  int                                    fd  = 0;
  int                                    ret = 0;

  if (req->type == SOCK_CTRL)
    {
      /* Only one CTRL socket supported for now */

      fd = CTRL_SOCKET_FIRST;
    }
  else
    {
      fd = nrf_socket(nx2nrf_family(req->domain),
                      nx2nrf_type(req->type),
                      nx2nrf_protocol(req->protocol));
      if (ret < 0)
        {
          nerr("nrf_socket failed %d", errno);
          ret = -errno;
        }
      else
        {
          cb.callback = nrf91_usrsock_poll_cb;
          cb.events   = NRF_POLLIN | NRF_POLLHUP;
          cb.oneshot  = false;

          ret = nrf_setsockopt(fd, NRF_SOL_SOCKET, NRF_SO_POLLCB, &cb,
                               sizeof(struct nrf_modem_pollcb));
          if (ret < 0)
            {
              nerr("failed to connect poll cb %d", errno);
              ret = -errno;
            }
        }
    }

  /* Mark socket as used */

  if (fd >= 0)
    {
      usrsock->sock[fd].in_use = true;
      usrsock->sock[fd].recvpending = false;
    }

  ret = nrf91_usrsock_send_ack(usrsock, req->head.xid, fd);

  if (ret >= 0 && fd >= 0)
    {
      ret = nrf91_usrsock_send_event(usrsock, fd,
                                     USRSOCK_EVENT_SENDTO_READY);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf91_usrsock_close_handler
 ****************************************************************************/

static int nrf91_usrsock_close_handler(struct nrf91_usrsock_s *usrsock,
                                       const void *data, size_t len)
{
  const struct usrsock_request_close_s *req = data;
  int                                   ret = OK;

  if (req->usockid < NRF_MODEM_MAX_SOCKET_COUNT)
    {
      ret = nrf_close(req->usockid);
      if (ret < 0)
        {
          nerr("nrf_close failed %d", errno);
          ret = -errno;
        }

      usrsock->sock[req->usockid].in_use = false;
    }

  return nrf91_usrsock_send_ack(usrsock, req->head.xid, ret);
}

/****************************************************************************
 * Name: nrf91_usrsock_connect_handler
 ****************************************************************************/

static int nrf91_usrsock_connect_handler(struct nrf91_usrsock_s *usrsock,
                                         const void *data, size_t len)
{
  const struct usrsock_request_connect_s *req = data;
  struct nrf_sockaddr_in6                 address;
  int                                     ret = 0;

  nx2nrf_sockaddr((struct sockaddr *)(req + 1),
                  (struct nrf_sockaddr *)&address);

  ret = nrf_connect(req->usockid,
                    (const struct nrf_sockaddr *)&address,
                    (nrf_socklen_t)req->addrlen);
  if (ret < 0)
    {
      nerr("nrf_connect failed %d", errno);
      ret = -errno;
    }

  return nrf91_usrsock_send_ack(usrsock, req->head.xid, ret);
}

/****************************************************************************
 * Name: nrf91_usrsock_sendto_handler
 ****************************************************************************/

static int nrf91_usrsock_sendto_handler(struct nrf91_usrsock_s *usrsock,
                                        const void *data, size_t len)
{
  const struct usrsock_request_sendto_s *req  = data;
  struct nrf_sockaddr_in6                address;
  struct sockaddr                       *tmp  = NULL;
  int                                    ret  = 0;
  bool                                   sent = 0;

  if (req->addrlen != 0)
    {
      tmp = (struct sockaddr *)(req + 1);
      nx2nrf_sockaddr(tmp, (struct nrf_sockaddr *)&address);
    }

  ret = nrf_sendto(req->usockid,
                   (const void *)(req + 1) + req->addrlen,
                   req->buflen,
                   req->flags,
                   req->addrlen ? (struct nrf_sockaddr *)&address : NULL,
                   (nrf_socklen_t)req->addrlen);
  if (ret < 0)
    {
      nerr("nrf_sendto failed %d", errno);
      ret = -errno;
    }

  sent = (ret > 0);
  ret = nrf91_usrsock_send_ack(usrsock, req->head.xid, ret);
  if (ret >= 0 && sent)
    {
      ret = nrf91_usrsock_send_event(usrsock, req->usockid,
                               USRSOCK_EVENT_SENDTO_READY);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf91_usrsock_recvfrom_handler
 ****************************************************************************/

static int nrf91_usrsock_recvfrom_handler(struct nrf91_usrsock_s *usrsock,
                                          const void *data, size_t len)
{
  const struct usrsock_request_recvfrom_s *req        = data;
  struct usrsock_message_datareq_ack_s    *ack        = NULL;
  struct sockaddr                         *tmp        = NULL;
  nrf_socklen_t                            outaddrlen = req->max_addrlen;
  socklen_t                                inaddrlen  = req->max_addrlen;
  size_t                                   buflen     = req->max_buflen;
  struct nrf_sockaddr_in6                  address;
  int                                      ret;
  int                                      recvlen;

  ack = (struct usrsock_message_datareq_ack_s *)usrsock->out;
  if (sizeof(*ack) + inaddrlen + buflen > sizeof(usrsock->out))
    {
      buflen = sizeof(usrsock->out) - sizeof(*ack) - inaddrlen;
    }

  ret = nrf_recvfrom(req->usockid,
                     (void *)(ack + 1) + inaddrlen,
                     buflen,
                     req->flags,
                     outaddrlen ? (struct nrf_sockaddr *)&address: NULL,
                     (nrf_socklen_t)outaddrlen ? &outaddrlen : NULL);
  if (ret < 0)
    {
      nerr("nrf_recvfrom failed %d", errno);
      ret = -errno;
    }

  if (outaddrlen != 0)
    {
      tmp = (struct sockaddr *)(ack + 1);
      nrf2nx_sockaddr((struct nrf_sockaddr *)&address, tmp);
    }

  recvlen = ret;

  if (ret > 0 && outaddrlen < inaddrlen)
    {
      memcpy((void *)(ack + 1) + outaddrlen,
             (void *)(ack + 1) + inaddrlen, ret);
    }

  /* Send data */

  ret = nrf91_usrsock_send_dack(usrsock, ack, req->head.xid,
                          ret, inaddrlen, outaddrlen);

  /* REVISIT: signal that more data can be available */

  if (ret >= 0 && buflen == recvlen)
    {
      usrsock->sock[req->usockid].recvpending = true;
      ret = nrf91_usrsock_send_event(usrsock, req->usockid,
                               USRSOCK_EVENT_RECVFROM_AVAIL);
    }
  else
    {
      usrsock->sock[req->usockid].recvpending = false;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf91_usrsock_setsockopt_handler
 ****************************************************************************/

static int nrf91_usrsock_setsockopt_handler(struct nrf91_usrsock_s *usrsock,
                                            const void *data, size_t len)
{
  const struct usrsock_request_setsockopt_s *req = data;
  int                                        ret = 0;

  ret = nrf91_usrsock_setsockopt(req->usockid, req->level, req->option,
                                 req + 1, req->valuelen);
  return nrf91_usrsock_send_ack(usrsock, req->head.xid, ret);
}

/****************************************************************************
 * Name: nrf91_usrsock_getsockopt_handler
 ****************************************************************************/

static int nrf91_usrsock_getsockopt_handler(struct nrf91_usrsock_s *usrsock,
                                            const void *data, size_t len)
{
  const struct usrsock_request_getsockopt_s *req    = data;
  struct usrsock_message_datareq_ack_s      *ack;
  socklen_t                                  optlen = req->max_valuelen;
  int                                        ret;

  ack = (struct usrsock_message_datareq_ack_s *)usrsock->out;
  ret = nrf91_usrsock_getsockopt(req->usockid, req->level, req->option,
                                 ack + 1, &optlen);

  return nrf91_usrsock_send_dack(usrsock, ack, req->head.xid,
                           ret, optlen, optlen);
}

/****************************************************************************
 * Name: nrf91_usrsock_getsockname_handler
 ****************************************************************************/

static int nrf91_usrsock_getsockname_handler(struct nrf91_usrsock_s *usrsock,
                                             const void *data, size_t len)
{
  return -1;
}

/****************************************************************************
 * Name: nrf91_usrsock_getpeername_handler
 ****************************************************************************/

static int nrf91_usrsock_getpeername_handler(struct nrf91_usrsock_s *usrsock,
                                             const void *data, size_t len)
{
  return -1;
}

/****************************************************************************
 * Name: nrf91_usrsock_bind_handler
 ****************************************************************************/

static int nrf91_usrsock_bind_handler(struct nrf91_usrsock_s *usrsock,
                                      const void *data, size_t len)
{
  const struct usrsock_request_bind_s *req = data;
  struct nrf_sockaddr_in6              address;
  int                                  ret = 0;

  nx2nrf_sockaddr((struct sockaddr *)(req + 1),
                  (struct nrf_sockaddr *)&address);
  ret = nrf_bind(req->usockid, (const struct nrf_sockaddr *)&address,
                 req->addrlen);

  return nrf91_usrsock_send_ack(usrsock, req->head.xid, ret);
}

/****************************************************************************
 * Name: nrf91_usrsock_listen_handler
 ****************************************************************************/

static int nrf91_usrsock_listen_handler(struct nrf91_usrsock_s *usrsock,
                                        const void *data, size_t len)
{
  const struct usrsock_request_listen_s *req = data;
  int                                    ret = 0;

  ret = nrf_listen(req->usockid, req->backlog);
  if (ret < 0)
    {
      ret = -errno;
    }

  return nrf91_usrsock_send_ack(usrsock, req->head.xid, ret);
}

/****************************************************************************
 * Name: nrf91_usrsock_accept_handler
 ****************************************************************************/

static int nrf91_usrsock_accept_handler(struct nrf91_usrsock_s *usrsock,
                                        const void *data, size_t len)
{
  const struct usrsock_request_accept_s *req        = data;
  struct usrsock_message_datareq_ack_s  *ack;
  struct nrf_sockaddr_in6                address;
  nrf_socklen_t                          outaddrlen;
  socklen_t                              inaddrlen  = req->max_addrlen;
  int                                    sockfd     = 0;
  int                                    ret        = 0;

  if (inaddrlen == sizeof(struct sockaddr_in))
    {
      outaddrlen = sizeof(struct nrf_sockaddr_in);
    }
  else
    {
      outaddrlen = sizeof(struct nrf_sockaddr_in6);
    }

  ack = (struct usrsock_message_datareq_ack_s *)usrsock->out;
  sockfd = nrf_accept(req->usockid,
                      outaddrlen ? (struct nrf_sockaddr *)&address : NULL,
                      (nrf_socklen_t)outaddrlen ? &outaddrlen : NULL);
  if (ret < 0)
    {
      nerr("nrf_accpet failed %d", errno);
      ret = -errno;
    }

  nrf2nx_sockaddr((struct nrf_sockaddr *)&address,
                  (struct sockaddr *)(ack + 1));

  if (sockfd >= 0)
    {
      /* Append index as usockid to the payload */

      if (outaddrlen <= inaddrlen)
        {
          *(int16_t *)((void *)(ack + 1) + outaddrlen) = sockfd;
        }
      else
        {
          *(int16_t *)((void *)(ack + 1) + inaddrlen) = sockfd;
        }

      ret = sizeof(int16_t); /* Return usockid size */
    }
  else
    {
      ret = sockfd;
    }

  ret = nrf91_usrsock_send_dack(usrsock, ack, req->head.xid, ret,
                          inaddrlen, outaddrlen);
  if (ret >= 0 && sockfd >= 0)
    {
      ret = nrf91_usrsock_send_event(usrsock, sockfd,
                                     USRSOCK_EVENT_SENDTO_READY);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf91_usrsock_ioctl_handler
 ****************************************************************************/

static int nrf91_usrsock_ioctl_handler(struct nrf91_usrsock_s *usrsock,
                                       const void *data, size_t len)
{
  const struct usrsock_request_ioctl_s *req = data;
  struct usrsock_message_datareq_ack_s *ack = NULL;
  size_t                                copylen;
  int                                   ret = 0;

  if (len < sizeof(*req))
    {
      return -EINVAL;
    }

  ack = (struct usrsock_message_datareq_ack_s *)usrsock->out;
  copylen = req->arglen;
  if (copylen > len - sizeof(*req) ||
      copylen > sizeof(usrsock->out) - sizeof(*ack))
    {
      return -EINVAL;
    }

  memcpy(ack + 1, req + 1, copylen);
  ret = nrf91_usrsock_ioctl(req->usockid,
                            req->cmd,
                            (unsigned long)(ack + 1));

  return nrf91_usrsock_send_dack(usrsock, ack, req->head.xid, ret,
                           copylen, copylen);
}

/****************************************************************************
 * Name: nrf91_usrsock_shutdown_handler
 ****************************************************************************/

static int nrf91_usrsock_shutdown_handler(struct nrf91_usrsock_s *usrsock,
                                          const void *data, size_t len)
{
  const struct usrsock_request_shutdown_s *req = data;
  int                                      ret = 0;

  ret = nrf_usrsock_shutdown(req->usockid, req->how);
  return nrf91_usrsock_send_ack(usrsock, req->head.xid, ret);
}

#ifndef CONFIG_NRF91_MODEM_AT
/****************************************************************************
 * Name: nrf91_modem_at_notify_handler
 ****************************************************************************/

static void nrf91_modem_at_notify_handler(const char *notif)
{
  /* TODO */

  printf("%s\n", notif);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_register
 ****************************************************************************/

void usrsock_register(void)
{
#ifndef CONFIG_NRF91_MODEM_AT
  /* Initialize AT modem */

  nrf_modem_at_notif_handler_set(nrf91_modem_at_notify_handler);
  nrf_modem_at_cmd_custom_set(NULL, 0);
#endif
}

/****************************************************************************
 * Name: usrsock_request
 ****************************************************************************/

int usrsock_request(struct iovec *iov, unsigned int iovcnt)
{
  struct usrsock_request_common_s *common = NULL;
  int                              ret    = 0;

  /* Copy request to buffer */

  ret = usrsock_iovec_get(g_usrsock.in, sizeof(g_usrsock.in),
                          iov, iovcnt, 0, NULL);
  if (ret <= 0)
    {
      return ret;
    }

  common = (struct usrsock_request_common_s *)g_usrsock.in;

  if (common->reqid >= 0 &&
      common->reqid < USRSOCK_REQUEST__MAX)
    {
      ret = g_usrsock_handler[common->reqid](&g_usrsock,
                                             g_usrsock.in, ret);
      if (ret < 0)
        {
          nerr("Usrsock request %" PRId32 " failed: %d\n",
               common->reqid, ret);
        }
    }
  else
    {
      nerr("Invalid request id: %" PRId32 "\n", common->reqid);
    }

  return ret;
}

/****************************************************************************
 * include/nuttx/wireless/gs2200m.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_GS2200M_H
#define __INCLUDE_NUTTX_WIRELESS_GS2200M_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/ioctl.h>

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

#define GS2200M_IOC_CONNECT  _WLCIOC(GS2200M_FIRST + 0)
#define GS2200M_IOC_SEND     _WLCIOC(GS2200M_FIRST + 1)
#define GS2200M_IOC_RECV     _WLCIOC(GS2200M_FIRST + 2)
#define GS2200M_IOC_CLOSE    _WLCIOC(GS2200M_FIRST + 3)
#define GS2200M_IOC_BIND     _WLCIOC(GS2200M_FIRST + 4)
#define GS2200M_IOC_ACCEPT   _WLCIOC(GS2200M_FIRST + 5)
#define GS2200M_IOC_ASSOC    _WLCIOC(GS2200M_FIRST + 6)
#define GS2200M_IOC_IFREQ    _WLCIOC(GS2200M_FIRST + 7)
#define GS2200M_IOC_NAME     _WLCIOC(GS2200M_FIRST + 8)

#define DISASSOCIATION_CID  ('x')

/* NOTE: do not forget to update include/nuttx/wireless/ioctl.h */

struct gs2200m_connect_msg
{
  char     cid;
  uint8_t  type;
  uint16_t lport;
  char     addr[17];
  char     port[6];
};

struct gs2200m_bind_msg
{
  char     cid;
  uint8_t  type;
  bool     is_tcp;
  char     port[6];
};

struct gs2200m_accept_msg
{
  struct sockaddr_in addr;
  char     cid;
  uint8_t  type;
};

struct gs2200m_send_msg
{
  struct sockaddr_in addr; /* used for udp */
  bool        is_tcp;
  char        cid;
  uint8_t     type;
  FAR uint8_t *buf;
  uint16_t len;
};

struct gs2200m_recv_msg
{
  struct sockaddr_in addr; /* used for udp */
  bool        is_tcp;
  char        cid;
  uint8_t     type;
  FAR uint8_t *buf;
  uint16_t    len;    /* actual buffer length */
  uint16_t    reqlen; /* requested size */
  int32_t     flags;  /* MSG_* flags */
};

struct gs2200m_close_msg
{
  char     cid;
  uint8_t  type;
};

struct gs2200m_assoc_msg
{
  FAR char *ssid;
  FAR char *key;
  uint8_t   mode;
  uint8_t   ch;
};

struct gs2200m_ifreq_msg
{
  uint32_t cmd;
  struct ifreq ifr;
};

struct gs2200m_name_msg
{
  struct sockaddr_in addr;
  bool        local;
  char        cid;
};

struct gs2200m_lower_s
{
  int  (*attach)(xcpt_t handler, FAR void *arg);
  void (*enable)(void);
  void (*disable)(void);
  uint32_t (*dready)(int *);
  void (*reset)(bool);
};

FAR void *gs2200m_register(FAR const char *devpath,
                           FAR struct spi_dev_s *dev,
                           FAR const struct gs2200m_lower_s *lower);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_WIRELESS_GS2200M_H */

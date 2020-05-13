/****************************************************************************
 * include/nuttx/wireless/gs2200m.h
 *
 *   Copyright 2019 Sony Home Entertainment & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

/* NOTE: do not forget to update include/nuttx/wireless/ioctl.h */

struct gs2200m_connect_msg
{
  char     cid;
  uint8_t  type;
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

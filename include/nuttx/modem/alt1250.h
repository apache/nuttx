/****************************************************************************
 * include/nuttx/modem/alt1250.h
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

#ifndef __INCLUDE_NUTTX_MODEM_ALT1250_H
#define __INCLUDE_NUTTX_MODEM_ALT1250_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/queue.h>
#include <nuttx/mutex.h>
#include <semaphore.h>
#include <debug.h>
#include <nuttx/irq.h>
#include <pthread.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing the modem driver */

#ifdef CONFIG_MODEM_ALT1250_DEBUG
#  define m_err     _err
#  define m_warn    _warn
#  define m_info    _info
#else
#  define m_err(...)
#  define m_warn(...)
#  define m_info(...)
#endif

#define ALT1250_IOC_POWER           _MODEMIOC(1)
#define ALT1250_IOC_SEND            _MODEMIOC(2)
#define ALT1250_IOC_SETEVTBUFF      _MODEMIOC(3)
#define ALT1250_IOC_EXCHGCONTAINER  _MODEMIOC(4)

#define ALT1250_EVTBIT_RESET (1ULL << 63)
#define ALT1250_EVTBIT_REPLY (1ULL << 62)

/* Number of sockets */

#define ALTCOM_NSOCKET             10

/* Macros for fcntl */

#define ALTCOM_GETFL               3
#define ALTCOM_SETFL               4

#define ALTCOM_O_NONBLOCK          0x01

#define ALTCOM_FD_SETSIZE       ALTCOM_NSOCKET

#define ALTCOM_FDSETSAFESET(s, code) \
  do { \
    if (((s) < ALTCOM_FD_SETSIZE) && ((int)(s) >= 0)) \
      { \
        code; \
      } \
  } while(0)
#define ALTCOM_FDSETSAFEGET(s, code) \
  (((s) < ALTCOM_FD_SETSIZE) && ((int)(s) >= 0) ? (code) : 0)

#define ALTCOM_FD_SET(s, set)   ALTCOM_FDSETSAFESET(s, (set)->fd_bits[(s)/8] |=  (1 << ((s) & 7)))
#define ALTCOM_FD_CLR(s, set)   ALTCOM_FDSETSAFESET(s, (set)->fd_bits[(s)/8] &= ~(1 << ((s) & 7)))
#define ALTCOM_FD_ISSET(s, set) ALTCOM_FDSETSAFEGET(s, (set)->fd_bits[(s)/8] &   (1 << ((s) & 7)))
#define ALTCOM_FD_ZERO(set)     memset((void*)(set), 0, sizeof(*(set)))

#define ALTCOM_DNS_SERVERS (4)
#define ALTCOM_REPEVT_FLAG_SIMSTAT (1 << 0)
#define ALTCOM_REPEVT_FLAG_LTIME   (1 << 1)

#define ALTCOM_VERX (0)
#define ALTCOM_VER1 (1)
#define ALTCOM_VER4 (4)

#define APICMDID_UNKNOWN                (0x0000)
#define APICMDID_POWER_ON               (0x0001)
#define ALTCOM_CMDID_POWER_ON_V1        (0x0001)
#define ALTCOM_CMDID_POWER_ON_V4        (0x0311)
#define APICMDID_GET_VERSION            (0x000C)
#define APICMDID_GET_PHONENO            (0x000D)
#define APICMDID_GET_IMSI               (0x000E)
#define APICMDID_GET_IMEI               (0x000F)
#define APICMDID_GET_PINSET             (0x0010)
#define APICMDID_SET_PIN_LOCK           (0x0011)
#define APICMDID_SET_PIN_CODE           (0x0012)
#define APICMDID_ENTER_PIN              (0x0013)
#define APICMDID_GET_LTIME              (0x0014)
#define APICMDID_GET_OPERATOR           (0x0015)
#define APICMDID_SET_REP_EVT            (0x0019)
#define APICMDID_SET_REP_QUALITY        (0x001A)
#define APICMDID_SET_REP_CELLINFO       (0x001B)
#define APICMDID_REPORT_EVT             (0x001D)
#define APICMDID_REPORT_QUALITY         (0x001E)
#define APICMDID_REPORT_CELLINFO        (0x001F)
#define APICMDID_GET_EDRX               (0x0020)
#define APICMDID_SET_EDRX               (0x0021)
#define APICMDID_GET_PSM                (0x0022)
#define APICMDID_SET_PSM                (0x0023)
#define APICMDID_GET_CE                 (0x0024)
#define APICMDID_SET_CE                 (0x0025)
#define APICMDID_RADIO_ON               (0x0026)
#define APICMDID_RADIO_OFF              (0x0027)
#define APICMDID_ACTIVATE_PDN           (0x0028)
#define APICMDID_DEACTIVATE_PDN         (0x0029)
#define APICMDID_DATA_ALLOW             (0x002A)
#define APICMDID_GET_NETINFO            (0x002B)
#define APICMDID_GET_IMS_CAP            (0x002C)
#define APICMDID_SETREP_NETINFO         (0x002D)
#define APICMDID_REPORT_NETINFO         (0x002E)
#define APICMDID_REPORT_RESTART         (0x002F)
#define APICMDID_ERRINFO                (0x0030)
#define APICMDID_SET_REP_EVT_LTIME      (0x0031)
#define APICMDID_SET_REP_EVT_SIMSTATE   (0x0032)
#define APICMDID_POWER_OFF              (0x0033)
#define APICMDID_GET_SIMINFO            (0x0034)
#define APICMDID_GET_DYNAMICEDRX        (0x0035)
#define APICMDID_GET_DYNAMICPSM         (0x0036)
#define APICMDID_GET_QUALITY            (0x0037)
#define APICMDID_ACTIVATE_PDN_CANCEL    (0x0038)
#define APICMDID_FW_INJECTDELTAIMG      (0x1040)
#define APICMDID_FW_GETDELTAIMGLEN      (0x1041)
#define APICMDID_FW_EXECDELTAUPDATE     (0x1042)
#define APICMDID_FW_GETUPDATERESULT     (0x1043)
#define APICMDID_CLOGS                  (0x1023)
#define APICMDID_LOGLIST                (0x1024)
#define APICMDID_LOGOPEN                (0x1030)
#define APICMDID_LOGCLOSE               (0x1031)
#define APICMDID_LOGREAD                (0x1033)
#define APICMDID_LOGREMOVE              (0x1034)
#define APICMDID_LOGLSEEK               (0x1035)

/* dummy ID because not support V1 */

#define APICMDID_GET_CELLINFO           (0x0039)
#define APICMDID_GET_RAT                (0x00A0)
#define APICMDID_SET_RAT                (0x00A1)
#define APICMDID_SEND_ATCMD             (0x00B0)
#define APICMDID_URC_EVENT              (0x00B2)

#define APICMDID_GET_VERSION_V4         (0x000B)
#define APICMDID_GET_PHONENO_V4         (0x000C)
#define APICMDID_GET_IMSI_V4            (0x000D)
#define APICMDID_GET_IMEI_V4            (0x000E)
#define APICMDID_GET_PINSET_V4          (0x000F)
#define APICMDID_SET_PIN_LOCK_V4        (0x0010)
#define APICMDID_SET_PIN_CODE_V4        (0x0011)
#define APICMDID_ENTER_PIN_V4           (0x0012)
#define APICMDID_GET_LTIME_V4           (0x0013)
#define APICMDID_GET_OPERATOR_V4        (0x0014)
#define APICMDID_GET_SLPMODESET_V4      (0x0015)
#define APICMDID_SET_SLPMODESET_V4      (0x0016)
#define APICMDID_SET_REP_NETSTAT_V4     (0x0017)
#define APICMDID_SET_REP_EVT_V4         (0x0018)
#define APICMDID_SET_REP_QUALITY_V4     (0x0019)
#define APICMDID_SET_REP_CELLINFO_V4    (0x001A)
#define APICMDID_REPORT_NETSTAT_V4      (0x001B)
#define APICMDID_REPORT_EVT_V4          (0x001C)
#define APICMDID_REPORT_QUALITY_V4      (0x001D)
#define APICMDID_REPORT_CELLINFO_V4     (0x001E)
#define APICMDID_GET_EDRX_V4            (0x001F)
#define APICMDID_SET_EDRX_V4            (0x0020)
#define APICMDID_GET_PSM_V4             (0x0021)
#define APICMDID_SET_PSM_V4             (0x0022)
#define APICMDID_GET_CELLINFO_V4        (0x0024)
#define APICMDID_GET_QUALITY_V4         (0x0025)
#define APICMDID_GET_SIMINFO_V4         (0x0310)
#define APICMDID_POWER_ON_V4            (0x0311)
#define APICMDID_RADIO_ON_V4            (0x0312)
#define APICMDID_RADIO_OFF_V4           (0x0313)
#define APICMDID_ACTIVATE_PDN_V4        (0x0314)
#define APICMDID_ACTIVATE_PDN_CANCEL_V4 (0x0315)
#define APICMDID_DEACTIVATE_PDN_V4      (0x0316)
#define APICMDID_DATA_ALLOW_V4          (0x0317)
#define APICMDID_GET_NETINFO_V4         (0x0318)
#define APICMDID_GET_IMS_CAP_V4         (0x0319)
#define APICMDID_SETREP_NETINFO_V4      (0x031A)
#define APICMDID_REPORT_NETINFO_V4      (0x031B)
#define APICMDID_ERRINFO_V4             (0xFFFD)
#define APICMDID_GET_RAT_V4             (0x00A0)
#define APICMDID_SET_RAT_V4             (0x00A1)
#define APICMDID_SEND_ATCMD_V4          (0x0030)
#define APICMDID_URC_EVENT_V4           (0x0032)
#define APICMDID_FW_INJECTDELTAIMG_V4   (0x0270)
#define APICMDID_FW_GETDELTAIMGLEN_V4   (0x0271)
#define APICMDID_FW_EXECDELTAUPDATE_V4  (0x0272)
#define APICMDID_FW_GETUPDATERESULT_V4  (0x0273)
#define APICMDID_CLOGS_V4               (0x0300)
#define APICMDID_LOGLIST_V4             (0x0301)
#define APICMDID_LOGOPEN_V4             (0x0280)
#define APICMDID_LOGCLOSE_V4            (0x0281)
#define APICMDID_LOGREAD_V4             (0x0283)
#define APICMDID_LOGREMOVE_V4           (0x0284)
#define APICMDID_LOGLSEEK_V4            (0x0285)

#define APICMDID_SOCK_ACCEPT            (0x0080)
#define APICMDID_SOCK_BIND              (0x0081)
#define APICMDID_SOCK_CLOSE             (0x0082)
#define APICMDID_SOCK_CONNECT           (0x0083)
#define APICMDID_SOCK_FCNTL             (0x0084)
#define APICMDID_SOCK_GETADDRINFO       (0x0085)
#define APICMDID_SOCK_GETHOSTBYNAME     (0x0086)
#define APICMDID_SOCK_GETHOSTBYNAMER    (0x0087)
#define APICMDID_SOCK_GETSOCKNAME       (0x0088)
#define APICMDID_SOCK_GETSOCKOPT        (0x0089)
#define APICMDID_SOCK_LISTEN            (0x008A)
#define APICMDID_SOCK_RECV              (0x008B)
#define APICMDID_SOCK_RECVFROM          (0x008C)
#define APICMDID_SOCK_SELECT            (0x008D)
#define APICMDID_SOCK_SEND              (0x008E)
#define APICMDID_SOCK_SENDTO            (0x008F)
#define APICMDID_SOCK_SHUTDOWN          (0x0090)
#define APICMDID_SOCK_SOCKET            (0x0091)
#define APICMDID_SOCK_SETSOCKOPT        (0x0092)

#define APICMDID_TLS_CONFIG_VERIFY_CALLBACK    (0x0129)
#define APICMDID_TLS_CONFIG_VERIFY_CALLBACK_V4 (0x0161)

#define APICMDID_SMS_INIT               (0x0330)
#define APICMDID_SMS_FIN                (0x0331)
#define APICMDID_SMS_SEND               (0x0332)
#define APICMDID_SMS_REPORT_RECV        (0x0333)
#define APICMDID_SMS_DELETE             (0x0334)

#define ALTCOM_CMDID_ERRIND      (0xFFFF)

#define ALTCOM_CMDID_REPLY_BIT (0x8000)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct alt_power_s
{
  uint32_t cmdid;
};

typedef struct alt_container_s
{
  sq_entry_t node;
  int sock;
  unsigned long priv;
  uint32_t cmdid;
  uint16_t altcid;
  uint16_t alttid;
  int result;
  FAR void **inparam;
  size_t inparamlen;
  FAR void **outparam;
  size_t outparamlen;
} alt_container_t;

typedef enum alt_evtbuf_state_e
{
  ALTEVTBUF_ST_WRITABLE = 0,
  ALTEVTBUF_ST_NOTWRITABLE = 1,
} alt_evtbuf_state_t;

typedef struct alt_evtbuf_inst_s
{
  uint32_t cmdid;
  uint16_t altcid;
  FAR void **outparam;
  size_t outparamlen;
  mutex_t stat_lock;
  alt_evtbuf_state_t stat;
} alt_evtbuf_inst_t;

struct alt_evtbuffer_s
{
  unsigned int ninst;
  alt_evtbuf_inst_t *inst;
};

struct alt_readdata_s
{
  uint64_t evtbitmap;
  FAR struct alt_container_s *head;
};

struct alt1250_lower_s
{
  FAR struct spi_dev_s * (*poweron)(void);
  void (*poweroff)(void);
  void (*reset)(void);
  void (*irqattach)(xcpt_t handler);
  void (*irqenable)(bool enable);
  bool (*get_sready)(void);
  void (*set_mready)(bool on);
  void (*set_wakeup)(bool on);
};

struct altcom_fd_set_s
{
  unsigned char fd_bits[(ALTCOM_FD_SETSIZE + 7) / 8];
};

typedef struct altcom_fd_set_s altcom_fd_set;

struct alt_queue_s
{
  sq_queue_t queue;
  mutex_t lock;
};

struct alt1250_dev_s
{
  FAR struct spi_dev_s *spi;
  FAR const struct alt1250_lower_s *lower;
  mutex_t refslock;
  uint8_t crefs;
  struct alt_queue_s waitlist;
  struct alt_queue_s replylist;
  uint64_t evtbitmap;
  mutex_t evtmaplock;
  mutex_t pfdlock;
  FAR struct pollfd *pfd;
  pthread_t recvthread;
  FAR struct alt_evtbuffer_s *evtbuff;
  uint32_t discardcnt;
  mutex_t senddisablelock;
  bool senddisable;
  FAR alt_container_t *select_container;
  struct alt_evtbuf_inst_s select_inst;
};

typedef int32_t (*compose_handler_t)(FAR void **arg, size_t arglen,
  uint8_t altver, FAR uint8_t *pktbuf, const size_t pktsz,
  FAR uint16_t *altcid);
typedef int32_t (*parse_handler_t)(FAR struct alt1250_dev_s *dev,
  FAR uint8_t *pktbuf, size_t pktsz, uint8_t altver, FAR void **arg,
  size_t arglen, FAR uint64_t *bitmap);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: alt1250_register
 *
 * Description:
 *   Register the ALT1250 character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/alt1250".
 *   dev     - An instance of the SPI interface to use to communicate with
 *             ALT1250.
 *   lower   - An instance of the lower interface.
 *
 * Returned Value:
 *   Not NULL on success; NULL on failure.
 *
 ****************************************************************************/

FAR void *alt1250_register(FAR const char *devpath,
  FAR struct spi_dev_s *dev, FAR const struct alt1250_lower_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_MODEM_ALT1250_H */

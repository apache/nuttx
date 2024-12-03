/****************************************************************************
 * boards/arm/cxd56xx/drivers/sensors/cxd5610_gnss.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <poll.h>
#include <spawn.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/cxd5610_gnss.h>
#include <arch/chip/gnss.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b)      (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#  define MAX(a,b)      (((a) > (b)) ? (a) : (b))
#endif /* MAX */

/* Configurations */

#ifndef CONFIG_SENSORS_CXD5610_GNSS_NPOLLWAITERS
#  define CONFIG_SENSORS_CXD5610_GNSS_NPOLLWAITERS        4
#endif

#ifndef CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS
#  define CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS    4
#endif

/* Communication buffer size */

#ifndef CONFIG_SENSORS_CXD5610_GNSS_SNDBUF_SIZE
#  define CONFIG_SENSORS_CXD5610_GNSS_SNDBUF_SIZE 64
#endif

#ifndef CONFIG_SENSORS_CXD5610_GNSS_RCVBUF_SIZE
#  define CONFIG_SENSORS_CXD5610_GNSS_RCVBUF_SIZE 64
#endif

#ifndef CONFIG_SENSORS_CXD5610_GNSS_NOTIFYBUF_SIZE
#  define CONFIG_SENSORS_CXD5610_GNSS_NOTIFYBUF_SIZE 1536
#endif

/* Receive thread */

#ifndef CONFIG_SENSORS_CXD5610_GNSS_RX_THREAD_PRIORITY
#  define CONFIG_SENSORS_CXD5610_GNSS_RX_THREAD_PRIORITY  120
#endif

#ifndef CONFIG_SENSORS_CXD5610_GNSS_RX_THREAD_STACKSIZE
#  define CONFIG_SENSORS_CXD5610_GNSS_RX_THREAD_STACKSIZE 1024
#endif

/* Read type */

#define CXD56_READ_DATA_TYPE_GNSS         0
#define CXD56_READ_DATA_TYPE_DCREPORT     15

/* OPC definitions */

#define OPC_SYS_STATE_CHANGE_INSTRUCTION  (0x00)
#define OPC_FWVER_REQ                     (0x06)
#define OPC_BACKUP_MANUAL                 (0x10)
#define OPC_PPS_OUTPUT                    (0x15)
#define OPC_GNSS_START                    (0x30)
#define OPC_GNSS_STOP                     (0x31)
#define OPC_BINARY_OUTPUT_SET             (0x34)
#define OPC_RECEIVER_POS_SET              (0x35)
#define OPC_UTC_TIME_SET                  (0x36)
#define OPC_OP_MODE_SET                   (0x3d)
#define OPC_OP_MODE_GET                   (0x3e)
#define OPC_TIME_NOTIFY                   (0x80)
#define OPC_RECEIVER_POS_NOTIFY           (0x81)
#define OPC_RECEIVER_VEL_NOTIFY           (0x82)
#define OPC_SAT_INFO_NOTIFY               (0x83)
#define OPC_ACCURACY_IDX_NOTIFY           (0x89)
#define OPC_DISASTER_CRISIS_NOTIFY        (0x8b)

/* Command packet definitions */

#define PACKET_SYNC         (0x7f)
#define PACKET_MAXLEN       (512)
#define PACKET_MASK         (PACKET_MAXLEN - 1)
#define PACKET_NR(n)        (((n) + PACKET_MASK) / PACKET_MAXLEN)
#define PACKET_HEADERLEN    (5)
#define PACKET_CHECKSUMLEN  (1)
#define PACKET_LEN(n)       ((n) + PACKET_HEADERLEN + PACKET_CHECKSUMLEN)

#define IS_NOTIFY_INVALID(v)  (((v) & 0x80) == 0)
#define GET_PACKET_VERSION(v) ((v) & 0x7f)

/* System state definitions */

#define STATE_RESET     0x01
#define STATE_WAKEUP    0x02
#define STATE_DEEPSLEEP 0x03
#define STATE_SLEEP     0x04

/* Command wait timeout in seconds */

#define COMMAND_WAIT_TIMEOUT  5

/* Boot wait timeout in seconds */

#define BOOT_WAIT_TIMEOUT  2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Structure for POSIX signal */

struct gnss_sig_s
{
  uint8_t                         enable;
  pid_t                           pid;
  struct cxd56_gnss_signal_info_s info;
};

/* Structure for cxd5610 driver */

struct cxd5610_gnss_dev_s
{
  struct cxd5610_gnss_lowerhalf_s *lower;

  pid_t         pid;
  uint8_t       cref;
  mutex_t       dev_lock;
  mutex_t       buf_lock;
  sem_t         cmd_sync;
  sem_t         boot_sync;
#if CONFIG_SENSORS_CXD5610_GNSS_NPOLLWAITERS != 0
  struct pollfd *fds[CONFIG_SENSORS_CXD5610_GNSS_NPOLLWAITERS];
  bool          has_event;
#endif
#if CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS != 0
  struct gnss_sig_s sigs[CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS];
#endif
  bool          wait_reset;
  bool          wait_wakeup;
  bool          sleeping;
  uint8_t       *sndbuf;
  uint8_t       *rcvbuf;
  uint8_t       *notifybuf;
  struct cxd56_gnss_positiondata2_s *posdat2;
  struct cxd56_gnss_dcreport_data_s *dcrdat;
};

/* Command packets */

begin_packed_struct struct cmd_op_mode_s
{
  uint8_t  mode8;
  uint32_t cycle32;
  uint32_t sleep32;
} end_packed_struct;

begin_packed_struct struct cmd_ellipsoidal_position_s
{
  int32_t lat32;
  int32_t lon32;
  int32_t alt32;
} end_packed_struct;

begin_packed_struct struct cmd_datetime_s
{
  uint8_t  type;
  uint8_t  yearl;
  uint8_t  yearu_mon;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  min;
  uint8_t  sec;
  uint8_t  sec100;
} end_packed_struct;

begin_packed_struct struct cmd_notify_time_s
{
  uint8_t ver8;
  uint8_t type;
  uint8_t yearl;
  uint8_t yearu_mon;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t sec100;
} end_packed_struct;

begin_packed_struct struct cmd_notify_pos_s
{
  uint8_t  ver8;
  uint8_t  mode;
  int32_t  lat32;
  int32_t  lon32;
  int32_t  alt32;
  int16_t  geo16;
} end_packed_struct;

begin_packed_struct struct cmd_notify_vel_s
{
  uint8_t  ver8;
  uint8_t  mode;
  uint16_t course16;
  uint16_t mag_course16;
  int16_t  vel16;
  int16_t  up_vel16;
} end_packed_struct;

begin_packed_struct struct cmd_notify_sat_s
{
  uint8_t  ver8;
  uint8_t  mode;
  uint16_t numsv;
} end_packed_struct;

begin_packed_struct struct cmd_notify_satinfo_s
{
  uint8_t  signal;
  uint8_t  svid;
  uint8_t  cn;
  uint8_t  elevation;
  uint16_t azimuth;
} end_packed_struct;

begin_packed_struct struct cmd_notify_acc_s
{
  uint8_t  ver8;
  uint16_t h_uc16;
  uint16_t v_uc16;
  uint16_t h_speed_uc16;
  uint16_t v_speed_uc16;
  uint8_t  pdop8;
  uint8_t  hdop8;
  uint8_t  vdop8;
  uint16_t semimajor16;
  uint16_t semiminor16;
  uint8_t  orientation;
} end_packed_struct;

begin_packed_struct struct cmd_notify_dcreport_s
{
  uint8_t  ver8;
  uint8_t  nr;
  struct mt43_data_s
  {
    uint8_t svid;
    uint8_t data[32];
  }
  msg[3];
} end_packed_struct;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int cxd5610_gnss_open(struct file *filep);
static int cxd5610_gnss_close(struct file *filep);
static ssize_t cxd5610_gnss_read(struct file *filep,
                                 char *buffer,
                                 size_t buflen);
static ssize_t cxd5610_gnss_write(struct file *filep,
                                  const char *buffer,
                                  size_t buflen);
static int cxd5610_gnss_ioctl(struct file *filep,
                              int cmd,
                              unsigned long arg);
static int cxd5610_gnss_poll(struct file *filep, struct pollfd *fds,
                             bool setup);

/* Semaphore controls */

static int cxd5610_gnss_device_init(struct cxd5610_gnss_dev_s *priv);
static int cxd5610_gnss_device_lock(struct cxd5610_gnss_dev_s *priv);
static int cxd5610_gnss_device_unlock(struct cxd5610_gnss_dev_s *priv);
static int cxd5610_gnss_buffer_init(struct cxd5610_gnss_dev_s *priv);
static int cxd5610_gnss_buffer_lock(struct cxd5610_gnss_dev_s *priv);
static int cxd5610_gnss_buffer_unlock(struct cxd5610_gnss_dev_s *priv);
static int cxd5610_gnss_init_boot(struct cxd5610_gnss_dev_s *priv);
static int cxd5610_gnss_wait_boot(struct cxd5610_gnss_dev_s *priv, int sec);
static int cxd5610_gnss_post_boot(struct cxd5610_gnss_dev_s *priv);
static int cxd5610_gnss_init_command(struct cxd5610_gnss_dev_s *priv);
static int cxd5610_gnss_wait_command(struct cxd5610_gnss_dev_s *priv,
                                     int sec);
static int cxd5610_gnss_post_command(struct cxd5610_gnss_dev_s *priv);
static int cxd5610_gnss_init_interrupt(void);
static int cxd5610_gnss_wait_interrupt(void);
static int cxd5610_gnss_post_interrupt(void);

#if CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS != 0
static void cxd5610_gnss_signalhandler(struct cxd5610_gnss_dev_s *priv,
                                       uint8_t sigtype);
#endif

#if CONFIG_SENSORS_CXD5610_GNSS_NPOLLWAITERS != 0
static void cxd5610_gnss_pollnotify(struct cxd5610_gnss_dev_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_cxd5610fops =
{
  cxd5610_gnss_open,  /* open */
  cxd5610_gnss_close, /* close */
  cxd5610_gnss_read,  /* read */
  cxd5610_gnss_write, /* write */
  NULL,               /* seek */
  cxd5610_gnss_ioctl, /* ioctl */
  NULL,               /* mmap */
  NULL,               /* truncate */
  cxd5610_gnss_poll   /* poll */
};

/* Semaphore for interrupt */

static sem_t g_int_sync;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd5610_checksum
 ****************************************************************************/

static uint8_t cxd5610_checksum(const uint8_t *data, uint16_t datalen)
{
  int i;
  uint8_t checksum = 0x00;

  for (i = 0; i < datalen; i++)
    {
      checksum += data[i];
    }

  return checksum;
}

/****************************************************************************
 * Name: cxd5610_store16
 ****************************************************************************/

static inline void cxd5610_store16(void *dst, void *src)
{
#if defined(CONFIG_SENSORS_CXD5610_GNSS_UNALIGNED_ACCESS)
  *(uint16_t *)dst = *(uint16_t *)src;
#elif !defined(CONFIG_ENDIAN_BIG)
  ((uint8_t *)dst)[0] = ((uint8_t *)src)[0];
  ((uint8_t *)dst)[1] = ((uint8_t *)src)[1];
#else
  ((uint8_t *)dst)[0] = ((uint8_t *)src)[1];
  ((uint8_t *)dst)[1] = ((uint8_t *)src)[0];
#endif
}

/****************************************************************************
 * Name: cxd5610_store32
 ****************************************************************************/

static void cxd5610_store32(void *dst, void *src)
{
#if defined(CONFIG_SENSORS_CXD5610_GNSS_UNALIGNED_ACCESS)
  *(uint32_t *)dst = *(uint32_t *)src;
#elif !defined(CONFIG_ENDIAN_BIG)
  ((uint8_t *)dst)[0] = ((uint8_t *)src)[0];
  ((uint8_t *)dst)[1] = ((uint8_t *)src)[1];
  ((uint8_t *)dst)[2] = ((uint8_t *)src)[2];
  ((uint8_t *)dst)[3] = ((uint8_t *)src)[3];
#else
  ((uint8_t *)dst)[0] = ((uint8_t *)src)[3];
  ((uint8_t *)dst)[1] = ((uint8_t *)src)[2];
  ((uint8_t *)dst)[2] = ((uint8_t *)src)[1];
  ((uint8_t *)dst)[3] = ((uint8_t *)src)[0];
#endif
}

/****************************************************************************
 * Name: cxd5610_gnss_recv
 ****************************************************************************/

static int cxd5610_gnss_recv(struct cxd5610_gnss_dev_s *priv,
                             uint8_t *buffer, int buflen)
{
  int ret = OK;

  if (priv->lower && priv->lower->ops->recv)
    {
      ret = priv->lower->ops->recv(priv->lower, buffer, buflen);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_notify_time
 ****************************************************************************/

static int cxd5610_gnss_notify_time(struct cxd5610_gnss_dev_s *priv, int len)
{
  struct cmd_notify_time_s *param =
                          (struct cmd_notify_time_s *)priv->notifybuf;
  struct cxd56_gnss_receiver2_s *receiver = &priv->posdat2->receiver;
  struct timespec ts;

  /* If the packet is invalid, do not update received data */

  if (IS_NOTIFY_INVALID(param->ver8))
    {
      return OK;
    }

  /* Get exclusive control for buffer access */

  cxd5610_gnss_buffer_lock(priv);

  /* Record the current timestamp in usec */

  clock_systime_timespec(&ts);
  priv->posdat2->timestamp  = 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;

  /* Receive UTC time information */

  receiver->date.year   = ((param->yearu_mon >> 4) << 8) | param->yearl;
  receiver->date.month  = param->yearu_mon & 0xf;
  receiver->date.day    = param->day;
  receiver->time.hour   = param->hour;
  receiver->time.minute = param->min;
  receiver->time.sec    = param->sec;
  receiver->time.usec   = param->sec100 * 10000;

  sninfo("%04d/%02d/%02d %02d:%02d:%02d.%ld\n",
         receiver->date.year,
         receiver->date.month,
         receiver->date.day,
         receiver->time.hour,
         receiver->time.minute,
         receiver->time.sec,
         receiver->time.usec);

  /* Release exclusive control for buffer access */

  cxd5610_gnss_buffer_unlock(priv);

  return OK;
}

/****************************************************************************
 * Name: cxd5610_gnss_notify_pos
 ****************************************************************************/

static int cxd5610_gnss_notify_pos(struct cxd5610_gnss_dev_s *priv, int len)
{
  struct cmd_notify_pos_s *param =
                             (struct cmd_notify_pos_s *)priv->notifybuf;
  struct cxd56_gnss_receiver2_s *receiver = &priv->posdat2->receiver;
  int32_t lat32;
  int32_t lon32;
  int32_t alt32;
  int16_t geo16 = 0;

  /* If the packet is invalid, do not update received data */

  if (IS_NOTIFY_INVALID(param->ver8))
    {
      return OK;
    }

  /* Get exclusive control for buffer access */

  cxd5610_gnss_buffer_lock(priv);

  /* Receive position information */

  cxd5610_store32(&lat32, &param->lat32);
  cxd5610_store32(&lon32, &param->lon32);
  cxd5610_store32(&alt32, &param->alt32);
  if (GET_PACKET_VERSION(param->ver8) > 0)
    {
      cxd5610_store16(&geo16, &param->geo16);
    }

  receiver->latitude  = (double)lat32 / 10000000.0;
  receiver->longitude = (double)lon32 / 10000000.0;
  receiver->altitude  = (double)alt32 / 100.0;
  receiver->geoid = (double)geo16 / 100.0;
  receiver->svtype = (param->mode >> 4);
  receiver->fix_indicator = (param->mode & 0xf);

  if (receiver->fix_indicator > 0)
    {
      receiver->pos_dataexist = 1;
    }

  sninfo("lat=%.7lf[deg] lon=%.7lf[deg] alt=%.2lf[m] "
         "geo=%.2lf[m] svtype=%d fix=%d\n",
         receiver->latitude,
         receiver->longitude,
         receiver->altitude,
         receiver->geoid,
         receiver->svtype,
         receiver->fix_indicator);

  /* Release exclusive control for buffer access */

  cxd5610_gnss_buffer_unlock(priv);

  return OK;
}

/****************************************************************************
 * Name: cxd5610_gnss_notify_vel
 ****************************************************************************/

static int cxd5610_gnss_notify_vel(struct cxd5610_gnss_dev_s *priv, int len)
{
  struct cmd_notify_vel_s *param =
                          (struct cmd_notify_vel_s *)priv->notifybuf;
  struct cxd56_gnss_receiver2_s *receiver = &priv->posdat2->receiver;
  uint16_t course16;
  uint16_t mag_course16;
  int16_t  vel16;
  int16_t  up_vel16;

  /* If the packet is invalid, do not update received data */

  if (IS_NOTIFY_INVALID(param->ver8))
    {
      return OK;
    }

  /* Get exclusive control for buffer access */

  cxd5610_gnss_buffer_lock(priv);

  /* Receive velocity information */

  cxd5610_store16(&course16, &param->course16);
  cxd5610_store16(&mag_course16, &param->mag_course16);
  cxd5610_store16(&vel16, &param->vel16);
  cxd5610_store16(&up_vel16, &param->up_vel16);
  receiver->velocity  = (float)vel16 / 10.0f;
  receiver->direction = (float)course16 / 10.0f;
  receiver->mag_course = (float)mag_course16 / 10.0f;
  receiver->up_velocity = (float)up_vel16 / 10.0f;
  receiver->vel_fixmode = (param->mode & 0x3);

  sninfo("vel=%.1lf[km/s] course=%.1lf[deg] upvel=%.1lf[km/s] mode=%d\n",
         receiver->velocity,
         receiver->direction,
         receiver->up_velocity,
         receiver->vel_fixmode);

  /* Release exclusive control for buffer access */

  cxd5610_gnss_buffer_unlock(priv);

  return OK;
}

/****************************************************************************
 * Name: cxd5610_gnss_notify_sat
 ****************************************************************************/

static int cxd5610_gnss_notify_sat(struct cxd5610_gnss_dev_s *priv, int len)
{
  struct cmd_notify_sat_s *param =
                          (struct cmd_notify_sat_s *)priv->notifybuf;
  struct cmd_notify_satinfo_s *info =
                          (struct cmd_notify_satinfo_s *)&priv->notifybuf[4];
  struct cxd56_gnss_receiver2_s *receiver = &priv->posdat2->receiver;
  struct cxd56_gnss_sv2_s *sv = priv->posdat2->sv;
  uint16_t numsv;
  uint16_t azimuth;
  int i;
  int j;
  int n;
  uint8_t stat;
  uint8_t type;
  uint8_t svid;
  bool isdup;

  /* If the packet is invalid, do not update received data */

  if (IS_NOTIFY_INVALID(param->ver8))
    {
      return OK;
    }

  /* Get exclusive control for buffer access */

  cxd5610_gnss_buffer_lock(priv);

  /* Receive satellite information */

  cxd5610_store16(&numsv, &param->numsv);
  numsv = MIN(numsv, CXD56_GNSS_MAX_SV2_NUM);
  receiver->pos_fixmode = param->mode & 0x3;

  for (i = 0, n = 0; i < numsv; i++)
    {
      isdup = false;
      stat = info[i].signal;
      type = info[i].signal & 0xf;
      svid = info[i].svid;

      /* Workaround an issue that there are duplicated satellites */

      for (j = 0; j < n; j++)
        {
          if ((sv[j].type == type) && (sv[j].svid == svid))
            {
              sninfo("Duplicated signal=%d svid=%d\n", type, svid);
              isdup = true;
              break;
            }
        }

      if (isdup)
        {
          continue;
        }

      sv[n].type = type;
      sv[n].svid = svid;
      sv[n].stat = CXD56_GNSS_SV_STAT_VISIBLE;
      sv[n].stat |= (stat & 0x20) ? CXD56_GNSS_SV_STAT_TRACKING : 0;
      sv[n].stat |= (stat & 0x80) ? CXD56_GNSS_SV_STAT_POSITIONING : 0;
      sv[n].stat |= (stat & 0x40) ? CXD56_GNSS_SV_STAT_CALC_VELOCITY : 0;
      cxd5610_store16(&azimuth, &info[i].azimuth);
      sv[n].azimuth = (azimuth < 360) ? azimuth : 0;
      sv[n].elevation = (info[i].elevation <= 90) ? info[i].elevation : 0;
      sv[n].siglevel = info[i].cn;
      n++;
    }

  priv->posdat2->svcount = receiver->numsv = n;

  sninfo("pos_fixmode=%d numsv=%d\n",
         receiver->pos_fixmode, receiver->numsv);

  for (i = 0; i < receiver->numsv; i++)
    {
      sninfo("%c%c%c signal=%2d svid=%3d cn=%2d elv=%2d azi=%3d\n",
             (sv[i].stat & 0x2) ? 'P' : '-',
             (sv[i].stat & 0x4) ? 'V' : '-',
             (sv[i].stat & 0x1) ? 'T' : '-',
             sv[i].type,
             sv[i].svid, sv[i].siglevel, sv[i].elevation, sv[i].azimuth);
    }

  /* Release exclusive control for buffer access */

  cxd5610_gnss_buffer_unlock(priv);

  return OK;
}

/****************************************************************************
 * Name: cxd5610_gnss_notify_acc
 ****************************************************************************/

static int cxd5610_gnss_notify_acc(struct cxd5610_gnss_dev_s *priv, int len)
{
  struct cmd_notify_acc_s *param =
                          (struct cmd_notify_acc_s *)priv->notifybuf;
  struct cxd56_gnss_receiver2_s *receiver = &priv->posdat2->receiver;
  uint16_t h_uc16;
  uint16_t v_uc16;
  uint16_t h_speed_uc16;
  uint16_t v_speed_uc16;
  uint16_t semimajor16;
  uint16_t semiminor16;

  /* If the packet is invalid, do not update received data */

  if (IS_NOTIFY_INVALID(param->ver8))
    {
      return OK;
    }

  /* Get exclusive control for buffer access */

  cxd5610_gnss_buffer_lock(priv);

  /* Receive accuracy information */

  receiver->pdop = (float)param->pdop8 / 10.0f;
  receiver->hdop = (float)param->hdop8 / 10.0f;
  receiver->vdop = (float)param->vdop8 / 10.0f;
  cxd5610_store16(&h_uc16, &param->h_uc16);
  cxd5610_store16(&v_uc16, &param->v_uc16);
  cxd5610_store16(&h_speed_uc16, &param->h_speed_uc16);
  cxd5610_store16(&v_speed_uc16, &param->v_speed_uc16);
  cxd5610_store16(&semimajor16, &param->semimajor16);
  cxd5610_store16(&semiminor16, &param->semiminor16);
  receiver->majdop = (float)semimajor16;
  receiver->mindop = (float)semiminor16;
  receiver->oridop = (float)param->orientation;
  receiver->hvar = (float)h_uc16;
  receiver->vvar = (float)v_uc16;
  receiver->hvar_speed = (float)h_speed_uc16 / 10.0f;
  receiver->vvar_speed = (float)v_speed_uc16 / 10.0f;

  sninfo("h=%.1f[m] v=%.1f[m] hs=%.1f[km/s] vs=%.1f[km/s] "
         "PDOP,HDOP,VDOP=%.1f,%.1f,%.1f "
         "maj,min,ori=%.1f[m],%.1f[m],%.1f[deg]\n",
         receiver->hvar, receiver->vvar,
         receiver->hvar_speed, receiver->vvar_speed,
         receiver->pdop, receiver->hdop, receiver->vdop,
         receiver->majdop, receiver->mindop, receiver->oridop);

  /* Release exclusive control for buffer access */

  cxd5610_gnss_buffer_unlock(priv);

  return OK;
}

/****************************************************************************
 * Name: cxd5610_gnss_notify_dcreport
 ****************************************************************************/

static int
cxd5610_gnss_notify_dcreport(struct cxd5610_gnss_dev_s *priv, int len)
{
  struct cmd_notify_dcreport_s *param =
                             (struct cmd_notify_dcreport_s *)priv->notifybuf;
  struct cxd56_gnss_dcreport_data_s *dcrdat = priv->dcrdat;
  int i;

  /* If the packet is invalid, do not update received data */

  if (IS_NOTIFY_INVALID(param->ver8))
    {
      return OK;
    }

  if (param->nr == 0)
    {
      return -ENOENT;
    }

  /* Only one message is received due to duplicate messages */

  param->nr = 1;

  /* Get exclusive control for buffer access */

  cxd5610_gnss_buffer_lock(priv);

  /* Receive disaster and crisis report information */

  for (i = 0; i < param->nr; i++)
    {
      sninfo("svid=%d\n", param->msg[i].svid);
      dcrdat->svid = param->msg[i].svid;
      memcpy(dcrdat->sf, param->msg[i].data, sizeof(dcrdat->sf));
    }

  /* Release exclusive control for buffer access */

  cxd5610_gnss_buffer_unlock(priv);

  return OK;
}

/****************************************************************************
 * Name: cxd5610_gnss_notify
 ****************************************************************************/

static int
cxd5610_gnss_notify(struct cxd5610_gnss_dev_s *priv, uint8_t opc, int len)
{
  int ret = OK;

  switch (opc)
    {
      case OPC_TIME_NOTIFY:
        ret = cxd5610_gnss_notify_time(priv, len);
        break;
      case OPC_RECEIVER_POS_NOTIFY:
        ret = cxd5610_gnss_notify_pos(priv, len);
        break;
      case OPC_RECEIVER_VEL_NOTIFY:
        ret = cxd5610_gnss_notify_vel(priv, len);
        break;
      case OPC_SAT_INFO_NOTIFY:
        ret = cxd5610_gnss_notify_sat(priv, len);
        break;
      case OPC_DISASTER_CRISIS_NOTIFY:
        ret = cxd5610_gnss_notify_dcreport(priv, len);
#if CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS != 0
        if (ret >= 0)
          {
            cxd5610_gnss_signalhandler(priv, CXD56_GNSS_SIG_DCREPORT);
          }
#endif
        break;
      case OPC_ACCURACY_IDX_NOTIFY:
        ret = cxd5610_gnss_notify_acc(priv, len);

        /* Notify after the last message received */

#if CONFIG_SENSORS_CXD5610_GNSS_NPOLLWAITERS != 0
        cxd5610_gnss_pollnotify(priv);
#endif
#if CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS != 0
        cxd5610_gnss_signalhandler(priv, CXD56_GNSS_SIG_GNSS);
#endif
        break;
      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_send
 ****************************************************************************/

static int cxd5610_gnss_send(struct cxd5610_gnss_dev_s *priv,
                             uint8_t *buffer, int buflen)
{
  int ret = OK;

  if (priv->lower && priv->lower->ops->send)
    {
      ret = priv->lower->ops->send(priv->lower, buffer, buflen);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_sendcmd
 ****************************************************************************/

static int cxd5610_gnss_sendcmd(struct cxd5610_gnss_dev_s *priv,
                                uint8_t opc, const void *opr, int oprlen)
{
  int ret;
  int buflen = 5;
  uint8_t *pkt = priv->sndbuf;
  int8_t errcode;

  /* Check parameter */

  if ((oprlen < 0) ||
      (CONFIG_SENSORS_CXD5610_GNSS_SNDBUF_SIZE < PACKET_LEN(oprlen)))
    {
      return -EINVAL;
    }

  /* Set command header */

  pkt[0] = PACKET_SYNC;
  pkt[1] = (uint8_t)(oprlen & 0xff);
  pkt[2] = (uint8_t)(oprlen >> 8);
  pkt[3] = opc;
  pkt[4] = cxd5610_checksum(pkt, 4);

  /* Set command payload */

  if (opr && (oprlen > 0))
    {
      memcpy(&pkt[5], opr, oprlen);
      pkt[5 + oprlen] = cxd5610_checksum(&pkt[5], oprlen);
      buflen += (oprlen + 1);
    }

  /* Send a command */

  ret = cxd5610_gnss_send(priv, priv->sndbuf, buflen);
  if (ret >= 0)
    {
      /* Wait for command response */

      ret = cxd5610_gnss_wait_command(priv, COMMAND_WAIT_TIMEOUT);
      if (ret >= 0)
        {
          errcode = (int8_t)priv->rcvbuf[0];
          ret = (int)errcode;
        }
    }
  else if ((opc == OPC_SYS_STATE_CHANGE_INSTRUCTION) &&
           (((uint8_t *)opr)[0] == STATE_WAKEUP))
    {
      /* Wait for wakeup command response */

      ret = cxd5610_gnss_wait_command(priv, COMMAND_WAIT_TIMEOUT);
      if (ret >= 0)
        {
          errcode = (int8_t)priv->rcvbuf[0];
          ret = (int)errcode;
        }
    }
  else
    {
      snerr("ERROR: send command opc=0x%02x ret=%d\n", opc, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_handler
 ****************************************************************************/

static void cxd5610_gnss_handler(void)
{
  /* Issue event from interrupt */

  cxd5610_gnss_post_interrupt();
}

/****************************************************************************
 * Name: cxd5610_gnss_thread_loop
 ****************************************************************************/

static int cxd5610_gnss_thread_loop(struct cxd5610_gnss_dev_s *priv)
{
  uint16_t oprlen;
  uint8_t header[5];
  uint8_t opc;
  uint8_t *pkt;
  int totallen;
  int num;
  int rcvlen;
  int maxlen;

  /* Wait for interrupt from device */

  cxd5610_gnss_wait_interrupt();

  /* Read header */

  memset(header, 0, sizeof(header));
  cxd5610_gnss_recv(priv, header, sizeof(header));
  if (header[0] != PACKET_SYNC)
    {
      snerr("ERROR: Failed to receive sync packet\n");
      return OK;
    }

  cxd5610_store16(&oprlen, &header[1]);
  opc = header[3];

  /* Verify header checksum */

  if (header[4] != cxd5610_checksum(header, 4))
    {
      snerr("ERROR: header checksum (opc=0x%02x oprlen=%d)\n",
            opc, oprlen);
      return OK;
    }

  sninfo("opc=0x%02x oprlen=%d\n", opc, oprlen);

  if (oprlen == 0)
    {
      /* If no payload, wait for next received data */

      return OK;
    }

  /* Check received buffer size */

  maxlen = (opc < OPC_TIME_NOTIFY) ?
              CONFIG_SENSORS_CXD5610_GNSS_RCVBUF_SIZE :
              CONFIG_SENSORS_CXD5610_GNSS_NOTIFYBUF_SIZE;
  if (maxlen <= oprlen)
    {
      snerr("ERROR: insufficient buffer size (opc=0x%02x oprlen=%d)\n",
            opc, oprlen);
      return ERROR;
    }

  /* Wait for interrupt from device */

  cxd5610_gnss_wait_interrupt();

  /* Read payload */

  pkt = (opc < OPC_TIME_NOTIFY) ? priv->rcvbuf : priv->notifybuf;

  totallen = oprlen + 1;
  num = PACKET_NR(totallen);
  rcvlen = MIN(PACKET_MAXLEN, totallen);
  cxd5610_gnss_recv(priv, pkt, rcvlen);
  while (--num)
    {
      /* Wait for interrupt from device */

      cxd5610_gnss_wait_interrupt();

      totallen -= rcvlen;
      rcvlen = MIN(PACKET_MAXLEN, totallen);
      pkt += PACKET_MAXLEN;
      cxd5610_gnss_recv(priv, pkt, rcvlen);
    }

  /* Verify payload checksum */

  pkt = (opc < OPC_TIME_NOTIFY) ? priv->rcvbuf : priv->notifybuf;
  if (pkt[oprlen] != cxd5610_checksum(pkt, oprlen))
    {
      snerr("ERROR: payload checksum (opc=0x%02x oprlen=%d)\n",
            opc, oprlen);
      return OK;
    }

  /* Synchronous command or asynchronous notify */

  if (opc < OPC_TIME_NOTIFY)
    {
      /* Release waiting for command response */

      if (opc == OPC_SYS_STATE_CHANGE_INSTRUCTION)
        {
          if (pkt[0] == STATE_RESET)
            {
              sninfo("Boot notification\n");
              cxd5610_gnss_post_boot(priv);
              if (priv->wait_reset)
                {
                  cxd5610_gnss_post_command(priv);
                  priv->wait_reset = false;
                }

              return OK;
            }
          else if (pkt[0] == STATE_WAKEUP)
            {
              sninfo("Wakeup notification\n");
              priv->sleeping = false;
              if (priv->wait_wakeup)
                {
                  cxd5610_gnss_post_command(priv);
                  priv->wait_wakeup = false;
                }

              return OK;
            }
        }

      cxd5610_gnss_post_command(priv);
    }
  else
    {
      cxd5610_gnss_notify(priv, opc, oprlen);
    }

  return OK;
}

/****************************************************************************
 * Name: cxd5610_gnss_start
 ****************************************************************************/

static int cxd5610_gnss_start(struct cxd5610_gnss_dev_s *priv,
                              unsigned long arg)
{
  int ret;
  uint8_t param = 3;

  ret = cxd5610_gnss_sendcmd(priv, OPC_GNSS_START, &param, sizeof(param));

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_stop
 ****************************************************************************/

static int cxd5610_gnss_stop(struct cxd5610_gnss_dev_s *priv,
                             unsigned long arg)
{
  int ret;

  ret = cxd5610_gnss_sendcmd(priv, OPC_GNSS_STOP, NULL, 0);

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_select_satellite_system
 ****************************************************************************/

static int
cxd5610_gnss_select_satellite_system(struct cxd5610_gnss_dev_s *priv,
                                     unsigned long arg)
{
  return OK;
}

/****************************************************************************
 * Name: cxd5610_gnss_get_satellite_system
 ****************************************************************************/

static int cxd5610_gnss_get_satellite_system(struct cxd5610_gnss_dev_s *priv,
                                             unsigned long arg)
{
  *(uint32_t *)arg = 0xffff;

  return OK;
}

/****************************************************************************
 * Name: cxd5610_gnss_set_receiver_position_ellipsoidal
 ****************************************************************************/

static int
cxd5610_gnss_set_receiver_position_ellipsoidal(
                                             struct cxd5610_gnss_dev_s *priv,
                                             unsigned long arg)
{
  int ret;
  struct cxd56_gnss_ellipsoidal_position_s *pos;
  struct cmd_ellipsoidal_position_s param;

  pos = (struct cxd56_gnss_ellipsoidal_position_s *)arg;

  param.lat32 = (int32_t)(pos->latitude * 1000000);
  param.lon32 = (int32_t)(pos->longitude * 1000000);
  param.alt32 = (int32_t)pos->altitude;

  ret = cxd5610_gnss_sendcmd(priv, OPC_RECEIVER_POS_SET,
                             &param, sizeof(param));

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_set_ope_mode
 ****************************************************************************/

static int cxd5610_gnss_set_ope_mode(struct cxd5610_gnss_dev_s *priv,
                                     unsigned long arg)
{
  int ret;
  struct cxd56_gnss_ope_mode_param_s *ope_mode;
  struct cmd_op_mode_s param;

  ope_mode = (struct cxd56_gnss_ope_mode_param_s *)arg;

  param.mode8 = (uint8_t)ope_mode->mode;
  param.sleep32 = 0;
  cxd5610_store32(&param.cycle32, &ope_mode->cycle);

  ret = cxd5610_gnss_sendcmd(priv, OPC_OP_MODE_SET, &param, sizeof(param));

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_get_ope_mode
 ****************************************************************************/

static int cxd5610_gnss_get_ope_mode(struct cxd5610_gnss_dev_s *priv,
                                     unsigned long arg)
{
  int ret;
  struct cxd56_gnss_ope_mode_param_s *ope_mode;

  ope_mode = (struct cxd56_gnss_ope_mode_param_s *)arg;

  ret = cxd5610_gnss_sendcmd(priv, OPC_OP_MODE_GET, NULL, 0);

  if (ret == OK)
    {
      uint8_t *res = priv->rcvbuf;
      ope_mode->mode = 1;
      cxd5610_store32(&ope_mode->cycle, &res[1]);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_set_time
 ****************************************************************************/

static int cxd5610_gnss_set_time(struct cxd5610_gnss_dev_s *priv,
                                 unsigned long arg)
{
  int ret;
  struct cxd56_gnss_datetime_s *date_time;
  struct cmd_datetime_s param;

  date_time = (struct cxd56_gnss_datetime_s *)arg;

  param.type = 0;
  param.yearl = (uint8_t)(date_time->date.year & 0xff);
  param.yearu_mon = (uint8_t)((date_time->date.year >> 8) << 4) |
                              (date_time->date.month & 0x0f);
  param.day = date_time->date.day;
  param.hour = date_time->time.hour;
  param.min = date_time->time.minute;
  param.sec = date_time->time.sec;
  param.sec100 = (uint8_t)(date_time->time.usec / 10000);

  ret = cxd5610_gnss_sendcmd(priv, OPC_UTC_TIME_SET, &param, sizeof(param));

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_save_backup_data
 ****************************************************************************/

static int cxd5610_gnss_save_backup_data(struct cxd5610_gnss_dev_s *priv,
                                         unsigned long arg)
{
  int ret;
  uint8_t param = 1;

  ret = cxd5610_gnss_sendcmd(priv, OPC_BACKUP_MANUAL, &param, sizeof(param));

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_set_1pps_output
 ****************************************************************************/

static int cxd5610_gnss_set_1pps_output(struct cxd5610_gnss_dev_s *priv,
                                        unsigned long arg)
{
  int ret;
  uint8_t param = (uint8_t)arg;

  ret = cxd5610_gnss_sendcmd(priv, OPC_PPS_OUTPUT, &param, sizeof(param));

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_get_version
 ****************************************************************************/

static int cxd5610_gnss_get_version(struct cxd5610_gnss_dev_s *priv,
                                    unsigned long arg)
{
  int ret;
  char *version;

  version = (char *)arg;

  memset(version, 0, CXD56_GNSS_VERSION_MAXLEN);

  ret = cxd5610_gnss_sendcmd(priv, OPC_FWVER_REQ, NULL, 0);

  if (ret == OK)
    {
      int i;
      char *ch = (char *)&priv->rcvbuf[1];

      /* Get version information */

      for (i = 0; i < CXD56_GNSS_VERSION_MAXLEN - 1; i++)
        {
          if ((*ch == '\r') || (*ch == '\n'))
            {
              version[i] = '\0';
              break;
            }

          version[i] = *ch++;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_sleep
 ****************************************************************************/

static int cxd5610_gnss_sleep(struct cxd5610_gnss_dev_s *priv,
                              unsigned long arg)
{
  int ret;
  uint8_t param = (arg == 0) ? STATE_SLEEP : STATE_DEEPSLEEP;

  /* If already in sleeping mode, do nothing. */

  if (priv->sleeping)
    {
      return OK;
    }

  ret = cxd5610_gnss_sendcmd(priv, OPC_SYS_STATE_CHANGE_INSTRUCTION,
                             &param, sizeof(param));

  /* If the command succeeds, enter sleeping mode. */

  if (ret == (int)param)
    {
      priv->sleeping = true;
      return OK;
    }
  else
    {
      return -EPERM;
    }
}

/****************************************************************************
 * Name: cxd5610_gnss_wakeup
 ****************************************************************************/

static int cxd5610_gnss_wakeup(struct cxd5610_gnss_dev_s *priv,
                               unsigned long arg)
{
  int ret;
  uint8_t param = STATE_WAKEUP;

  /* If it's not in sleeping mode, do nothing. */

  if (!priv->sleeping)
    {
      return OK;
    }

  priv->wait_wakeup = true;
  ret = cxd5610_gnss_sendcmd(priv, OPC_SYS_STATE_CHANGE_INSTRUCTION,
                             &param, sizeof(param));

  return (ret == (int)param) ? OK : -EPERM;
}

/****************************************************************************
 * Name: cxd5610_gnss_reset
 ****************************************************************************/

static int cxd5610_gnss_reset(struct cxd5610_gnss_dev_s *priv,
                              unsigned long arg)
{
  int ret;
  uint8_t param = STATE_RESET;

  priv->wait_reset = true;
  ret = cxd5610_gnss_sendcmd(priv, OPC_SYS_STATE_CHANGE_INSTRUCTION,
                             &param, sizeof(param));

  return (ret == (int)param) ? OK : -EPERM;
}

/****************************************************************************
 * Name: cxd5610_gnss_set_notify
 ****************************************************************************/

static int cxd5610_gnss_set_notify(struct cxd5610_gnss_dev_s *priv)
{
  int ret;
  uint8_t param[] = {
    OPC_TIME_NOTIFY,
    OPC_RECEIVER_POS_NOTIFY,
    OPC_RECEIVER_VEL_NOTIFY,
    OPC_SAT_INFO_NOTIFY,
    OPC_ACCURACY_IDX_NOTIFY,
    OPC_DISASTER_CRISIS_NOTIFY
  };

  ret = cxd5610_gnss_sendcmd(priv, OPC_BINARY_OUTPUT_SET,
                             &param, sizeof(param));

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_core_initialize
 ****************************************************************************/

static int cxd5610_gnss_core_initialize(struct cxd5610_gnss_dev_s *priv)
{
  int ret = OK;

  /* Enable interrupt from CXD5610 device */

  if (priv->lower && priv->lower->ops->enableint)
    {
      ret = priv->lower->ops->enableint(priv->lower, cxd5610_gnss_handler);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_core_finalize
 ****************************************************************************/

static int cxd5610_gnss_core_finalize(struct cxd5610_gnss_dev_s *priv)
{
  int ret = OK;

  /* Disable interrupt */

  if (priv->lower && priv->lower->ops->disableint)
    {
      ret = priv->lower->ops->disableint(priv->lower);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_thread
 ****************************************************************************/

static int cxd5610_gnss_thread(int argc, char** argv)
{
  int ret;
  struct cxd5610_gnss_dev_s *priv = (struct cxd5610_gnss_dev_s *)
        ((uintptr_t)strtoul(argv[1], NULL, 0));

  do
    {
      ret = cxd5610_gnss_thread_loop(priv);
    }
  while (ret == OK);

  return OK;
}

/****************************************************************************
 * Name: cxd5610_gnss_set_signal
 ****************************************************************************/

static int cxd5610_gnss_set_signal(struct cxd5610_gnss_dev_s *priv,
                                   unsigned long arg)
{
  int ret = 0;

#if CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS != 0
  struct cxd56_gnss_signal_setting_s *setting;
  struct gnss_sig_s                  *sig;
  struct gnss_sig_s                  *checksig;
  pid_t                              pid;
  int                                i;

  DEBUGASSERT(arg != 0);

  setting = (struct cxd56_gnss_signal_setting_s *)arg;
  if ((setting->gnsssig != CXD56_GNSS_SIG_GNSS) &&
      (setting->gnsssig != CXD56_GNSS_SIG_DCREPORT))
    {
      return -EPROTOTYPE;
    }

  sig = NULL;
  pid = getpid();
  for (i = 0; i < CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS; i++)
    {
      checksig = &priv->sigs[i];
      if (setting->enable)
        {
          if (sig == NULL && !checksig->enable)
            {
              sig = checksig;
            }
          else if (checksig->info.gnsssig == setting->gnsssig &&
                   checksig->pid == pid)
            {
              sig = checksig;
              break;
            }
        }
      else if (checksig->info.gnsssig == setting->gnsssig &&
               checksig->pid == pid)
        {
          checksig->enable = 0;
          goto errout;
        }
    }

  if (sig == NULL)
    {
      ret = -ENOENT;
      goto errout;
    }

  sig->enable       = 1;
  sig->pid          = pid;
  sig->info.fd      = setting->fd;
  sig->info.gnsssig = setting->gnsssig;
  sig->info.signo   = setting->signo;
  sig->info.data    = setting->data;

errout:
#endif /* CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS != 0 */

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_initialize
 ****************************************************************************/

static int cxd5610_gnss_initialize(struct cxd5610_gnss_dev_s *priv)
{
  int ret = OK;
  char *argv[2];
  char arg1[32];
  posix_spawnattr_t attr;

  /* Create thread for receiving from CXD5610 device */

  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;

  posix_spawnattr_init(&attr);
  attr.priority  = CONFIG_SENSORS_CXD5610_GNSS_RX_THREAD_PRIORITY;
  attr.stacksize = CONFIG_SENSORS_CXD5610_GNSS_RX_THREAD_STACKSIZE;

  priv->pid = task_spawn("cxd5610_gnss_thread",
                         cxd5610_gnss_thread,
                         NULL, &attr, argv, NULL);

  posix_spawnattr_destroy(&attr);

  /* Allocate various buffers */

  priv->sndbuf = kmm_malloc(CONFIG_SENSORS_CXD5610_GNSS_SNDBUF_SIZE);
  if (priv->sndbuf == NULL)
    {
      snerr("ERROR: Failed to allocate sndbuf\n");
      ret = -ENOMEM;
      goto errout1;
    }

  priv->rcvbuf = kmm_malloc(CONFIG_SENSORS_CXD5610_GNSS_RCVBUF_SIZE);
  if (priv->rcvbuf == NULL)
    {
      snerr("ERROR: Failed to allocate rcvbuf\n");
      ret = -ENOMEM;
      goto errout2;
    }

  priv->notifybuf = kmm_malloc(CONFIG_SENSORS_CXD5610_GNSS_NOTIFYBUF_SIZE);
  if (priv->notifybuf == NULL)
    {
      snerr("ERROR: Failed to allocate notifybuf\n");
      ret = -ENOMEM;
      goto errout3;
    }

  priv->posdat2 = kmm_zalloc(sizeof(struct cxd56_gnss_positiondata2_s));
  if (priv->posdat2 == NULL)
    {
      snerr("ERROR: Failed to allocate position data\n");
      ret = -ENOMEM;
      goto errout4;
    }

  priv->dcrdat = kmm_zalloc(sizeof(struct cxd56_gnss_dcreport_data_s));
  if (priv->dcrdat == NULL)
    {
      snerr("ERROR: Failed to allocate dcreport data\n");
      ret = -ENOMEM;
      goto errout5;
    }

  /* Initialize CXD5610 device */

  cxd5610_gnss_core_initialize(priv);

  return OK;

errout5:
  kmm_free(priv->posdat2);
errout4:
  kmm_free(priv->notifybuf);
errout3:
  kmm_free(priv->rcvbuf);
errout2:
  kmm_free(priv->sndbuf);
errout1:
  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_finalize
 ****************************************************************************/

static int cxd5610_gnss_finalize(struct cxd5610_gnss_dev_s *priv)
{
  int ret = 0;

  /* Finalize CXD5610 device */

  cxd5610_gnss_core_finalize(priv);
  nxsig_sleep(1);

  /* Terminate thread */

  ret = nxtask_delete(priv->pid);

  /* Free buffers */

  kmm_free(priv->sndbuf);
  kmm_free(priv->rcvbuf);
  kmm_free(priv->notifybuf);
  kmm_free(priv->posdat2);
  kmm_free(priv->dcrdat);

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_open
 ****************************************************************************/

static int cxd5610_gnss_open(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct cxd5610_gnss_dev_s *priv = inode->i_private;
  int ret = OK;

  cxd5610_gnss_device_lock(priv);

  priv->cref++;

  if (priv->cref == 1)
    {
      ret = cxd5610_gnss_initialize(priv);

      if (ret == OK)
        {
          cxd5610_gnss_wait_boot(priv, BOOT_WAIT_TIMEOUT);
          cxd5610_gnss_set_notify(priv);
        }
    }

  cxd5610_gnss_device_unlock(priv);

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_close
 ****************************************************************************/

static int cxd5610_gnss_close(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct cxd5610_gnss_dev_s *priv = inode->i_private;
  int ret = OK;

  cxd5610_gnss_device_lock(priv);

  priv->cref--;

  if (priv->cref == 0)
    {
      ret = cxd5610_gnss_finalize(priv);
    }

  cxd5610_gnss_device_unlock(priv);

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_select_readtype
 ****************************************************************************/

static int8_t cxd5610_gnss_select_readtype(off_t fpos, uint32_t *offset)
{
  int8_t type;

  if ((fpos >= CXD56_GNSS_READ_OFFSET_LAST_GNSS) &&
      (fpos < CXD56_GNSS_READ_OFFSET_AGPS))
    {
      type = CXD56_READ_DATA_TYPE_GNSS;
      *offset = 0;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_DCREPORT)
    {
      type = CXD56_READ_DATA_TYPE_DCREPORT;
      *offset = 0;
    }
  else
    {
      type = -1;
    }

  return type;
}

#ifdef CONFIG_SENSORS_CXD5610_GNSS_READ_COMPAT
/****************************************************************************
 * Name: cxd5610_gnss_signal2type
 ****************************************************************************/

static uint8_t cxd5610_gnss_signal2type(uint8_t signal)
{
  return (signal <= CXD56_GNSS_SIGNAL_GPS_L5)   ? CXD56_GNSS_SAT_GPS :
         (signal <= CXD56_GNSS_SIGNAL_GLN_L1OF) ? CXD56_GNSS_SAT_GLONASS :
         (signal == CXD56_GNSS_SIGNAL_QZS_L1S)  ? CXD56_GNSS_SAT_QZ_L1S :
         (signal <= CXD56_GNSS_SIGNAL_QZS_L5)   ? CXD56_GNSS_SAT_QZ_L1CA :
         (signal <= CXD56_GNSS_SIGNAL_BDS_B2A)  ? CXD56_GNSS_SAT_BEIDOU :
         (signal <= CXD56_GNSS_SIGNAL_GAL_E5A)  ? CXD56_GNSS_SAT_GALILEO : 0;
}

/****************************************************************************
 * Name: cxd5610_gnss_read_compat
 ****************************************************************************/

static ssize_t cxd5610_gnss_read_compat(struct cxd5610_gnss_dev_s *priv,
                                        char *buffer, size_t len)
{
  uint8_t numsv_tracking = 0;
  uint8_t numsv_calcpos = 0;
  uint8_t numsv_calcvel = 0;
  uint16_t svtype = 0;
  uint16_t pos_svtype = 0;
  uint16_t vel_svtype = 0;
  uint32_t svnum;
  int i;
  struct cxd56_gnss_positiondata_s *posdat =
                          (struct cxd56_gnss_positiondata_s *)buffer;

  /* Copy the position data into the old structure for compatibility */

  posdat->data_timestamp = priv->posdat2->timestamp;
  posdat->status = 0;
  svnum = MIN(priv->posdat2->svcount, CXD56_GNSS_MAX_SV_NUM);
  posdat->svcount = svnum;
  posdat->receiver.type = (priv->posdat2->receiver.fix_indicator > 0) ?
                           CXD56_GNSS_PVT_TYPE_GNSS : 0;
  posdat->receiver.dgps = (priv->posdat2->receiver.fix_indicator == 2) ?
                            1 : 0;
  posdat->receiver.pos_fixmode = priv->posdat2->receiver.pos_fixmode;
  posdat->receiver.vel_fixmode = CXD56_GNSS_PVT_VELFIX_3D;
  posdat->receiver.numsv = priv->posdat2->receiver.numsv;
  posdat->receiver.assist = CXD56_GNSS_PVT_RECEIVER_ASSIST_NONE;
  posdat->receiver.pos_dataexist = priv->posdat2->receiver.pos_dataexist;
  posdat->receiver.possource = CXD56_GNSS_PVT_TYPE_GNSS;
  posdat->receiver.tcxo_offset = 0;
  posdat->receiver.pos_dop.pdop = priv->posdat2->receiver.pdop;
  posdat->receiver.pos_dop.hdop = priv->posdat2->receiver.hdop;
  posdat->receiver.pos_dop.vdop = priv->posdat2->receiver.vdop;
  posdat->receiver.pos_dop.tdop = 0;
  posdat->receiver.pos_dop.ewdop = 0;
  posdat->receiver.pos_dop.nsdop = 0;
  posdat->receiver.pos_dop.majdop = priv->posdat2->receiver.majdop;
  posdat->receiver.pos_dop.mindop = priv->posdat2->receiver.mindop;
  posdat->receiver.pos_dop.oridop = priv->posdat2->receiver.oridop;
  posdat->receiver.pos_accuracy.hvar = priv->posdat2->receiver.hvar;
  posdat->receiver.pos_accuracy.vvar = priv->posdat2->receiver.vvar;
  posdat->receiver.latitude = priv->posdat2->receiver.latitude;
  posdat->receiver.longitude = priv->posdat2->receiver.longitude;
  posdat->receiver.altitude = priv->posdat2->receiver.altitude;
  posdat->receiver.geoid = priv->posdat2->receiver.geoid;
  posdat->receiver.velocity = priv->posdat2->receiver.velocity;
  posdat->receiver.direction = priv->posdat2->receiver.direction;
  posdat->receiver.date = priv->posdat2->receiver.date;
  posdat->receiver.time = priv->posdat2->receiver.time;
  for (i = 0; i < priv->posdat2->svcount; i++)
    {
      uint8_t stat = priv->posdat2->sv[i].stat;
      uint8_t type = priv->posdat2->sv[i].type;

      if (stat & 0x1)
        {
          svtype |= cxd5610_gnss_signal2type(type);
          numsv_tracking++;
        }

      if (stat & 0x2)
        {
          pos_svtype |= cxd5610_gnss_signal2type(type);
          numsv_calcpos++;
        }

      if (stat & 0x4)
        {
          vel_svtype |= cxd5610_gnss_signal2type(type);
          numsv_calcvel++;
        }
    }

  posdat->receiver.numsv_tracking = numsv_tracking;
  posdat->receiver.numsv_calcpos = numsv_calcpos;
  posdat->receiver.numsv_calcvel = numsv_calcvel;
  posdat->receiver.svtype = svtype;
  posdat->receiver.pos_svtype = pos_svtype;
  posdat->receiver.vel_svtype = vel_svtype;

  for (i = 0; i < svnum; i++)
    {
      uint8_t type = priv->posdat2->sv[i].type;
      posdat->sv[i].type = cxd5610_gnss_signal2type(type);
      posdat->sv[i].svid = priv->posdat2->sv[i].svid;
      posdat->sv[i].stat = priv->posdat2->sv[i].stat;
      posdat->sv[i].azimuth = priv->posdat2->sv[i].azimuth;
      posdat->sv[i].elevation = priv->posdat2->sv[i].elevation;
      posdat->sv[i].siglevel = priv->posdat2->sv[i].siglevel;
    }

  return len;
}
#endif /* CONFIG_SENSORS_CXD5610_GNSS_READ_COMPAT */

/****************************************************************************
 * Name: cxd5610_gnss_read
 ****************************************************************************/

static ssize_t cxd5610_gnss_read(struct file *filep, char *buffer,
                                 size_t len)
{
  struct inode *inode = filep->f_inode;
  struct cxd5610_gnss_dev_s *priv = inode->i_private;
  uint32_t offset = 0;
  int8_t type;

  if (!buffer)
    {
      return -EINVAL;
    }

  if (len == 0)
    {
      return OK;
    }

  cxd5610_gnss_device_lock(priv);

  /* Setect data type */

  type = cxd5610_gnss_select_readtype(filep->f_pos, &offset);
  if (type < 0)
    {
      cxd5610_gnss_device_unlock(priv);
      return -ESPIPE;
    }

  /* Get exclusive control for buffer access */

  cxd5610_gnss_buffer_lock(priv);

  if (type == CXD56_READ_DATA_TYPE_GNSS)
    {
#ifdef CONFIG_SENSORS_CXD5610_GNSS_READ_COMPAT
      if (len == sizeof(struct cxd56_gnss_positiondata_s))
        {
          cxd5610_gnss_read_compat(priv, buffer, len);
        }
      else
#endif
        {
          len = MIN(sizeof(struct cxd56_gnss_positiondata2_s), len);
          memcpy(buffer, priv->posdat2, len);
        }
    }
  else if (type == CXD56_READ_DATA_TYPE_DCREPORT)
    {
      len = MIN(sizeof(struct cxd56_gnss_dcreport_data_s), len);
      memcpy(buffer, priv->dcrdat, len);
    }

  /* Release exclusive control for buffer access */

  cxd5610_gnss_buffer_unlock(priv);

  /* Reset read position after read is complete */

  filep->f_pos = 0;

  cxd5610_gnss_device_unlock(priv);
  return len;
}

/****************************************************************************
 * Name: cxd5610_gnss_write
 ****************************************************************************/

static ssize_t cxd5610_gnss_write(struct file *filep, const char *buffer,
                                  size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: cxd5610_gnss_ioctl
 ****************************************************************************/

static int cxd5610_gnss_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct cxd5610_gnss_dev_s *priv = inode->i_private;
  int ret = OK;

  sninfo("cmd=%d arg=0x%08lx\n", cmd, arg);

  cxd5610_gnss_device_lock(priv);

  switch (cmd)
    {
      case CXD56_GNSS_IOCTL_START:
        ret = cxd5610_gnss_start(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_STOP:
        ret = cxd5610_gnss_stop(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM:
        ret = cxd5610_gnss_select_satellite_system(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_GET_SATELLITE_SYSTEM:
        DEBUGASSERT(arg != 0);
        ret = cxd5610_gnss_get_satellite_system(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ELLIPSOIDAL:
        DEBUGASSERT(arg != 0);
        ret = cxd5610_gnss_set_receiver_position_ellipsoidal(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_SET_OPE_MODE:
        DEBUGASSERT(arg != 0);
        ret = cxd5610_gnss_set_ope_mode(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_GET_OPE_MODE:
        DEBUGASSERT(arg != 0);
        ret = cxd5610_gnss_get_ope_mode(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_SET_TIME:
        DEBUGASSERT(arg != 0);
        ret = cxd5610_gnss_set_time(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_SAVE_BACKUP_DATA:
        ret = cxd5610_gnss_save_backup_data(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_SIGNAL_SET:
        DEBUGASSERT(arg != 0);
        ret = cxd5610_gnss_set_signal(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_SET_1PPS_OUTPUT:
        ret = cxd5610_gnss_set_1pps_output(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_GET_VERSION:
        DEBUGASSERT(arg != 0);
        ret = cxd5610_gnss_get_version(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_SLEEP:
        ret = cxd5610_gnss_sleep(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_WAKEUP:
        ret = cxd5610_gnss_wakeup(priv, arg);
        break;
      case CXD56_GNSS_IOCTL_RESET:
        ret = cxd5610_gnss_reset(priv, arg);
        break;
      default:
        break;
    }

  cxd5610_gnss_device_unlock(priv);

  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_poll
 ****************************************************************************/

static int cxd5610_gnss_poll(struct file *filep, struct pollfd *fds,
                             bool setup)
{
  int                      ret = OK;
#if CONFIG_SENSORS_CXD5610_GNSS_NPOLLWAITERS != 0
  struct inode            *inode = filep->f_inode;
  struct cxd5610_gnss_dev_s *priv = inode->i_private;
  int                      i;

  cxd5610_gnss_device_lock(priv);

  if (setup)
    {
      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      for (i = 0; i < CONFIG_SENSORS_CXD5610_GNSS_NPOLLWAITERS; i++)
        {
          /* Find an unused slot */

          if (priv->fds[i] == NULL)
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      /* No space in priv fds array for poll handling */

      if (i >= CONFIG_SENSORS_CXD5610_GNSS_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret       = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->has_event)
        {
          poll_notify(&fds, 1, POLLIN);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot           = NULL;
      fds->priv       = NULL;
      priv->has_event = false;
    }

errout:
  cxd5610_gnss_device_unlock(priv);
#endif
  return ret;
}

/****************************************************************************
 * Name: cxd5610_gnss_signalhandler
 ****************************************************************************/

#if CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS != 0
static void cxd5610_gnss_signalhandler(struct cxd5610_gnss_dev_s *priv,
                                       uint8_t sigtype)
{
  int i;

  for (i = 0; i < CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS; i++)
    {
      struct gnss_sig_s *sig = &priv->sigs[i];
      if (sig->enable && sig->info.gnsssig == sigtype)
        {
          union sigval value;

          value.sival_ptr = &sig->info;
          nxsig_queue(sig->pid, sig->info.signo, value);
        }
    }
}
#endif

/****************************************************************************
 * Name: cxd5610_gnss_pollnotify
 ****************************************************************************/

#if CONFIG_SENSORS_CXD5610_GNSS_NPOLLWAITERS != 0
static void cxd5610_gnss_pollnotify(struct cxd5610_gnss_dev_s *dev)
{
  poll_notify(dev->fds, CONFIG_SENSORS_CXD5610_GNSS_NPOLLWAITERS, POLLIN);
  dev->has_event = true;
}
#endif

/****************************************************************************
 * Name: cxd5610_gnss_device_init/lock/unlock
 ****************************************************************************/

static int cxd5610_gnss_device_init(struct cxd5610_gnss_dev_s *priv)
{
  return nxmutex_init(&priv->dev_lock);
}

static int cxd5610_gnss_device_lock(struct cxd5610_gnss_dev_s *priv)
{
  return nxmutex_lock(&priv->dev_lock);
}

static int cxd5610_gnss_device_unlock(struct cxd5610_gnss_dev_s *priv)
{
  return nxmutex_unlock(&priv->dev_lock);
}

/****************************************************************************
 * Name: cxd5610_gnss_buffer_init/lock/unlock
 ****************************************************************************/

static int cxd5610_gnss_buffer_init(struct cxd5610_gnss_dev_s *priv)
{
  return nxmutex_init(&priv->buf_lock);
}

static int cxd5610_gnss_buffer_lock(struct cxd5610_gnss_dev_s *priv)
{
  return nxmutex_lock(&priv->buf_lock);
}

static int cxd5610_gnss_buffer_unlock(struct cxd5610_gnss_dev_s *priv)
{
  return nxmutex_unlock(&priv->buf_lock);
}

/****************************************************************************
 * Name: cxd5610_gnss_init/wait/post_boot
 ****************************************************************************/

static int cxd5610_gnss_init_boot(struct cxd5610_gnss_dev_s *priv)
{
  return nxsem_init(&priv->boot_sync, 0, 0);
}

static int cxd5610_gnss_wait_boot(struct cxd5610_gnss_dev_s *priv,
                                     int sec)
{
  if (sec <= 0)
    {
      return nxsem_wait_uninterruptible(&priv->boot_sync);
    }
  else
    {
      return nxsem_tickwait_uninterruptible(&priv->boot_sync, SEC2TICK(sec));
    }
}

static int cxd5610_gnss_post_boot(struct cxd5610_gnss_dev_s *priv)
{
  return nxsem_post(&priv->boot_sync);
}

/****************************************************************************
 * Name: cxd5610_gnss_init/wait/post_command
 ****************************************************************************/

static int cxd5610_gnss_init_command(struct cxd5610_gnss_dev_s *priv)
{
  return nxsem_init(&priv->cmd_sync, 0, 0);
}

static int cxd5610_gnss_wait_command(struct cxd5610_gnss_dev_s *priv,
                                     int sec)
{
  if (sec <= 0)
    {
      return nxsem_wait_uninterruptible(&priv->cmd_sync);
    }
  else
    {
      return nxsem_tickwait_uninterruptible(&priv->cmd_sync, SEC2TICK(sec));
    }
}

static int cxd5610_gnss_post_command(struct cxd5610_gnss_dev_s *priv)
{
  return nxsem_post(&priv->cmd_sync);
}

/****************************************************************************
 * Name: cxd5610_gnss_init/wait/post_interrupt
 ****************************************************************************/

static int cxd5610_gnss_init_interrupt(void)
{
  return nxsem_init(&g_int_sync, 0, 0);
}

static int cxd5610_gnss_wait_interrupt(void)
{
  return nxsem_wait_uninterruptible(&g_int_sync);
}

static int cxd5610_gnss_post_interrupt(void)
{
  return nxsem_post(&g_int_sync);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd5610_gnss_register
 *
 * Description:
 *   Register the CXD5610 GNSS character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gps2"
 *   lower   - An instance of the lower half interface
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd5610_gnss_register(const char *devpath,
                          struct cxd5610_gnss_lowerhalf_s *lower)
{
  struct cxd5610_gnss_dev_s *priv;
  int ret;

  /* Initialize the CXD5610 GNSS device structure */

  priv = (struct cxd5610_gnss_dev_s *)
           kmm_zalloc(sizeof(struct cxd5610_gnss_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Register lower-half driver */

  priv->lower = lower;

  cxd5610_gnss_device_init(priv);
  cxd5610_gnss_buffer_init(priv);
  cxd5610_gnss_init_boot(priv);
  cxd5610_gnss_init_command(priv);
  cxd5610_gnss_init_interrupt();

  /* Register the character driver */

  ret = register_driver(devpath, &g_cxd5610fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("CXD5610 GNSS driver loaded successfully!\n");

  return ret;
}

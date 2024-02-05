/****************************************************************************
 * arch/arm/src/nrf91/nrf91_modem_gnss.c
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

#include <nuttx/kthread.h>

#include <debug.h>
#include <string.h>
#include <time.h>

#include <nuttx/sensors/gps.h>
#include <nuttx/sensors/sensor.h>

#include "nrf_modem.h"
#include "nrf_modem_gnss.h"

#include "nrf91_modem_at.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SENSORS_GPS
#  error nRF91 GNSS driver needs CONFIG_SENSORS_GPS
#endif

/* NMEA and AGPS not supported yet */

#undef NRF91_GNSS_NMEA
#undef NRF91_GNSS_AGPS

/* Some default values */

#define NRF91_GNSS_USE_CASE     NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START
#define NRF91_GNSS_POWER_MODE   NRF_MODEM_GNSS_PSM_DISABLED
#ifdef NRF91_GNSS_NMEA
#  define NRF91_GNSS_NMEA_MASK  (NRF_MODEM_GNSS_NMEA_GGA_MASK | \
                                 NRF_MODEM_GNSS_NMEA_GLL_MASK | \
                                 NRF_MODEM_GNSS_NMEA_GSA_MASK | \
                                 NRF_MODEM_GNSS_NMEA_GSV_MASK | \
                                 NRF_MODEM_GNSS_NMEA_RMC_MASK)
#else
#  define NRF91_GNSS_NMEA_MASK  0
#endif

/* Shorten some names */

#define NRF_GNSS_PVT_FRAME_LEN  (sizeof(struct nrf_modem_gnss_pvt_data_frame))
#define NRF_GNSS_NMEA_FRAME_LEN (sizeof(struct nrf_modem_gnss_nmea_data_frame))
#define NRF_GNSS_AGPS_FRAME_LEN (sizeof(struct nrf_modem_gnss_agps_data_frame))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf91_gnss_s
{
  /* lower must be first element */

  struct gps_lowerhalf_s                lower;
  bool                                  running;
  bool                                  singlefix;
  int                                   notime_cntr;
  sem_t                                 rx_sem;

  /* PVT support */

  bool                                  pvt_evt;
  struct nrf_modem_gnss_pvt_data_frame  pvt;

#ifdef NRF91_GNSS_NMEA
  /* NMEA support */

  bool                                  nmea_evt;
  struct nrf_modem_gnss_nmea_data_frame nmea;
#endif

#ifdef NRF91_GNSS_AGPS
  /* AGPS support */

  bool                                  agps_evt;
  struct nrf_modem_gnss_agps_data_frame agps;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* GPS operations */

static int nrf91_gnss_activate(struct gps_lowerhalf_s *lower,
                               struct file *filep, bool enable);
static int nrf91_gnss_set_interval(struct gps_lowerhalf_s *lower,
                                   struct file *filep,
                                   unsigned long *period_us);
static int nrf91_gnss_control(struct gps_lowerhalf_s *lower,
                              struct file *filep,
                              int cmd, unsigned long arg);
static ssize_t nrf91_gnss_inject_data(struct gps_lowerhalf_s *lower,
                                      struct file *filep,
                                      const void *buffer, size_t buflen);

/* Helpers */

static int nrf91_gnss_configure(void);
static bool nrf91_gnss_isactive(int cfun);
static int nrf91_gnss_enable(struct nrf91_gnss_s *priv, bool enable);
#ifdef CONFIG_NRF91_MODEM_GNSS_BOOST_PRIO
static void nrf91_gnss_boost_prio(struct nrf91_gnss_s *priv);
#endif
static void nrf91_gnss_pvt_event(struct nrf91_gnss_s *priv);
static void nrf91_gnss_event_handler(int event);
static int nrf91_gnss_thread(int argc, char** argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gps_ops_s g_nrf91_gnss_ops =
{
  .activate     = nrf91_gnss_activate,
  .set_interval = nrf91_gnss_set_interval,
  .control      = nrf91_gnss_control,
  .inject_data  = nrf91_gnss_inject_data,
};

static struct nrf91_gnss_s g_nrf91_gnss;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_gnss_configure
 ****************************************************************************/

static int nrf91_gnss_configure(void)
{
  int ret = 0;

  /* Set NMEA strings */

  ret = nrf_modem_gnss_nmea_mask_set(NRF91_GNSS_NMEA_MASK);
  if (ret < 0)
    {
      goto errout;
    }

  /* Set targeted start and runtime performance */

  ret = nrf_modem_gnss_use_case_set(NRF91_GNSS_USE_CASE);
  if (ret < 0)
    {
      goto errout;
    }

  /* Set the used power saving mode */

  ret = nrf_modem_gnss_power_mode_set(NRF91_GNSS_POWER_MODE);
  if (ret < 0)
    {
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf91_gnss_isactive
 ****************************************************************************/

static bool nrf91_gnss_isactive(int cfun)
{
  return (cfun == NRF91_MODEM_FUNC_FULL ||
          cfun == NRF91_MODEM_FUNC_RXONLY ||
          cfun == NRF91_MODEM_FUNC_DEACTIVATE_LTE ||
          cfun == NRF91_MODEM_FUNC_ACTIVATE_GNSS);
}

/****************************************************************************
 * Name: nrf91_gnss_enable
 ****************************************************************************/

static int nrf91_gnss_enable(struct nrf91_gnss_s *priv, bool enable)
{
  int ret  = OK;
  int cfun = 0;

  if (enable)
    {
      /* Get modem functional mode */

      ret = nrf_modem_at_scanf("AT+CFUN?", "+CFUN: %d", &cfun);
      if (ret < 0)
        {
          snerr("nrf_modem_at_scanf failed %d", ret);
          goto errout;
        }

      /* GNSS must be active */

      if (!nrf91_gnss_isactive(cfun))
        {
          snerr("GNSS is not activated!");
          ret = -EACCES;
          goto errout;
        }

      /* Configure GNSS modem */

      ret = nrf91_gnss_configure();
      if (ret < 0)
        {
          snerr("nrf91_gnss_configure failed %d\n", ret);
          return ret;
        }

      ret = nrf_modem_gnss_start();
      if (ret < 0)
        {
          snerr("nrf_modem_gnss_start failed %d", ret);
          goto errout;
        }

      priv->running = true;
    }
  else
    {
      ret = nrf_modem_gnss_stop();
      if (ret < 0)
        {
          snerr("nrf_modem_gnss_stop failed %d", ret);
          goto errout;
        }

      priv->running = false;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf91_gnss_activate
 ****************************************************************************/

static int nrf91_gnss_activate(struct gps_lowerhalf_s *lower,
                               struct file *filep, bool enable)
{
  struct nrf91_gnss_s *priv = (struct nrf91_gnss_s *)lower;

  return nrf91_gnss_enable(priv, enable);
}

/****************************************************************************
 * Name: nrf91_gnss_set_interval
 ****************************************************************************/

static int nrf91_gnss_set_interval(struct gps_lowerhalf_s *lower,
                                   struct file *filep,
                                   unsigned long *period_us)
{
  struct nrf91_gnss_s *priv         = (struct nrf91_gnss_s *)lower;
  uint16_t             fix_interval = 0;
  int                  ret          = OK;
  bool                 running      = priv->running;

  /* GNSS must be disabled when fix interval change */

  if (running == true)
    {
      nrf91_gnss_enable(priv, false);
    }

  /* Fix interval in seconds */

  fix_interval = (*period_us) / 1000000;

  /* Handle GNSS mode */

  if (fix_interval == 1)
    {
      /* Continuous navigation with 1 Hz rate */
    }
  else if (fix_interval == 0)
    {
      /* Single fix */

      priv->singlefix = true;
    }
  else if (fix_interval < 10)
    {
      /* Periodic navigation, minimum interval is 10s */

      fix_interval = 10;
      *period_us   = fix_interval * 1000000;
    }
  else if (fix_interval > 65535)
    {
      /* Periodic navigation, maximum interval is 65535s */

      fix_interval = 65535;
      *period_us   = fix_interval * 1000000;
    }

  ret = nrf_modem_gnss_fix_interval_set(fix_interval);
  if (ret < 0)
    {
      snerr("nrf_modem_gnss_fix_interval_set %d\n", ret);
    }

  if (running == true)
    {
      nrf91_gnss_enable(priv, true);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf91_gnss_control
 ****************************************************************************/

static int nrf91_gnss_control(struct gps_lowerhalf_s *lower,
                              struct file *filep,
                              int cmd, unsigned long arg)
{
  /* TODO */

  return 0;
}

/****************************************************************************
 * Name: nrf91_gnss_inject_data
 ****************************************************************************/

static ssize_t nrf91_gnss_inject_data(struct gps_lowerhalf_s *lower,
                                      struct file *filep,
                                      const void *buffer, size_t buflen)
{
  /* TODO */

  return 0;
}

#ifdef CONFIG_NRF91_MODEM_GNSS_BOOST_PRIO
/****************************************************************************
 * Name: nrf91_gnss_boost_prio
 ****************************************************************************/

static void nrf91_gnss_boost_prio(struct nrf91_gnss_s *priv)
{
  int ret;

  /* Boost GNSS priority only once - we don't want to block LTE too long */

  if (priv->notime_cntr != -1)
    {
      if (priv->notime_cntr > 5)
        {
          ret = nrf_modem_gnss_prio_mode_enable();
          if (ret < 0)
            {
              snerr("nrf_modem_gnss_prio_mode_enable failed %d!", ret);
            }

          priv->notime_cntr = -1;
        }
    }
}
#endif

/****************************************************************************
 * Name: nrf91_gnss_pvt_event
 ****************************************************************************/

static void nrf91_gnss_pvt_event(struct nrf91_gnss_s *priv)
{
  struct sensor_gps           gps;
  struct sensor_gps_satellite sat;
  struct tm                   tm;
  uint64_t                    timestamp;
  int                         i    = 0;
  int                         j    = 0;
  int                         sv   = 0;
  int                         used = 0;

  timestamp = sensor_get_timestamp();

  for (i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++)
    {
      if (priv->pvt.sv[i].sv > 0)
        {
          sv += 1;

          if (j < SENSOR_GPS_SAT_INFO_MAX)
            {
              sat.info[j].svid      = priv->pvt.sv[i].sv;
              sat.info[j].elevation = priv->pvt.sv[i].elevation;
              sat.info[j].azimuth   = priv->pvt.sv[i].azimuth;
              sat.info[j].snr       = priv->pvt.sv[i].cn0 / 10;
              j++;
            }

          if (priv->pvt.sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX)
            {
              used += 1;
            }
        }
    }

  /* TODO: limit GPS-SATELLITE data rate when warning flags set */

  if (sv > 0)
    {
      sat.satellites = sv;
      sat.timestamp  = timestamp;
      sat.count      = j;

      /* Push GPS-SATELLITE event to upper-half */

      priv->lower.push_event(priv->lower.priv,
                             &sat,
                             sizeof(struct sensor_gps_satellite),
                             SENSOR_TYPE_GPS_SATELLITE);
    }

  if (priv->pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID)
    {
      /* Get UTC timestamp */

      tm.tm_sec   = priv->pvt.datetime.seconds;
      tm.tm_min   = priv->pvt.datetime.minute;
      tm.tm_hour  = priv->pvt.datetime.hour;
      tm.tm_mday  = priv->pvt.datetime.day;
      tm.tm_mon   = priv->pvt.datetime.month - 1;
      tm.tm_year  = priv->pvt.datetime.year - 1900;
      tm.tm_isdst = 0;

      gps.timestamp          = timestamp;
      gps.time_utc           = mktime(&tm);
      gps.latitude           = priv->pvt.latitude;
      gps.longitude          = priv->pvt.longitude;
      gps.altitude           = priv->pvt.altitude;
      gps.altitude_ellipsoid = 0;
      gps.eph                = priv->pvt.accuracy;
      gps.epv                = priv->pvt.altitude_accuracy;
      gps.hdop               = priv->pvt.hdop;
      gps.pdop               = priv->pvt.pdop;
      gps.vdop               = priv->pvt.vdop;
      gps.ground_speed       = priv->pvt.speed;
      gps.course             = 0;
      gps.satellites_used    = used;

      /* Push GPS event to upper-half */

      priv->lower.push_event(priv->lower.priv,
                             &gps,
                             sizeof(struct sensor_gps),
                             SENSOR_TYPE_GPS);
    }

  if (priv->pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_SLEEP_BETWEEN_PVT)
    {
      snerr("GNSS SLEEP_BETWEEN_PVT!");
    }

  if (priv->pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_DEADLINE_MISSED)
    {
      snerr("GNSS DEADLINE_MISSED!");
    }

  if (priv->pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME)
    {
      snerr("GNSS NOT_ENOUGH_WINDOW_TIME!");

#ifdef CONFIG_NRF91_MODEM_GNSS_BOOST_PRIO
      /* GNSS priority boost over LTE idle */

      nrf91_gnss_boost_prio(priv);
#endif

      priv->notime_cntr++;
    }
  else
    {
      /* Reset counter */

      priv->notime_cntr = 0;
    }
}

/****************************************************************************
 * Name: nrf91_gnss_event_handler
 ****************************************************************************/

static void nrf91_gnss_event_handler(int event)
{
  struct nrf91_gnss_s *priv = &g_nrf91_gnss;
  int ret                   = OK;

  switch (event)
    {
      case NRF_MODEM_GNSS_EVT_PVT:
      {
        ret = nrf_modem_gnss_read(&priv->pvt,
                                  NRF_GNSS_PVT_FRAME_LEN,
                                  NRF_MODEM_GNSS_DATA_PVT);
        if (ret < 0)
          {
            snerr("nrf_modem_gnss_read failed %d", ret);
            return;
          }

        priv->pvt_evt = true;

        /* Wake-up any thread waiting */

        nxsem_post(&g_nrf91_gnss.rx_sem);

        break;
      }

#ifdef NRF91_GNSS_NMEA
    case NRF_MODEM_GNSS_EVT_NMEA:
      {
        ret = nrf_modem_gnss_read(&priv->nmea,
                                  NRF_GNSS_NMEA_FRAME_LEN,
                                  NRF_MODEM_GNSS_DATA_NMEA);
        if (ret < 0)
          {
            snerr("nrf_modem_gnss_read failed %d", ret);
            return;
          }

        priv->nmea_evt = true;

        /* Wake-up any thread waiting */

        nxsem_post(&g_nrf91_gnss.rx_sem);

        break;
      }
#endif

#ifdef NRF91_GNSS_AGPS
    case NRF_MODEM_GNSS_EVT_AGPS_REQ:
        {
          ret = nrf_modem_gnss_read(&priv->agps,
                                    NRF_GNSS_AGPS_FRAME_LEN,
                                    NRF_MODEM_GNSS_DATA_AGPS_REQ);
          if (ret < 0)
            {
              snerr("nrf_modem_gnss_read failed %d", ret);
              return;
            }

          priv->agps_evt = true;

          /* Wake-up any thread waiting */

          nxsem_post(&g_nrf91_gnss.rx_sem);

          break;
        }
#endif

      case NRF_MODEM_GNSS_EVT_FIX:
      case NRF_MODEM_GNSS_EVT_BLOCKED:
      case NRF_MODEM_GNSS_EVT_UNBLOCKED:
      case NRF_MODEM_GNSS_EVT_PERIODIC_WAKEUP:
      case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_TIMEOUT:
      case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_FIX:
      case NRF_MODEM_GNSS_EVT_REF_ALT_EXPIRED:
        {
          /* Not supported yet */

          break;
        }

      default:
        {
          snerr("Unknown GNSS event %d", event);
          break;
        }
    }
}

/****************************************************************************
 * Name: nrf91_gnss_thread
 ****************************************************************************/

static int nrf91_gnss_thread(int argc, char** argv)
{
  struct nrf91_gnss_s *priv = &g_nrf91_gnss;
  int                  ret;

  while (true)
    {
      /* Read data */

      ret = nxsem_wait(&priv->rx_sem);
      if (ret < 0)
        {
          return ret;
        }

      if (priv->running)
        {
          if (priv->pvt_evt)
            {
              nrf91_gnss_pvt_event(priv);
              priv->pvt_evt = false;

              /* Disable GNSS if this was single fix */

              if (priv->singlefix)
                {
                  nrf91_gnss_enable(priv, false);
                }
            }

#ifdef NRF91_GNSS_NMEA
          if (priv->nmea_evt)
            {
              /* TODO: Forward NEMA messages to the character device */

              priv->nmea_evt = false;
            }
#endif

#ifdef NRF91_GNSS_AGPS
          if (priv->agps_evt)
            {
              /* TODO: APGS support */

              priv->agps_evt = false;
            }
#endif
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_gnss_register
 *
 * Description:
 *   NRF91 GNSS driver registration
 *
 * Input Parameters:
 *   devno        - The user specifies which device of this type, from 0.
 *   batch_number - The maximum number of batch.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nrf91_gnss_register(int devno, uint32_t batch_number)
{
  int ret = OK;

  if (!nrf_modem_is_initialized())
    {
      snerr("Modem not initialized!");
      return -EACCES;
    }

  /* Reset memory */

  memset(&g_nrf91_gnss, 0, sizeof(struct nrf91_gnss_s));

  /* Configure sem */

  nxsem_init(&g_nrf91_gnss.rx_sem, 0, 0);

  /* Register the GNSS event handler */

  ret = nrf_modem_gnss_event_handler_set(nrf91_gnss_event_handler);
  if (ret < 0)
    {
      return ret;
    }

  /* Create thread for sensor */

  ret = kthread_create("gnss_thread",
                       SCHED_PRIORITY_DEFAULT,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       nrf91_gnss_thread,
                       NULL);
  if (ret < 0)
    {
      return ret;
    }

  /* Register sensor */

  g_nrf91_gnss.lower.ops = &g_nrf91_gnss_ops;

  return gps_register(&g_nrf91_gnss.lower, devno, batch_number);
}

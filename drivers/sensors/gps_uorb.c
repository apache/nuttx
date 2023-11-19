/****************************************************************************
 * drivers/sensors/gps_uorb.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/mm/circbuf.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/gps.h>

#include <fcntl.h>
#include <poll.h>
#include <debug.h>
#include <minmea/minmea.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPS_PATH_FMT          "/dev/ttyGPS%d"

#define GPS_IDX               0
#define GPS_SATELLITE_IDX     1
#define GPS_MAX_IDX           2

#define GPS_RECV_BUFFERSIZE   2048
#define GPS_PARSE_BUFFERSIZE  256

#define GPS_KNOT_TO_KMH       1.852f
#define GPS_KMH_TO_MPS        3.6f

#define GPS_FLAG_GGA          (1 << 0)
#define GPS_FLAG_RMC          (1 << 1)
#define GPS_FLAG_MARK         (GPS_FLAG_GGA | GPS_FLAG_RMC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes gps sensor info */

struct gps_sensor_s
{
  struct sensor_lowerhalf_s lower;
  FAR void                 *upper;
};

/* This structure describes user info of gps */

struct gps_user_s
{
  FAR struct pollfd *fds;
  size_t pos;
};

/* This structure describes the state of the upper half driver */

struct gps_upperhalf_s
{
  struct gps_sensor_s         dev[GPS_MAX_IDX];
  FAR struct gps_lowerhalf_s *lower;
  uint8_t                     crefs;
  uint8_t                     flags;
  mutex_t                     lock;
  sem_t                       buffersem;
  size_t                      parsenext;
  char                        parsebuffer[GPS_PARSE_BUFFERSIZE];
  struct circbuf_s            buffer;
  struct sensor_gps           gps;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gps_activate(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, bool enable);
static int gps_set_interval(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR unsigned long *interval);
static int gps_control(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep, int cmd, unsigned long arg);

static int     gps_open(FAR struct file *filep);
static int     gps_close(FAR struct file *filep);
static ssize_t gps_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t gps_write(FAR struct file *filep,
                         FAR const char *buffer, size_t buflen);
static int     gps_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);
static int     gps_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_gps_sensor_ops =
{
  .activate     = gps_activate,
  .set_interval = gps_set_interval,
  .control      = gps_control,
};

static const struct file_operations g_gps_fops =
{
  gps_open,   /* open  */
  gps_close,  /* close */
  gps_read,   /* read  */
  gps_write,  /* write */
  NULL,       /* seek  */
  gps_ioctl,  /* ioctl */
  NULL,       /* mmap */
  NULL,       /* truncate */
  gps_poll    /* poll  */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gps_activate(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, bool enable)
{
  FAR struct gps_sensor_s *dev = (FAR struct gps_sensor_s *)lower;
  FAR struct gps_upperhalf_s *upper = dev->upper;
  int ret = OK;

  nxmutex_lock(&upper->lock);
  if ((upper->crefs == 0 && enable) || (upper->crefs == 1 && !enable))
    {
      ret = upper->lower->ops->activate(upper->lower, filep, enable);
      if (ret >= 0)
        {
          upper->crefs += enable ? 1 : -1;
        }
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

static int gps_set_interval(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR unsigned long *interval)
{
  FAR struct gps_sensor_s *dev = (FAR struct gps_sensor_s *)lower;
  FAR struct gps_upperhalf_s *upper = dev->upper;

  if (upper->lower->ops->set_interval == NULL)
    {
      return -ENOTTY;
    }

  return upper->lower->ops->set_interval(upper->lower, filep, interval);
}

static int gps_control(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct gps_sensor_s *dev = (FAR struct gps_sensor_s *)lower;
  FAR struct gps_upperhalf_s *upper = dev->upper;

  if (upper->lower->ops->control == NULL)
    {
      return -ENOTTY;
    }

  return upper->lower->ops->control(upper->lower, filep, cmd, arg);
}

static int gps_open(FAR struct file *filep)
{
  FAR struct gps_upperhalf_s *upper;
  FAR struct gps_user_s *user;
  int ret = OK;

  upper = filep->f_inode->i_private;

  user = kmm_zalloc(sizeof(struct gps_user_s));
  if (user == NULL)
    {
      return -ENOMEM;
    }

  nxmutex_lock(&upper->lock);
  if (upper->crefs >= 255)
    {
      ret = -EMFILE;
      kmm_free(user);
      goto out;
    }
  else
    {
      if (upper->crefs == 0)
        {
          ret = upper->lower->ops->activate(upper->lower, filep, true);
          if (ret < 0)
            {
              kmm_free(user);
              goto out;
            }
        }

      upper->crefs++;
    }

  filep->f_priv = user;
  user->pos = upper->buffer.head;

out:
  nxmutex_unlock(&upper->lock);
  return ret;
}

static int gps_close(FAR struct file *filep)
{
  FAR struct gps_upperhalf_s *upper;
  FAR struct gps_user_s *user;
  int ret = OK;

  DEBUGASSERT(filep->f_priv);
  upper = filep->f_inode->i_private;
  user = filep->f_priv;

  nxmutex_lock(&upper->lock);
  if (upper->crefs > 0)
    {
      if (upper->crefs == 1)
        {
          ret = upper->lower->ops->activate(upper->lower, filep, false);
          if (ret < 0)
            {
              goto out;
            }
        }

      upper->crefs--;
    }

  kmm_free(user);

out:
  nxmutex_unlock(&upper->lock);
  return ret;
}

static ssize_t gps_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct gps_upperhalf_s *upper;
  FAR struct gps_user_s *user;
  ssize_t ret;

  if (buffer == NULL || buflen == 0)
    {
      return 0;
    }

  upper = filep->f_inode->i_private;
  user = filep->f_priv;

  nxmutex_lock(&upper->lock);
  if (user->pos < upper->buffer.tail)
    {
      user->pos = upper->buffer.tail;
    }

check:
  if (upper->buffer.head - user->pos == 0)
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto out;
        }
      else
        {
          nxmutex_unlock(&upper->lock);
          ret = nxsem_wait_uninterruptible(&upper->buffersem);
          if (ret < 0)
            {
              return ret;
            }

          nxmutex_lock(&upper->lock);
          goto check;
        }
    }

  ret = circbuf_peekat(&upper->buffer, user->pos,
                       buffer, buflen);
  user->pos += ret;

out:
  nxmutex_unlock(&upper->lock);
  return ret;
}

static ssize_t gps_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  FAR struct gps_upperhalf_s *upper;
  int ret = -ENOTSUP;

  upper = filep->f_inode->i_private;

  nxmutex_lock(&upper->lock);
  if (upper->lower->ops->inject_data != NULL)
    {
      ret = upper->lower->ops->inject_data(upper->lower, filep,
                                           buffer, buflen);
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

static int gps_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct gps_upperhalf_s *upper;
  int ret = -ENOTTY;

  upper = filep->f_inode->i_private;

  nxmutex_lock(&upper->lock);
  if (cmd == SNIOC_SET_INTERVAL)
    {
      if (upper->lower->ops->set_interval != NULL)
        {
          ret = upper->lower->ops->set_interval(upper->lower, filep,
                               (FAR unsigned long *)(uintptr_t)arg);
        }
    }
  else if (upper->lower->ops->control != NULL)
    {
      ret = upper->lower->ops->control(upper->lower, filep, cmd, arg);
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

static int gps_poll(FAR struct file *filep, FAR struct pollfd *fds,
                    bool setup)
{
  FAR struct gps_upperhalf_s *upper;
  FAR struct gps_user_s *user;
  ssize_t ret = OK;

  upper = filep->f_inode->i_private;
  user = filep->f_priv;

  nxmutex_lock(&upper->lock);
  if (setup)
    {
      if (user->fds)
        {
          ret = -ENOSPC;
          goto out;
        }

      user->fds = fds;
      fds->priv = filep;
      if (upper->buffer.head > user->pos)
        {
          poll_notify(&fds, 1, POLLIN);
        }
    }
  else if (user->fds)
    {
      user->fds = NULL;
      fds->priv = NULL;
    }

out:
  nxmutex_unlock(&upper->lock);
  return ret;
}

static void gps_init_data(FAR struct sensor_gps *gps)
{
  gps->timestamp = ULONG_MAX;
  gps->time_utc = ULONG_MAX;
  gps->latitude = NAN;
  gps->longitude = NAN;
  gps->altitude = NAN;
  gps->altitude_ellipsoid = NAN;
  gps->eph = NAN;
  gps->epv = NAN;
  gps->hdop = NAN;
  gps->pdop = NAN;
  gps->vdop = NAN;
  gps->ground_speed = NAN;
  gps->course = NAN;
  gps->satellites_used = UINT_MAX;
}

static void gps_parse_nmea(FAR struct gps_upperhalf_s *upper,
                           FAR const char *nmea)
{
  FAR struct sensor_lowerhalf_s *lower;

  switch (minmea_sentence_id(nmea, false))
    {
      case MINMEA_SENTENCE_GGA:
        {
          struct minmea_sentence_gga frame;

          if (minmea_parse_gga(&frame, nmea))
            {
              upper->gps.altitude = minmea_tofloat(&frame.altitude);
              upper->gps.altitude_ellipsoid =
                             minmea_tofloat(&frame.height);
              upper->gps.hdop = minmea_tofloat(&frame.hdop);
              upper->gps.satellites_used = frame.satellites_tracked;
              upper->flags |= GPS_FLAG_GGA;
            }

          break;
        }

      case MINMEA_SENTENCE_RMC:
        {
          struct minmea_sentence_rmc frame;
          struct tm t;

          if (minmea_parse_rmc(&frame, nmea))
            {
              upper->gps.timestamp = sensor_get_timestamp();
              memset(&t, 0, sizeof(t));
              t.tm_sec = frame.time.seconds;
              t.tm_min = frame.time.minutes;
              t.tm_hour = frame.time.hours;
              t.tm_mday = frame.date.day;
              t.tm_mon = frame.date.month - 1;
              t.tm_year = frame.date.year + 2000 - 1900;
              t.tm_isdst = 0;
              upper->gps.time_utc = mktime(&t);
              upper->gps.latitude = minmea_tocoord(&frame.latitude);
              upper->gps.longitude = minmea_tocoord(&frame.longitude);
              upper->gps.ground_speed = minmea_tofloat(&frame.speed) *
                            GPS_KNOT_TO_KMH / GPS_KMH_TO_MPS;
              upper->gps.course = minmea_tofloat(&frame.course);
              if (frame.valid)
                {
                  upper->flags |= GPS_FLAG_RMC;
                }
            }

          break;
        }

      case MINMEA_SENTENCE_GST:
        {
          struct minmea_sentence_gst frame;
          float lat_err;
          float lon_err;

          if (minmea_parse_gst(&frame, nmea))
            {
              lat_err = minmea_tofloat(
                        &frame.latitude_error_deviation);
              lat_err *= lat_err;
              lon_err = minmea_tofloat(
                        &frame.longitude_error_deviation);
              lon_err *= lon_err;
              upper->gps.eph = sqrtf(lat_err + lon_err);
              upper->gps.epv = minmea_tofloat(
                        &frame.altitude_error_deviation);
            }

          break;
        }

      case MINMEA_SENTENCE_GSV:
        {
          struct minmea_sentence_gsv frame;
          struct sensor_gps_satellite satellite;

          memset(&satellite, 0, sizeof(satellite));
          if (minmea_parse_gsv(&frame, nmea))
            {
              satellite.timestamp = sensor_get_timestamp();
              satellite.count = frame.total_msgs;
              satellite.satellites = frame.total_sats;
              memcpy(satellite.info, frame.sats,
                     sizeof(satellite.info[0]) * 4);
              lower = &upper->dev[GPS_SATELLITE_IDX].lower;
              lower->push_event(lower->priv, &satellite,
                                sizeof(satellite));
            }

          break;
        }

       default:
         break;
    }

  if (GPS_FLAG_MARK == upper->flags)
    {
      upper->flags &= ~GPS_FLAG_MARK;
      lower = &upper->dev[GPS_IDX].lower;
      lower->push_event(lower->priv, &upper->gps, sizeof(upper->gps));
      gps_init_data(&upper->gps);
    }
}

static void gps_parse(FAR struct gps_upperhalf_s *upper,
                      FAR const char *buffer, size_t bytes)
{
  bool newline = upper->parsenext != 0;

  for (; bytes > 0; buffer++, bytes--)
    {
      if (*buffer == '$')
        {
          newline = true;
        }

      if (newline)
        {
          if (*buffer != '\r' && *buffer != '\n')
            {
              upper->parsebuffer[upper->parsenext++] = *buffer;
              continue;
            }

          upper->parsebuffer[upper->parsenext] = '\0';
          gps_parse_nmea(upper, upper->parsebuffer);
          upper->parsenext = 0;
          newline = false;
        }
    }
}

static void gps_push_data(FAR void *priv, FAR const void *data,
                           size_t bytes, bool is_nmea)
{
  FAR struct gps_upperhalf_s *upper = priv;
  int semcount;

  if (data == NULL || bytes == 0)
    {
      return;
    }

  nxmutex_lock(&upper->lock);
  if (is_nmea)
    {
      gps_parse(upper, data, bytes);
    }

  circbuf_overwrite(&upper->buffer, data, bytes);
  nxmutex_unlock(&upper->lock);

  nxsem_get_value(&upper->buffersem, &semcount);
  while (semcount++ <= 0)
    {
      nxsem_post(&upper->buffersem);
    }
}

static void gps_push_event(FAR void *priv, FAR const void *data,
                            size_t bytes, int type)
{
  FAR struct gps_upperhalf_s *upper = priv;
  FAR struct sensor_lowerhalf_s *lower;

  if (data == NULL || bytes == 0)
    {
      return;
    }

  nxmutex_lock(&upper->lock);
  if (type == SENSOR_TYPE_GPS)
    {
      lower = &upper->dev[GPS_IDX].lower;
      lower->push_event(lower->priv, data, bytes);
    }
  else if (type == SENSOR_TYPE_GPS_SATELLITE)
    {
      lower = &upper->dev[GPS_SATELLITE_IDX].lower;
      lower->push_event(lower->priv, data, bytes);
    }

  nxmutex_unlock(&upper->lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gps_register
 *
 * Description:
 *   This function binds an instance of a "lower half" gps driver with the
 *   "upper half" gps device and registers that device so that can be used
 *   by application code.
 *
 * Input Parameters:
 *   dev     - A pointer to an instance of lower half gps driver. This
 *             instance is bound to the gps driver and must persist as long
 *             as the driver persists.
 *   devno   - The user specifies which device of this type, from 0. If the
 *             devno alerady exists, -EEXIST will be returned.
 *   nbuffer - The number of events that the circular buffer can hold.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int gps_register(FAR struct gps_lowerhalf_s *lower, int devno,
                 uint32_t nbuffer)
{
  FAR struct gps_upperhalf_s *upper;
  FAR struct gps_sensor_s *dev;
  char path[PATH_MAX];
  int ret;

  upper = kmm_zalloc(sizeof(struct gps_upperhalf_s));
  if (upper == NULL)
    {
      return -ENOMEM;
    }

  lower->push_data = gps_push_data;
  lower->push_event = gps_push_event;
  lower->priv = upper;
  upper->lower = lower;

  nxmutex_init(&upper->lock);
  nxsem_init(&upper->buffersem, 0, 0);
  gps_init_data(&upper->gps);

  /* GPS register */

  dev = &upper->dev[GPS_IDX];
  dev->lower.ops = &g_gps_sensor_ops;
  dev->lower.type = SENSOR_TYPE_GPS;
  dev->lower.nbuffer = nbuffer;
  dev->upper = upper;
  ret = sensor_register(&dev->lower, devno);
  if (ret < 0)
    {
      goto gps_err;
    }

  /* Satellite register */

  dev = &upper->dev[GPS_SATELLITE_IDX];
  dev->lower.ops = &g_gps_sensor_ops;
  dev->lower.type = SENSOR_TYPE_GPS_SATELLITE;
  dev->lower.nbuffer = nbuffer;
  dev->upper = upper;
  ret = sensor_register(&dev->lower, devno);
  if (ret < 0)
    {
      goto satellite_err;
    }

  ret = circbuf_init(&upper->buffer, NULL, GPS_RECV_BUFFERSIZE);
  if (ret < 0)
    {
      goto circ_err;
    }

  snprintf(path, PATH_MAX, GPS_PATH_FMT, devno);
  ret = register_driver(path, &g_gps_fops, 0666, upper);
  if (ret < 0)
    {
      goto driver_err;
    }

  return ret;

driver_err:
  circbuf_uninit(&upper->buffer);
circ_err:
  sensor_unregister(&upper->dev[GPS_SATELLITE_IDX].lower, devno);
satellite_err:
  sensor_unregister(&upper->dev[GPS_IDX].lower, devno);
gps_err:
  nxmutex_destroy(&upper->lock);
  nxsem_destroy(&upper->buffersem);
  kmm_free(upper);
  return ret;
}

/****************************************************************************
 * Name: gps_unregister
 *
 * Description:
 *   This function unregisters character node and releases all resource from
 *   upper half driver. This API corresponds to the gps_register.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half gps driver. This
 *           instance is bound to the gps driver and must persists as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0.
 ****************************************************************************/

void gps_unregister(FAR struct gps_lowerhalf_s *lower, int devno)
{
  FAR struct gps_upperhalf_s *upper = lower->priv;
  char path[PATH_MAX];

  sensor_unregister(&upper->dev[GPS_IDX].lower, devno);
  sensor_unregister(&upper->dev[GPS_SATELLITE_IDX].lower, devno);
  snprintf(path, PATH_MAX, GPS_PATH_FMT, devno);
  unregister_driver(path);
  nxsem_destroy(&upper->buffersem);
  circbuf_uninit(&upper->buffer);
  kmm_free(upper);
}

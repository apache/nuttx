/****************************************************************************
 * drivers/sensors/gnss_uorb.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/circbuf.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/gnss.h>
#include <nuttx/lib/lib.h>

#include <fcntl.h>
#include <poll.h>
#include <debug.h>
#include <minmea/minmea.h>
#include <sys/param.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GNSS_PATH_FMT          "/dev/ttyGNSS%d"

#define GNSS_PARSE_BUFFERSIZE  256

#define GNSS_KNOT_TO_KMH       1.852f
#define GNSS_KMH_TO_MPS        3.6f

#define GNSS_FLAG_GGA          (1 << 0)
#define GNSS_FLAG_RMC          (1 << 1)
#define GNSS_FLAG_MARK         (GNSS_FLAG_GGA | GNSS_FLAG_RMC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes GNSS sensor info */

struct gnss_sensor_s
{
  struct sensor_lowerhalf_s lower;
  FAR void                 *upper;
};

/* This structure describes user info of GNSS */

struct gnss_user_s
{
  struct list_node node;
  FAR struct pollfd *fds;
  size_t pos;
};

/* This structure describes the state of the upper half driver */

struct gnss_upperhalf_s
{
  struct gnss_sensor_s         dev[SENSOR_GNSS_IDX_GNSS_MAX];
  struct list_node             userlist;
  FAR struct gnss_lowerhalf_s *lower;
  uint8_t                      crefs;
  uint8_t                      flags;
  mutex_t                      lock;
  mutex_t                      bufferlock;
  sem_t                        buffersem;
  size_t                       parsenext;
  char                         parsebuffer[GNSS_PARSE_BUFFERSIZE];
  struct circbuf_s             buffer;
  struct sensor_gnss           gnss;
};

struct gnss_constellation_s
{
  FAR const char *prefix;
  int             id;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gnss_activate(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, bool enable);
static int gnss_set_interval(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR uint32_t *interval);
static int gnss_control(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, int cmd, unsigned long arg);

static int     gnss_open(FAR struct file *filep);
static int     gnss_close(FAR struct file *filep);
static ssize_t gnss_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t gnss_write(FAR struct file *filep,
                          FAR const char *buffer, size_t buflen);
static int     gnss_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
static int     gnss_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_gnss_sensor_ops =
{
  .activate     = gnss_activate,
  .set_interval = gnss_set_interval,
  .control      = gnss_control,
};

static const struct file_operations g_gnss_fops =
{
  gnss_open,   /* open  */
  gnss_close,  /* close */
  gnss_read,   /* read  */
  gnss_write,  /* write */
  NULL,        /* seek  */
  gnss_ioctl,  /* ioctl */
  NULL,        /* mmap */
  NULL,        /* truncate */
  gnss_poll    /* poll  */
};

static const struct gnss_constellation_s g_gnss_constellation[] =
{
  { "GP", SENSOR_GNSS_CONSTELLATION_GPS},
  { "GL", SENSOR_GNSS_CONSTELLATION_GLONASS},
  { "GQ", SENSOR_GNSS_CONSTELLATION_QZSS},
  { "GB", SENSOR_GNSS_CONSTELLATION_BEIDOU},
  { "BD", SENSOR_GNSS_CONSTELLATION_BEIDOU},
  { "GA", SENSOR_GNSS_CONSTELLATION_GALILEO},
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gnss_activate(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, bool enable)
{
  FAR struct gnss_sensor_s *dev = (FAR struct gnss_sensor_s *)lower;
  FAR struct gnss_upperhalf_s *upper = dev->upper;
  int ret = OK;

  nxmutex_lock(&upper->lock);
  if ((upper->crefs == UINT8_MAX && enable) ||
      (upper->crefs == 0 && !enable))
    {
      ret = -EINVAL;
    }
  else
    {
      if ((upper->crefs == 0 && enable) || (upper->crefs == 1 && !enable))
        {
          ret = upper->lower->ops->activate(upper->lower, filep, enable);
        }

      if (ret >= 0)
        {
          upper->crefs += enable ? 1 : -1;
        }
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

static int gnss_set_interval(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR uint32_t *interval)
{
  FAR struct gnss_sensor_s *dev = (FAR struct gnss_sensor_s *)lower;
  FAR struct gnss_upperhalf_s *upper = dev->upper;

  if (upper->lower->ops->set_interval == NULL)
    {
      return -ENOTTY;
    }

  return upper->lower->ops->set_interval(upper->lower, filep, interval);
}

static int gnss_control(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct gnss_sensor_s *dev = (FAR struct gnss_sensor_s *)lower;
  FAR struct gnss_upperhalf_s *upper = dev->upper;

  if (upper->lower->ops->control == NULL)
    {
      return -ENOTTY;
    }

  return upper->lower->ops->control(upper->lower, filep, cmd, arg);
}

static int gnss_open(FAR struct file *filep)
{
  FAR struct gnss_upperhalf_s *upper;
  FAR struct gnss_user_s *user;
  int ret = OK;

  upper = filep->f_inode->i_private;

  user = kmm_zalloc(sizeof(struct gnss_user_s));
  if (user == NULL)
    {
      return -ENOMEM;
    }

  nxmutex_lock(&upper->lock);
  if (upper->crefs >= UINT8_MAX)
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
  list_add_tail(&upper->userlist, &user->node);
  user->pos = upper->buffer.head;

out:
  nxmutex_unlock(&upper->lock);
  return ret;
}

static int gnss_close(FAR struct file *filep)
{
  FAR struct gnss_upperhalf_s *upper;
  FAR struct gnss_user_s *user;
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

  list_delete(&user->node);
  kmm_free(user);

out:
  nxmutex_unlock(&upper->lock);
  return ret;
}

static ssize_t gnss_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct gnss_upperhalf_s *upper;
  FAR struct gnss_user_s *user;
  ssize_t ret;

  if (buffer == NULL || buflen == 0)
    {
      return 0;
    }

  upper = filep->f_inode->i_private;
  user = filep->f_priv;

  nxmutex_lock(&upper->bufferlock);
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
          nxmutex_unlock(&upper->bufferlock);
          ret = nxsem_wait_uninterruptible(&upper->buffersem);
          if (ret < 0)
            {
              return ret;
            }

          nxmutex_lock(&upper->bufferlock);
          goto check;
        }
    }

  ret = circbuf_peekat(&upper->buffer, user->pos,
                       buffer, buflen);
  user->pos += ret;

out:
  nxmutex_unlock(&upper->bufferlock);
  return ret;
}

static ssize_t gnss_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct gnss_upperhalf_s *upper;
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

static int gnss_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct gnss_upperhalf_s *upper;
  int ret = -ENOTTY;

  upper = filep->f_inode->i_private;

  nxmutex_lock(&upper->lock);
  if (cmd == SNIOC_SET_INTERVAL)
    {
      if (upper->lower->ops->set_interval != NULL)
        {
          ret = upper->lower->ops->set_interval(upper->lower, filep,
                               (FAR uint32_t *)(uintptr_t)arg);
        }
    }
  else if (upper->lower->ops->control != NULL)
    {
      ret = upper->lower->ops->control(upper->lower, filep, cmd, arg);
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

static int gnss_poll(FAR struct file *filep, FAR struct pollfd *fds,
                     bool setup)
{
  FAR struct gnss_upperhalf_s *upper;
  FAR struct gnss_user_s *user;
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

static void gnss_init_data(FAR struct sensor_gnss *gnss)
{
  gnss->timestamp = ULONG_MAX;
  gnss->time_utc = ULONG_MAX;
  gnss->latitude = NAN;
  gnss->longitude = NAN;
  gnss->altitude = NAN;
  gnss->altitude_ellipsoid = NAN;
  gnss->eph = NAN;
  gnss->epv = NAN;
  gnss->hdop = NAN;
  gnss->pdop = NAN;
  gnss->vdop = NAN;
  gnss->ground_speed = NAN;
  gnss->course = NAN;
  gnss->satellites_used = UINT_MAX;
}

static void gnss_parse_nmea(FAR struct gnss_upperhalf_s *upper,
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
              upper->gnss.altitude = minmea_tofloat(&frame.altitude);
              upper->gnss.altitude_ellipsoid =
                             minmea_tofloat(&frame.height);
              upper->gnss.hdop = minmea_tofloat(&frame.hdop);
              upper->gnss.satellites_used = frame.satellites_tracked;
              upper->flags |= GNSS_FLAG_GGA;
            }

          break;
        }

      case MINMEA_SENTENCE_RMC:
        {
          struct minmea_sentence_rmc frame;
          struct tm t;

          if (minmea_parse_rmc(&frame, nmea))
            {
              upper->gnss.timestamp = sensor_get_timestamp();
              memset(&t, 0, sizeof(t));
              t.tm_sec = frame.time.seconds;
              t.tm_min = frame.time.minutes;
              t.tm_hour = frame.time.hours;
              t.tm_mday = frame.date.day;
              t.tm_mon = frame.date.month - 1;
              t.tm_year = frame.date.year + 2000 - 1900;
              t.tm_isdst = 0;
              upper->gnss.time_utc = mktime(&t);
              upper->gnss.latitude = minmea_tocoord(&frame.latitude);
              upper->gnss.longitude = minmea_tocoord(&frame.longitude);
              upper->gnss.ground_speed = minmea_tofloat(&frame.speed) *
                            GNSS_KNOT_TO_KMH / GNSS_KMH_TO_MPS;
              upper->gnss.course = minmea_tofloat(&frame.course);
              if (frame.valid)
                {
                  upper->flags |= GNSS_FLAG_RMC;
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
              upper->gnss.eph = sqrtf(lat_err + lon_err);
              upper->gnss.epv = minmea_tofloat(
                        &frame.altitude_error_deviation);
            }

          break;
        }

      case MINMEA_SENTENCE_GSV:
        {
          struct minmea_sentence_gsv frame;
          struct sensor_gnss_satellite satellite;
          size_t i;

          memset(&satellite, 0, sizeof(satellite));
          if (minmea_parse_gsv(&frame, nmea))
            {
              satellite.timestamp = sensor_get_timestamp();
              satellite.count = frame.total_msgs;
              satellite.satellites = frame.total_sats;
              memcpy(satellite.info, frame.sats,
                     sizeof(satellite.info[0]) * 4);
              lower = &upper->dev[SENSOR_GNSS_IDX_GNSS_SATELLITE].lower;

              for (i = 0; i < nitems(g_gnss_constellation); i++)
                {
                  if (!strncmp(g_gnss_constellation[i].prefix, nmea,
                               strlen(g_gnss_constellation[i].prefix)))
                    {
                      satellite.constellation = g_gnss_constellation[i].id;

                      if ((satellite.constellation ==
                           SENSOR_GNSS_CONSTELLATION_GPS) &&
                          (frame.msg_nr > 32))
                        {
                          satellite.constellation =
                           SENSOR_GNSS_CONSTELLATION_SBAS;
                        }

                      break;
                    }
                }

              lower->push_event(lower->priv, &satellite,
                                sizeof(satellite));
            }

          break;
        }

       default:
         break;
    }

  if (GNSS_FLAG_MARK == upper->flags)
    {
      upper->flags &= ~GNSS_FLAG_MARK;
      lower = &upper->dev[SENSOR_GNSS_IDX_GNSS].lower;
      lower->push_event(lower->priv, &upper->gnss, sizeof(upper->gnss));
      gnss_init_data(&upper->gnss);
    }
}

static void gnss_parse(FAR struct gnss_upperhalf_s *upper,
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
              if (upper->parsenext + 1 < GNSS_PARSE_BUFFERSIZE)
                {
                  upper->parsebuffer[upper->parsenext++] = *buffer;
                  continue;
                }

              upper->parsebuffer[upper->parsenext] = '\0';
              snerr("NMEA buffer overflow, invalid statement:%s\n",
                    upper->parsebuffer);
            }

          upper->parsebuffer[upper->parsenext] = '\0';
          gnss_parse_nmea(upper, upper->parsebuffer);
          upper->parsenext = 0;
          newline = false;
        }
    }
}

static void gnss_push_data(FAR void *priv, FAR const void *data,
                           size_t bytes, bool is_nmea)
{
  FAR struct gnss_upperhalf_s *upper = priv;
  FAR struct gnss_user_s *user;
  int semcount;

  if (data == NULL || bytes == 0)
    {
      return;
    }

  nxmutex_lock(&upper->bufferlock);
  if (is_nmea)
    {
      gnss_parse(upper, data, bytes);
    }

  circbuf_overwrite(&upper->buffer, data, bytes);

  list_for_every_entry(&upper->userlist, user, struct gnss_user_s, node)
    {
      poll_notify(&user->fds, 1, POLLIN);
    }

  nxmutex_unlock(&upper->bufferlock);

  nxsem_get_value(&upper->buffersem, &semcount);
  while (semcount++ <= 0)
    {
      nxsem_post(&upper->buffersem);
    }
}

static void gnss_push_event(FAR void *priv, FAR const void *data,
                            size_t bytes, int type)
{
  FAR struct gnss_upperhalf_s *upper = priv;
  FAR struct sensor_lowerhalf_s *lower;

  if (data == NULL || bytes == 0)
    {
      return;
    }

  if (type == SENSOR_TYPE_GNSS)
    {
      lower = &upper->dev[SENSOR_GNSS_IDX_GNSS].lower;
      lower->push_event(lower->priv, data, bytes);
    }
  else if (type == SENSOR_TYPE_GNSS_SATELLITE)
    {
      lower = &upper->dev[SENSOR_GNSS_IDX_GNSS_SATELLITE].lower;
      lower->push_event(lower->priv, data, bytes);
    }
  else if (type == SENSOR_TYPE_GNSS_MEASUREMENT)
    {
      lower = &upper->dev[SENSOR_GNSS_IDX_GNSS_MEASUREMENT].lower;
      lower->push_event(lower->priv, data, bytes);
    }
  else if (type == SENSOR_TYPE_GNSS_CLOCK)
    {
      lower = &upper->dev[SENSOR_GNSS_IDX_GNSS_CLOCK].lower;
      lower->push_event(lower->priv, data, bytes);
    }
  else if (type == SENSOR_TYPE_GNSS_GEOFENCE)
    {
      lower = &upper->dev[SENSOR_GNSS_IDX_GNSS_GEOFENCE].lower;
      lower->push_event(lower->priv, data, bytes);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gnss_register
 *
 * Description:
 *   This function binds an instance of a "lower half" GNSS driver with the
 *   "upper half" GNSS device and registers that device so that can be used
 *   by application code.
 *
 * Input Parameters:
 *   dev     - A pointer to an instance of lower half GNSS driver. This
 *             instance is bound to the GNSS driver and must persist as long
 *             as the driver persists.
 *   devno   - The user specifies which device of this type, from 0. If the
 *             devno alerady exists, -EEXIST will be returned.
 *   nbuffer - The number of events that the circular buffer can hold.
 *   count   - The array size of nbuffer.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int gnss_register(FAR struct gnss_lowerhalf_s *lower, int devno,
                  uint32_t nbuffer[], size_t count)
{
  FAR struct gnss_upperhalf_s *upper;
  FAR struct gnss_sensor_s *dev;
  FAR char *path;
  int ret;

  if (count != SENSOR_GNSS_IDX_GNSS_MAX)
    {
      return -EINVAL;
    }

  upper = kmm_zalloc(sizeof(struct gnss_upperhalf_s));
  if (upper == NULL)
    {
      return -ENOMEM;
    }

  path = lib_get_pathbuffer();
  if (path == NULL)
    {
      kmm_free(upper);
      return -ENOMEM;
    }

  lower->push_data = gnss_push_data;
  lower->push_event = gnss_push_event;
  lower->priv = upper;
  upper->lower = lower;

  nxmutex_init(&upper->lock);
  nxmutex_init(&upper->bufferlock);
  nxsem_init(&upper->buffersem, 0, 0);
  gnss_init_data(&upper->gnss);
  list_initialize(&upper->userlist);

  /* GNSS register */

  dev = &upper->dev[SENSOR_GNSS_IDX_GNSS];
  dev->lower.ops = &g_gnss_sensor_ops;
  dev->lower.type = SENSOR_TYPE_GNSS;
  dev->lower.nbuffer = nbuffer[SENSOR_GNSS_IDX_GNSS];
  dev->upper = upper;
  ret = sensor_register(&dev->lower, devno);
  if (ret < 0)
    {
      goto gnss_err;
    }

  /* Satellite register */

  dev = &upper->dev[SENSOR_GNSS_IDX_GNSS_SATELLITE];
  dev->lower.ops = &g_gnss_sensor_ops;
  dev->lower.type = SENSOR_TYPE_GNSS_SATELLITE;
  dev->lower.nbuffer = nbuffer[SENSOR_GNSS_IDX_GNSS_SATELLITE];
  dev->upper = upper;
  ret = sensor_register(&dev->lower, devno);
  if (ret < 0)
    {
      goto satellite_err;
    }

  /* GNSS Measurement register */

  dev = &upper->dev[SENSOR_GNSS_IDX_GNSS_MEASUREMENT];
  dev->lower.ops = &g_gnss_sensor_ops;
  dev->lower.type = SENSOR_TYPE_GNSS_MEASUREMENT;
  dev->lower.nbuffer = nbuffer[SENSOR_GNSS_IDX_GNSS_MEASUREMENT];
  dev->upper = upper;
  ret = sensor_register(&dev->lower, devno);
  if (ret < 0)
    {
      goto gnss_measurement_err;
    }

  /* GNSS Colck register */

  dev = &upper->dev[SENSOR_GNSS_IDX_GNSS_CLOCK];
  dev->lower.ops = &g_gnss_sensor_ops;
  dev->lower.type = SENSOR_TYPE_GNSS_CLOCK;
  dev->lower.nbuffer = nbuffer[SENSOR_GNSS_IDX_GNSS_CLOCK];
  dev->upper = upper;
  ret = sensor_register(&dev->lower, devno);
  if (ret < 0)
    {
      goto gnss_clock_err;
    }

  /* GNSS Geofence */

  dev = &upper->dev[SENSOR_GNSS_IDX_GNSS_GEOFENCE];
  dev->lower.ops = &g_gnss_sensor_ops;
  dev->lower.type = SENSOR_TYPE_GNSS_GEOFENCE;
  dev->lower.nbuffer = nbuffer[SENSOR_GNSS_IDX_GNSS_GEOFENCE];
  dev->upper = upper;
  ret = sensor_register(&dev->lower, devno);
  if (ret < 0)
    {
      goto gnss_geofence_err;
    }

  ret = circbuf_init(&upper->buffer, NULL,
                     CONFIG_SENSORS_GNSS_RECV_BUFFERSIZE);
  if (ret < 0)
    {
      goto circ_err;
    }

  snprintf(path, PATH_MAX, GNSS_PATH_FMT, devno);
  ret = register_driver(path, &g_gnss_fops, 0666, upper);
  if (ret < 0)
    {
      goto driver_err;
    }

  lib_put_pathbuffer(path);
  return ret;

driver_err:
  circbuf_uninit(&upper->buffer);
circ_err:
  sensor_unregister(&upper->dev[SENSOR_GNSS_IDX_GNSS_GEOFENCE].lower, devno);
gnss_geofence_err:
  sensor_unregister(&upper->dev[SENSOR_GNSS_IDX_GNSS_CLOCK].lower, devno);
gnss_clock_err:
  sensor_unregister(&upper->dev[SENSOR_GNSS_IDX_GNSS_MEASUREMENT].lower,
                    devno);
gnss_measurement_err:
  sensor_unregister(&upper->dev[SENSOR_GNSS_IDX_GNSS_SATELLITE].lower,
                    devno);
satellite_err:
  sensor_unregister(&upper->dev[SENSOR_GNSS_IDX_GNSS].lower, devno);
gnss_err:
  nxmutex_destroy(&upper->lock);
  nxmutex_destroy(&upper->bufferlock);
  nxsem_destroy(&upper->buffersem);
  lib_put_pathbuffer(path);
  kmm_free(upper);
  return ret;
}

/****************************************************************************
 * Name: gnss_unregister
 *
 * Description:
 *   This function unregisters character node and releases all resource from
 *   upper half driver. This API corresponds to the gnss_register.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half GNSS driver. This
 *           instance is bound to the GNSS driver and must persists as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0.
 *
 ****************************************************************************/

void gnss_unregister(FAR struct gnss_lowerhalf_s *lower, int devno)
{
  FAR struct gnss_upperhalf_s *upper = lower->priv;
  FAR char *path;

  path = lib_get_pathbuffer();
  if (path == NULL)
    {
      return;
    }

  sensor_unregister(&upper->dev[SENSOR_GNSS_IDX_GNSS].lower, devno);
  sensor_unregister(&upper->dev[SENSOR_GNSS_IDX_GNSS_SATELLITE].lower,
                    devno);
  sensor_unregister(&upper->dev[SENSOR_GNSS_IDX_GNSS_MEASUREMENT].lower,
                    devno);
  sensor_unregister(&upper->dev[SENSOR_GNSS_IDX_GNSS_CLOCK].lower, devno);
  sensor_unregister(&upper->dev[SENSOR_GNSS_IDX_GNSS_GEOFENCE].lower, devno);
  snprintf(path, PATH_MAX, GNSS_PATH_FMT, devno);
  unregister_driver(path);
  nxsem_destroy(&upper->buffersem);
  circbuf_uninit(&upper->buffer);
  lib_put_pathbuffer(path);
  kmm_free(upper);
}

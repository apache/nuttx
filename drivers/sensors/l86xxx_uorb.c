/****************************************************************************
 * drivers/sensors/l86xxx_uorb.c
 *
 * NOTE: EXPERIMENTAL DRIVER
 *
 * Contributed by Carleton University InSpace
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
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
#include <nuttx/nuttx.h>
#include <debug.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <termios.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>
#include <nuttx/sensors/sensor.h>
#include <minmea/minmea.h>

#include <nuttx/sensors/l86xxx.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SENSORS_L86_XXX_THREAD_STACKSIZE
#define CONFIG_SENSORS_L86_XXX_THREAD_STACKSIZE 10000
#endif

#ifndef CONFIG_L86_XXX_BAUD
#define CONFIG_L86_XXX_BAUD 9600
#endif

#if CONFIG_L86_XXX_BAUD == 4800 || CONFIG_L86_XXX_BAUD == 9600 || CONFIG_L86_XXX_BAUD == 14400 || CONFIG_L86_XXX_BAUD == 19200 || CONFIG_L86_XXX_BAUD == 38400 || CONFIG_L86_XXX_BAUD == 57600 || CONFIG_L86_XXX_BAUD == 115200
  #define L86_XXX_BAUD_RATE CONFIG_L86_XXX_BAUD
#else
  #error "Invalid baud rate. Supported baud rates are: 4800, 5600, 14400, 19200, 38400, 57600, 115200"
#endif

#if CONFIG_L86_XXX_FIX_INT < 100 || CONFIG_L86_XXX_FIX_INT > 10000
  #error "Invalid fix interval rate. Values must be between 100 and 10000"  
#endif

/* Helper to get array length */

#define MINMEA_MAX_LENGTH    256

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

/* GNSS device struct */

typedef struct
{
  FAR struct file uart;               /* UART interface */
  struct sensor_lowerhalf_s lower;    /* UORB lower-half */
  mutex_t devlock;                    /* Exclusive access */
  sem_t run;                          /* Start/stop collection thread */
  bool enabled;                       /* If module has started */
  char buffer[MINMEA_MAX_LENGTH];     /* Buffer for UART interface */
  int bufbytes;
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs; /* Number of open references */
#endif
} l86xxx_dev_s;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int l86xxx_control(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, int cmd, unsigned long arg);
static int l86xxx_activate(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, bool enable);
static int l86xxx_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR uint32_t *period_us);
#ifdef CONFIG_SERIAL_TERMIOS
static int set_baud_rate(l86xxx_dev_s *dev, int br);
#endif
static int send_command(l86xxx_dev_s *dev,
                          L86XXX_PMTK_COMMAND cmd, unsigned long arg);
static int read_line(l86xxx_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
  .control = l86xxx_control,
  .activate = l86xxx_activate,
  .set_interval = l86xxx_set_interval,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TERMIOS
/****************************************************************************
 * Name: set_baud_rate
 *
 * Description:
 *   Sets baud rate of UART interface
 *
 * Arguments:
 *    dev       -  Pointer L86-XXX priv struct
 *    br        -  Baud rate
 *
 * Returns:
 *  -EINVAL    - Invalid baud rate
 *  0          - Command succeeded, baud rate changed
 *  Other      - Error occurred during ioctl call
 ****************************************************************************/

static int set_baud_rate(l86xxx_dev_s *dev, int br)
{
  struct termios opt;
  int err;
  err = file_ioctl(&dev->uart, TCGETS, &opt);
  if (err < 0)
    {
      snwarn("Couldn't get interface settings: %d\n", err);
      return err;
    }

  cfmakeraw(&opt);

  switch (br)
    {
      case 4800:
      case 9600:
      case 14400:
      case 19200:
      case 38400:
      case 57600:
      case 115200:
      {
        cfsetispeed(&opt, br);
        cfsetospeed(&opt, br);
        break;
      }

      default:
      {
        snerr("Invalid baud rate, %ld\n", br);
        return -EINVAL;
      }
    }

  err = file_ioctl(&dev->uart, TCSETS, &opt);
  if (err < 0)
    {
      snerr("Couldn't change baud rate of U(S)ART interface: %d\n", err);
      return err;
    }

  /* These calls to read_line will flush out the buffer
  after the baud rate change
  */

  for (int i = 0; i < 5; ++i)
    {
      err = read_line(dev);
      if (err < 0)
        {
          snerr("Problem occurred when flushing buffer %d\n", err);
          return err;
        }
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: send_command
 *
 * Description:
 *   Sends command L86-XXX GNSS device and waits for acknowledgement
 *   if command supports it.
 *
 * Arguments:
 *    dev       -  Pointer L86-XXX priv struct
 *    cmd       -  L86XXX_COMMAND enum
 *    arg       -  Dependent on command type. Could be used for preset
 *                 enum, numeric args or struct pointers
 *
 * Returns:
 *  Flag defined by device
 *  -EINVAL - Invalid packet
 *  -ENOSYS - Unsupported packet type
 *  -EIO    - Valid packet, but action failed
 *  0       - Valid packet, action succeeded
 *  Other   - Command failed during writing
 ****************************************************************************/

static int send_command(l86xxx_dev_s *dev,
                          L86XXX_PMTK_COMMAND cmd, unsigned long arg)
{
  char buf[50];
  int bw1;
  int bw2;
  int err;
  int ret;
  uint8_t checksum;

  nxmutex_lock(&dev->devlock);
  switch (cmd)
  {
    case CMD_HOT_START:
    case CMD_WARM_START:
    case CMD_COLD_START:
    case CMD_FULL_COLD_START:
    {
      bw1 = snprintf(buf, sizeof(buf), "$PMTK%d", cmd);
      break;
    }

    case CMD_STANDBY_MODE:
    {
      bw1 = snprintf(buf, sizeof(buf), "$PMTK%d,0", cmd);
      break;
    }

    case SET_NMEA_BAUDRATE:
    {
      bw1 = snprintf(buf, sizeof(buf), "$PMTK%d,%d", cmd, (int)arg);
      break;
    }

    case SET_POS_FIX:
    {
      bw1 = snprintf(buf, sizeof(buf), "$PMTK%d,%d", cmd, (int)arg);
      break;
    }

    case FR_MODE:
    {
      bw1 = snprintf(buf, sizeof(buf), "$PMTK%d,%d", cmd, (int)arg);
      break;
    }

    default:
      return -ENOSYS;
  }

  sninfo("Sending command: %s to L86", buf);

  checksum = minmea_checksum(buf);
  bw2 = snprintf(buf + bw1, sizeof(buf) - bw1, "*%02X\r\n", checksum);

  err = file_write(&dev->uart, buf, bw1 + bw2);
  if (err < 0)
  {
    snerr("Could not send command to device\n");
    return err;
  }

  /* These commands do not send ACKs so just return after they've been
   * written
   */

  if (cmd == CMD_HOT_START ||
      cmd == CMD_WARM_START ||
      cmd == CMD_COLD_START ||
      cmd == CMD_FULL_COLD_START)
    {
      nxmutex_unlock(&dev->devlock);
      return 0;
    }

  /* Setting baud rate also doesn't send an ACK but the interface baud rate
   * needs to be updated
   */

  if (cmd == SET_NMEA_BAUDRATE)
  {
#ifdef CONFIG_SERIAL_TERMIOS
    nxsig_usleep(20000); /* Should wait for a bit before changing interface baud rate */
    ret = set_baud_rate(dev, (int)arg);
#else
    ret = -EINVAL;
#endif
    nxmutex_unlock(&dev->devlock);
    return ret;
  }

  /* Some commands will send ACKs,
   * wait for them here before unlocking the mutex
   */

  /* ACK message will be $PMTK001,<cmd num>,<flag> flag num indicates success
   * of command:
   *
   * 0 = Invalid packet
   * 1 = Unsupported packet type
   * 2 = Valid packet, but action failed
   * 3 = Valid packet, action succeeded
   */

  memset(buf, '\0', 50);
  snprintf(buf, 50, "$PMTK001,%d", cmd);
  sninfo("Waiting for ACK from L86...\n");
  read_line(dev);

  if (strncmp(buf, dev->buffer, strlen(buf)) == 0)
    {
      sninfo("ACK received!\n");
    }
  else
    {
      snerr("Did not get ACK!\n");
      nxmutex_unlock(&dev->devlock);
      return -EIO;
    }

  nxmutex_unlock(&dev->devlock);

  /* Flag num is always in position 13 of ack, subtract by '0'
  to obtain return val
  */

  ret = dev->buffer[13] - '0';
  if (ret == 1)
    {
      return -ENOSYS;
    }
  else if (ret == 2)
    {
      return -EIO;
    }
  else if (ret == 3)
    {
      return 0;
    }
  else
    {
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: read_line
 *
 * Description:
 *   Reads line from UART interface and stores in priv buffer
 *
 * Arguments:
 *    dev       -  Pointer L86-XXX priv struct
 *
 * Returns:
 *  Negative number if reading from device failed, else number
 *  of bytes read
 ****************************************************************************/

static int read_line(l86xxx_dev_s *dev)
{
  memset(dev->buffer, '\0', sizeof(dev->buffer));
  int line_len = 0;
  int err;
  char next_char;
  do
    {
      err = file_read(&dev->uart, &next_char, 1);
      if (err < 0)
        {
          snerr("Couldn't read from UART device: %d\n", err);
          return err;
        }

      if (next_char != '\r' && next_char != '\n')
        {
          dev->buffer[line_len++] = next_char;
        }
    }
  while (next_char != '\r' && next_char != '\n'
             && line_len < sizeof(dev->buffer));
  dev->buffer[line_len] = '\0';
  return line_len;
}

/****************************************************************************
 * Name: l86xxx_control
 *
 * Description:
 *   Send commands to the l86xxx GNSS module
 *
 * Returns:
 *   -ENOSYS if ioctl command is not supported,
 *   else return value from send_command
 ****************************************************************************/

static int l86xxx_control(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR l86xxx_dev_s *dev = container_of(lower, FAR l86xxx_dev_s, lower);
  L86XXX_PMTK_COMMAND pmtk_cmd;
  switch (cmd)
  {
    case SNIOC_HOT_START:
      pmtk_cmd = CMD_HOT_START;
      break;
    case SNIOC_WARM_START:
      pmtk_cmd = CMD_WARM_START;
      break;
    case SNIOC_COLD_START:
      pmtk_cmd = CMD_COLD_START;
      break;
    case SNIOC_FULL_COLD_START:
      pmtk_cmd = CMD_FULL_COLD_START;
      break;
    case SNIOC_SET_INTERVAL:
      pmtk_cmd = SET_POS_FIX;
      break;
    case SNIOC_SET_BAUD:
      pmtk_cmd = SET_NMEA_BAUDRATE;
      break;
    case SNIOC_SET_OPERATIONAL_MODE:
      if (arg == STANDBY)
        {
          pmtk_cmd = CMD_STANDBY_MODE;
        }
      else
        {
          pmtk_cmd = FR_MODE;
        }

      break;
    default:
      snerr("Unsupported command\n");
      return -ENOSYS;
  }

  return send_command(dev, pmtk_cmd, arg);
}

/****************************************************************************
 * Name: l86xxx_activate
 *
 * Description:
 *   Puts GNSS module into hot start mode or standby mode depending
 *   on args
 *
 * Returns:
 *   Return value from send_command
 ****************************************************************************/

static int l86xxx_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable)
{
  FAR l86xxx_dev_s *dev = container_of(lower, FAR l86xxx_dev_s, lower);

  /* If not already enabled, start gps */

  if (enable && !dev->enabled)
    {
      nxsem_post(&dev->run);
      dev->enabled = true;
      return send_command(dev, CMD_HOT_START, (int)NULL);
    }

  /* If not already disabled, send gps into standby mode */

  else if (!enable && dev->enabled)
    {
      dev->enabled = false;
      return send_command(dev, CMD_STANDBY_MODE, 0);
    }

  return 0;
}

/****************************************************************************
 * Name: l86xxx_set_interval
 *
 * Description:
 *   Set position fix interval of L86-XXX GNSS module
 *
 * Returns:
 *   -EINVAL if invalid interval, else return value from send_command
 ****************************************************************************/

static int l86xxx_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    FAR uint32_t *period_us)
{
  FAR l86xxx_dev_s *dev = container_of(lower, FAR l86xxx_dev_s, lower);
  int fix_interval = *period_us;
  if (fix_interval < 100 || fix_interval > 10000)
    {
      return -EINVAL;
    }

  int ret = send_command(dev, SET_POS_FIX, fix_interval);
  return ret;
}

/****************************************************************************
 * Name: l86xxx_thread
 *
 * Description:
 *   Kernel thread to poll the l86xxx
 ****************************************************************************/

static int l86xxx_thread(int argc, FAR char *argv[])
{
  FAR l86xxx_dev_s *dev =
      (FAR l86xxx_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  struct sensor_gnss gps;
  memset(&gps, 0, sizeof(gps));
  dev->enabled = true;
  int err;
  int bw;

  /* Read full line of NMEA output */

  for (; ; )
    {
      /* If the sensor is disabled
       * wait until enabled with activate function
       */

      if (!dev->enabled)
        {
          err = nxsem_wait(&dev->run);
          if (err < 0)
            {
              snerr("Couldn't wait on semaphore\n");
              continue;
            }
        }

      /* Mutex required because some commands send ACKS */

      err = nxmutex_lock(&dev->devlock);
      if (err < 0)
        {
          snerr("Couldn't lock mutex\n");
          return err;
        }

      bw = read_line(dev);

      /* Parse line based on NMEA sentence type */

      if (bw > 0)
        {
          switch (minmea_sentence_id(dev->buffer, false))
          {
            /* Time data is obtained from RMC sentence */

            case MINMEA_SENTENCE_RMC:
            {
              struct minmea_sentence_rmc frame;
              struct tm tm;
              if (minmea_check(dev->buffer, false) &&
                  minmea_parse_rmc(&frame, dev->buffer))
                {
                  gps.timestamp = sensor_get_timestamp();
                  minmea_getdatetime(&tm, &frame.date, &frame.time);
                  gps.time_utc = mktime(&tm);
                }
              break;
            }

            /* Velocity data is obtained from VTG sentence */

            case MINMEA_SENTENCE_VTG:
            {
              struct minmea_sentence_vtg frame;

              if (minmea_parse_vtg(&frame, dev->buffer))
                {
                  gps.ground_speed = minmea_tofloat(&frame.speed_kph) * 3.6; /* Convert speed in kph to mps */
                  gps.course = minmea_tofloat(&frame.true_track_degrees);
                }
              break;
            }

            /* 3D positional data is obtained from GGA sentence */

            case MINMEA_SENTENCE_GGA:
            {
              struct minmea_sentence_gga frame;

              if (minmea_parse_gga(&frame, dev->buffer))
                {
                  gps.latitude = minmea_tocoord(&frame.latitude);
                  gps.longitude = minmea_tocoord(&frame.longitude);
                  gps.altitude = minmea_tofloat(&frame.altitude);
                  gps.altitude_ellipsoid = minmea_tofloat(&frame.height);
                }
              break;
            }

            /* Precision dilution and satellite data is obtained from
            * GSA sentence
            */

            case MINMEA_SENTENCE_GSA:
            {
              struct minmea_sentence_gsa frame;

              if (minmea_parse_gsa(&frame, dev->buffer))
                {
                  gps.hdop = minmea_tofloat(&frame.hdop);
                  gps.pdop = minmea_tofloat(&frame.pdop);
                  gps.vdop = minmea_tofloat(&frame.vdop);
                  uint32_t sats = 0;
                  for (int i = 0; i < 12; ++i)
                    {
                      if (frame.sats[i] != 0)
                        {
                          ++sats;
                        }
                    }

                  gps.satellites_used = sats;
                }
              break;
            }

            /* GSV and GLL data are transmitted by the l86-XXX but do not
            provide additional information. Since GLL is always the last
            message transmitted, events will be pushed whenever that
            sentence is read
            */

            case MINMEA_SENTENCE_GLL:
            {
                dev->lower.push_event(dev->lower.priv, &gps, sizeof(gps));
            }

            /* All remaining sentences are not transmitted by the module */

            case MINMEA_SENTENCE_GSV:
            case MINMEA_SENTENCE_GBS:
            case MINMEA_SENTENCE_GST:
            case MINMEA_SENTENCE_ZDA:
            {
              break;
            }

            case MINMEA_INVALID:
            {
              snerr("Read invalid NMEA statement: %s\n", dev->buffer);
              break;
            }

            case MINMEA_UNKNOWN:
            {
              /* Message could be non-standard NMEA message,
              in that case just ignore
              */

              if (strncmp("$GPTXT", dev->buffer, strlen("$GPTXT")) == 0)
                {
                  break;
                }

              snwarn("Read unknown NMEA statement: %s %d\n",
                      dev->buffer, bw);
              break;
            }
          }
        }

      nxmutex_unlock(&dev->devlock);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: l86xxx_register
 *
 * Description:
 *   Register the L86-XXX GNSS device driver.
 *
 * Arguments:
 *    uartpath  -  The path to the UART character driver connected to the
 *                 transceiver
 *    devno     -  The device number to use for the topic (i.e. /dev/mag0)
 ****************************************************************************/

int l86xxx_register(FAR const char *uartpath, int devno)
{
  FAR l86xxx_dev_s *priv = NULL;
  int err;
  char *buf;
  FAR char *argv[2];
  char arg1[32];

  DEBUGASSERT(uartpath != NULL);

  /* Initialize device structure */

  priv = kmm_zalloc(sizeof(l86xxx_dev_s));
  if (priv == NULL)
    {
      snerr("Failed to allocate instance of L86-XXX driver.\n");
      return -ENOMEM;
    }

  memset(priv, 0, sizeof(l86xxx_dev_s));

  /* Initialize mutex */

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      snerr("Failed to initialize mutex for L86-XXX device: %d\n", err);
      goto free_mem;
    }

  /* Initialize semaphore */

  err = nxsem_init(&priv->run, 0, 0);
  if (err < 0)
    {
      snerr("Failed to register L86-XXX driver: %d\n", err);
      goto destroy_mutex;
    }

  /* Open UART interface for use */

  err = file_open(&priv->uart, uartpath, O_RDWR | O_CLOEXEC);
  if (err < 0)
    {
      wlerr("Failed to open UART interface %s for L86-XXX driver: %d\n",
            uartpath, err);
      goto destroy_sem;
    }

  /* Setup sensor with configured settings */

  sninfo("Waiting for GNSS to start...\n");

  buf = "$PMTK010,001*2E";
  read_line(priv);
  if (strncmp(buf, priv->buffer, strlen(buf)) == 0)
    {
      sninfo("GNSS module started.\n");
    }

#ifdef CONFIG_SERIAL_TERMIOS
  err = send_command(priv, SET_NMEA_BAUDRATE, L86_XXX_BAUD_RATE);
  if (err < 0)
    {
      snwarn("Couldn't set baud rate of device: %d\n", err);
    }
  #endif

  err = send_command(priv, SET_POS_FIX, CONFIG_L86_XXX_FIX_INT);
  if (err < 0)
    {
      snwarn("Couldn't set position fix interval, %d\n", err);
    }

  /* Register UORB Sensor */

  priv->lower.ops = &g_sensor_ops;
  priv->lower.type = SENSOR_TYPE_GNSS;

  err = sensor_register(&priv->lower, devno);
  if (err < 0)
    {
      snerr("Failed to register L86-XXX driver: %d\n", err);
      goto close_file;
    }

  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;
  err = kthread_create("l86xxx_thread", SCHED_PRIORITY_DEFAULT,
                      CONFIG_SENSORS_L86_XXX_THREAD_STACKSIZE,
                      l86xxx_thread, argv);

  if (err < 0)
    {
      snerr("Failed to create the l86xxx notification kthread\n");
      goto sensor_unreg;
    }

  sninfo("Registered L86-XXX driver with kernel polling thread"
          " with baud rate %d and update rate %d",
          L86_XXX_BAUD_RATE,
          CONFIG_L86_XXX_FIX_INT);

  /* Cleanup items on error */

  if (err < 0)
    {
    sensor_unreg:
      sensor_unregister(&priv->lower, devno);
    close_file:
      file_close(&priv->uart);
    destroy_sem:
      nxsem_destroy(&priv->run);
    destroy_mutex:
      nxmutex_destroy(&priv->devlock);
    free_mem:
      kmm_free(priv);
    }

  return err;
}

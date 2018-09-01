/****************************************************************************
 * include/nuttx/sensors/dhtxx.c
 *
 *   Copyright (C) 2018 Abdelatif GUETTOUCHE. All rights reserved.
 *   Author: Abdelatif GUETTOUCHE <abdelatif.guettouche@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/time.h>
#include <nuttx/clock.h>

#include <nuttx/sensors/dhtxx.h>

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DHTXX_START_SIGNAL_LOW_US         20000
#define DHTXX_START_SIGNAL_HIGH_US        100
#define DHTXX_RESPONSE_SIGNAL_US          85
#define DHTXX_TRANSMISSION_START_US       55
#define DHTXX_MIN_ZERO_DURATION           22
#define DHTXX_MAX_ZERO_DURATION           30
#define DHTXX_MIN_ONE_DURATION            40
#define DHTXX_MAX_ONE_DURATION            75
#define DHTXX_SAMPLING_PERIOD_S           2

#define DHTXX_RESPONSE_BITS               40U

#define DHT11_MIN_HUM                     20.0F
#define DHT11_MAX_HUM                     90.0F
#define DHT11_MIN_TEMP                    0.0F
#define DHT11_MAX_TEMP                    50.0F

#define DHT12_MIN_HUM                     20.0F
#define DHT12_MAX_HUM                     95.0F
#define DHT12_MIN_TEMP                   -20.0F
#define DHT12_MAX_TEMP                    60.0F

#define DHT22_MIN_HUM                     0.0F
#define DHT22_MAX_HUM                     100.0F
#define DHT22_MIN_TEMP                   -40.0F
#define DHT22_MAX_TEMP                    80.0F

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct dhtxx_dev_s
{
  FAR struct dhtxx_config_s *config;
  sem_t devsem;
  uint8_t raw_data[5];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void dht_standby_mode(FAR struct dhtxx_dev_s *priv);
static void dht_send_start_signal(FAR struct dhtxx_dev_s *priv);
static int  dht_prepare_reading(FAR struct dhtxx_dev_s *priv);
static int  dht_read_raw_data(FAR struct dhtxx_dev_s *priv);
static bool dht_verify_checksum(FAR struct dhtxx_dev_s *priv);
static bool dht_check_data(FAR struct dhtxx_sensor_data_s *data,
                           float min_hum, float max_hum,
                           float min_temp, float max_temp);
static int  dht_parse_data(FAR struct dhtxx_dev_s *priv, 
                          FAR struct dhtxx_sensor_data_s *data);

/* Character driver methods */

static int     dhtxx_open(FAR struct file *filep);
static int     dhtxx_close(FAR struct file *filep);
static ssize_t dhtxx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t dhtxx_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static int     dhtxx_ioctl(FAR struct file *filep, int cmd, 
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_dhtxxfops =
{
  dhtxx_open,   /* open */
  dhtxx_close,  /* close */
  dhtxx_read,   /* read */
  dhtxx_write,  /* write */
  NULL,         /* seek */
  dhtxx_ioctl   /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL        /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dht_standby_mode
 *
 * Description:
 *  Put the sensor in low power consumption (standby) mode.
 *
 ****************************************************************************/

static void dht_standby_mode(FAR struct dhtxx_dev_s *priv)
{
  priv->config->config_data_pin(priv->config, false);
  priv->config->set_data_pin(priv->config, true);  
}

/****************************************************************************
 * Name: dht_send_start_signal
 *
 * Description:
 *  Send a start signal to the sensor.
 *
 ****************************************************************************/

static void dht_send_start_signal(FAR struct dhtxx_dev_s *priv)
{
  int64_t start_time;
  int64_t current_time;

  priv->config->config_data_pin(priv->config, false);

  priv->config->set_data_pin(priv->config, false);
  start_time = priv->config->get_clock(priv->config);
  while (1)
    {
      current_time = priv->config->get_clock(priv->config);
      if (current_time - start_time >= DHTXX_START_SIGNAL_LOW_US)
        {
          break;
        }
    }

  priv->config->set_data_pin(priv->config, true);
  start_time = priv->config->get_clock(priv->config);
  while (1)
    {
      current_time = priv->config->get_clock(priv->config);
      if (current_time - start_time >= DHTXX_START_SIGNAL_HIGH_US)
        {
          break;
        }
    }
}

/****************************************************************************
 * Name: dht_prepare_reading
 *
 * Description:
 *  Gets the sensor ready for transmitting data.
 *
 ****************************************************************************/

static int dht_prepare_reading(FAR struct dhtxx_dev_s *priv)
{
  int64_t start_time;
  int64_t current_time;

  /* Prepare for reading data. */

  priv->config->config_data_pin(priv->config, true);

  /* The DHT will clear the data pin for about 80uS. */

  start_time = priv->config->get_clock(priv->config);
  while (!priv->config->read_data_pin(priv->config))
    {
      current_time = priv->config->get_clock(priv->config);
      if (current_time - start_time > DHTXX_RESPONSE_SIGNAL_US)
        {
          return -1;
        }
    }

  /* The DHT will set the data pin for about 80uS. */

  while (priv->config->read_data_pin(priv->config))
    {
      current_time = priv->config->get_clock(priv->config);
      if (current_time - start_time > DHTXX_RESPONSE_SIGNAL_US)
        {
          return -1;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: dht_read_raw_data
 *
 * Description:
 *  Reads raw data from the sensor.
 *
 ****************************************************************************/

static int dht_read_raw_data(FAR struct dhtxx_dev_s *priv)
{
  int64_t start_time;
  int64_t end_time;
  int64_t current_time;
  uint8_t j, i;

  j = 0U;
  for (i = 0U; i < DHTXX_RESPONSE_BITS; i++)
    {
      /* Start of transmission begins with a ~50uS low. */

      start_time = priv->config->get_clock(priv->config);
      while (!priv->config->read_data_pin(priv->config))
        {
          current_time = priv->config->get_clock(priv->config);
          if (current_time - start_time > DHTXX_TRANSMISSION_START_US)
            {
              return -1;
            }
        }

      /* Get start time. */

      start_time = priv->config->get_clock(priv->config);
      while (priv->config->read_data_pin(priv->config))
        {
          current_time = priv->config->get_clock(priv->config);
          if (current_time - start_time > DHTXX_MAX_ONE_DURATION)
            {
              return -1;
            }
        }

      end_time = priv->config->get_clock(priv->config);
      if (end_time - start_time >= DHTXX_MIN_ONE_DURATION)
        {
          priv->raw_data[j] = (priv->raw_data[j] << 1U) | 1U;
        }
      else
        {
          priv->raw_data[j] = priv->raw_data[j] << 1U;
        }

      if (i % 8U == 7U)
        {
          j++;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: dht_verify_checksum
 *
 * Description:
 *  Verify the sent checksum with the calculated checksum.
 *
 ****************************************************************************/

static bool dht_verify_checksum(FAR struct dhtxx_dev_s *priv)
{
  uint8_t sum;

  sum = (priv->raw_data[0] + priv->raw_data[1] + 
         priv->raw_data[2] + priv->raw_data[3]) & 0xFFU;

  return (sum == priv->raw_data[4]);
}

/****************************************************************************
 * Name: dht_check_data
 *
 * Description:
 *  Check for data integrity in regards with the sensors range.
 *
 ****************************************************************************/

static bool dht_check_data(FAR struct dhtxx_sensor_data_s *data,
                           float min_hum, float max_hum,
                           float min_temp, float max_temp)
{
  if (data->hum < min_hum || data->hum > max_hum)
    {
      return false;
    }

  if (data->temp < min_temp || data->temp > max_temp)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: dht_check_data
 *
 * Description:
 *  Get the humidity and temperature's values from raw data.
 *
 ****************************************************************************/

static int dht_parse_data(FAR struct dhtxx_dev_s *priv, 
                          FAR struct dhtxx_sensor_data_s *data)
{
  int ret = OK;

  switch (priv->config->type)
  {
    case DHTXX_DHT11:
      data->hum  = priv->raw_data[0];
      data->temp = priv->raw_data[2];

      /* if data is not within sensor's measurement range,
       * an error must have accured.
       */

      if (!dht_check_data(data, DHT11_MIN_HUM, DHT11_MAX_HUM,
                        DHT11_MIN_TEMP, DHT11_MAX_TEMP))
        {
          ret = -1;
        }
      
    break;

    case DHTXX_DHT12:
      data->hum  = priv->raw_data[0] + priv->raw_data[1] * 0.1F;

      data->temp = priv->raw_data[2] + (priv->raw_data[3] & 0x7FU) * 0.1F;
      if (priv->raw_data[3] & 0x80U)
        {
          data->temp *= -1;
        }

      if (!dht_check_data(data, DHT12_MIN_HUM, DHT12_MAX_HUM,
                         DHT12_MIN_TEMP, DHT12_MAX_TEMP))
        {
          ret = -1;
        }

    break;

    case DHTXX_DHT21:
    case DHTXX_DHT22:
    case DHTXX_DHT33:
    case DHTXX_DHT44:
      data->hum  = (priv->raw_data[0] << 8U | priv->raw_data[1]) * 0.1F;

      data->temp = (((priv->raw_data[2] & 0x7FU) << 8U) | 
                    priv->raw_data[3]) * 0.1F;
      if (priv->raw_data[2] & 0x80U)
        {
          data->temp *= -1;
        }

      if (!dht_check_data(data, DHT22_MIN_HUM, DHT22_MAX_HUM,
                          DHT22_MIN_TEMP, DHT22_MAX_TEMP))
        {
          ret = -1;
        }

    break;
  }

  return ret;
}

/****************************************************************************
 * Name: dhtxx_open
 *
 * Description:
 *   This function is called whenever the Dhtxx device is opened.
 *
 ****************************************************************************/

static int dhtxx_open(FAR struct file *filep)
{
  int ret;
  FAR struct inode        *inode = filep->f_inode;
  FAR struct dhtxx_dev_s  *priv  = inode->i_private;

  /* Acquire the semaphore, wait the sampling time before sending anything to
   * pass unstable state.
   */

  do
    {
      ret = nxsem_wait(&priv->devsem);
    }
  while (ret == -EINTR);

  dht_standby_mode(priv);

  nxsig_sleep(DHTXX_SAMPLING_PERIOD_S);

  /* Sensor ready. */

  nxsem_post(&priv->devsem);
  return OK;
}

/****************************************************************************
 * Name: dhtxx_close
 *
 * Description:
 *   This routine is called when the Dhtxx device is closed.
 *
 ****************************************************************************/

static int dhtxx_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: dhtxx_read
 ****************************************************************************/

static ssize_t dhtxx_read(FAR struct file *filep, FAR char *buffer, 
                         size_t buflen)
{
  int ret;
  FAR struct inode                *inode = filep->f_inode;
  FAR struct dhtxx_dev_s          *priv  = inode->i_private;
  FAR struct dhtxx_sensor_data_s  *data  = 
             (FAR struct dhtxx_sensor_data_s *)buffer;
  
  if (!buffer)
    {
      snerr("ERROR: Buffer is null.\n");
      return -1;
    }

  if (buflen < sizeof(FAR struct dhtxx_sensor_data_s))
    {
      snerr("ERROR: Not enough memory for reading out a sensor data sample.\n");
      return -ENOSYS;
    }

  memset(priv->raw_data, 0U, sizeof(priv->raw_data));

  do
    {
      ret = nxsem_wait(&priv->devsem);
    }
  while (ret == -EINTR);

  dht_send_start_signal(priv);

  if (dht_prepare_reading(priv) != 0)
    {
      data->status = DHTXX_TIMEOUT;
      ret = -1;
      goto out;
    }

  if (dht_read_raw_data(priv) != 0)
    {
      data->status = DHTXX_TIMEOUT;
      ret = -1;
      goto out;
    }

  if (!dht_verify_checksum(priv))
    {
      data->status = DHTXX_CHECKSUM_ERROR;
      ret = -1;
      goto out;
    }

  if (dht_parse_data(priv, data) != 0)
    {
      data->status = DHTXX_READ_ERROR;
      ret = -1;
    }
  else
    {
      data->status = DHTXX_SUCCESS;
    }

out:
  /* Done reading, set the sensor back to low mode. */

  dht_standby_mode(priv);

  /* Don't release the semaphore just yet.  The sensor needs time between
   * consecutive readings.
   */

  nxsig_sleep(DHTXX_SAMPLING_PERIOD_S);

  /* Sensor ready for new reading */

  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: dhtxx_write
 ****************************************************************************/

static ssize_t dhtxx_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: dhtxx_ioctl
 ****************************************************************************/

static int dhtxx_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      /* Command was not recognized */

    default:
      snerr("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dhtxx_register
 *
 * Description:
 *   Register the Dhtxx character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/dht0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int dhtxx_register(FAR const char *devpath, FAR struct dhtxx_config_s *config)
{
  FAR struct dhtxx_dev_s *priv;
  int ret;

  /* Initialize the Dhtxx device structure */

  priv = (FAR struct dhtxx_dev_s *)kmm_malloc(sizeof(struct dhtxx_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }
  priv->config = config;

  /* Register the character driver */

  ret = register_driver(devpath, &g_dhtxxfops, 0666, priv);
  if (ret < 0)
    {
      kmm_free(priv);
      snerr("ERROR: Failed to register driver: %d\n", ret);
    }

  return ret;
}

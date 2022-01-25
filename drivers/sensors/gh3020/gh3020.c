/****************************************************************************
 * drivers/sensors/gh3020/gh3020.c
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

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <math.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched_note.h>
#include <nuttx/sensors/gh3020.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/wqueue.h>
#include <sys/types.h>

#include "gh3020_bridge.h"
#include "gh3x2x_drv.h"
#include "gh3x2x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configurations */

#define GH3020_BATCH_NUMBER      12         /* Maximum slots for each PPG */

/* SPI parameters */

#define GH3020_SPI_FREQ_MAX      8000000    /* Maximum SPI CLK = 8MHz */
#define GH3020_SPI_FREQ_MIN      1000000    /* Minimum SPI CLK = 1MHz */
#define GH3020_SPI_NBITS         8          /* Bit per word on SPI */

/* Misc parameters */

#define GH3020_SAMPLERATE_MAX    1000.0f    /* Maximum sample rate = 1kHz */
#define GH3020_SAMPLERATE_MIN    25.0f      /* Minimum sample rate = 25Hz */
#define GH3020_DATA_PER_SAMPLE   4          /* Each sample has 4 PD data */
#define GH3020_RDMODE_INTERRPT   0          /* Read data in interrupts */
#define GH3020_RDMODE_POLLING    1          /* Read data with polling */

/* Control commands */

#define GH3020_CTRL_CHECKID      0          /* Check device ID. */
#define GH3020_CTRL_LED_CURRENT  0x90       /* Set LED driver current */
#define GH3020_CTRL_OPEN_FACTEST 0x91       /* Enter factory test mode */
#define GH3020_CTRL_EXIT_FACTEST 0x92       /* Exit factory test mode */

/* Default settings */

#define GH3020_INTVL_DFT         40000      /* Default interval = 40 ms */
#define GH3020_SR_DFT            25         /* Default sample rate = 25 Hz */
#define GH3020_CURRENT_DFT       10000      /* Default LED current = 10mA */

#define GH3020_ONE_SECOND        1000000.0f /* 1 second = 1000000 us */

#define GH3020_CEILING(x, y)     ((x) + ((y)- 1)) / (y)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Sensor struct */

struct gh3020_sensor_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct sensor_lowerhalf_s lower;       /* Lower half sensor driver */
  FAR struct gh3020_dev_s *dev;          /* Pointer to the device struct */
  uint32_t interval;                     /* Sample interval (us) */
  uint32_t batch;                        /* Batch latency (us) */
  uint32_t current;                      /* LED driver current (uA) */
  uint8_t chidx;                         /* PPG channel index */
  bool activated;                        /* If it's activated now. */
  bool activating;                       /* If it will be activated later */
  bool inactivating;                     /* If it will be inactivated later */
};

/* Device struct */

struct gh3020_dev_s
{
  /* PPG sensors structures. */

  struct gh3020_sensor_s sensor[GH3020_SENSOR_NUM];
  uint64_t timestamp;                    /* Current timestamp (us) */
  struct work_s work_intrpt;             /* Interrupt worker */
  struct work_s work_poll;               /* Polling worker */

  /* Device configuration struct pointer. */

  FAR const struct gh3020_config_s *config;

  /* Buffer for pushing PPG data. */

  struct sensor_event_ppgq ppgdata[GH3020_SENSOR_NUM][GH3020_BATCH_NUMBER];
  uint32_t batch;                        /* Common batch(us) for interrupts */
  uint32_t interval;                     /* Common interval(us) for polling */
  uint32_t intvl_prev;                   /* Previous common interval(us) */
  uint32_t channelmode;                  /* PPG channels status mode */
  uint16_t fifowtm;                      /* FIFO water marker */
  uint8_t ppgdatacnt[GH3020_SENSOR_NUM]; /* Data number of each PPG channel */
  uint8_t activated;                     /* How many sensors are activated */
  bool factest_mode;                     /* If it's in factory test mode */
  bool updating;                         /* If any sensor need updating */
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* GH3020 common handle functions */

static int gh3020_configspi(FAR struct gh3020_dev_s *priv);
static uint16_t gh3020_calcu_fifowtm(FAR struct gh3020_dev_s *priv);
static void gh3x2x_factest_start(uint32_t channelmode, uint32_t current);
static void gh3020_push_data(FAR struct gh3020_dev_s *priv);
static void gh3020_restart_new_fifowtm(FAR struct gh3020_dev_s *priv,
                                       uint16_t fifowtm);
static void gh3020_switch_poll2intrpt(FAR struct gh3020_dev_s *priv);
static void gh3020_switch_intrpt2poll(FAR struct gh3020_dev_s *priv);
static void gh3020_update_sensor(FAR struct gh3020_dev_s *priv);

/* Sensor ops functions */

static int gh3020_activate(FAR struct sensor_lowerhalf_s *lower,
                           bool enable);
static int gh3020_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR unsigned int *period_us);
static int gh3020_batch(FAR struct sensor_lowerhalf_s *lower,
                        FAR unsigned int *latency_us);
static int gh3020_selftest(FAR struct sensor_lowerhalf_s *lower,
                           unsigned long arg);
static int gh3020_control(FAR struct sensor_lowerhalf_s *lower, int cmd,
                          unsigned long arg);

/* Sensor interrupt/polling functions */

static int gh3020_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset, FAR void *arg);
static void gh3020_worker_intrpt(FAR void *arg);
static void gh3020_worker_poll(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Local storage for device struct pointer. */

FAR static struct gh3020_dev_s *g_priv;

/* The function code of each PPG channel. */

static const uint32_t gh3020_channel_function_list[GH3020_SENSOR_NUM] =
  {
    GH3X2X_FUNCTION_HR,                  /* HR function for ch0 (green) */
    GH3X2X_FUNCTION_SPO2,                /* SPO2 function for ch1 (red) */
    GH3X2X_FUNCTION_HRV,                 /* HRV function for ch2 (IR) */
    GH3X2X_FUNCTION_RESP                 /* RESP function for ch3 (dark) */
  };

/* Sensor operations */

static const struct sensor_ops_s g_gh3020_ops =
{
  .activate     = gh3020_activate,
  .set_interval = gh3020_set_interval,
  .batch        = gh3020_batch,
  .selftest     = gh3020_selftest,
  .control      = gh3020_control,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gh3020_configspi
 *
 * Description:
 *   Set SPI mode, frequency and bits per word, according to gh3020_dev_s->
 *   gh3020_config_s.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_configspi(FAR struct gh3020_dev_s *priv)
{
  int freq = priv->config->freq;

  /* Set SPI frequency in an acceptable range. */

  if (freq > GH3020_SPI_FREQ_MAX)
    {
      freq = GH3020_SPI_FREQ_MAX;
    }
  else if (freq < GH3020_SPI_FREQ_MIN)
    {
      freq = GH3020_SPI_FREQ_MIN;
    }

  if (SPI_SETFREQUENCY(priv->config->spi, freq) != freq)
    {
      return -EIO;
    }

  /* GH3020 SPI supports default mode0. If one must used mode3, GH3020 can
   * set SPI in mode3 under mode0 first.
   */

  SPI_SETMODE(priv->config->spi, SPIDEV_MODE0);

  /* Set number of bits per word. */

  SPI_SETBITS(priv->config->spi, GH3020_SPI_NBITS);

  return OK;
}

/****************************************************************************
 * Name: gh3020_calcu_fifowtm
 *
 * Description:
 *   Calculate FIFO watermark according to the common minium batch latency
 *   and each channel's sample interval.
 *
 * Input Parameters:
 *   priv - The device struct pointer.
 *
 * Returned Value:
 *   If there's no activated channel, return 0.
 *   If the common minium batch latency is 0, polling mode is used, return 0.
 *   Otherwise return calculated FIFO watermark (uints in data, each sample
 *   has 4 data from 4 PD).
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static uint16_t gh3020_calcu_fifowtm(FAR struct gh3020_dev_s *priv)
{
  uint32_t batch_min = 0xffffffff;
  uint16_t fifowtm = 0;
  uint8_t idx;

  /* Search minium batch latency among all activated PPG channel. */

  for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
    {
      if (priv->sensor[idx].activated == true &&
          priv->sensor[idx].batch < batch_min)
        {
          batch_min = priv->sensor[idx].batch;
        }
    }

  /* If any PPG is activated, update the batch in device struct */

  if (batch_min != 0xffffffff)
    {
      priv->batch = batch_min;

      /* If minium batch > 0, reading will use FIFO watermark interrupt. */

      if (batch_min > 0)
        {
          for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
            {
              if (priv->sensor[idx].activated == true)
                {
                  /* Device's watermark is sum of all components. */

                  fifowtm = fifowtm + batch_min / priv->sensor[idx].interval;
                }
            }

          /* Each sample has 4 PD data */

          fifowtm = fifowtm * GH3020_DATA_PER_SAMPLE;
        }
    }

  return fifowtm;
}

/****************************************************************************
 * Name: gh3020_factest_start
 *
 * Description:
 *   GH3020 start sample in factest mode with specialized LED color and
 *   driver current.
 *
 * Input Parameters:
 *   channelmode - The PPG channel want to test.
 *   current     - LED driver current.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3x2x_factest_start(uint32_t channelmode, uint32_t current)
{
  STGh3x2xEngineeringModeSampleParam sample_para;

  memset(&sample_para, 0, sizeof(sample_para));
  memset(&sample_para.uchTiaGain[4],
    GH3X2X_ENGINEERING_MODE_TIA_GAIN_100_K, 12);
  sample_para.unFunctionID = GH3X2X_FUNCTION_TEST1;
  sample_para.uchTiaGainChangeEn = 1;
  sample_para.uchSampleRateChangeEn = 1;
  sample_para.uchIntTimeChangeEn = 1;
  sample_para.uchLedCurrentChangeEn = 1;
  sample_para.usSampleRate = GH3020_SR_DFT;
  sample_para.uchIntTime = GH3X2X_ENGINEERING_MODE_INT_TIME_79_US;

  switch (channelmode)
    {
      case GH3X2X_FUNCTION_HR:    /* Green */
        {
          sample_para.uchLedDrv0Current[7] = (uint8_t)(current / 2000);
          sample_para.uchLedDrv1Current[7] = (uint8_t)(current / 2000);
        }
        break;

      case GH3X2X_FUNCTION_SPO2:  /* Red */
        {
          sample_para.uchLedDrv0Current[11] = (uint8_t)(current / 2000);
          sample_para.uchLedDrv1Current[11] = (uint8_t)(current / 2000);
        }
        break;

      case GH3X2X_FUNCTION_HRV:   /* IR */
        {
          sample_para.uchLedDrv0Current[15] = (uint8_t)(current / 2000);
          sample_para.uchLedDrv1Current[15] = (uint8_t)(current / 2000);
        }
        break;

      case GH3X2X_FUNCTION_RESP:  /* Dark */
          break;

      default:
        {
          snerr("Invalid PPG channel mode.\n");
          return;
        }
  }

  gh3020_start_sampling_factest(GH3X2X_FUNCTION_TEST1, &sample_para, 1);
}

/****************************************************************************
 * Name: gh3020_push_data
 *
 * Description:
 *   Push activated PPG channel's data.
 *
 * Input Parameters:
 *   priv - The device struct pointer.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_push_data(FAR struct gh3020_dev_s *priv)
{
  uint8_t idx;
  uint8_t j;

  /* Calculate timestamp for each sample and push data for each channel. */

  for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
    {
      if (priv->ppgdatacnt[idx] > 0 && priv->sensor[idx].activated == true)
        {
          for (j = 0; j < priv->ppgdatacnt[idx]; j++)
            {
              priv->ppgdata[idx][j].timestamp = priv->timestamp -
                (priv->ppgdatacnt[idx] - 1 - j) * priv->sensor[idx].interval;
            }

          priv->sensor[idx].lower.push_event(priv->sensor[idx].lower.priv,
                                             priv->ppgdata[idx],
                                             sizeof(struct sensor_event_ppgq)
                                             * priv->ppgdatacnt[idx]);
        }
    }
}

/****************************************************************************
 * Name: gh3020_restart_new_fifowtm
 *
 * Description:
 *   Congifure the GH3020 with new FIFO watermark and start it. Reading mode
 *   will alse change if needed.
 *
 * Input Parameters:
 *   priv    - The device struct pointer.
 *   fifowtm - New FIFO watermark.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   GH3020 has been stopped or has not been started.
 *
 ****************************************************************************/

static void gh3020_restart_new_fifowtm(FAR struct gh3020_dev_s *priv,
                                       uint16_t fifowtm)
{
  uint8_t idx;

  /* Either old or new FIFO watermark is 0 while the other one is not 0,
   * reading mode should switch between interrupt mode and polling mode.
   */

  if ((fifowtm > 0 && priv->fifowtm == 0) ||
      (fifowtm == 0 && priv->fifowtm > 0))
    {
      /* GH3020 changes into corresponding reading mode. */

      if (fifowtm > 0)
        {
          gh3020_rdmode_switch(GH3020_RDMODE_INTERRPT);
        }
      else
        {
          gh3020_rdmode_switch(GH3020_RDMODE_POLLING);
        }

      /* After switch reading mode, GH3020 must be initialized again. Then
       * set each activated or to-be-activated channel's sample rate again.
       */

      gh3020_init();
      for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
        {
          if (priv->sensor[idx].activated)
            {
              gh3020_samplerate_set(gh3020_channel_function_list[idx],
                (uint16_t)
                (GH3020_ONE_SECOND / priv->sensor[idx].interval));
            }
        }

      /* Set FIFO watermark if not 0, then restart sampling. */

      if (fifowtm > 0)
        {
          gh3020_set_fifowtm(fifowtm);
        }

      gh3020_start_sampling(priv->channelmode);

      /* Switch MCU's reading mode between polling and interrupt. */

      if (fifowtm > 0)
        {
          gh3020_switch_poll2intrpt(priv);
        }
      else
        {
          gh3020_switch_intrpt2poll(priv);
        }
    }
  else
    {
      /* GH3020 remains interrupt/polling, only watermark shall be changed. */

      if (fifowtm > 0 && fifowtm != priv->fifowtm)
        {
          gh3020_set_fifowtm(fifowtm);
        }

      gh3020_start_sampling(priv->channelmode);
    }

  sched_note_printf("GH3020 starts, fifowtm=%u", fifowtm);
  priv->fifowtm = fifowtm;
}

/****************************************************************************
 * Name: gh3020_switch_poll2intrpt
 *
 * Description:
 *   Reading mode swtiched from polling to interrupt.
 *
 * Input Parameters:
 *   priv - The device struct pointer.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_switch_poll2intrpt(FAR struct gh3020_dev_s *priv)
{
  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_RISING);
  work_cancel(HPWORK, &priv->work_poll);
}

/****************************************************************************
 * Name: gh3020_switch_intrpt2poll
 *
 * Description:
 *   Reading mode swtiched from interrupt to polling.
 *
 * Input Parameters:
 *   priv - The device struct pointer.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_switch_intrpt2poll(FAR struct gh3020_dev_s *priv)
{
  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);
  work_queue(HPWORK, &priv->work_poll, gh3020_worker_poll, priv,
             priv->interval / USEC_PER_TICK);
}

/****************************************************************************
 * Name: gh3020_update_sensor
 *
 * Description:
 *   Update sensors' activating status after FIFO reading.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_update_sensor(FAR struct gh3020_dev_s *priv)
{
  FAR struct gh3020_sensor_s *sensor;
  uint32_t interval_min;
  uint16_t fifowtm;
  uint16_t rate;
  uint8_t idx;

  if (priv->updating == true)
    {
      gh3020_stop_sampling(priv->channelmode);
      priv->updating = false;
      priv->activated = 0;
      priv->channelmode = 0;

      /* Check each sensor, add activating ones, remove inactivating ones. */

      for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
        {
          sensor = &priv->sensor[idx];
          if (sensor->activating == true || (sensor->activated == true &&
              sensor->inactivating == false))
            {
              rate = (uint16_t)(GH3020_ONE_SECOND / sensor->interval);
              priv->channelmode = priv->channelmode |
                                  gh3020_channel_function_list[idx];
              gh3020_samplerate_set(
                    gh3020_channel_function_list[idx], rate);
              if (sensor->activating == true)
                {
                  sched_note_printf("activate ppgq%u, rate=%u", idx, rate);
                }

              sensor->activating = false;
              sensor->activated = true;
              priv->activated++;
            }
          else if (sensor->inactivating == true)
            {
              sensor->inactivating = false;
              sensor->activated = false;
              sched_note_printf("inactivate ppgq%u", idx);
            }
        }

      /* If GH3020 shall restart because of some activated channels */

      if (priv->activated > 0)
        {
          fifowtm = gh3020_calcu_fifowtm(priv);

          /* Seek for minium sample interval. */

          interval_min = 0xffffffff;
          for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
            {
              if (priv->sensor[idx].activated == true &&
                  priv->sensor[idx].interval < interval_min)
                {
                  interval_min = priv->sensor[idx].interval;
                }
            }

          priv->interval = interval_min;
          gh3020_restart_new_fifowtm(priv, fifowtm);
        }

      /* Otherwise GH3020 will not start as no channel is activated */

      else
        {
          if (priv->fifowtm > 0)
            {
              IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                              IOEXPANDER_OPTION_INTCFG,
                              (FAR void *)IOEXPANDER_VAL_DISABLE);
            }
          else
            {
              work_cancel(HPWORK, &priv->work_poll);
            }

          sched_note_printf("GH3020 stops");
        }
    }
}

/****************************************************************************
 * Name: gh3020_activate
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   lower  - The instance of lower half sensor driver
 *   enable - true(enable) and false(disable)
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_activate(FAR struct sensor_lowerhalf_s *lower, bool enable)
{
  FAR struct gh3020_sensor_s *sensor = (FAR struct gh3020_sensor_s *)lower;
  FAR struct gh3020_dev_s *priv;
  uint16_t rate;

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL);

  priv = sensor->dev;

  /* Operate only if the activated status will change. */

  if (sensor->activated != enable)
    {
      sensor->activated = enable;
      if (priv->factest_mode == true)
        {
          if (enable == true)
            {
              gh3020_rdmode_switch(GH3020_RDMODE_POLLING);
              gh3020_init();
              gh3x2x_factest_start(
                gh3020_channel_function_list[sensor->chidx],
                sensor->current);
              priv->activated++;
              priv->intvl_prev = priv->interval;
              priv->interval = GH3020_INTVL_DFT;
              work_queue(HPWORK, &priv->work_poll, gh3020_worker_poll, priv,
                         priv->interval / USEC_PER_TICK);
            }
          else
            {
              gh3020_stop_sampling_factest();
              priv->activated--;
              priv->interval = priv->intvl_prev;
              work_cancel(HPWORK, &priv->work_poll);
            }
        }
      else
        {
          /* If any PPG channel has been activated, mark it. */

          if (priv->activated > 0)
            {
              priv->updating = true;
              if (enable == true)
                {
                  sensor->activating = true;
                }
              else
                {
                  sensor->inactivating = true;
                }
            }

          /* Otherwise no channel is activated now. */

          else
            {
              /* If one want activated a channel. Here "enable" should not be
               * false since no channel is activated now.
               */

              if (enable == true)
                {
                  rate = (uint16_t)(GH3020_ONE_SECOND / sensor->interval);
                  if (sensor->batch > 0)
                    {
                      gh3020_rdmode_switch(GH3020_RDMODE_INTERRPT);
                    }
                  else
                    {
                      gh3020_rdmode_switch(GH3020_RDMODE_POLLING);
                    }

                  /* GH3020 must be initialized after switch reading mode. */

                  gh3020_init();
                  priv->interval = sensor->interval;
                  priv->batch = sensor->batch;
                  priv->activated++;

                  /* Enable the channel with desired sample rate. */

                  priv->channelmode =
                    gh3020_channel_function_list[sensor->chidx];
                  gh3020_samplerate_set(
                    gh3020_channel_function_list[sensor->chidx], rate);

                  /* Set FIFO watermark if needed. */

                  priv->fifowtm = sensor->batch / sensor->interval *
                                  GH3020_DATA_PER_SAMPLE;
                  if (sensor->batch > 0)
                    {
                      gh3020_set_fifowtm(priv->fifowtm);
                    }

                  /* Some events may be processed between init and start */

                  gh3020_fifo_process();
                  gh3020_start_sampling(priv->channelmode);
                  if (sensor->batch > 0)
                    {
                      IOEXP_SETOPTION(priv->config->ioedev,
                                      priv->config->intpin,
                                      IOEXPANDER_OPTION_INTCFG,
                                      (FAR void *)IOEXPANDER_VAL_RISING);
                    }
                  else
                    {
                      work_queue(HPWORK, &priv->work_poll,
                                 gh3020_worker_poll, priv,
                                 priv->interval / USEC_PER_TICK);
                    }

                  sched_note_printf("activate ppgq%u, rate=%u, GH3020 starts"
                                    ", fifowtm=%u", sensor->chidx, rate,
                                    priv->fifowtm);
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: gh3020_batch
 *
 * Description:
 *   Set PPG sensor's maximum report latency in microseconds.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
 *   latency_us - the time between batch data, in us. It may by overwrite
 *                by lower half driver.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_batch(FAR struct sensor_lowerhalf_s *lower,
                        FAR unsigned int *latency_us)
{
  FAR struct gh3020_sensor_s *sensor = (FAR struct gh3020_sensor_s *)lower;
  FAR struct gh3020_dev_s *priv;
  uint32_t max_latency;
  uint16_t fifowtm = 0;

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL && latency_us != NULL);

  priv = sensor->dev;
  if (priv->factest_mode == true)
    {
      return OK;
    }

  /* Latency is modified to reasonable value. */

  if (*latency_us > 0)
    {
      max_latency = sensor->lower.batch_number * sensor->interval;
      if (*latency_us > max_latency)
        {
          *latency_us = max_latency;
        }
      else if (*latency_us < sensor->interval)
        {
          *latency_us = sensor->interval;
        }

      fifowtm = GH3020_CEILING(*latency_us, sensor->interval);
      *latency_us = fifowtm * sensor->interval;
    }

  /* Do something only when the batch changed. */

  if (*latency_us != sensor->batch)
    {
      sensor->batch = (uint32_t)*latency_us;

      /* If GH3020 is running, flag it and new batch will take effect when
       * next FIFO procession comes. Otherwise the new batch will take effect
       * when activating this PPG channel.
       */

      if (priv->activated > 0)
        {
          priv->updating = true;
        }
    }
  else
    {
      return OK;
    }

  return OK;
}

/****************************************************************************
 * Name: gh3020_set_interval
 *
 * Description:
 *   Set the sensor output data period in microseconds for a given sensor.
 *   If *period_us > max_interval it will be truncated to max_interval and
 *   if *period_us < min_interval it will be replaced by min_interval. The
 *   new interval will take effect when activating or reading FIFO.
 *
 * Input Parameters:
 *   lower     - The instance of lower half sensor driver.
 *   period_us - The time between report data, in us. It may by overwrite
 *               by lower half driver.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR unsigned int *period_us)
{
  FAR struct gh3020_sensor_s *sensor = (FAR struct gh3020_sensor_s *)lower;
  FAR struct gh3020_dev_s *priv;
  float freq;

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL && period_us != NULL);

  priv = sensor->dev;
  if (priv->factest_mode == true)
    {
      return OK;
    }

  /* Calculate best sample rate */

  freq = GH3020_ONE_SECOND / *period_us;
  if (freq > GH3020_SAMPLERATE_MAX)
    {
      freq = GH3020_SAMPLERATE_MAX;
    }
  else if (freq < GH3020_SAMPLERATE_MIN)
    {
      freq = GH3020_SAMPLERATE_MIN;
    }

  *period_us = (unsigned int)(GH3020_ONE_SECOND / (uint32_t)freq);

  /* Do something only when the interval changed. */

  if (*period_us != sensor->interval)
    {
      sensor->interval = (uint32_t)*period_us;

      /* If this PPG channel is running, new interval will take effect when
       * next FIFO procession comes. Otherwise do nothing, the new interval
       * will take effect when activating this sensor (PPG channel).
       */

      if (sensor->activated == true)
        {
          priv->updating = true;
        }
    }
  else
    {
      return OK;
    }

  return OK;
}

/****************************************************************************
 * Name: gh3020_selftest
 *
 * Description:
 *   Selftest of PPG sensor, i.e. the selftest of GH3020.
 *
 * Input Parameters:
 *   lower - The instance of lower half sensor driver.
 *   arg   - The parameters associated with cmd.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_selftest(FAR struct sensor_lowerhalf_s *lower,
                           unsigned long arg)
{
  FAR struct gh3020_dev_s *priv = (FAR struct gh3020_dev_s *)lower;

  DEBUGASSERT(priv != NULL);

  switch (arg)
    {
      case GH3020_CTRL_CHECKID:             /* Check ID command. */
        {
          if (gh3020_init() != OK)
            {
              return -ENODEV;
            }
          else
            {
              return OK;
            }
        }

      /* In the case above, function has returned thus no break is need. */

      default:                              /* Other cmd tag */
        {
          snerr("The cmd was not supported: %d\n", -ENOTTY);
          return -ENOTTY;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: gh3020_control
 *
 * Description:
 *   With this method, the user can set some special config for the sensor,
 *   such as changing the custom mode, setting the custom resolution, reset,
 *   etc, which are all parsed and implemented by lower half driver.
 *
 * Input Parameters:
 *   lower - The instance of lower half sensor driver.
 *   cmd   - The special cmd for sensor.
 *   arg   - The parameters associated with cmd.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_control(FAR struct sensor_lowerhalf_s *lower, int cmd,
                          unsigned long arg)
{
  FAR struct gh3020_sensor_s *sensor = (FAR struct gh3020_sensor_s *)lower;
  FAR struct gh3020_dev_s *priv;

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL);

  priv = sensor->dev;

  switch (cmd)
    {
      case GH3020_CTRL_LED_CURRENT:      /* Set LED current(uA) */
        {
          if (priv->factest_mode == true)
            {
              FAR uint32_t *current = (FAR uint32_t *)arg;
              sensor->current = *current / 1000 * 1000;
              *current = sensor->current;
              if (sensor->activated == true)
                {
                  gh3020_stop_sampling_factest();
                  gh3x2x_factest_start(
                    gh3020_channel_function_list[sensor->chidx],
                    sensor->current);
                }
            }
          else
            {
              return -EINVAL;
            }
        };
        break;

      case GH3020_CTRL_OPEN_FACTEST:    /* Enter factory test mode */
        {
          if (priv->factest_mode == false)
            {
              priv->factest_mode = true;
              if (priv->activated > 0)
                {
                  gh3020_stop_sampling(priv->channelmode);
                }
            }
        };
        break;

      case GH3020_CTRL_EXIT_FACTEST:     /* Exit factory test mode */
        {
          if (priv->factest_mode == true)
            {
              priv->factest_mode = false;
              if (priv->activated > 0)
                {
                  gh3020_stop_sampling_factest();
                }
            }
        };
        break;

      default:
        {
          snerr("No such command.\n");
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: gh3020_interrupt_handler
 *
 * Description:
 *   Handle the sensor interrupt.
 *
 * Input Parameters:
 *   dev     - ioexpander device.
 *   pinset  - Interrupt pin.
 *   arg     - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the GH3020 INTx
   * pin, when an event, such as FIFO is almost full, has occured.
   */

  FAR struct gh3020_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp */

  priv->timestamp = sensor_get_timestamp();

  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long, and neither should we
   * lock the I2C/SPI bus within an interrupt.
   */

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);
  work_queue(HPWORK, &priv->work_intrpt, gh3020_worker_intrpt, priv, 0);

  return OK;
}

/****************************************************************************
 * Name: gh3020_worker_intrpt
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data. We should not do
 *   this in a interrupt since it might take too long, and neither should we
 *   lock the I2C/SPI bus within an interrupt.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_worker_intrpt(FAR void *arg)
{
  FAR struct gh3020_dev_s *priv = arg;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Enable entering next interrupt. */

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_RISING);

  /* Start new counter for each channel. */

  memset(priv->ppgdatacnt, 0, GH3020_SENSOR_NUM);

  /* Goodix's documents said that this function must be called before calling
   * gh3020_fifo_process() in an interrupt procession.
   */

  gh3x2x_int_handler_call_back();

  /* This function from Goodix's library has an obvious delay and must be
   * called after arranging next worker.
   */

  gh3020_fifo_process();
  if (priv->updating == true)
    {
      gh3020_update_sensor(priv);
    }

  gh3020_push_data(priv);
}

/****************************************************************************
 * Name: gh3020_worker_poll
 *
 * Description:
 *   Task the worker with polling the latest sensor data.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_worker_poll(FAR void *arg)
{
  FAR struct gh3020_dev_s *priv = arg;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Get timestamp and arrange next worker immediately once enter polling. */

  priv->timestamp = sensor_get_timestamp();
  if (priv->activated > 0)
    {
      work_queue(HPWORK, &priv->work_poll, gh3020_worker_poll, priv,
                 priv->interval / USEC_PER_TICK);
    }

  /* Start new counter for each channel. */

  memset(priv->ppgdatacnt, 0, GH3020_SENSOR_NUM);

  /* This function from Goodix's library has an obvious delay and must be
   * called after arranging next worker.
   */

  gh3020_fifo_process();
  if (priv->updating == true)
    {
      gh3020_update_sensor(priv);
    }

  gh3020_push_data(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gh3020_spiread
 *
 * Description:
 *   Read content from SPI.
 *
 * Input Parameters:
 *   recvbuf - A pointer to the buffer which stores data read.
 *   nwords  - The numbers of word (8bits per word) to be read.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int gh3020_spiread(FAR uint8_t *recvbuf, uint16_t nwords)
{
  int ret;

  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(g_priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  ret = gh3020_configspi(g_priv);
  if (ret < 0)
    {
      snerr("SPI configuration failed: %d\n", ret);
      SPI_LOCK(g_priv->config->spi, false);
      return ret;
    }

  /* Selects the device. */

  SPI_SELECT(g_priv->config->spi, g_priv->config->cs, true);

  /* First send read command, then sends arbitrary content (nwords bytes) to
   * read a block of nwords bytes.
   */

  SPI_EXCHANGE(g_priv->config->spi, recvbuf, recvbuf, nwords);

  /* Deselect the device and release the SPI bus. */

  SPI_SELECT(g_priv->config->spi, g_priv->config->cs, false);
  SPI_LOCK(g_priv->config->spi, false);

  return OK;
}

/****************************************************************************
 * Name: gh3020_spiwrite
 *
 * Description:
 *   Write the content to SPI.
 *
 * Input Parameters:
 *   sendbuf - A pointer to the buffer which stores data to be written.
 *   nwords  - The numbers of word (8bits per word) to be written.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int gh3020_spiwrite(FAR uint8_t *sendbuf, uint16_t nwords)
{
  int ret;

  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(g_priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  ret = gh3020_configspi(g_priv);
  if (ret < 0)
    {
      snerr("SPI configuration failed: %d\n", ret);
      SPI_LOCK(g_priv->config->spi, false);
      return ret;
    }

  /* Selects the device. Set CS (as a GPIO) low. */

  SPI_SELECT(g_priv->config->spi, g_priv->config->cs, true);

  /* First send start address and 0x00(means reading). Then write a block. */

  SPI_EXCHANGE(g_priv->config->spi, sendbuf, sendbuf, nwords);

  /* Deselect the device, set CS high and release the SPI bus. */

  SPI_SELECT(g_priv->config->spi, g_priv->config->cs, false);
  SPI_LOCK(g_priv->config->spi, false);

  return OK;
}

/****************************************************************************
 * Name: gh3020_spi_csctrl
 *
 * Description:
 *   Control the level of SPI CS pin.
 *
 * Input Parameters:
 *   pinlevel - Desired level of SPI CS pin.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

void gh3020_spi_csctrl(uint8_t pinlevel)
{
  if (pinlevel == 0)
    {
      IOEXP_WRITEPIN(g_priv->config->ioedev, g_priv->config->gpiocs, 0);
    }
  else
    {
      IOEXP_WRITEPIN(g_priv->config->ioedev, g_priv->config->gpiocs, 1);
    }
}

/****************************************************************************
 * Name: gh3020_rstctrl
 *
 * Description:
 *   Control the level of reset pin.
 *
 * Input Parameters:
 *   pinlevel - Desired level of reset pin.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

void gh3020_rstctrl(uint8_t pinlevel)
{
  if (pinlevel == 0)
    {
      IOEXP_WRITEPIN(g_priv->config->ioerpmsg, g_priv->config->rstpin, 0);
    }
  else
    {
      IOEXP_WRITEPIN(g_priv->config->ioerpmsg, g_priv->config->rstpin, 1);
    }
}

/****************************************************************************
 * Name: gh3020_transdata
 *
 * Description:
 *   Control the level of SPI CS pin.
 *
 * Input Parameters:
 *   priv      - Device struct.
 *   startaddr - Address of the 1st register to be read.
 *   recvbuf   - A pointer to the buffer which stores the readouts.
 *   nwords    - The numbers of word (8bits per word) to be read.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

void gh3020_transdata(FAR struct sensor_event_ppgq *ppg, uint8_t chidx)
{
  if (g_priv->sensor[chidx].activated == true)
    {
      g_priv->ppgdata[chidx][g_priv->ppgdatacnt[chidx]] = *ppg;
      g_priv->ppgdatacnt[chidx]++;
    }
}

/****************************************************************************
 * Name: gh3020_register
 *
 * Description:
 *   Register the GH3020 character device.
 *
 * Input Parameters:
 *   devno  - The full path to the driver to register. E.g., "/dev/gh3020"
 *   config - An instance of the SPI interface to use to communicate with
 *            gh3020
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int gh3020_register(int devno, FAR const struct gh3020_config_s *config)
{
  FAR struct gh3020_dev_s *priv;
  FAR void *ioehandle;
  uint8_t idx;
  int ret;

  /* Sanity check */

  DEBUGASSERT(config != NULL);

  /* Initialize the GH3020 device structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (!priv)
    {
      snerr("Failed to allocate instance.\n");
      return -ENOMEM;
    }

  g_priv = priv;
  priv->config = config;
  for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
    {
      priv->sensor[idx].lower.ops = &g_gh3020_ops;
      priv->sensor[idx].lower.type = SENSOR_TYPE_PPGQ;
      priv->sensor[idx].lower.batch_number = GH3020_BATCH_NUMBER;
      priv->sensor[idx].dev = priv;
      priv->sensor[idx].interval = GH3020_INTVL_DFT;
      priv->sensor[idx].current = GH3020_CURRENT_DFT;
      priv->sensor[idx].chidx = (uint8_t)idx;
    }

  /* Initialize the device. After that, it will enter low-power mode(10uA).
   * Note that gh3020_init will return 0 for OK or a positive error code.
   */

  gh3020_rdmode_switch(GH3020_RDMODE_POLLING);
  ret = gh3020_init();
  if (ret != OK)
    {
      ret = -ret;
      snerr("Device ID doesn't match: %d\n", ret);
      goto err_exit;
    }

  /* Configure interrupt pin */

  ret = IOEXP_SETDIRECTION(priv->config->ioedev, priv->config->intpin,
                           IOEXPANDER_DIRECTION_IN_PULLDOWN);
  if (ret < 0)
    {
      snerr("Failed to set direction: %d\n", ret);
      goto err_exit;
    }

  ioehandle = IOEP_ATTACH(priv->config->ioedev, priv->config->intpin,
                          gh3020_interrupt_handler, priv);
  if (ioehandle == NULL)
    {
      ret = -EIO;
      snerr("Failed to attach: %d\n", ret);
      goto err_exit;
    }

  ret = IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                        IOEXPANDER_OPTION_INTCFG,
                        (FAR void *)IOEXPANDER_VAL_DISABLE);
  if (ret < 0)
    {
      snerr("Failed to set option: %d\n", ret);
      goto err_iodetach;
    }

  /* Register the character driver */

  for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
    {
      ret = sensor_register((&(priv->sensor[idx].lower)),
                            devno * GH3020_SENSOR_NUM + idx);
      if (ret < 0)
        {
          snerr("Failed to register GH3020 PPGQ%d driver: %d\n",
                devno * GH3020_SENSOR_NUM + idx, ret);

          /* Unregister all registered ppgq sensors */

          while (idx)
            {
              idx--;
              sensor_unregister(&priv->sensor[idx].lower,
                                devno * GH3020_SENSOR_NUM + idx);
            }

          goto err_iodetach;
        }
    }

  return ret;

err_iodetach:
  IOEP_DETACH(priv->config->ioedev, gh3020_interrupt_handler);

err_exit:
  kmm_free(priv);
  return ret;
}

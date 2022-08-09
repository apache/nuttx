/****************************************************************************
 * drivers/rc/dummy.c
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

#include <nuttx/rc/lirc_dev.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <debug.h>
#include <inttypes.h>
#include <stdio.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DUMMY_MAX_EVENT_SIZE        512
#define DUMMY_WORK_PERIOD           SEC2TICK(1)

/****************************************************************************
 * Private
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  dummy_open(FAR struct lirc_lowerhalf_s *lower);
static void dummy_close(FAR struct lirc_lowerhalf_s *lower);
static int  dummy_s_tx_mask(FAR struct lirc_lowerhalf_s *lower,
                            unsigned int mask);
static int  dummy_s_tx_carrier(FAR struct lirc_lowerhalf_s *lower,
                               unsigned int carrier);
static int  dummy_s_tx_duty_cycle(FAR struct lirc_lowerhalf_s *lower,
                                  unsigned int duty_cycle);
static int  dummy_s_rx_carrier_range(FAR struct lirc_lowerhalf_s *lower,
                                     unsigned int min, unsigned int max);
static int  dummy_tx_ir(FAR struct lirc_lowerhalf_s *lower,
                        FAR unsigned int *txbuf, unsigned int n);
static int  dummy_tx_scancode(FAR struct lirc_lowerhalf_s *lower,
                              FAR struct lirc_scancode *txbuf);
static int  dummy_s_learning_mode(FAR struct lirc_lowerhalf_s *lower,
                                  int enable);
static int  dummy_s_carrier_report(FAR struct lirc_lowerhalf_s *lower,
                                   int enable);
static int  dummy_s_timeout(FAR struct lirc_lowerhalf_s *lower,
                            unsigned int timeout);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dummy_dev_s
{
  struct lirc_lowerhalf_s lower;
  struct work_s work;
  unsigned test_sample;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct lirc_ops_s g_dummy_ops =
{
  LIRC_DRIVER_IR_RAW,           /* driver_type */
  dummy_open,                   /* open */
  dummy_close,                  /* close */
  dummy_s_tx_mask,              /* s_tx_mask */
  dummy_s_tx_carrier,           /* s_tx_carrier */
  dummy_s_tx_duty_cycle,        /* s_tx_duty_cycle */
  dummy_s_rx_carrier_range,     /* s_rx_carrier_range */
  dummy_tx_ir,                  /* tx_ir */
  dummy_tx_scancode,            /* tx_scancode */
  dummy_s_learning_mode,        /* s_learning_mode */
  dummy_s_carrier_report,       /* s_carrier_report */
  dummy_s_timeout               /* s_timeout */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void dummy_worker(FAR void *arg)
{
  struct dummy_dev_s *dummy = arg;
  unsigned sample = dummy->test_sample;

  if (sample % 2 == 0)
    {
      sample = LIRC_SPACE(sample);
    }
  else
    {
      sample = LIRC_PULSE(sample);
    }

  lirc_sample_event(&dummy->lower, sample);

  rcinfo("Dummy RC read Raw Data from device:%d\n", sample);
  dummy->test_sample++;

  work_queue(LPWORK, &dummy->work, dummy_worker, dummy,
             DUMMY_WORK_PERIOD);
}

static int dummy_open(FAR struct lirc_lowerhalf_s *lower)
{
  struct dummy_dev_s *dummy = (struct dummy_dev_s *)lower;

  rcinfo("Called %s\n", __func__);

  dummy->test_sample = 0;
  return work_queue(LPWORK, &dummy->work, dummy_worker, dummy,
                   DUMMY_WORK_PERIOD);
}

static void dummy_close(FAR struct lirc_lowerhalf_s *lower)
{
  struct dummy_dev_s *dummy = (struct dummy_dev_s *)lower;

  rcinfo("Called %s\n", __func__);

  work_cancel(LPWORK, &dummy->work);
}

static int dummy_s_tx_mask(FAR struct lirc_lowerhalf_s *lower,
                           unsigned int mask)
{
  rcinfo("Called %s, mask:%u\n", __func__, mask);
  return 0;
}

static int dummy_s_tx_carrier(FAR struct lirc_lowerhalf_s *lower,
                              unsigned int carrier)
{
  rcinfo("Called %s, carrier:%u\n", __func__, carrier);
  return 0;
}

static int dummy_s_tx_duty_cycle(FAR struct lirc_lowerhalf_s *lower,
                                 unsigned int duty_cycle)
{
  rcinfo("Called %s, duty_cycle:%u\n", __func__, duty_cycle);
  return 0;
}

static int dummy_s_rx_carrier_range(FAR struct lirc_lowerhalf_s *lower,
                                    unsigned int min, unsigned int max)
{
  rcinfo("Called %s, min:%u, max:%u\n", __func__, min, max);
  return 0;
}

static int dummy_tx_ir(FAR struct lirc_lowerhalf_s *lower,
                       unsigned *txbuf, unsigned int n)
{
  rcinfo("Dummy RC send raw data:%d(size:%d) to device\n", *txbuf, n);
  return n;
}

static int dummy_tx_scancode(FAR struct lirc_lowerhalf_s *lower,
                             FAR struct lirc_scancode *txbuf)
{
  rcinfo("Dummy RC send scancode data:%" PRIu64 " to device\n",
         txbuf->scancode);
  return 0;
}

static int dummy_s_learning_mode(FAR struct lirc_lowerhalf_s *lower,
                                 int enable)
{
  rcinfo("Called %s, enable:%d\n", __func__, enable);
  return 0;
}

static int dummy_s_carrier_report(FAR struct lirc_lowerhalf_s *lower,
                                  int enable)
{
  rcinfo("Called %s, enable:%d\n", __func__, enable);
  return 0;
}

static int dummy_s_timeout(FAR struct lirc_lowerhalf_s *lower,
                           unsigned int timeout)
{
  rcinfo("Called %s, timeout:%u\n", __func__, timeout);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rc_dummy_initialize(int devno)
{
  struct dummy_dev_s *dummy;

  dummy = kmm_malloc(sizeof(*dummy));
  if (!dummy)
    {
      rcerr("failed to alloc memory for dummy\n");
      return -ENOMEM;
    }

  dummy->lower.ops = &g_dummy_ops;
  dummy->lower.buffer_bytes = DUMMY_MAX_EVENT_SIZE * sizeof(unsigned);

  dummy->work.worker = NULL;
  dummy->test_sample = 0;

  return lirc_register(&dummy->lower, devno);
}

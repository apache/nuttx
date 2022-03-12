/****************************************************************************
 * include/nuttx/rc/lirc_dev.h
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

#ifndef __INCLUDE_NUTTX_RC_LIRC_DEV_H
#define __INCLUDE_NUTTX_RC_LIRC_DEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/list.h>
#include <nuttx/semaphore.h>
#include <nuttx/lirc.h>

#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint64_t lirc_get_timestamp(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  return 1000000000ull * ts.tv_sec + ts.tv_nsec;
}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* enum lirc_driver_type_e - type of the LIRC driver.
 *
 * LIRC_DRIVER_SCANCODE:  Driver or hardware generates a scancode.
 * LIRC_DRIVER_IR_RAW:    Driver or hardware generates pulse/space sequences.
 *                        It needs a Infra-Red pulse/space decoder
 * LIRC_DRIVER_IR_RAW_TX: Device transmitter only,
 *                        driver requires pulse/space data sequence.
 */

enum lirc_driver_type_e
{
  LIRC_DRIVER_SCANCODE = 0,
  LIRC_DRIVER_IR_RAW,
  LIRC_DRIVER_IR_RAW_TX,
};

/* The Raw interface is specific to InfraRed. */

struct lirc_raw_event_s
{
  union
  {
    uint32_t      duration;
    uint32_t      carrier;
  };

  uint8_t         duty_cycle;

  unsigned        pulse:1;
  unsigned        reset:1;
  unsigned        timeout:1;
  unsigned        carrier_report:1;
};

/* struct lirc_ops_s - The lower half driver operations
 * driver_type: specifies if protocol decoding is done in hard/software
 * open: allow drivers to enable polling/irq when IR input dev is opened.
 * close: allow drivers to disable polling/irq when IR input dev is closed.
 * s_tx_mask: set transmitter mask (for devices with multiple tx outputs)
 * s_tx_carrier: set transmit carrier frequency
 * s_tx_duty_cycle: set transmit duty cycle (0% - 100%)
 * s_rx_carrier_range: inform driver about carrier it is expected to handle
 * tx_ir: transmit raw data to IR
 * tx_scancode: transmit scancodes to IR
 * s_learning_mode: enable wide band receiver used for learning
 * s_carrier_report: enable carrier reports
 * s_timeout: set hardware timeout in us
 */

struct lirc_lowerhalf_s;
struct lirc_ops_s
{
  enum lirc_driver_type_e driver_type;
  CODE int  (*open)(FAR struct lirc_lowerhalf_s *lower);
  CODE void (*close)(FAR struct lirc_lowerhalf_s *lower);
  CODE int  (*s_tx_mask)(FAR struct lirc_lowerhalf_s *lower,
                         unsigned int mask);
  CODE int  (*s_tx_carrier)(FAR struct lirc_lowerhalf_s *lower,
                            unsigned int carrier);
  CODE int  (*s_tx_duty_cycle)(FAR struct lirc_lowerhalf_s *lower,
                               unsigned int duty_cycle);
  CODE int  (*s_rx_carrier_range)(FAR struct lirc_lowerhalf_s *lower,
                                  unsigned int min, unsigned int max);
  CODE int  (*tx_ir)(FAR struct lirc_lowerhalf_s *lower,
                     FAR unsigned int *txbuf, unsigned int n);
  CODE int  (*tx_scancode)(FAR struct lirc_lowerhalf_s *lower,
                           FAR struct lirc_scancode *txbuf);
  CODE int  (*s_learning_mode)(FAR struct lirc_lowerhalf_s *lower,
                               int enable);
  CODE int  (*s_carrier_report) (FAR struct lirc_lowerhalf_s *lower,
                                 int enable);
  CODE int  (*s_timeout)(FAR struct lirc_lowerhalf_s *lower,
                         unsigned int timeout);
};

/* struct lirc_lowerhalf_s - represents a remote control device
 * ops: the lirc lowerhalf driver operations
 * priv: the pointer to lirc upperhalf handle, it's updated by lirc_register.
 * timeout: optional time after which device stops sending data
 * min_timeout: minimum timeout supported by device
 * max_timeout: maximum timeout supported by device
 * buffer_bytes: The size of intermediate buffer, in bytes unit. we recommend
 *               size is a multiple of unsigned int for LIRC_DRIVER_IR_RAW,
 *               is a multiple of struct lirc_scancode for
 *               LIRC_DRIVER_SCANCODE. we don't need to set buffer_bytes for
 *               LIRC_DRIVER_IR_RAW_TX.
 * rx_resolution: resolution (in us) of input sampler
 * tx_resolution: resolution (in us) of output sampler
 */

struct lirc_lowerhalf_s
{
  FAR const struct lirc_ops_s  *ops;
  FAR void                     *priv;
  unsigned int                  timeout;
  unsigned int                  min_timeout;
  unsigned int                  max_timeout;
  unsigned int                  buffer_bytes;
  unsigned int                  rx_resolution;
  unsigned int                  tx_resolution;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: lirc_register
 *
 * Description:
 *   This function binds an instance of a "lower half" lirc driver with the
 *   "upper half" RC device and registers that device so that can be used
 *   by application code.
 *
 *   We will register the chararter device. ex: /dev/lirc%d(0, 1, ...)
 *
 * Input Parameters:
 *   lower - A pointer to an instance of lower half lirc driver.
 *   devno - The user specifies device number, from 0. If the
 *           devno alerady exists, -EEXIST will be returned.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int lirc_register(FAR struct lirc_lowerhalf_s *lower, int devno);

/****************************************************************************
 * Name: lirc_unregister
 *
 * Description:
 *   This function unregister character node and release all resource about
 *   upper half driver.
 *
 * Input Parameters:
 *   lower - A pointer to an instance of lower half lirc driver.
 *   devno - The user specifies device number, from 0.
 ****************************************************************************/

void lirc_unregister(FAR struct lirc_lowerhalf_s *lower, int devno);

/****************************************************************************
 * Name: lirc_sample_event
 *
 * Description:
 *   Lirc lowerhalf driver sends raw IR data to lirc upperhalf buffer, to
 *   notify userspace to read IR data.
 *
 *   The type of data is a sequence of pulse and space codes, as a seriers
 *   of unsigned values.
 *
 *   The upper 8 bits determine the packet type, and the lower 24 bits the
 *   payload.
 *
 * Input Parameters:
 *   lower  - A pointer to an instance of lower half lirc driver.
 *   sample - The data of receiving from IR device
 ****************************************************************************/

void lirc_sample_event(FAR struct lirc_lowerhalf_s *lower,
                       unsigned int sample);

/****************************************************************************
 * Name: lirc_raw_event
 *
 * Description:
 *   Lirc lowerhalf driver sends IR data to lirc upperhalf buffer, to
 *   notify userspace to read IR data.
 *
 *   The type of data is struct lirc_raw_event_s.
 *
 * Input Parameters:
 *   lower  - A pointer to an instance of lower half lirc driver.
 *   ev     - The data of receiving from IR device
 ****************************************************************************/

void lirc_raw_event(FAR struct lirc_lowerhalf_s *lower,
                    struct lirc_raw_event_s ev);

/****************************************************************************
 * Name: lirc_scancode_event
 *
 * Description:
 *   Lirc lowerhalf driver sends IR data to lirc upperhalf buffer, to
 *   notify userspace to read IR data.
 *
 *   The type of data is struct lirc_scancode.
 *
 * Input Parameters:
 *   lower  - A pointer to an instance of lower half lirc driver.
 *   lsc    - The data of receiving from IR device
 ****************************************************************************/

void lirc_scancode_event(FAR struct lirc_lowerhalf_s *lower,
                         FAR struct lirc_scancode *lsc);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif

/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wtd.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_WTD_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_WTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 #define FEED_DOG        0xffffffff

/* Helpers ******************************************************************/

#define ESP32_WTD_START(d)                      ((d)->ops->start(d))
#define ESP32_WTD_STOP(d)                       ((d)->ops->stop(d))
#define ESP32_WTD_LOCK(d)                       ((d)->ops->enablewp(d))
#define ESP32_WTD_UNLOCK(d)                     ((d)->ops->disablewp(d))
#define ESP32_WTD_INITCONF(d)                   ((d)->ops->initconf(d))
#define ESP32_WTD_PRE(d, v)                     ((d)->ops->pre(d, v))
#define ESP32_WTD_STO(d, v, s)                  ((d)->ops->settimeout(d, v, s))
#define ESP32_WTD_FEED(d)                       ((d)->ops->feed(d))
#define ESP32_WTD_STG_CONF(d, s, c)             ((d)->ops->stg_conf(d, s, c))
#define ESP32_RWDT_CLK(d)                       ((d)->ops->rtc_clk(d))
#define ESP32_WTD_SETISR(d, hnd, arg)           ((d)->ops->setisr(d, hnd, arg))
#define ESP32_WTD_ENABLEINT(d)                  ((d)->ops->enableint(d))
#define ESP32_WTD_DISABLEINT(d)                 ((d)->ops->disableint(d))
#define ESP32_WTD_ACKINT(d)                     ((d)->ops->ackint(d))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ESP32 WTD device */

struct esp32_wtd_dev_s
{
  struct esp32_wtd_ops_s *ops;
};

/* ESP32 WTD ops */

/* This is a struct containing the pointers to the wtd operations */

struct esp32_wtd_ops_s
{
  /* WTD tasks */

  CODE int (*start)(FAR struct esp32_wtd_dev_s *dev);
  CODE int (*stop)(FAR struct esp32_wtd_dev_s *dev);

  /* WTD configuration */

  CODE int (*enablewp)(FAR struct esp32_wtd_dev_s *dev);
  CODE int (*disablewp)(FAR struct esp32_wtd_dev_s *dev);
  CODE int (*initconf)(FAR struct esp32_wtd_dev_s *dev);
  CODE int (*pre)(FAR struct esp32_wtd_dev_s *dev, uint16_t value);
  CODE int (*settimeout)(FAR struct esp32_wtd_dev_s *dev,
                         uint32_t value, uint8_t stage);
  CODE int (*feed)(FAR struct esp32_wtd_dev_s *dev);
  CODE int (*stg_conf)(FAR struct esp32_wtd_dev_s *dev,
                                   uint8_t stage, uint8_t conf);
  CODE uint16_t (*rtc_clk)(FAR struct esp32_wtd_dev_s *dev);

  /* WTD interrupts */

  CODE int (*setisr)(FAR struct esp32_wtd_dev_s *dev, xcpt_t handler,
                     FAR void * arg);
  CODE int (*enableint)(FAR struct esp32_wtd_dev_s *dev);
  CODE int (*disableint)(FAR struct esp32_wtd_dev_s *dev);
  CODE int (*ackint)(FAR struct esp32_wtd_dev_s *dev);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct esp32_wtd_dev_s *esp32_wtd_init(uint8_t wdt_id);
int esp32_wtd_deinit(FAR struct esp32_wtd_dev_s *dev);

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_WTD_H */

/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wdt.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_WDT_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_WDT_H

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

#define ESP32_WDT_START(d)                      ((d)->ops->start(d))
#define ESP32_WDT_STOP(d)                       ((d)->ops->stop(d))
#define ESP32_WDT_LOCK(d)                       ((d)->ops->enablewp(d))
#define ESP32_WDT_UNLOCK(d)                     ((d)->ops->disablewp(d))
#define ESP32_WDT_PRE(d, v)                     ((d)->ops->pre(d, v))
#define ESP32_WDT_STO(d, v, s)                  ((d)->ops->settimeout(d, v, s))
#define ESP32_WDT_FEED(d)                       ((d)->ops->feed(d))
#define ESP32_WDT_STG_CONF(d, s, c)             ((d)->ops->stg_conf(d, s, c))
#define ESP32_RWDT_CLK(d)                       ((d)->ops->rtc_clk(d))
#define ESP32_WDT_SETISR(d, hnd, arg)           ((d)->ops->setisr(d, hnd, arg))
#define ESP32_WDT_ENABLEINT(d)                  ((d)->ops->enableint(d))
#define ESP32_WDT_DISABLEINT(d)                 ((d)->ops->disableint(d))
#define ESP32_WDT_ACKINT(d)                     ((d)->ops->ackint(d))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ESP32 WDT device */

struct esp32_wdt_dev_s
{
  struct esp32_wdt_ops_s *ops;
};

/* ESP32 WDT ops */

/* This is a struct containing the pointers to the wdt operations */

struct esp32_wdt_ops_s
{
  /* WDT tasks */

  CODE int (*start)(FAR struct esp32_wdt_dev_s *dev);
  CODE int (*stop)(FAR struct esp32_wdt_dev_s *dev);

  /* WDT configuration */

  CODE int (*enablewp)(FAR struct esp32_wdt_dev_s *dev);
  CODE int (*disablewp)(FAR struct esp32_wdt_dev_s *dev);
  CODE int (*pre)(FAR struct esp32_wdt_dev_s *dev, uint16_t value);
  CODE int (*settimeout)(FAR struct esp32_wdt_dev_s *dev,
                         uint32_t value, uint8_t stage);
  CODE int (*feed)(FAR struct esp32_wdt_dev_s *dev);
  CODE int (*stg_conf)(FAR struct esp32_wdt_dev_s *dev,
                                   uint8_t stage, uint8_t conf);
  CODE uint16_t (*rtc_clk)(FAR struct esp32_wdt_dev_s *dev);

  /* WDT interrupts */

  CODE int (*setisr)(FAR struct esp32_wdt_dev_s *dev, xcpt_t handler,
                     FAR void * arg);
  CODE int (*enableint)(FAR struct esp32_wdt_dev_s *dev);
  CODE int (*disableint)(FAR struct esp32_wdt_dev_s *dev);
  CODE int (*ackint)(FAR struct esp32_wdt_dev_s *dev);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct esp32_wdt_dev_s *esp32_wdt_init(uint8_t wdt_id);
void esp32_wdt_early_deinit(void);
int esp32_wdt_deinit(FAR struct esp32_wdt_dev_s *dev);
bool esp32_wdt_is_running(FAR struct esp32_wdt_dev_s *dev);

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_WDT_H */

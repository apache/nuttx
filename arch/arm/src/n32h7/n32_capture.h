/****************************************************************************
 * arch/arm/src/n32h7/n32_capture.h
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

#ifndef __ARCH_ARM_SRC_N32H7_N32_CAPTURE_H
#define __ARCH_ARM_SRC_N32H7_N32_CAPTURE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include <arch/board/board.h>
#include "hardware/n32h7_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define N32_CAP_SETSMC(d,cfg)                 ((d)->ops->setsmc(d,cfg))
#define N32_CAP_SETCLOCK(d,clk,max)           ((d)->ops->setclock(d,clk,max))
#define N32_CAP_SETCHANNEL(d,ch,cfg)          ((d)->ops->setchannel(d,ch,cfg))
#define N32_CAP_GETCAPTURE(d,ch)              ((d)->ops->getcapture(d,ch))
#define N32_CAP_GETEDGES(d,ch)                ((d)->ops->getedges(d,ch))
#define N32_CAP_SETISR(d,hnd,arg)             ((d)->ops->setisr(d,hnd,arg))
#define N32_CAP_ENABLEINT(d,s,on)             ((d)->ops->enableint(d,s,on))
#define N32_CAP_ACKFLAGS(d,f)                 ((d)->ops->ackflags(d,f))
#define N32_CAP_GETFLAGS(d)                   ((d)->ops->getflags(d))

#define N32_CAP_EDGE_MASK           (3 << 8)
#define N32_CAP_EDGE_DISABLED       (0 << 8)
#define N32_CAP_EDGE_RISING         (1 << 8)
#define N32_CAP_EDGE_FALLING        (2 << 8)
#define N32_CAP_EDGE_BOTH           (3 << 8)

/* Flags */
#define N32_CAP_FLAG_IRQ_COUNTER    (N32_ATIM_STS_UDITF)
#define N32_CAP_FLAG_IRQ_CH_1       (N32_ATIM_STS_CC1ITF)
#define N32_CAP_FLAG_IRQ_CH_2       (N32_ATIM_STS_CC2ITF)
#define N32_CAP_FLAG_IRQ_CH_3       (N32_ATIM_STS_CC3ITF)
#define N32_CAP_FLAG_IRQ_CH_4       (N32_ATIM_STS_CC4ITF)
#define N32_CAP_FLAG_OF_CH_1        (N32_ATIM_STS_CC1OCF)
#define N32_CAP_FLAG_OF_CH_2        (N32_ATIM_STS_CC2OCF)
#define N32_CAP_FLAG_OF_CH_3        (N32_ATIM_STS_CC3OCF)
#define N32_CAP_FLAG_OF_CH_4        (N32_ATIM_STS_CC4OCF)

#define N32_CAP_FLAG_IRQ_CH(ch)     (N32_ATIM_STS_CC1ITF << ((ch)-1))
#define N32_CAP_FLAG_OF_CH(ch)      (N32_ATIM_STS_CC1OCF << ((ch)-1))
#define N32_CAP_CHANNEL_COUNTER     0

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint16_t n32_cap_ch_cfg_t;
typedef uint32_t n32_cap_smc_cfg_t;
typedef uint32_t n32_cap_flags_t;

struct n32_cap_dev_s
{
  const struct n32_cap_ops_s *ops;
};

struct n32_cap_ops_s
{
  int  (*setsmc)(struct n32_cap_dev_s *dev, n32_cap_smc_cfg_t cfg);
  int  (*setclock)(struct n32_cap_dev_s *dev, uint32_t freq, uint32_t max);
  int  (*setchannel)(struct n32_cap_dev_s *dev, uint8_t channel,
                     n32_cap_ch_cfg_t cfg);
  uint32_t (*getcapture)(struct n32_cap_dev_s *dev, uint8_t channel);
  uint32_t (*getedges)(struct n32_cap_dev_s *dev, uint8_t channel);
  int  (*setisr)(struct n32_cap_dev_s *dev, xcpt_t handler, void *arg);
  void (*enableint)(struct n32_cap_dev_s *dev, n32_cap_flags_t src, bool on);
  void (*ackflags)(struct n32_cap_dev_s *dev, int flags);
  n32_cap_flags_t (*getflags)(struct n32_cap_dev_s *dev);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct n32_cap_dev_s *n32_cap_init(int timer);
int n32_cap_deinit(struct n32_cap_dev_s *dev);

#ifdef CONFIG_CAPTURE
struct cap_lowerhalf_s *n32_cap_initialize(int timer);
#endif

#endif /* __ARCH_ARM_SRC_N32H7_N32_CAPTURE_H */

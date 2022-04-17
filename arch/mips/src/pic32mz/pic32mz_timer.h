/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_timer.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_TIMER_H
#define __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define PIC32MZ_TIMER_START(d)              ((d)->ops->start(d))
#define PIC32MZ_TIMER_STOP(d)               ((d)->ops->stop(d))
#define PIC32MZ_TIMER_SETPERIOD(d,p)        ((d)->ops->setperiod(d,p))
#define PIC32MZ_TIMER_GETCOUNTER(d)         ((d)->ops->getcounter(d))
#define PIC32MZ_TIMER_SETCOUNTER(d,c)       ((d)->ops->setcounter(d,c))
#define PIC32MZ_TIMER_GETFREQ(d)            ((d)->ops->getfreq(d))
#define PIC32MZ_TIMER_SETFREQ(d,f)          ((d)->ops->setfreq(d,f))
#define PIC32MZ_TIMER_GETWIDTH(d)           ((d)->ops->getwidth(d))

#define PIC32MZ_TIMER_SETISR(d,hnd,arg)     ((d)->ops->setisr(d,hnd,arg))
#define PIC32MZ_TIMER_ACKINT(d)             ((d)->ops->ackint(d))
#define PIC32MZ_TIMER_CHECKINT(d)           ((d)->ops->checkint(d))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Timer's Operations */

struct pic32mz_timer_dev_s; /* Forward reference */
struct pic32mz_timer_ops_s
{
  /* Timer's methods */

  void     (*start)(struct pic32mz_timer_dev_s *dev);
  void     (*stop)(struct pic32mz_timer_dev_s *dev);
  void     (*setperiod)(struct pic32mz_timer_dev_s *dev, uint32_t p);
  uint32_t (*getcounter)(struct pic32mz_timer_dev_s *dev);
  void     (*setcounter)(struct pic32mz_timer_dev_s *dev, uint32_t c);
  uint32_t (*getfreq)(struct pic32mz_timer_dev_s *dev);
  bool     (*setfreq)(struct pic32mz_timer_dev_s *dev, uint32_t freq);
  uint8_t  (*getwidth)(struct pic32mz_timer_dev_s *dev);

  /* Timer's interrupts */

  int      (*setisr)(struct pic32mz_timer_dev_s *dev, xcpt_t handler,
                     void * arg);
  void     (*ackint)(struct pic32mz_timer_dev_s *dev);
  bool     (*checkint)(struct pic32mz_timer_dev_s *dev);
};

/* Timer's Device Structure */

struct pic32mz_timer_dev_s
{
  struct pic32mz_timer_ops_s *ops;
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_timer_init
 *
 * Description:
 *   Power-up the timer and get its structure.
 *
 ****************************************************************************/

struct pic32mz_timer_dev_s *pic32mz_timer_init(int timer);

/****************************************************************************
 * Name: pic32mz_timer_deinit
 *
 * Description:
 *   Power-down the timer and mark it as unused.
 *
 ****************************************************************************/

int pic32mz_timer_deinit(struct pic32mz_timer_dev_s *dev);

/****************************************************************************
 * Name: pic32mz_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath   The full path to the timer device.
 *             This should be of the form /dev/timer0
 *   timer     The timer's number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int pic32mz_timer_initialize(const char *devpath, int timer);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_TIMER_H */

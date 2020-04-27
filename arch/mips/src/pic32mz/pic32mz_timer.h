/****************************************************************************
 * arch/mips/src/pic32mz/chip/pic32mz_timer.h
 *
 *   Copyright (C) 2019 Abdelatif Guettouche. All rights reserved.
 *   Author: Abdelatif Guettouche <abdelatif.guettouche@gmail.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

  void     (*start)(FAR struct pic32mz_timer_dev_s *dev);
  void     (*stop)(FAR struct pic32mz_timer_dev_s *dev);
  void     (*setperiod)(FAR struct pic32mz_timer_dev_s *dev, uint32_t p);
  uint32_t (*getcounter)(FAR struct pic32mz_timer_dev_s *dev);
  void     (*setcounter)(FAR struct pic32mz_timer_dev_s *dev, uint32_t c);
  uint32_t (*getfreq)(FAR struct pic32mz_timer_dev_s *dev);
  bool     (*setfreq)(FAR struct pic32mz_timer_dev_s *dev, uint32_t freq);
  uint8_t  (*getwidth)(FAR struct pic32mz_timer_dev_s *dev);

  /* Timer's interrupts */

  int      (*setisr)(FAR struct pic32mz_timer_dev_s *dev, xcpt_t handler,
                     void * arg);
  void     (*ackint)(FAR struct pic32mz_timer_dev_s *dev);
  bool     (*checkint)(FAR struct pic32mz_timer_dev_s *dev);
};

/* Timer's Device Structure */

struct pic32mz_timer_dev_s
{
  FAR struct pic32mz_timer_ops_s *ops;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_timer_init
 *
 * Description:
 *   Power-up the timer and get its structure.
 *
 ****************************************************************************/

FAR struct pic32mz_timer_dev_s *pic32mz_timer_init(int timer);

/****************************************************************************
 * Name: pic32mz_timer_deinit
 *
 * Description:
 *   Power-down the timer and mark it as unused.
 *
 ****************************************************************************/

int pic32mz_timer_deinit(FAR struct pic32mz_timer_dev_s *dev);

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
int pic32mz_timer_initialize(FAR const char *devpath, int timer);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_TIMER_H */

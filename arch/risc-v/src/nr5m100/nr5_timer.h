/************************************************************************************
 * arch/risc-v/src/nr5m100/nr5_timer.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Modified for RISC-V:
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
 ************************************************************************************/

#ifndef __ARCH_RISCV_SRC_NR5M100_NR5_TIMER_H
#define __ARCH_RISCV_SRC_NR5M100_NR5_TIMER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/nr5m1xx_timer.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Helpers **************************************************************************/

#define NR5_TIMER_SETMODE(d,mode)       ((d)->ops->setmode(d,mode))
#define NR5_TIMER_SETCLOCK(d,freq)      ((d)->ops->setclock(d,freq))
#define NR5_TIMER_SETPERIOD(d,period)   ((d)->ops->setperiod(d,period))
#define NR5_TIMER_SETCOMPARE(d,ch,comp) ((d)->ops->setcompare(d,ch,comp))
#define NR5_TIMER_SETISR(d,hnd,arg,s)   ((d)->ops->setisr(d,hnd,arg,s))
#define NR5_TIMER_ENABLEINT(d,s)        ((d)->ops->enableint(d,s))
#define NR5_TIMER_DISABLEINT(d,s)       ((d)->ops->disableint(d,s))
#define NR5_TIMER_ACKINT(d,s)           ((d)->ops->ackint(d,s))

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Timer Modes of Operation */

typedef enum
{
  NR5_TIMER_MODE_UNUSED       = -1,

  /* One of the following */

  NR5_TIMER_MODE_DISABLED     = 0x0000,
  NR5_TIMER_MODE_UP           = 0x0001,
  NR5_TIMER_MODE_CONTINUOUS   = 0x0002,
  NR5_TIMER_MODE_UPDOWN       = 0x0003,

  /* One of the following */

  NR5_TIM_MODE_CK_SLOW        = 0x0000,
  NR5_TIM_MODE_CK_MED         = 0x0010,
  NR5_TIM_MODE_CK_SYS         = 0x0020,
  NR5_TIM_MODE_CK_EXT         = 0x0030,

} nr5_timer_mode_t;

/* Timer Operations */

struct nr5_timer_dev_s;

struct nr5_timer_ops_s
{
  /* Basic Timer Operations */

  int  (*setmode)(FAR struct nr5_timer_dev_s *dev, nr5_timer_mode_t mode);
  int  (*setclock)(FAR struct nr5_timer_dev_s *dev, uint32_t freq);
  void (*setperiod)(FAR struct nr5_timer_dev_s *dev, uint32_t period);

  /* Timer Interrupt Operations */

  int  (*setisr)(FAR struct nr5_timer_dev_s *dev, xcpt_t handler, void *arg, int source);
  void (*enableint)(FAR struct nr5_timer_dev_s *dev, int source);
  void (*disableint)(FAR struct nr5_timer_dev_s *dev, int source);
  void (*ackint)(FAR struct nr5_timer_dev_s *dev, int source);
};

/* Timer Device Structure */

struct nr5_timer_dev_s
{
  struct nr5_timer_ops_s *ops;
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* Power-up timer and get its structure */

FAR struct nr5_timer_dev_s *nr5_timer_init(int timer);

/* Power-down timer, mark it as unused */

int nr5_timer_deinit(FAR struct nr5_timer_dev_s *dev);

/****************************************************************************
 * Name: nr5_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device. This should be of the form /dev/timer0
 *   timer - the timer number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int nr5_timer_initialize(FAR const char *devpath, int timer);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_NR5M100_NR5_TIMER_H */


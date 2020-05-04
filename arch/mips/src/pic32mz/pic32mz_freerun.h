/****************************************************************************
 * arch/mips/src/pic32mz/chip/pic32mz_freerun.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_FREERUN_H
#define __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_FREERUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>

#include "pic32mz_timer.h"

#ifdef CONFIG_PIC32MZ_FREERUN

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The freerun client must allocate an instance of this structure and call
 * pic32mz_freerun_initialize() before using the freerun facilities.
 * The client should not access the contents of this structure directly
 * since the contents are subject to change.
 */

struct pic32mz_freerun_s
{
  uint8_t chan;                            /* The timer in use            */
  uint8_t width;                           /* Width of timer (16- or 32)  */
  bool running;                            /* True: the timer is running  */
  FAR struct pic32mz_timer_dev_s *timer;   /* PIC32MZ timer driver        */
  uint32_t freq;                           /* Timer's frequency (Hz)      */

#ifndef CONFIG_CLOCK_TIMEKEEPING
  uint32_t overflow;                       /* Timer's counter overflow    */
#endif

#ifdef CONFIG_CLOCK_TIMEKEEPING
  uint64_t counter_mask;                   /* Timer's count register mask */
#endif
};

/****************************************************************************
 * Public Data
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_freerun_initialize
 *
 * Description:
 *   Initialize the freerun timer wrapper
 *
 * Input Parameters:
 *   freerun    Caller allocated instance of the freerun state structure
 *   chan       Timer channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds. NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int pic32mz_freerun_initialize(struct pic32mz_freerun_s *freerun, int chan,
                               uint16_t resolution);

/****************************************************************************
 * Name: pic32mz_freerun_counter
 *
 * Description:
 *   Read the counter register of the free-running timer.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           pic32mz_freerun_initialize();
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_CLOCK_TIMEKEEPING

int pic32mz_freerun_counter(struct pic32mz_freerun_s *freerun,
                            struct timespec *ts);

#else /* CONFIG_CLOCK_TIMEKEEPING */

int pic32mz_freerun_counter(struct pic32mz_freerun_s *freerun,
                            uint64_t *counter);

#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Name: pic32mz_freerun_uninitialize
 *
 * Description:
 *   Stop the free-running timer and release all resources that it uses.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           pic32mz_freerun_initialize();
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int pic32mz_freerun_uninitialize(struct pic32mz_freerun_s *freerun);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_PIC32MZ_FREERUN */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_FREERUN_H */

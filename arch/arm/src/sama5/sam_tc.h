/****************************************************************************
 * arch/arm/src/sama5/sam_adc.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_TC_H
#define __ARCH_ARM_SRC_SAMA5_SAM_TC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"
#include "chip/sam_tc.h"

#if defined(CONFIG_SAMA5_TC0) || defined(CONFIG_SAMA5_TC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The timer/counter and channel arguments to sam_tc_allocate() */

#define TC_CHAN0     0
#define TC_CHAN1     1
#define TC_CHAN2     2
#define TC_CHAN3     3
#define TC_CHAN4     4
#define TC_CHAN5     5

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void *TCHANDLE;

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
 * Name: sam_tc_allocate
 *
 * Description:
 *   Configures a Timer Counter to operate in the given mode.  The timer is
 *   stopped after configuration and must be restarted with sam_tc_start().
 *   All the interrupts of the timer are also disabled.
 *
 * Input Parameters:
 *   channel TC channel number (see TC_CHANx definitions)
 *   mode    Operating mode (TC_CMR value).
 *
 * Returned Value:
 *   On success, a non-NULL handle value is returned.  This handle may be
 *   used with subsequent timer/counter interfaces to manage the timer.  A
 *   NULL handle value is returned on a failure.
 *   
 ****************************************************************************/

TCHANDLE sam_tc_allocate(int channel, int mode);

/****************************************************************************
 * Name: sam_tc_free
 *
 * Description:
 *   Release the handle previously allocated by sam_tc_allocate().
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *
 * Returned Value:
 *   None
 *   
 ****************************************************************************/

void sam_tc_free(TCHANDLE handle);

/****************************************************************************
 * Name: sam_tc_start
 *
 * Description:
 *   Reset and Start the TC Channel.  Enables the timer clock and performs a
 *   software reset to start the counting.
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *
 * Returned Value:
 *   
 ****************************************************************************/

void sam_tc_start(TCHANDLE handle);

/****************************************************************************
 * Name: sam_tc_stop
 *
 * Description:
 *   Stop TC Channel.  Disables the timer clock, stopping the counting.
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *
 * Returned Value:
 *   
 ****************************************************************************/

void sam_tc_stop(TCHANDLE handle);

/****************************************************************************
 * Name: sam_tc_divisor
 *
 * Description:
 *   Finds the best MCK divisor given the timer frequency and MCK.  The
 *   result is guaranteed to satisfy the following equation:
 *
 *     (MCK / (DIV * 65536)) <= freq <= (MCK / DIV)
 *
 *   with DIV being the highest possible value.
 *
 * Input Parameters:
 *
 *   frequency  Desired timer frequency.
 *   mck        Master clock frequency.
 *   div        Divisor value.
 *   tcclks     TCCLKS field value for divisor.
 *   boardmck   Board clock frequency.
 *
 * Returned Value:
 *   Zero (OK) if a proper divisor has been found, otherwise a negated errno
 *   value indicating the nature of the failure.
 *
 ****************************************************************************/

uint32_t sam_tc_divisor(uint32_t frequency, uint32_t mck, uint32_t *div,
                        uint32_t *tcclks, uint32_t boardmck);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SAMA5_TC0 || CONFIG_SAMA5_TC1 */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_TC_H */


/****************************************************************************
 * arch/arm/src/samv7/sam_tc.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_TC_H
#define __ARCH_ARM_SRC_SAMV7_SAM_TC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include "chip.h"
#include "hardware/sam_tc.h"

#if defined(CONFIG_SAMV7_TC0) || defined(CONFIG_SAMV7_TC1) || \
    defined(CONFIG_SAMV7_TC2) || defined(CONFIG_SAMV7_TC3)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The timer/counter and channel arguments to sam_tc_allocate() */

#define TC_CHAN0     0  /* TC0 */
#define TC_CHAN1     1
#define TC_CHAN2     2
#define TC_CHAN3     3  /* TC1 */
#define TC_CHAN4     4
#define TC_CHAN5     5
#define TC_CHAN6     6  /* TC2 */
#define TC_CHAN7     7
#define TC_CHAN8     8

/* Register identifier used with sam_tc_setregister */

#define TC_REGA      0
#define TC_REGB      1
#define TC_REGC      2

/* Timer debug is enabled if any timer client is enabled */

#ifndef CONFIG_DEBUG_TIMER_INFO
#  undef CONFIG_SAMV7_TC_REGDEBUG
#endif

#if !defined(CONFIG_SAMV7_TC_DEBUG) && defined(CONFIG_SAMV7_ADC) && defined(CONFIG_DEBUG_ANALOG)
#  define CONFIG_SAMV7_TC_DEBUG 1
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* An opaque handle used to represent a timer channel */

typedef void *TC_HANDLE;

/* Timer interrupt callback.  When a timer interrupt expires, the client will
 * receive:
 *
 *   tch - The handle that represents the timer state
 *   arg - An opaque argument provided when the interrupt was registered
 *   sr  - The value of the timer interrupt status register at the time
 *         that the interrupt occurred.
 */

typedef void (*tc_handler_t)(TC_HANDLE tch, void *arg, uint32_t sr);

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

TC_HANDLE sam_tc_allocate(int channel, int mode);

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

void sam_tc_free(TC_HANDLE handle);

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

void sam_tc_start(TC_HANDLE handle);

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

void sam_tc_stop(TC_HANDLE handle);

/****************************************************************************
 * Name: sam_tc_attach/sam_tc_detach
 *
 * Description:
 *   Attach or detach an interrupt handler to the timer interrupt.  The
 *   interrupt is detached if the handler argument is NULL.
 *
 * Input Parameters:
 *   handle  The handle that represents the timer state
 *   handler The interrupt handler that will be invoked when the interrupt
 *           condition occurs
 *   arg     An opaque argument that will be provided when the interrupt
 *           handler callback is executed.  Ignored if handler is NULL.
 *   mask    The value of the timer interrupt mask register that defines
 *           which interrupts should be disabled.  Ignored if handler is
 *           NULL.
 *
 * Returned Value:
 *   The address of the previous handler, if any.
 *
 ****************************************************************************/

tc_handler_t sam_tc_attach(TC_HANDLE handle, tc_handler_t handler,
                           void *arg, uint32_t mask);

#define sam_tc_detach(h)  sam_tc_attach(h, NULL, NULL, 0)

/****************************************************************************
 * Name: sam_tc_getpending
 *
 * Description:
 *   Return the current contents of the interrutp status register, clearing
 *   all pending interrupts.
 *
 * Input Parameters:
 *   handle  The handle that represents the timer state
 *
 * Returned Value:
 *   The value of the channel interrupt status register.
 *
 ****************************************************************************/

uint32_t sam_tc_getpending(TC_HANDLE handle);

/****************************************************************************
 * Name: sam_tc_setregister
 *
 * Description:
 *    Set TC_REGA, TC_REGB, or TC_REGC register.
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *   regid  One of {TC_REGA, TC_REGB, or TC_REGC}
 *   regval Then value to set in the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_tc_setregister(TC_HANDLE handle, int regid, uint32_t regval);

/****************************************************************************
 * Name: sam_tc_getregister
 *
 * Description:
 *    Get the current value of the TC_REGA, TC_REGB, or TC_REGC register.
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *   regid  One of {TC_REGA, TC_REGB, or TC_REGC}
 *
 * Returned Value:
 *   The value of the specified register.
 *
 ****************************************************************************/

uint32_t sam_tc_getregister(TC_HANDLE handle, int regid);

/****************************************************************************
 * Name: sam_tc_getcounter
 *
 * Description:
 *   Return the current value of the timer counter register
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *
 * Returned Value:
 *  The current value of the timer counter register for this channel.
 *
 ****************************************************************************/

uint32_t sam_tc_getcounter(TC_HANDLE handle);

/****************************************************************************
 * Name: sam_tc_infreq
 *
 * Description:
 *   Return the timer input frequency, that is, the MCK frequency divided
 *   down so that the timer/counter is driven within its maximum frequency.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *  The timer input frequency.
 *
 ****************************************************************************/

uint32_t sam_tc_infreq(void);

/****************************************************************************
 * Name: sam_tc_divfreq
 *
 * Description:
 *   Return the divided timer input frequency that is currently driving the
 *   the timer counter.
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *
 * Returned Value:
 *  The timer counter frequency.
 *
 ****************************************************************************/

uint32_t sam_tc_divfreq(TC_HANDLE handle);

/****************************************************************************
 * Name: sam_tc_clockselect
 *
 * Description:
 *   Finds the best MCK divisor given the timer frequency and MCK.  The
 *   result is guaranteed to satisfy the following equation:
 *
 *     (Ftcin / (div * 65536)) <= freq <= (Ftcin / div)
 *
 *   where:
 *     freq  - the desired frequency
 *     Ftcin - The timer/counter input frequency
 *     div   - With DIV being the highest possible value.
 *
 * Input Parameters:
 *   frequency  Desired timer frequency.
 *   tcclks     TCCLKS field value for divisor.
 *   actual     The actual frequency of the MCK
 *
 * Returned Value:
 *   Zero (OK) if a proper divisor has been found, otherwise a negated errno
 *   value indicating the nature of the failure.
 *
 ****************************************************************************/

int sam_tc_clockselect(uint32_t frequency, uint32_t *tcclks,
                       uint32_t *actual);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SAMV7_TC0 || CONFIG_SAMV7_TC1 || CONFIG_SAMV7_TC2 || CONFIG_SAMV7_TC3 */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_TC_H */

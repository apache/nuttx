/****************************************************************************
 * arch/arm/src/samd5e5/sam_tc.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_SAM_TC_H
#define __ARCH_ARM_SRC_SAMD5E5_SAM_TC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "sam_config.h"
#include "sam_port.h"
#include "sam_periphclks.h"
#include "hardware/sam_tc.h"
#include "hardware/sam_pinmap.h"
#include "semaphore.h"

#ifdef CONFIG_SAMD5E5_TC

/* Register identifier used with sam_tc_setregister */

#define TC_REGCC0      0
#define TC_REGCC1      1

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

struct tc_attr_s
{
  uint8_t tc;                 /* Identifies the TC peripheral */
  uint8_t irq;                /* Tc IRQ number */
  uint8_t coregen;            /* Source GCLK generator */
  port_pinset_t cc0;          /* Pin configuration CC0 - Compare/Capture Channel */
  port_pinset_t cc1;          /* Pin configuration CC1 - Compare/Capture Channel */
  uint32_t srcfreq;           /* Source clock frequency */
  uintptr_t base;             /* Base address of Tc registers */
};

/* Setup TC -> Add the timer register settings here. */

struct sam_tc_dev_s
{
  const struct tc_attr_s *attr;   /* Invariant attributes of TC device */
  uint32_t mode;                  /* TC mode */
  uint32_t prescaler;             /* TC prescaler */
  uint32_t prescsync;             /* TC prescsync */
  uint8_t wave;                   /* TC wave mode */
  uint16_t flags;                 /* Transfer flags */
  uint32_t freq;                  /* TC freq  */
  uint32_t duty;                  /* TC duty cycle */

  sem_t exclsem;              /* Only one thread can access at a time */
  sem_t waitsem;              /* Wait for TC  */

  bool initialized;        /* True: Timer data has been initialized */
  bool inuse;              /* True: channel is in use */
  tc_handler_t handler;    /* Attached interrupt handler */
  void *arg;               /* Interrupt handler argument */
};

/****************************************************************************
 * Name: sam_tc_configure
 *
 * Description:
 *   Configure the interrupt edge sensitivity in CONFIGn register of the
 *   tc.  The interrupt will be enabled at the tc (but not at the NVIC).
 *
 * Input Parameters:
 *   eirq    - Pin to be configured (0..15)
 *   pinset  - Configuration of the pin
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int sam_tc_configure(uint8_t tcrq, port_pinset_t pinset);

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

TC_HANDLE sam_tc_allocate(int channel, int frequency);

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
 *   Return the current contents of the interrupt status register, clearing
 *   all pending interrupts.
 *
 * Input Parameters:
 *   handle  The handle that represents the timer state
 *
 * Returned Value:
 *   The value of the channel interrupt status register.
 *
 ****************************************************************************/

uint8_t sam_tc_getpending(TC_HANDLE handle);

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
 * Name: sam_tc_divisor
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
 *   div        Divisor value.
 *   tcclks     TCCLKS field value for divisor.
 *
 * Returned Value:
 *   Zero (OK) if a proper divisor has been found, otherwise a negated errno
 *   value indicating the nature of the failure.
 *
 ****************************************************************************/

int sam_tc_divisor(struct sam_tc_dev_s *tc, uint32_t frequency,
                   uint32_t *div, uint32_t *tcclks);

uint32_t sam_tc_getctrla(TC_HANDLE handle);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* CONFIG_SAMD5E5_TC */
#endif /* __ARCH_ARM_SRC_SAMD5E5_SAM_TC_H */

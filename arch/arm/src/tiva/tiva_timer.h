/****************************************************************************
 * arch/arm/src/tiva/tiva_timer.h
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_TIMER_H
#define __ARCH_ARM_SRC_TIVA_TIVA_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/tiva/chip.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/tiva_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Make sure that no timers are enabled that are not supported by the
 * architecture.
 */

#if TIVA_NTIMERS < 8
#  undef CONFIG_TIVA_TIMER7
#  if TIVA_NTIMERS < 7
#    undef CONFIG_TIVA_TIMER6
#    if TIVA_NTIMERS < 6
#      undef CONFIG_TIVA_TIMER5
#      if TIVA_NTIMERS < 5
#        undef CONFIG_TIVA_TIMER4
#        if TIVA_NTIMERS < 4
#          undef CONFIG_TIVA_TIMER3
#          if TIVA_NTIMERS < 3
#            undef CONFIG_TIVA_TIMER2
#            if TIVA_NTIMERS < 2
#              undef CONFIG_TIVA_TIMER1
#              if TIVA_NTIMERS < 1
#                undef CONFIG_TIVA_TIMER0
#              endif
#            endif
#          endif
#        endif
#      endif
#    endif
#  endif
#endif

/* Used with the synca and syncb fields of struct tiva_timerconfig_s */

#define TIMER_SYNC(n)         (1 << (n))

/* Identifies 16-bit timer A and timer B.  In contexts where an index is
 * needed, the 32-bit timer is equivalent to the timer A index.
 */

#define TIMER32               0
#define TIMER16A              0
#define TIMER16B              1

/* Flags bit definitions in configuration structures.  NOTE: not all flags
 * apply in all timer modes.  Applicable modes noted with:
 *
 *   a. 16-bit one shot timer
 *   b. 16-bit periodic timer
 *   c. 32-bit one shot timer
 *   d. 32-bit periodic timer
 *   e. 32-bit RTC timer
 *   f. 16-bit PWM timer
 */

#define TIMER_FLAG_COUNTUP    (1 << 0)  /* Bit 0: Count up (abcd) */
#define TIMER_FLAG_ADCTIMEOUT (1 << 1)  /* Bit 1: Generate ADC trigger on
                                         * timeout (abcd) */
#define TIMER_FLAG_ADCRTCM    (1 << 2)  /* Bit 2: Generate ADC trigger on
                                         * RTC match (e) */
#define TIMER_FLAG_ADCMATCH   (1 << 3)  /* Bit 3: Generate ADC trigger on
                                         * match (abcd) */
#define TIMER_FLAG_DMATIMEOUT (1 << 4)  /* Bit 4: Generate uDMA trigger on
                                         * timeout (abcd) */
#define TIMER_FLAG_DMARTCM    (1 << 5)  /* Bit 5: Generate uDMA trigger on
                                         * RTC match (e) */
#define TIMER_FLAG_DMAMATCH   (1 << 6)  /* Bit 6: Generate uDMA trigger on
                                         * match (abcd) */
#define TIMER_FLAG_PWMINVERT  (1 << 7)  /* Bit 7: Invert PWM signal (f) */
#define TIMER_FLAG_PWMINTPOS  (1 << 8)  /* Bit 8: Interrupt on PWM positive
                                         * edge (f) */
#define TIMER_FLAG_PWMINTNEG  (1 << 9)  /* Bit 9: Interrupt on PWM negative
                                         * edge (f) */
#define TIMER_FLAG_STALL      (1 << 10) /* Bit 10: Stall timer when CPU paused
                                         * in debug breakpoint (abcdef) */

#define TIMER_ISCOUNTUP(c)    ((((c)->flags) & TIMER_FLAG_COUNTUP) != 0)
#define TIMER_ISADCTIMEOUT(c) ((((c)->flags) & TIMER_FLAG_ADCTIMEOUT) != 0)
#define TIMER_ISADCRTCM(c)    ((((c)->flags) & TIMER_FLAG_ADCRTCM) != 0)
#define TIMER_ISADCMATCH(c)   ((((c)->flags) & TIMER_FLAG_ADCMATCH) != 0)
#define TIMER_ISDMATIMEOUT(c) ((((c)->flags) & TIMER_FLAG_DMATIMEOUT) != 0)
#define TIMER_ISDMARTCM(c)    ((((c)->flags) & TIMER_FLAG_DMARTCM) != 0)
#define TIMER_ISDMAMATCH(c)   ((((c)->flags) & TIMER_FLAG_DMAMATCH) != 0)
#define TIMER_ISPWMINVERT(c)  ((((c)->flags) & TIMER_FLAG_PWMINVERT) != 0)
#define TIMER_ISPWMINTPOS(c)  ((((c)->flags) & TIMER_FLAG_PWMINTPOS) != 0)
#define TIMER_ISPWMINTNEG(c)  ((((c)->flags) & TIMER_FLAG_PWMINTNEG) != 0)
#define TIMER_ISSTALL         ((((c)->flags) & TIMER_FLAG_STALL) != 0)

#define TIMER_ISPWMINTBOTH(c) (TIMER_ISPWMINTPOS(c) && TIMER_ISPWMINTNEG(c))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This enumeration identifies all supported 32-bit timer modes of operation
 *
 * NOTES:
 * - TIMER32_MODE_RTC: The input clock on a CCP0 input is required to be
 *   32.768 KHz in RTC mode. The clock signal is then divided down to a 1-Hz
 *   rate and is passed along to the input of the counter.
 */

enum tiva_timer32mode_e
{
  TIMER16_MODE = 0,              /* Use 16-bit timers, not 32-bit timer */
  TIMER32_MODE_ONESHOT,          /* 32-bit programmable one-shot timer */
  TIMER32_MODE_PERIODIC,         /* 32-bit programmable periodic timer */
  TIMER32_MODE_RTC               /* 32-bit RTC with external 32.768-KHz input */
};

/* This enumeration identifies all supported 16-bit timer A/B modes of
 * operation.
 */

enum tiva_timer16mode_e
{
  TIMER16_MODE_NONE = 0,         /* 16-bit timer not used */
  TIMER16_MODE_ONESHOT,          /* 16-bit programmable one-shot timer */
  TIMER16_MODE_PERIODIC,         /* 16-bit programmable periodic timer */
  TIMER16_MODE_COUNT_CAPTURE,    /* 16-bit input edge-count capture mode w/8-bit prescaler */
  TIMER16_MODE_TIME_CAPTURE,     /* 16-bit input time capture mode w/8-bit prescaler */
  TIMER16_MODE_PWM               /* 16-bit PWM output mode w/8-bit prescaler */
};

/* This type represents the opaque handler returned by
 * tiva_gptm_configure()
 */

typedef void *TIMER_HANDLE;

#ifdef CONFIG_TIVA_TIMER_32BIT
/* This type describes the 32-bit timer interrupt handler.
 *
 * Input Parameters:
 *   handle - The same value as returned by tiva_gptm_configure()
 *   arg    - The same value provided in struct tiva_timer32config_s
 *   status - The value of the GPTM masked status register that caused the
 *            interrupt
 */

struct tiva_gptm32config_s;
typedef void (*timer32_handler_t)(TIMER_HANDLE handle, void *arg,
                                  uint32_t status);

/* This structure describes the configuration of one 32-bit timer */

struct tiva_timer32config_s
{
  uint32_t flags;                /* See TIMER_FLAG_* definitions */
  timer32_handler_t handler;     /* Non-NULL: Interrupts will be enabled
                                  * and forwarded to this function */
  void *arg;                     /* Argument that accompanies the handler
                                  * callback.
                                  */

  /* Mode-specific parameters may follow */
};
#endif

#ifdef CONFIG_TIVA_TIMER_16BIT
/* This type describes the 16-bit timer interrupt handler
 *
 * Input Parameters:
 *   handle - The same value as returned by tiva_gptm_configure()
 *   arg    - The same value provided in struct tiva_timer16config_s
 *   status - The value of the GPTM masked status register that caused the
 *            interrupt.
 *   tmndx  - Either TIMER16A or TIMER16B.  This may be useful in the
 *            event that the same handler is used for Timer A and B.
 */

struct tiva_gptm16config_s;
typedef void (*timer16_handler_t)(TIMER_HANDLE handle, void *arg,
                                  uint32_t status, int tmndx);

/* This structure describes the configuration of one 16-bit timer A/B */

struct tiva_timer16config_s
{
  uint8_t mode;                  /* See enum tiva_timer16mode_e */
  uint32_t flags;                /* See TIMER_FLAG_* definitions */
  timer16_handler_t handler;     /* Non-NULL: Interrupts will be enabled
                                  * and forwarded to this function */
  void *arg;                     /* Argument that accompanies the handler
                                  * callback.
                                  */

  /* Mode-specific parameters may follow */
};
#endif

/* This structure describes usage of both timers on a GPTIM module */

struct tiva_gptmconfig_s
{
  uint8_t gptm;                  /* GPTM number */
  uint8_t mode;                  /* See enum tiva_timer32mode_e */
  bool alternate;                /* False: Use SysClk; True: Use alternate clock source */
};

#ifdef CONFIG_TIVA_TIMER_32BIT
/* This structure is cast compatible with struct tiva_gptmconfig_s and
 * describes usage of the single 32-bit timers on a GPTM module.
 */

struct tiva_gptm32config_s
{
  struct tiva_gptmconfig_s cmn;
  struct tiva_timer32config_s config;
};
#endif

/* This structure is cast compatible with struct tiva_gptmconfig_s and
 * describes usage of both 16-bit half-timers A/B on a GPTM module.
 */

#ifdef CONFIG_TIVA_TIMER_16BIT
struct tiva_gptm16config_s
{
  struct tiva_gptmconfig_s cmn;
  struct tiva_timer16config_s config[2];
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_gptm_configure
 *
 * Description:
 *   Configure a general purpose timer module to operate in the provided
 *   modes.
 *
 * Input Parameters:
 *   gptm - Describes the configure of the GPTM timer resources
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned that can be used with the
 *   other timer interfaces.  NULL is returned on any failure to initialize
 *   the timer.
 *
 ****************************************************************************/

TIMER_HANDLE tiva_gptm_configure(const struct tiva_gptmconfig_s *gptm);

/****************************************************************************
 * Name: tiva_gptm_release
 *
 * Description:
 *   Release resources held by the timer instance.  After this function is
 *   called, the timer handle is invalid and must not be used further.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tiva_gptm_release(TIMER_HANDLE handle);

/****************************************************************************
 * Name: tiva_gptm_putreg
 *
 * Description:
 *   This function permits setting of any timer register by its offset into
 *   the timer block.  Its primary purpose is to support inline functions
 *   defined in this header file.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   offset - The offset to the timer register to be written
 *   value  - The value to write to the timer register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tiva_gptm_putreg(TIMER_HANDLE handle, unsigned int offset,
                      uint32_t value);

/****************************************************************************
 * Name: tiva_gptm_getreg
 *
 * Description:
 *   This function permits reading of any timer register by its offset into
 *   the timer block.  Its primary purpose is to support inline functions
 *   defined in this header file.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   offset - The offset to the timer register to be written
 *
 * Returned Value:
 *   The 32-bit value read at the provided offset into the timer register
 *   base address.
 *
 ****************************************************************************/

uint32_t tiva_gptm_getreg(TIMER_HANDLE handle, unsigned int offset);

/****************************************************************************
 * Name: tiva_gptm_modifyreg
 *
 * Description:
 *   This function permits atomic of any timer register by its offset into
 *   the timer block.  Its primary purpose is to support inline functions
 *   defined in this header file.
 *
 * Input Parameters:
 *   handle  - The handle value returned  by tiva_gptm_configure()
 *   offset  - The offset to the timer register to be written
 *   clrbits - The collection of bits to be cleared in the register
 *   setbits - The collection of bits to be set in the register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tiva_gptm_modifyreg(TIMER_HANDLE handle, unsigned int offset,
                         uint32_t clrbits, uint32_t setbits);

/****************************************************************************
 * Name: tiva_timer32_start
 *
 * Description:
 *   After tiva_gptm_configure() has been called to configure a 32-bit timer,
 *   this function must be called to start the timer(s).
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_32BIT
void tiva_timer32_start(TIMER_HANDLE handle);
#endif

/****************************************************************************
 * Name: tiva_timer16_start
 *
 * Description:
 *   After tiva_gptm_configure() has been called to configure 16-bit
 *   timer(s), this function must be called to start one 16-bit timer.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_16BIT
void tiva_timer16_start(TIMER_HANDLE handle, int tmndx);

#  define tiva_timer16a_start(h) tiva_timer16_start(h, TIMER16A)
#  define tiva_timer16b_start(h) tiva_timer16_start(h, TIMER16B)
#endif

/****************************************************************************
 * Name: tiva_timer32_stop
 *
 * Description:
 *   After tiva_timer32_start() has been called to start a 32-bit timer,
 *   this function may be called to stop the timer.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_32BIT
void tiva_timer32_stop(TIMER_HANDLE handle);
#endif

/****************************************************************************
 * Name: tiva_timer16_stop
 *
 * Description:
 *   After tiva_timer32_start() has been called to start a 16-bit timer,
 *   this function may be called to stop the timer.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_16BIT
void tiva_timer16_stop(TIMER_HANDLE handle, int tmndx);

#  define tiva_timer16a_stop(h) tiva_timer16_stop(h, TIMER16A)
#  define tiva_timer16b_stop(h) tiva_timer16_stop(h, TIMER16B)
#endif

/****************************************************************************
 * Name: tiva_timer32_counter
 *
 * Description:
 *   Return the current 32-bit counter value of the 32-bit timer.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *
 * Returned Value:
 *   The current 32-bit counter value.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_32BIT
static inline uint32_t tiva_timer32_counter(TIMER_HANDLE handle)
{
  return tiva_gptm_getreg(handle, TIVA_TIMER_TAR_OFFSET);
}
#endif

/****************************************************************************
 * Name: tiva_timer16_counter
 *
 * Description:
 *   Return the current 24-bit counter value of the 16-bit timer.
 *
 *   The timer 24-bit value is the 16-bit counter value AND the 8-bit
 *   prescaler value.  From the caller's point of view the match value is
 *   the 24-bit timer at the timer input clock frequency.
 *
 *   When counting down in periodic modes, the prescaler contains the
 *   least-significant bits of the count. When counting up, the prescaler
 *   holds the most-significant bits of the count.  But the caller is
 *   protected from this complexity.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   The current 24-bit counter value.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_16BIT
uint32_t tiva_timer16_counter(TIMER_HANDLE handle, int tmndx);

#  define tiva_timer16a_counter(h) tiva_timer16_counter(h, TIMER16A)
#  define tiva_timer16b_counter(h) tiva_timer16_counter(h, TIMER16B)
#endif

/****************************************************************************
 * Name: tiva_timer32_setinterval
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   load value of a 32-bit timer.
 *
 *   It the timer is configured as a 32-bit one-shot or periodic timer, then
 *   then function will also enable timeout interrupts.
 *
 *   NOTE: As of this writing, there is no interface to disable the timeout
 *   interrupts once they have been enabled.
 *
 * Input Parameters:
 *   handle   - The handle value returned  by tiva_gptm_configure()
 *   interval - The value to write to the timer interval load register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_32BIT
void tiva_timer32_setinterval(TIMER_HANDLE handle, uint32_t interval);
#endif

/****************************************************************************
 * Name: tiva_timer16_setinterval
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   load value of a 16-bit timer.
 *
 *   It the timer is configured as a 16-bit one-shot or periodic timer, then
 *   then function will also enable timeout interrupts.
 *
 *   NOTE: As of this writing, there is no interface to disable the timeout
 *   interrupts once they have been enabled.
 *
 * Input Parameters:
 *   handle   - The handle value returned  by tiva_gptm_configure()
 *   interval - The value to write to the timer interval load register
 *   tmndx    - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_16BIT
void tiva_timer16_setinterval(TIMER_HANDLE handle, uint16_t interval,
                              int tmndx);

#  define tiva_timer16a_setinterval(h,l) tiva_timer16_setinterval(h,l,TIMER16A)
#  define tiva_timer16b_setinterval(h,l) tiva_timer16_setinterval(h,l,TIMER16B)
#endif

/****************************************************************************
 * Name: tiva_timer32_remaining
 *
 * Description:
 *   Get the time remaining before a one-shot or periodic 32-bit timer
 *   expires.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure().
 *
 * Returned Value:
 *   Time remaining until the next timeout interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER32_PERIODIC
uint32_t tiva_timer32_remaining(TIMER_HANDLE handle);
#endif

/****************************************************************************
 * Name: tiva_timer16_remaining
 *
 * Description:
 *   Get the time remaining before a one-shot or periodic 16-bit timer
 *   expires.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure().
 *
 * Returned Value:
 *   Time remaining until the next timeout interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER16_PERIODIC
/* To be provided */
#endif

/****************************************************************************
 * Name: tiva_timer32_absmatch
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   match value of a 32-bit timer.  This function sets the match register
 *   the absolute value specified.
 *
 * Input Parameters:
 *   handle   - The handle value returned  by tiva_gptm_configure()
 *   absmatch - The absolute value to write to the timer match register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_32BIT
static inline void tiva_timer32_absmatch(TIMER_HANDLE handle,
                                         uint32_t absmatch)
{
  tiva_gptm_putreg(handle, TIVA_TIMER_TAMATCHR_OFFSET, absmatch);
}
#endif

/****************************************************************************
 * Name: tiva_timer16_absmatch
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   match value of a 16-bit timer.  This function sets the match register
 *   the absolute value specified.
 *
 * Input Parameters:
 *   handle   - The handle value returned  by tiva_gptm_configure()
 *   absmatch - The value to write to the timer match register
 *   tmndx    - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_16BIT
static inline void tiva_timer16_absmatch(TIMER_HANDLE handle,
                                         uint16_t absmatch, int tmndx)
{
  unsigned int regoffset =
    tmndx ? TIVA_TIMER_TBMATCHR_OFFSET : TIVA_TIMER_TAMATCHR_OFFSET;

  tiva_gptm_putreg(handle, regoffset, absmatch);
}

static inline void tiva_timer16a_absmatch(TIMER_HANDLE handle,
                                          uint16_t absmatch)
{
  tiva_gptm_putreg(handle, TIVA_TIMER_TAMATCHR_OFFSET, absmatch);
}

static inline void tiva_timer16b_absmatch(TIMER_HANDLE handle,
                                          uint16_t absmatch)
{
  tiva_gptm_putreg(handle, TIVA_TIMER_TBMATCHR_OFFSET, absmatch);
}
#endif

/****************************************************************************
 * Name: tiva_rtc_settime
 *
 * Description:
 *   Set the 32-bit RTC timer counter.  When RTC mode is selected for the
 *   first time after reset, the counter is loaded with a value of 1. All
 *   subsequent load values must be written to the concatenated GPTM Timer A
 *   Interval Load (GPTMTAILR) registers. If the GPTMTnILR register is
 *   loaded with a new value, the counter begins counting at that value
 *   and rolls over at the fixed value of 0xffffffff.
 *
 * Input Parameters:
 *   handle  - The handle value returned  by tiva_gptm_configure()
 *   newtime - The new RTC time (seconds)
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER32_RTC
static inline void tiva_rtc_settime(TIMER_HANDLE handle, uint32_t newtime)
{
  tiva_gptm_putreg(handle, TIVA_TIMER_TAILR_OFFSET, newtime);
}
#endif

/****************************************************************************
 * Name: tiva_rtc_setalarm
 *
 * Description:
 *   Setup to receive an interrupt when the RTC counter equals a match time
 *   value.  This function sets the match register to the current timer
 *   counter register value PLUS the relative value provided.  The relative
 *   value then is an offset in seconds from the current time.
 *
 *   If an interrupt handler is provided, then the RTC match interrupt will
 *   be enabled.  A single RTC match interrupt will be generated; further
 *   RTC match interrupts will be disabled.
 *
 *   NOTE: Use of this function is only meaningful for a a 32-bit RTC time.
 *   NOTE: An interrupt handler address must provided in the configuration.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   delay  - A relative time in the future (seconds)
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER32_RTC
void tiva_rtc_setalarm(TIMER_HANDLE handle, uint32_t delay);
#endif

/****************************************************************************
 * Name: tiva_timer32_relmatch
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   match value of a 32-bit timer.  This function sets the match register
 *   to the current timer counter register value PLUS the relative value
 *   provided.  The relative value then is some the offset to some timer
 *   counter value in the future.
 *
 *   If an interrupt handler is provided, then the match interrupt will also
 *   be enabled.  A single match interrupt will be generated; further match
 *   interrupts will be disabled.
 *
 *   NOTE: Use of this function is only meaningful for a 32-bit free-
 *   running, periodic timer.
 *
 *   WARNING: For free-running timers, the relative match value should be
 *   sufficiently far in the future to avoid race conditions.
 *
 * Input Parameters:
 *   handle   - The handle value returned  by tiva_gptm_configure()
 *   relmatch - The value to write to the timer match register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER32_PERIODIC
void tiva_timer32_relmatch(TIMER_HANDLE handle, uint32_t relmatch);
#endif

/****************************************************************************
 * Name: tiva_timer16_relmatch
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   match value of a 16-bit timer.  This function sets the match register
 *   to the current timer counter register value PLUS the relative value
 *   provided.  The relative value then is some the offset to some timer
 *   counter value in the future.
 *
 *   If an interrupt handler is provided, then the match interrupt will also
 *   be enabled.  A single match interrupt will be generated; further match
 *   interrupts will be disabled.
 *
 *   NOTE: Use of this function is only meaningful for a 16-bit free-
 *   running, periodic timer.
 *
 *   NOTE: The relmatch input is a really a 24-bit value; it is the 16-bit
 *   match counter match value AND the 8-bit prescaler match value.  From
 *   the caller's point of view the match value is the 24-bit time to match
 *   driven at the timer input clock frequency.
 *
 *   When counting down in periodic modes, the prescaler contains the
 *   least-significant bits of the count. When counting up, the prescaler
 *   holds the most-significant bits of the count.  But the caller is
 *   protected from this complexity.
 *
 *   WARNING: For free-running timers, the relative match value should be
 *   sufficiently far in the future to avoid race conditions.
 *
 * Input Parameters:
 *   handle   - The handle value returned  by tiva_gptm_configure()
 *   relmatch - The value to write to the timer match register
 *   tmndx    - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER16_PERIODIC
void tiva_timer16_relmatch(TIMER_HANDLE handle, uint32_t relmatch,
                           int tmndx);

#  define tiva_timer16a_relmatch(h,r) tiva_timer16_relmatch(h,r,TIMER16A)
#  define tiva_timer16b_relmatch(h,r) tiva_timer16_relmatch(h,r,TIMER16B)
#endif

/****************************************************************************
 * Name: tiva_timer16pwm_setperiodduty
 *
 * Description:
 *   Set the period and initial duty cycle for a 16-bit timer operating in
 *   PWM mode. Also, enable interrupts if a handler is provided. The timer
 *   is not started until tiva_timer16_start() is called.
 *
 * Input Parameters:
 *   handle - The handle value returned by tiva_gptm_configure()
 *   period - The PWM period, a 24-bit value.
 *   duty   - The initial PWM duty cycle, a 24-bit value.
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER16_PWM
void tiva_timer16pwm_setperiodduty(TIMER_HANDLE handle, uint32_t period,
                                   uint32_t duty, int tmndx);

#  define tiva_timer16apwm_setperiodduty(h,p,d) tiva_timer16pwm_setperiodduty(h,p,d,TIMER16A)
#  define tiva_timer16bpwm_setperiodduty(h,p,d) tiva_timer16pwm_setperiodduty(h,p,d,TIMER16B)
#endif

/****************************************************************************
 * Name: tiva_timer16pwm_setduty
 *
 * Description:
 *   Update the duty cycle for a 16-bit timer operating in PWM mode.
 *
 * Input Parameters:
 *   handle - The handle value returned by tiva_gptm_configure()
 *   duty   - The initial PWM duty cycle, a 24-bit value.
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER16_PWM
void tiva_timer16pwm_setduty(TIMER_HANDLE handle, uint32_t duty, int tmndx);

#  define tiva_timer16apwm_setduty(h,d) tiva_timer16pwm_setduty(h,d,TIMER16A)
#  define tiva_timer16bpwm_setduty(h,d) tiva_timer16pwm_setduty(h,d,TIMER16B)
#endif

/****************************************************************************
 * Name: tiva_gptm0_synchronize
 *
 * Description:
 *   Trigger timers from GPTM0 output. This is part of the timer
 *   configuration logic and should be called before timers are enabled.
 *
 * Input Parameters:
 *   sync   - The value to write to the GPTM0 SYNC register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVER_TIMER0
static inline void tiva_gptm0_synchronize(uint32_t sync)
{
  putreg32(sync, TIVA_TIMER0_SYNC);
}
#endif

/****************************************************************************
 * Name: tiva_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 *   NOTES:
 *   1. Only 32-bit periodic timers are supported.
 *   2. Timeout interrupts are disabled until tiva_timer32_setinterval() is
 *      called.
 *   3. Match interrupts are disabled until tiva_timer32_relmatch() is
 *      called.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *     form /dev/timer0
 *   config - 32-bit timer configuration values.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int tiva_timer_initialize(const char *devpath,
                          struct tiva_gptm32config_s *config);
#endif

#endif /* __ARCH_ARM_SRC_TIVA_TIVA_TIMER_H */

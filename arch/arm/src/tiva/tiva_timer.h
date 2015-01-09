/****************************************************************************
 * arch/arm/src/tiva/tiva_timer.h
 *
 *   Copyright (C) 201r Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_TIMER_H
#define __ARCH_ARM_SRC_TIVA_TIVA_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/tiva/chip.h>

#include "chip/tiva_timer.h"

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

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This enumeration identifies all supported 32-bit timer modes of operation */

enum tiva_timer32mode_e
{
  TIMER16_MODE = 0,              /* Use 16-bit timers, not 32-bit timer */
  TIMER32_MODE_ONESHOT,          /* 32-bit programmable one-shot timer */
  TIMER32_MODE_PERIODIC,         /* 32-bit programmable periodic timer */
  TIMER32_MODE_RTC               /* 32-bit RTC with external 32.768-KHz input */
};

/* This enumeration identifies all supported 16-bit timer A/Bmodes of
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

/* This type represents the opaque handler returned by tiva_gptm_configure() */

typedef FAR void *TIMER_HANDLE;

/* This type describes the 32-bit timer interrupt handler */

struct tiva_gptm32config_s;
typedef void (*timer32_handler_t)(TIMER_HANDLE handle,
                                  const struct tiva_gptm32config_s *config);

/* This structure describes the configuration of one 32-bit timer */

struct tiva_timer32config_s
{
  bool countup;                  /* True: Count up; False: Count down */
  timer32_handler_t handler;     /* Non-NULL: Interrupts will be enabled
                                  * and forwarded to this function */
  /* TODO:  Add fields to support ADC trigger events */

  /* Mode-specific parameters */

  union
  {
    /* 32-bit programmable one-shot or periodic timer */

    struct
    {
      uint32_t interval;         /* Value for interval load register */
    } periodic;

    /* 32-bit RTC with external 32.768-KHz input */

    struct
    {
    } rtc;
  } u;
};

/* This type describes the 16-bit timer interrupt handler */

struct tiva_gptm16config_s;
typedef void (*timer16_handler_t)(TIMER_HANDLE handle,
                                  const struct tiva_gptm16config_s *config,
                                  int tmndx);

/* This structure describes the configuration of one 16-bit timer A/B */

struct tiva_timer16config_s
{
  uint8_t mode;                  /* See enum tiva_timermode_e */
  bool countup;                  /* True: Count up; False: Count down */
  timer16_handler_t handler;     /* Non-NULL: Interrupts will be enabled
                                  * and forwarded to this function */
  /* TODO:  Add fields to support ADC trigger events */

  /* Mode-specific parameters */

  union
  {
    /* 16-bit programmable one-shot or periodic timer */

    struct
    {
      uint16_t interval;         /* Value for interval load register */
    } periodic;

    /* 16-bit input edge-count capture mode w/8-bit prescaler */

    struct
    {
    } count;

    /* 16-bit input time capture mode w/8-bit prescaler */

    struct
    {
    } time;

    /* 16-bit PWM output mode w/8-bit prescaler */

    struct
    {
    } pwm;
  } u;
};

/* This structure describes usage of both timers on a GPTIM module */

struct tiva_gptmconfig_s
{
  uint8_t gptm;                  /* GPTM number */
  uint8_t mode;                  /* See enum tiva_timer32mode_e */
  bool alternate;                /* False: Use SysClk; True: Use alternate clock source */
};

/* This structure is cast compatible with struct tiva_gptmconfig_s and
 * describes usage of the single 32-bit timers on a GPTM module.
 */

struct tiva_gptm32config_s
{
  struct tiva_gptmconfig_s cmn;
  struct tiva_timer32config_s config;
};

/* This structure is cast compatible with struct tiva_gptmconfig_s and
 * describes usage of both bit-bit timers A/B on a GPTM module.
 */

struct tiva_gptm16config_s
{
  struct tiva_gptmconfig_s cmn;
  struct tiva_timer16config_s config[2];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_gptm_configure
 *
 * Description:
 *   Configure a general purpose timer module to operate in the provided
 *   modes.
 *
 ****************************************************************************/

TIMER_HANDLE tiva_gptm_configure(const struct tiva_gptmconfig_s *gptm);

/****************************************************************************
 * Name: tiva_gptm_putreg
 *
 * Description:
 *   This function permits setting of any timer register by its offset into
 *   the timer block.  Its primary purpose is to support inline functions
 *   defined in this header file.
 *
 ****************************************************************************/

void tiva_gptm_putreg(TIMER_HANDLE handle, unsigned int offset, uint32_t value);

/****************************************************************************
 * Name: tiva_gptm_getreg
 *
 * Description:
 *   This function permits reading of any timer register by its offset into
 *   the timer block.  Its primary purpose is to support inline functions
 *   defined in this header file.
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
 ****************************************************************************/

void tiva_timer32_start(TIMER_HANDLE handle);

/****************************************************************************
 * Name: tiva_timer16_start
 *
 * Description:
 *   After tiva_gptm_configure() has been called to configure 16-bit timer(s),
 *   this function must be called to start one 16-bit timer.
 *
 ****************************************************************************/

void tiva_timer16_start(TIMER_HANDLE handle, int tmndx);

#define tiva_timer16a_start(h) tiva_timer16_start(h, TIMER16A)
#define tiva_timer16b_start(h) tiva_timer16_start(h, TIMER16B)

/****************************************************************************
 * Name: tiva_timer32_stop
 *
 * Description:
 *   After tiva_timer32_start() has been called to start a 32-bit timer,
 *   this function may be called to stop the timer.
 *
 ****************************************************************************/

void tiva_timer32_stop(TIMER_HANDLE handle);

/****************************************************************************
 * Name: tiva_timer16_stop
 *
 * Description:
 *   After tiva_timer32_start() has been called to start a 16-bit timer,
 *   this function may be called to stop the timer.
 *
 ****************************************************************************/

void tiva_timer16_stop(TIMER_HANDLE handle, int tmndx);

#define tiva_timer16a_stop(h) tiva_timer16_stop(h, TIMER16A)
#define tiva_timer16b_stop(h) tiva_timer16_stop(h, TIMER16B)

/****************************************************************************
 * Name: tiva_timer32_setload
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   load value of a 32-bit timer.
 *
 ****************************************************************************/

static inline void tiva_timer32_setload(TIMER_HANDLE handle, uint32_t load)
{
  tiva_gptm_putreg(handle, TIVA_TIMER_TAILR_OFFSET, load);
}

/****************************************************************************
 * Name: tiva_timer16_setload
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   load value of a 16-bit timer.
 *
 ****************************************************************************/

static inline void tiva_timer16_setload(TIMER_HANDLE handle, uint16_t load,
                                        int tmndx)
{
  unsigned int regoffset =
    tmndx ? TIVA_TIMER_TBILR_OFFSET : TIVA_TIMER_TAILR_OFFSET;

  tiva_gptm_putreg(handle, regoffset, load);
}

static inline void tiva_timer16a_setload(TIMER_HANDLE handle, uint16_t load)
{
  tiva_gptm_putreg(handle, TIVA_TIMER_TAILR_OFFSET, load);
}

static inline void tiva_timer16b_setload(TIMER_HANDLE handle, uint16_t load)
{
  tiva_gptm_putreg(handle, TIVA_TIMER_TBILR_OFFSET, load);
}

/****************************************************************************
 * Name: tiva_timer32_setmatch
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   match value of a 32-bit timer.
 *
 ****************************************************************************/

static inline void tiva_timer32_setmatch(TIMER_HANDLE handle, uint32_t match)
{
  tiva_gptm_putreg(handle, TIVA_TIMER0_TAMATCHR, match);
}

/****************************************************************************
 * Name: tiva_timer16_setmatch
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   match value of a 16-bit timer.
 *
 ****************************************************************************/

static inline void tiva_timer16_setmatch(TIMER_HANDLE handle, uint16_t match,
                                        int tmndx)
{
  unsigned int regoffset =
    tmndx ? TIVA_TIMER0_TBMATCHR : TIVA_TIMER0_TAMATCHR;

  tiva_gptm_putreg(handle, regoffset, match);
}

static inline void tiva_timer16a_setmatch(TIMER_HANDLE handle, uint16_t match)
{
  tiva_gptm_putreg(handle, TIVA_TIMER0_TAMATCHR, match);
}

static inline void tiva_timer16b_setmatch(TIMER_HANDLE handle, uint16_t match)
{
  tiva_gptm_putreg(handle, TIVA_TIMER0_TBMATCHR, match);
}

/****************************************************************************
 * Name: tiva_gptm0_synchronize
 *
 * Description:
 *   Trigger timers from GPTM0 output. This is part of the timer
 *   configuration logic and should be called before timers are enabled.
 *
 ****************************************************************************/

static inline void tiva_gptm0_synchronize(uint32_t sync)
{
  putreg32(sync, TIVA_TIMER0_SYNC);
}

#endif /* __ARCH_ARM_SRC_TIVA_TIVA_TIMER_H */

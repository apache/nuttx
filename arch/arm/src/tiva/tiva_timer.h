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

/* Identifies timer A and timer B */

#define TIMER_A               0
#define TIMER_B               1

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

/* This enumeration describes the timer clock source */

enum tiva_timersource_e
{
  TIMER_SOURCE_SYSCLK = 0,       /* Timer clock source is SysClk */
  TIMER_SOURCE_PIOSC,            /* Timer clock source is PIOSC */
  TIMER_SOURCE_RTCOSC,           /* Source is  Hibernation Module Real-time clock */
  TIMER_SOURCE_LFIOSC            /* Timer clock source is LFI oscillator */
};

/* This structure describes the configuration of one 32-bit timer */

struct tiva_timer32config_s
{
  bool down;                     /* False: Count up; True: Count down */
  /* TODO:  Add fields to support ADC trigger events */
};

/* This structure describes the configuration of one 16-bit timer A/B */

struct tiva_timer16config_s
{
  uint8_t mode;                  /* See enum tiva_timermode_e */
  bool down;                     /* False: Count up; True: Count down */
  /* TODO:  Add fields to support ADC trigger events */
};

/* This structure describes usage of both timers on a GPTIM module */

struct tiva_gptmconfig_s
{
  uint8_t gptm;                  /* GPTM number */
  uint8_t source;                /* See enum tiva_timersource_e */
  uint8_t mode;                  /* See enum tiva_timer32mode_e */
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

/* This type represents the opaque handler returned by tiva_gptm_configure() */

typedef FAR void *TIMER_HANDLE;

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
 * Name: tiva_gptm0_synchronize
 *
 * Description:
 *   Trigger timers from GPTM0 output.
 *
 ****************************************************************************/

int tiva_gptm0_synchronize(uint32_t sync);

#endif /* __ARCH_ARM_SRC_TIVA_TIVA_TIMER_H */

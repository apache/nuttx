/************************************************************************************
 * arch/arm/src/stm32/stm32_hrtim.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32_HRTIM_H
#define __ARCH_ARM_SRC_STM32_STM32_HRTIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_STM32_HRTIM1

#if defined(CONFIG_STM32_STM32F33XX)
#  include "chip/stm32f33xxx_hrtim.h"
#else
#  error
#endif

/************************************************************************************
 * Pre-processor definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* HRTIM Timer X index */

enum stm32_hrtim_tim_e
{
  HRTIM_TIMER_MASTER,
#ifdef CONFIG_STM32_HRTIM_TIMA
  HRTIM_TIMER_TIMA,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB
  HRTIM_TIMER_TIMB,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC
  HRTIM_TIMER_TIMC,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD
  HRTIM_TIMER_TIMD,
#endif
#ifdef CONFIG_STM32_HRTIM_TIME
  HRTIM_TIMER_TIME,
#endif
};

/* Source which can force the Tx1/Tx2 output to its inactive state */

enum stm32_hrtim_out_rst_e
{
  HRTIM_OUT_RST_UPDATE    = (1 << 0),
  HRTIM_OUT_RST_EXTEVNT10 = (1 << 1),
  HRTIM_OUT_RST_EXTEVNT9  = (1 << 2),
  HRTIM_OUT_RST_EXTEVNT8  = (1 << 3),
  HRTIM_OUT_RST_EXTEVNT7  = (1 << 4),
  HRTIM_OUT_RST_EXTEVNT6  = (1 << 5),
  HRTIM_OUT_RST_EXTEVNT5  = (1 << 6),
  HRTIM_OUT_RST_EXTEVNT4  = (1 << 7),
  HRTIM_OUT_RST_EXTEVNT3  = (1 << 8),
  HRTIM_OUT_RST_EXTEVNT2  = (1 << 9),
  HRTIM_OUT_RST_EXTEVNT1  = (1 << 10),
  HRTIM_OUT_RST_TIMEVNT9  = (1 << 11),
  HRTIM_OUT_RST_TIMEVNT8  = (1 << 12),
  HRTIM_OUT_RST_TIMEVNT7  = (1 << 13),
  HRTIM_OUT_RST_TIMEVNT6  = (1 << 14),
  HRTIM_OUT_RST_TIMEVNT5  = (1 << 15),
  HRTIM_OUT_RST_TIMEVNT4  = (1 << 16),
  HRTIM_OUT_RST_TIMEVNT3  = (1 << 17),
  HRTIM_OUT_RST_TIMEVNT2  = (1 << 18),
  HRTIM_OUT_RST_TIMEVNT1  = (1 << 19),
  HRTIM_OUT_RST_MSTCMP4   = (1 << 20),
  HRTIM_OUT_RST_MSTCMP3   = (1 << 21),
  HRTIM_OUT_RST_MSTCMP2   = (1 << 22),
  HRTIM_OUT_RST_MSTCMP1   = (1 << 23),
  HRTIM_OUT_RST_MSTPER    = (1 << 24),
  HRTIM_OUT_RST_CMP4      = (1 << 25),
  HRTIM_OUT_RST_CMP3      = (1 << 26),
  HRTIM_OUT_RST_CMP2      = (1 << 27),
  HRTIM_OUT_RST_CMP1      = (1 << 28),
  HRTIM_OUT_RST_PER       = (1 << 29),
  HRTIM_OUT_RST_RESYNC    = (1 << 30),
  HRTIM_OUT_RST_SOFT      = (1 << 31),
};

/* Source which can force the Tx1/Tx2 output to its active state  */

enum stm32_hrtim_out_set_e
{
  HRTIM_OUT_SET_UPDATE    = (1 << 0),
  HRTIM_OUT_SET_EXTEVNT10 = (1 << 1),
  HRTIM_OUT_SET_EXTEVNT9  = (1 << 2),
  HRTIM_OUT_SET_EXTEVNT8  = (1 << 3),
  HRTIM_OUT_SET_EXTEVNT7  = (1 << 4),
  HRTIM_OUT_SET_EXTEVNT6  = (1 << 5),
  HRTIM_OUT_SET_EXTEVNT5  = (1 << 6),
  HRTIM_OUT_SET_EXTEVNT4  = (1 << 7),
  HRTIM_OUT_SET_EXTEVNT3  = (1 << 8),
  HRTIM_OUT_SET_EXTEVNT2  = (1 << 9),
  HRTIM_OUT_SET_EXTEVNT1  = (1 << 10),
  HRTIM_OUT_SET_TIMEVNT9  = (1 << 11),
  HRTIM_OUT_SET_TIMEVNT8  = (1 << 12),
  HRTIM_OUT_SET_TIMEVNT7  = (1 << 13),
  HRTIM_OUT_SET_TIMEVNT6  = (1 << 14),
  HRTIM_OUT_SET_TIMEVNT5  = (1 << 15),
  HRTIM_OUT_SET_TIMEVNT4  = (1 << 16),
  HRTIM_OUT_SET_TIMEVNT3  = (1 << 17),
  HRTIM_OUT_SET_TIMEVNT2  = (1 << 18),
  HRTIM_OUT_SET_TIMEVNT1  = (1 << 19),
  HRTIM_OUT_SET_MSTCMP4   = (1 << 20),
  HRTIM_OUT_SET_MSTCMP3   = (1 << 21),
  HRTIM_OUT_SET_MSTCMP2   = (1 << 22),
  HRTIM_OUT_SET_MSTCMP1   = (1 << 23),
  HRTIM_OUT_SET_MSTPER    = (1 << 24),
  HRTIM_OUT_SET_CMP4      = (1 << 25),
  HRTIM_OUT_SET_CMP3      = (1 << 26),
  HRTIM_OUT_SET_CMP2      = (1 << 27),
  HRTIM_OUT_SET_CMP1      = (1 << 28),
  HRTIM_OUT_SET_PER       = (1 << 29),
  HRTIM_OUT_SET_RESYNC    = (1 << 30),
  HRTIM_OUT_SET_SOFT      = (1 << 31),
};

/* Events that can reset TimerX Counter */

enum stm32_hrtim_tim_rst_e
{
  /* Timer owns events */

  HRTIM_RST_UPDT,
  HRTIM_RST_CMP4,
  HRTIM_RST_CMP2,

  /* Master Timer Events */

  HRTIM_RST_MSTCMP4,
  HRTIM_RST_MSTCMP3,
  HRTIM_RST_MSTCMP2,
  HRTIM_RST_MSTCMP1,
  HRTIM_RST_MSTPER,

  /* TimerX events */

  HRTIM_RST_TECMP4,
  HRTIM_RST_TECMP2,
  HRTIM_RST_TECMP1,
  HRTIM_RST_TDCMP4,
  HRTIM_RST_TDCMP2,
  HRTIM_RST_TDCMP1,
  HRTIM_RST_TCCMP4,
  HRTIM_RST_TCCMP2,
  HRTIM_RST_TCCMP1,
  HRTIM_RST_TBCMP4,
  HRTIM_RST_TBCMP2,
  HRTIM_RST_TBCMP1,
  HRTIM_RST_TACMP4,
  HRTIM_RST_TACMP2,
  HRTIM_RST_TACMP1,

  /* External Events */

  HRTIM_RST_EXTEVNT10,
  HRTIM_RST_EXTEVNT9,
  HRTIM_RST_EXTEVNT8,
  HRTIM_RST_EXTEVNT7,
  HRTIM_RST_EXTEVNT6,
  HRTIM_RST_EXTEVNT5,
  HRTIM_RST_EXTEVNT4,
  HRTIM_RST_EXTEVNT3,
  HRTIM_RST_EXTEVNT2,
  HRTIM_RST_EXTEVNT1,
};

/* HRTIM Timer X prescaler */

enum stm32_hrtim_tim_prescaler_e
{
  HRTIM_PRESCALER_1,
  HRTIM_PRESCALER_2,
  HRTIM_PRESCALER_4,
  HRTIM_PRESCALER_8,
  HRTIM_PRESCALER_16,
  HRTIM_PRESCALER_32,
  HRTIM_PRESCALER_64,
  HRTIM_PRESCALER_128,
};

/* HRTIM Fault Source */

enum stm32_hrtim_fault_src_e
{
  HRTIM_FAULT_SRC_PIN,
  HRTIM_FAULT_SRC_INTERNAL
};

/* HRTIM External Event Source
 * NOTE: according to Table 82 from STM32F334XX Manual
 */

enum stm32_hrtim_eev_src_e
{
  HRTIM_EEV_SRC_PIN,
  HRTIM_EEV_SRC_ANALOG,
  HRTIM_EEV_SRC_TRGO,
  HRTIM_EEV_SRC_ADC
};

struct hrtim_dev_s
{
#ifdef CONFIG_HRTIM
  /* Fields managed by common upper half HRTIM logic */

  uint8_t                 hd_ocount;    /* The number of times the device has been opened */
  sem_t                   hd_closesem;  /* Locks out new opens while close is in progress */
#endif

  /* Fields provided by lower half HRTIM logic */

  FAR void                     *hd_priv; /* Used by the arch-specific logic */
};

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32_hrtiminitialize
 *
 * Description:
 *   Initialize the HRTIM.
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   Valid HRTIM device structure reference on succcess; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the HRTIM block has enabled,
 *   2. Board-specific logic has already configured
 *
 ****************************************************************************/

FAR struct hrtim_dev_s* stm32_hrtiminitialize(void);

/****************************************************************************
 * Name: hrtim_register
 ****************************************************************************/

int hrtim_register(FAR const char *path, FAR struct hrtim_dev_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM32_HRTIM1 */
#endif /* __ARCH_ARM_SRC_STM32_STM32_HRTIM_H */

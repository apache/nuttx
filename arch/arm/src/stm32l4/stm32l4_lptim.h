/************************************************************************************
 * arch/arm/src/stm32l4/stm32l4_lptim.h
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With modifications and updates by:
 *
 *   Copyright (C) 2016 Motorola Mobility, LLC. All rights reserved.
 *   Copyright (C) 2011-2012, 2017 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 *   Copyright (c) 2015 Google, Inc.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_LPTIM_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_LPTIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32l4_lptim.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Helpers **************************************************************************/

#define STM32L4_LPTIM_SETMODE(d,mode)     ((d)->ops->setmode(d,mode))
#define STM32L4_LPTIM_SETCLOCK(d,freq)    ((d)->ops->setclock(d,freq))
#define STM32L4_LPTIM_SETCHANNEL(d,ch,en) ((d)->ops->setchannel(d,ch,en))
#define STM32L4_LPTIM_SETCLOCKSOURCE(d,s) ((d)->ops->setclocksource(d,s))
#define STM32L4_LPTIM_GETCOUNTER(d)       ((d)->ops->getcounter(d))
#define STM32L4_LPTIM_SETCOUNTMODE(d,m)   ((d)->ops->setcountmode(d,m))
#define STM32L4_LPTIM_SETPERIOD(d,period) ((d)->ops->setperiod(d,period))
#define STM32L4_LPTIM_GETPERIOD(d)        ((d)->ops->getperiod(d))

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

/* LPTIM Device Structure */

struct stm32l4_lptim_dev_s
{
  struct stm32l4_lptim_ops_s *ops;
};

/* LPTIM Modes of Operation */

typedef enum
{
  STM32L4_LPTIM_MODE_UNUSED       = -1,

  /* MODES */

  STM32L4_LPTIM_MODE_DISABLED     = 0x0000,
  STM32L4_LPTIM_MODE_SINGLE       = 0x0001,
  STM32L4_LPTIM_MODE_CONTINUOUS   = 0x0002,
  STM32L4_LPTIM_MODE_MASK         = 0x000f,
} stm32l4_lptim_mode_t;

/* LPTIM Clock Source */

typedef enum
{
  /* Clock Sources */

  STM32L4_LPTIM_CLK_PCLK          = 0x0000,
  STM32L4_LPTIM_CLK_LSI           = 0x0001,
  STM32L4_LPTIM_CLK_HSI           = 0x0002,
  STM32L4_LPTIM_CLK_LSE           = 0x0003,
  STM32L4_LPTIM_CLK_EXT           = 0x0004,
} stm32l4_lptim_clksrc_t;

/* LPTIM Counter Modes */

typedef enum
{
  /* Modes */

  STM32L4_LPTIM_COUNT_CLOCK       = 0x0000,
  STM32L4_LPTIM_COUNT_EXTTRIG     = 0x0001,
} stm32l4_lptim_cntmode_t;

/* LPTIM Clock Polarity */

typedef enum
{
  /* MODES */

  STM32L4_LPTIM_CLKPOL_RISING     = 0x0000,
  STM32L4_LPTIM_CLKPOL_FALLING    = 0x0001,
  STM32L4_LPTIM_CLKPOL_BOTH       = 0x0002,
} stm32l4_lptim_clkpol_t;

/* LPTIM Channel Modes */

typedef enum
{
  STM32L4_LPTIM_CH_DISABLED       = 0x0000,

  /* CHANNELS */

  STM32L4_LPTIM_CH_CHINVALID      = 0x0000,
  STM32L4_LPTIM_CH_CH1            = 0x0001,
  STM32L4_LPTIM_CH_CH2            = 0x0002,
  STM32L4_LPTIM_CH_CH3            = 0x0003,
  STM32L4_LPTIM_CH_MASK           = 0x000f,
} stm32l4_lptim_channel_t;

/* LPTIM Operations */

struct stm32l4_lptim_ops_s
{
  int  (*setmode)(FAR struct stm32l4_lptim_dev_s *dev, stm32l4_lptim_mode_t mode);
  int  (*setclock)(FAR struct stm32l4_lptim_dev_s *dev, uint32_t freq);
  int  (*setchannel)(FAR struct stm32l4_lptim_dev_s *dev,
                     stm32l4_lptim_channel_t channel, int enable);
  int  (*setclocksource)(FAR struct stm32l4_lptim_dev_s *dev,
                         stm32l4_lptim_clksrc_t clksrc);
  int  (*setpolarity)(FAR struct stm32l4_lptim_dev_s *dev,
                      stm32l4_lptim_clkpol_t polarity);
  uint32_t (*getcounter)(FAR struct stm32l4_lptim_dev_s *dev);
  int  (*setcountmode)(FAR struct stm32l4_lptim_dev_s *dev,
                       stm32l4_lptim_cntmode_t cntmode);
  void (*setperiod)(FAR struct stm32l4_lptim_dev_s *dev, uint32_t period);
  uint32_t (*getperiod)(FAR struct stm32l4_lptim_dev_s *dev);
};

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/* Get timer structure, power-up, reset, and mark it as used */

FAR struct stm32l4_lptim_dev_s *stm32l4_lptim_init(int timer);

/* Power-down timer, mark it as unused */

int stm32l4_lptim_deinit(FAR struct stm32l4_lptim_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_LPTIM_H */

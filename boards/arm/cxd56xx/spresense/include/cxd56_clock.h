/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_clock.h
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_CLOCK_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: board_clock_initialize
 *
 * Description:
 *   Initialize clock control.
 *   If this API is called and is returned, the clock mode is the highest
 *   clock mode. For example, CPU frequency is 156MHz.
 *
 ****************************************************************************/

void board_clock_initialize(void);

/****************************************************************************
 * Name: board_clock_enable
 *
 * Description:
 *   Enable dynamic clock control.
 *   If this API is called and is returned, the clock mode is the lowest
 *   clock mode. Later, you can control the clock mode dynamically by calling
 *   up_pm_acquire_freqlock() or up_pm_release_freqlock.
 *
 ****************************************************************************/

void board_clock_enable(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_CLOCK_H */

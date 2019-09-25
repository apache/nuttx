/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_isx012.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ISX012_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ISX012_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/video/isx012.h>
#include <nuttx/video/video.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 * Name: board_isx012_power_on
 *
 * Description:
 *   Power on ISX012
 *
 ****************************************************************************/

int board_isx012_power_on(void);

/****************************************************************************
 * Name: board_isx012_power_off
 *
 * Description:
 *   Power off ISX012
 *
 ****************************************************************************/

int board_isx012_power_off(void);

/****************************************************************************
 * Name: board_isx012_set_reset
 *
 * Description:
 *   Set reset ISX012
 *
 ****************************************************************************/

void board_isx012_set_reset(void);

/****************************************************************************
 * Name: board_isx012_release_reset
 *
 * Description:
 *   Release reset ISX012
 *
 ****************************************************************************/

void board_isx012_release_reset(void);

/****************************************************************************
 * Name: board_isx012_set_sleep
 *
 * Description:
 *   Set sleep ISX012
 *
 ****************************************************************************/

void board_isx012_set_sleep(int kind);

/****************************************************************************
 * Name: board_isx012_release_sleep
 *
 * Description:
 *   Release sleep ISX012
 *
 ****************************************************************************/

void board_isx012_release_sleep(void);

/****************************************************************************
 * Name: board_isx012_initialize
 *
 * Description:
 *   Initialize ISX012 i2c driver and register the ISX012 device.
 *
 ****************************************************************************/

int board_isx012_initialize(int i2c_bus_num);

/****************************************************************************
 * Name: board_isx012_uninitialize
 *
 * Description:
 *   Uninitialize ISX012 i2c driver and register the ISX012 device.
 *
 ****************************************************************************/

int board_isx012_uninitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ISX012_H */

/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_sdcard.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_SDCARD_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_SDCARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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
 * Name: board_sdcard_initialize
 *
 * Description:
 *   Initialize SD Card on the board.
 *
 ****************************************************************************/

int board_sdcard_initialize(void);

/****************************************************************************
 * Name: board_sdcard_pin_initialize
 *
 * Description:
 *   Initialize SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_initialize(void);

/****************************************************************************
 * Name: board_sdcard_pin_finalize
 *
 * Description:
 *   Finalize SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_finalize(void);

/****************************************************************************
 * Name: board_sdcard_pin_configuraton
 *
 * Description:
 *   Configure SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_configuraton(void);

/****************************************************************************
 * Name: board_sdcard_pin_enable
 *
 * Description:
 *   Enable SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_enable(void);

/****************************************************************************
 * Name: board_sdcard_pin_disable
 *
 * Description:
 *   Disable SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_disable(void);

/****************************************************************************
 * Name: board_sdcard_set_high_voltage
 *
 * Description:
 *   Set SD Card IO voltage to 3.3V
 *
 ****************************************************************************/

void board_sdcard_set_high_voltage(void);

/****************************************************************************
 * Name: board_sdcard_set_low_voltage
 *
 * Description:
 *   Set SD Card IO voltage to 1.8V
 *
 ****************************************************************************/

void board_sdcard_set_low_voltage(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_SDCARD_H */

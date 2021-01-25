/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_audio.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_AUDIO_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_AUDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

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
 * Name: board_audio_power_control
 *
 * Description:
 *   Power on/off the audio on the board.
 *
 ****************************************************************************/

bool board_audio_power_control(bool en);

/****************************************************************************
 * Name: board_aca_power_control
 *
 * Description:
 *   Power on/off the Aca device on the board.
 *
 ****************************************************************************/

int board_aca_power_control(int target, bool en);

/****************************************************************************
 * Name: board_aca_power_monitor
 *
 * Description:
 *   Get status of Power on/off the Aca device on the board.
 *
 ****************************************************************************/

bool board_aca_power_monitor(int target);

/****************************************************************************
 * Name: board_external_amp_mute_control
 *
 * Description:
 *   External Amp. Mute on/off.
 *    true:  Mute on
 *    false: Mute off
 *
 ****************************************************************************/

int board_external_amp_mute_control(bool en);

/****************************************************************************
 * Name: board_external_amp_mute_monitor
 *
 * Description:
 *   Get External Amp. Mute status.
 *    true:  Mute on
 *    false: Mute off
 *
 ****************************************************************************/

bool board_external_amp_mute_monitor(void);

/****************************************************************************
 * Name: board_audio_i2s_enable
 *
 * Description:
 *   Enable I2S on the board.
 *   Used by the audio driver. Do not use by users.
 *
 ****************************************************************************/

void board_audio_i2s_enable(void);

/****************************************************************************
 * Name: board_audio_i2s_disable
 *
 * Description:
 *   Disable I2S on the board.
 *   Used by the audio driver. Do not use by users.
 *
 ****************************************************************************/

void board_audio_i2s_disable(void);

/****************************************************************************
 * Name: board_audio_initialize
 *
 * Description:
 *   Initialize audio I/O on the board.
 *   Used by the audio driver. Do not use by users.
 *
 ****************************************************************************/

void board_audio_initialize(void);

/****************************************************************************
 * Name: board_audio_finalize
 *
 * Description:
 *   Finalize audio I/O on the board.
 *   Used by the audio driver. Do not use by users.
 *
 ****************************************************************************/

void board_audio_finalize(void);

/****************************************************************************
 * Name: board_audio_initialize_driver
 *
 * Description:
 *   Initializes a CXD56 audio device driver node with the given number.
 *   Used by the audio driver. Should not be used by users.
 *
 ****************************************************************************/

int board_audio_initialize_driver(int minor);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_AUDIO_H */

/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_analog.h
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

#ifndef __BOARDS_ARM_CXD56XX_DRIVERS_AUDIO_CXD56_AUDIO_ANALOG_H
#define __BOARDS_ARM_CXD56XX_DRIVERS_AUDIO_CXD56_AUDIO_ANALOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/audio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_analog_poweron(void);
CXD56_AUDIO_ECODE cxd56_audio_analog_poweroff(void);
CXD56_AUDIO_ECODE cxd56_audio_analog_poweron_input(
                                     cxd56_audio_mic_gain_t *gain);
CXD56_AUDIO_ECODE cxd56_audio_analog_poweron_output(void);
CXD56_AUDIO_ECODE cxd56_audio_analog_poweroff_input(void);
CXD56_AUDIO_ECODE cxd56_audio_analog_poweroff_output(void);
CXD56_AUDIO_ECODE cxd56_audio_analog_enable_output(void);
CXD56_AUDIO_ECODE cxd56_audio_analog_disable_output(void);
CXD56_AUDIO_ECODE cxd56_audio_analog_set_micgain(
                                     cxd56_audio_mic_gain_t *gain);
CXD56_AUDIO_ECODE cxd56_audio_analog_wait_input_standby(void);

#endif /* __BOARDS_ARM_CXD56XX_DRIVERS_AUDIO_CXD56_AUDIO_ANALOG_H */

/****************************************************************************
 * include/nuttx/analog/ads7828.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_ADS7828_H
#define __INCLUDE_NUTTX_ANALOG_ADS7828_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands
 * Cmd: ANIOC_ADS7828_SET_REF       Arg: enum ads7828_ref_e
 * Cmd: ANIOC_ADS7828_MODE          Arg: ads7828_mode_e
 * Cmd: ANIOC_ADS7828_POWER_SAVE    Arg: bool value
 * Cmd: ANIOC_ADS7828_ADD_CHAN      Arg: uint8_t value
 * Cmd: ANIOC_ADS7828_REMOVE_CHAN   ARG: uint8_t value
 * Cmd: ANIOC_ADS7828_READ_CHANNEL  Arg: struct adc_msg_s *channel
 */

#define ANIOC_ADS7828_SET_REF       _ANIOC(AN_ADS7828_FIRST + 0)
#define ANIOC_ADS7828_MODE          _ANIOC(AN_ADS7828_FIRST + 1)
#define ANIOC_ADS7828_POWER_SAVE    _ANIOC(AN_ADS7828_FIRST + 2)
#define ANIOC_ADS7828_ADD_CHAN      _ANIOC(AN_ADS7828_FIRST + 3)
#define ANIOC_ADS7828_REMOVE_CHAN   _ANIOC(AN_ADS7828_FIRST + 4)
#define ANIOC_ADS7828_READ_CHANNEL  _ANIOC(AN_ADS7828_FIRST + 5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum ads7828_ref_e
{
  ADS7828_REF_EXTERNAL = 0u,
  ADS7828_REF_INTERNAL
};

enum ads7828_mode_e
{
  ADS7828_DIFFERENTIAL = 0u,
  ADS7828_SINGLE_ENDED
};

enum ads7828_diffch_e
{
  ADS7828_DIFF_PCH0_NCH1 = 0u,
  ADS7828_DIFF_PCH2_NCH3,
  ADS7828_DIFF_PCH4_NCH5,
  ADS7828_DIFF_PCH6_NCH7,
  ADS7828_DIFF_PCH1_NCH0,
  ADS7828_DIFF_PCH3_NCH2,
  ADS7828_DIFF_PCH5_NCH4,
  ADS7828_DIFF_PCH7_NCH6
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_ANALOG_ADS7828_H */

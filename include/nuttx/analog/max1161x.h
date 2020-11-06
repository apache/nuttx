/****************************************************************************
 * include/nuttx/analog/max1161x.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_MAX1161X_H
#define __INCLUDE_NUTTX_ANALOG_MAX1161X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands
 * Cmd: ANIOC_MAX1161X_SET_REF       Arg: enum max1161x_ref_e
 * Cmd: ANIOC_MAX1161X_SET_CLOCK     Arg: enum max1161x_clock_e
 * Cmd: ANIOC_MAX1161X_SET_UNIBIP    Arg: enum max1161x_unibip_e
 * Cmd: ANIOC_MAX1161X_SET_SCAN      Arg: enum max1161x_scan_e
 * Cmd: ANIOC_MAX1161X_ADD_CHAN      Arg: uint8_t value
 * Cmd: ANIOC_MAX1161X_REMOVE_CHAN   Arg: uint8_t value
 * Cmd: ANIOC_MAX1161X_SET_SNGDIF    Arg: enum max1161x_sngdif_e
 * Cmd: ANIOC_MAX1161X_READ_CHANNEL  Arg: struct adc_msg_s *channel
 */

#define ANIOC_MAX1161X_SET_REF       _ANIOC(AN_MAX1161X_FIRST + 0)
#define ANIOC_MAX1161X_SET_CLOCK     _ANIOC(AN_MAX1161X_FIRST + 1)
#define ANIOC_MAX1161X_SET_UNIBIP    _ANIOC(AN_MAX1161X_FIRST + 2)
#define ANIOC_MAX1161X_SET_SCAN      _ANIOC(AN_MAX1161X_FIRST + 3)
#define ANIOC_MAX1161X_ADD_CHAN      _ANIOC(AN_MAX1161X_FIRST + 4)
#define ANIOC_MAX1161X_REMOVE_CHAN   _ANIOC(AN_MAX1161X_FIRST + 5)
#define ANIOC_MAX1161X_SET_SNGDIF    _ANIOC(AN_MAX1161X_FIRST + 6)
#define ANIOC_MAX1161X_READ_CHANNEL  _ANIOC(AN_MAX1161X_FIRST + 7)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Setup Byte */

/****************************************************************************
 * +----+----+----+------+---------+---------+---------------+
 * |SEL2|SEL1|SEL0| VREF | Ref Pin | Ref Dir | Int Ref State |
 * +----+----+----+------+---------+---------+---------------+
 * | 0  | 0  | x  | VDD  |  AIN    |   NC    |     off       |
 * | 0  | 1  | x  | EXT  |  RIN    |   IN    |     off       |
 * | 1  | 0  | 0  | INT  |  AIN    |   NC    |     off       |
 * | 1  | 0  | 1  | INT  |  AIN    |   NC    |     on        |
 * | 1  | 1  | 0  | INT  |  ROUT   |   OUT   |     off       |
 * | 1  | 1  | 1  | INT  |  ROUT   |   OUT   |     on        |
 * +----+----+----+------+---------+---------+---------------+
 ****************************************************************************/

enum max1161x_ref_e
{
  MAX1161X_REF_VDD_AIN_NC_OFF   = 0u,
  MAX1161X_REF_EXT_RIN_IN_OFF   = 2u,
  MAX1161X_REF_INT_AIN_NC_OFF   = 4u,
  MAX1161X_REF_INT_AIN_NC_ON    = 5u,
  MAX1161X_REF_INT_ROUT_OUT_OFF = 6u,
  MAX1161X_REF_INT_ROUT_OUT_ON  = 7u,
};

enum max1161x_clock_e
{
  MAX1161X_CLOCK_INT = 0u,
  MAX1161X_CLOCK_EXT
};

enum max1161x_unibip_e
{
  MAX1161X_UNIPOLAR = 0u,
  MAX1161X_BIPOLAR
};

enum max1161x_reset_e
{
  MAX1161X_NO_RESET = 0u,
  MAX1161X_RESET
};

/* Configuration Byte */

enum max1161x_scan_e
{
  MAX1161X_SCAN_FROM_ZERO = 0u,
  MAX1161X_SCAN_EIGHT_TIMES,
  MAX1161X_SCAN_UPPER,
  MAX1161X_SCAN_NONE
};

enum max1161x_sngdif_e
{
  MAX1161X_DIFFERENTIAL = 0u,
  MAX1161X_SINGLE_ENDED
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_ANALOG_MAX1161X_H */

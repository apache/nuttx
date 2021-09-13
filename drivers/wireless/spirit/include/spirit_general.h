/******************************************************************************
 * drivers/wireless/spirit/include/spirit_general.h
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_GENERAL_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_GENERAL_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_types.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#define SPIRIT_GENERAL_LIBVERSION() "Spirit1_Libraries_v.3.2.0"

/* Macros used in debug assertions */

#define IS_MODE_EXT(mode)  (mode  == MODE_EXT_XO   ||  mode == MODE_EXT_XIN)
#define IS_BLD_LVL(level)  (level == BLD_LVL_2_7_V || level == BLD_LVL_2_5_V || \
                            level == BLD_LVL_2_3_V || level == BLD_LVL_2_1_V)
#define IS_GM_CONF(mode)   (mode  == GM_SU_13_2    ||  mode == GM_SU_18_2    || \
                            mode  == GM_SU_21_5    ||  mode == GM_SU_25_6    || \
                            mode  == GM_SU_28_8    ||  mode == GM_SU_33_9    || \
                            mode  == GM_SU_38_5    ||  mode == GM_SU_43_0)
#define IS_PKT_TYPE(type)  (type  == PKT_BASIC     ||  type == PKT_MBUS      || \
                            type  == PKT_STACK)

/******************************************************************************
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* SPIRIT Mode external reference enumeration */

enum mode_extref_e
{
  MODE_EXT_XO  = 0,
  MODE_EXT_XIN = !MODE_EXT_XO
};

/* SPIRIT Battery level enumeration */

enum battery_level_e
{
  BLD_LVL_2_7_V = 0,
  BLD_LVL_2_5_V = 1,
  BLD_LVL_2_3_V = 2,
  BLD_LVL_2_1_V = 3
};

/* SPIRIT GM conf enumeration */

enum gm_conf_e
{
  GM_SU_13_2 = 0,
  GM_SU_18_2,
  GM_SU_21_5,
  GM_SU_25_6,
  GM_SU_28_8,
  GM_SU_33_9,
  GM_SU_38_5,
  GM_SU_43_0
};

/* SPIRIT packet type enumeration */

enum packet_type_e
{
  PKT_BASIC = 0x00,
  PKT_MBUS = 0x02,
  PKT_STACK
};

/* SPIRIT version type enumeration */

enum spirit_version_e
{
  SPIRIT_VERSION_2_1 = 0x01,     /* Deprecated */
  SPIRIT_VERSION_3_0,            /* The only version of SPIRIT1 */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_GENERAL_H */

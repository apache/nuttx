/********************************************************************************************
 * arch/arm/src/samd/chip/sam_sercom.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *   Datasheet", 42129J–SAM–12/2013
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD_CHIP_SAM_SERCOM_H
#define __ARCH_ARM_SRC_SAMD_CHIP_SAM_SERCOM_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Two generic clocks are used by the SERCOM: GCLK_SERCOMx_CORE and GCLK_SERCOMx_SLOW.  The
 * core clock (GCLK_SERCOMx_CORE) is required to clock the SERCOM while operating as a
 * master, while the slow clock (GCLK_SERCOM_SLOW) is only required for certain functions.
 * SERCOM modules must share the same slow GCLK channel ID.
 *
 * The baud-rate generator runs off the GCLK_SERCOMx_CORE clock (or, optionally, external
 * clock).
 */

#define SERCOM_GCLK_ID_SLOW          12
#define SERCOM_GCLK_ID_CORE(n)       (13+(n))
#  define SERCOM0_GCLK_ID_CORE       13
#  define SERCOM1_GCLK_ID_CORE       14
#  define SERCOM2_GCLK_ID_CORE       15
#  define SERCOM3_GCLK_ID_CORE       16
#  define SERCOM4_GCLK_ID_CORE       17
#  define SERCOM5_GCLK_ID_CORE       18

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD_CHIP_SAM_SERCOM_H */

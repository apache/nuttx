/************************************************************************************
 * arch/arm/src/samd2l2/sam_adc.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMD2L2_SAM_ADC_H
#define __ARCH_ARM_SRC_SAMD2L2_SAM_ADC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL definitions */

#define SAMD_ADC_IOCTL_START       0 /* start adc conversion */
#define SAMD_ADC_IOCTL_STOP        1 /* stop adc conversion */
#define SAMD_ADC_IOCTL_SET_PARAMS  2 /* set parameters when adc is stopped */
#define SAMD_ADC_IOCTL_GET_PARAMS  3 /* get parameters */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct sam_adc_param_s
{
  uint8_t samplen;      /* sampling time length */
  uint8_t prescaler;    /* prescaler configuration */
  uint8_t averaging;    /* number of samples to be collected */
};

#endif /* __ARCH_ARM_SRC_SAMD2L2_SAM_ADC_H */

/****************************************************************************
 * arch/arm/src/imxrt/imxrt_ocotp.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_OCOTP_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_OCOTP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_ocotp.h"

#include <sys/types.h>
#include <stdint.h>

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
 * Name: imxrt_ocotp_read
 *
 * Description:
 *   Read one value from the OTP.
 *
 * Input Parameters:
 *   otp_index - otp index (0-63)
 *   data      - a pointer to store the retrieved data
 *
 * Returned Value
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int imxrt_ocotp_read(uint32_t otp_index, uint32_t *data);

/****************************************************************************
 * Name: imxrt_ocotp_write
 *
 * Description:
 *   Write one value to the OTP.
 *
 * Input Parameters:
 *   otp_index - otp index (0-63)
 *   data      - data to write to OTP
 *
 * Returned Value
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 *
 ****************************************************************************/

int imxrt_ocotp_write(uint32_t otp_index, uint32_t data);

/****************************************************************************
 * Name: imxrt_ocotp_reload
 *
 * Description:
 *   Reload the Shadow from OTP.
 *
 ****************************************************************************/

int imxrt_ocotp_reload(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_OCOTP_H */

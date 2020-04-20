/****************************************************************************
 * arch/arm/include/cxd56xx/adc.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_ADC_H
#define __ARCH_ARM_INCLUDE_CXD56XX_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ANIOC_USER                 (AN_FIRST + AN_NCMDS)

/* Start sampling
 *
 * param None
 * return ioctl return value provides success/failure indication
 */

#define ANIOC_CXD56_START          _ANIOC(ANIOC_USER + 0)

/* Stop sampling
 *
 * param None
 * return ioctl return value provides success/failure indication
 */

#define ANIOC_CXD56_STOP           _ANIOC(ANIOC_USER + 1)

/* Set sampling frequency
 *
 * param None
 * return ioctl return value provides success/failure indication
 */

#define ANIOC_CXD56_FREQ           _ANIOC(ANIOC_USER + 2)

/* Set fifo size
 *
 * param None
 * return ioctl return value provides success/failure indication
 */

#define ANIOC_CXD56_FIFOSIZE       _ANIOC(ANIOC_USER + 3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Initialize valid ADC channels
 *
 * return OK(0) is success. negative value is failure.
 */

int cxd56_adcinitialize(void);

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_ADC_H */

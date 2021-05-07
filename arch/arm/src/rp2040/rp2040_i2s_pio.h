/****************************************************************************
 * arch/arm/src/rp2040/rp2040_i2s_pio.h
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

#ifndef __ARCH_ARM_SRC_RP2040_RP2040_I2S_PIO_H
#define __ARCH_ARM_SRC_RP2040_RP2040_I2S_PIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP2040_I2S_PIO_16BIT_STEREO         0
#define RP2040_I2S_PIO_16BIT_MONO           1
#define RP2040_I2S_PIO_8BIT_STEREO          2
#define RP2040_I2S_PIO_8BIT_MONO            3
#define RP2040_I2S_PIO_MAX_MODE             4

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_i2s_pio_configure
 *
 * Description:
 *   Configure RP2040 PIO for I2S
 *
 ****************************************************************************/

int rp2040_i2s_pio_configure(int mode, uint32_t samplerate);

/****************************************************************************
 * Name: rp2040_i2s_pio_enable
 *
 * Description:
 *   Set enable I2S transfer
 *
 ****************************************************************************/

void rp2040_i2s_pio_enable(bool enable);

/****************************************************************************
 * Name: rp2040_i2s_pio_getdmaaddr
 *
 * Description:
 *   Get DMA peripheral address for I2S transfer
 *
 ****************************************************************************/

uintptr_t rp2040_i2s_pio_getdmaaddr(void);

/****************************************************************************
 * Name: rp2040_i2s_pio_getdmaaddr
 *
 * Description:
 *   Get DREQ number for I2S transfer
 *
 ****************************************************************************/

uint8_t rp2040_i2s_pio_getdreq(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP2040_RP2040_I2S_PIO_H */

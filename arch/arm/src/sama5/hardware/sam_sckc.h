/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_sckc.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SCKC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SCKC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SCKC Register Offsets ****************************************************/

#define SAM_SCKC_CR_OFFSET    0x0000 /* Slow Clock Controller Configuration Register  */

/* SCKC Register Addresses **************************************************/

#define SAM_SCKC_CR           (SAM_SCKCR_VBASE+SAM_SCKC_CR_OFFSET)

/* SCKC Register Bit Definitions ********************************************/

/* Slow Clock Controller Configuration Register */

#ifdef ATSAMA5D3
#  define SCKC_CR_RCEN        (1 << 0)  /* Bit 0: Internal 32 kHz RC Oscillator */
#  define SCKC_CR_OSC32EN     (1 << 1)  /* Bit 1: 32768 Hz Oscillator */
#  define SCKC_CR_OSC32BYP    (1 << 2)  /* Bit 2: 2768Hz Oscillator Bypass */
#endif

#define SCKC_CR_OSCSEL        (1 << 3)  /* Bit 3:  Slow Clock Selector */

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SCKC_H */

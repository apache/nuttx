/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_flexcom.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_CLASSD_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_CLASSD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ClassD Register Offsets ******************************************/
#define SAM_CLASSD_CR_OFFSET      0x00  /* CLASSD Control Register */
#define SAM_CLASSD_MR_OFFSET      0x04  /* CLASSD Mode Register */
#define SAM_CLASSD_INTPMR_OFFSET  0x08  /* CLASSD Interpolator Mode Register */
#define SAM_CLASSD_INTSR_OFFSET   0x0C  /* CLASSD Interpolator Status Register */
#define SAM_CLASSD_THR_OFFSET     0x10  /* CLASSD Transmit Holding Register */
#define SAM_CLASSD_IER_OFFSET     0x14  /* CLASSD Interrupt Enable Register */
#define SAM_CLASSD_IDR_OFFSET     0x18  /* CLASSD Interrupt Disable Register */
#define SAM_CLASSD_IMR_OFFSET     0x1C  /* CLASSD Interrupt Mask Register */
#define SAM_CLASSD_ISR_OFFSET     0x20  /* CLASSD Interrupt Status Register */
                                        /* 0x24–0xE0 Reserved */
#define SAM_CLASSD_WPMR_OFFSET    0xE4  /* CLASSD Protection Mode Register */
                                        /* 0xE8–0xFC Reserved */

/* ClassD Register Addresses ****************************************/

#define SAM_CLASSD_CR       (SAM_CLASSD_VBASE + SAM_CLASSD_CR_OFFSET)
#define SAM_CLASSD_MR       (SAM_CLASSD_VBASE + SAM_CLASSD_MR_OFFSET)   
#define SAM_CLASSD_INTPMR   (SAM_CLASSD_VBASE + SAM_CLASSD_INTPMR_OFFSET) 
#define SAM_CLASSD_INTSR    (SAM_CLASSD_VBASE + SAM_CLASSD_INTSR_OFFSET) 
#define SAM_CLASSD_THR      (SAM_CLASSD_VBASE + SAM_CLASSD_THR_OFFSET) 
#define SAM_CLASSD_IER      (SAM_CLASSD_VBASE + SAM_CLASSD_IER_OFFSET) 
#define SAM_CLASSD_IDR      (SAM_CLASSD_VBASE + SAM_CLASSD_IDR_OFFSET) 
#define SAM_CLASSD_IMR      (SAM_CLASSD_VBASE + SAM_CLASSD_IMR_OFFSET) 
#define SAM_CLASSD_ISR      (SAM_CLASSD_VBASE + SAM_CLASSD_ISR_OFFSET) 
#define SAM_CLASSD_WPMR     (SAM_CLASSD_VBASE + SAM_CLASSD_WPMR_OFFSET) 


/* ClassD Register Bit Field Definitions ****************************/
#define CLASSD_CR_SWRST_SHIFT     (0) /*Bit 0, Software Reset */
#define CLASSD_CR_SWRST_MASK      (1 << CLASSD_CR_SWRST_SHIFT)

#define CLASSD_MR_LEN_SHIFT       (0) /*Bit 0, Left Channel Enable */
#define CLASSD_MR_LEN_MASK        (1 << CLASSD_MR_LEN_SHIFT)
#define CLASSD_MR_LMUTE_SHIFT     (1) /*Bit 1, Left Channel Mute */
#define CLASSD_MR_LMUTE_MASK      (1 << CLASSD_MR_LMUTE_SHIFT)
#define CLASSD_MR_REN_SHIFT       (4) /*Bit 4, Right Channel Enable */
#define CLASSD_MR_REN_MASK        (1 << CLASSD_MR_REN_SHIFT)
#define CLASSD_MR_RMUTE_SHIFT     (5) /*Bit 5, Right Channel Mute */
#define CLASSD_MR_RMUTE_MASK      (1 << CLASSD_MR_RMUTE_SHIFT)
#define CLASSD_MR_PWMTYP_SHIFT    (8) /* Bit 8 PWM Modulation Type */
#define CLASSD_MR_PWMTYP_MASK     (1 << CLASSD_MR_PWMTYP_SHIFT) /* Bit 8,
                                                  PWM Modulation Type */
#define CLASSD_MR_NOVR_SHIFT      (16) /* Bit 16, Non-Overlapping Enable */
#define CLASSD_MR_NOVR_MASK       (1 << CLASSD_MR_NOVR_SHIFT) 
#define CLASSD_MR_NOVRVAL_SHIFT   (20) /* Bit 20/21, Non-Overlapping Value */
#define CLASSD_MR_NOVRVAL_MASK    (3 << CLASSD_MR_NOVRVAL_SHIFT)
#define CLASSD_MR_NOVR(n)         (((uint32_t)(n) << CLASSD_MR_NOVRVAL_SHIFT) \
                                        & CLASSD_MR_NOVRVAL_MASK)

#define CLASSD_INTPMR_ATTL_SHIFT  (0) /*Bits 0-6, Left Channel Attenuation */
#define CLASSD_INTPMR_ATTL_MASK   (0x7F << CLASSD_INTPMR_ATTL_SHIFT)
#define CLASSD_VOL_LEFT(n)        (((uint32_t)(n) << CLASSD_INTPMR_ATTL_SHIFT) \
                                    & CLASSD_INTPMR_ATTL_MASK)

#define CLASSD_INTPMR_ATTR_SHIFT  (8) /*Bits 8-14, Right Channel Attenuation */
#define CLASSD_INTPMR_ATTR_MASK   (0x7F << CLASSD_INTPMR_ATTR_SHIFT)
#define CLASSD_VOL_RIGHT(n)       (((uint32_t)(n) << CLASSD_INTPMR_ATTR_SHIFT) \
                                    & CLASSD_INTPMR_ATTR_MASK)
                                  

#define CLASSD_INTPMR_DSPCLKF_SHIFT (16) /* Bit 16, DSP Clock Frequency */
#define CLASSD_INTPMR_DSPCLKF_MASK (1 << CLASSD_INTPMR_DSPCLKF_SHIFT) 
#define CLASSD_INTPMR_DEEMP_SHIFT (18) /*Bit 18, Enable De-emphasis filter */
#define CLASSD_INTPMR_DEEMP_MASK  (1 << CLASSD_INTPMR_DEEMP_SHIFT)
#define CLASSD_INTPMR_SWAP_SHIFT  (18) /*Bit 19, Swap L&R Channels */
#define CLASSD_INTPMR_SWAP_MASK   (1 << CLASSD_INTPMR_SWAP_SHIFT)
#define CLASSD_INTPMR_FRAME_SHIFT (20) /*(Bits 20-22, Incoming Data Sample F. */
#define CLASSD_INTPMR_FRAME_MASK  (7 << CLASSD_INTPMR_FRAME_SHIFT)
#define CLASSD_INTRMR_FRAME(n)    (((uint32_t)(n) << CLASSD_INTPMR_FRAME_SHIFT) \
                                    & CLASSD_INTPMR_FRAME_MASK)
                                  

#define CLASSD_INTPMR_EQCFG_SHIFT (24) /* Bits24-27, EQ selection */
#define CLASSD_INTPMR_EQCFG_MASK  (0xF << CLASSD_INTPMR_EQCFG_SHIFT)
#define CLASSD_INTRMR_EQCFG(n)    (((uint32_t)(n) << CLASSD_INTPMR_EQCFG_SHIFT) \
                                    & CLASSD_INTPMR_EQCFG_MASK) 
                                  

#define CLASSD_INTPMR_MONO_SHIFT  (28) /* Bit 28, Mono Enable */
#define CLASSD_INTPMR_MONO_MASK   (1 << CLASSD_INTPMR_MONO_SHIFT)
#define CLASSD_INTPMR_MONOMODE_SHIFT (29) /*Bits 29-30, Mono Mode */
#define CLASSD_INTPMR_MONOMODE_MASK (3 << CLASSD_INTPMR_MONOMODE_SHIFT)
#define CLASSD_INTRMR_MONOMODE(n)    (((uint32_t)(n) << CLASSD_INTPMR_MONOMODE_SHIFT) \
                                      & CLASSD_INTPMR_MONOMODE_MASK)
                                  
#define CLASSD_INTSR_CFGERR_SHIFT (0) /*Bit 0, Configuration Error */
#define CLASSD_INTSR_CFGERR_MASK  (1 << CLASSD_INTSR_CFGERR_SHIFT)

#define CLASSD_IER_DATRDY_SHIFT   (0) /* Bit 0,  Data Ready Interrupt Enable*/
#define CLASSD_IER_DATRDY_MASK    (1 << CLASSD_IER_DATRDY_SHIFT)

#define CLASSD_IDR_DATRDY_SHIFT   (0) /* Bit 0, Data Ready Interrupt Disable*/
#define CLASSD_IDR_DATRDY_MASK    (1 << CLASSD_IDR_DATRDY_SHIFT)

#define CLASSD_IMR_DATRDY_SHIFT   (0) /* Bit 0, Data Ready Interrupt Mask*/
#define CLASSD_IMR_DATRDY_MASK    (1 << CLASSD_IDR_DATRDY_SHIFT)

#define CLASSD_ISR_DATRDY_SHIFT   (0) /* Bit 0, Data Ready Interrupt Status*/
#define CLASSD_ISR_DATRDY_MASK    (1 << CLASSD_ISR_DATRDY_SHIFT)

#define CLASSD_WPMR_WPEN_SHIFT    (0) /* Bit 0, Write Protection Enable */
#define CLASSD_WPMR_WPEN_MASK     (1 << CLASSD_WPMR_WPEN_SHIFT)
#define CLASSD_WPMR_WPKEY_SHIFT   (8) /* Bits8-31, Write Protection Key */
#define CLASSD_WPMR_WPKEY_MASK    (0xFFFFFF << CLASSD_WPMR_WPKEY_SHIFT)
#define CLASSD_WPMR_PASSWD        (0x434C44 << CLASSD_WPMR_WPKEY_SHIFT)


#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_CLASSD_H */

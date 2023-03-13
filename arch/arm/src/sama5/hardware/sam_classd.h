/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_classd.h
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

/* Pre-requisites */

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO is required for audio subsystem support
#endif

/* Default configuration values */

#ifndef CONFIG_AUDIO_SAMA5_CLASSD_MSG_PRIO
#  define CONFIG_AUDIO_SAMA5_CLASSD_MSG_PRIO          2
#endif

#ifndef CCONFIG_AUDIO_SAMA5_CLASSD_BUFFER_SIZE
#  define CONFIG_AUDIO_SAMA5_CLASSD_BUFFER_SIZE       8192
#endif

#ifndef CCONFIG_AUDIO_SAMA5_CLASSD_NUM_BUFFERS
#  define CONFIG_AUDIO_SAMA5_CLASSD_NUM_BUFFERS       4
#endif

#ifndef CONFIG_AUDIO_SAMA5_CLASSD_WORKER_STACKSIZE
#  define CONFIG_AUDIO_SAMA5_CLASSD_WORKER_STACKSIZE  4096
#endif

/* ClassD Register Offsets */

#define SAM_CLASSD_CR_OFFSET     0x00 /* CLASSD Control Register            */
#define SAM_CLASSD_MR_OFFSET     0x04 /* CLASSD Mode Register               */
#define SAM_CLASSD_INTPMR_OFFSET 0x08 /* CLASSD Interpolator Mode Register  */
#define SAM_CLASSD_INTSR_OFFSET  0x0c /* CLASSD Interpolator Status Reg.    */
#define SAM_CLASSD_THR_OFFSET    0x10 /* CLASSD Transmit Holding Register   */
#define SAM_CLASSD_IER_OFFSET    0x14 /* CLASSD Interrupt Enable Register   */
#define SAM_CLASSD_IDR_OFFSET    0x18 /* CLASSD Interrupt Disable Register  */
#define SAM_CLASSD_IMR_OFFSET    0x1c /* CLASSD Interrupt Mask Register     */
#define SAM_CLASSD_ISR_OFFSET    0x20 /* CLASSD Interrupt Status Register   */
                                      /* 0x24–0xE0 Reserved                 */
#define SAM_CLASSD_WPMR_OFFSET   0xe4 /* CLASSD Protection Mode Register    */
                                      /* 0xE8–0xFC Reserved                 */

/* ClassD Register Addresses */

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

/* ClassD Register Bit Field Definitions */

#define CLASSD_CR_SWRST_SHIFT        (0) /* Bit 0, Software Reset           */
#define CLASSD_CR_SWRST_BIT          (1 << CLASSD_CR_SWRST_SHIFT)

#define CLASSD_MR_LEN_SHIFT          (0) /* Bit 0, Left Channel Enable      */
#define CLASSD_MR_LEN_BIT            (1 << CLASSD_MR_LEN_SHIFT)
#define CLASSD_MR_LMUTE_SHIFT        (1) /* Bit 1, Left Channel Mute        */
#define CLASSD_MR_LMUTE_BIT          (1 << CLASSD_MR_LMUTE_SHIFT)
#define CLASSD_MR_REN_SHIFT          (4) /* Bit 4, Right Channel Enable     */
#define CLASSD_MR_REN_BIT            (1 << CLASSD_MR_REN_SHIFT)
#define CLASSD_MR_RMUTE_SHIFT        (5) /* Bit 5, Right Channel Mute       */
#define CLASSD_MR_RMUTE_BIT          (1 << CLASSD_MR_RMUTE_SHIFT)
#define CLASSD_MR_PWMTYP_SHIFT       (8) /* Bit 8 PWM Modulation Type       */
#define CLASSD_MR_PWMTYP_BIT         (1 << CLASSD_MR_PWMTYP_SHIFT)
                                          /* Bit 8, PWM Modulation Type     */
#define CLASSD_MR_NOVR_SHIFT         (16) /* Bit 16, Non-Overlapping Enable */
#define CLASSD_MR_NOVR_BIT           (1 << CLASSD_MR_NOVR_SHIFT) 
#define CLASSD_MR_NOVRVAL_SHIFT      (20) /* Bit 20/21, Non-Overlap Value   */
#define CLASSD_MR_NOVRVAL_MASK       (3 << CLASSD_MR_NOVRVAL_SHIFT)
#define CLASSD_MR_NOVR(n)            ((n) << CLASSD_MR_NOVRVAL_SHIFT)

#define CLASSD_INTPMR_ATTL_SHIFT     (0) /* Bits 0-6, Left Channel Atten.   */
#define CLASSD_INTPMR_ATTL_MASK      (0x7f << CLASSD_INTPMR_ATTL_SHIFT)
#define CLASSD_VOL_LEFT(n)           ((n) << CLASSD_INTPMR_ATTL_SHIFT)

#define CLASSD_INTPMR_ATTR_SHIFT     (8) /* Bits 8-14, Right Channel Atten. */
#define CLASSD_INTPMR_ATTR_MASK      (0x7f << CLASSD_INTPMR_ATTR_SHIFT)
#define CLASSD_VOL_RIGHT(n)          ((n) << CLASSD_INTPMR_ATTR_SHIFT)

#define CLASSD_INTPMR_DSPCLKF_SHIFT  (16) /* Bit 16, DSP Clock Frequency    */
#define CLASSD_INTPMR_DSPCLKF_BIT    (1 << CLASSD_INTPMR_DSPCLKF_SHIFT) 
#define CLASSD_INTPMR_DEEMP_SHIFT    (18) /* Bit 18, Enable De-emph. filter */
#define CLASSD_INTPMR_DEEMP_BIT      (1 << CLASSD_INTPMR_DEEMP_SHIFT)
#define CLASSD_INTPMR_SWAP_SHIFT     (19) /* Bit 19, Swap L&R Channels      */
#define CLASSD_INTPMR_SWAP_BIT       (1 << CLASSD_INTPMR_SWAP_SHIFT)
#define CLASSD_INTPMR_FRAME_SHIFT    (20) /* Bits 20-22, Data Sample Freq   */
#define CLASSD_INTPMR_FRAME_MASK     (7 << CLASSD_INTPMR_FRAME_SHIFT)
#define CLASSD_INTRMR_FRAME(n)       ((n) << CLASSD_INTPMR_FRAME_SHIFT)

#define CLASSD_INTPMR_EQCFG_SHIFT    (24) /* Bits24-27, EQ selection        */
#define CLASSD_INTPMR_EQCFG_MASK     (16 << CLASSD_INTPMR_EQCFG_SHIFT)
#define CLASSD_INTRMR_EQCFG(n)       ((n) << CLASSD_INTPMR_EQCFG_SHIFT)

#define CLASSD_INTPMR_MONO_SHIFT     (28) /* Bit 28, Mono Enable            */
#define CLASSD_INTPMR_MONO_BIT       (1 << CLASSD_INTPMR_MONO_SHIFT)
#define CLASSD_INTPMR_MONOMODE_SHIFT (29) /* Bits 29-30, Mono Mode          */
#define CLASSD_INTPMR_MONOMODE_MASK  (3 << CLASSD_INTPMR_MONOMODE_SHIFT)
#define CLASSD_INTRMR_MONOMODE(n)    ((n) << CLASSD_INTPMR_MONOMODE_SHIFT)

#define CLASSD_INTSR_CFGERR_SHIFT    (0) /* Bit 0, Configuration Error      */
#define CLASSD_INTSR_CFGERR_BIT      (1 << CLASSD_INTSR_CFGERR_SHIFT)

#define CLASSD_IER_DATRDY_SHIFT      (0) /* Bit 0,  Data Ready Int. Enable  */
#define CLASSD_IER_DATRDY_BIT        (1 << CLASSD_IER_DATRDY_SHIFT)

#define CLASSD_IDR_DATRDY_SHIFT      (0) /* Bit 0, Data Ready Int. Disable  */
#define CLASSD_IDR_DATRDY_BIT        (1 << CLASSD_IDR_DATRDY_SHIFT)

#define CLASSD_IMR_DATRDY_SHIFT      (0) /* Bit 0, Data Ready Int. Mask     */
#define CLASSD_IMR_DATRDY_BIT        (1 << CLASSD_IDR_DATRDY_SHIFT)

#define CLASSD_ISR_DATRDY_SHIFT      (0) /* Bit 0, Data Ready Int. Status   */
#define CLASSD_ISR_DATRDY_BIT        (1 << CLASSD_ISR_DATRDY_SHIFT)

#define CLASSD_WPMR_WPEN_SHIFT       (0) /* Bit 0, Write Protection Enable  */
#define CLASSD_WPMR_WPEN_BIT         (1 << CLASSD_WPMR_WPEN_SHIFT)
#define CLASSD_WPMR_WPKEY_SHIFT      (8) /* Bits8-31, Write Protection Key  */
#define CLASSD_WPMR_WPKEY_MASK       (0xffffff << CLASSD_WPMR_WPKEY_SHIFT)
#define CLASSD_WPMR_PASSWD           (0x434c44 << CLASSD_WPMR_WPKEY_SHIFT)

struct audio_lowerhalf_s * sama5_classd_initialize(void);

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_CLASSD_H */

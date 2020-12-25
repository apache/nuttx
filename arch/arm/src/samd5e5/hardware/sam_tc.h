/****************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_tc.h
 *
 *   Copyright 2020 Falker Automacao Agricola LTDA.
 *   Author: Leomar Mateus Radke <leomar@falker.com.br>
 *   Author: Ricardo Wartchow <wartchow@gmail.com>
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_CHIP_SAMD_TC_H
#define __ARCH_ARM_SRC_SAMD5E5_CHIP_SAMD_TC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TC - COUNTx mode register offsets */

#define SAM_TC_CTRLA_OFFSET                 0x0000  /* Control A register */
#define SAM_TC_CTRLBCLR_OFFSET              0x0004  /* Control B clear register */
#define SAM_TC_CTRLBSET_OFFSET              0x0005  /* Control B Set register */
#define SAM_TC_EVCTRL_OFFSET                0x0006  /* Event Control registerr */
#define SAM_TC_INTENCLR_OFFSET              0x0008  /* Interrupt Enable Clear register */
#define SAM_TC_INTENSET_OFFSET              0x0009  /* Interrupt Enable Set register */
#define SAM_TC_INTFLAG_OFFSET               0x000A  /* Interrupt Flag Status and Clear register */
#define SAM_TC_STATUS_OFFSET                0x000B  /* Status register */
#define SAM_TC_WAVE_OFFSET                  0x000C  /* Waveform Generation Control register */
#define SAM_TC_DRVCTRL_OFFSET               0x000D  /* Driver Control register */
#define SAM_TC_DBGCTRL_OFFSET               0x000F  /* Debug Control register */
#define SAM_TC_SYNCBUSY_OFFSET              0x0010  /* Synchronization Busy register*/
#define SAM_TC_COUNT_OFFSET                 0x0014  /* Counter Value register */

/* TC-8bits mode register offsets */

#define SAM_TC_COUNT8_PER_OFFSET            0x001B  /* Period Value, 8-bit Mode register */
#define SAM_TC_COUNT8_CC0_OFFSET            0x001C  /* Channel 0 Compare/Capture Value, 8-bit Mode register */
#define SAM_TC_COUNT8_CC1_OFFSET            0x001D  /* Channel 1 Compare/Capture Value, 8-bit Mode register */
#define SAM_TC_COUNT8_PERBUF_OFFSET         0x002F  /* Period Buffer Value, 8-bit Mode register */
#define SAM_TC_COUNT8_CCBUF0_OFFSET         0x0030  /* Channel 0 Compare Buffer Value, 8-bit Mode register */
#define SAM_TC_COUNT8_CCBUF1_OFFSET         0x0031  /* Channel 1 Compare Buffer Value, 8-bit Mode register */

/* TC-16bits mode register offsets */

#define SAM_TC_COUNT16_CC0_OFFSET            0x001C  /* Channel 0 Compare/Capture Value, 16-bit Mode register */
#define SAM_TC_COUNT16_CC1_OFFSET            0x001E  /* Channel 1 Compare/Capture Value, 16-bit Mode register */
#define SAM_TC_COUNT16_CCBUF0_OFFSET         0x0030  /* Channel 0 Compare Buffer Value, 16-bit Mode register */
#define SAM_TC_COUNT16_CCBUF1_OFFSET         0x0032  /* Channel 1 Compare Buffer Value, 16-bit Mode register */

/* TC-32bits mode register offsets */

#define SAM_TC_COUNT32_CC0_OFFSET            0x001C  /* Channel 0 Compare/Capture Value, 32-bit Mode register */
#define SAM_TC_COUNT32_CC1_OFFSET            0x0020  /* Channel 1 Compare/Capture Value, 32-bit Mode register */
#define SAM_TC_COUNT32_CCBUF0_OFFSET         0x0030  /* Channel 0 Compare Buffer Value, 32-bit Mode register */
#define SAM_TC_COUNT32_CCBUF1_OFFSET         0x0034  /* Channel 1 Compare Buffer Value, 32-bit Mode register */

/* TCx register addresses */

#define SAM_TC0_CTRLA                 (SAM_TC0_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC0_CTRLBCLR              (SAM_TC0_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC0_CTRLBSET              (SAM_TC0_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC0_EVCTRL                (SAM_TC0_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC0_INTENCLR              (SAM_TC0_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC0_INTENSET              (SAM_TC0_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC0_INTFLAG               (SAM_TC0_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC0_STATUS                (SAM_TC0_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC0_WAVE 				  (SAM_TC0_BASE+SAM_TC_WAVE_OFFSET)
#define SAM_TC0_DRVCTRL 			  (SAM_TC0_BASE+SAM_TC_DRVCTRL_OFFSET)
#define SAM_TC0_DBGCTRL               (SAM_TC0_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC0_SYNCBUSY 			  (SAM_TC0_BASE+SAM_TC_SYNCBUSY_OFFSET)
#define SAM_TC0_COUNT                 (SAM_TC0_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC0_COUNT8_PER 			  (SAM_TC0_BASE+SAM_TC_COUNT8_PER_OFFSET)
#define SAM_TC0_COUNT8_CC0            (SAM_TC0_BASE+SAM_TC_COUNT8_CC0_OFFSET)
#define SAM_TC0_COUNT8_CC1            (SAM_TC0_BASE+SAM_TC_COUNT8_CC1_OFFSET)
#define SAM_TC0_COUNT8_PERBUF		  (SAM_TC0_BASE+SAM_TC_COUNT8_PERBUF_OFFSET)
#define SAM_TC0_COUNT8_CCBUF0		  (SAM_TC0_BASE+SAM_TC_COUNT8_CCBUF0_OFFSET)
#define SAM_TC0_COUNT8_CCBUF1		  (SAM_TC0_BASE+SAM_TC_COUNT8_CCBUF1_OFFSET)
#define SAM_TC0_COUNT16_CC0           (SAM_TC0_BASE+SAM_TC_COUNT16_CC0_OFFSET)
#define SAM_TC0_COUNT16_CC1           (SAM_TC0_BASE+SAM_TC_COUNT16_CC1_OFFSET)
#define SAM_TC0_COUNT16_CCBUF0		  (SAM_TC0_BASE+SAM_TC_COUNT16_CCBUF0_OFFSET)
#define SAM_TC0_COUNT16_CCBUF1	      (SAM_TC0_BASE+SAM_TC_COUNT16_CCBUF1_OFFSET)
#define SAM_TC0_COUNT32_CC0           (SAM_TC0_BASE+SAM_TC_COUNT32_CC0_OFFSET)
#define SAM_TC0_COUNT32_CC1           (SAM_TC0_BASE+SAM_TC_COUNT32_CC1_OFFSET)
#define SAM_TC0_COUNT32_CCBUF0		  (SAM_TC0_BASE+SAM_TC_COUNT32_CCBUF0_OFFSET)
#define SAM_TC0_COUNT32_CCBUF1	      (SAM_TC0_BASE+SAM_TC_COUNT32_CCBUF1_OFFSET)

#define SAM_TC1_CTRLA                 (SAM_TC1_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC1_CTRLBCLR              (SAM_TC1_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC1_CTRLBSET              (SAM_TC1_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC1_EVCTRL                (SAM_TC1_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC1_INTENCLR              (SAM_TC1_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC1_INTENSET              (SAM_TC1_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC1_INTFLAG               (SAM_TC1_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC1_STATUS                (SAM_TC1_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC1_WAVE 		     	  (SAM_TC1_BASE+SAM_TC_WAVE_OFFSET)
#define SAM_TC1_DRVCTRL 		      (SAM_TC1_BASE+SAM_TC_DRVCTRL_OFFSET)
#define SAM_TC1_DBGCTRL               (SAM_TC1_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC1_SYNCBUSY 	     	  (SAM_TC1_BASE+SAM_TC_SYNCBUSY_OFFSET)
#define SAM_TC1_COUNT                 (SAM_TC1_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC1_COUNT8_PER 		      (SAM_TC1_BASE+SAM_TC_COUNT8_PER_OFFSET)
#define SAM_TC1_COUNT8_CC0            (SAM_TC1_BASE+SAM_TC_COUNT8_CC0_OFFSET)
#define SAM_TC1_COUNT8_CC1            (SAM_TC1_BASE+SAM_TC_COUNT8_CC1_OFFSET)
#define SAM_TC1_COUNT8_PERBUF		  (SAM_TC1_BASE+SAM_TC_COUNT8_PERBUF_OFFSET)
#define SAM_TC1_COUNT8_CCBUF0		  (SAM_TC1_BASE+SAM_TC_COUNT8_CCBUF0_OFFSET)
#define SAM_TC1_COUNT8_CCBUF1	      (SAM_TC1_BASE+SAM_TC_COUNT8_CCBUF1_OFFSET)
#define SAM_TC1_COUNT16_CC0           (SAM_TC1_BASE+SAM_TC_COUNT16_CC0_OFFSET)
#define SAM_TC1_COUNT16_CC1           (SAM_TC1_BASE+SAM_TC_COUNT16_CC1_OFFSET)
#define SAM_TC1_COUNT16_CCBUF0		  (SAM_TC1_BASE+SAM_TC_COUNT16_CCBUF0_OFFSET)
#define SAM_TC1_COUNT16_CCBUF1	      (SAM_TC1_BASE+SAM_TC_COUNT16_CCBUF1_OFFSET)
#define SAM_TC1_COUNT32_CC0           (SAM_TC1_BASE+SAM_TC_COUNT32_CC0_OFFSET)
#define SAM_TC1_COUNT32_CC1           (SAM_TC1_BASE+SAM_TC_COUNT32_CC1_OFFSET)
#define SAM_TC1_COUNT32_CCBUF0		  (SAM_TC1_BASE+SAM_TC_COUNT32_CCBUF0_OFFSET)
#define SAM_TC1_COUNT32_CCBUF1	      (SAM_TC1_BASE+SAM_TC_COUNT32_CCBUF1_OFFSET)

#define SAM_TC2_CTRLA                 (SAM_TC2_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC2_CTRLBCLR              (SAM_TC2_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC2_CTRLBSET              (SAM_TC2_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC2_EVCTRL                (SAM_TC2_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC2_INTENCLR              (SAM_TC2_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC2_INTENSET              (SAM_TC2_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC2_INTFLAG               (SAM_TC2_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC2_STATUS                (SAM_TC2_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC2_WAVE 		     	  (SAM_TC2_BASE+SAM_TC_WAVE_OFFSET)
#define SAM_TC2_DRVCTRL 		      (SAM_TC2_BASE+SAM_TC_DRVCTRL_OFFSET)
#define SAM_TC2_DBGCTRL               (SAM_TC2_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC2_SYNCBUSY 	     	  (SAM_TC2_BASE+SAM_TC_SYNCBUSY_OFFSET)
#define SAM_TC2_COUNT                 (SAM_TC2_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC2_COUNT8_PER 		      (SAM_TC2_BASE+SAM_TC_COUNT8_PER_OFFSET)
#define SAM_TC2_COUNT8_CC0            (SAM_TC2_BASE+SAM_TC_COUNT8_CC0_OFFSET)
#define SAM_TC2_COUNT8_CC1            (SAM_TC2_BASE+SAM_TC_COUNT8_CC1_OFFSET)
#define SAM_TC2_COUNT8_PERBUF		  (SAM_TC2_BASE+SAM_TC_COUNT8_PERBUF_OFFSET)
#define SAM_TC2_COUNT8_CCBUF0		  (SAM_TC2_BASE+SAM_TC_COUNT8_CCBUF0_OFFSET)
#define SAM_TC2_COUNT8_CCBUF1	      (SAM_TC2_BASE+SAM_TC_COUNT8_CCBUF1_OFFSET)
#define SAM_TC2_COUNT16_CC0           (SAM_TC2_BASE+SAM_TC_COUNT16_CC0_OFFSET)
#define SAM_TC2_COUNT16_CC1           (SAM_TC2_BASE+SAM_TC_COUNT16_CC1_OFFSET)
#define SAM_TC2_COUNT16_CCBUF0		  (SAM_TC2_BASE+SAM_TC_COUNT16_CCBUF0_OFFSET)
#define SAM_TC2_COUNT16_CCBUF1	      (SAM_TC2_BASE+SAM_TC_COUNT16_CCBUF1_OFFSET)
#define SAM_TC2_COUNT32_CC0           (SAM_TC2_BASE+SAM_TC_COUNT32_CC0_OFFSET)
#define SAM_TC2_COUNT32_CC1           (SAM_TC2_BASE+SAM_TC_COUNT32_CC1_OFFSET)
#define SAM_TC2_COUNT32_CCBUF0		  (SAM_TC2_BASE+SAM_TC_COUNT32_CCBUF0_OFFSET)
#define SAM_TC2_COUNT32_CCBUF1	      (SAM_TC2_BASE+SAM_TC_COUNT32_CCBUF1_OFFSET)

#define SAM_TC3_CTRLA                 (SAM_TC3_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC3_CTRLBCLR              (SAM_TC3_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC3_CTRLBSET              (SAM_TC3_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC3_EVCTRL                (SAM_TC3_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC3_INTENCLR              (SAM_TC3_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC3_INTENSET              (SAM_TC3_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC3_INTFLAG               (SAM_TC3_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC3_STATUS                (SAM_TC3_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC3_WAVE 		     	  (SAM_TC3_BASE+SAM_TC_WAVE_OFFSET)
#define SAM_TC3_DRVCTRL 		      (SAM_TC3_BASE+SAM_TC_DRVCTRL_OFFSET)
#define SAM_TC3_DBGCTRL               (SAM_TC3_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC3_SYNCBUSY 	     	  (SAM_TC3_BASE+SAM_TC_SYNCBUSY_OFFSET)
#define SAM_TC3_COUNT                 (SAM_TC3_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC3_COUNT8_PER 		      (SAM_TC3_BASE+SAM_TC_COUNT8_PER_OFFSET)
#define SAM_TC3_COUNT8_CC0            (SAM_TC3_BASE+SAM_TC_COUNT8_CC0_OFFSET)
#define SAM_TC3_COUNT8_CC1            (SAM_TC3_BASE+SAM_TC_COUNT8_CC1_OFFSET)
#define SAM_TC3_COUNT8_PERBUF		  (SAM_TC3_BASE+SAM_TC_COUNT8_PERBUF_OFFSET)
#define SAM_TC3_COUNT8_CCBUF0		  (SAM_TC3_BASE+SAM_TC_COUNT8_CCBUF0_OFFSET)
#define SAM_TC3_COUNT8_CCBUF1	      (SAM_TC3_BASE+SAM_TC_COUNT8_CCBUF1_OFFSET)
#define SAM_TC3_COUNT16_CC0           (SAM_TC3_BASE+SAM_TC_COUNT16_CC0_OFFSET)
#define SAM_TC3_COUNT16_CC1           (SAM_TC3_BASE+SAM_TC_COUNT16_CC1_OFFSET)
#define SAM_TC3_COUNT16_CCBUF0		  (SAM_TC3_BASE+SAM_TC_COUNT16_CCBUF0_OFFSET)
#define SAM_TC3_COUNT16_CCBUF1	      (SAM_TC3_BASE+SAM_TC_COUNT16_CCBUF1_OFFSET)
#define SAM_TC3_COUNT32_CC0           (SAM_TC3_BASE+SAM_TC_COUNT32_CC0_OFFSET)
#define SAM_TC3_COUNT32_CC1           (SAM_TC3_BASE+SAM_TC_COUNT32_CC1_OFFSET)
#define SAM_TC3_COUNT32_CCBUF0		  (SAM_TC3_BASE+SAM_TC_COUNT32_CCBUF0_OFFSET)
#define SAM_TC3_COUNT32_CCBUF1	      (SAM_TC3_BASE+SAM_TC_COUNT32_CCBUF1_OFFSET)

#define SAM_TC4_CTRLA                 (SAM_TC4_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC4_CTRLBCLR              (SAM_TC4_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC4_CTRLBSET              (SAM_TC4_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC4_EVCTRL                (SAM_TC4_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC4_INTENCLR              (SAM_TC4_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC4_INTENSET              (SAM_TC4_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC4_INTFLAG               (SAM_TC4_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC4_STATUS                (SAM_TC4_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC4_WAVE 		     	  (SAM_TC4_BASE+SAM_TC_WAVE_OFFSET)
#define SAM_TC4_DRVCTRL 		      (SAM_TC4_BASE+SAM_TC_DRVCTRL_OFFSET)
#define SAM_TC4_DBGCTRL               (SAM_TC4_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC4_SYNCBUSY 	     	  (SAM_TC4_BASE+SAM_TC_SYNCBUSY_OFFSET)
#define SAM_TC4_COUNT                 (SAM_TC4_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC4_COUNT8_PER 		      (SAM_TC4_BASE+SAM_TC_COUNT8_PER_OFFSET)
#define SAM_TC4_COUNT8_CC0            (SAM_TC4_BASE+SAM_TC_COUNT8_CC0_OFFSET)
#define SAM_TC4_COUNT8_CC1            (SAM_TC4_BASE+SAM_TC_COUNT8_CC1_OFFSET)
#define SAM_TC4_COUNT8_PERBUF		  (SAM_TC4_BASE+SAM_TC_COUNT8_PERBUF_OFFSET)
#define SAM_TC4_COUNT8_CCBUF0		  (SAM_TC4_BASE+SAM_TC_COUNT8_CCBUF0_OFFSET)
#define SAM_TC4_COUNT8_CCBUF1	      (SAM_TC4_BASE+SAM_TC_COUNT8_CCBUF1_OFFSET)
#define SAM_TC4_COUNT16_CC0           (SAM_TC4_BASE+SAM_TC_COUNT16_CC0_OFFSET)
#define SAM_TC4_COUNT16_CC1           (SAM_TC4_BASE+SAM_TC_COUNT16_CC1_OFFSET)
#define SAM_TC4_COUNT16_CCBUF0		  (SAM_TC4_BASE+SAM_TC_COUNT16_CCBUF0_OFFSET)
#define SAM_TC4_COUNT16_CCBUF1	      (SAM_TC4_BASE+SAM_TC_COUNT16_CCBUF1_OFFSET)
#define SAM_TC4_COUNT32_CC0           (SAM_TC4_BASE+SAM_TC_COUNT32_CC0_OFFSET)
#define SAM_TC4_COUNT32_CC1           (SAM_TC4_BASE+SAM_TC_COUNT32_CC1_OFFSET)
#define SAM_TC4_COUNT32_CCBUF0		  (SAM_TC4_BASE+SAM_TC_COUNT32_CCBUF0_OFFSET)
#define SAM_TC4_COUNT32_CCBUF1	      (SAM_TC4_BASE+SAM_TC_COUNT32_CCBUF1_OFFSET)

#define SAM_TC5_CTRLA                 (SAM_TC5_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC5_CTRLBCLR              (SAM_TC5_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC5_CTRLBSET              (SAM_TC5_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC5_EVCTRL                (SAM_TC5_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC5_INTENCLR              (SAM_TC5_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC5_INTENSET              (SAM_TC5_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC5_INTFLAG               (SAM_TC5_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC5_STATUS                (SAM_TC5_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC5_WAVE 		     	  (SAM_TC5_BASE+SAM_TC_WAVE_OFFSET)
#define SAM_TC5_DRVCTRL 		      (SAM_TC5_BASE+SAM_TC_DRVCTRL_OFFSET)
#define SAM_TC5_DBGCTRL               (SAM_TC5_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC5_SYNCBUSY 	     	  (SAM_TC5_BASE+SAM_TC_SYNCBUSY_OFFSET)
#define SAM_TC5_COUNT                 (SAM_TC5_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC5_COUNT8_PER 		      (SAM_TC5_BASE+SAM_TC_COUNT8_PER_OFFSET)
#define SAM_TC5_COUNT8_CC0            (SAM_TC5_BASE+SAM_TC_COUNT8_CC0_OFFSET)
#define SAM_TC5_COUNT8_CC1            (SAM_TC5_BASE+SAM_TC_COUNT8_CC1_OFFSET)
#define SAM_TC5_COUNT8_PERBUF		  (SAM_TC5_BASE+SAM_TC_COUNT8_PERBUF_OFFSET)
#define SAM_TC5_COUNT8_CCBUF0		  (SAM_TC5_BASE+SAM_TC_COUNT8_CCBUF0_OFFSET)
#define SAM_TC5_COUNT8_CCBUF1	      (SAM_TC5_BASE+SAM_TC_COUNT8_CCBUF1_OFFSET)
#define SAM_TC5_COUNT16_CC0           (SAM_TC5_BASE+SAM_TC_COUNT16_CC0_OFFSET)
#define SAM_TC5_COUNT16_CC1           (SAM_TC5_BASE+SAM_TC_COUNT16_CC1_OFFSET)
#define SAM_TC5_COUNT16_CCBUF0		  (SAM_TC5_BASE+SAM_TC_COUNT16_CCBUF0_OFFSET)
#define SAM_TC5_COUNT16_CCBUF1	      (SAM_TC5_BASE+SAM_TC_COUNT16_CCBUF1_OFFSET)
#define SAM_TC5_COUNT32_CC0           (SAM_TC5_BASE+SAM_TC_COUNT32_CC0_OFFSET)
#define SAM_TC5_COUNT32_CC1           (SAM_TC5_BASE+SAM_TC_COUNT32_CC1_OFFSET)
#define SAM_TC5_COUNT32_CCBUF0		  (SAM_TC5_BASE+SAM_TC_COUNT32_CCBUF0_OFFSET)
#define SAM_TC5_COUNT32_CCBUF1	      (SAM_TC5_BASE+SAM_TC_COUNT32_CCBUF1_OFFSET)

/****************************************************************************
 * Not used
 ****************************************************************************/

/****************************************************************************
#define SAM_TC6_CTRLA                 (SAM_TC6_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC6_CTRLBCLR              (SAM_TC6_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC6_CTRLBSET              (SAM_TC6_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC6_EVCTRL                (SAM_TC6_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC6_INTENCLR              (SAM_TC6_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC6_INTENSET              (SAM_TC6_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC6_INTFLAG               (SAM_TC6_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC6_STATUS                (SAM_TC6_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC6_WAVE 		     	  (SAM_TC6_BASE+SAM_TC_WAVE_OFFSET)
#define SAM_TC6_DRVCTRL 		      (SAM_TC6_BASE+SAM_TC_DRVCTRL_OFFSET)
#define SAM_TC6_DBGCTRL               (SAM_TC6_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC6_SYNCBUSY 	     	  (SAM_TC6_BASE+SAM_TC_SYNCBUSY_OFFSET)
#define SAM_TC6_COUNT                 (SAM_TC6_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC6_COUNT8_PER 		      (SAM_TC6_BASE+SAM_TC_COUNT8_PER_OFFSET)
#define SAM_TC6_COUNT8_CC0            (SAM_TC6_BASE+SAM_TC_COUNT8_CC0_OFFSET)
#define SAM_TC6_COUNT8_CC1            (SAM_TC6_BASE+SAM_TC_COUNT8_CC1_OFFSET)
#define SAM_TC6_COUNT8_PERBUF		  (SAM_TC6_BASE+SAM_TC_COUNT8_PERBUF_OFFSET)
#define SAM_TC6_COUNT8_CCBUF0		  (SAM_TC6_BASE+SAM_TC_COUNT8_CCBUF0_OFFSET)
#define SAM_TC6_COUNT8_CCBUF1	      (SAM_TC6_BASE+SAM_TC_COUNT8_CCBUF1_OFFSET)
#define SAM_TC6_COUNT16_CC0           (SAM_TC6_BASE+SAM_TC_COUNT16_CC0_OFFSET)
#define SAM_TC6_COUNT16_CC1           (SAM_TC6_BASE+SAM_TC_COUNT16_CC1_OFFSET)
#define SAM_TC6_COUNT16_CCBUF0		  (SAM_TC6_BASE+SAM_TC_COUNT16_CCBUF0_OFFSET)
#define SAM_TC6_COUNT16_CCBUF1	      (SAM_TC6_BASE+SAM_TC_COUNT16_CCBUF1_OFFSET)
#define SAM_TC6_COUNT32_CC0           (SAM_TC6_BASE+SAM_TC_COUNT32_CC0_OFFSET)
#define SAM_TC6_COUNT32_CC1           (SAM_TC6_BASE+SAM_TC_COUNT32_CC1_OFFSET)
#define SAM_TC6_COUNT32_CCBUF0		  (SAM_TC6_BASE+SAM_TC_COUNT32_CCBUF0_OFFSET)
#define SAM_TC6_COUNT32_CCBUF1	      (SAM_TC6_BASE+SAM_TC_COUNT32_CCBUF1_OFFSET)

#define SAM_TC7_CTRLA                 (SAM_TC7_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC7_CTRLBCLR              (SAM_TC7_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC7_CTRLBSET              (SAM_TC7_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC7_EVCTRL                (SAM_TC7_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC7_INTENCLR              (SAM_TC7_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC7_INTENSET              (SAM_TC7_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC7_INTFLAG               (SAM_TC7_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC7_STATUS                (SAM_TC7_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC7_WAVE 		     	  (SAM_TC7_BASE+SAM_TC_WAVE_OFFSET)
#define SAM_TC7_DRVCTRL 		      (SAM_TC7_BASE+SAM_TC_DRVCTRL_OFFSET)
#define SAM_TC7_DBGCTRL               (SAM_TC7_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC7_SYNCBUSY 	     	  (SAM_TC7_BASE+SAM_TC_SYNCBUSY_OFFSET)
#define SAM_TC7_COUNT                 (SAM_TC7_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC7_COUNT8_PER 		      (SAM_TC7_BASE+SAM_TC_COUNT8_PER_OFFSET)
#define SAM_TC7_COUNT8_CC0            (SAM_TC7_BASE+SAM_TC_COUNT8_CC0_OFFSET)
#define SAM_TC7_COUNT8_CC1            (SAM_TC7_BASE+SAM_TC_COUNT8_CC1_OFFSET)
#define SAM_TC7_COUNT8_PERBUF		  (SAM_TC7_BASE+SAM_TC_COUNT8_PERBUF_OFFSET)
#define SAM_TC7_COUNT8_CCBUF0		  (SAM_TC7_BASE+SAM_TC_COUNT8_CCBUF0_OFFSET)
#define SAM_TC7_COUNT8_CCBUF1	      (SAM_TC7_BASE+SAM_TC_COUNT8_CCBUF1_OFFSET)
#define SAM_TC7_COUNT16_CC0           (SAM_TC7_BASE+SAM_TC_COUNT16_CC0_OFFSET)
#define SAM_TC7_COUNT16_CC1           (SAM_TC7_BASE+SAM_TC_COUNT16_CC1_OFFSET)
#define SAM_TC7_COUNT16_CCBUF0		  (SAM_TC7_BASE+SAM_TC_COUNT16_CCBUF0_OFFSET)
#define SAM_TC7_COUNT16_CCBUF1	      (SAM_TC7_BASE+SAM_TC_COUNT16_CCBUF1_OFFSET)
#define SAM_TC7_COUNT32_CC0           (SAM_TC7_BASE+SAM_TC_COUNT32_CC0_OFFSET)
#define SAM_TC7_COUNT32_CC1           (SAM_TC7_BASE+SAM_TC_COUNT32_CC1_OFFSET)
#define SAM_TC7_COUNT32_CCBUF0		  (SAM_TC7_BASE+SAM_TC_COUNT32_CCBUF0_OFFSET)
#define SAM_TC7_COUNT32_CCBUF1	      (SAM_TC7_BASE+SAM_TC_COUNT32_CCBUF1_OFFSET)

 ****************************************************************************/

/* TC register bit definitions */

/* Control A register */

#define TC_CTRLA_SWRST               (1 << 0)  /* Bit 0:  Software reset */
#define TC_CTRLA_ENABLE              (1 << 1)  /* Bit 1:  Enable */
#define TC_CTRLA_DISABLE             (0 << 1)  /* Bit 1:  Disable */
#define TC_CTRLA_MODE_SHIFT          (2)
#define TC_CTRLA_MODE_MASK           (3 << TC_CTRLA_MODE_SHIFT)
#define TC_CTRLA_MODE_COUNT16      	 (0 << TC_CTRLA_MODE_SHIFT)
#define TC_CTRLA_MODE_COUNT8         (1 << TC_CTRLA_MODE_SHIFT)
#define TC_CTRLA_MODE_COUNT32        (2 << TC_CTRLA_MODE_SHIFT)
#define TC_CTRLA_PRESCSYNC_SHIFT     (4)
#define TC_CTRLA_PRESCSYNC_MASK      (3 << TC_CTRLA_PRESCSYNC_SHIFT)
#define TC_CTRLA_PRESCSYNC_GCLK      (0 << TC_CTRLA_PRESCSYNC_SHIFT)
#define TC_CTRLA_PRESCSYNC_PRESC     (1 << TC_CTRLA_PRESCSYNC_SHIFT)
#define TC_CTRLA_PRESCSYNC_RESYNC    (2 << TC_CTRLA_PRESCSYNC_SHIFT)
#define TC_CTRLA_RUNSTDBY            (1 << 6)
#define TC_CTRLA_ONDEMAND            (1 << 7)
#define TC_CTRLA_PRESCALER_SHIFT     (8)
#define TC_CTRLA_PRESCALER_MASK      (7 << TC_CTRLA_PRESCALER_SHIFT)
#define TC_CTRLA_PRESCALER(n)        ((uint32_t)(n) << TC_CTRLA_PRESCALER_SHIFT)
#define TC_CTRLA_PRESCALER_DIV1    	 (0 << TC_CTRLA_PRESCALER_SHIFT)
#define TC_CTRLA_PRESCALER_DIV2      (1 << TC_CTRLA_PRESCALER_SHIFT)
#define TC_CTRLA_PRESCALER_DIV4      (2 << TC_CTRLA_PRESCALER_SHIFT)
#define TC_CTRLA_PRESCALER_DIV8      (3 << TC_CTRLA_PRESCALER_SHIFT)
#define TC_CTRLA_PRESCALER_DIV16     (4 << TC_CTRLA_PRESCALER_SHIFT)
#define TC_CTRLA_PRESCALER_DIV64     (5 << TC_CTRLA_PRESCALER_SHIFT)
#define TC_CTRLA_PRESCALER_DIV256    (6 << TC_CTRLA_PRESCALER_SHIFT)
#define TC_CTRLA_PRESCALER_DIV1024   (7 << TC_CTRLA_PRESCALER_SHIFT)
#define TC_CTRLA_ALOCK	             (1 << 11)
#define TC_CTRLA_CAPTEN0_SHIFT       (16)           /* (TC_CTRLA) Capture Channel 0 Enable */
#define TC_CTRLA_CAPTEN0             (1 << TC_CTRLA_CAPTEN0_SHIFT)
#define TC_CTRLA_CAPTEN1_SHIFT       (17)           /* (TC_CTRLA) Capture Channel 1 Enable */
#define TC_CTRLA_CAPTEN1             (1 << TC_CTRLA_CAPTEN1_SHIFT)
#define TC_CTRLA_COPEN0_SHIFT        (20)            /* (TC_CTRLA) Capture On Pin 0 Enable */
#define TC_CTRLA_COPEN0              (1 << TC_CTRLA_COPEN1_SHIFT)
#define TC_CTRLA_COPEN1_SHIFT        (21)           /* (TC_CTRLA) Capture On Pin 1 Enable */
#define TC_CTRLA_COPEN1              (1 << TC_CTRLA_CAPTEN1_SHIFT)
#define TC_CTRLA_CAPTMODE0_SHIFT     (24)  /* (TC_CTRLA) Capture Mode Channel 0 */
#define TC_CTRLA_CAPTMODE0_MASK      (3 << TC_CTRLA_CAPTMODE0_SHIFT)
#define TC_CTRLA_CAPTMODE0_CAPTD     (0 << TC_CTRLA_CAPTMODE0_SHIFT) /* (TC_CTRLA) Default capture */
#define TC_CTRLA_CAPTMODE0_CAPTMIN   (1 << TC_CTRLA_CAPTMODE0_SHIFT) /* (TC_CTRLA) Minimum capture */
#define TC_CTRLA_CAPTMODE0_CAPTMAX   (2 << TC_CTRLA_CAPTMODE0_SHIFT) /* (TC_CTRLA) Maximum capture */
#define TC_CTRLA_CAPTMODE1_SHIFT     (27)                            /* (TC_CTRLA) Capture Mode Channel 0 */
#define TC_CTRLA_CAPTMODE1_MASK      (3 << TC_CTRLA_CAPTMODE1_SHIFT)
#define TC_CTRLA_CAPTMODE1_CAPTD     (0 << TC_CTRLA_CAPTMODE1_SHIFT) /* (TC_CTRLA) Default capture */
#define TC_CTRLA_CAPTMODE1_CAPTMIN   (1 << TC_CTRLA_CAPTMODE1_SHIFT) /* (TC_CTRLA) Minimum capture */
#define TC_CTRLA_CAPTMODE1_CAPTMAX   (2 << TC_CTRLA_CAPTMODE1_SHIFT) /* (TC_CTRLA) Maximum capture */

/* Control B Clear register */

#define TC_CTRLBCLR_DIR               (1 << 0)
#define TC_CTRLBCLR_LUPD              (1 << 1)
#define TC_CTRLBCLR_ONESHOT           (1 << 2)
#define TC_CTRLBCLR_CMD_SHIFT         (5)
#define TC_CTRLBCLR_CMD_MASK          (7 << TC_CTRLBCLR_CMD_SHIFT)
#define TC_CTRLBCLR_CMD_NONE          (0 << TC_CTRLBCLR_CMD_SHIFT)   /* (TC_CTRLBCLR) No action */
#define TC_CTRLBCLR_CMD_RETRIGGER     (1 << TC_CTRLBCLR_CMD_SHIFT)   /* (TC_CTRLBCLR) Force a start, restart or retrigger */
#define TC_CTRLBCLR_CMD_STOP          (2 << TC_CTRLBCLR_CMD_SHIFT)   /* (TC_CTRLBCLR) Force a stop */
#define TC_CTRLBCLR_CMD_UPDATE        (3 << TC_CTRLBCLR_CMD_SHIFT)   /* (TC_CTRLBCLR) Force update of double-buffered register */
#define TC_CTRLBCLR_CMD_READSYNC      (4 << TC_CTRLBCLR_CMD_SHIFT)   /* (TC_CTRLBCLR) Force a read synchronization of COUNT */
#define TC_CTRLBCLR_CMD_DMAOS         (5 << TC_CTRLBCLR_CMD_SHIFT)   /* (TC_CTRLBCLR) One-shot DMA trigger */

/* Control B Set register */

#define TC_CTRLBSET_DIR               (1 << 0)
#define TC_CTRLBSET_LUPD              (1 << 1)
#define TC_CTRLBSET_ONESHOT           (1 << 2)
#define TC_CTRLBSET_CMD_SHIFT         (5)
#define TC_CTRLBSET_CMD_MASK          (7 << TC_CTRLBSET_CMD_SHIFT)
#define TC_CTRLBSET_CMD_NONE          (0 << TC_CTRLBSET_CMD_SHIFT)   /* (TC_CTRLBCLR) No action */
#define TC_CTRLBSET_CMD_RETRIGGER     (1 << TC_CTRLBSET_CMD_SHIFT)   /* (TC_CTRLBCLR) Force a start, restart or retrigger */
#define TC_CTRLBSET_CMD_STOP          (2 << TC_CTRLBSET_CMD_SHIFT)   /* (TC_CTRLBCLR) Force a stop */
#define TC_CTRLBSET_CMD_UPDATE        (3 << TC_CTRLBSET_CMD_SHIFT)   /* (TC_CTRLBCLR) Force update of double-buffered register */
#define TC_CTRLBSET_CMD_READSYNC      (4 << TC_CTRLBSET_CMD_SHIFT)   /* (TC_CTRLBCLR) Force a read synchronization of COUNT */
#define TC_CTRLBSET_CMD_DMAOS         (5 << TC_CTRLBSET_CMD_SHIFT)   /* (TC_CTRLBCLR) One-shot DMA trigger */

/* Event control register */

#define TC_EVCTRL_EVACT_SHIFT        (0)
#define TC_EVCTRL_EVACT_MASK         (7 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_OFF        (0 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_RETRIGGER  (1 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_COUNT      (2 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_START      (3 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_STAMP      (4 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_PPW        (5 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_PWP        (6 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_PW         (7 << TC_EVCTRL_EVACT_SHIFT)
#define TC_EVCTRL_TCINV              (1 << 4)
#define TC_EVCTRL_TCEI               (1 << 5)
#define TC_EVCTRL_OVFEO              (1 << 8)
#define TC_EVCTRL_MCEO0              (1 << 12)
#define TC_EVCTRL_MCEO1              (1 << 13)

/* TC_INTENCLR : Interrupt Enable Clear */

#define TC_INTENCLR_OVF             (1 << 0)            /* (TC_INTENCLR) OVF Interrupt Disable */
#define TC_INTENCLR_ERR             (1 << 1)            /* (TC_INTENCLR) ERR Interrupt Disable */
#define TC_INTENCLR_MC0             (1 << 4)            /* (TC_INTENCLR) MC Interrupt Disable 0 */
#define TC_INTENCLR_MC1             (1 << 5)            /* (TC_INTENCLR) MC Interrupt Disable 1 */

/* TC_INTENSET : Interrupt Enable Set */

#define TC_INTENSET_OVF             (1 << 0)            /* (TC_INTENCLR) OVF Interrupt Disable */
#define TC_INTENSET_ERR             (1 << 1)            /* (TC_INTENCLR) ERR Interrupt Disable */
#define TC_INTENSET_MC0             (1 << 4)            /* (TC_INTENCLR) MC Interrupt Disable 0 */
#define TC_INTENSET_MC1             (1 << 5)            /* (TC_INTENCLR) MC Interrupt Disable 1 */

/* TC_INTFLAG : Interrupt Flag Status and Clear */

#define TC_INTFLAG_OVF             (1 << 0)            /* (TC_INTENCLR) OVF Interrupt Disable */
#define TC_INTFLAG_ERR             (1 << 1)            /* (TC_INTENCLR) ERR Interrupt Disable */
#define TC_INTFLAG_MC0             (1 << 4)            /* (TC_INTENCLR) MC Interrupt Disable 0 */
#define TC_INTFLAG_MC1             (1 << 5)            /* (TC_INTENCLR) MC Interrupt Disable 1 */
#define TC_INTFLAG_ALL             (TC_INTFLAG_OVF | TC_INTFLAG_ERR | TC_INTFLAG_MC0 | TC_INTFLAG_MC1)

/* Status register */

#define TC_STATUS_STOP               (1 << 0)
#define TC_STATUS_SLAVE              (1 << 1)
#define TC_STATUS_PERBUFV            (1 << 3)
#define TC_STATUS_CCBUFV0            (1 << 4)
#define TC_STATUS_CCBUFV1            (1 << 5)

/* TC_WAVE : Waveform Generation Control */

#define TC_WAVE_WAVEGEN_SHIFT       (0)   /* (TC_WAVE) Waveform Generation Mode */
#define TC_WAVE_WAVEGEN_Msk         (3 << TC_WAVE_WAVEGEN_SHIFT)

#define TC_WAVE_WAVEGEN_NFRQ        (0 << TC_WAVE_WAVEGEN_SHIFT) /* (TC_WAVE) Normal frequency */
#define TC_WAVE_WAVEGEN_MFRQ        (1 << TC_WAVE_WAVEGEN_SHIFT) /* (TC_WAVE) Match frequency */
#define TC_WAVE_WAVEGEN_NPWM        (2 << TC_WAVE_WAVEGEN_SHIFT) /* (TC_WAVE) Normal PWM */
#define TC_WAVE_WAVEGEN_MPWM        (3 << TC_WAVE_WAVEGEN_SHIFT) /* (TC_WAVE) Match PWM */

/* TC_DRVCTRL : Control C */

#define TC_DRVCTRL_INVEN0           (1 << 0  /* (TC_DRVCTRL) Output Waveform Invert Enable 0 */
#define TC_DRVCTRL_INVEN1           (1 << 1) /* (TC_DRVCTRL) Output Waveform Invert Enable 1 */

/* TC_DBGCTRL : Debug Control */

#define TC_DBGCTRL_DBGRUN           (1 << 0) /* (TC_DBGCTRL) Run During Debug */

/* TC_SYNCBUSY : Synchronization Status */

#define TC_SYNCBUSY_SWRST       (1 << 0)            /* (TC_SYNCBUSY) swrst */
#define TC_SYNCBUSY_ENABLE      (1 << 1)            /* (TC_SYNCBUSY) enable */
#define TC_SYNCBUSY_CTRLB       (1 << 2)            /* (TC_SYNCBUSY) CTRLB */
#define TC_SYNCBUSY_STATUS      (1 << 3)            /* (TC_SYNCBUSY) STATUS */
#define TC_SYNCBUSY_COUNT       (1 << 4)            /* (TC_SYNCBUSY) Counter */
#define TC_SYNCBUSY_CC0         (1 << 6)            /* (TC_SYNCBUSY) Compare Channel 0 */
#define TC_SYNCBUSY_CC1         (1 << 7)            /* (TC_SYNCBUSY) Compare Channel 1 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_CHIP_SAMD_TC_H */

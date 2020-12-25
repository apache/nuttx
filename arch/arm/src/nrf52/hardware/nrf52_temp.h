/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_temp.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_TEMP_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_TEMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF52_TEMP_TASKS_START_OFFSET     0x000000  /* Start temperature measurement */
#define NRF52_TEMP_TASKS_STOP_OFFSET      0x000004  /* Stop temperature measurement */
#define NRF52_TEMP_EVENTS_DATARDY_OFFSET  0x000100  /* Temperature measurement complete, data ready */
#define NRF52_TEMP_INTENSET_OFFSET        0x000304  /* Enable interrupt */
#define NRF52_TEMP_INTENCLR_OFFSET        0x000308  /* Disable interrupt */
#define NRF52_TEMP_TEMP_OFFSET            0x000508  /* Temperature in degC */

#define NRF52_TEMP_A0_OFFSET              0x000520  /* Slope of 1st piece wise linear function */
#define NRF52_TEMP_A1_OFFSET              0x000524  /* Slope of 2nd piece wise linear function */
#define NRF52_TEMP_A2_OFFSET              0x000528  /* Slope of 3rd piece wise linear function */
#define NRF52_TEMP_A3_OFFSET              0x00052C  /* Slope of 4th piece wise linear function */
#define NRF52_TEMP_A4_OFFSET              0x000530  /* Slope of 5th piece wise linear function */
#define NRF52_TEMP_A5_OFFSET              0x000534  /* Slope of 6th piece wise linear function */

#define NRF52_TEMP_B0_OFFSET              0x000540  /* y-intercept of 1st piece wise linear function */
#define NRF52_TEMP_B1_OFFSET              0x000544  /* y-intercept of 2nd piece wise linear function */
#define NRF52_TEMP_B2_OFFSET              0x000548  /* y-intercept of 3rd piece wise linear function */
#define NRF52_TEMP_B3_OFFSET              0x00054C  /* y-intercept of 4th piece wise linear function */
#define NRF52_TEMP_B4_OFFSET              0x000550  /* y-intercept of 5th piece wise linear function */
#define NRF52_TEMP_B5_OFFSET              0x000554  /* y-intercept of 6th piece wise linear function */

#define NRF52_TEMP_T0_OFFSET              0x000560  /* End point of 1st piece wise linear function */
#define NRF52_TEMP_T1_OFFSET              0x000564  /* End point of 2nd piece wise linear function */
#define NRF52_TEMP_T2_OFFSET              0x000568  /* End point of 3rd piece wise linear function */
#define NRF52_TEMP_T3_OFFSET              0x00056C  /* End point of 4th piece wise linear function */
#define NRF52_TEMP_T4_OFFSET              0x000570  /* End point of 5th piece wise linear function */

/* Register definitions *****************************************************/

#define NRF52_TEMP_TASKS_START     (NRF52_TEMP_BASE + NRF52_TEMP_TASKS_START_OFFSET)
#define NRF52_TEMP_TASKS_STOP      (NRF52_TEMP_BASE + NRF52_TEMP_TASKS_STOP_OFFSET)
#define NRF52_TEMP_EVENTS_DATARDY  (NRF52_TEMP_BASE + NRF52_TEMP_EVENTS_DATARDY_OFFSET)
#define NRF52_TEMP_INTENSET        (NRF52_TEMP_BASE + NRF52_TEMP_INTENSET_OFFSET)
#define NRF52_TEMP_INTENCLR        (NRF52_TEMP_BASE + NRF52_TEMP_INTENCLR_OFFSET)
#define NRF52_TEMP_TEMP            (NRF52_TEMP_BASE + NRF52_TEMP_TEMP_OFFSET)

#define NRF52_TEMP_A0              (NRF52_TEMP_BASE + NRF52_TEMP_A0_OFFSET)
#define NRF52_TEMP_A1              (NRF52_TEMP_BASE + NRF52_TEMP_A1_OFFSET)
#define NRF52_TEMP_A2              (NRF52_TEMP_BASE + NRF52_TEMP_A2_OFFSET)
#define NRF52_TEMP_A3              (NRF52_TEMP_BASE + NRF52_TEMP_A3_OFFSET)
#define NRF52_TEMP_A4              (NRF52_TEMP_BASE + NRF52_TEMP_A4_OFFSET)
#define NRF52_TEMP_A5              (NRF52_TEMP_BASE + NRF52_TEMP_A5_OFFSET)

#define NRF52_TEMP_B0              (NRF52_TEMP_BASE + NRF52_TEMP_B0_OFFSET)
#define NRF52_TEMP_B1              (NRF52_TEMP_BASE + NRF52_TEMP_B1_OFFSET)
#define NRF52_TEMP_B2              (NRF52_TEMP_BASE + NRF52_TEMP_B2_OFFSET)
#define NRF52_TEMP_B3              (NRF52_TEMP_BASE + NRF52_TEMP_B3_OFFSET)
#define NRF52_TEMP_B4              (NRF52_TEMP_BASE + NRF52_TEMP_B4_OFFSET)
#define NRF52_TEMP_B5              (NRF52_TEMP_BASE + NRF52_TEMP_B5_OFFSET)

#define NRF52_TEMP_T0              (NRF52_TEMP_BASE + NRF52_TEMP_T0_OFFSET)
#define NRF52_TEMP_T1              (NRF52_TEMP_BASE + NRF52_TEMP_T1_OFFSET)
#define NRF52_TEMP_T2              (NRF52_TEMP_BASE + NRF52_TEMP_T2_OFFSET)
#define NRF52_TEMP_T3              (NRF52_TEMP_BASE + NRF52_TEMP_T3_OFFSET)
#define NRF52_TEMP_T4              (NRF52_TEMP_BASE + NRF52_TEMP_T4_OFFSET)

/* Register bit definitions *************************************************/

#define NRF52_TEMP_INTENSET_DATARDY  (1 << 0)  /* Read: Enabled */

#define NRF52_TEMP_INTENCLR_DATARDY  (1 << 0)  /* Read: Enabled */

#endif // __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_TEMP_H

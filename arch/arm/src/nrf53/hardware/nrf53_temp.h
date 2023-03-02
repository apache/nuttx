/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_temp.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_TEMP_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_TEMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF53_TEMP_TASKS_START_OFFSET     0x0000  /* Start temperature measurement */
#define NRF53_TEMP_TASKS_STOP_OFFSET      0x0004  /* Stop temperature measurement */
#define NRF53_TEMP_SUBSCRIBE_START_OFFSET 0x0080  /* Subscribe configuration for task START */
#define NRF53_TEMP_SUBSCRIBE_STOP_OFFSET  0x0084  /* Subscribe configuration for task STOP */
#define NRF53_TEMP_EVENTS_DATARDY_OFFSET  0x0100  /* Temperature measurement complete, data ready */
#define NRF53_TEMP_PUBLISH_DATARDY_OFFSET 0x0100  /* Publish configuration for event DATARDY */
#define NRF53_TEMP_INTENSET_OFFSET        0x0304  /* Enable interrupt */
#define NRF53_TEMP_INTENCLR_OFFSET        0x0308  /* Disable interrupt */
#define NRF53_TEMP_TEMP_OFFSET            0x0508  /* Temperature in degC */
#define NRF53_TEMP_A0_OFFSET              0x0520  /* Slope of 1st piece wise linear function */
#define NRF53_TEMP_A1_OFFSET              0x0524  /* Slope of 2nd piece wise linear function */
#define NRF53_TEMP_A2_OFFSET              0x0528  /* Slope of 3rd piece wise linear function */
#define NRF53_TEMP_A3_OFFSET              0x052C  /* Slope of 4th piece wise linear function */
#define NRF53_TEMP_A4_OFFSET              0x0530  /* Slope of 5th piece wise linear function */
#define NRF53_TEMP_A5_OFFSET              0x0534  /* Slope of 6th piece wise linear function */
#define NRF53_TEMP_B0_OFFSET              0x0540  /* y-intercept of 1st piece wise linear function */
#define NRF53_TEMP_B1_OFFSET              0x0544  /* y-intercept of 2nd piece wise linear function */
#define NRF53_TEMP_B2_OFFSET              0x0548  /* y-intercept of 3rd piece wise linear function */
#define NRF53_TEMP_B3_OFFSET              0x054C  /* y-intercept of 4th piece wise linear function */
#define NRF53_TEMP_B4_OFFSET              0x0550  /* y-intercept of 5th piece wise linear function */
#define NRF53_TEMP_B5_OFFSET              0x0554  /* y-intercept of 6th piece wise linear function */
#define NRF53_TEMP_T0_OFFSET              0x0560  /* End point of 1st piece wise linear function */
#define NRF53_TEMP_T1_OFFSET              0x0564  /* End point of 2nd piece wise linear function */
#define NRF53_TEMP_T2_OFFSET              0x0568  /* End point of 3rd piece wise linear function */
#define NRF53_TEMP_T3_OFFSET              0x056C  /* End point of 4th piece wise linear function */
#define NRF53_TEMP_T4_OFFSET              0x0570  /* End point of 5th piece wise linear function */

/* Register definitions *****************************************************/

#define NRF53_TEMP_TASKS_START      (NRF53_TEMP_BASE + NRF53_TEMP_TASKS_START_OFFSET)
#define NRF53_TEMP_TASKS_STOP       (NRF53_TEMP_BASE + NRF53_TEMP_TASKS_STOP_OFFSET)
#define NRF53_TEMP_SUBSCRIBES_START (NRF53_TEMP_BASE + NRF53_TEMP_SUBSCRIBES_START_OFFSET)
#define NRF53_TEMP_SUBSCRIBES_STOP  (NRF53_TEMP_BASE + NRF53_TEMP_SUBSCRIBES_STOP_OFFSET)
#define NRF53_TEMP_EVENTS_DATARDY   (NRF53_TEMP_BASE + NRF53_TEMP_EVENTS_DATARDY_OFFSET)
#define NRF53_TEMP_INTENSET         (NRF53_TEMP_BASE + NRF53_TEMP_INTENSET_OFFSET)
#define NRF53_TEMP_INTENCLR         (NRF53_TEMP_BASE + NRF53_TEMP_INTENCLR_OFFSET)
#define NRF53_TEMP_TEMP             (NRF53_TEMP_BASE + NRF53_TEMP_TEMP_OFFSET)
#define NRF53_TEMP_A0               (NRF53_TEMP_BASE + NRF53_TEMP_A0_OFFSET)
#define NRF53_TEMP_A1               (NRF53_TEMP_BASE + NRF53_TEMP_A1_OFFSET)
#define NRF53_TEMP_A2               (NRF53_TEMP_BASE + NRF53_TEMP_A2_OFFSET)
#define NRF53_TEMP_A3               (NRF53_TEMP_BASE + NRF53_TEMP_A3_OFFSET)
#define NRF53_TEMP_A4               (NRF53_TEMP_BASE + NRF53_TEMP_A4_OFFSET)
#define NRF53_TEMP_A5               (NRF53_TEMP_BASE + NRF53_TEMP_A5_OFFSET)
#define NRF53_TEMP_B0               (NRF53_TEMP_BASE + NRF53_TEMP_B0_OFFSET)
#define NRF53_TEMP_B1               (NRF53_TEMP_BASE + NRF53_TEMP_B1_OFFSET)
#define NRF53_TEMP_B2               (NRF53_TEMP_BASE + NRF53_TEMP_B2_OFFSET)
#define NRF53_TEMP_B3               (NRF53_TEMP_BASE + NRF53_TEMP_B3_OFFSET)
#define NRF53_TEMP_B4               (NRF53_TEMP_BASE + NRF53_TEMP_B4_OFFSET)
#define NRF53_TEMP_B5               (NRF53_TEMP_BASE + NRF53_TEMP_B5_OFFSET)
#define NRF53_TEMP_T0               (NRF53_TEMP_BASE + NRF53_TEMP_T0_OFFSET)
#define NRF53_TEMP_T1               (NRF53_TEMP_BASE + NRF53_TEMP_T1_OFFSET)
#define NRF53_TEMP_T2               (NRF53_TEMP_BASE + NRF53_TEMP_T2_OFFSET)
#define NRF53_TEMP_T3               (NRF53_TEMP_BASE + NRF53_TEMP_T3_OFFSET)
#define NRF53_TEMP_T4               (NRF53_TEMP_BASE + NRF53_TEMP_T4_OFFSET)

/* Register bit definitions *************************************************/

#define NRF53_TEMP_INTENSET_DATARDY  (1 << 0)  /* Read: Enabled */

#define NRF53_TEMP_INTENCLR_DATARDY  (1 << 0)  /* Read: Enabled */

#endif // __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_TEMP_H

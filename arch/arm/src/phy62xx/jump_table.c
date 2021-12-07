/****************************************************************************
 * arch/arm/src/phy62xx/jump_table.c
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

/****************************************************************************
 *  Filename:       jump_table.c
 *  Revised:
 *  Revision:
 *
 *  Description:    Jump table that holds function pointers and veriables
 *  used in ROM code.
 ****************************************************************************/

typedef unsigned int uint32_t;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LOG

/* static void hard_fault(void)
 * {
 *  while (1)
 *    {
 *      ;
 *    }
 *}
 */

/****************************************************************************
 *  ANTS
 ****************************************************************************/

/****************************************************************************
 * jump table, this table save the function entry
 * which will be called by ROM code
 * item 1 - 4 for OSAL task entry
 * item 224 - 255 for ISR(Interrupt Service Routine) entry
 * others are reserved by ROM code
 ****************************************************************************/

uint32_t *jump_table_base[256] __attribute__((section(".jumptbls"))) =
{
    (uint32_t *)0,                                     /* 0. write Log */
    (uint32_t *)0,                                     /* 1. init entry of app */
    (uint32_t *)0,                                     /* 2. task list */
    (uint32_t *)0,                                     /* 3. task count */
    (uint32_t *)0,                                     /* 4. task events */
    0, 0, 0, 0, 0,                                     /* 5 - 9, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 10 - 19, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 20 - 29, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0,                            /* <30 - - 37> */
    0, 0,
    0, 0, 0, 0, 0, 0,                                  /* 40 - 45 */
    0, 0, 0, 0,                                        /* 46 - 49 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 50 - 59, reserved by phyplus */
    0,                                                 /* < 60 - */
    0,
    0,
    0,
    0, 0, 0, 0, 0, 0,                                  /* -69>, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 70 -79, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 80 - 89, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 90 - 99, reserved by phyplus */
    (uint32_t *)0,                                     /* <100 - */
    (uint32_t *)0,
    (uint32_t *)0,
    0,
    0,
    0,
    0, 0, 0, 0,                                        /* - 109, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 110 -119, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 120 -129, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 130 -139, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 140 -149, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 150 -159, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 160 -169, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 170 -179, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 180 -189, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 190 -199, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 200 - 209, reserved by phyplus */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      /* 210 - 219, reserved by phyplus */
    (uint32_t *)0, 0, 0, 0, 0, 0, (uint32_t *)1234, 0, /* 220 - 227 */
    0,
    (uint32_t *)0,                                     /* 228 - 229 */
    0, 0, 0, (uint32_t *)0, 0,                         /* 230 - 234 */
    (uint32_t *)0,                                     /* 235 uart irq handler */
    (uint32_t *)0,
    (uint32_t *)0,
    (uint32_t *)1122,
    (uint32_t *)0,                                     /* 236 - 239 */
    (uint32_t *)0x67656961,                            /* 240 gpio interrupt handler */
    0, 0, 0, 0, 0, 0, 0, 0, 0,                         /* 241 - 249, for ISR entry */
    0, 0, 0, (uint32_t *)0, 0, 0                       /* 250 - 255, for ISR entry */
};

/* uint32_t  global_config[256] __attribute__((section(".gcfgtbls")))={0}; */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

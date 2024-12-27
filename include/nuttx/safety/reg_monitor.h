/****************************************************************************
 * include/nuttx/safety/reg_monitor.h
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

#ifndef __INCLUDE_NUTTX_SAFETY_REG_MONITOR_H
#define __INCLUDE_NUTTX_SAFETY_REG_MONITOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Enum for reporting the results of actions performed by the monitor */

enum action_type_e
{
  ACTION_NONE = 0,        /* No action taken */
  ACTION_REWRITE_OK = 1,  /* Rewrite operation success. */
  ACTION_REWRITE_FAIL,    /* Rewrite operation failed. */
  ACTION_RESET,           /* Reset is required. */
};

/* Structure defining register monitoring result
 *  index - index of the register in the map
 *  type  - action type taken for the fault
 *  The reg_result is reported in an array form, with the length of the
 *  array depending on the num specified in reg_monitor_initialize.
 */

begin_packed_struct struct reg_result_s
{
  uint16_t index : 13;  /* Index pointing to the map, 13 bits */
  uint8_t  type  : 3;   /* Action type taken for the fault, 3 bits */
} end_packed_struct;

/* Structure defining register monitoring properties
 *
 * addr    - Physical address of the register to monitor
 * value   - Expected value of the register
 * mask    - Bit mask to apply when checking register value
 * err_max_cnt - Maximum number of errors before taking action
 */

struct reg_map_s
{
  uintptr_t addr;          /* Register physical address */
  uint32_t  value;         /* Expected register value */
  uint32_t  mask;          /* Register value mask */
  uint8_t   err_max_cnt;   /* Error threshold count */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: register_monitor_init
 *
 * Description:
 *   Initialize the register monitor
 *
 *  When an anomaly is detected, it will be reported to the safety upper-half
 *  in the form of reg_result_s.
 ****************************************************************************/

int reg_monitor_initialize(FAR const struct reg_map_s *map, size_t map_num);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SAFETY_REG_MONITOR_H */

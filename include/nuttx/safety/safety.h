/****************************************************************************
 * include/nuttx/safety/safety.h
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

#ifndef __INCLUDE_NUTTX_SAFETY_SAFETY_H
#define __INCLUDE_NUTTX_SAFETY_SAFETY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL commands */

#define SAFETYIOC_INJECT        _SAFETYIOC(1)  /* Arg: void* */
#define SAFETYIOC_SELFTEST      _SAFETYIOC(2)  /* Arg: None */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Safety module types */

enum safety_module_e
{
  SAFETY_MODULE_CLOCK = 0,    /* Clock monitoring module */
  SAFETY_MODULE_POWER,        /* Power monitoring module */
  SAFETY_MODULE_CPU,          /* CPU protection module */
  SAFETY_MODULE_FLASH,        /* Flash protection module */
  SAFETY_MODULE_RAM,          /* RAM protection module */
  SAFETY_MODULE_MPU,          /* MPU configuration module */
  SAFETY_MODULE_BUS,          /* Bus monitoring module */
  SAFETY_MODULE_TEMPERATURE,  /* Temperature monitoring module */
  SAFETY_MODULE_ERROR_REPORT, /* Error report module */
  SAFETY_MODULE_SMU,          /* Safety Management & Fault control Unit */
  SAFETY_MODULE_REG,          /* Register monitoring module */
  SAFETY_MODULE_MAX
};

struct safety_lowerhalf_s;     /* Forward reference */

/* Safety module fault handler function pointer type.
 *
 * This handler is used to report specific result data from safety modules.
 *
 * Input Parameters:
 *   arg    - Handler specific argument passed during callback registration
 *   result - Pointer to the result data to be reported
 *   offset - Offset position where the data should be updated
 *   len    - Size of the result data to be updated
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 */

typedef int (*safety_handler_t)(FAR void *arg, FAR void *result,
                                off_t offset, size_t len);

/* Standard operations for safety module */

struct safety_ops_s
{
  /* Required operations */

  CODE int (*setup)(FAR struct safety_lowerhalf_s *lower);
  CODE int (*shutdown)(FAR struct safety_lowerhalf_s *lower);
  CODE int (*inject)(FAR struct safety_lowerhalf_s *lower,
                     FAR void *arg);
  CODE int (*selftest)(FAR struct safety_lowerhalf_s *lower);
  CODE int (*set_callback)(FAR struct safety_lowerhalf_s *lower,
                           safety_handler_t handler,
                           FAR void *arg);

  /* Optional ioctl operation */

  CODE int (*ioctl)(FAR struct safety_lowerhalf_s *lower,
                    int cmd, unsigned long arg);
};

/* Lower half driver structure */

struct safety_lowerhalf_s
{
  FAR const struct safety_ops_s *ops; /* Driver operations */
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
 * Name: safety_register
 *
 * Description:
 *   Register a safety module driver. This function will create a device node
 *   at /dev/safety/<type> that can be used by applications to interact with
 *   the safety module.
 *
 * Input Parameters:
 *   lower - A pointer to the lower half safety driver instance
 *   module - The module to register
 *   result_size - The size of the result buffer
 *
 * Returned Value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int safety_register(FAR struct safety_lowerhalf_s *lower,
                    enum safety_module_e module,
                    size_t result_size);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SAFETY_SAFETY_H */

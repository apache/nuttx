/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_systemreset.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_SYSTEMRESET_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_SYSTEMRESET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

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

/* Shutdown handler type */

typedef void (*shutdown_handler_t)(void);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_register_shutdown_handler
 *
 * Description:
 *   This function allows you to register a handler that gets invoked before
 *   the application is restarted.
 *
 * Input Parameters:
 *   handler - Function to execute on restart
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp32c3_register_shutdown_handler(shutdown_handler_t handler);

/****************************************************************************
 * Name: esp32c3_unregister_shutdown_handler
 *
 * Description:
 *   This function allows you to unregister a handler which was previously
 *   registered using esp32c3_register_shutdown_handler function.
 *
 * Input Parameters:
 *   handler - Function to execute on restart
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp32c3_unregister_shutdown_handler(shutdown_handler_t handler);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_SYSTEMRESET_H */

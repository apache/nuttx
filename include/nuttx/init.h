/****************************************************************************
 * include/nuttx/init.h
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

#ifndef __INCLUDE_NUTTX_INIT_H
#define __INCLUDE_NUTTX_INIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros for testing which OS services are available at this phase of
 * initialization.
 */

#define OSINIT_MM_READY()        (g_nx_initstate >= OSINIT_MEMORY)
#define OSINIT_HW_READY()        (g_nx_initstate >= OSINIT_HARDWARE)
#define OSINIT_OS_READY()        (g_nx_initstate >= OSINIT_OSREADY)
#define OSINIT_IDLELOOP()        (g_nx_initstate >= OSINIT_IDLELOOP)
#define OSINIT_OS_INITIALIZING() (g_nx_initstate  < OSINIT_OSREADY)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Initialization state.  OS bring-up occurs in several phases: */

enum nx_initstate_e
{
  OSINIT_POWERUP   = 0,  /* Power-up.  No initialization yet performed.
                          * Depends on .bss initialization logic for this
                          * value. */
  OSINIT_BOOT      = 1,  /* Basic boot up initialization is complete.  OS
                          * services and hardware resources are not yet
                          * available. */
  OSINIT_TASKLISTS = 2,  /* Head of ready-to-run/assigned task lists valid */
  OSINIT_MEMORY    = 3,  /* The memory manager has been initialized */
  OSINIT_HARDWARE  = 4,  /* MCU-specific hardware is initialized.  Hardware
                          * resources such as timers and device drivers are
                          * now available.  Low-level OS services sufficient
                          * to support the hardware are also available but
                          * the OS has not yet completed its full
                          * initialization. */
  OSINIT_OSREADY   = 5,  /* The OS is fully initialized and multi-tasking is
                          * active. */
  OSINIT_IDLELOOP  = 6   /* The OS enter idle loop */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* This is the current initialization state.  The level of initialization
 * is only important early in the start-up sequence when certain OS or
 * hardware resources may not yet be available to the OS-internal logic.
 */

EXTERN uint8_t g_nx_initstate;  /* See enum nx_initstate_e */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* This entry point must be supplied by the application */

//int CONFIG_INIT_ENTRYPOINT(int argc, char *argv[]);
int nsh_main(int argc, char *argv[]);

/* Functions contained in nx_task.c *****************************************/

/* OS entry point called by boot logic */

void nx_start(void) noreturn_function;

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INIT_H */

/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_serial.h
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

#ifndef __ARCH_ARM_SRC_MX8MP_MX8MP_SERIAL_H
#define __ARCH_ARM_SRC_MX8MP_MX8MP_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "mx8mp_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_serialinit
 *
 * Description:
 *   Register the UART serial console and serial ports.  This assumes that
 *   mx8mp_earlyserialinit was called previously.
 *
 ****************************************************************************/

#ifdef MX8MP_HAVE_UART
void arm_serialinit(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_MX8MP_MX8MP_SERIAL_H */

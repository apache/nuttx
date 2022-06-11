/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_bh1721fvc.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_BH1721FVC_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_BH1721FVC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
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
 * Name: board_bh1721fvc_initialize
 *
 * Description:
 *   Initialize BH1721FVC i2c driver and register the BH1721FVC device.
 *
 ****************************************************************************/

#if defined(CONFIG_SENSORS_BH1721FVC) || defined(CONFIG_SENSORS_BH1721FVC_SCU)
int board_bh1721fvc_initialize(const char *devpath, int bus);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_BH1721FVC_H */

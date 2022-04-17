/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_sph.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_SPH_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_SPH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _HSIOCVALID(c) (_IOC_TYPE(c)==0x7f00)
#define _HSIOC(nr)     _IOC(0x7f00,nr)

#define HSLOCK         _HSIOC(0x01)
#define HSTRYLOCK      _HSIOC(0x02)
#define HSUNLOCK       _HSIOC(0x03)

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int cxd56_sphinitialize(const char *devname);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_SPH_H */

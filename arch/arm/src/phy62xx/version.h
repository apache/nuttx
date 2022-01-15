/****************************************************************************
 * arch/arm/src/phy62xx/version.h
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
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_PHY62XX_VERSION_H
#define __ARCH_ARM_SRC_PHY62XX_VERSION_H

#define __DEF_CHIP_QFN32__                  (0x0001)
#define __DEF_CHIP_TSOP16__                  (0x0002)
#define SDK_VER_MAJOR                      3
#define SDK_VER_MINOR                      0
#define SDK_VER_REVISION                   7
#define SDK_VER_RELEASE_ID                 ((SDK_VER_MAJOR<<16)|(SDK_VER_MINOR<<8)|(SDK_VER_REVISION))
#define SDK_VER_CHIP                      __DEF_CHIP_QFN32__
/* #define SDK_VER_TEST_BUILD "" */
#endif /* __ARCH_ARM_SRC_PHY62XX_VERSION_H */


/****************************************************************************
 * arch/arm/src/imxrt/imxrt_enc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_ENC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_ENC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/qencoder.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QEIOC_POSDIFF       _QEIOC(QE_IMXRT_FIRST)
#define QEIOC_REVOLUTION    _QEIOC(QE_IMXRT_FIRST + 1)
#define QEIOC_RECONFIG      _QEIOC(QE_IMXRT_FIRST + 2)
#define QEIOC_INITTO        _QEIOC(QE_IMXRT_FIRST + 3)
#define QEIOC_RESETAT       _QEIOC(QE_IMXRT_FIRST + 4)
#define QEIOC_RESETATMAX    _QEIOC(QE_IMXRT_FIRST + 5)
#define QEIOC_TEST_GEN      _QEIOC(QE_IMXRT_FIRST + 6)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.  This function must be called
 *   from board-specific logic..
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   enc     - The encoder peripheral to use.
 *             'enc' must be an element of {1,2,3,4}
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int imxrt_qeinitialize(const char *devpath, int enc);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_ENC_H */

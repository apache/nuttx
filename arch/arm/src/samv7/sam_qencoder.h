/****************************************************************************
 * arch/arm/src/samv7/sam_qencoder.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_QENCODER_H
#define __ARCH_ARM_SRC_SAMV7_SAM_QENCODER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * as a quadrature encoder input device.  CONFIG_SAMV7_TCn is defined
 * then the CONFIG_SAMV7_TCn_QE must also be defined to indicate that timer
 * "n" is intended to be used for as a quadrature encoder.
 */

#ifndef CONFIG_SAMV7_TC0
#  undef CONFIG_SAMV7_TC0_QE
#endif
#ifndef CONFIG_SAMV7_TC1
#  undef CONFIG_SAMV7_TC1_QE
#endif
#ifndef CONFIG_SAMV7_TC2
#  undef CONFIG_SAMV7_TC2_QE
#endif
#ifndef CONFIG_SAMV7_TC3
#  undef CONFIG_SAMV7_TC3_QE
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Name: sam_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.  This function must be called
 *   from board-specific logic..
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   tc      - The timer counter number to used.  'tc' must be an element of
 *             {0,1,2,3}
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int sam_qeinitialize(const char *devpath, int tc);

#endif /* CONFIG_SENSORS_QENCODER */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_QENCODER_H */

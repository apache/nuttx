/****************************************************************************
 * arch/arm/src/kinetis/kinetis_tpm.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_TPM_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_TPM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* This file is just a wrapper around tmp header files for the Kinetis family
 * selected by the logic in chip.h.
 */

#if defined(KINETIS_K66)
#  include "hardware/kinetis_kx6tpm.h"
#else
#  error "No TMP definitions for this Kinetis part"
#endif

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_TPM_H */

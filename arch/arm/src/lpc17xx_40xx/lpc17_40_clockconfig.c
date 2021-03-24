/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_clockconfig.c
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

#include <nuttx/config.h>

/* This file is only a thin shell that includes the correct clock
 * configuration logic for the selected LPC17xx/LPC40xx family.  The correct
 * file cannot be selected by the make system because it needs the
 * intelligence that only exists in chip.h that can associate an
 * LPC17xx/LPC40xx part number with an LPC17xx/LPC40xx family.
 *
 * The LPC176x and LPC178x_40xx system control block is *nearly* identical
 * but we have found that the LPC178x_40xx is more sensitive to the ordering
 * of certain operations.  So, although the hardware seems very similar, the
 * safer thing to do is to separate the LPC176x and LPC178x_40xx into
 * separate files.
 */

#include <arch/lpc17xx_40xx/chip.h>

#if defined(LPC176x)
#  include "lpc176x_clockconfig.c"
#elif defined(LPC178x_40xx)
#  include "lpc178x_40xx_clockconfig.c"
#else
#  error "Unrecognized LPC17xx/LPC40xx family"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

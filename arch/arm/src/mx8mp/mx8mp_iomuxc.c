/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_iomuxc.c
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

#include <sys/types.h>
#include <assert.h>
#include <errno.h>

#include "arm_internal.h"
#include "mx8mp_iomuxc.h"

/* MUX_MODE - MUX Mode Select Field. */

#define IOMUXC_SW_MUX_CTL_PAD_MUX_MODE_MASK      (0x7U)
#define IOMUXC_SW_MUX_CTL_PAD_MUX_MODE_SHIFT     (0U)
#define IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(x)        (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_MUX_CTL_PAD_MUX_MODE_SHIFT)) & IOMUXC_SW_MUX_CTL_PAD_MUX_MODE_MASK)

/* SION - Software Input On Field. */

#define IOMUXC_SW_MUX_CTL_PAD_SION_MASK          (0x10U)
#define IOMUXC_SW_MUX_CTL_PAD_SION_SHIFT         (4U)
#define IOMUXC_SW_MUX_CTL_PAD_SION(x)            (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_MUX_CTL_PAD_SION_SHIFT)) & IOMUXC_SW_MUX_CTL_PAD_SION_MASK)

/* DAISY - Selecting Pads Involved in Daisy Chain. */

#define IOMUXC_SELECT_INPUT_DAISY_MASK           (0xFU)
#define IOMUXC_SELECT_INPUT_DAISY_SHIFT          (0U)
#define IOMUXC_SELECT_INPUT_DAISY(x)             (((uint32_t)(((uint32_t)(x)) << IOMUXC_SELECT_INPUT_DAISY_SHIFT)) & IOMUXC_SELECT_INPUT_DAISY_MASK)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_iomuxc_config
 ****************************************************************************/

void mx8mp_iomuxc_config(uint32_t mux_register,
                         uint32_t mux_mode,
                         uint32_t input_register,
                         uint32_t input_daisy,
                         uint32_t config_register,
                         uint32_t sion,
                         uint32_t config)
{
  putreg32(IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(mux_mode) |
           IOMUXC_SW_MUX_CTL_PAD_SION(sion),
           mux_register);

  if (input_register)
    {
      putreg32(IOMUXC_SELECT_INPUT_DAISY(input_daisy), input_register);
    }

  if (config_register)
    {
      putreg32(config, config_register);
    }
}

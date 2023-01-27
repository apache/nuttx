/****************************************************************************
 * arch/arm/src/rp2040/rp2040_cyw43439.h
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

#ifndef __BOARDS_ARM_RP2040_COMMON_INCLUDE_RP2040_CYW43439_H
#define __BOARDS_ARM_RP2040_COMMON_INCLUDE_RP2040_CYW43439_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/netdev.h>

#include <nuttx/wireless/ieee80211/bcmf_gspi.h>

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
 * Public Types
 ****************************************************************************/

typedef struct rp2040_gspi_s
{
  uint32_t              pio;          /* The pio instance we are using.    */
  uint32_t              pio_sm;       /* The state machine we are using.   */
  uint32_t              pio_location; /* the program location in the pio.  */
  uint8_t               gpio_on;      /* Set high to power chip on         */
  uint8_t               gpio_select;  /* Pull low to select chip           */
  uint8_t               gpio_data;    /* Data line -- idle high            */
  uint8_t               gpio_clock;   /* Clock line -- idle low            */
  uint8_t               gpio_intr;    /* May be shared with data           */
} rp2040_gspi_t;

typedef struct cyw_pio_program_s
{
  const uint16_t *instructions;
  uint8_t         length;
  int8_t          origin; /* required instruction memory origin or -1 */
} cyw_pio_program_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_cyw_setup
 *
 * Description:
 *   Initialize the cyw43439 device.
 *
 * Returns:
 *   A pointer to the cyw43439 device
 *
 ****************************************************************************/

gspi_dev_t *rp2040_cyw_setup(uint8_t gpio_on,
                             uint8_t gpio_select,
                             uint8_t gpio_data,
                             uint8_t gpio_clock,
                             uint8_t gpio_intr);

/****************************************************************************
 * Name: rp2040_cyw_remove
 *
 * Description:
 *   Deinitialize the cyw43439 device.
 *
 * Parameters:
 *   A pointer (as returned by rp2040_cyw_setup) to the device to remove.
 *
 ****************************************************************************/

void rp2040_cyw_remove(gspi_dev_t *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_RP2040_COMMON_INCLUDE_RP2040_CYW43439_H */

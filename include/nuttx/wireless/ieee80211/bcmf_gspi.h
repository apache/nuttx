/****************************************************************************
 * include/nuttx/ieee80211/bmcf_gspi.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE80211_BCMF_GSPI_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE80211_BCMF_GSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>

#include "nuttx/net/net.h"
#include "nuttx/net/netdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum gspi_cmd_func_e
  {
    gspi_f0_bus       = 0x0,
    gspi_f1_backplane = 0x1,
    gspi_f2_dma       = 0x2,
    gspi_f3_dma       = 0x3,
    gspi_f0_bus_rev16 = 0x4  /* variant of gspi_f0_bus that does REV16 */
  };

/* --- Our extension to struct net_driver_s --- */

typedef struct gspi_dev_s
{
  /* --------------------------------------------------------
   * Each board that implements CYW43439 must initialize the
   * following fields before calling gspi_register.
   */

  FAR int  (*init)            (FAR struct gspi_dev_s   *gspi);

  FAR int  (*deinit)          (FAR struct gspi_dev_s   *gspi);

  FAR int  (*set_isr)         (FAR struct gspi_dev_s   *gspi,
                               xcpt_t                   thread_isr,
                               FAR void                *thread_isr_arg);

  FAR int  (*interrupt_enable)(FAR struct gspi_dev_s   *gspi,
                              bool                     enable);

  FAR int  (*write)           (FAR struct gspi_dev_s  *gspi,
                               bool                    increment,
                               enum gspi_cmd_func_e    function,
                               uint32_t                address,
                               uint16_t                length,
                               FAR const uint32_t     *data);

  FAR int  (*read)            (FAR struct gspi_dev_s  *gspi,
                               bool                    increment,
                               enum gspi_cmd_func_e    function,
                               uint32_t                address,
                               uint16_t                length,
                               FAR uint32_t           *buffer);

  sem_t            exclsem;

  /* --------------------------------------------------------
   * Other fields must be set to zero.
   */

  void              *private;    /* Private data for opened device.    */
} gspi_dev_t;

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
 * Name: gspi_initialize
 *
 * Description:
 *   Initialize the cyw43439 driver.
 *
 ****************************************************************************/

int gspi_initialize(FAR gspi_dev_t *gspi);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE80211_BCMF_GSPI_H */

/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_fsp_err.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_RZV2H_RZV2H_FSP_ERR_H
#define __ARCH_ARM_SRC_RZV2H_RZV2H_FSP_ERR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* FSP headers -- included last, per espressif precedent (R2) */

#include "bsp_api.h"
#include "r_spi_api.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: rzv2h_fsp_err_to_errno
 *
 * Description:
 *   Convert a Renesas FSP fsp_err_t return code to a NuttX negated errno.
 *
 * Input Parameters:
 *   err - FSP error code returned by any R_*() function
 *
 * Returned Value:
 *   OK (0) for FSP_SUCCESS; negated errno for all other codes.
 *
 ****************************************************************************/

int rzv2h_fsp_err_to_errno(fsp_err_t err);

/****************************************************************************
 * Name: rzv2h_spi_event_to_errno
 *
 * Description:
 *   Convert an SPI_B transfer-completion event to a NuttX errno value.
 ****************************************************************************/

int rzv2h_spi_event_to_errno(spi_event_t event);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RZV2H_RZV2H_FSP_ERR_H */

/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_fsp_err.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include "rzv2h_fsp_err.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int rzv2h_fsp_err_to_errno(fsp_err_t err)
{
  switch (err)
    {
      case FSP_SUCCESS:
        return OK;

      case FSP_ERR_IN_USE:
        return -EBUSY;

      case FSP_ERR_INVALID_ARGUMENT:
      case FSP_ERR_INVALID_CHANNEL:
      case FSP_ERR_INVALID_POINTER:
      case FSP_ERR_INVALID_MODE:
      case FSP_ERR_ASSERTION:
        return -EINVAL;

      case FSP_ERR_NOT_OPEN:
        return -ENODEV;

      case FSP_ERR_ALREADY_OPEN:
        return -EALREADY;

      case FSP_ERR_TIMEOUT:
        return -ETIMEDOUT;

      case FSP_ERR_ABORTED:
        return -ECANCELED;

      case FSP_ERR_UNSUPPORTED:
      case FSP_ERR_NOT_ENABLED:
        return -ENOTSUP;

      case FSP_ERR_OUT_OF_MEMORY:
        return -ENOMEM;

      default:
        return -EIO;
    }
}

/****************************************************************************
 * Name: rzv2h_spi_event_to_errno
 *
 * Description:
 *   Convert an SPI_B transfer-completion event to a NuttX errno value.
 ****************************************************************************/

int rzv2h_spi_event_to_errno(spi_event_t event)
{
  switch (event)
    {
      case SPI_EVENT_TRANSFER_COMPLETE:
        return OK;

      case SPI_EVENT_TRANSFER_ABORTED:
        return -ECANCELED;

      case SPI_EVENT_ERR_READ_OVERFLOW:
      case SPI_EVENT_ERR_OVERRUN:
        return -EOVERFLOW;

      case SPI_EVENT_ERR_PARITY:
        return -EILSEQ;

      case SPI_EVENT_ERR_FRAMING:
        return -EPROTO;

      case SPI_EVENT_ERR_MODE_FAULT:
      case SPI_EVENT_ERR_MODE_UNDERRUN:
      default:
        return -EIO;
    }
}

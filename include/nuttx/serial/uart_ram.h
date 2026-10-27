/****************************************************************************
 * include/nuttx/serial/uart_ram.h
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

#ifndef __INCLUDE_NUTTX_SERIAL_UART_RAM_H
#define __INCLUDE_NUTTX_SERIAL_UART_RAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RAM_UART

#include <stdbool.h>
#include <nuttx/ring.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Rounded up so that in an array of two the second blob stays aligned */

#define UART_RAMBUF_BLOBSIZE \
  ALIGN_UP(sizeof(struct ring_blob_s) + CONFIG_RAM_UART_BUFSIZE, \
           RING_BLOB_ALIGN)

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/* One direction of the link: a ring_spsc blob written by exactly one side */

typedef uint8_t uart_rambuf_t[UART_RAMBUF_BLOBSIZE]
        aligned_data(RING_BLOB_ALIGN);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: uart_ram_register
 ****************************************************************************/

int uart_ram_register(FAR const char *devname,
                      FAR uart_rambuf_t buffer[2], bool slave);

/****************************************************************************
 * Name: ram_serialinit
 ****************************************************************************/

void ram_serialinit(void);

#ifdef __cplusplus
}
#endif

#endif

#endif

/****************************************************************************
 * drivers/serial/pty.h
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

#ifndef __DRIVERS_SERIAL_PTY_H
#define __DRIVERS_SERIAL_PTY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ptmx_minor_free
 *
 * Description:
 *   De-allocate a PTY minor number.
 *
 * Assumptions:
 *   Caller hold the px_exclsem
 *
 ****************************************************************************/

#ifdef CONFIG_PSEUDOTERM_SUSV1
void ptmx_minor_free(uint8_t minor);
#else
#  define ptmx_minor_free(minor)
#endif

/****************************************************************************
 * Name: pty_register2
 *
 * Description:
 *   Create and register PTY master and slave devices.  The slave side of
 *   the interface is always locked initially.  The master must call
 *   unlockpt() before the slave device can be opened.
 *
 * Input Parameters:
 *   minor - The number that qualifies the naming of the created devices.
 *   susv1 - select SUSv1 or BSD behaviour
 *
 * Returned Value:
 *   0 is returned on success; otherwise, the negative error code return
 *   appropriately.
 *
 ****************************************************************************/

int pty_register2(int minor, bool susv1);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_SERIAL_PTY_H */

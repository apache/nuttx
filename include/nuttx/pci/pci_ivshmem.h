/****************************************************************************
 * include/nuttx/pci/pci_ivshmem.h
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

#ifndef __INCLUDE_NUTTX_PCI_PCI_IVSHMEM_H
#define __INCLUDE_NUTTX_PCI_PCI_IVSHMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#ifdef CONFIG_PCI_IVSHMEM

#include <nuttx/ivshmem.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: pci_ivshemem_register
 *
 * Description:
 *   Register the pci ivshmem driver to the pci bus.
 *
 ****************************************************************************/

int pci_ivshemem_register(void);

#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_PCI_IVSHMEM */
#endif /* __INCLUDE_NUTTX_PCI_PCI_IVSHMEM_H */

/****************************************************************************
 * include/nuttx/pci/pci_ecam.h
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

#ifndef __INCLUDE_NUTTX_PCI_PCI_ECAM_H
#define __INCLUDE_NUTTX_PCI_PCI_ECAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/pci/pci.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pci_ecam_register
 *
 * Description:
 *   This function is used to register an ecam driver for pci.
 *
 * Input Parameters:
 *   cfg      - Configuration space data
 *   io       - I/O space data
 *   mem      - No-prefetchable space data
 *   mem_pref - Prefetchable space data
 *
 * Returned Value:
 *   Return 0 if success, nageative if failed
 *
 ****************************************************************************/

int pci_ecam_register(FAR const struct pci_resource_s *cfg,
                      FAR const struct pci_resource_s *io,
                      FAR const struct pci_resource_s *mem,
                      FAR const struct pci_resource_s *mem_pref);

#endif /* __INCLUDE_NUTTX_PCI_PCI_ECAM_H */

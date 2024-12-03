/****************************************************************************
 * drivers/pci/pci_drivers.h
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

#ifndef __DRIVERS_PCI_PCI_DRIVERS_H
#define __DRIVERS_PCI_PCI_DRIVERS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pci_dev_register
 *
 * Description:
 *   Create an pci dev driver.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

int pci_dev_register(void);

/****************************************************************************
 * Name: pci_ivshmem_register
 *
 * Description:
 *   Register common pci ivshmem driver
 *
 ****************************************************************************/

#ifdef CONFIG_PCI_IVSHMEM
int pci_ivshmem_register(void);
#endif

/****************************************************************************
 * Name: pci_register_uio_ivshmem_driver
 *
 * Description:
 *   Register uio ivshmem device pci driver
 *
 ****************************************************************************/

#ifdef CONFIG_PCI_UIO_IVSHMEM
int pci_register_uio_ivshmem_driver(void);
#endif

#ifdef CONFIG_PCI_QEMU_EPC
/****************************************************************************
 * Name: pci_register_qemu_epc_driver
 *
 * Description:
 *   Register a pci epc driver
 *
 ****************************************************************************/

int pci_register_qemu_epc_driver(void);
#endif

/****************************************************************************
 * Name: pci_register_epf_test_driver
 *
 * Description:
 *  Init a epf test driver
 *
 ****************************************************************************/

#ifdef CONFIG_PCI_EPF_TEST
int pci_register_epf_test_driver(void);
#endif

/****************************************************************************
 * Name: pci_register_ep_test_driver
 *
 * Description:
 *   Register endpoint test device pci driver
 *
 ****************************************************************************/

#ifdef CONFIG_PCI_EP_TEST
int pci_register_ep_test_driver(void);
#endif

/****************************************************************************
 * Name: pci_register_qemu_test_driver
 *
 * Description:
 *   Register qemu test device pci driver
 *
 ****************************************************************************/

#ifdef CONFIG_PCI_QEMU_TEST
int pci_register_qemu_test_driver(void);
#endif

/****************************************************************************
 * Name: pci_register_qemu_edu_driver
 *
 * Description:
 *   Register qemu edu device pci driver
 *
 ****************************************************************************/

#ifdef CONFIG_PCI_QEMU_EDU
int pci_register_qemu_edu_driver(void);
#endif

#endif /* __DRIVERS_PCI_PCI_DRIVERS_H */

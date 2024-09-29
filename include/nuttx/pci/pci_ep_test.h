/****************************************************************************
 * include/nuttx/pci/pci_ep_test.h
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

#ifndef __INCLUDE_NUTTX_PCI_EP_TEST_H
#define __INCLUDE_NUTTX_PCI_EP_TEST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_PCI_EPF_TEST

/****************************************************************************
 * Name: pci_register_epf_test_device
 *
 * Description:
 *  Init a epf device test
 *
 ****************************************************************************/

int pci_register_epf_test_device(FAR const char *epc_name);
#endif

#define PCITEST_BAR                 _PCIIOC(0x1)
#define PCITEST_LEGACY_IRQ          _PCIIOC(0x2)
#define PCITEST_MSI                 _PCIIOC(0x3)
#define PCITEST_WRITE               _PCIIOC(0x4)
#define PCITEST_READ                _PCIIOC(0x5)
#define PCITEST_COPY                _PCIIOC(0x6)
#define PCITEST_MSIX                _PCIIOC(0x7)
#define PCITEST_SET_IRQTYPE         _PCIIOC(0x8)
#define PCITEST_GET_IRQTYPE         _PCIIOC(0x9)
#define PCITEST_CLEAR_IRQ           _PCIIOC(0x10)

#define PCITEST_FLAGS_USE_DMA       0x00000001

/* struct pci_ep_test_param_s - Params config by user
 *
 * size: xfer data length
 * flag: xfer mode flag
 */

struct pci_ep_test_param_s
{
  unsigned int size;
  unsigned int flags;
};

#endif /* __INCLUDE_NUTTX_PCI_EP_TEST_H */

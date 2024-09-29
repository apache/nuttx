/****************************************************************************
 * drivers/pci/pci_drivers.c
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

#include <debug.h>

#include <nuttx/pci/pci.h>
#include <nuttx/pci/pci_qemu_edu.h>
#include <nuttx/pci/pci_qemu_test.h>
#include <nuttx/rptun/rptun_ivshmem.h>
#include <nuttx/rpmsg/rpmsg_virtio_ivshmem.h>
#include <nuttx/virtio/virtio-pci.h>
#include <nuttx/net/e1000.h>
#include <nuttx/net/igc.h>

#include "pci_drivers.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_register_drivers
 *
 * Description:
 *   Register all the pci drivers to pci bus
 *
 ****************************************************************************/

int pci_register_drivers(void)
{
  int ret = OK;

#ifdef CONFIG_PCI_IVSHMEM
  ret = pci_ivshmem_register();
  if (ret < 0)
    {
      pcierr("pci_ivshemem_register failed ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_PCI_UIO_IVSHMEM
  ret = pci_register_uio_ivshmem_driver();
  if (ret < 0)
    {
      pcierr("pci_register_uio_ivshmem_driver failed, ret=%d\n", ret);
    }
#endif

  /* Initialization rptun ivshmem driver */

#ifdef CONFIG_RPTUN_IVSHMEM
  ret = pci_register_rptun_ivshmem_driver();
  if (ret < 0)
    {
      pcierr("pci_register_rptun_ivshmem_driver failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_RPMSG_VIRTIO_IVSHMEM
  ret = pci_register_rpmsg_virtio_ivshmem_driver();
  if (ret < 0)
    {
      pcierr("pci_register_rpmsg_virtio_ivshmem_driver failed, ret=%d\n",
             ret);
    }
#endif

  /* Initialization pci qemu test driver */

#ifdef CONFIG_PCI_QEMU_TEST
  ret = pci_register_qemu_test_driver();
  if (ret < 0)
    {
      pcierr("pci_register_qemu_test_driver failed, ret=%d\n", ret);
    }
#endif

  /* Initialization qemu edu driver */

#ifdef CONFIG_PCI_QEMU_EDU
  ret = pci_register_qemu_edu_driver();
  if (ret < 0)
    {
      pcierr("pci_register_qemu_edu_driver failed, ret=%d\n", ret);
    }
#endif

  /* Initialization virtio pci driver */

#ifdef CONFIG_DRIVERS_VIRTIO_PCI
  ret = register_virtio_pci_driver();
  if (ret < 0)
    {
      pcierr("register_virtio_pci_driver failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_PCI_QEMU_EPC
  ret = pci_register_qemu_epc_driver();
  if (ret < 0)
    {
      pcierr("pci_register_qemu_ep_driver failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_PCI_EPF_TEST
  ret = pci_register_epf_test_driver();
  if (ret < 0)
    {
      pcierr("pci_register_epf_test_driver failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_PCI_EP_TEST
  ret = pci_register_ep_test_driver();
  if (ret < 0)
    {
      pcierr("pci_register_ep_test_driver failed, ret=%d\n", ret);
    }
#endif

  /* Initialization e1000 driver */

#ifdef CONFIG_NET_E1000
  ret = pci_e1000_init();
  if (ret < 0)
    {
      pcierr("pci_e1000_init failed, ret=%d\n", ret);
    }
#endif

  /* Initialization igc driver */

#ifdef CONFIG_NET_IGC
  ret = pci_igc_init();
  if (ret < 0)
    {
      pcierr("pci_igc_init failed, ret=%d\n", ret);
    }
#endif

  UNUSED(ret);
  return ret;
}

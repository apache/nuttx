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
#include <nuttx/pci/pci_qemu_test.h>

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
  int ret;

  /* Initialization pci qemu test driver */

#ifdef CONFIG_PCI_QEMU_TEST
  ret = pci_register_qemu_test_driver();
  if (ret < 0)
    {
      pcierr("pci_register_qemu_test_driver failed, ret=%d\n", ret);
    }
#endif

  UNUSED(ret);
  return ret;
}

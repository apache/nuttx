/****************************************************************************
 * boards/x86_64/intel64/qemu-intel64/src/qemu_pcie_readwrite.h
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

/* The PCI-E Definitions and part of the access routines are taken from
 * Jailhouse inmate library
 *
 * Jailhouse, a Linux-based partitioning hypervisor
 *
 * Copyright (c) Siemens AG, 2014
 *
 * Authors:
 *  Jan Kiszka <jan.kiszka@siemens.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 * Alternatively, you can use or redistribute this file under the following
 * BSD license:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __INCLUDE_NUTTX_PCIE_PCIE_READWRITE_H
#define __INCLUDE_NUTTX_PCIE_PCIE_READWRITE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/pcie/pcie.h>

#include <nuttx/board.h>
#include <nuttx/serial/uart_16550.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCI_REG_ADDR_PORT       0xcf8
#define PCI_REG_DATA_PORT       0xcfc

#define PCI_CONE                (1 << 31)

/****************************************************************************
 * Name: __qemu_pci_cfg_write
 *
 * Description:
 *  Write 8, 16, 32 bits data to PCI-E configuration space of device
 *  specified by dev
 *
 * Input Parameters:
 *   bfd    - Device private data
 *   buffer - A pointer to the read-only buffer of data to be written
 *   size   - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static inline int __qemu_pci_cfg_write(uint16_t bfd, uintptr_t addr,
                                       FAR const void *buffer,
                                       unsigned int size)
{
  outl(PCI_CONE | ((uint32_t)bfd << 8) | (addr & 0xfc), PCI_REG_ADDR_PORT);

  switch (size)
    {
      case 1:
        outb(*(uint8_t *)(buffer), PCI_REG_DATA_PORT + (addr & 0x3));
        break;
      case 2:
        outw(*(uint16_t *)(buffer), PCI_REG_DATA_PORT + (addr & 0x3));
        break;
      case 4:
        outl(*(uint32_t *)(buffer), PCI_REG_DATA_PORT);
        break;
      default:
        return -EINVAL;
    }
  return OK;
}

/****************************************************************************
 * Name: __qemu_pci_cfg_write64
 *
 * Description:
 *  Write 64 bits data to PCI-E configuration space of device
 *  specified by dev
 *
 * Input Parameters:
 *   bfd    - Device private data
 *   buffer - A pointer to the read-only buffer of data to be written
 *   size   - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static inline int __qemu_pci_cfg_write64(uint16_t bfd, uintptr_t addr,
                                         FAR const void *buffer,
                                         unsigned int size)
{
  int ret;

  ret = __qemu_pci_cfg_write(bfd, addr + 4, buffer + 4, 4);
  ret |= __qemu_pci_cfg_write(bfd, addr, buffer, 4);

  return ret;
}

/****************************************************************************
 * Name: __qemu_pci_cfg_read
 *
 * Description:
 *  Read 8, 16, 32 bits data from PCI-E configuration space of device
 *  specified by dev
 *
 * Input Parameters:
 *   dev    - Device private data
 *   buffer - A pointer to a buffer to receive the data from the device
 *   size   - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static inline int __qemu_pci_cfg_read(uint16_t bfd, uintptr_t addr,
                                      FAR void *buffer, unsigned int size)
{
  outl(PCI_CONE | ((uint32_t)bfd << 8) | (addr & 0xfc), PCI_REG_ADDR_PORT);

  switch (size)
    {
      case 1:
        *(uint8_t *)(buffer) = inb(PCI_REG_DATA_PORT + (addr & 0x3));
        break;
      case 2:
        *(uint16_t *)(buffer) = inw(PCI_REG_DATA_PORT + (addr & 0x3));
        break;
      case 4:
        *(uint32_t *)(buffer) = inl(PCI_REG_DATA_PORT);
        break;
      default:
        return -EINVAL;
    }

    return OK;
}

/****************************************************************************
 * Name: __qemu_pci_cfg_read
 *
 * Description:
 *  Read 64 bits data from PCI-E configuration space of device
 *  specified by dev
 *
 * Input Parameters:
 *   dev    - Device private data
 *   buffer - A pointer to a buffer to receive the data from the device
 *   size   - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static inline int __qemu_pci_cfg_read64(uint16_t bfd,
                                        uintptr_t addr,
                                        FAR void *buffer,
                                        unsigned int size)
{
  int ret;

  ret = __qemu_pci_cfg_read(bfd, addr + 4, buffer + 4, 4);
  ret |= __qemu_pci_cfg_read(bfd, addr, buffer, 4);

  return ret;
}

#endif /* __INCLUDE_NUTTX_PCIE_PCIE_READWRITE_H */

/****************************************************************************
 * include/sys/pciio.h
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

#ifndef __INCLUDE_SYS_PCIIO_H
#define __INCLUDE_SYS_PCIIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCIOCREAD      _PCIIOC(1)
#define PCIOCWRITE     _PCIIOC(2)
#define PCIOCGETROMLEN _PCIIOC(3)
#define PCIOCGETROM    _PCIIOC(4)
#define PCIOCREADMASK  _PCIIOC(5)
#define PCIOCGETVPD    _PCIIOC(6)

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

struct pcisel
{
  uint8_t pc_domain;
  uint8_t pc_bus;
  uint8_t pc_dev;
  uint8_t pc_func;
};

struct pci_io
{
  struct pcisel pi_sel;
  int pi_reg;
  int pi_width;
  uint32_t pi_data;
};

struct pci_rom
{
  struct pcisel pr_sel;
  int pr_romlen;
  FAR char *pr_rom;
};

struct pci_vpd_req
{
  struct pcisel pv_sel;
  int pv_offset;
  int pv_count;
  FAR uint32_t *pv_data;
};

#endif /* __INCLUDE_SYS_PCIIO_H */

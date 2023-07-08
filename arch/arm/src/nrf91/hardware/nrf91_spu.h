/****************************************************************************
 * arch/arm/src/nrf91/hardware/nrf91_spu.h
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

#ifndef __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_SPU_H
#define __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_SPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf91_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF91_SPU_EVENTS_RAMACCERR_OFFSET        0x100                /* A security violation has been detected for the RAM memory space */
#define NRF91_SPU_EVENTS_FLASHACCERR_OFFSET      0x104                /* A security violation has been detected for the FLASH memory space */
#define NRF91_SPU_EVENTS_PERIPHACCERR_OFFSET     0x108                /* A security violation has been detected on one or several peripherals */
#define NRF91_SPU_PUBLISH_RAMACCERR_OFFSET       0x180                /* Publish configuration for event RAMACCERR */
#define NRF91_SPU_PUBLISH_FLASHACCERR_OFFSET     0x184                /* Publish configuration for event FLASHACCERR */
#define NRF91_SPU_PUBLISH_PERIPHACCERR_OFFSET    0x188                /* Publish configuration for event PERIPHACCERR */
#define NRF91_SPU_INTEN_OFFSET                   0x300                /* Enable or disable interrupt */
#define NRF91_SPU_INTSEL_OFFSET                  0x304                /* Enable interrupt */
#define NRF91_SPU_INTCLR_OFFSET                  0x308                /* Disable interrupt */
#define NRF91_SPU_CAP_OFFSET                     0x400                /* Show implemented features for the current device */
#define NRF91_SPU_CPULOCK_OFFSET                 0x404                /* Configure bits to lock down CPU features at runtime */
#define NRF91_SPU_EXTDOMAIN_OFFSET(n)            (0x440 + (0x4 * n))  /* Access for bus access generated from the external domain n */
#define NRF91_SPU_DPPIPERM_OFFSET(n)             (0x480 + (0x8 * n))  /* Select between secure and non-secure attribute for the DPPI channels. */
#define NRF91_SPU_DPPILOCK_OFFSET(n)             (0x484 + (0x8 * n))  /* Prevent further modification of the corresponding PERM register */
#define NRF91_SPU_GPIOPORTPERM_OFFSET(n)         (0x4C0 + (0x8 * n))  /* Select between secure and non-secure attribute for pins 0 to 31 of port n. */
#define NRF91_SPU_GPIOPORTLOCK_OFFSET(n)         (0x4C4 + (0x8 * n))  /* Prevent further modification of the corresponding PERM register */
#define NRF91_SPU_FLASHNSCREGION_OFFSET(n)       (0x500 + (0x8 * n))  /* Define which flash region can contain the non-secure callable (NSC) region n */
#define NRF91_SPU_FLASHNSCSIZE_OFFSET(n)         (0x504 + (0x8 * n))  /* Define the size of the non-secure callable (NSC) region n */
#define NRF91_SPU_RAMNSCREGION_OFFSET(n)         (0x540 + (0x8 * n))  /* Define which RAM region can contain the non-secure callable (NSC) region n */
#define NRF91_SPU_RAMNSCSIZE_OFFSET(n)           (0x544 + (0x8 * n))  /* Define the size of the non-secure callable (NSC) region n */
#define NRF91_SPU_FLASHREGIONPERM_OFFSET(n)      (0x600 + (0x4 * n))  /* Access permissions for flash region n */
#define NRF91_SPU_RAMREGIONPERM_OFFSET(n)        (0x700 + (0x4 * n))  /* Access permissions for RAM region n */
#define NRF91_SPU_PERIPHIDPERM_OFFSET(n)         (0x800 + (0x4 * n))  /* List capabilities and access permissions for the peripheral with ID n */

/* Register definitions *****************************************************/

#define NRF91_SPU_EVENTS_RAMACCERR        (NRF91_SPU_BASE + NRF91_SPU_EVENTS_RAMACCERR_OFFSET)
#define NRF91_SPU_EVENTS_FLASHACCERR      (NRF91_SPU_BASE + NRF91_SPU_EVENTS_FLASHACCERR_OFFSET)
#define NRF91_SPU_EVENTS_PERIPHACCERR     (NRF91_SPU_BASE + NRF91_SPU_EVENTS_PERIPHACCERR_OFFSET)
#define NRF91_SPU_PUBLISH_RAMACCERR       (NRF91_SPU_BASE + NRF91_SPU_PUBLISH_RAMACCERR_OFFSET)
#define NRF91_SPU_PUBLISH_FLASHACCERR     (NRF91_SPU_BASE + NRF91_SPU_PUBLISH_FLASHACCERR_OFFSET)
#define NRF91_SPU_PUBLISH_PERIPHACCERR    (NRF91_SPU_BASE + NRF91_SPU_PUBLISH_PERIPHACCERR_OFFSET)
#define NRF91_SPU_INTEN                   (NRF91_SPU_BASE + NRF91_SPU_INTEN_OFFSET)
#define NRF91_SPU_INTSEL                  (NRF91_SPU_BASE + NRF91_SPU_INTSEL_OFFSET)
#define NRF91_SPU_INTCLR                  (NRF91_SPU_BASE + NRF91_SPU_INTCLR_OFFSET)
#define NRF91_SPU_CAP                     (NRF91_SPU_BASE + NRF91_SPU_CAP_OFFSET)
#define NRF91_SPU_CPULOCK                 (NRF91_SPU_BASE + NRF91_SPU_CPULOCK_OFFSET)
#define NRF91_SPU_EXTDOMAIN(n)            (NRF91_SPU_BASE + NRF91_SPU_EXTDOMAIN_OFFSET(n))
#define NRF91_SPU_DPPIPERM(n)             (NRF91_SPU_BASE + NRF91_SPU_DPPIPERM_OFFSET(n))
#define NRF91_SPU_DPPILOCK(n)             (NRF91_SPU_BASE + NRF91_SPU_DPPILOCK_OFFSET(n))
#define NRF91_SPU_GPIOPORTPERM(n)         (NRF91_SPU_BASE + NRF91_SPU_GPIOPORTPERM_OFFSET(n))
#define NRF91_SPU_GPIOPORTLOCK(n)         (NRF91_SPU_BASE + NRF91_SPU_GPIOPORTLOCK_OFFSET(n))
#define NRF91_SPU_FLASHNSCREGION(n)       (NRF91_SPU_BASE + NRF91_SPU_FLASHNSCREGION_OFFSET(n))
#define NRF91_SPU_FLASHNSCSIZE(n)         (NRF91_SPU_BASE + NRF91_SPU_FLASHNSCSIZE_OFFSET(n))
#define NRF91_SPU_RAMNSCREGION(n)         (NRF91_SPU_BASE + NRF91_SPU_RAMNSCREGION_OFFSET(n))
#define NRF91_SPU_RAMNSCSIZE(n)           (NRF91_SPU_BASE + NRF91_SPU_RAMNSCSIZE_OFFSET(n))
#define NRF91_SPU_FLASHREGIONPERM(n)      (NRF91_SPU_BASE + NRF91_SPU_FLASHREGIONPERM_OFFSET(n))
#define NRF91_SPU_RAMREGIONPERM(n)        (NRF91_SPU_BASE + NRF91_SPU_RAMREGIONPERM_OFFSET(n))
#define NRF91_SPU_PERIPHIDPERM(n)         (NRF91_SPU_BASE + NRF91_SPU_PERIPHIDPERM_OFFSET(n))

/* Register bit definitions *************************************************/

#define SPU_RAM_REGIONS                      (32)
#define SPU_FLASH_REGIONS                    (32)

#define SPU_EXTDOMAIN_SECUREMAPPING_SHIFT    (0)
#define SPU_EXTDOMAIN_SECUREMAPPING_MASK     (3 << SPU_EXTDOMAIN_SECUREMAPPING_SHIFT)
#  define SPU_EXTDOMAIN_SECUREMAPPING_NONSEC (0 << SPU_EXTDOMAIN_SECUREMAPPING_SHIFT)
#  define SPU_EXTDOMAIN_SECUREMAPPING_SEC    (1 << SPU_EXTDOMAIN_SECUREMAPPING_SHIFT)
#  define SPU_EXTDOMAIN_SECUREMAPPING_USER   (2 << SPU_EXTDOMAIN_SECUREMAPPING_SHIFT)
#define SPU_EXTDOMAIN_SECUREMAPPING_SECATTR  (1 << 4)
#define SPU_EXTDOMAIN_SECUREMAPPING_LOCK     (1 << 8)

#define SPU_FLASHREGION_PERM_EXEC            (1 << 0)
#define SPU_FLASHREGION_PERM_WRITE           (1 << 1)
#define SPU_FLASHREGION_PERM_READ            (1 << 2)
#define SPU_FLASHREGION_PERM_SECATTR         (1 << 4)
#define SPU_FLASHREGION_PERM_LOCK            (1 << 8)

#define SPU_RAMREGION_PERM_EXEC              (1 << 0)
#define SPU_RAMREGION_PERM_WRITE             (1 << 1)
#define SPU_RAMREGION_PERM_READ              (1 << 2)
#define SPU_RAMREGION_PERM_SECATTR           (1 << 4)
#define SPU_RAMREGION_PERM_LOCK              (1 << 8)

#define SPU_PERIPHID_SECUREMAPPING_SHIFT     (0)
#define SPU_PERIPHID_SECUREMAPPING_MASK      (3 << SPU_PERIPHID_SECUREMAPPING_SHIFT)
#  define SPU_PERIPHID_SECUREMAPPING_NONSEC  (0 << SPU_PERIPHID_SECUREMAPPING_SHIFT)
#  define SPU_PERIPHID_SECUREMAPPING_SEC     (1 << SPU_PERIPHID_SECUREMAPPING_SHIFT)
#  define SPU_PERIPHID_SECUREMAPPING_USER    (2 << SPU_PERIPHID_SECUREMAPPING_SHIFT)
#  define SPU_PERIPHID_SECUREMAPPING_SPLIT   (3 << SPU_PERIPHID_SECUREMAPPING_SHIFT)
#define SPU_PERIPHID_PERM_SECATTR            (1 << 4)
#define SPU_PERIPHID_PERM_DMASEC             (1 << 5)

#endif /* __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_SPU_H */

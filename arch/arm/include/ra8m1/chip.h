/****************************************************************************
 * arch/arm/include/ra8m1/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_RA8M1_CHIP_H
#define __ARCH_ARM_INCLUDE_RA8M1_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_R7FA8M1AFECAM)
#  define RA_NSCI_B  6  /* SCI_B0-4, SCI_B9 (not contiguous: no SCI_B5-8) */

#  define RA_RAM_SIZE         (896*1024)   /* 896Kb RAM */
#  define RA_FLASH_SIZE       (1024*1024)  /* 1024Kb FLASH */
#  define RA_DATA_FLASH_SIZE  (12*1024)    /* 12Kb DATA_FLASH */
#  if defined(CONFIG_ARMV8M_HAVE_ITCM)
#      define RA_ITCM_SIZE    (64*1024)    /* 64Kb ITCM on TCM interface */
#  else
#      define RA_ITCM_SIZE    (0)          /* No ITCM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV8M_HAVE_DTCM)
#      define RA_DTCM_SIZE    (64*1024)    /* 64Kb DTCM on TCM interface */
#  else
#      define RA_DTCM_SIZE    (0)          /* No DTCM on TCM interface */
#  endif

#elif defined(CONFIG_ARCH_CHIP_R7FA8M1AFECBD)
#  define RA_NSCI_B  6  /* SCI_B0-4, SCI_B9 (not contiguous: no SCI_B5-8) */

#  define RA_RAM_SIZE         (896*1024)   /* 896Kb RAM */
#  define RA_FLASH_SIZE       (1024*1024)  /* 1024Kb FLASH */
#  define RA_DATA_FLASH_SIZE  (12*1024)    /* 12Kb DATA_FLASH */
#  if defined(CONFIG_ARMV8M_HAVE_ITCM)
#      define RA_ITCM_SIZE    (64*1024)    /* 64Kb ITCM on TCM interface */
#  else
#      define RA_ITCM_SIZE    (0)          /* No ITCM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV8M_HAVE_DTCM)
#      define RA_DTCM_SIZE    (64*1024)    /* 64Kb DTCM on TCM interface */
#  else
#      define RA_DTCM_SIZE    (0)          /* No DTCM on TCM interface */
#  endif

#elif defined(CONFIG_ARCH_CHIP_R7FA8M1AFECFB)
#  define RA_NSCI_B  6  /* SCI_B0-4, SCI_B9 (not contiguous: no SCI_B5-8) */

#  define RA_RAM_SIZE         (896*1024)   /* 896Kb RAM */
#  define RA_FLASH_SIZE       (1024*1024)  /* 1024Kb FLASH */
#  define RA_DATA_FLASH_SIZE  (12*1024)    /* 12Kb DATA_FLASH */
#  if defined(CONFIG_ARMV8M_HAVE_ITCM)
#      define RA_ITCM_SIZE    (64*1024)    /* 64Kb ITCM on TCM interface */
#  else
#      define RA_ITCM_SIZE    (0)          /* No ITCM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV8M_HAVE_DTCM)
#      define RA_DTCM_SIZE    (64*1024)    /* 64Kb DTCM on TCM interface */
#  else
#      define RA_DTCM_SIZE    (0)          /* No DTCM on TCM interface */
#  endif

#elif defined(CONFIG_ARCH_CHIP_R7FA8M1AFECFC)
#  define RA_NSCI_B  6  /* SCI_B0-4, SCI_B9 (not contiguous: no SCI_B5-8) */

#  define RA_RAM_SIZE         (896*1024)   /* 896Kb RAM */
#  define RA_FLASH_SIZE       (1024*1024)  /* 1024Kb FLASH */
#  define RA_DATA_FLASH_SIZE  (12*1024)    /* 12Kb DATA_FLASH */
#  if defined(CONFIG_ARMV8M_HAVE_ITCM)
#      define RA_ITCM_SIZE    (64*1024)    /* 64Kb ITCM on TCM interface */
#  else
#      define RA_ITCM_SIZE    (0)          /* No ITCM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV8M_HAVE_DTCM)
#      define RA_DTCM_SIZE    (64*1024)    /* 64Kb DTCM on TCM interface */
#  else
#      define RA_DTCM_SIZE    (0)          /* No DTCM on TCM interface */
#  endif

#elif defined(CONFIG_ARCH_CHIP_R7FA8M1AFECFP)
#  define RA_NSCI_B  6  /* SCI_B0-4, SCI_B9 (not contiguous: no SCI_B5-8) */

#  define RA_RAM_SIZE         (896*1024)   /* 896Kb RAM */
#  define RA_FLASH_SIZE       (1024*1024)  /* 1024Kb FLASH */
#  define RA_DATA_FLASH_SIZE  (12*1024)    /* 12Kb DATA_FLASH */
#  if defined(CONFIG_ARMV8M_HAVE_ITCM)
#      define RA_ITCM_SIZE    (64*1024)    /* 64Kb ITCM on TCM interface */
#  else
#      define RA_ITCM_SIZE    (0)          /* No ITCM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV8M_HAVE_DTCM)
#      define RA_DTCM_SIZE    (64*1024)    /* 64Kb DTCM on TCM interface */
#  else
#      define RA_DTCM_SIZE    (0)          /* No DTCM on TCM interface */
#  endif

#elif defined(CONFIG_ARCH_CHIP_R7FA8M1AHECAM)
#  define RA_NSCI_B  6  /* SCI_B0-4, SCI_B9 (not contiguous: no SCI_B5-8) */

#  define RA_RAM_SIZE         (896*1024)   /* 896Kb RAM */
#  define RA_FLASH_SIZE       (2016*1024)  /* 2016Kb FLASH */
#  define RA_DATA_FLASH_SIZE  (12*1024)    /* 12Kb DATA_FLASH */
#  if defined(CONFIG_ARMV8M_HAVE_ITCM)
#      define RA_ITCM_SIZE    (64*1024)    /* 64Kb ITCM on TCM interface */
#  else
#      define RA_ITCM_SIZE    (0)          /* No ITCM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV8M_HAVE_DTCM)
#      define RA_DTCM_SIZE    (64*1024)    /* 64Kb DTCM on TCM interface */
#  else
#      define RA_DTCM_SIZE    (0)          /* No DTCM on TCM interface */
#  endif

#elif defined(CONFIG_ARCH_CHIP_R7FA8M1AHECBD)
#  define RA_NSCI_B  6  /* SCI_B0-4, SCI_B9 (not contiguous: no SCI_B5-8) */

#  define RA_RAM_SIZE         (896*1024)   /* 896Kb RAM */
#  define RA_FLASH_SIZE       (2016*1024)  /* 2016Kb FLASH */
#  define RA_DATA_FLASH_SIZE  (12*1024)    /* 12Kb DATA_FLASH */
#  if defined(CONFIG_ARMV8M_HAVE_ITCM)
#      define RA_ITCM_SIZE    (64*1024)    /* 64Kb ITCM on TCM interface */
#  else
#      define RA_ITCM_SIZE    (0)          /* No ITCM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV8M_HAVE_DTCM)
#      define RA_DTCM_SIZE    (64*1024)    /* 64Kb DTCM on TCM interface */
#  else
#      define RA_DTCM_SIZE    (0)          /* No DTCM on TCM interface */
#  endif

#elif defined(CONFIG_ARCH_CHIP_R7FA8M1AHECFB)
#  define RA_NSCI_B  6  /* SCI_B0-4, SCI_B9 (not contiguous: no SCI_B5-8) */

#  define RA_RAM_SIZE         (896*1024)   /* 896Kb RAM */
#  define RA_FLASH_SIZE       (2016*1024)  /* 2016Kb FLASH */
#  define RA_DATA_FLASH_SIZE  (12*1024)    /* 12Kb DATA_FLASH */
#  if defined(CONFIG_ARMV8M_HAVE_ITCM)
#      define RA_ITCM_SIZE    (64*1024)    /* 64Kb ITCM on TCM interface */
#  else
#      define RA_ITCM_SIZE    (0)          /* No ITCM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV8M_HAVE_DTCM)
#      define RA_DTCM_SIZE    (64*1024)    /* 64Kb DTCM on TCM interface */
#  else
#      define RA_DTCM_SIZE    (0)          /* No DTCM on TCM interface */
#  endif

#elif defined(CONFIG_ARCH_CHIP_R7FA8M1AHECFC)
#  define RA_NSCI_B  6  /* SCI_B0-4, SCI_B9 (not contiguous: no SCI_B5-8) */

#  define RA_RAM_SIZE         (896*1024)   /* 896Kb RAM */
#  define RA_FLASH_SIZE       (2016*1024)  /* 2016Kb FLASH */
#  define RA_DATA_FLASH_SIZE  (12*1024)    /* 12Kb DATA_FLASH */
#  if defined(CONFIG_ARMV8M_HAVE_ITCM)
#      define RA_ITCM_SIZE    (64*1024)    /* 64Kb ITCM on TCM interface */
#  else
#      define RA_ITCM_SIZE    (0)          /* No ITCM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV8M_HAVE_DTCM)
#      define RA_DTCM_SIZE    (64*1024)    /* 64Kb DTCM on TCM interface */
#  else
#      define RA_DTCM_SIZE    (0)          /* No DTCM on TCM interface */
#  endif

#elif defined(CONFIG_ARCH_CHIP_R7FA8M1AHECFP)
#  define RA_NSCI_B  6  /* SCI_B0-4, SCI_B9 (not contiguous: no SCI_B5-8) */

#  define RA_RAM_SIZE         (896*1024)   /* 896Kb RAM */
#  define RA_FLASH_SIZE       (2016*1024)  /* 2016Kb FLASH */
#  define RA_DATA_FLASH_SIZE  (12*1024)    /* 12Kb DATA_FLASH */
#  if defined(CONFIG_ARMV8M_HAVE_ITCM)
#      define RA_ITCM_SIZE    (64*1024)    /* 64Kb ITCM on TCM interface */
#  else
#      define RA_ITCM_SIZE    (0)          /* No ITCM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV8M_HAVE_DTCM)
#      define RA_DTCM_SIZE    (64*1024)    /* 64Kb DTCM on TCM interface */
#  else
#      define RA_DTCM_SIZE    (0)          /* No DTCM on TCM interface */
#  endif

#else
#  error "Unsupported RA8M1 chip"
#endif

/* NVIC Priority Levels *****************************************************/

/* Each priority field holds a priority value, 0-15. The lower
 * the value, the greater the priority of the corresponding
 * interrupt. The processor implements only bits[7:4] of each
 * field, bits[3:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN      0xf0  /* All bits[7:4] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT  0x80  /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX      0x00  /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP     0x10  /* Four bits of interrupt priority used */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_RA8M1_CHIP_H */

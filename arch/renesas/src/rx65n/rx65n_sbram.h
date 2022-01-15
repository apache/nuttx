/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_sbram.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_RX65N_SBSRAM_H
#define __ARCH_RENESAS_SRC_RX65N_RX65N_SBSRAM_H

/****************************************************************************
 * The purpose of this driver is to add battery backup file to the file
 * system. There can be CONFIG_RX65N_BBRSRAM_COUNT files defined.
 * These files are of fixed size up to the maximum of the backing 4K SRAM.
 *
 * If CONFIG_SAVE_CRASHDUMP is defined The driver also supports a feature
 * to save the context of a PANIC in one of these files.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#  define RX65N_SBRAM_SIZE 8192

#if !defined(CONFIG_RX65N_SBRAM_FILES)
#  define CONFIG_RX65N_SBRAM_FILES 4
#endif

/* REVISIT: What guarantees that RX65N_SBRAM_GETDESC_IOCTL has a unique
 * value among all over _DIOC() values?
 */

#define RX65N_SBRAM_GETDESC_IOCTL _DIOC(0x0010) /* Returns a sbramd_s */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

enum sbramdf_e
{
  SBRAM_CRC_VALID = 1,        /* The crc is valid */
  SBRAM_DIRTY     = 2,        /* The file was closed */
};

struct sbramd_s
{
  uint8_t flags;               /* The crc is valid and the file was closed */
  uint8_t fileno;              /* The minor number */
  uint16_t len;                /* Total Bytes in this file */
  struct timespec lastwrite;   /* Last write time */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Function: rx65n_sbraminitialize
 *
 * Description:
 *   Initialize the Battery Backed up SRAM driver.
 *
 * Input Parameters:
 *   devpath - the path to instantiate the files.
 *   sizes   - Pointer to a any array of file sizes to create
 *             the last entry should be 0
 *             A size of -1 will use all the remaining spaces
 *
 * If the length of sizes is greater then CONFIG_RX65N_SBRAM_FILES
 * CONFIG_RX65N_SBRAM_FILES will be returned.
 *
 * Returned Value:
 *   Number of files created on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int rx65n_sbraminitialize(char *devpath, int *sizes);

/****************************************************************************
* Function: rx65n_sbram_savepanic
*
* Description:
*   Saves the panic context in a previously allocated SBRAM file
*
* Parameters:
*   fileno  - the value returned by the ioctl RX65N_SBRAM_GETDESC_IOCTL
*   context - Pointer to a any array of bytes to save
*   length  - The length of the data pointed to byt context
*
* Returned Value:
*   Length saved or negated errno.
*
* Assumptions:
*
*****************************************************************************/

#if defined(CONFIG_RX65N_SAVE_CRASHDUMP)
int rx65n_sbram_savepanic(int fileno, uint8_t *context, int length);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RENESAS_SRC_RX65N_RX65N_SBSRAM_H */

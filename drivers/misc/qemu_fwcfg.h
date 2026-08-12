/****************************************************************************
 * drivers/misc/qemu_fwcfg.h
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

#ifndef __DRIVERS_MISC_QEMU_FWCFG_H
#define __DRIVERS_MISC_QEMU_FWCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Well-known fw_cfg selector keys.
 *
 * See the authoritative list in QEMU "docs/specs/fw_cfg.rst".  Keys in the
 * 0x0000-0x7fff range are generic; 0x8000-0xffff are architecture specific.
 * Only a few of these are used internally by the driver (signature, id and
 * the file directory); the rest are documented for completeness.  User
 * space only sees the "file" items exposed through the configdata ioctls.
 */

#define FW_CFG_SIGNATURE      0x0000  /* "QEMU" signature (4 bytes)         */
#define FW_CFG_ID             0x0001  /* Revision / feature bitmap (LE32)   */
#define FW_CFG_UUID           0x0002  /* System UUID                        */
#define FW_CFG_RAM_SIZE       0x0003  /* RAM size                           */
#define FW_CFG_NOGRAPHIC      0x0004  /* No-graphic flag                    */
#define FW_CFG_NB_CPUS        0x0005  /* Number of CPUs                     */
#define FW_CFG_MACHINE_ID     0x0006  /* Machine ID                         */
#define FW_CFG_KERNEL_ADDR    0x0007  /* Kernel load address                */
#define FW_CFG_KERNEL_SIZE    0x0008  /* Kernel size                        */
#define FW_CFG_KERNEL_CMDLINE 0x0009  /* Kernel command line                */
#define FW_CFG_INITRD_ADDR    0x000a  /* Initrd load address                */
#define FW_CFG_INITRD_SIZE    0x000b  /* Initrd size                        */
#define FW_CFG_BOOT_DEVICE    0x000c  /* Boot device                        */
#define FW_CFG_NUMA           0x000d  /* NUMA information                   */
#define FW_CFG_BOOT_MENU      0x000e  /* Boot menu flag                     */
#define FW_CFG_MAX_CPUS       0x000f  /* Maximum number of CPUs             */
#define FW_CFG_KERNEL_ENTRY   0x0010  /* Kernel entry point                 */
#define FW_CFG_KERNEL_DATA    0x0011  /* Kernel data                        */
#define FW_CFG_INITRD_DATA    0x0012  /* Initrd data                        */
#define FW_CFG_CMDLINE_ADDR   0x0013  /* Command line address               */
#define FW_CFG_CMDLINE_SIZE   0x0014  /* Command line size                  */
#define FW_CFG_CMDLINE_DATA   0x0015  /* Command line data                  */
#define FW_CFG_SETUP_ADDR     0x0016  /* Setup address                      */
#define FW_CFG_SETUP_SIZE     0x0017  /* Setup size                         */
#define FW_CFG_SETUP_DATA     0x0018  /* Setup data                         */
#define FW_CFG_FILE_DIR       0x0019  /* File directory                     */

#define FW_CFG_FILE_FIRST     0x0020  /* First user-defined file entry      */
#define FW_CFG_FILE_SLOTS_MIN 0x0010  /* Minimum number of file slots       */

/* Selector key modifier bits */

#define FW_CFG_WRITE_CHANNEL  0x4000  /* Write mode (bit 14)                */
#define FW_CFG_ARCH_LOCAL     0x8000  /* Architecture specific (bit 15)     */
#define FW_CFG_ENTRY_MASK     (~(FW_CFG_WRITE_CHANNEL | FW_CFG_ARCH_LOCAL))

#define FW_CFG_INVALID        0xffff  /* Invalid selector key               */

/* FW_CFG_ID feature bits */

#define FW_CFG_VERSION        0x01    /* Traditional interface (always set) */
#define FW_CFG_VERSION_DMA    0x02    /* DMA interface available            */

/* An fw_cfg "file name" is up to 56 characters (including terminating NUL) */

#define FW_CFG_MAX_FILE_PATH  56

/* Size in bytes of the fw_cfg signature */

#define FW_CFG_SIG_SIZE       4

#endif /* __DRIVERS_MISC_QEMU_FWCFG_H */

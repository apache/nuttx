/****************************************************************************
 * arch/risc-v/src/common/riscv_sbi.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_RISCV_SBI_H
#define __ARCH_RISCV_SRC_COMMON_RISCV_SBI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SBI Extension IDs */

#define SBI_EXT_BASE            0x00000010
#define SBI_EXT_IPI             0x00735049
#define SBI_EXT_TIME            0x54494D45
#define SBI_EXT_RFENCE          0x52464E43
#define SBI_EXT_HSM             0x0048534D
#define SBI_EXT_SRST            0x53525354
#define SBI_EXT_PMU             0x00504D55
#define SBI_EXT_DBCN            0x4442434E
#define SBI_EXT_SUSP            0x53555350

/* SBI function IDs for BASE extension */

#define SBI_EXT_BASE_GET_SPEC_VERSION           0x0
#define SBI_EXT_BASE_GET_IMP_ID                 0x1
#define SBI_EXT_BASE_GET_IMP_VERSION            0x2
#define SBI_EXT_BASE_PROBE_EXT                  0x3
#define SBI_EXT_BASE_GET_MVENDORID              0x4
#define SBI_EXT_BASE_GET_MARCHID                0x5
#define SBI_EXT_BASE_GET_MIMPID                 0x6

/* SBI function IDs for TIME extension */

#define SBI_EXT_TIME_SET_TIMER                  0x0

/* SBI function IDs for IPI extension */

#define SBI_EXT_IPI_SEND_IPI                    0x0

/* SBI function IDs for RFENCE extension */
#define SBI_EXT_RFENCE_REMOTE_FENCE_I           0x0
#define SBI_EXT_RFENCE_REMOTE_SFENCE_VMA        0x1
#define SBI_EXT_RFENCE_REMOTE_SFENCE_VMA_ASID   0x2
#define SBI_EXT_RFENCE_REMOTE_HFENCE_GVMA_VMID  0x3
#define SBI_EXT_RFENCE_REMOTE_HFENCE_GVMA       0x4
#define SBI_EXT_RFENCE_REMOTE_HFENCE_VVMA_ASID  0x5
#define SBI_EXT_RFENCE_REMOTE_HFENCE_VVMA       0x6

/* SBI function IDs for HSM extension */

#define SBI_EXT_HSM_HART_START                  0x0
#define SBI_EXT_HSM_HART_STOP                   0x1
#define SBI_EXT_HSM_HART_GET_STATUS             0x2
#define SBI_EXT_HSM_HART_SUSPEND                0x3

#define SBI_HSM_STATE_STARTED                   0x0
#define SBI_HSM_STATE_STOPPED                   0x1
#define SBI_HSM_STATE_START_PENDING             0x2
#define SBI_HSM_STATE_STOP_PENDING              0x3
#define SBI_HSM_STATE_SUSPENDED                 0x4
#define SBI_HSM_STATE_SUSPEND_PENDING           0x5
#define SBI_HSM_STATE_RESUME_PENDING            0x6

/* SBI function IDs for SRST extension */

#define SBI_EXT_SRST_SYS_RESET                  0x0

/* SBI system reset type */

#define SBI_SRST_TYPE_SHUTDOWN                  0
#define SBI_SRST_TYPE_REBOOT_COLD               1
#define SBI_SRST_TYPE_REBOOT_WARM               1

/* SBI system reset reason */

#define SBI_SRST_REASON_NONE                    0
#define SBI_SRST_REASON_FAILURE                 1

/* SBI function IDs for PMU extension */

#define SBI_EXT_PMU_NUM_COUNTERS                0x0
#define SBI_EXT_PMU_COUNTER_GET_INFO            0x1
#define SBI_EXT_PMU_COUNTER_CFG_MATCH           0x2
#define SBI_EXT_PMU_COUNTER_START               0x3
#define SBI_EXT_PMU_COUNTER_STOP                0x4
#define SBI_EXT_PMU_COUNTER_FW_READ             0x5
#define SBI_EXT_PMU_COUNTER_FW_READ_HI          0x6
#define SBI_EXT_PMU_SNAPSHOT_SET_SHMEM          0x7

/* SBI PMU counter config match flags */

#define SBI_PMU_CFG_FLAG_SKIP_MATCH             (1 << 0)
#define SBI_PMU_CFG_FLAG_CLEAR_VALUE            (1 << 1)
#define SBI_PMU_CFG_FLAG_AUTO_START             (1 << 2)
#define SBI_PMU_CFG_FLAG_SET_VUINH              (1 << 3)
#define SBI_PMU_CFG_FLAG_SET_VSINH              (1 << 4)
#define SBI_PMU_CFG_FLAG_SET_UINH               (1 << 5)
#define SBI_PMU_CFG_FLAG_SET_SINH               (1 << 6)
#define SBI_PMU_CFG_FLAG_SET_MINH               (1 << 7)

/* SBI PMU counter start flags */

#define SBI_PMU_START_FLAG_SET_INIT_VALUE       (1 << 0)
#define SBI_PMU_START_FLAG_INIT_FROM_SNAPSHOT   (1 << 1)

/* SBI PMU counter stop flags */

#define SBI_PMU_STOP_FLAG_RESET                 (1 << 0)
#define SBI_PMU_STOP_FLAG_TAKE_SNAPSHOT         (1 << 1)

/* SBI function IDs for DBCN extension */

#define SBI_EXT_DBCN_CONSOLE_WRITE              0x0
#define SBI_EXT_DBCN_CONSOLE_READ               0x1
#define SBI_EXT_DBCN_CONSOLE_WRITE_BYTE         0x2

/* SBI function IDs for SUSP extension */

#define SBI_EXT_SUSP_SUSPEND                    0x0

/* SBI system suspend type */

#define SBI_SUSP_SLEEP_TYPE_SUSPEND             0x0

/* SBI return error codes */

#define SBI_SUCCESS                             0
#define SBI_ERR_FAILED                          -1
#define SBI_ERR_NOT_SUPPORTED                   -2
#define SBI_ERR_INVALID_PARAM                   -3
#define SBI_ERR_DENIED                          -4
#define SBI_ERR_INVALID_ADDRESS                 -5
#define SBI_ERR_ALREADY_AVAILABLE               -6
#define SBI_ERR_ALREADY_STARTED                 -7
#define SBI_ERR_ALREADY_STOPPED                 -8
#define SBI_ERR_NO_SHMEM                        -9
#define SBI_ERR_INVALID_STATE                   -10
#define SBI_ERR_BAD_RANGE                       -11

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct sbiret_s
{
  intreg_t    error;
  uintreg_t   value;
};
typedef struct sbiret_s sbiret_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* RISC-V SBI wrappers ******************************************************/

#ifdef CONFIG_ARCH_USE_S_MODE

/****************************************************************************
 * Name: sbi_err_map_errno
 *
 * Description:
 *   Convert SBI error value to posix error value
 *
 * Input Parameters:
 *   err - error value of SBI function
 *
 * Return:
 *   Posix error value representing the input
 *
 ****************************************************************************/

int sbi_err_map_errno(intreg_t err);

/****************************************************************************
 * Name: riscv_sbi_send_ipi
 *
 * Description:
 *   Send a ipi to the target harts
 *
 * Input Parameters:
 *   hmask - bit mask for target harts
 *   hbase - the start hart id to send
 *
 ****************************************************************************/

void riscv_sbi_send_ipi(uintreg_t hmask, uintreg_t hbase);

/****************************************************************************
 * Name: riscv_sbi_set_timer
 *
 * Description:
 *   Set new compare match value for timer
 *
 * Input Parameters:
 *   stime_value - Value to set
 *
 ****************************************************************************/

void riscv_sbi_set_timer(uint64_t stime_value);

/****************************************************************************
 * Name: riscv_sbi_get_time
 *
 * Description:
 *   Get value of mtime
 *
 * Return:
 *   Value of mtime
 *
 ****************************************************************************/

uint64_t riscv_sbi_get_time(void);

/****************************************************************************
 * Name: riscv_sbi_boot_secondary
 *
 * Description:
 *   Start the target hart
 *
 * Input Parameters:
 *   hartid - hart id to start
 *   addr   - start address for the hart
 *   a1     - opaque parameter for the hart
 *
 ****************************************************************************/

int riscv_sbi_boot_secondary(uintreg_t hartid, uintreg_t addr,
                                  uintreg_t a1);

/****************************************************************************
 * Name: riscv_sbi_system_reset
 *
 * Description:
 *   Reset the system with specific type and reason.
 *
 ****************************************************************************/

int riscv_sbi_system_reset(uint32_t type, uint32_t reason);

#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_SRC_COMMON_RISCV_SBI_H */

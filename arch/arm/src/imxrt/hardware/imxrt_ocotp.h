/************************************************************************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_ocotp.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_OCOTP_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_OCOTP_H

/* The OCOTP IP block provides a set of register to access the On Chip OPT.
 * It also provides a shadow image of the 64 OTP entries that are read only
 * memory addressable (OCOTP Shadow Offsets). To read or write the actual
 * OTP, OCOTP Indexes are used.
 */

/************************************************************************************************************************************
 * Included Files
 ************************************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/************************************************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************************************************/

/* Register Offsets *****************************************************************/

#define IMXRT_OCOTP_CTRL_OFFSET               0x0000  /* OTP Controller Control Register */
#define IMXRT_OCOTP_CTRL_SET_OFFSET           0x0004  /* OTP Controller Control Register */
#define IMXRT_OCOTP_CTRL_CLR_OFFSET           0x0008  /* OTP Controller Control Register */
#define IMXRT_OCOTP_CTRL_TOG_OFFSET           0x000c  /* OTP Controller Control Register */
#define IMXRT_OCOTP_TIMING_OFFSET             0x0010  /* OTP Controller Timing Register */
#define IMXRT_OCOTP_DATA_OFFSET               0x0020  /* OTP Controller Write Data Register */
#define IMXRT_OCOTP_READ_CTRL_OFFSET          0x0030  /* OTP Controller Write Data Register */
#define IMXRT_OCOTP_READ_FUSE_DATA_OFFSET     0x0040  /* OTP Controller Read Data Register */
#define IMXRT_OCOTP_SW_STICKY_OFFSET          0x0050  /* Sticky bit Register */
#define IMXRT_OCOTP_SCS_OFFSET                0x0060  /* Software Controllable Signals Register */
#define IMXRT_OCOTP_SCS_SET_OFFSET            0x0064  /* Software Controllable Signals Register */
#define IMXRT_OCOTP_SCS_CLR_OFFSET            0x0068  /* Software Controllable Signals Register */
#define IMXRT_OCOTP_SCS_TOG_OFFSET            0x006c  /* Software Controllable Signals Register */
#define IMXRT_OCOTP_CRC_ADDR_OFFSET           0x0070  /* OTP Controller CRC test address */
#define IMXRT_OCOTP_CRC_VALUE_OFFSET          0x0080  /* OTP Controller CRC Value Register */
#define IMXRT_OCOTP_VERSION_OFFSET            0x0090  /* OTP Controller Version Register */
#define IMXRT_OCOTP_TIMING2_OFFSET            0x0100  /* OTP Controller Timing Register */

/* OCOTP Shadow Offsets *************************************************************/

#define IMXRT_OCOTP_LOCK_OFFSET               0x0400  /* Value of OTP Bank0 Word0 (Lock controls) */
#define IMXRT_OCOTP_CFG0_OFFSET               0x0410  /* Value of OTP Bank0 Word1 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG1_OFFSET               0x0420  /* Value of OTP Bank0 Word2 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG2_OFFSET               0x0430  /* Value of OTP Bank0 Word3 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG3_OFFSET               0x0440  /* Value of OTP Bank0 Word4 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG4_OFFSET               0x0450  /* Value of OTP Bank0 Word5 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG5_OFFSET               0x0460  /* Value of OTP Bank0 Word6 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG6_OFFSET               0x0470  /* Value of OTP Bank0 Word7 (Configuration and Manufacturing */
#define IMXRT_OCOTP_MEM0_OFFSET               0x0480  /* Value of OTP Bank1 Word0 (Memory Related Info.) */
#define IMXRT_OCOTP_MEM1_OFFSET               0x0490  /* Value of OTP Bank1 Word1 (Memory Related Info.) */
#define IMXRT_OCOTP_MEM2_OFFSET               0x04a0  /* Value of OTP Bank1 Word2 (Memory Related Info.) */
#define IMXRT_OCOTP_MEM3_OFFSET               0x04b0  /* Value of OTP Bank1 Word3 (Memory Related Info.) */
#define IMXRT_OCOTP_MEM4_OFFSET               0x04c0  /* Value of OTP Bank1 Word4 (Memory Related Info.) */
#define IMXRT_OCOTP_ANA0_OFFSET               0x04d0  /* Value of OTP Bank1 Word5 (Analog Info.) */
#define IMXRT_OCOTP_ANA1_OFFSET               0x04e0  /* Value of OTP Bank1 Word6 (Analog Info.) */
#define IMXRT_OCOTP_ANA2_OFFSET               0x04f0  /* Value of OTP Bank1 Word7 (Analog Info.) */
#define IMXRT_OCOTP_OTPMK0_OFFSET             0x0500  /* Value of OTP Bank2 Word0 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK1_OFFSET             0x0510  /* Value of OTP Bank2 Word1 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK2_OFFSET             0x0520  /* Value of OTP Bank2 Word2 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK3_OFFSET             0x0530  /* Value of OTP Bank2 Word3 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK4_OFFSET             0x0540  /* Value of OTP Bank2 Word4 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK5_OFFSET             0x0550  /* Value of OTP Bank2 Word5 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK6_OFFSET             0x0560  /* Value of OTP Bank2 Word6 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK7_OFFSET             0x0570  /* Value of OTP Bank2 Word7 (OTPMK Key) */
#define IMXRT_OCOTP_SRK0_OFFSET               0x0580  /* Shadow Register for OTP Bank3 Word0 (SRK Hash) */
#define IMXRT_OCOTP_SRK1_OFFSET               0x0590  /* Shadow Register for OTP Bank3 Word1 (SRK Hash) */
#define IMXRT_OCOTP_SRK2_OFFSET               0x05a0  /* Shadow Register for OTP Bank3 Word2 (SRK Hash) */
#define IMXRT_OCOTP_SRK3_OFFSET               0x05b0  /* Shadow Register for OTP Bank3 Word3 (SRK Hash) */
#define IMXRT_OCOTP_SRK4_OFFSET               0x05c0  /* Shadow Register for OTP Bank3 Word4 (SRK Hash) */
#define IMXRT_OCOTP_SRK5_OFFSET               0x05d0  /* Shadow Register for OTP Bank3 Word5 (SRK Hash) */
#define IMXRT_OCOTP_SRK6_OFFSET               0x05e0  /* Shadow Register for OTP Bank3 Word6 (SRK Hash) */
#define IMXRT_OCOTP_SRK7_OFFSET               0x05f0  /* Shadow Register for OTP Bank3 Word7 (SRK Hash) */
#define IMXRT_OCOTP_SJC_RESP0_OFFSET          0x0600  /* Value of OTP Bank4 Word0 (Secure JTAG Response Field) */
#define IMXRT_OCOTP_SJC_RESP1_OFFSET          0x0610  /* Value of OTP Bank4 Word1 (Secure JTAG Response Field) */
#define IMXRT_OCOTP_MAC0_OFFSET               0x0620  /* Value of OTP Bank4 Word2 (MAC Address) */
#define IMXRT_OCOTP_MAC1_OFFSET               0x0630  /* Value of OTP Bank4 Word3 (MAC Address) */
#define IMXRT_OCOTP_MAC2_OFFSET               0x0640  /* Value of OTP Bank4 Word4 (MAC Address) */
#define IMXRT_OCOTP_OTPMK_CRC32_OFFSET        0x0650  /* Value of OTP Bank4 Word5 (CRC Key) */
#define IMXRT_OCOTP_SW_GP1_OFFSET             0x0680  /* Value of OTP Bank5 Word0 (SW GP1) */
#define IMXRT_OCOTP_SW_GP20_OFFSET            0x0690  /* Value of OTP Bank5 Word1 (SW GP2) */
#define IMXRT_OCOTP_SW_GP21_OFFSET            0x06a0  /* Value of OTP Bank5 Word2 (SW GP2) */
#define IMXRT_OCOTP_SW_GP22_OFFSET            0x06b0  /* Value of OTP Bank5 Word3 (SW GP2) */
#define IMXRT_OCOTP_SW_GP23_OFFSET            0x06c0  /* Value of OTP Bank5 Word4 (SW GP2) */
#define IMXRT_OCOTP_MISC_CONF0_OFFSET         0x06d0  /* Value of OTP Bank5 Word5 (Misc Conf) */
#define IMXRT_OCOTP_MISC_CONF1_OFFSET         0x06e0  /* Value of OTP Bank5 Word6 (Misc Conf) */
#define IMXRT_OCOTP_SRK_REVOKE_OFFSET         0x06f0  /* Value of OTP Bank5 Word7 (SRK Revoke) */

#define IMXRT_OCOTP_ROM_PATCH0_OFFSET         0x0800  /* Value of OTP Bank6 Word0 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH1_OFFSET         0x0810  /* Value of OTP Bank6 Word1 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH2_OFFSET         0x0820  /* Value of OTP Bank6 Word2 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH3_OFFSET         0x0830  /* Value of OTP Bank6 Word3 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH4_OFFSET         0x0840  /* Value of OTP Bank6 Word4 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH5_OFFSET         0x0850  /* Value of OTP Bank6 Word5 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH6_OFFSET         0x0860  /* Value of OTP Bank6 Word6 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH7_OFFSET         0x0870  /* Value of OTP Bank6 Word7 (ROM Patch) */
#define IMXRT_OCOTP_GP30_OFFSET               0x0880  /* Value of OTP Bank7 Word0 (GP3) */
#define IMXRT_OCOTP_GP31_OFFSET               0x0890  /* Value of OTP Bank7 Word1 (GP3) */
#define IMXRT_OCOTP_GP32_OFFSET               0x08a0  /* Value of OTP Bank7 Word2 (GP3) */
#define IMXRT_OCOTP_GP33_OFFSET               0x08b0  /* Value of OTP Bank7 Word3 (GP3) */
#define IMXRT_OCOTP_GP40_OFFSET               0x08c0  /* Value of OTP Bank7 Word4 (GP4) */
#define IMXRT_OCOTP_GP41_OFFSET               0x08d0  /* Value of OTP Bank7 Word5 (GP4) */
#define IMXRT_OCOTP_GP42_OFFSET               0x08e0  /* Value of OTP Bank7 Word6 (GP4) */
#define IMXRT_OCOTP_GP43_OFFSET               0x08f0  /* Value of OTP Bank7 Word7 (GP4) */

/* OCOTP Indexes *****************************************************************/

#define IMXRT_OCOTP_O2I(offset)               (((offset) - IMXRT_OCOTP_LOCK_OFFSET) >> 4)

#define IMXRT_OCOTP_LOCK_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_LOCK_OFFSET)         /* Value of OTP Bank0 Word0 (Lock controls) */
#define IMXRT_OCOTP_CFG0_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_CFG0_OFFSET)         /* Value of OTP Bank0 Word1 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG1_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_CFG1_OFFSET)         /* Value of OTP Bank0 Word2 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG2_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_CFG2_OFFSET)         /* Value of OTP Bank0 Word3 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG3_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_CFG3_OFFSET)         /* Value of OTP Bank0 Word4 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG4_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_CFG4_OFFSET)         /* Value of OTP Bank0 Word5 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG5_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_CFG5_OFFSET)         /* Value of OTP Bank0 Word6 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG6_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_CFG6_OFFSET)         /* Value of OTP Bank0 Word7 (Configuration and Manufacturing */
#define IMXRT_OCOTP_MEM0_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_MEM0_OFFSET)         /* Value of OTP Bank1 Word0 (Memory Related Info.) */
#define IMXRT_OCOTP_MEM1_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_MEM1_OFFSET)         /* Value of OTP Bank1 Word1 (Memory Related Info.) */
#define IMXRT_OCOTP_MEM2_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_MEM2_OFFSET)         /* Value of OTP Bank1 Word2 (Memory Related Info.) */
#define IMXRT_OCOTP_MEM3_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_MEM3_OFFSET)         /* Value of OTP Bank1 Word3 (Memory Related Info.) */
#define IMXRT_OCOTP_MEM4_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_MEM4_OFFSET)         /* Value of OTP Bank1 Word4 (Memory Related Info.) */
#define IMXRT_OCOTP_ANA0_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_ANA0_OFFSET)         /* Value of OTP Bank1 Word5 (Analog Info.) */
#define IMXRT_OCOTP_ANA1_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_ANA1_OFFSET)         /* Value of OTP Bank1 Word6 (Analog Info.) */
#define IMXRT_OCOTP_ANA2_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_ANA2_OFFSET)         /* Value of OTP Bank1 Word7 (Analog Info.) */
#define IMXRT_OCOTP_OTPMK0_INDEX              IMXRT_OCOTP_O2I(IMXRT_OCOTP_OTPMK0_OFFSET)       /* Value of OTP Bank2 Word0 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK1_INDEX              IMXRT_OCOTP_O2I(IMXRT_OCOTP_OTPMK1_OFFSET)       /* Value of OTP Bank2 Word1 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK2_INDEX              IMXRT_OCOTP_O2I(IMXRT_OCOTP_OTPMK2_OFFSET)       /* Value of OTP Bank2 Word2 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK3_INDEX              IMXRT_OCOTP_O2I(IMXRT_OCOTP_OTPMK3_OFFSET)       /* Value of OTP Bank2 Word3 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK4_INDEX              IMXRT_OCOTP_O2I(IMXRT_OCOTP_OTPMK4_OFFSET)       /* Value of OTP Bank2 Word4 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK5_INDEX              IMXRT_OCOTP_O2I(IMXRT_OCOTP_OTPMK5_OFFSET)       /* Value of OTP Bank2 Word5 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK6_INDEX              IMXRT_OCOTP_O2I(IMXRT_OCOTP_OTPMK6_OFFSET)       /* Value of OTP Bank2 Word6 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK7_INDEX              IMXRT_OCOTP_O2I(IMXRT_OCOTP_OTPMK7_OFFSET)       /* Value of OTP Bank2 Word7 (OTPMK Key) */
#define IMXRT_OCOTP_SRK0_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_SRK0_OFFSET)         /* Shadow Register for OTP Bank3 Word0 (SRK Hash) */
#define IMXRT_OCOTP_SRK1_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_SRK1_OFFSET)         /* Shadow Register for OTP Bank3 Word1 (SRK Hash) */
#define IMXRT_OCOTP_SRK2_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_SRK2_OFFSET)         /* Shadow Register for OTP Bank3 Word2 (SRK Hash) */
#define IMXRT_OCOTP_SRK3_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_SRK3_OFFSET)         /* Shadow Register for OTP Bank3 Word3 (SRK Hash) */
#define IMXRT_OCOTP_SRK4_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_SRK4_OFFSET)         /* Shadow Register for OTP Bank3 Word4 (SRK Hash) */
#define IMXRT_OCOTP_SRK5_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_SRK5_OFFSET)         /* Shadow Register for OTP Bank3 Word5 (SRK Hash) */
#define IMXRT_OCOTP_SRK6_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_SRK6_OFFSET)         /* Shadow Register for OTP Bank3 Word6 (SRK Hash) */
#define IMXRT_OCOTP_SRK7_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_SRK7_OFFSET)         /* Shadow Register for OTP Bank3 Word7 (SRK Hash) */
#define IMXRT_OCOTP_SJC_RESP0_INDEX           IMXRT_OCOTP_O2I(IMXRT_OCOTP_SJC_RESP0_OFFSET)    /* Value of OTP Bank4 Word0 (Secure JTAG Response Field) */
#define IMXRT_OCOTP_SJC_RESP1_INDEX           IMXRT_OCOTP_O2I(IMXRT_OCOTP_SJC_RESP1_OFFSET)    /* Value of OTP Bank4 Word1 (Secure JTAG Response Field) */
#define IMXRT_OCOTP_MAC0_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_MAC0_OFFSET)         /* Value of OTP Bank4 Word2 (MAC Address) */
#define IMXRT_OCOTP_MAC1_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_MAC1_OFFSET)         /* Value of OTP Bank4 Word3 (MAC Address) */
#define IMXRT_OCOTP_MAC2_INDEX                IMXRT_OCOTP_O2I(IMXRT_OCOTP_MAC2_OFFSET)         /* Value of OTP Bank4 Word4 (MAC Address) */
#define IMXRT_OCOTP_OTPMK_CRC32_INDEX         IMXRT_OCOTP_O2I(IMXRT_OCOTP_OTPMK_CRC32_OFFSET)  /* Value of OTP Bank4 Word5 (CRC Key) */
#define IMXRT_OCOTP_SW_GP1_INDEX              IMXRT_OCOTP_O2I(IMXRT_OCOTP_SW_GP1_OFFSET)       /* Value of OTP Bank5 Word0 (SW GP1) */
#define IMXRT_OCOTP_SW_GP20_INDEX             IMXRT_OCOTP_O2I(IMXRT_OCOTP_SW_GP20_OFFSET)      /* Value of OTP Bank5 Word1 (SW GP2) */
#define IMXRT_OCOTP_SW_GP21_INDEX             IMXRT_OCOTP_O2I(IMXRT_OCOTP_SW_GP21_OFFSET)      /* Value of OTP Bank5 Word2 (SW GP2) */
#define IMXRT_OCOTP_SW_GP22_INDEX             IMXRT_OCOTP_O2I(IMXRT_OCOTP_SW_GP22_OFFSET)      /* Value of OTP Bank5 Word3 (SW GP2) */
#define IMXRT_OCOTP_SW_GP23_INDEX             IMXRT_OCOTP_O2I(IMXRT_OCOTP_SW_GP23_OFFSET)      /* Value of OTP Bank5 Word4 (SW GP2) */
#define IMXRT_OCOTP_MISC_CONF0_INDEX          IMXRT_OCOTP_O2I(IMXRT_OCOTP_MISC_CONF0_OFFSET)   /* Value of OTP Bank5 Word5 (Misc Conf) */
#define IMXRT_OCOTP_MISC_CONF1_INDEX          IMXRT_OCOTP_O2I(IMXRT_OCOTP_MISC_CONF1_OFFSET)   /* Value of OTP Bank5 Word6 (Misc Conf) */
#define IMXRT_OCOTP_SRK_REVOKE_INDEX          IMXRT_OCOTP_O2I(IMXRT_OCOTP_SRK_REVOKE_OFFSET)   /* Value of OTP Bank5 Word7 (SRK Revoke) */

#define IMXRT_OCOTP_O2IP(offset)              (((offset) - (IMXRT_OCOTP_LOCK_OFFSET + 0x100)) >> 4)

#define IMXRT_OCOTP_ROM_PATCH0_INDEX          IMXRT_OCOTP_O2IP(IMXRT_OCOTP_ROM_PATCH0_OFFSET)   /* Value of OTP Bank6 Word0 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH1_INDEX          IMXRT_OCOTP_O2IP(IMXRT_OCOTP_ROM_PATCH1_OFFSET)   /* Value of OTP Bank6 Word1 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH2_INDEX          IMXRT_OCOTP_O2IP(IMXRT_OCOTP_ROM_PATCH2_OFFSET)   /* Value of OTP Bank6 Word2 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH3_INDEX          IMXRT_OCOTP_O2IP(IMXRT_OCOTP_ROM_PATCH3_OFFSET)   /* Value of OTP Bank6 Word3 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH4_INDEX          IMXRT_OCOTP_O2IP(IMXRT_OCOTP_ROM_PATCH4_OFFSET)   /* Value of OTP Bank6 Word4 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH5_INDEX          IMXRT_OCOTP_O2IP(IMXRT_OCOTP_ROM_PATCH5_OFFSET)   /* Value of OTP Bank6 Word5 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH6_INDEX          IMXRT_OCOTP_O2IP(IMXRT_OCOTP_ROM_PATCH6_OFFSET)   /* Value of OTP Bank6 Word6 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH7_INDEX          IMXRT_OCOTP_O2IP(IMXRT_OCOTP_ROM_PATCH7_OFFSET)   /* Value of OTP Bank6 Word7 (ROM Patch) */
#define IMXRT_OCOTP_GP30_INDEX                IMXRT_OCOTP_O2IP(IMXRT_OCOTP_GP30_OFFSET)         /* Value of OTP Bank7 Word0 (GP3) */
#define IMXRT_OCOTP_GP31_INDEX                IMXRT_OCOTP_O2IP(IMXRT_OCOTP_GP31_OFFSET)         /* Value of OTP Bank7 Word1 (GP3) */
#define IMXRT_OCOTP_GP32_INDEX                IMXRT_OCOTP_O2IP(IMXRT_OCOTP_GP32_OFFSET)         /* Value of OTP Bank7 Word2 (GP3) */
#define IMXRT_OCOTP_GP33_INDEX                IMXRT_OCOTP_O2IP(IMXRT_OCOTP_GP33_OFFSET)         /* Value of OTP Bank7 Word3 (GP3) */
#define IMXRT_OCOTP_GP40_INDEX                IMXRT_OCOTP_O2IP(IMXRT_OCOTP_GP40_OFFSET)         /* Value of OTP Bank7 Word4 (GP4) */
#define IMXRT_OCOTP_GP41_INDEX                IMXRT_OCOTP_O2IP(IMXRT_OCOTP_GP41_OFFSET)         /* Value of OTP Bank7 Word5 (GP4) */
#define IMXRT_OCOTP_GP42_INDEX                IMXRT_OCOTP_O2IP(IMXRT_OCOTP_GP42_OFFSET)         /* Value of OTP Bank7 Word6 (GP4) */
#define IMXRT_OCOTP_GP43_INDEX                IMXRT_OCOTP_O2IP(IMXRT_OCOTP_GP43_OFFSET)         /* Value of OTP Bank7 Word7 (GP4) */

/* Register addresses ***********************************************************************************************************************/

#define IMXRT_OCOTP_CTRL                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CTRL_OFFSET)            /* OTP Controller Control Register */
#define IMXRT_OCOTP_CTRL_SET                  (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CTRL_SET_OFFSET)        /* OTP Controller Control Register */
#define IMXRT_OCOTP_CTRL_CLR                  (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CTRL_CLR_OFFSET)        /* OTP Controller Control Register */
#define IMXRT_OCOTP_CTRL_TOG                  (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CTRL_TOG_OFFSET)        /* OTP Controller Control Register */
#define IMXRT_OCOTP_TIMING                    (IMXRT_OCOTP_BASE + IMXRT_OCOTP_TIMING_OFFSET)          /* OTP Controller Timing Register */
#define IMXRT_OCOTP_DATA                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_DATA_OFFSET)            /* OTP Controller Write Data Register */
#define IMXRT_OCOTP_READ_CTRL                 (IMXRT_OCOTP_BASE + IMXRT_OCOTP_READ_CTRL_OFFSET)       /* OTP Controller Write Data Register */
#define IMXRT_OCOTP_READ_FUSE_DATA            (IMXRT_OCOTP_BASE + IMXRT_OCOTP_READ_FUSE_DATA_OFFSET)  /* OTP Controller Read Data Register */
#define IMXRT_OCOTP_SW_STICKY                 (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SW_STICKY_OFFSET)       /* Sticky bit Register */
#define IMXRT_OCOTP_SCS                       (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SCS_OFFSET)             /* Software Controllable Signals Register */
#define IMXRT_OCOTP_SCS_SET                   (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SCS_SET_OFFSET)         /* Software Controllable Signals Register */
#define IMXRT_OCOTP_SCS_CLR                   (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SCS_CLR_OFFSET)         /* Software Controllable Signals Register */
#define IMXRT_OCOTP_SCS_TOG                   (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SCS_TOG_OFFSET)         /* Software Controllable Signals Register */
#define IMXRT_OCOTP_CRC_ADDR                  (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CRC_ADDR_OFFSET)        /* OTP Controller CRC test address */
#define IMXRT_OCOTP_CRC_VALUE                 (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CRC_VALUE_OFFSET)       /* OTP Controller CRC Value Register */
#define IMXRT_OCOTP_VERSION                   (IMXRT_OCOTP_BASE + IMXRT_OCOTP_VERSION_OFFSET)         /* OTP Controller Version Register */
#define IMXRT_OCOTP_TIMING2                   (IMXRT_OCOTP_BASE + IMXRT_OCOTP_TIMING2_OFFSET)         /* OTP Controller Timing Register */
#define IMXRT_OCOTP_LOCK                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_LOCK_OFFSET)            /* Value of OTP Bank0 Word0 (Lock controls) */
#define IMXRT_OCOTP_CFG0                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CFG0_OFFSET)            /* Value of OTP Bank0 Word1 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG1                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CFG1_OFFSET)            /* Value of OTP Bank0 Word2 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG2                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CFG2_OFFSET)            /* Value of OTP Bank0 Word3 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG3                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CFG3_OFFSET)            /* Value of OTP Bank0 Word4 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG4                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CFG4_OFFSET)            /* Value of OTP Bank0 Word5 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG5                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CFG5_OFFSET)            /* Value of OTP Bank0 Word6 (Configuration and Manufacturing */
#define IMXRT_OCOTP_CFG6                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CFG6_OFFSET)            /* Value of OTP Bank0 Word7 (Configuration and Manufacturing */
#define IMXRT_OCOTP_MEM0                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_MEM0_OFFSET)            /* Value of OTP Bank1 Word0 (Memory Related Info.) */
#define IMXRT_OCOTP_MEM1                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_MEM1_OFFSET)            /* Value of OTP Bank1 Word1 (Memory Related Info.) */
#define IMXRT_OCOTP_MEM2                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_MEM2_OFFSET)            /* Value of OTP Bank1 Word2 (Memory Related Info.) */
#define IMXRT_OCOTP_MEM3                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_MEM3_OFFSET)            /* Value of OTP Bank1 Word3 (Memory Related Info.) */
#define IMXRT_OCOTP_MEM4                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_MEM4_OFFSET)            /* Value of OTP Bank1 Word4 (Memory Related Info.) */
#define IMXRT_OCOTP_ANA0                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_ANA0_OFFSET)            /* Value of OTP Bank1 Word5 (Analog Info.) */
#define IMXRT_OCOTP_ANA1                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_ANA1_OFFSET)            /* Value of OTP Bank1 Word6 (Analog Info.) */
#define IMXRT_OCOTP_ANA2                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_ANA2_OFFSET)            /* Value of OTP Bank1 Word7 (Analog Info.) */
#define IMXRT_OCOTP_OTPMK0                    (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OTPMK0_OFFSET)          /* Value of OTP Bank2 Word0 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK1                    (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OTPMK1_OFFSET)          /* Value of OTP Bank2 Word1 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK2                    (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OTPMK2_OFFSET)          /* Value of OTP Bank2 Word2 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK3                    (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OTPMK3_OFFSET)          /* Value of OTP Bank2 Word3 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK4                    (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OTPMK4_OFFSET)          /* Value of OTP Bank2 Word4 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK5                    (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OTPMK5_OFFSET)          /* Value of OTP Bank2 Word5 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK6                    (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OTPMK6_OFFSET)          /* Value of OTP Bank2 Word6 (OTPMK Key) */
#define IMXRT_OCOTP_OTPMK7                    (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OTPMK7_OFFSET)          /* Value of OTP Bank2 Word7 (OTPMK Key) */
#define IMXRT_OCOTP_SRK0                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SRK0_OFFSET)            /* Shadow Register for OTP Bank3 Word0 (SRK Hash) */
#define IMXRT_OCOTP_SRK1                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SRK1_OFFSET)            /* Shadow Register for OTP Bank3 Word1 (SRK Hash) */
#define IMXRT_OCOTP_SRK2                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SRK2_OFFSET)            /* Shadow Register for OTP Bank3 Word2 (SRK Hash) */
#define IMXRT_OCOTP_SRK3                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SRK3_OFFSET)            /* Shadow Register for OTP Bank3 Word3 (SRK Hash) */
#define IMXRT_OCOTP_SRK4                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SRK4_OFFSET)            /* Shadow Register for OTP Bank3 Word4 (SRK Hash) */
#define IMXRT_OCOTP_SRK5                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SRK5_OFFSET)            /* Shadow Register for OTP Bank3 Word5 (SRK Hash) */
#define IMXRT_OCOTP_SRK6                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SRK6_OFFSET)            /* Shadow Register for OTP Bank3 Word6 (SRK Hash) */
#define IMXRT_OCOTP_SRK7                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SRK7_OFFSET)            /* Shadow Register for OTP Bank3 Word7 (SRK Hash) */
#define IMXRT_OCOTP_SJC_RESP0                 (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SJC_RESP0_OFFSET)       /* Value of OTP Bank4 Word0 (Secure JTAG Response Field) */
#define IMXRT_OCOTP_SJC_RESP1                 (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SJC_RESP1_OFFSET)       /* Value of OTP Bank4 Word1 (Secure JTAG Response Field) */
#define IMXRT_OCOTP_MAC0                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_MAC0_OFFSET)            /* Value of OTP Bank4 Word2 (MAC Address) */
#define IMXRT_OCOTP_MAC1                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_MAC1_OFFSET)            /* Value of OTP Bank4 Word3 (MAC Address) */
#define IMXRT_OCOTP_MAC2                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_MAC2_OFFSET)            /* Value of OTP Bank4 Word4 (MAC Address) */
#define IMXRT_OCOTP_OTPMK_CRC32               (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OTPMK_CRC32_OFFSET)     /* Value of OTP Bank4 Word5 (CRC Key) */
#define IMXRT_OCOTP_SW_GP1                    (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SW_GP1_OFFSET)          /* Value of OTP Bank5 Word0 (SW GP1) */
#define IMXRT_OCOTP_SW_GP20                   (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SW_GP20_OFFSET)         /* Value of OTP Bank5 Word1 (SW GP2) */
#define IMXRT_OCOTP_SW_GP21                   (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SW_GP21_OFFSET)         /* Value of OTP Bank5 Word2 (SW GP2) */
#define IMXRT_OCOTP_SW_GP22                   (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SW_GP22_OFFSET)         /* Value of OTP Bank5 Word3 (SW GP2) */
#define IMXRT_OCOTP_SW_GP23                   (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SW_GP23_OFFSET)         /* Value of OTP Bank5 Word4 (SW GP2) */
#define IMXRT_OCOTP_MISC_CONF0                (IMXRT_OCOTP_BASE + IMXRT_OCOTP_MISC_CONF0_OFFSET)      /* Value of OTP Bank5 Word5 (Misc Conf) */
#define IMXRT_OCOTP_MISC_CONF1                (IMXRT_OCOTP_BASE + IMXRT_OCOTP_MISC_CONF1_OFFSET)      /* Value of OTP Bank5 Word6 (Misc Conf) */
#define IMXRT_OCOTP_SRK_REVOKE                (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SRK_REVOKE_OFFSET)      /* Value of OTP Bank5 Word7 (SRK Revoke) */
#define IMXRT_OCOTP_ROM_PATCH0                (IMXRT_OCOTP_BASE + IMXRT_OCOTP_ROM_PATCH0_OFFSET)      /* Value of OTP Bank6 Word0 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH1                (IMXRT_OCOTP_BASE + IMXRT_OCOTP_ROM_PATCH1_OFFSET)      /* Value of OTP Bank6 Word1 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH2                (IMXRT_OCOTP_BASE + IMXRT_OCOTP_ROM_PATCH2_OFFSET)      /* Value of OTP Bank6 Word2 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH3                (IMXRT_OCOTP_BASE + IMXRT_OCOTP_ROM_PATCH3_OFFSET)      /* Value of OTP Bank6 Word3 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH4                (IMXRT_OCOTP_BASE + IMXRT_OCOTP_ROM_PATCH4_OFFSET)      /* Value of OTP Bank6 Word4 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH5                (IMXRT_OCOTP_BASE + IMXRT_OCOTP_ROM_PATCH5_OFFSET)      /* Value of OTP Bank6 Word5 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH6                (IMXRT_OCOTP_BASE + IMXRT_OCOTP_ROM_PATCH6_OFFSET)      /* Value of OTP Bank6 Word6 (ROM Patch) */
#define IMXRT_OCOTP_ROM_PATCH7                (IMXRT_OCOTP_BASE + IMXRT_OCOTP_ROM_PATCH7_OFFSET)      /* Value of OTP Bank6 Word7 (ROM Patch) */
#define IMXRT_OCOTP_GP30                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_GP30_OFFSET)            /* Value of OTP Bank7 Word0 (GP3) */
#define IMXRT_OCOTP_GP31                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_GP31_OFFSET)            /* Value of OTP Bank7 Word1 (GP3) */
#define IMXRT_OCOTP_GP32                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_GP32_OFFSET)            /* Value of OTP Bank7 Word2 (GP3) */
#define IMXRT_OCOTP_GP33                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_GP33_OFFSET)            /* Value of OTP Bank7 Word3 (GP3) */
#define IMXRT_OCOTP_GP40                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_GP40_OFFSET)            /* Value of OTP Bank7 Word4 (GP4) */
#define IMXRT_OCOTP_GP41                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_GP41_OFFSET)            /* Value of OTP Bank7 Word5 (GP4) */
#define IMXRT_OCOTP_GP42                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_GP42_OFFSET)            /* Value of OTP Bank7 Word6 (GP4) */
#define IMXRT_OCOTP_GP43                      (IMXRT_OCOTP_BASE + IMXRT_OCOTP_GP43_OFFSET)            /* Value of OTP Bank7 Word7 (GP4) */

/* Register Bit Definitions *********************************************************************************************************/

/* OTP Controller Control Register */

#define OCOTP_CTRL_ADDR_SHIFT                 (0)        /* Bits: 0-5  ADDR */
#define OCOTP_CTRL_ADDR_MASK                  (0x3f << OCOTP_CTRL_ADDR_SHIFT)
#  define OCOTP_CTRL_ADDR(n)                  ((uint32_t)(n) << OCOTP_CTRL_ADDR_SHIFT)
                                                         /* Bits: 6-7  Reserved */
#define OCOTP_CTRL_BUSY                       (1 << 8)   /* Bit: 8  BUSY */
#define OCOTP_CTRL_ERROR                      (1 << 9)   /* Bit: 9  ERROR */
#define OCOTP_CTRL_RELOAD_SHADOWS             (1 << 10)  /* Bit: 10 OWS */
#define OCOTP_CTRL_CRC_TEST                   (1 << 11)  /* Bit: 11 CRC_TEST */
#define OCOTP_CTRL_CRC_FAIL                   (1 << 12)  /* Bit: 12 CRC_FAIL */
                                                         /* Bits: 13-15 Reserved */
#define OCOTP_CTRL_WR_UNLOCK_SHIFT            (16)       /* Bits: 16-31  WR_UNLOCK */
#define OCOTP_CTRL_WR_UNLOCK_MASK             (0xffff << OCOTP_CTRL_WR_UNLOCK_SHIFT)
#  define OCOTP_CTRL_WR_UNLOCK                (0x3e77 << OCOTP_CTRL_WR_UNLOCK_SHIFT)

/* OTP Controller Timing Register */

#define OCOTP_TIMING_STROBE_PROG_SHIFT        (0)        /* Bits: 0-11  Specifies the strobe period in one time write OTP */
#define OCOTP_TIMING_STROBE_PROG_MASK         (0xfff << OCOTP_TIMING_STROBE_PROG_SHIFT)
#  define OCOTP_TIMING_STROBE_PROG(n)         ((uint32_t)(n) << OCOTP_TIMING_STROBE_PROG_SHIFT)
#define OCOTP_TIMING_RELAX_SHIFT              (12)       /* Bits: 12-15  Specifies the time to add to all default timing parameters other than the Tpgm and Trd. */
#define OCOTP_TIMING_RELAX_MASK               (15 << OCOTP_TIMING_RELAX_SHIFT)
#  define OCOTP_TIMING_RELAX(n)               ((uint32_t)(n) << OCOTP_TIMING_RELAX_SHIFT)
#define OCOTP_TIMING_STROBE_READ_SHIFT        (16)       /* Bits: 16-21  Specifies the strobe period in one time read OTP. */
#define OCOTP_TIMING_STROBE_READ_MASK         (0x3f << OCOTP_TIMING_STROBE_READ_SHIFT)
#  define OCOTP_TIMING_STROBE_READ(n)         ((uint32_t)(n) << OCOTP_TIMING_STROBE_READ_SHIFT)
#define OCOTP_TIMING_WAIT_SHIFT               (22)       /* Bits: 22-27  Specifies time interval between auto read and write access in one time program */
#define OCOTP_TIMING_WAIT_MASK                (0x3f << OCOTP_TIMING_WAIT_SHIFT)
#  define OCOTP_TIMING_WAIT(n)                ((uint32_t)(n) << OCOTP_TIMING_WAIT_SHIFT)
                                                         /* Bits: 28-31  Reserved */

/* OTP Controller Write Data Register */

#define OCOTP_READ_CTRL_READ_FUSE             (1 << 0)   /* Bit: 0 Used to initiate a read to OTP */
                                                         /* Bits: 1-31 Reserved */

/* Sticky bit Register */

#define OCOTP_SW_STICKY_BLOCK_DTCP_KEY        (1 << 0)   /* Bit: 0  Shadow register read and OTP read lock for DTCP_KEY region. */
#define OCOTP_SW_STICKY_SRK_REVOKE_LOCK       (1 << 1)   /* Bit: 1  Shadow register write and OTP write lock for SRK_REVOKE region. */
#define OCOTP_SW_STICKY_FIELD_RETURN_LOCK     (1 << 2)   /* Bit: 2  Shadow register write and OTP write lock for FIELD_RETURN region. */
#define OCOTP_SW_STICKY_BLOCK_ROM_PART        (1 << 3)   /* Bit: 3  Set by ARM during Boot after DTCP is initialized and before test mode entry, if ROM_PART_LOCK=1.*/
#define OCOTP_SW_STICKY_JTAG_BLOCK_RELEASE    (1 << 4)   /* Bit: 4  Set by ARM during Boot after DTCP is initialized and before test mode entry. */
                                                         /* Bits: 5-31  Reserved */

/* Software Controllable Signals Register */

#define OCOTP_SCS_HAB_JDE                     (1 << 0)   /* Bit: 0  HAB JTAG Debug Enable. */
#define OCOTP_SCS_SPARE_SHIFT                 (1)        /* Bits: 1-30  Unallocated read/write bits for implementation specific software use. */
#define OCOTP_SCS_SPARE_MASK                  (0x3fffffff << OCOTP_SCS_SPARE_SHIFT)
#  define OCOTP_SCS_SPARE(n)                  ((uint32_t)(n) << OCOTP_SCS_SPARE_SHIFT)
#define OCOTP_SCS_LOCK                        (1 << 31)  /* Bit: 31 LOCK */

/* OTP Controller CRC test address */

#define OCOTP_CRC_ADDR_DATA_START_ADDR_SHIFT  (0)        /* Bits: 0-7  Start address of fuse location for CRC calculation */
#define OCOTP_CRC_ADDR_DATA_START_ADDR_MASK   (0xff << OCOTP_CRC_ADDR_DATA_START_ADDR_SHIFT)
#  define OCOTP_CRC_ADDR_DATA_START_ADDR(n)   ((uint32_t)(n) << OCOTP_CRC_ADDR_DATA_START_ADDR_SHIFT)
#define OCOTP_CRC_ADDR_DATA_END_ADDR_SHIFT    (8)        /* Bits: 8-15  End address of fuse location for CRC calculation */
#define OCOTP_CRC_ADDR_DATA_END_ADDR_MASK     (0xff << OCOTP_CRC_ADDR_DATA_END_ADDR_SHIFT)
#  define OCOTP_CRC_ADDR_DATA_END_ADDR(n)     ((uint32_t)(n) << OCOTP_CRC_ADDR_DATA_END_ADDR_SHIFT)
#define OCOTP_CRC_ADDR_CRC_ADDR_SHIFT         (16)       /* Bits: 16-23  Address of 32-bit CRC result for comparing */
#define OCOTP_CRC_ADDR_CRC_ADDR_MASK          (0xff << OCOTP_CRC_ADDR_CRC_ADDR_SHIFT)
#  define OCOTP_CRC_ADDR_CRC_ADDR(n)          ((uint32_t)(n) << OCOTP_CRC_ADDR_CRC_ADDR_SHIFT)
#define OCOTP_CRC_ADDR_OTPMK_CRC              (1 << 24)  /* Bit: 24 Enable bit for OCOTP CRC32 calculation address. */
                                                         /* Bits: 25-31  Reserved */

/* OTP Controller Version Register */

#define OCOTP_VERSION_STEP_SHIFT              (0)        /* Bits: 0-15  STEP field of the RTL version.*/
#define OCOTP_VERSION_STEP_MASK               (0xffff << OCOTP_VERSION_STEP_SHIFT)
#  define OCOTP_VERSION_STEP(n)               ((uint32_t)(n) << OCOTP_VERSION_STEP_SHIFT)
#define OCOTP_VERSION_MINOR_SHIFT             (16)       /* Bits: 16-23  MINOR field of the RTL version. */
#define OCOTP_VERSION_MINOR_MASK              (0xff << OCOTP_VERSION_MINOR_SHIFT)
#  define OCOTP_VERSION_MINOR(n)              ((uint32_t)(n) << OCOTP_VERSION_MINOR_SHIFT)
#define OCOTP_VERSION_MAJOR_SHIFT             (24)       /* Bits: 24-31  MAJOR field of the RTL version. */
#define OCOTP_VERSION_MAJOR_MASK              (0xff << OCOTP_VERSION_MAJOR_SHIFT)
#  define OCOTP_VERSION_MAJOR(n)              ((uint32_t)(n) << OCOTP_VERSION_MAJOR_SHIFT)

/* OTP Controller Timing Register */

#define OCOTP_TIMING2_RELAX_PROG_SHIFT        (0)        /* Bits: 0-11  Specifies the strobe period in one time write OTP. */
#define OCOTP_TIMING2_RELAX_PROG_MASK         (0xfff << OCOTP_TIMING2_RELAX_PROG_SHIFT)
#  define OCOTP_TIMING2_RELAX_PROG(n)         ((uint32_t)(n) << OCOTP_TIMING2_RELAX_PROG_SHIFT)
                                                         /* Bits: 12-15  Reserved. These bits always read back zero. */
#define OCOTP_TIMING2_RELAX_READ_SHIFT        (16)       /* Bits: 16-21  Specifies the strobe period in one time read OTP. */
#define OCOTP_TIMING2_RELAX_READ_MASK         (0x3f << OCOTP_TIMING2_RELAX_READ_SHIFT)
#  define OCOTP_TIMING2_RELAX_READ(n)         ((uint32_t)(n) << OCOTP_TIMING2_RELAX_READ_SHIFT)
                                                         /* Bits: 22-31  Reserved. These bits always read back zero. */

/* Value of OTP Bank0 Word0 (Lock controls) */

#define OCOTP_LOCK_TESTER_SHIFT               (0)        /* Bits: 0-1  Chapter 22 On-Chip OTP Controller (OCOTP_CTRL) */
#define OCOTP_LOCK_TESTER_MASK                (3 << OCOTP_LOCK_TESTER_SHIFT)
#  define OCOTP_LOCK_TESTER(n)                ((uint32_t)(n) << OCOTP_LOCK_TESTER_SHIFT)
#define OCOTP_LOCK_BOOT_CFG_SHIFT             (2)        /* Bits: 2-3  Status of shadow register and OTP write lock for boot_cfg region. */
#define OCOTP_LOCK_BOOT_CFG_MASK              (3 << OCOTP_LOCK_BOOT_CFG_SHIFT)
#  define OCOTP_LOCK_BOOT_CFG(n)              ((uint32_t)(n) << OCOTP_LOCK_BOOT_CFG_SHIFT)
#define OCOTP_LOCK_MEM_TRIM_SHIFT             (4)        /* Bits: 4-5  Status of shadow register and OTP write lock for mem_trim region.  */
#define OCOTP_LOCK_MEM_TRIM_MASK              (3 << OCOTP_LOCK_MEM_TRIM_SHIFT)
#  define OCOTP_LOCK_MEM_TRIM(n)              ((uint32_t)(n) << OCOTP_LOCK_MEM_TRIM_SHIFT)
#define OCOTP_LOCK_SJC_RESP                   (1 << 6)   /* Bit: 6  Status of shadow register read and write, OTP read and write lock for sjc_resp region. */
#define OCOTP_LOCK_GP4_RLOCK                  (1 << 7)   /* Bit: 7  Status of shadow register and OTP read lock for gp4 region. */
#define OCOTP_LOCK_MAC_ADDR_SHIFT             (8)        /* Bits: 8-9  Status of shadow register and OTP write lock for mac_addr region.  */
#define OCOTP_LOCK_MAC_ADDR_MASK              (3 << OCOTP_LOCK_MAC_ADDR_SHIFT)
#  define OCOTP_LOCK_MAC_ADDR(n)              ((uint32_t)(n) << OCOTP_LOCK_MAC_ADDR_SHIFT)
#define OCOTP_LOCK_GP1_SHIFT                  (10)       /* Bits: 10-11  Status of shadow register and OTP write lock for gp2 region. */
#define OCOTP_LOCK_GP1_MASK                   (3 << OCOTP_LOCK_GP1_SHIFT)
#  define OCOTP_LOCK_GP1(n)                   ((uint32_t)(n) << OCOTP_LOCK_GP1_SHIFT)
#define OCOTP_LOCK_GP2_SHIFT                  (12)       /* Bits: 12-13  Status of shadow register and OTP write lock for gp2 region.*/
#define OCOTP_LOCK_GP2_MASK                   (3 << OCOTP_LOCK_GP2_SHIFT)
#  define OCOTP_LOCK_GP2(n)                   ((uint32_t)(n) << OCOTP_LOCK_GP2_SHIFT)
#define OCOTP_LOCK_SRK                        (1 << 14)  /* Bit: 14 Status of shadow register and OTP write lock for srk region.  */
#define OCOTP_LOCK_ROM_PATCH                  (1 << 15)  /* Bit: 15 Status of shadow register and OTP write lock for rom_patch region.  */
#define OCOTP_LOCK_SW_GP1                     (1 << 16)  /* Bit: 16 Status of shadow register and OTP write lock for sw_gp1 region.*/
#define OCOTP_LOCK_OTPMK                      (1 << 17)  /* Bit: 17 Status of shadow register and OTP write lock for OTPMK region.  */
#define OCOTP_LOCK_ANALOG_SHIFT               (18)       /* Bits: 18-19  Status of shadow register and OTP write lock for analog region. */
#define OCOTP_LOCK_ANALOG_MASK                (3 << OCOTP_LOCK_ANALOG_SHIFT)
#define OCOTP_LOCK_SW_GP2_LOCK                (1 << 21)  /* Bit: 21 Status of shadow register and OTP write lock for sw_gp2 region.*/
#define OCOTP_LOCK_MISC_CONF                  (1 << 22)  /* Bit: 22 Status of shadow register and OTP write lock for misc_conf region.*/
#define OCOTP_LOCK_SW_GP2_RLOCK               (1 << 23)  /* Bit: 23 Status of shadow register and OTP read lock for sw_gp2 region. */
#define OCOTP_LOCK_GP4_SHIFT                  (24)       /* Bits: 24-25  Status of shadow register and OTP write lock for GP4 region. */
#define OCOTP_LOCK_GP4_MASK                   (3 << OCOTP_LOCK_GP4_SHIFT)
#  define OCOTP_LOCK_GP4(n)                   ((uint32_t)(n) << OCOTP_LOCK_GP4_SHIFT)
#define OCOTP_LOCK_GP3_SHIFT                  (26)       /* Bits: 26-27  Status of shadow register and OTP write lock for GP3 region. */
#define OCOTP_LOCK_GP3_MASK                   (3 << OCOTP_LOCK_GP3_SHIFT)
#  define OCOTP_LOCK_GP3(n)                   ((uint32_t)(n) << OCOTP_LOCK_GP3_SHIFT)
#define OCOTP_LOCK_FIELD_RETURN_SHIFT         (28)       /* Bits: 28-31  When write field_return shadow register(only highest 4bits valid), the bits[27:0] must be kept as 0. */
#define OCOTP_LOCK_FIELD_RETURN_MASK          (15 << OCOTP_LOCK_FIELD_RETURN_SHIFT)
#  define OCOTP_LOCK_FIELD_RETURN(n)          ((uint32_t)(n) << OCOTP_LOCK_FIELD_RETURN_SHIFT)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_OCOTP_H */

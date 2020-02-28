/*******************************************************************************************************************************
 * arch/arm/src/efm32/chip/efm32_aes.h
 *
 *  Copyright 2014 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.@n
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.@n
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Laboratories, Inc.
 * has no obligation to support this Software. Silicon Laboratories, Inc. is
 * providing the Software "AS IS", with no express or implied warranties of any
 * kind, including, but not limited to, any implied warranties of
 * merchantability or fitness for any particular purpose or warranties against
 * infringement of any proprietary rights of a third party.
 *
 * Silicon Laboratories, Inc. will not be liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this Software.
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville . All rights reserved.
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Pierre-noel Bouteville <pnb990@gmail.com>
 *            Gregory Nutt <gnutt@nuttx.org>
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
 *******************************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_EFM32_CHIP_EFM32_AES_H_
#define __ARCH_ARM_SRC_EFM32_CHIP_EFM32_AES_H_

/*******************************************************************************************************************************
 * Included Files
 *******************************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/efm32_memorymap.h"

#if !defined(CONFIG_EFM32_EFM32GG)
#  warning This is the EFM32GG header file; Review/modification needed for this architecture
#endif

/*******************************************************************************************************************************
 * Pre-processor Definitions
 *******************************************************************************************************************************/
/* AES Register Offsets ********************************************************************************************************/

#define EFM32_AES_CTRL_OFFSET           0x0000  /* Control Register */
#define EFM32_AES_CMD_OFFSET            0x0004  /* Command Register */
#define EFM32_AES_STATUS_OFFSET         0x0008  /* Status Register */
#define EFM32_AES_IEN_OFFSET            0x000c  /* Interrupt Enable Register */
#define EFM32_AES_IF_OFFSET             0x0010  /* Interrupt Flag Register */
#define EFM32_AES_IFS_OFFSET            0x0014  /* Interrupt Flag Set Register */
#define EFM32_AES_IFC_OFFSET            0x0018  /* Interrupt Flag Clear Register */
#define EFM32_AES_DATA_OFFSET           0x001c  /* DATA Register */
#define EFM32_AES_XORDATA_OFFSET        0x0020  /* XORDATA Register */
#define EFM32_AES_KEYLA_OFFSET          0x0030  /* KEY Low Register */
#define EFM32_AES_KEYLB_OFFSET          0x0034  /* KEY Low Register */
#define EFM32_AES_KEYLC_OFFSET          0x0038  /* KEY Low Register */
#define EFM32_AES_KEYLD_OFFSET          0x003c  /* KEY Low Register */
#define EFM32_AES_KEYHA_OFFSET          0x0040  /* KEY High Register */
#define EFM32_AES_KEYHB_OFFSET          0x0044  /* KEY High Register */
#define EFM32_AES_KEYHC_OFFSET          0x0048  /* KEY High Register */
#define EFM32_AES_KEYHD_OFFSET          0x004c  /* KEY High Register */

/* AES Register Addresses ******************************************************************************************************/

#define EFM32_AES_CTRL                  (EFM32_AES_BASE+EFM32_AES_CTRL_OFFSET)
#define EFM32_AES_CMD                   (EFM32_AES_BASE+EFM32_AES_CMD_OFFSET)
#define EFM32_AES_STATUS                (EFM32_AES_BASE+EFM32_AES_STATUS_OFFSET)
#define EFM32_AES_IEN                   (EFM32_AES_BASE+EFM32_AES_IEN_OFFSET)
#define EFM32_AES_IF                    (EFM32_AES_BASE+EFM32_AES_IF_OFFSET)
#define EFM32_AES_IFS                   (EFM32_AES_BASE+EFM32_AES_IFS_OFFSET)
#define EFM32_AES_IFC                   (EFM32_AES_BASE+EFM32_AES_IFC_OFFSET)
#define EFM32_AES_DATA                  (EFM32_AES_BASE+EFM32_AES_DATA_OFFSET)
#define EFM32_AES_XORDATA               (EFM32_AES_BASE+EFM32_AES_XORDATA_OFFSET)
#define EFM32_AES_KEYLA                 (EFM32_AES_BASE+EFM32_AES_KEYLA_OFFSET)
#define EFM32_AES_KEYLB                 (EFM32_AES_BASE+EFM32_AES_KEYLB_OFFSET)
#define EFM32_AES_KEYLC                 (EFM32_AES_BASE+EFM32_AES_KEYLC_OFFSET)
#define EFM32_AES_KEYLD                 (EFM32_AES_BASE+EFM32_AES_KEYLD_OFFSET)
#define EFM32_AES_KEYHA                 (EFM32_AES_BASE+EFM32_AES_KEYHA_OFFSET)
#define EFM32_AES_KEYHB                 (EFM32_AES_BASE+EFM32_AES_KEYHB_OFFSET)
#define EFM32_AES_KEYHC                 (EFM32_AES_BASE+EFM32_AES_KEYHC_OFFSET)
#define EFM32_AES_KEYHD                 (EFM32_AES_BASE+EFM32_AES_KEYHD_OFFSET)

/* AES Register Bit Field Definitions ******************************************************************************************/

/* Bit fields for AES CTRL */

#define _AES_CTRL_RESETVALUE            0x00000000UL                       /* Default value for AES_CTRL */
#define _AES_CTRL_MASK                  0x00000077UL                       /* Mask for AES_CTRL */

#define AES_CTRL_DECRYPT                (0x1UL << 0)                       /* Decryption/Encryption Mode */
#define _AES_CTRL_DECRYPT_SHIFT         0                                  /* Shift value for AES_DECRYPT */
#define _AES_CTRL_DECRYPT_MASK          0x1UL                              /* Bit mask for AES_DECRYPT */
#define _AES_CTRL_DECRYPT_DEFAULT       0x00000000UL                       /* Mode DEFAULT for AES_CTRL */
#define AES_CTRL_DECRYPT_DEFAULT        (_AES_CTRL_DECRYPT_DEFAULT << 0)   /* Shifted mode DEFAULT for AES_CTRL */
#define AES_CTRL_AES256                 (0x1UL << 1)                       /* AES-256 Mode */
#define _AES_CTRL_AES256_SHIFT          1                                  /* Shift value for AES_AES256 */
#define _AES_CTRL_AES256_MASK           0x2UL                              /* Bit mask for AES_AES256 */
#define _AES_CTRL_AES256_DEFAULT        0x00000000UL                       /* Mode DEFAULT for AES_CTRL */
#define AES_CTRL_AES256_DEFAULT         (_AES_CTRL_AES256_DEFAULT << 1)    /* Shifted mode DEFAULT for AES_CTRL */
#define AES_CTRL_KEYBUFEN               (0x1UL << 2)                       /* Key Buffer Enable */
#define _AES_CTRL_KEYBUFEN_SHIFT        2                                  /* Shift value for AES_KEYBUFEN */
#define _AES_CTRL_KEYBUFEN_MASK         0x4UL                              /* Bit mask for AES_KEYBUFEN */
#define _AES_CTRL_KEYBUFEN_DEFAULT      0x00000000UL                       /* Mode DEFAULT for AES_CTRL */
#define AES_CTRL_KEYBUFEN_DEFAULT       (_AES_CTRL_KEYBUFEN_DEFAULT << 2)  /* Shifted mode DEFAULT for AES_CTRL */
#define AES_CTRL_DATASTART              (0x1UL << 4)                       /* AES_DATA Write Start */
#define _AES_CTRL_DATASTART_SHIFT       4                                  /* Shift value for AES_DATASTART */
#define _AES_CTRL_DATASTART_MASK        0x10UL                             /* Bit mask for AES_DATASTART */
#define _AES_CTRL_DATASTART_DEFAULT     0x00000000UL                       /* Mode DEFAULT for AES_CTRL */
#define AES_CTRL_DATASTART_DEFAULT      (_AES_CTRL_DATASTART_DEFAULT << 4) /* Shifted mode DEFAULT for AES_CTRL */
#define AES_CTRL_XORSTART               (0x1UL << 5)                       /* AES_XORDATA Write Start */
#define _AES_CTRL_XORSTART_SHIFT        5                                  /* Shift value for AES_XORSTART */
#define _AES_CTRL_XORSTART_MASK         0x20UL                             /* Bit mask for AES_XORSTART */
#define _AES_CTRL_XORSTART_DEFAULT      0x00000000UL                       /* Mode DEFAULT for AES_CTRL */
#define AES_CTRL_XORSTART_DEFAULT       (_AES_CTRL_XORSTART_DEFAULT << 5)  /* Shifted mode DEFAULT for AES_CTRL */
#define AES_CTRL_BYTEORDER              (0x1UL << 6)                       /* Configure byte order in data and key registers */
#define _AES_CTRL_BYTEORDER_SHIFT       6                                  /* Shift value for AES_BYTEORDER */
#define _AES_CTRL_BYTEORDER_MASK        0x40UL                             /* Bit mask for AES_BYTEORDER */
#define _AES_CTRL_BYTEORDER_DEFAULT     0x00000000UL                       /* Mode DEFAULT for AES_CTRL */
#define AES_CTRL_BYTEORDER_DEFAULT      (_AES_CTRL_BYTEORDER_DEFAULT << 6) /* Shifted mode DEFAULT for AES_CTRL */

/* Bit fields for AES CMD */

#define _AES_CMD_RESETVALUE             0x00000000UL                  /* Default value for AES_CMD */
#define _AES_CMD_MASK                   0x00000003UL                  /* Mask for AES_CMD */

#define AES_CMD_START                   (0x1UL << 0)                  /* Encryption/Decryption Start */
#define _AES_CMD_START_SHIFT            0                             /* Shift value for AES_START */
#define _AES_CMD_START_MASK             0x1UL                         /* Bit mask for AES_START */
#define _AES_CMD_START_DEFAULT          0x00000000UL                  /* Mode DEFAULT for AES_CMD */
#define AES_CMD_START_DEFAULT           (_AES_CMD_START_DEFAULT << 0) /* Shifted mode DEFAULT for AES_CMD */
#define AES_CMD_STOP                    (0x1UL << 1)                  /* Encryption/Decryption Stop */
#define _AES_CMD_STOP_SHIFT             1                             /* Shift value for AES_STOP */
#define _AES_CMD_STOP_MASK              0x2UL                         /* Bit mask for AES_STOP */
#define _AES_CMD_STOP_DEFAULT           0x00000000UL                  /* Mode DEFAULT for AES_CMD */
#define AES_CMD_STOP_DEFAULT            (_AES_CMD_STOP_DEFAULT << 1)  /* Shifted mode DEFAULT for AES_CMD */

/* Bit fields for AES STATUS */

#define _AES_STATUS_RESETVALUE          0x00000000UL                       /* Default value for AES_STATUS */
#define _AES_STATUS_MASK                0x00000001UL                       /* Mask for AES_STATUS */

#define AES_STATUS_RUNNING              (0x1UL << 0)                       /* AES Running */
#define _AES_STATUS_RUNNING_SHIFT       0                                  /* Shift value for AES_RUNNING */
#define _AES_STATUS_RUNNING_MASK        0x1UL                              /* Bit mask for AES_RUNNING */
#define _AES_STATUS_RUNNING_DEFAULT     0x00000000UL                       /* Mode DEFAULT for AES_STATUS */
#define AES_STATUS_RUNNING_DEFAULT      (_AES_STATUS_RUNNING_DEFAULT << 0) /* Shifted mode DEFAULT for AES_STATUS */

/* Bit fields for AES IEN */

#define _AES_IEN_RESETVALUE             0x00000000UL                 /* Default value for AES_IEN */
#define _AES_IEN_MASK                   0x00000001UL                 /* Mask for AES_IEN */

#define AES_IEN_DONE                    (0x1UL << 0)                 /* Encryption/Decryption Done Interrupt Enable */
#define _AES_IEN_DONE_SHIFT             0                            /* Shift value for AES_DONE */
#define _AES_IEN_DONE_MASK              0x1UL                        /* Bit mask for AES_DONE */
#define _AES_IEN_DONE_DEFAULT           0x00000000UL                 /* Mode DEFAULT for AES_IEN */
#define AES_IEN_DONE_DEFAULT            (_AES_IEN_DONE_DEFAULT << 0) /* Shifted mode DEFAULT for AES_IEN */

/* Bit fields for AES IF */

#define _AES_IF_RESETVALUE              0x00000000UL                /* Default value for AES_IF */
#define _AES_IF_MASK                    0x00000001UL                /* Mask for AES_IF */

#define AES_IF_DONE                     (0x1UL << 0)                /* Encryption/Decryption Done Interrupt Flag */
#define _AES_IF_DONE_SHIFT              0                           /* Shift value for AES_DONE */
#define _AES_IF_DONE_MASK               0x1UL                       /* Bit mask for AES_DONE */
#define _AES_IF_DONE_DEFAULT            0x00000000UL                /* Mode DEFAULT for AES_IF */
#define AES_IF_DONE_DEFAULT             (_AES_IF_DONE_DEFAULT << 0) /* Shifted mode DEFAULT for AES_IF */

/* Bit fields for AES IFS */

#define _AES_IFS_RESETVALUE             0x00000000UL                 /* Default value for AES_IFS */
#define _AES_IFS_MASK                   0x00000001UL                 /* Mask for AES_IFS */

#define AES_IFS_DONE                    (0x1UL << 0)                 /* Encryption/Decryption Done Interrupt Flag Set */
#define _AES_IFS_DONE_SHIFT             0                            /* Shift value for AES_DONE */
#define _AES_IFS_DONE_MASK              0x1UL                        /* Bit mask for AES_DONE */
#define _AES_IFS_DONE_DEFAULT           0x00000000UL                 /* Mode DEFAULT for AES_IFS */
#define AES_IFS_DONE_DEFAULT            (_AES_IFS_DONE_DEFAULT << 0) /* Shifted mode DEFAULT for AES_IFS */

/* Bit fields for AES IFC */

#define _AES_IFC_RESETVALUE             0x00000000UL                 /* Default value for AES_IFC */
#define _AES_IFC_MASK                   0x00000001UL                 /* Mask for AES_IFC */

#define AES_IFC_DONE                    (0x1UL << 0)                 /* Encryption/Decryption Done Interrupt Flag Clear */
#define _AES_IFC_DONE_SHIFT             0                            /* Shift value for AES_DONE */
#define _AES_IFC_DONE_MASK              0x1UL                        /* Bit mask for AES_DONE */
#define _AES_IFC_DONE_DEFAULT           0x00000000UL                 /* Mode DEFAULT for AES_IFC */
#define AES_IFC_DONE_DEFAULT            (_AES_IFC_DONE_DEFAULT << 0) /* Shifted mode DEFAULT for AES_IFC */

/* Bit fields for AES DATA */

#define _AES_DATA_RESETVALUE            0x00000000UL                  /* Default value for AES_DATA */
#define _AES_DATA_MASK                  0xFFFFFFFFUL                  /* Mask for AES_DATA */

#define _AES_DATA_DATA_SHIFT            0                             /* Shift value for AES_DATA */
#define _AES_DATA_DATA_MASK             0xFFFFFFFFUL                  /* Bit mask for AES_DATA */
#define _AES_DATA_DATA_DEFAULT          0x00000000UL                  /* Mode DEFAULT for AES_DATA */
#define AES_DATA_DATA_DEFAULT           (_AES_DATA_DATA_DEFAULT << 0) /* Shifted mode DEFAULT for AES_DATA */

/* Bit fields for AES XORDATA */

#define _AES_XORDATA_RESETVALUE         0x00000000UL                        /* Default value for AES_XORDATA */
#define _AES_XORDATA_MASK               0xFFFFFFFFUL                        /* Mask for AES_XORDATA */

#define _AES_XORDATA_XORDATA_SHIFT      0                                   /* Shift value for AES_XORDATA */
#define _AES_XORDATA_XORDATA_MASK       0xFFFFFFFFUL                        /* Bit mask for AES_XORDATA */
#define _AES_XORDATA_XORDATA_DEFAULT    0x00000000UL                        /* Mode DEFAULT for AES_XORDATA */
#define AES_XORDATA_XORDATA_DEFAULT     (_AES_XORDATA_XORDATA_DEFAULT << 0) /* Shifted mode DEFAULT for AES_XORDATA */

/* Bit fields for AES KEYLA */

#define _AES_KEYLA_RESETVALUE           0x00000000UL                    /* Default value for AES_KEYLA */
#define _AES_KEYLA_MASK                 0xFFFFFFFFUL                    /* Mask for AES_KEYLA */

#define _AES_KEYLA_KEYLA_SHIFT          0                               /* Shift value for AES_KEYLA */
#define _AES_KEYLA_KEYLA_MASK           0xFFFFFFFFUL                    /* Bit mask for AES_KEYLA */
#define _AES_KEYLA_KEYLA_DEFAULT        0x00000000UL                    /* Mode DEFAULT for AES_KEYLA */
#define AES_KEYLA_KEYLA_DEFAULT         (_AES_KEYLA_KEYLA_DEFAULT << 0) /* Shifted mode DEFAULT for AES_KEYLA */

/* Bit fields for AES KEYLB */

#define _AES_KEYLB_RESETVALUE           0x00000000UL                    /* Default value for AES_KEYLB */
#define _AES_KEYLB_MASK                 0xFFFFFFFFUL                    /* Mask for AES_KEYLB */

#define _AES_KEYLB_KEYLB_SHIFT          0                               /* Shift value for AES_KEYLB */
#define _AES_KEYLB_KEYLB_MASK           0xFFFFFFFFUL                    /* Bit mask for AES_KEYLB */
#define _AES_KEYLB_KEYLB_DEFAULT        0x00000000UL                    /* Mode DEFAULT for AES_KEYLB */
#define AES_KEYLB_KEYLB_DEFAULT         (_AES_KEYLB_KEYLB_DEFAULT << 0) /* Shifted mode DEFAULT for AES_KEYLB */

/* Bit fields for AES KEYLC */

#define _AES_KEYLC_RESETVALUE           0x00000000UL                    /* Default value for AES_KEYLC */
#define _AES_KEYLC_MASK                 0xFFFFFFFFUL                    /* Mask for AES_KEYLC */

#define _AES_KEYLC_KEYLC_SHIFT          0                               /* Shift value for AES_KEYLC */
#define _AES_KEYLC_KEYLC_MASK           0xFFFFFFFFUL                    /* Bit mask for AES_KEYLC */
#define _AES_KEYLC_KEYLC_DEFAULT        0x00000000UL                    /* Mode DEFAULT for AES_KEYLC */
#define AES_KEYLC_KEYLC_DEFAULT         (_AES_KEYLC_KEYLC_DEFAULT << 0) /* Shifted mode DEFAULT for AES_KEYLC */

/* Bit fields for AES KEYLD */

#define _AES_KEYLD_RESETVALUE           0x00000000UL                    /* Default value for AES_KEYLD */
#define _AES_KEYLD_MASK                 0xFFFFFFFFUL                    /* Mask for AES_KEYLD */

#define _AES_KEYLD_KEYLD_SHIFT          0                               /* Shift value for AES_KEYLD */
#define _AES_KEYLD_KEYLD_MASK           0xFFFFFFFFUL                    /* Bit mask for AES_KEYLD */
#define _AES_KEYLD_KEYLD_DEFAULT        0x00000000UL                    /* Mode DEFAULT for AES_KEYLD */
#define AES_KEYLD_KEYLD_DEFAULT         (_AES_KEYLD_KEYLD_DEFAULT << 0) /* Shifted mode DEFAULT for AES_KEYLD */

/* Bit fields for AES KEYHA */

#define _AES_KEYHA_RESETVALUE           0x00000000UL                    /* Default value for AES_KEYHA */
#define _AES_KEYHA_MASK                 0xFFFFFFFFUL                    /* Mask for AES_KEYHA */

#define _AES_KEYHA_KEYHA_SHIFT          0                               /* Shift value for AES_KEYHA */
#define _AES_KEYHA_KEYHA_MASK           0xFFFFFFFFUL                    /* Bit mask for AES_KEYHA */
#define _AES_KEYHA_KEYHA_DEFAULT        0x00000000UL                    /* Mode DEFAULT for AES_KEYHA */
#define AES_KEYHA_KEYHA_DEFAULT         (_AES_KEYHA_KEYHA_DEFAULT << 0) /* Shifted mode DEFAULT for AES_KEYHA */

/* Bit fields for AES KEYHB */

#define _AES_KEYHB_RESETVALUE           0x00000000UL                    /* Default value for AES_KEYHB */
#define _AES_KEYHB_MASK                 0xFFFFFFFFUL                    /* Mask for AES_KEYHB */

#define _AES_KEYHB_KEYHB_SHIFT          0                               /* Shift value for AES_KEYHB */
#define _AES_KEYHB_KEYHB_MASK           0xFFFFFFFFUL                    /* Bit mask for AES_KEYHB */
#define _AES_KEYHB_KEYHB_DEFAULT        0x00000000UL                    /* Mode DEFAULT for AES_KEYHB */
#define AES_KEYHB_KEYHB_DEFAULT         (_AES_KEYHB_KEYHB_DEFAULT << 0) /* Shifted mode DEFAULT for AES_KEYHB */

/* Bit fields for AES KEYHC */

#define _AES_KEYHC_RESETVALUE           0x00000000UL                    /* Default value for AES_KEYHC */
#define _AES_KEYHC_MASK                 0xFFFFFFFFUL                    /* Mask for AES_KEYHC */

#define _AES_KEYHC_KEYHC_SHIFT          0                               /* Shift value for AES_KEYHC */
#define _AES_KEYHC_KEYHC_MASK           0xFFFFFFFFUL                    /* Bit mask for AES_KEYHC */
#define _AES_KEYHC_KEYHC_DEFAULT        0x00000000UL                    /* Mode DEFAULT for AES_KEYHC */
#define AES_KEYHC_KEYHC_DEFAULT         (_AES_KEYHC_KEYHC_DEFAULT << 0) /* Shifted mode DEFAULT for AES_KEYHC */

/* Bit fields for AES KEYHD */

#define _AES_KEYHD_RESETVALUE           0x00000000UL                    /* Default value for AES_KEYHD */
#define _AES_KEYHD_MASK                 0xFFFFFFFFUL                    /* Mask for AES_KEYHD */

#define _AES_KEYHD_KEYHD_SHIFT          0                               /* Shift value for AES_KEYHD */
#define _AES_KEYHD_KEYHD_MASK           0xFFFFFFFFUL                    /* Bit mask for AES_KEYHD */
#define _AES_KEYHD_KEYHD_DEFAULT        0x00000000UL                    /* Mode DEFAULT for AES_KEYHD */
#define AES_KEYHD_KEYHD_DEFAULT         (_AES_KEYHD_KEYHD_DEFAULT << 0) /* Shifted mode DEFAULT for AES_KEYHD */

#endif /* __ARCH_ARM_SRC_EFM32_CHIP_EFM32_AES_H_ */

/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_ihc.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_IHC_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_IHC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_NUM_HARTS               5
#define UNDEFINED_HART_ID            99

#define LIBERO_SETTING_CONTEXT_A_HART_EN ((CONFIG_MPFS_IHC_LINUX_ON_HART4 << 4) | \
                                          (CONFIG_MPFS_IHC_LINUX_ON_HART3 << 3) | \
                                          (CONFIG_MPFS_IHC_LINUX_ON_HART2 << 2) | \
                                          (CONFIG_MPFS_IHC_LINUX_ON_HART1 << 1))

#define LIBERO_SETTING_CONTEXT_B_HART_EN ((CONFIG_MPFS_IHC_NUTTX_ON_HART4 << 4) | \
                                          (CONFIG_MPFS_IHC_NUTTX_ON_HART3 << 3) | \
                                          (CONFIG_MPFS_IHC_NUTTX_ON_HART2 << 2) | \
                                          (CONFIG_MPFS_IHC_NUTTX_ON_HART1 << 1))

#if (LIBERO_SETTING_CONTEXT_A_HART_EN & LIBERO_SETTING_CONTEXT_B_HART_EN)
#  error Harts misconfigured. Cannot use the same harts.
#endif

/* Contex A and B hart ID's used in this system. Context A is the master. */

#if CONFIG_MPFS_IHC_LINUX_ON_HART1 == 1
#define CONTEXTA_HARTID 0x01
#elif CONFIG_MPFS_IHC_LINUX_ON_HART2 == 1
#define CONTEXTA_HARTID 0x02
#elif CONFIG_MPFS_IHC_LINUX_ON_HART3 == 1
#define CONTEXTA_HARTID 0x03
#elif CONFIG_MPFS_IHC_LINUX_ON_HART4 == 1
#define CONTEXTA_HARTID 0x04
#else
#  error Context A is required
#endif

#if CONFIG_MPFS_IHC_NUTTX_ON_HART1 == 1
#define CONTEXTB_HARTID 0x01
#elif CONFIG_MPFS_IHC_NUTTX_ON_HART2 == 1
#define CONTEXTB_HARTID 0x02
#elif CONFIG_MPFS_IHC_NUTTX_ON_HART3 == 1
#define CONTEXTB_HARTID 0x03
#elif CONFIG_MPFS_IHC_NUTTX_ON_HART4 == 1
#define CONTEXTB_HARTID 0x04
#else
#  error Context B is required
#endif

#if (CONTEXTA_HARTID == CONTEXTB_HARTID)
#  error Context A cannot be the same as Context B
#endif

/* My Hart 0 */

#define IHC_LOCAL_H0_REMOTE_H1       0x50000000
#define IHC_LOCAL_H0_REMOTE_H2       0x50000100
#define IHC_LOCAL_H0_REMOTE_H3       0x50000200
#define IHC_LOCAL_H0_REMOTE_H4       0x50000300
#define IHCIA_LOCAL_H0               0x50000400

/* My Hart 1 */

#define IHC_LOCAL_H1_REMOTE_H0       0x50000500
#define IHC_LOCAL_H1_REMOTE_H2       0x50000600
#define IHC_LOCAL_H1_REMOTE_H3       0x50000700
#define IHC_LOCAL_H1_REMOTE_H4       0x50000800
#define IHCIA_LOCAL_H1               0x50000900

/* My Hart 2 */

#define IHC_LOCAL_H2_REMOTE_H0       0x50000a00
#define IHC_LOCAL_H2_REMOTE_H1       0x50000b00
#define IHC_LOCAL_H2_REMOTE_H3       0x50000c00
#define IHC_LOCAL_H2_REMOTE_H4       0x50000d00
#define IHCIA_LOCAL_H2               0x50000e00

/* My Hart 3 */

#define IHC_LOCAL_H3_REMOTE_H0       0x50000f00
#define IHC_LOCAL_H3_REMOTE_H1       0x50001000
#define IHC_LOCAL_H3_REMOTE_H2       0x50001100
#define IHC_LOCAL_H3_REMOTE_H4       0x50001200
#define IHCIA_LOCAL_H3               0x50001300

/* My Hart 4 */

#define IHC_LOCAL_H4_REMOTE_H0       0x50001400
#define IHC_LOCAL_H4_REMOTE_H1       0x50001500
#define IHC_LOCAL_H4_REMOTE_H2       0x50001600
#define IHC_LOCAL_H4_REMOTE_H3       0x50001700
#define IHCIA_LOCAL_H4               0x50001800

#define MPFS_IHC_VERSION_OFFSET      0x00
#define MPFS_IHC_CTRL_OFFSET         0x04
#define MPFS_IHC_LOCAL_HARTID_OFFSET 0x08
#define MPFS_IHC_MSG_SIZE_OFFSET     0x0c
#define MPFS_IHC_MSG_UNUSED_OFFSET   0x10
#define MPFS_IHC_MSG_IN_OFFSET       0x20
#define MPFS_IHC_MSG_OUT_OFFSET      0x30

#define MPFS_IHC_INT_EN_OFFSET       0x04
#define MPFS_IHC_MSG_AVAIL_OFFSET    0x08

#define MPFS_LOCAL_REMOTE_OFFSET(l, r) (0x500 * l + 0x100 * r)

/* The registers don't go linearly in all cases, use a fixup */

#define MPFS_L_R_FIXUP(l, r)        (((l > 0 && l < 4) && (l < r)) ? -0x100 : 0)

#define MPFS_IHC_VERSION(l, r)      (IHC_LOCAL_H0_REMOTE_H1 + MPFS_IHC_VERSION_OFFSET + MPFS_LOCAL_REMOTE_OFFSET(l, r) + MPFS_L_R_FIXUP(l, r))
#define MPFS_IHC_CTRL(l, r)         (IHC_LOCAL_H0_REMOTE_H1 + MPFS_IHC_CTRL_OFFSET + MPFS_LOCAL_REMOTE_OFFSET(l, r) + MPFS_L_R_FIXUP(l, r))
#define MPFS_IHC_LOCAL_HARTID(l, r) (IHC_LOCAL_H0_REMOTE_H1 + MPFS_IHC_LOCAL_HARTID_OFFSET + MPFS_LOCAL_REMOTE_OFFSET(l, r) + MPFS_L_R_FIXUP(l, r))
#define MPFS_IHC_MSG_SIZE(l, r)     (IHC_LOCAL_H0_REMOTE_H1 + MPFS_IHC_MSG_SIZE_OFFSET + MPFS_LOCAL_REMOTE_OFFSET(l, r) + MPFS_L_R_FIXUP(l, r))
#define MPFS_IHC_MSG_IN(l, r)       (IHC_LOCAL_H0_REMOTE_H1 + MPFS_IHC_MSG_IN_OFFSET + MPFS_LOCAL_REMOTE_OFFSET(l, r) + MPFS_L_R_FIXUP(l, r))
#define MPFS_IHC_MSG_OUT(l, r)      (IHC_LOCAL_H0_REMOTE_H1 + MPFS_IHC_MSG_OUT_OFFSET + MPFS_LOCAL_REMOTE_OFFSET(l, r) + MPFS_L_R_FIXUP(l, r))

#define MPFS_IHC_INT_EN(l)          (IHCIA_LOCAL_H0 + MPFS_IHC_INT_EN_OFFSET + 0x500 * l)
#define MPFS_IHC_MSG_AVAIL(l)       (IHCIA_LOCAL_H0 + MPFS_IHC_MSG_AVAIL_OFFSET + 0x500 * l)

/* Hart mask defines */

#define HART0_ID                0
#define HART1_ID                1
#define HART2_ID                2
#define HART3_ID                3
#define HART4_ID                4

#define HART0_MASK              1
#define HART1_MASK              2
#define HART2_MASK              4
#define HART3_MASK              8
#define HART4_MASK              0x10

/* Monitor hart (HSS hart) used in our system */

#define HSS_HART_MASK           HART0_MASK
#define HSS_HART_ID             HART0_ID

/* HSS_REMOTE_HARTS_MASK: This is used to define the harts the HSS is
 * communicating with
 */

#define HSS_REMOTE_HARTS_MASK 	(HART1_MASK | HART2_MASK | HART3_MASK | HART4_MASK)

/* Define which harts are connected via comms channels to a particular hart
 * user defined.
 */

#define IHCIA_H0_REMOTE_HARTS	((~HSS_HART_MASK) & HSS_REMOTE_HARTS_MASK)

#define IHCIA_H1_REMOTE_HARTS	(HSS_HART_MASK | (1 << CONTEXTA_HARTID))
#define IHCIA_H2_REMOTE_HARTS	(HSS_HART_MASK | (1 << CONTEXTA_HARTID))
#define IHCIA_H3_REMOTE_HARTS	(HSS_HART_MASK | (1 << CONTEXTA_HARTID))
#define IHCIA_H4_REMOTE_HARTS	(HSS_HART_MASK | (1 << CONTEXTA_HARTID))

#define HSS_HART_DEFAULT_INT_EN (0 << 0)

#define HSS_HART_MP_INT_EN      (1 << 0)
#define HSS_HART_ACK_INT_EN     (1 << 1)

#define HART1_MP_INT_EN         (1 << 2)
#define HART1_ACK_INT_EN        (1 << 3)

#define HART2_MP_INT_EN         (1 << 4)
#define HART2_ACK_INT_EN        (1 << 5)

#define HART3_MP_INT_EN         (1 << 6)
#define HART3_ACK_INT_EN        (1 << 7)

#define HART4_MP_INT_EN         (1 << 8)
#define HART4_ACK_INT_EN        (1 << 9)

/* Connected to all harts */

#define IHCIA_H0_REMOTE_HARTS_INTS  HSS_HART_DEFAULT_INT_EN

/* HSS and Context B connected */

#define IHCIA_CONTEXTA_INTS        (HSS_HART_MP_INT_EN           | \
                                    HSS_HART_ACK_INT_EN          | \
                                    (1 << (CONTEXTB_HARTID * 2)) | \
                                    (1 << (CONTEXTB_HARTID * 2 + 1)))

#define IHCIA_CONTEXTB_INTS        (HSS_HART_MP_INT_EN           | \
                                    HSS_HART_ACK_INT_EN          | \
                                    (1 << (CONTEXTA_HARTID * 2)) | \
                                    (1 << (CONTEXTA_HARTID * 2 + 1)))

/* Context B interrupts */

#if CONTEXTB_HARTID == 1
#define IHCIA_H1_REMOTE_HARTS_INTS  IHCIA_CONTEXTB_INTS
#else
#define IHCIA_H1_REMOTE_HARTS_INTS  HSS_HART_DEFAULT_INT_EN
#endif

#if CONTEXTB_HARTID == 2
#define IHCIA_H2_REMOTE_HARTS_INTS  IHCIA_CONTEXTB_INTS
#else
#define IHCIA_H2_REMOTE_HARTS_INTS  HSS_HART_DEFAULT_INT_EN
#endif

#if CONTEXTB_HARTID == 3
#define IHCIA_H3_REMOTE_HARTS_INTS  IHCIA_CONTEXTB_INTS
#else
#define IHCIA_H3_REMOTE_HARTS_INTS  HSS_HART_DEFAULT_INT_EN
#endif

#if CONTEXTB_HARTID == 4
#define IHCIA_H4_REMOTE_HARTS_INTS  IHCIA_CONTEXTB_INTS
#else
#define IHCIA_H4_REMOTE_HARTS_INTS  HSS_HART_DEFAULT_INT_EN
#endif

/* Context A interrupts */

#if CONTEXTA_HARTID == 1
#undef IHCIA_H1_REMOTE_HARTS_INTS
#define IHCIA_H1_REMOTE_HARTS_INTS  IHCIA_CONTEXTA_INTS
#endif

#if CONTEXTA_HARTID == 2
#undef IHCIA_H2_REMOTE_HARTS_INTS
#define IHCIA_H2_REMOTE_HARTS_INTS  IHCIA_CONTEXTA_INTS
#endif

#if CONTEXTA_HARTID == 3
#undef IHCIA_H3_REMOTE_HARTS_INTS
#define IHCIA_H3_REMOTE_HARTS_INTS  IHCIA_CONTEXTA_INTS
#endif

#if CONTEXTA_HARTID == 4
#undef IHCIA_H4_REMOTE_HARTS_INTS
#define IHCIA_H4_REMOTE_HARTS_INTS  IHCIA_CONTEXTA_INTS
#endif

/* MiV-IHCC register bit definitions */

#define RMP_MESSAGE_PRESENT    (1 << 0)  /* Remote side message present */
#define MP_MESSAGE_PRESENT     (1 << 1)  /* Local side message present */
#define MPIE_EN                (1 << 2)  /* Enable MP interrupt */
#define ACK_INT                (1 << 3)  /* Incoming ACK */
#define ACK_CLR                (1 << 4)  /* Clear ACK */
#define ACKIE_EN               (1 << 5)  /* Enable Ack Interrupt */

/* Control register bit MASKS */

#define RMP_MASK               (1 << 0)
#define MP_MASK                (1 << 1)
#define MPIE_MASK              (1 << 2)
#define ACK_INT_MASK           (1 << 3)

#define IHC_MAX_MESSAGE_SIZE    2

#define SBI_EXT_IHC_CTX_INIT    0
#define SBI_EXT_IHC_SEND        1
#define SBI_EXT_IHC_RECEIVE     2

enum ihc_channel_e
{
  IHC_CHANNEL_TO_HART0    = 0x00, /* Your hart to hart 0 */
  IHC_CHANNEL_TO_HART1    = 0x01, /* Your hart to hart 1 */
  IHC_CHANNEL_TO_HART2    = 0x02, /* Your hart to hart 2 */
  IHC_CHANNEL_TO_HART3    = 0x03, /* Your hart to hart 3 */
  IHC_CHANNEL_TO_HART4    = 0x04, /* Your hart to hart 4 */
  IHC_CHANNEL_TO_CONTEXTA = 0x05, /* Your hart to context A */
  IHC_CHANNEL_TO_CONTEXTB = 0x06, /* Your hart to context B */
};

typedef enum ihc_channel_e ihc_channel_t;

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_IHC_H */

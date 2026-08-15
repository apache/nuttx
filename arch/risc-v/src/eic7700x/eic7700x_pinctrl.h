/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_pinctrl.h
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

#ifndef __ARCH_RISCV_SRC_EIC7700X_EIC7700X_PINCTRL_H
#define __ARCH_RISCV_SRC_EIC7700X_EIC7700X_PINCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/pinctrl/pinctrl.h>

#include <arch/chip/eic7700x_pinctrl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many distinct reset values the 166 pads have between them.  The
 * manual gives a reset value per field; reassembled into words they
 * collapse to this many, so the table stores an index instead of a word.
 */

#define EIC7700X_PAD_NDEFAULTS     (15)

/* How many GPIO lines the SoC has.  Two of them, 5 and 11, are balls in
 * part 1 table 2-4 that the register detail description gives no pad
 * register for, so the reverse map below has holes there.
 */

#define EIC7700X_PAD_NGPIOS        (112)
#define EIC7700X_PAD_NOPAD         (0xff)

/* A pad's gpiofunc when the pad offers no GPIO function at all. */

#define EIC7700X_PAD_NOGPIO        (0xff)

/* Pad flags */

#define EIC7700X_PAD_LOCKED        (1 << 0)  /* Refuses to be written */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Which of the four field layouts a pad uses.  See the layout notes in
 * hardware/eic7700x_pinctrl.h: the same bit means different things, or
 * nothing, depending on this.
 */

enum eic7700x_padshape_e
{
  EIC7700X_PADSHAPE_GENERAL = 0,  /* Function select, pull, drive, input  */
  EIC7700X_PADSHAPE_RGMII,        /* Voltage mode instead of function     */
  EIC7700X_PADSHAPE_OSC,          /* Drive, damping and feedback only     */
  EIC7700X_PADSHAPE_MODESEL       /* Group voltage select, carries no pin */
};

/* One pad register.
 *
 * wmask comes straight from the access column of the manual's register
 * detail description, one bit per writable register bit.  It is the only
 * thing standing between a caller and a write the hardware will ignore,
 * and five pads have it zero: the hardware reports their state and nothing
 * more.
 */

struct eic7700x_pad_s
{
  uint32_t wmask;        /* Bits the hardware will let software drive     */
  uint8_t  shape;        /* enum eic7700x_padshape_e                      */
  uint8_t  dflt;         /* Index into g_eic7700x_pad_defaults[]          */
  uint8_t  funcmask;     /* Bit n set: function select n is documented    */
  uint8_t  gpiofunc;     /* Function select giving GPIO, or NOGPIO        */
  uint8_t  flags;        /* EIC7700X_PAD_* flags                          */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN const struct eic7700x_pad_s
             g_eic7700x_pads[EIC7700X_PAD_NPADS];
EXTERN const uint32_t
             g_eic7700x_pad_defaults[EIC7700X_PAD_NDEFAULTS];
EXTERN const uint8_t
             g_eic7700x_pad_bygpio[EIC7700X_PAD_NGPIOS];
EXTERN const struct pinctrl_ops_s g_eic7700x_pinctrl_ops;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_pinctrl_initialize
 *
 * Description:
 *   Bring up the pad multiplexing block.  This writes nothing to the
 *   hardware: a pad only moves when a driver asks it to.
 *
 * Returned Value:
 *   OK on success, or a negated errno on failure.
 *
 ****************************************************************************/

int eic7700x_pinctrl_initialize(void);

/****************************************************************************
 * Name: eic7700x_pinctrl_count
 *
 * Description:
 *   How many pads the block has, and through changed how many currently
 *   differ from their reset defaults.
 *
 * Input Parameters:
 *   changed - Receives the changed-pad count; may be NULL.
 *
 * Returned Value:
 *   The number of pads.
 *
 ****************************************************************************/

unsigned int eic7700x_pinctrl_count(FAR unsigned int *changed);

/****************************************************************************
 * Name: eic7700x_pad_lookup
 *
 * Description:
 *   Bound a pad id and return the row describing it.  Nothing else in this
 *   driver may index the pad table directly: this is the single place an
 *   out of range id is caught.
 *
 * Input Parameters:
 *   pad - The pad id, an enum eic7700x_pad_e value
 *
 * Returned Value:
 *   The pad description, or NULL if the id names no pad.
 *
 ****************************************************************************/

FAR const struct eic7700x_pad_s *eic7700x_pad_lookup(unsigned int pad);

/****************************************************************************
 * Name: eic7700x_pad_config
 *
 * Description:
 *   Configure one pad.  The value is a whole pad register, built from the
 *   PINCTRL_* field macros for the layout the pad uses, and every field
 *   lands in a single write so that a pad is never briefly half
 *   configured.  Bits the hardware will not let software drive keep the
 *   value they already had.
 *
 * Input Parameters:
 *   pad - The pad id, an enum eic7700x_pad_e value
 *   cfg - The register value to install
 *
 * Returned Value:
 *   OK on success, or a negated errno:
 *
 *     -EINVAL  the id names no pad, or the value selects a function the
 *              manual does not document for this pad
 *     -EROFS   the value tries to drive a bit that reports state only
 *     -EPERM   the pad is one this driver refuses to move
 *
 ****************************************************************************/

int eic7700x_pad_config(unsigned int pad, uint32_t cfg);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_EIC7700X_EIC7700X_PINCTRL_H */

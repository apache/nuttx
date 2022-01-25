/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_cmt.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_RX65N_CMT_H
#define __ARCH_RENESAS_SRC_RX65N_RX65N_CMT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Compare Match Timer Control Register (CMCR) */

/* Interrupt Source Priority Register n (IPRn) */

/* Interrupt Priority Level Select (IPR[3:0]) */

#define _00_CMT_PRIORITY_LEVEL0    (0x00U) /* Level 0 (interrupt disabled) */
#define _01_CMT_PRIORITY_LEVEL1    (0x01U) /* Level 1 */
#define _02_CMT_PRIORITY_LEVEL2    (0x02U) /* Level 2 */
#define _03_CMT_PRIORITY_LEVEL3    (0x03U) /* Level 3 */
#define _04_CMT_PRIORITY_LEVEL4    (0x04U) /* Level 4 */
#define _05_CMT_PRIORITY_LEVEL5    (0x05U) /* Level 5 */
#define _06_CMT_PRIORITY_LEVEL6    (0x06U) /* Level 6 */
#define _07_CMT_PRIORITY_LEVEL7    (0x07U) /* Level 7 */
#define _08_CMT_PRIORITY_LEVEL8    (0x08U) /* Level 8 */
#define _09_CMT_PRIORITY_LEVEL9    (0x09U) /* Level 9 */
#define _0A_CMT_PRIORITY_LEVEL10   (0x0aU) /* Level 10 */
#define _0B_CMT_PRIORITY_LEVEL11   (0x0bU) /* Level 11 */
#define _0C_CMT_PRIORITY_LEVEL12   (0x0cU) /* Level 12 */
#define _0D_CMT_PRIORITY_LEVEL13   (0x0dU) /* Level 13 */
#define _0E_CMT_PRIORITY_LEVEL14   (0x0eU) /* Level 14 */
#define _0F_CMT_PRIORITY_LEVEL15   (0x0fU) /* Level 15 (highest) */

#endif /* __ARCH_RENESAS_SRC_RX65N_RX65N_CMT_H */

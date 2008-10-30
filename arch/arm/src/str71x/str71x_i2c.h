/************************************************************************************
 * arch/arm/src/str71x/str71x_i2c.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_I2C_H
#define __ARCH_ARM_SRC_STR71X_STR71X_I2C_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include "str71x_map.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register offets ******************************************************************/

#define STR71X_I2C_SR_OFFSET   (0x0000) /* 8-bits wide */
#define STR71X_I2C_SR1_OFFSET  (0x0004) /* 8-bits wide */
#define STR71X_I2C_SR2_OFFSET  (0x0008) /* 8-bits wide */
#define STR71X_I2C_CCR_OFFSET  (0x000c) /* 8-bits wide */
#define STR71X_I2C_OAR1_OFFSET (0x0010) /* 8-bits wide */
#define STR71X_I2C_OAR2_OFFSET (0x0014) /* 8-bits wide */
#define STR71X_I2C_DR_OFFSET   (0x0018) /* 8-bits wide */
#define STR71X_I2C_ECCR_OFFSET (0x001c) /* 8-bits wide */

/* Registers ************************************************************************/

#define STR71X_I2C_SR(b)   ((b) + STR71X_I2C_SR_OFFSET)
#define STR71X_I2C_SR1(b)  ((b) + STR71X_I2C_SR1_OFFSET)
#define STR71X_I2C_SR2(b)  ((b) + STR71X_I2C_SR2_OFFSET)
#define STR71X_I2C_CCR(b)  ((b) + STR71X_I2C_CCR_OFFSET)
#define STR71X_I2C_OAR1(b) ((b) + STR71X_I2C_OAR1_OFFSET)
#define STR71X_I2C_OAR2(b) ((b) + STR71X_I2C_OAR2_OFFSET)
#define STR71X_I2C_DR(b)   ((b) + STR71X_I2C_DR_OFFSET)
#define STR71X_I2C_ECCR(b) ((b) + STR71X_I2C_ECCR_OFFSET)

#define STR71X_I2C0_SR     (STR71X_I2C0_BASE + STR71X_I2C_SR_OFFSET)
#define STR71X_I2C0_SR1    (STR71X_I2C0_BASE + STR71X_I2C_SR1_OFFSET)
#define STR71X_I2C0_SR2    (STR71X_I2C0_BASE + STR71X_I2C_SR2_OFFSET)
#define STR71X_I2C0_CCR    (STR71X_I2C0_BASE + STR71X_I2C_CCR_OFFSET)
#define STR71X_I2C0_OAR1   (STR71X_I2C0_BASE + STR71X_I2C_OAR1_OFFSET)
#define STR71X_I2C0_OAR2   (STR71X_I2C0_BASE + STR71X_I2C_OAR2_OFFSET)
#define STR71X_I2C0_DR     (STR71X_I2C0_BASE + STR71X_I2C_DR_OFFSET)
#define STR71X_I2C0_ECCR   (STR71X_I2C0_BASE + STR71X_I2C_ECCR_OFFSET)

#define STR71X_I2C1_SR     (STR71X_I2C1_BASE + STR71X_I2C_SR_OFFSET)
#define STR71X_I2C1_SR1    (STR71X_I2C1_BASE + STR71X_I2C_SR1_OFFSET)
#define STR71X_I2C1_SR2    (STR71X_I2C1_BASE + STR71X_I2C_SR2_OFFSET)
#define STR71X_I2C1_CCR    (STR71X_I2C1_BASE + STR71X_I2C_CCR_OFFSET)
#define STR71X_I2C1_OAR1   (STR71X_I2C1_BASE + STR71X_I2C_OAR1_OFFSET)
#define STR71X_I2C1_OAR2   (STR71X_I2C1_BASE + STR71X_I2C_OAR2_OFFSET)
#define STR71X_I2C1_DR     (STR71X_I2C1_BASE + STR71X_I2C_DR_OFFSET)
#define STR71X_I2C1_ECCR   (STR71X_I2C1_BASE + STR71X_I2C_ECCR_OFFSET)

/* Register bit settings ***********************************************************/

  ST71X_I2C_CR   = 0x00,
  ST71X_I2C_SR1  = 0x04,
  ST71X_I2C_SR2  = 0x08,
  ST71X_I2C_CCR  = 0x0C,
  ST71X_I2C_OAR1 = 0x10,
  ST71X_I2C_OAR2 = 0x14,
  ST71X_I2C_DR   = 0x18,
  ST71X_I2C_ECCR = 0x1C
} ST71X_I2C_Registers;

#define ST71X_I2C_SB               (0x00000001)
#define ST71X_I2C_MSL              (0x00000002)
#define ST71X_I2C_ADSL             (0x00000004)
#define ST71X_I2C_BTF              (0x00000008)
#define ST71X_I2C_BUSY             (0x00000010)
#define ST71X_I2C_TRA              (0x00000020)
#define ST71X_I2C_ADD10            (0x00000040)
#define ST71X_I2C_EVF              (0x00000080)
#define ST71X_I2C_GCAL             (0x00000100)
#define ST71X_I2C_BERR             (0x00000200)
#define ST71X_I2C_ARLO             (0x00000400)
#define ST71X_I2C_STOPF            (0x00000800)
#define ST71X_I2C_AF               (0x00001000)
#define ST71X_I2C_ENDAD            (0x00002000)
#define ST71X_I2C_STOP             (0x00008000)
#define ST71X_I2C_ACK              (0x00010000)
#define ST71X_I2C_START            (0x00020000)

#define ST71X_I2C_PESET            (0x0020)
#define ST71X_I2C_PERESET          (0x00df)
#define ST71X_I2C_ENGC             (0x0010)
#define ST71X_I2C_START            (0x0008)
#define ST71X_I2C_STOP             (0x0002)
#define ST71X_I2C_ACK              (0x0004)
#define ST71X_I2C_ITE              (0x0001)
#define ST71X_I2C_EVENT            (0x3fff)

/* I2C Events */

#define  ST71X_I2C_EVENT_SLAVEADDRESSMATCHED   (ST71X_I2C_EVF|ST71X_I2C_BUSY|ST71X_I2C_ADSL)
#define  ST71X_I2C_EVENT_SLAVEBYTERECEIVED     (ST71X_I2C_EVF|ST71X_I2C_BUSY|ST71X_I2C_BTF)
#define  ST71X_I2C_EVENT_SLAVEBYTETRANSMITTED  (ST71X_I2C_EVF|ST71X_I2C_BUSY|ST71X_I2C_BTF|ST71X_I2C_TRA)
#define  ST71X_I2C_EVENT_MASTERMODESELECT      (ST71X_I2C_EVF|ST71X_I2C_BUSY|ST71X_I2C_MSL|ST71X_I2C_SB)
#define  ST71X_I2C_EVENT_MASTERMODESELECTED    (ST71X_I2C_EVF|ST71X_I2C_BUSY|ST71X_I2C_MSL|ST71X_I2C_ENDAD)
#define  ST71X_I2C_EVENT_MASTERBYTERECEIVED    (ST71X_I2C_EVF|ST71X_I2C_BUSY|ST71X_I2C_MSL|ST71X_I2C_BTF)
#define  ST71X_I2C_EVENT_MASTERBYTETRANSMITTED (ST71X_I2C_EVF|ST71X_I2C_BUSY|ST71X_I2C_MSL|ST71X_I2C_BTF|ST71X_I2C_TRA)
#define  ST71X_I2C_EVENT_MASTERMODEADDRESS10   (ST71X_I2C_EVF|ST71X_I2C_BUSY|ST71X_I2C_MSL|ST71X_I2C_ADD10)
#define  ST71X_I2C_EVENT_SLAVESTOPDETECTED      ST71X_I2C_STOPF
#define  ST71X_I2C_EVENT_SLAVEACKFAILURE       (ST71X_I2C_EVF|ST71X_I2C_BUSY|ST71X_I2C_BTF|ST71X_I2C_TRA|ST71X_I2C_AF)

#define  ST71X_I2C_BUSERRORDETECTED             ST71X_I2C_BERR
#define  ST71X_I2C_ARBITRATIONLOST              ST71X_I2C_ARLO
#define  ST71X_I2C_SLAVEGENERALCALL            (ST71X_I2C_BUSY|ST71X_I2C_GCAL)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_I2C_H */

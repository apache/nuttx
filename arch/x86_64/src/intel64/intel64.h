/****************************************************************************
 * arch/x86_64/src/intel64/intel64.h
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

#ifndef __ARCH_X86_64_SRC_INTEL64_INTEL64_H
#define __ARCH_X86_64_SRC_INTEL64_INTEL64_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "x86_64_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: vector_*
 *
 * Description:
 *   These are the various ISR/IRQ vector address exported from
 *   intel64_vectors.S.  These addresses need to have global scope
 *   so that they can be known to the interrupt initialization logic in
 *   intel64_irq.c.
 *
 ****************************************************************************/

void vector_isr0(void);
void vector_isr1(void);
void vector_isr2(void);
void vector_isr3(void);
void vector_isr4(void);
void vector_isr5(void);
void vector_isr6(void);
void vector_isr7(void);
void vector_isr8(void);
void vector_isr9(void);
void vector_isr10(void);
void vector_isr11(void);
void vector_isr12(void);
void vector_isr13(void);
void vector_isr14(void);
void vector_isr15(void);
void vector_isr16(void);
void vector_isr17(void);
void vector_isr18(void);
void vector_isr19(void);
void vector_isr20(void);
void vector_isr21(void);
void vector_isr22(void);
void vector_isr23(void);
void vector_isr24(void);
void vector_isr25(void);
void vector_isr26(void);
void vector_isr27(void);
void vector_isr28(void);
void vector_isr29(void);
void vector_isr30(void);
void vector_isr31(void);
void vector_irq0(void);
void vector_irq1(void);
void vector_irq2(void);
void vector_irq3(void);
void vector_irq4(void);
void vector_irq5(void);
void vector_irq6(void);
void vector_irq7(void);
void vector_irq8(void);
void vector_irq9(void);
void vector_irq10(void);
void vector_irq11(void);
void vector_irq12(void);
void vector_irq13(void);
void vector_irq14(void);
void vector_irq15(void);
void vector_irq16(void);
void vector_irq17(void);
void vector_irq18(void);
void vector_irq19(void);
void vector_irq20(void);
void vector_irq21(void);
void vector_irq22(void);
void vector_irq23(void);
void vector_irq24(void);
void vector_irq25(void);
void vector_irq26(void);
void vector_irq27(void);
void vector_irq28(void);
void vector_irq29(void);
void vector_irq30(void);
void vector_irq31(void);
void vector_irq32(void);
void vector_irq33(void);
void vector_irq34(void);
void vector_irq35(void);
void vector_irq36(void);
void vector_irq37(void);
void vector_irq38(void);
void vector_irq39(void);
void vector_irq40(void);
void vector_irq41(void);
void vector_irq42(void);
void vector_irq43(void);
void vector_irq44(void);
void vector_irq45(void);
void vector_irq46(void);
void vector_irq47(void);
void vector_irq48(void);
void vector_irq49(void);
void vector_irq50(void);
void vector_irq51(void);
void vector_irq52(void);
void vector_irq53(void);
void vector_irq54(void);
void vector_irq55(void);
void vector_irq56(void);
void vector_irq57(void);
void vector_irq58(void);
void vector_irq59(void);
void vector_irq60(void);
void vector_irq61(void);
void vector_irq62(void);
void vector_irq63(void);
void vector_irq64(void);
void vector_irq65(void);
void vector_irq66(void);
void vector_irq67(void);
void vector_irq68(void);
void vector_irq69(void);
void vector_irq70(void);
void vector_irq71(void);
void vector_irq72(void);
void vector_irq73(void);
void vector_irq74(void);
void vector_irq75(void);
void vector_irq76(void);
void vector_irq77(void);
void vector_irq78(void);
void vector_irq79(void);
void vector_irq80(void);
void vector_irq81(void);
void vector_irq82(void);
void vector_irq83(void);
void vector_irq84(void);
void vector_irq85(void);
void vector_irq86(void);
void vector_irq87(void);
void vector_irq88(void);
void vector_irq89(void);
void vector_irq90(void);
void vector_irq91(void);
void vector_irq92(void);
void vector_irq93(void);
void vector_irq94(void);
void vector_irq95(void);
void vector_irq96(void);
void vector_irq97(void);
void vector_irq98(void);
void vector_irq99(void);
void vector_irq100(void);
void vector_irq101(void);
void vector_irq102(void);
void vector_irq103(void);
void vector_irq104(void);
void vector_irq105(void);
void vector_irq106(void);
void vector_irq107(void);
void vector_irq108(void);
void vector_irq109(void);
void vector_irq110(void);
void vector_irq111(void);
void vector_irq112(void);
void vector_irq113(void);
void vector_irq114(void);
void vector_irq115(void);
void vector_irq116(void);
void vector_irq117(void);
void vector_irq118(void);
void vector_irq119(void);
void vector_irq120(void);
void vector_irq121(void);
void vector_irq122(void);
void vector_irq123(void);
void vector_irq124(void);
void vector_irq125(void);
void vector_irq126(void);
void vector_irq127(void);
void vector_irq128(void);
void vector_irq129(void);
void vector_irq130(void);
void vector_irq131(void);
void vector_irq132(void);
void vector_irq133(void);
void vector_irq134(void);
void vector_irq135(void);
void vector_irq136(void);
void vector_irq137(void);
void vector_irq138(void);
void vector_irq139(void);
void vector_irq140(void);
void vector_irq141(void);
void vector_irq142(void);
void vector_irq143(void);
void vector_irq144(void);
void vector_irq145(void);
void vector_irq146(void);
void vector_irq147(void);
void vector_irq148(void);
void vector_irq149(void);
void vector_irq150(void);
void vector_irq151(void);
void vector_irq152(void);
void vector_irq153(void);
void vector_irq154(void);
void vector_irq155(void);
void vector_irq156(void);
void vector_irq157(void);
void vector_irq158(void);
void vector_irq159(void);
void vector_irq160(void);
void vector_irq161(void);
void vector_irq162(void);
void vector_irq163(void);
void vector_irq164(void);
void vector_irq165(void);
void vector_irq166(void);
void vector_irq167(void);
void vector_irq168(void);
void vector_irq169(void);
void vector_irq170(void);
void vector_irq171(void);
void vector_irq172(void);
void vector_irq173(void);
void vector_irq174(void);
void vector_irq175(void);
void vector_irq176(void);
void vector_irq177(void);
void vector_irq178(void);
void vector_irq179(void);
void vector_irq180(void);
void vector_irq181(void);
void vector_irq182(void);
void vector_irq183(void);
void vector_irq184(void);
void vector_irq185(void);
void vector_irq186(void);
void vector_irq187(void);
void vector_irq188(void);
void vector_irq189(void);
void vector_irq190(void);
void vector_irq191(void);
void vector_irq192(void);
void vector_irq193(void);
void vector_irq194(void);
void vector_irq195(void);
void vector_irq196(void);
void vector_irq197(void);
void vector_irq198(void);
void vector_irq199(void);
void vector_irq200(void);
void vector_irq201(void);
void vector_irq202(void);
void vector_irq203(void);
void vector_irq204(void);
void vector_irq205(void);
void vector_irq206(void);
void vector_irq207(void);
void vector_irq208(void);
void vector_irq209(void);
void vector_irq210(void);
void vector_irq211(void);
void vector_irq212(void);
void vector_irq213(void);
void vector_irq214(void);
void vector_irq215(void);
void vector_irq216(void);
void vector_irq217(void);
void vector_irq218(void);
void vector_irq219(void);
void vector_irq220(void);
void vector_irq221(void);
void vector_irq222(void);
void vector_irq223(void);
void vector_irq224(void);
void vector_irq225(void);
void vector_irq226(void);
void vector_irq227(void);
void vector_irq228(void);
void vector_irq229(void);
void vector_irq230(void);
void vector_irq231(void);
void vector_irq232(void);
void vector_irq233(void);
void vector_irq234(void);
void vector_irq235(void);
void vector_irq236(void);
void vector_irq237(void);
void vector_irq238(void);
void vector_irq239(void);
void vector_irq240(void);
void vector_irq241(void);
void vector_irq242(void);
void vector_irq243(void);
void vector_irq244(void);
void vector_irq245(void);
void vector_irq246(void);
void vector_irq247(void);
void vector_irq248(void);
void vector_irq249(void);
void vector_irq250(void);
void vector_irq251(void);
void vector_irq252(void);
void vector_irq253(void);
void vector_irq254(void);
void vector_irq255(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_64_SRC_INTEL64_INTEL64_H */

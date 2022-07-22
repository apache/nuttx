/****************************************************************************
 * arch/arm/src/s32k3xx/startup.S
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

/* Copyright 2022 NXP */

    .syntax unified
    .arch armv7-m
	.text
	.global	__start
	.type	__start, function
__start:
    cpsid i
    mov   r0, #0
    mov   r1, #0
    mov   r2, #0
    mov   r3, #0
    mov   r4, #0
    mov   r5, #0
    mov   r6, #0
    mov   r7, #0

    /* Initialize SRAM ECC */
    ldr r1, =SRAM_BASE_ADDR
    ldr r2, =SRAM_END_ADDR

    subs    r2, r1
    subs    r2, #1
    ble SRAM_LOOP_END

    movs    r0, 0
    movs    r3, 0
SRAM_LOOP:
    stm r1!, {r0,r3}
    subs r2, 8
    bge SRAM_LOOP
SRAM_LOOP_END:
    b s32k3xx_start

.align 4
.ltorg

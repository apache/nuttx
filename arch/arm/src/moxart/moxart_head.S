/****************************************************************************
 * arch/arm/src/moxart/moxart_head.S
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

/* Place a branch to the real head at the entry point */

.section	.text.start
	b	__start

/* Exception Vectors like they are needed for the exception vector
 * indirection of the internal boot ROM.  The following section must be
 * linked to appear at 0x80001c
 */

.section	.text.exceptions
_undef_instr:
	b	arm_vectorundefinsn
_sw_interr:
	b	arm_vectorsvc
_prefetch_abort:
	b	arm_vectorprefetch
_data_abort:
	b	arm_vectordata
_reserved:
	b	_reserved
_irq:
	b	arm_vectorirq
_fiq:
	b	arm_vectorfiq

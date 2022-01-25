/****************************************************************************
 * arch/ceva/src/xc5/cpm.S
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

	.file		"cpm.S"

	.text
	.public		_getcpm
	.func_start	2 _getcpm
_getcpm:
	push {dw}	r0
	mov		a0, r0
	nop
	in {dw,cpm}	(r0), r0
	nop
	nop
	nop
	mov		r0, a0
	pop {dw}	r0
	ret
	.func_end	2 _getcpm

	.public		_putcpm
	.func_start	2 _putcpm
_putcpm:
	push {dw}	r0
	push {dw}	r1
	mov		a0, r0
	mov		a1, r1
	nop
	out {dw,cpm}	r1, (r0)
	pop {dw}	r1
	pop {dw}	r0
	ret
	.func_end	2 _putcpm

	.public		_modifycpm
	.func_start	2 _modifycpm
_modifycpm:
	push {dw}	r0
	push {dw}	r1
	mov		a0, r0
	nop
	in {dw,cpm}	(r0), r1
	nop
	nop
	nop
	mov		r1, a0
	not		a1, a1
	and		a0, a1, a0
	or		a0, a2, a0
	nop
	mov		a0, r1
	nop
	out {dw,cpm}	r1, (r0)
	pop {dw}	r1
	pop {dw}	r0
	ret
	.func_end	2 _modifycpm

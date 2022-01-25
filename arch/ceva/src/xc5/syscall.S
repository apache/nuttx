/****************************************************************************
 * arch/ceva/src/xc5/syscall.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file		"syscall.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sys_callx
 *
 * Description:
 *   This function generate the trap exception with the specified number of
 *   parameters.
 *
 *     a0 = SYS_ call number
 *     a1 = parm0
 *     a2 = parm1
 *     a3 = parm2
 *     a4 = parm3
 *     a5 = parm4
 *     a6 = parm5
 *     a7 = parm6
 *
 ****************************************************************************/

	.text

	.public		_sys_call0
	.public		_sys_call1
	.public		_sys_call2
	.public		_sys_call3
	.public		_sys_call4
	.public		_sys_call5
	.public		_sys_call6

	.func_start	2 _sys_call0

_sys_call0:
_sys_call1:
_sys_call2:
_sys_call3:
_sys_call4:
_sys_call5:
_sys_call6:
	trap0
	nop
	ret

	.func_end	2 _sys_call0

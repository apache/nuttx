/****************************************************************************
 * arch/xtensa/src/esp32/esp32_cpuindex.S
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

	.file	"xtensa_cpumacros.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/xtensa/xtensa_abi.h>
#include "chip_macros.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return the real core number regardless CONFIG_SMP setting
 *
 ****************************************************************************/

	.text
	.align	4
	.global	up_cpu_index
	.type	up_cpu_index, @function

up_cpu_index:
	ENTRY(16)
	getcoreid	a2
	RET(16)

	.size	up_cpu_index, . - up_cpu_index

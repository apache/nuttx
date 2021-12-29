/****************************************************************************
 * boards/arm/samv7/common/include/board_progmem.h
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

#ifndef __BOARDS_ARM_SAMV7_COMMON_INCLUDE_BOARD_PROGMEM_H
#define __BOARDS_ARM_SAMV7_COMMON_INCLUDE_BOARD_PROGMEM_H

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: board_progmem_init
 *
 * Description:
 *   Initialize the FLASH and register MTD devices.
 *
 * Input Parameters:
 *   minor - The starting minor number for progmem MTD partitions.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_progmem_init(int minor);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMV7_COMMON_INCLUDE_BOARD_PROGMEM_H */

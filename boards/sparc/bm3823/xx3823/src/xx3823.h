/****************************************************************************
 * boards/sparc/bm3823/xx3823/src/xx3823.h
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

#ifndef __BOARDS_SPARC_BM3823_XX3823_SRC_XX3823_H
#define __BOARDS_SPARC_BM3823_XX3823_SRC_XX3823_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bm3823_ledinit
 *
 * Description:
 *   Configure on-board LEDs if LED support has been selected.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void bm3823_led_initialize(void);
#endif

/****************************************************************************
 * Name: bm3823_am29lv_initialize
 *
 * Description:
 *   Initialize and register the AM29LV FLASH file system.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_AM29LV
int bm3823_am29lv_initialize(int minor);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_SPARC_BM3823_XX3823_SRC_XX3823_H */

/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_altmdm.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ALTMDM_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ALTMDM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Name: board_altmdm_initialize
 *
 * Description:
 *   Initialize Altair modem
 *
 ****************************************************************************/

int board_altmdm_initialize(FAR const char *devpath);

/****************************************************************************
 * Name: board_altmdm_uninitialize
 *
 * Description:
 *   Uninitialize Altair modem
 *
 ****************************************************************************/

int board_altmdm_uninitialize(void);

/****************************************************************************
 * Name: board_altmdm_poweron
 *
 * Description:
 *   Power on the Altair modem device on the board.
 *
 ****************************************************************************/

void board_altmdm_poweron(void);

/****************************************************************************
 * Name: board_altmdm_poweroff
 *
 * Description:
 *   Power off the Altair modem device on the board.
 *
 ****************************************************************************/

void board_altmdm_poweroff(void);

#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ALTMDM_H */

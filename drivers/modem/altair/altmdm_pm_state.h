/****************************************************************************
 * drivers/modem/altair/altmdm_pm_state.h
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

#ifndef __DRIVERS_MODEM_ALTAIR_ALTMDM_PM_STATE_H
#define __DRIVERS_MODEM_ALTAIR_ALTMDM_PM_STATE_H

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MODEM_PM_INTERNAL_STATE_SLEEP          (0)
#define MODEM_PM_INTERNAL_STATE_GOING_TO_WAKE  (1)
#define MODEM_PM_INTERNAL_STATE_WAKE           (2)
#define MODEM_PM_INTERNAL_STATE_GOING_TO_SLEEP (3)
#define MODEM_PM_INTERNAL_STATE_MAX            (4)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_pm_getstate
 *
 * Description:
 *   Get current modem state.
 *
 ****************************************************************************/

uint32_t altmdm_pm_getstate(void);

/****************************************************************************
 * Name: altmdm_pm_getinternalstate
 *
 * Description:
 *   Get internal modem state.
 *
 ****************************************************************************/

uint32_t altmdm_pm_getinternalstate(void);

/****************************************************************************
 * Name: altmdm_pm_setinternalstate
 *
 * Description:
 *   Set internal modem state.
 *
 ****************************************************************************/

void altmdm_pm_setinternalstate(uint32_t state);

#endif
#endif /* __DRIVERS_MODEM_ALTAIR_ALTMDM_PM_STATE_H */

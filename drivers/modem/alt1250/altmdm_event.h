/****************************************************************************
 * drivers/modem/alt1250/altmdm_event.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTMDM_EVENT_H
#define __DRIVERS_MODEM_ALT1250_ALTMDM_EVENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct altmdm_event_s
{
  sem_t sem;
  uint32_t event;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_event_init
 *
 * Description:
 *   Initialize the event flag.
 *
 * Input Parameters:
 *   evt     - An instance pointer to the event flag.
 *
 * Returned Value:
 *   Returns 0 on success.
 *   When an error occurs, a negative value is returned.
 *
 ****************************************************************************/

int altmdm_event_init(FAR struct altmdm_event_s *evt);

/****************************************************************************
 * Name: altmdm_event_destroy
 *
 * Description:
 *   Destroy the event flag.
 *
 * Input Parameters:
 *   evt     - An instance pointer to the event flag.
 *
 * Returned Value:
 *   Returns 0 on success.
 *   When an error occurs, a negative value is returned.
 *
 ****************************************************************************/

int altmdm_event_destroy(FAR struct altmdm_event_s *evt);

/****************************************************************************
 * Name: altmdm_event_wait
 *
 * Description:
 *   Wait for the specified event.
 *
 * Input Parameters:
 *   evt        - An instance pointer to the event flag.
 *   event      - A bitmap that means an event to wait for.
 *   with_clear - A flag to clear the event flag when an event is received.
 *   timeout_ms - Time-out value to wait for an event. 0 means infinite wait.
 *
 * Returned Value:
 *   Returns the received event. When an error occurs,
 *   a negative value is returned.
 *
 ****************************************************************************/

uint32_t altmdm_event_wait(FAR struct altmdm_event_s *evt,
  uint32_t event, bool with_clear, int timeout_ms);

/****************************************************************************
 * Name: altmdm_event_set
 *
 * Description:
 *   Send event.
 *
 * Input Parameters:
 *   evt        - An instance pointer to the event flag.
 *   event      - Bitmap of the event to send.
 *
 * Returned Value:
 *   Returns 0 on success.
 *   When an error occurs, a negative value is returned.
 *
 ****************************************************************************/

int altmdm_event_set(FAR struct altmdm_event_s *evt, uint32_t event);

/****************************************************************************
 * Name: altmdm_event_clear
 *
 * Description:
 *   Clear event.
 *
 * Input Parameters:
 *   evt        - An instance pointer to the event flag.
 *   event      - Bitmap of the event to clear.
 *
 * Returned Value:
 *   Returns 0 on success.
 *   When an error occurs, a negative value is returned.
 *
 ****************************************************************************/

int altmdm_event_clear(FAR struct altmdm_event_s *evt, uint32_t event);

/****************************************************************************
 * Name: altmdm_event_refer
 *
 * Description:
 *   Get which event is being received.
 *
 * Input Parameters:
 *   evt        - An instance pointer to the event flag.
 *
 * Returned Value:
 *   Returns the received event. When an error occurs,
 *   a negative value is returned.
 *
 ****************************************************************************/

uint32_t altmdm_event_refer(FAR struct altmdm_event_s *evt);

#endif  /* __DRIVERS_MODEM_ALT1250_ALTMDM_EVENT_H */

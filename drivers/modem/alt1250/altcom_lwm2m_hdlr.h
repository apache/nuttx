/****************************************************************************
 * drivers/modem/alt1250/altcom_lwm2m_hdlr.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTCOM_LWM2M_HDLR_H
#define __DRIVERS_MODEM_ALT1250_ALTCOM_LWM2M_HDLR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef CODE int32_t (*lwm2mstub_hndl_t)(FAR uint8_t *, size_t,
                          FAR void **, size_t);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lwm2mstub_get_handler
 *
 * Description:
 *   Get handler for lwm2m stub.
 *
 * Input Parameters:
 *   pktbuf   - Pointer to the received packet.
 *   pktsz    - Pointer to the data size after parsing the received packet.
 *   lcmdid   - Pointer to a variable that stores the lwm2m LAPI command ID.
 *
 * Returned Value:
 *   If a lwm2m handler associated with the received packet is found, the
 *   function pointer of the handler is returned. If not found, returns NULL.
 *
 ****************************************************************************/

lwm2mstub_hndl_t lwm2mstub_get_handler(FAR uint8_t **pktbuf,
                                       FAR size_t *pktsz,
                                       uint32_t *lcmdid);

#endif  /* __DRIVERS_MODEM_ALT1250_ALTCOM_LWM2M_HDLR_H */

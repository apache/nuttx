/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTCOM_HDLR_H
#define __DRIVERS_MODEM_ALT1250_ALTCOM_HDLR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/modem/alt1250.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: alt1250_composehdlr
 *
 * Description:
 *   Get the function pointer of the compose handler associated with the
 *   LAPI command ID.
 *
 * Input Parameters:
 *   cmdid   - ID of the LAPI command.
 *
 * Returned Value:
 *   Returns a function pointer for the compose handler. If there is no
 *   compose handler associated with the LAPI command ID, NULL is returned.
 *
 ****************************************************************************/

compose_handler_t alt1250_composehdlr(uint32_t cmdid);

/****************************************************************************
 * Name: alt1250_parsehdlr
 *
 * Description:
 *   Get the function pointer of the parse handler associated with the
 *   ALTCOM command ID.
 *
 * Input Parameters:
 *   altcid   - ID of the ALTCOM command.
 *   altver   - Version of the ALTCOM command.
 *
 * Returned Value:
 *   Returns a function pointer for the parse handler. If there is no
 *   parse handler associated with the ALTCOM command ID, NULL is returned.
 *
 ****************************************************************************/

parse_handler_t alt1250_parsehdlr(uint16_t altcid, uint8_t altver);

#endif  /* __DRIVERS_MODEM_ALT1250_ALTCOM_HDLR_H */

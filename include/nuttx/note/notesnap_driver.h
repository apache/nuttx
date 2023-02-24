/****************************************************************************
 * include/nuttx/note/notesnap_driver.h
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

#ifndef __INCLUDE_NUTTX_NOTESNAP_DRIVER_H
#define __INCLUDE_NUTTX_NOTESNAP_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/streams.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_NOTESNAP

/****************************************************************************
 * Name: notesnap_register
 ****************************************************************************/

int notesnap_register(void);

/****************************************************************************
 * Name: notesnap_dump_with_stream
 ****************************************************************************/

void notesnap_dump_with_stream(FAR struct lib_outstream_s *stream);

/****************************************************************************
 * Name: notesnap_dump
 ****************************************************************************/

void notesnap_dump(void);

#endif
#endif /* __INCLUDE_NUTTX_NOTESNAP_DRIVER_H */

/****************************************************************************
 * include/nuttx/note/notestream_driver.h
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

#ifndef __INCLUDE_NUTTX_NOTE_NOTESTREAM_DRIVER_H
#define __INCLUDE_NUTTX_NOTE_NOTESTREAM_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/note/note_driver.h>
#include <nuttx/streams.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct notestream_driver_s
{
  struct note_driver_s driver;
  struct lib_outstream_s *stream;
};

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_NOTELOWEROUT
extern struct notestream_driver_s g_notestream_lowerout;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: note_driver_unregister
 ****************************************************************************/
#ifdef CONFIG_DRIVERS_NOTEFILE
int notefile_register(FAR const char *filename);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NOTE_NOTESTREAM_DRIVER_H */

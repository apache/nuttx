/****************************************************************************
 * include/nuttx/aie/ai_engine.h
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

#ifndef __INCLUDE_NUTTX_AIE_AI_ENGINE_H
#define __INCLUDE_NUTTX_AIE_AI_ENGINE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stddef.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This enumeration type indicates work of aie_ioctl. */

enum aie_cmd_e
{
  AIE_CMD_LOAD = 0,
  AIE_CMD_FEED_INPUT,
  AIE_CMD_GET_OUTPUT
};

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct aie_lowerhalf_s;
struct aie_ops_s
{
  CODE int (*init)(FAR struct aie_lowerhalf_s *lower, uintptr_t model);

  CODE int (*deinit)(FAR struct aie_lowerhalf_s *lower, int id);

  CODE int (*feed_input)(FAR struct aie_lowerhalf_s *lower, int id,
                         uintptr_t input);

  CODE int (*get_output)(FAR struct aie_lowerhalf_s *lower, int id,
                         uintptr_t output);

  CODE int (*control)(FAR struct aie_lowerhalf_s *lower, int id,
                      int cmd, unsigned long arg);
};

/* This structure provides the publicly visible representation of the
 * "lower-half" driver state structure.  "lower half" drivers will have an
 * internal structure definition that will be cast-compatible with this
 * structure definitions.
 */

struct aie_lowerhalf_s
{
  /* The heading fields of this struct must be as follow. */

  FAR const struct aie_ops_s *ops;

  /* The custom AI Engine may include additional fields after here. */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ai_engine_register
 *
 * Description:
 *   Register all ai engine related drivers.
 *
 ****************************************************************************/

int aie_register(FAR const char *path,
                 FAR struct aie_lowerhalf_s *lower);

#endif /* __INCLUDE_NUTTX_AIE_AI_ENGINE_H */

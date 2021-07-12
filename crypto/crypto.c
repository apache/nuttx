/****************************************************************************
 * crypto/crypto.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/crypto/crypto.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int up_cryptoinitialize(void)
{
#ifdef CONFIG_CRYPTO_ALGTEST
  int ret;

#ifdef CONFIG_CRYPTO_ALGTEST
  ret = crypto_test();
  if (ret)
    {
      crypterr("ERROR: crypto test failed\n");
    }
  else
    {
      cryptinfo("crypto test OK\n");
    }
#endif

  return ret;
#else
  return OK;
#endif
}

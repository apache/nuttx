/****************************************************************************
 * libs/libc/pthread/pthread_keydelete.c
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

#include <pthread.h>

#include <nuttx/tls.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_key_delete
 *
 * Description:
 *   This POSIX function deletes a thread-specific data key
 *   previously returned by pthread_key_create().
 *
 * Input Parameters:
 *   key - the key to delete
 *
 * Returned Value:
 *   Returns zero (OK) on success.  EINVAL may be returned if an invalid
 *   key is received.
 *
 * POSIX Compatibility:
 *
 ****************************************************************************/

int pthread_key_delete(pthread_key_t key)
{
  /* Free the TLS index */

  int ret = tls_free((int)key);
  return ret < 0 ? -ret : 0;
}

/****************************************************************************
 * libs/libc/unistd/lib_sync.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sync
 *
 * Description:
 *   sync() causes all pending modifications to filesystem metadata and
 *   cached file data to be written to the underlying filesystems.
 *
 * Returned Value:
 *   sync() is always successful.
 *
 * Assumptions:
 *
 ****************************************************************************/

void sync(void)
{
}

/****************************************************************************
 * Name: syncfs
 *
 * Description:
 *   syncfs() is like sync(), but synchronizes just the filesystem
 *   containing file referred to by the open file descriptor fd.
 *
 * Returned Value:
 *   syncfs() returns 0 on success; on error, it returns -1 and sets
 *   errno to indicate the error.
 *
 * Assumptions:
 *
 ****************************************************************************/

int syncfs(int fd)
{
  return 0;
}

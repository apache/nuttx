/****************************************************************************
 * libs/libc/unistd/lib_pathconf.c
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

#include <unistd.h>
#include <limits.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fpathconf/pathconf
 *
 * Description:
 *   The fpathconf() and pathconf() functions shall determine the current
 *   value of a configurable limit or option (variable) that is associated
 *   with a file or directory.
 *
 *   For pathconf(), the path argument points to the pathname of a file or
 *   directory.
 *
 *   For fpathconf(), the fildes argument is an open file descriptor.
 *
 *   The name argument represents the variable to be queried relative to that
 *   file or directory. Implementations shall support all of the variables
 *   listed in the following table and may support others. The variables in
 *   the following table come from <limits.h> or <unistd.h> and the symbolic
 *   constants, defined in <unistd.h>, are the corresponding values used for
 *   name.
 *
 *     Variable                           Value of Name
 *
 *     {FILESIZEBITS}                     _PC_FILESIZEBITS
 *     {LINK_MAX}                         _PC_LINK_MAX
 *     {MAX_CANON}                        _PC_MAX_CANON
 *     {MAX_INPUT}                        _PC_MAX_INPUT
 *     {NAME_MAX}                         _PC_NAME_MAX
 *     {PATH_MAX}                         _PC_PATH_MAX
 *     {PIPE_BUF}                         _PC_PIPE_BUF
 *     {POSIX2_SYMLINKS}                  _PC_2_SYMLINKS
 *     {POSIX_ALLOC_SIZE_MIN}             _PC_ALLOC_SIZE_MIN
 *     {POSIX_REC_INCR_XFER_SIZE}         _PC_REC_INCR_XFER_SIZE
 *     {POSIX_REC_MAX_XFER_SIZE}          _PC_REC_MAX_XFER_SIZE
 *     {POSIX_REC_MIN_XFER_SIZE}          _PC_REC_MIN_XFER_SIZE
 *     {POSIX_REC_XFER_ALIGN}             _PC_REC_XFER_ALIGN
 *     {SYMLINK_MAX}                      _PC_SYMLINK_MAX
 *     _POSIX_CHOWN_RESTRICTED            _PC_CHOWN_RESTRICTED
 *     _POSIX_NO_TRUNC                    _PC_NO_TRUNC
 *     _POSIX_VDISABLE                    _PC_VDISABLE
 *     _POSIX_ASYNC_IO                    _PC_ASYNC_IO
 *     _POSIX_PRIO_IO                     _PC_PRIO_IO
 *     _POSIX_SYNC_IO                     _PC_SYNC_IO
 *
 * Returned Value:
 *   If name is an invalid value, both pathconf() and fpathconf() shall
 *   return -1 and set errno to indicate the error.
 *
 *   If the variable corresponding to name has no limit for the path or file
 *   descriptor, both pathconf() and fpathconf() shall return -1 without
 *   changing errno. If the implementation needs to use path to determine the
 *   value of name and the implementation does not support the association of
 *   name with the file specified by path, or if the process did not have
 *   appropriate privileges to query the file specified by path, or path does
 *   not exist, pathconf() shall return -1 and set errno to indicate the
 *   error.
 *
 *   If the implementation needs to use fildes to determine the value of name
 *   and the implementation does not support the association of name with the
 *   file specified by fildes, or if fildes is an invalid file descriptor,
 *   fpathconf() shall return -1 and set errno to indicate the error.
 *
 *   Otherwise, pathconf() or fpathconf() shall return the current variable
 *   value for the file or directory without changing errno. The value
 *   returned shall not be more restrictive than the corresponding value
 *   available to the application when it was compiled with the
 *   implementation's <limits.h> or <unistd.h>.
 *
 *   If the variable corresponding to name is dependent on an unsupported
 *   option, the results are unspecified.
 *
 ****************************************************************************/

long fpathconf(int fildes, int name)
{
  /* NOTE:  The initialize implementation of this interface is very sparse.
   * It was originally created to support only the functionality of libcxx
   * but can be extended to support as much of the standard POSIX as is
   * necessary.
   */

  UNUSED(fildes);

  switch (name)
    {
      case _PC_PATH_MAX:
        return PATH_MAX;

      case _PC_LINK_MAX:
        return _POSIX_LINK_MAX;

      case _PC_NAME_MAX:
        return _POSIX_NAME_MAX;

      case _PC_PIPE_BUF:
        return _POSIX_PIPE_BUF;

      case _PC_MAX_CANON:
        return _POSIX_MAX_CANON;

      case _PC_MAX_INPUT:
        return _POSIX_MAX_INPUT;

      default:

        /* Assume valid but not implemented for the time being */

        set_errno(ENOSYS);
        return ERROR;
    }
}

long pathconf(FAR const char *path, int name)
{
  UNUSED(path);

  return fpathconf(-1, name);
}

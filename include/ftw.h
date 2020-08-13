/****************************************************************************
 * include/ftw.h
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

#ifndef __INCLUDE_FTW_H
#define __INCLUDE_FTW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* The <ftw.h> header defines the stat structure and the symbolic names for
 * st_mode and the file type test macros as described in sys/stat.h.
 *
 * Inclusion of the <ftw.h> header may also make visible all symbols from
 * sys/stat.h.
 */

#include <sys/stat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros for use as values of the third argument to the application-supplied
 * function that is passed as the second argument to ftw() and nftw():
 */

#define FTW_F      0  /* File */
#define FTW_D      1  /* Directory */
#define FTW_DNR    2  /* Directory without read permission */
#define FTW_DP     3  /* Directory with subdirectories visited */
#define FTW_NS     4  /* Unknown type; stat() failed */
#define FTW_SL     5  /* Symbolic link */
#define FTW_SLN    6  /* Symbolic link that names a nonexistent file */

/* Macros for use as values of the fourth argument to nftw() */

#define FTW_PHYS   1  /* Physical walk, does not follow symbolic links.
                       * Otherwise, nftw() follows links but does not walk
                       * down any path that crosses itself. */
#define FTW_MOUNT  2  /* The walk does not cross a mount point. */
#define FTW_DEPTH  4  /* All subdirectories are visited before the directory
                       * itself. */
#ifndef CONFIG_DISABLE_ENVIRON
#define FTW_CHDIR  8  /* The walk changes to each directory before reading
                       * it.  */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The fourth argument of the ftw/nftw callback is a pointer to an FTW
 * structure.  The value of base is the offset of the object's filename in
 * the pathname passed as the first argument to the callback.  The value of
 * level indicates depth relative to the root of the walk, where the root
 * level is 0.
 */

struct FTW
{
  int base;    /* Offset of object's filename in the pathname */
  int level;   /* Depth relative to the root of the walk */
};

/* This is the type of the ftw callback */

typedef int (*ftw_cb_t)(FAR const char *path, FAR const struct stat *buf,
                        int info);

/* This is the type of the nftw callback */

typedef int (*nftw_cb_t)(FAR const char *path, FAR const struct stat *buf,
                         int info, FAR struct FTW *pftw);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ftw
 *
 * Description:
 *   The ftw() function will recursively descend the directory hierarchy
 *   rooted in 'path'. For each object in the hierarchy, ftw() will call
 *   the function pointed to by 'fn', passing it a pointer to a null-
 *   terminated character string containing the name of the object, a
 *   pointer to a stat structure containing information about the object,
 *   and an integer that characterizes object.
 *
 *   The ftw() function will visit a directory before visiting any of its
 *   descendants.
 *
 *   The ftw() function will use at most one file descriptor for each level
 *   in the tree.
 *
 *   The tree traversal will continue until either the tree is exhausted, an
 *   invocation of 'fn' returns a non-zero value, or some error, other than
 *   EACCES, is detected within ftw().
 *
 *   When ftw() returns it will close any directory streams and file
 *   descriptors it uses not counting any opened by the application-supplied
 *   'fn' function.
 *
 *   The results are unspecified if the application-supplied 'fn' function
 *   does not preserve the current working directory.
 *
 *   The ftw() function need not be reentrant. A function that is not
 *   required to be reentrant is not required to be thread-safe.
 *
 * Input Parameters:
 *   path    - The 'root' of the directory hierarchy to descend
 *   fn      - The callback function to be invoked as each object in the
 *             heirarchy is encountered.
 *   fdlimit - The fdlimit argument specifies the maximum number of directory
 *             streams or file descriptors or both available for use by ftw()
 *             while traversing the tree.The maximum depth of the directories
 *             to visit.  The fdlimit argument should be in the range [1,
 *             {OPEN_MAX}].
 *
 * Returned Value:
 *   If the tree is exhausted, ftw() will return 0. If the function pointed
 *   to by fn returns a non-zero value, ftw() will stop its tree traversal
 *   and return whatever value was returned by the function pointed to by
 *   'fn'. If ftw() detects an error, it will return -1 and set errno to
 *   indicate the error.
 *
 *   If ftw() encounters an error other than EACCES (see FTW_DNR and FTW_NS),
 *   it will return -1 and set errno to indicate the error.  The external
 *   variable errno may contain any error value that is possible when a
 *   directory is opened or when one of the stat functions is executed on a
 *   directory or file.
 *
 ****************************************************************************/

int ftw(FAR const char *path, ftw_cb_t fn, int fdlimit);

/****************************************************************************
 * Name: nftw
 *
 * Description:
 *   The nftw() function will recursively descend the directory hierarchy
 *   rooted in 'path'.  The nftw() function has a similar effect to ftw()
 *   except that it takes an additional argument 'flags'
 *
 * Input Parameters:
 *
 *   path    - The 'root' of the directory hierarchy to descend
 *   fn      - The callback function to be invoked as each object in the
 *             heirarchy is encountered.
 *   fdlimit - The fdlimit argument specifies the maximum number of directory
 *             streams or file descriptors or both available for use by
 *             nftw() while traversing the tree.The maximum depth of the
 *             directories to visit.  The fdlimit argument should be in the
 *             range [1, {OPEN_MAX}].
 *   flags  - A bitwise-inclusive OR of zero or more of the following flags:
 *
 *     FTW_CHDIR
 *       If set, nftw() will change the current working directory to each
 *       directory as it reports files in that directory.  If clear, nftw()
 *       will not change the current working directory.
 *     FTW_DEPTH
 *       If set, nftw() will report all files in a directory before
 *       reporting the directory itself. If clear, nftw() will report any
 *       directory before reporting the files in that directory.
 *     FTW_MOUNT
 *       If set, nftw() will only report files in the same file system as
 *       path. If clear, nftw() will report all files encountered during
 *       the walk.
 *     FTW_PHYS
 *       If set, nftw() will perform a physical walk and will not follow
 *       symbolic links.
 *
 * Returned Value:
 *   If the tree is exhausted, nftw() will return 0. If the function pointed
 *   to by fn returns a non-zero value, nftw() will stop its tree traversal
 *   and return whatever value was returned by the function pointed to by
 *   'fn'. If nftw() detects an error, it will return -1 and set errno to
 *   indicate the error.
 *
 *   If nftw() encounters an error other than EACCES (see FTW_DNR and
 *   FTW_NS), it will return -1 and set errno to indicate the error.  The
 *   external variable errno may contain any error value that is possible
 *   when a directory is opened or when one of the stat functions is
 *   executed on a directory or file.
 *
 *
 ****************************************************************************/

int nftw(FAR const char *path, nftw_cb_t fn, int fdlimit, int flags);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_FTW_H */

/****************************************************************************
 * fs/vfs/fs_lock.c
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

#include <fcntl.h>
#include <errno.h>
#include <search.h>
#include <unistd.h>
#include <sys/stat.h>

#include <nuttx/lib/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/list.h>

#include "lock.h"
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_FS_LARGEFILE
#  define OFFSET_MAX INT64_MAX
#else
#  define OFFSET_MAX INT32_MAX
#endif

#define l_end l_len

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct file_lock_s
{
  struct flock     fl_lock;  /* File lock related information */
  FAR struct file  *fl_file; /* Identifies the file descriptor information
                              * held by the caller
                              */
  struct list_node fl_node;  /* Used to manage each filelock by means of a
                              * chained list.
                              */
};

struct file_lock_bucket_s
{
  struct list_node list;         /* Manage a chained list for each
                                  * filelock
                                  */
  sem_t            wait;         /* Blocking lock, called when SETLKW is
                                  * called and there is a conflict.
                                  */
  size_t           nwaiter;      /* Indicates how many blocking locks are
                                  * currently blocked.
                                  */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct hsearch_data g_file_lock_table;
static mutex_t g_protect_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_lock_get_path
 ****************************************************************************/

static int file_lock_get_path(FAR struct file *filep, FAR char *path)
{
  FAR struct tcb_s *tcb = this_task();

  /* We only apply file lock on mount points (f_inode won't be NULL). */

  if (!INODE_IS_MOUNTPT(filep->f_inode) ||
      tcb->flags & TCB_FLAG_SIGNAL_ACTION)
    {
      return -EBADF;
    }

  return file_fcntl(filep, F_GETPATH, path);
}

/****************************************************************************
 * Name: file_lock_normalize
 ****************************************************************************/

static int file_lock_normalize(FAR struct file *filep,
                               FAR struct flock *flock,
                               FAR struct flock *out)
{
  off_t start;
  off_t end;

  /* Check that the type brought in the flock is correct */

  switch (flock->l_type)
    {
      case F_RDLCK:
      case F_WRLCK:
      case F_UNLCK:
        break;
      default:
        return -EINVAL;
    }

  /* Converts and saves flock information */

  switch (flock->l_whence)
    {
      case SEEK_SET:
        {
          start = 0;
        }

        break;
      case SEEK_CUR:
        {
          start = filep->f_pos;
        }

        break;
      case SEEK_END:
        {
          struct stat st;
          int ret;

          ret = file_fstat(filep, &st);
          if (ret < 0)
            {
              return ret;
            }

          start = st.st_size;
        }

        break;
      default:
        return -EINVAL;
    }

  /* Check for overflow in converted flock */

  if (flock->l_start > OFFSET_MAX - start)
    {
      return -EOVERFLOW;
    }

  start += flock->l_start;
  if (start < 0)
    {
      return -EINVAL;
    }

  if (flock->l_len > 0)
    {
      if (flock->l_len - 1 > OFFSET_MAX - start)
        {
          return -EOVERFLOW;
        }

      end = start + flock->l_len - 1;
    }
  else if (flock->l_len < 0)
    {
      if (start + flock->l_len < 0)
        {
          return -EINVAL;
        }

      end = start - 1;
      start += flock->l_len;
    }
  else
    {
      end = OFFSET_MAX;
    }

  out->l_whence = SEEK_SET;
  out->l_type = flock->l_type;
  out->l_start = start;
  out->l_end = end;

  return OK;
}

/****************************************************************************
 * Name: file_lock_delete
 ****************************************************************************/

static void file_lock_delete(FAR struct file_lock_s *file_lock)
{
  list_delete(&file_lock->fl_node);
  kmm_free(file_lock);
}

/****************************************************************************
 * Name: file_lock_delete_bucket
 ****************************************************************************/

static void file_lock_delete_bucket(FAR struct file_lock_bucket_s *bucket,
                                    FAR const char *filepath)
{
  ENTRY item;

  /* If there is still a lock on the chain table at this point, it means
   * that there is still someone else holding it, so it doesn't need to be
   * released
   */

  if (list_is_empty(&bucket->list))
    {
      /* At this point, the file has no lock information context, so we can
       * remove it from the hash table, and the return result is 0 or 1 means
       * that the node does not exist, so we do not need to care about the
       * final return results
       */

      item.key = (FAR char *)filepath;
      hsearch_r(item, DELETE, NULL, &g_file_lock_table);
    }
}

/****************************************************************************
 * Name: file_lock_is_conflict
 ****************************************************************************/

static bool file_lock_is_conflict(FAR struct flock *request,
                                  FAR struct flock *internal)
{
  /* If the request is not exactly to the left or right of the internal,
   * then there is an overlap.
   */

  if (request->l_start <= internal->l_end && request->l_end >=
      internal->l_start)
    {
      if (request->l_type == F_WRLCK || internal->l_type == F_WRLCK)
        {
          return request->l_pid != internal->l_pid;
        }
    }

  return false;
}

/****************************************************************************
 * Name: file_lock_find_bucket
 ****************************************************************************/

static FAR struct file_lock_bucket_s *
file_lock_find_bucket(FAR const char *filepath)
{
  FAR ENTRY *hretvalue;
  ENTRY item;

  item.key = (FAR char *)filepath;
  item.data = NULL;

  if (hsearch_r(item, FIND, &hretvalue, &g_file_lock_table) == 1)
    {
      return hretvalue->data;
    }

  return NULL;
}

/****************************************************************************
 * Name: file_lock_create_bucket
 ****************************************************************************/

static FAR struct file_lock_bucket_s *
file_lock_create_bucket(FAR const char *filepath)
{
  FAR struct file_lock_bucket_s *bucket;
  FAR ENTRY *hretvalue;
  ENTRY item;

  bucket = kmm_zalloc(sizeof(*bucket));
  if (bucket == NULL)
    {
      return NULL;
    }

  /* Creating an instance store */

  item.key = strdup(filepath);
  if (item.key == NULL)
    {
      kmm_free(bucket);
      return NULL;
    }

  item.data = bucket;

  if (hsearch_r(item, ENTER, &hretvalue, &g_file_lock_table) == 0)
    {
      lib_free(item.key);
      kmm_free(bucket);
      return NULL;
    }

  list_initialize(&bucket->list);
  nxsem_init(&bucket->wait, 0, 0);

  return bucket;
}

/****************************************************************************
 * Name: file_lock_modify
 ****************************************************************************/

static int file_lock_modify(FAR struct file *filep,
                            FAR struct file_lock_bucket_s *bucket,
                            FAR struct flock *request)
{
  FAR struct file_lock_s *new_file_lock = NULL;
  FAR struct file_lock_s *right = NULL;
  FAR struct file_lock_s *left = NULL;
  FAR struct file_lock_s *file_lock;
  FAR struct file_lock_s *tmp;
  bool added = false;
  bool find = false;

  list_for_every_entry_safe(&bucket->list, file_lock, tmp,
                            struct file_lock_s, fl_node)
    {
      if (request->l_pid != file_lock->fl_lock.l_pid)
        {
          /* Only file locks with the same pid need to be processed, so the
           * lookup is skipped.
           */

          if (find)
            {
              /* We've searched around and come back to the beginning. */

              break;
            }
        }
      else
        {
          find = true;

          /* Checking the type of overlapping locks */

          if (request->l_type == file_lock->fl_lock.l_type)
            {
              /* Compare the starting point of the last lock with the
               * starting point of the request, and use start - 1 instead of
               * end + 1, because if end is "off_t" max, then end + 1 will
               * be negative.
               */

              if (request->l_start - 1 > file_lock->fl_lock.l_end)
                {
                  continue;
                }

              if (request->l_end < file_lock->fl_lock.l_start - 1)
                {
                  break;
                }

              /* If the two locks are of the same type, then they are merged
               * into one lock with a lower start position and a higher end
               * position.
               */

              if (request->l_start < file_lock->fl_lock.l_start)
                {
                  file_lock->fl_lock.l_start = request->l_start;
                }
              else
                {
                  request->l_start = file_lock->fl_lock.l_start;
                }

              if (request->l_end > file_lock->fl_lock.l_end)
                {
                  file_lock->fl_lock.l_end = request->l_end;
                }
              else
                {
                  request->l_end = file_lock->fl_lock.l_end;
                }

              if (added)
                {
                  file_lock_delete(file_lock);
                  continue;
                }

              request = &file_lock->fl_lock;
              added = true;
            }
          else
            {
              if (request->l_start > file_lock->fl_lock.l_end)
                {
                  continue;
                }

              if (request->l_end < file_lock->fl_lock.l_start)
                {
                  break;
                }

              /* Scenarios for handling different types of locks */

              if (request->l_type == F_UNLCK)
                {
                  added = true;
                }

              /* The new lock and the old lock are adjacent or overlapping.
               * The code will handle this depending on the situation.
               * If the end address of the old lock is higher than the
               * new lock, then go ahead and insert the new lock here.
               */

              if (request->l_start > file_lock->fl_lock.l_start)
                {
                  left = file_lock;
                }

              if (request->l_end < file_lock->fl_lock.l_end)
                {
                  right = file_lock;
                  break;
                }

              if (request->l_start <= file_lock->fl_lock.l_start)
                {
                  /* In other cases, we are replacing old locks with new
                   * ones
                   */

                  if (added)
                    {
                      file_lock_delete(file_lock);
                      continue;
                    }

                  memcpy(&file_lock->fl_lock, request, sizeof(struct flock));
                  added = true;
                }
            }
        }
    }

  if (!added)
    {
      if (request->l_type == F_UNLCK)
        {
          return OK;
        }

      /* insert a new lock */

      new_file_lock = kmm_zalloc(sizeof(struct file_lock_s));
      if (new_file_lock == NULL)
        {
          return -ENOMEM;
        }

      new_file_lock->fl_file = filep;
      memcpy(&new_file_lock->fl_lock, request, sizeof(struct flock));
      list_add_before(&file_lock->fl_node, &new_file_lock->fl_node);
      file_lock = new_file_lock;
    }

  if (right)
    {
      if (left == right)
        {
          /* Splitting old locks */

          new_file_lock = kmm_zalloc(sizeof(struct file_lock_s));
          if (new_file_lock == NULL)
            {
              return -ENOMEM;
            }

          left = new_file_lock;
          memcpy(left, right, sizeof(struct file_lock_s));
          list_add_before(&file_lock->fl_node, &left->fl_node);
        }

      right->fl_lock.l_start = request->l_end + 1;
    }

  if (left)
    {
      left->fl_lock.l_end = request->l_start - 1;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_getlk
 *
 * Description:
 *   Attempts to lock the region (not a real lock), and if there is a
 *   conflict then returns information about the conflicting locks
 *
 * Input Parameters:
 *   filep - File structure instance
 *   flock - Lock types to be converted
 *
 * Returned Value:
 *   The resulting 0 on success. A errno value is returned on any failure.
 *
 ****************************************************************************/

int file_getlk(FAR struct file *filep, FAR struct flock *flock)
{
  FAR struct file_lock_bucket_s *bucket;
  FAR struct file_lock_s *file_lock;
  char path[PATH_MAX];
  int ret;

  /* We need to get the unique identifier (Path) via filep */

  ret = file_lock_get_path(filep, path);
  if (ret < 0)
    {
      return ret;
    }

  /* Convert a flock to a posix lock */

  ret = file_lock_normalize(filep, flock, flock);
  if (ret < 0)
    {
      return ret;
    }

  nxmutex_lock(&g_protect_lock);

  bucket = file_lock_find_bucket(path);
  if (bucket != NULL)
    {
      list_for_every_entry(&bucket->list, file_lock, struct file_lock_s,
                           fl_node)
        {
          if (file_lock_is_conflict(flock, &file_lock->fl_lock))
            {
              memcpy(flock, &file_lock->fl_lock, sizeof(*flock));
              goto out;
            }
        }
    }

  flock->l_type = F_UNLCK;

  /* Convert back to flock
   * The flock information saved in filelock is used as an offset
   * to the relative position. And for upper level applications,
   * l_len should be converted to cover the data quantity
   */

out:
  nxmutex_unlock(&g_protect_lock);
  if (flock->l_end == OFFSET_MAX)
    {
      flock->l_len = 0;
    }
  else
    {
      flock->l_len = flock->l_end - flock->l_start + 1;
    }

  return OK;
}

/****************************************************************************
 * Name: file_setlk
 *
 * Description:
 *   Actual execution of locking and unlocking behaviors
 *
 * Input Parameters:
 *   filep    - File structure instance
 *   flock    - Lock types to be converted
 *   nonblock - Waiting for lock
 *
 * Returned Value:
 *   The resulting 0 on success. A errno value is returned on any failure.
 *
 ****************************************************************************/

int file_setlk(FAR struct file *filep, FAR struct flock *flock,
               bool nonblock)
{
  FAR struct file_lock_bucket_s *bucket;
  FAR struct file_lock_s *file_lock;
  struct flock request;
  char path[PATH_MAX];
  int ret;

  /* We need to get the unique identifier (Path) via filep */

  ret = file_lock_get_path(filep, path);
  if (ret < 0)
    {
      return ret;
    }

  /* Convert a flock to a posix lock */

  ret = file_lock_normalize(filep, flock, &request);
  if (ret < 0)
    {
      return ret;
    }

  request.l_pid = getpid();

  nxmutex_lock(&g_protect_lock);

  bucket = file_lock_find_bucket(path);
  if (bucket == NULL)
    {
      /* If we request to unlock and the bucket is not found, it means
       * there is no lock here.
       */

      if (request.l_type == F_UNLCK)
        {
          nxmutex_unlock(&g_protect_lock);
          return OK;
        }

      /* It looks like we didn't find a bucket, let's go create one */

      bucket = file_lock_create_bucket(path);
      if (bucket == NULL)
        {
          nxmutex_unlock(&g_protect_lock);
          return -ENOMEM;
        }
    }
  else if (request.l_type != F_UNLCK)
    {
retry:
      list_for_every_entry(&bucket->list, file_lock, struct file_lock_s,
                           fl_node)
        {
          if (file_lock_is_conflict(&request, &file_lock->fl_lock))
            {
              if (nonblock)
                {
                  ret = -EAGAIN;
                  goto out;
                }

              bucket->nwaiter++;
              nxmutex_unlock(&g_protect_lock);
              nxsem_wait(&bucket->wait);
              nxmutex_lock(&g_protect_lock);
              bucket->nwaiter--;
              goto retry;
            }
        }
    }

  ret = file_lock_modify(filep, bucket, &request);
  if (ret < 0)
    {
      goto out;
    }

  /* When there is a lock change, we need to wake up the blocking lock */

  if (bucket->nwaiter > 0)
    {
      nxsem_post(&bucket->wait);
    }

out:
  file_lock_delete_bucket(bucket, path);
  nxmutex_unlock(&g_protect_lock);
  return ret;
}

/****************************************************************************
 * Name: file_closelk
 *
 * Description:
 *   Remove all locks associated with the filep when call close is applied.
 *
 * Input Parameters:
 *   filep - The filep that corresponds to the shutdown.
 *
 ****************************************************************************/

void file_closelk(FAR struct file *filep)
{
  FAR struct file_lock_bucket_s *bucket;
  FAR struct file_lock_s *file_lock;
  FAR struct file_lock_s *temp;
  char path[PATH_MAX];
  bool deleted = false;
  int ret;

  ret = file_lock_get_path(filep, path);
  if (ret < 0)
    {
      /* It isn't an error if fs doesn't support F_GETPATH, so we just end
       * it.
       */

      return;
    }

  bucket = file_lock_find_bucket(path);
  if (bucket == NULL)
    {
      /* There is no bucket here, so we don't need to free it. */

      return;
    }

  nxmutex_lock(&g_protect_lock);
  list_for_every_entry_safe(&bucket->list, file_lock, temp,
                            struct file_lock_s, fl_node)
    {
      if (file_lock->fl_file == filep)
        {
          deleted = true;
          file_lock_delete(file_lock);
        }
    }

  if (bucket->nwaiter > 0 && deleted)
    {
      nxsem_post(&bucket->wait);
    }
  else if (deleted)
    {
      file_lock_delete_bucket(bucket, path);
    }

  nxmutex_unlock(&g_protect_lock);
}

/****************************************************************************
 * Name: file_initlk
 *
 * Description:
 *   Initializing file locks
 *
 ****************************************************************************/

void file_initlk(void)
{
  /* Initialize file lock context hash table */

  hcreate_r(CONFIG_FS_LOCK_BUCKET_SIZE, &g_file_lock_table);
}

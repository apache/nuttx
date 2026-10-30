/****************************************************************************
 * include/nuttx/dma-buf.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_DMA_BUF_H
#define __INCLUDE_NUTTX_DMA_BUF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/atomic.h>
#include <nuttx/list.h>
#include <nuttx/mm/map.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMA_BUF_IOCTL_SYNC            _DMABUFIOC(0x1)
#define DMA_BUF_SET_NAME              _DMABUFIOC(0x2)
#define DMA_BUF_HEAP_IOCTL_ALLOC      _DMABUFIOC(0x3)

#define DMA_BUF_SYNC_READ             (1 << 0)
#define DMA_BUF_SYNC_WRITE            (2 << 0)
#define DMA_BUF_SYNC_RW               (DMA_BUF_SYNC_READ | DMA_BUF_SYNC_WRITE)
#define DMA_BUF_SYNC_START            (0 << 2)
#define DMA_BUF_SYNC_END              (1 << 2)
#define DMA_BUF_SYNC_MASK             (DMA_BUF_SYNC_RW | DMA_BUF_SYNC_END)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum dma_data_direction
{
  DMA_BIDIRECTIONAL = 0,
  DMA_TO_DEVICE = 1,
  DMA_FROM_DEVICE = 2,
  DMA_NONE = 3,
};

/* struct dma_buf_sync - Synchronize with CPU access.
 *
 * When a DMA buffer is accessed from the CPU via mmap, it is not always
 * possible to guarantee coherency between the CPU-visible map and underlying
 * memory.  To manage coherency, DMA_BUF_IOCTL_SYNC must be used to bracket
 * any CPU access to give the kernel the chance to shuffle memory around if
 * needed.
 *
 * Prior to accessing the map, the client must call DMA_BUF_IOCTL_SYNC
 * with DMA_BUF_SYNC_START and the appropriate read/write flags.  Once the
 * access is complete, the client should call DMA_BUF_IOCTL_SYNC with
 * DMA_BUF_SYNC_END and the same read/write flags.
 *
 * The synchronization provided via DMA_BUF_IOCTL_SYNC only provides cache
 * coherency.  It does not prevent other processes or devices from
 * accessing the memory at the same time.  If synchronization with a GPU or
 * other device driver is required, it is the client's responsibility to
 * wait for buffer to be ready for reading or writing before calling this
 * ioctl with DMA_BUF_SYNC_START.  Likewise, the client must ensure that
 * follow-up work is not submitted to GPU or other device driver until
 * after this ioctl has been called with DMA_BUF_SYNC_END?
 *
 * If the driver or API with which the client is interacting uses implicit
 * synchronization, waiting for prior work to complete can be done via
 * poll() on the DMA buffer file descriptor.  If the driver or API requires
 * explicit synchronization, the client may have to wait on a sync_file or
 * other synchronization primitive outside the scope of the DMA buffer API.
 */

struct dma_buf_sync
{
  /* flags: Set of access flags
   *
   * DMA_BUF_SYNC_START:
   *     Indicates the start of a map access session.
   *
   * DMA_BUF_SYNC_END:
   *     Indicates the end of a map access session.
   *
   * DMA_BUF_SYNC_READ:
   *     Indicates that the mapped DMA buffer will be read by the
   *     client via the CPU map.
   *
   * DMA_BUF_SYNC_WRITE:
   *     Indicates that the mapped DMA buffer will be written by the
   *     client via the CPU map.
   *
   * DMA_BUF_SYNC_RW:
   *     An alias for DMA_BUF_SYNC_READ | DMA_BUF_SYNC_WRITE.
   */

  uint64_t flags;
};

/* struct dma_buf_ops - operations possible on struct dma_buf */

struct dma_buf;
struct dma_buf_attachment;
struct dma_buf_ops
{
  /**************************************************************************
   * Name: attach
   *
   * Description:
   *   This is called from dma_buf_attach() to make sure that a given
   *   dma_buf_attachment can access the provided &dma_buf. Exporters
   *   which support buffer objects in special locations like VRAM or
   *   device-specific carveout areas should check whether the buffer could
   *   be move to system memory (or directly accessed by the provided
   *   device), and otherwise need to fail the attach operation.
   *
   *   The exporter should also in general check whether the current
   *   allocation fulfills the DMA constraints of the new device. If this
   *   is not the case, and the allocation cannot be moved, it should also
   *   fail the attach operation.
   *
   *   Any exporter-private housekeeping data can be stored in the
   *   &dma_buf_attachment.priv pointer.
   *
   *   This callback is optional.
   *
   * Returned Value:
   *   0 on success, negative error code on failure. It might return -EBUSY
   *   to signal that backing storage is already allocated and incompatible
   *   with the requirements of requesting device.
   *
   **************************************************************************/

  CODE int (*attach)(FAR struct dma_buf *dmabuf,
                     FAR struct dma_buf_attachment *attachment);

  /**************************************************************************
   * Name: detach
   *
   * Description:
   *   This is called by dma_buf_detach() to release a &dma_buf_attachment.
   *   Provided so that exporters can clean up any housekeeping for an
   *   &dma_buf_attachment.
   *
   * This callback is optional.
   *
   **************************************************************************/

  CODE void (*detach)(FAR struct dma_buf *dmabuf,
                      FAR struct dma_buf_attachment *attachment);

  /**************************************************************************
   * Name: map_dma_buf
   *
   * Description:
   *   This is called by dma_buf_map_attachment() and is used to map a
   *   shared &dma_buf into device address space, and it is mandatory.
   *   It can only be called if @attach has been called successfully.
   *
   *   This call may sleep, e.g. when the backing storage first needs to be
   *   allocated, or moved to a location suitable for all currently attached
   *   devices.
   *
   *   If this is being called for the first time, the exporter can now
   *   choose to scan through the list of attachments for this buffer,
   *   collate the requirements of the attached devices, and choose an
   *   appropriate backing storage for the buffer.
   *
   *   Based on enum dma_data_direction, it might be possible to have
   *   multiple users accessing at the same time (for reading, maybe), or
   *   any other kind of sharing that the exporter might wish to make
   *   available to buffer-users.
   *
   *   Note that for non-dynamic exporters the driver must guarantee that
   *   that the memory is available for use and cleared of any old data by
   *   the time this function returns.  Drivers which pipeline their buffer
   *   moves internally must wait for all moves and clears to complete.
   *
   * Returned Value:
   *   A address of the backing storage of the DMA buffer,
   *   already mapped into the device address space of the &device attached
   *   with the provided &dma_buf_attachment. The addresses and lengths in
   *   the scatter list are PAGE_SIZE aligned.
   *
   *   On failure, returns a negative error value wrapped into a pointer.
   *   May also return -EINTR when a signal was received while being
   *   blocked.
   *
   **************************************************************************/

  CODE uintptr_t (*map_dma_buf)(FAR struct dma_buf_attachment *attachment,
                                enum dma_data_direction direction);

  /**************************************************************************
   * Name: unmap_dma_buf
   *
   * Description:
   *   This is called by dma_buf_unmap_attachment() and should unmap and
   *   release the space allocated in @map_dma_buf, and it is mandatory.
   *
   **************************************************************************/

  CODE void (*unmap_dma_buf)(FAR struct dma_buf_attachment *attachment,
                             uintptr_t buf,
                             enum dma_data_direction direction);

  /**************************************************************************
   * Name: release
   *
   * Description:
   *   Called after the last dma_buf_put to release the &dma_buf, and
   *   mandatory.
   *
   **************************************************************************/

  CODE void (*release)(FAR struct dma_buf *dmabuf);

  /**************************************************************************
   * Name: begin_cpu_access
   *
   * Description:
   *   This is called from dma_buf_begin_cpu_access() and allows the
   *   exporter to ensure that the memory is actually coherent for cpu
   *   access. The exporter also needs to ensure that cpu access is coherent
   *   for the access direction. The direction can be used by the exporter
   *   to optimize the cache flushing, i.e. access with a different
   *   direction (read instead of write) might return stale or even bogus
   *   data (e.g. when the exporter needs to copy the data to temporary
   *   storage).
   *
   *   Note that this is both called through the DMA_BUF_IOCTL_SYNC IOCTL
   *   command for userspace mappings established through mmap().
   *
   *   This callback is optional.
   *
   * Returned Value:
   *   0 on success or a negative error code on failure. This can for
   *   example fail when the backing storage can't be allocated. Can also
   *   return -ERESTARTSYS or -EINTR when the call has been interrupted and
   *   needs to be restarted.
   *
   **************************************************************************/

  CODE int (*begin_cpu_access)(FAR struct dma_buf *dmabuf,
                               enum dma_data_direction direction);

  /**************************************************************************
   * Name: end_cpu_access
   *
   * Description:
   *   This is called from dma_buf_end_cpu_access() when the importer is
   *   done accessing the CPU. The exporter can use this to flush caches and
   *   undo anything else done in @begin_cpu_access.
   *
   *   This callback is optional.
   *
   * Returned Value:
   *   0 on success or a negative error code on failure. Can return
   *   -ERESTARTSYS or -EINTR when the call has been interrupted and needs
   *   to be restarted.
   *
   **************************************************************************/

  CODE int (*end_cpu_access)(FAR struct dma_buf *dmabuf,
                             enum dma_data_direction direction);

  /**************************************************************************
   * Name: mmap
   *
   * Description:
   *   This callback is used by the dma_buf_mmap() function
   *
   *   It should establish a mapping for the dma_buf in the provided vma.
   *   The exporter is responsible for setting up the vma to point to the
   *   backing storage of the dma_buf.
   *
   *   Note that the mapping needs to be incoherent, userspace is expected
   *   to bracket CPU access using the DMA_BUF_IOCTL_SYNC interface.
   *
   *   This callback is optional.
   *
   * Returned Value:
   *   0 on success or a negative error code on failure.
   *
   **************************************************************************/

  CODE int (*mmap)(FAR struct dma_buf *dmabuf,
                   FAR struct mm_map_entry_s *vma);
};

/* struct dma_buf_attachment - holds device-buffer attachment data
 *
 * This structure holds the attachment information between the dma_buf buffer
 * and its user. The list contains one attachment struct per device
 * attached to the buffer.
 *
 * An attachment is created by calling dma_buf_attach(), and released again
 * by calling dma_buf_detach(). The DMA mapping itself needed to initiate a
 * transfer is created by dma_buf_map_attachment() and freed again by calling
 * dma_buf_unmap_attachment().
 */

struct dma_buf_attachment
{
  FAR struct dma_buf *dmabuf; /* Buffer for this attachment. */
  FAR void *importer_priv;    /* Importer specific attachment data. */
  FAR void *priv;             /* Exporter specific attachment data. */
  struct list_node node;      /* List of dma_buf_attachment, protected by resv lock of the dmabuf. */
};

/* struct dma_buf_export_info - holds information needed to export a dma_buf
 * Used with dma_buf_export() only.
 */

struct dma_buf_export_info
{
  FAR const char *exp_name;          /* Name of the exporter */
  FAR const struct dma_buf_ops *ops; /* Attach allocator-defined dma buf ops to the new buffer */
  FAR void *priv;                    /* Attach private data of allocator to this buffer */
  size_t size;                       /* Size of the buffer - invariant over the lifetime of the buffer */
  int flags;                         /* Mode flags for the file */
};

/* struct dma_buf - shared buffer object
 *
 * This represents a shared buffer, created by calling dma_buf_export(). The
 * userspace representation is a normal file descriptor, which can be
 * created by calling dma_buf_fd().
 *
 * Shared dma buffers are reference counted using dma_buf_put() and
 * get_dma_buf().
 *
 * Device DMA access is handled by the separate &struct dma_buf_attachment.
 */

struct dma_buf
{
  /* Size of the buffer, invariant over the lifetime of the buffer */

  size_t size;

  /* Reference count for the buffer object */

  atomic_t refcount;

  /* List of dma_buf_attachment that denotes all devices attached,
   * protected by &dma_resv lock @resv.
   */

  struct list_node attachments;

  /* dma_buf_ops associated with this buffer object. */

  FAR const struct dma_buf_ops *ops;

  /* Name of the exporter; useful for debugging. Must not be NULL */

  FAR const char *exp_name;

  /* Userspace-provided name. Default value is NULL.
   * Useful for accounting and debugging. Read/Write accesses
   * are protected by name_lock
   *
   * See the IOCTLs DMA_BUF_SET_NAME
   */

  FAR char *name;

  /* Mutext to protect dma-buf access. */

  mutex_t lock;

  /* Exporter specific private data for this buffer object. */

  FAR void *priv;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: dma_buf_fd
 *
 * Description:
 *   Returns a file descriptor for the given struct dma_buf
 *
 * Input Parameters:
 *   dmabuf - pointer to dma_buf for which fd is required.
 *   flags  - flags to give to fd
 *
 * Returned Value:
 *   On success, returns an associated 'fd'. Else, returns error.
 *
 ****************************************************************************/

int dma_buf_fd(FAR struct dma_buf *dmabuf, int flags);

/****************************************************************************
 * Name: dma_buf_get
 *
 * Description:
 *   Returns the struct dma_buf related to an fd
 *
 * Input Parameters:
 *   fd - fd associated with the struct dma_buf to be returned
 *
 * Returned Value:
 *   On success, returns the struct dma_buf associated with an fd; uses
 *   file's refcounting done by file_get to increase refcount. returns NULL
 *   otherwise.
 *
 ****************************************************************************/

FAR struct dma_buf *dma_buf_get(int fd);

/****************************************************************************
 * Name: dma_buf_put
 *
 * Description:
 *   Decreases refcount of the buffer
 *   If, as a result of this call, the refcount becomes 0, the 'release' file
 *   operation related to this fd is called. It calls &dma_buf_ops.release
 *   vfunc in turn, and frees the memory allocated for dmabuf when exported.
 *
 * Input Parameters:
 *   dmabuf - buffer to reduce refcount of
 *
 ****************************************************************************/

void dma_buf_put(FAR struct dma_buf *dmabuf);

/****************************************************************************
 * Name: dma_buf_attach
 *
 * Description:
 *   Wrapper to call dma_buf_dynamic_attach() for drivers which still use
 *   a static mapping.
 *
 * Input Parameters:
 *   dmabuf - buffer to reduce refcount of
 *
 ****************************************************************************/

FAR struct dma_buf_attachment *dma_buf_attach(FAR struct dma_buf *dmabuf);

/****************************************************************************
 * Name: dma_buf_detach
 *
 * Description:
 *   Remove the given attachment from dmabuf's attachments list
 *   Clean up a device attachment obtained by calling dma_buf_attach().
 *   Optionally this calls &dma_buf_ops.detach for device-specific detach.
 *
 * Input Parameters:
 *   dmabuf - buffer to detach from.
 *   attach - attachment to be detached; is free'd after this call.
 *
 ****************************************************************************/

void dma_buf_detach(FAR struct dma_buf *dmabuf,
                    FAR struct dma_buf_attachment *attach);

/****************************************************************************
 * Name: dma_buf_begin_cpu_access
 *
 * Description:
 *   Must be called before accessing a dma_buf from the cpu in the kernel.
 *   Calls begin_cpu_access to allow exporter-specific preparations.
 *   Coherency is only guaranteed in the specified range for the specified
 *   access direction.
 *
 *   After the cpu access is complete the caller should call
 *   dma_buf_end_cpu_access(). Only when cpu access is bracketed by both
 *   calls is it guaranteed to be coherent with other DMA access.
 *
 *   This function will also wait for any DMA transactions tracked through
 *   implicit synchronization in &dma_buf.resv. For DMA transactions with
 *   explicit synchronization this function will only ensure cache
 *   coherency, callers must ensure synchronization with such DMA
 *   transactions on their own.
 *
 * Input Parameters:
 *   dmabuf    - buffer to prepare cpu access for.
 *   direction - direction of access.
 *
 * Returned Value:
 *   return negative error values, returns 0 on success.
 *
 ****************************************************************************/

int dma_buf_begin_cpu_access(FAR struct dma_buf *dmabuf,
                             enum dma_data_direction direction);

/****************************************************************************
 * Name: dma_buf_end_cpu_access
 *
 * Description:
 *   Must be called after accessing a dma_buf from the cpu in the kernel
 *   context. Calls end_cpu_access to allow exporter-specific actions.
 *   Coherency is only guaranteed in the specified range for the
 *   specified access direction.
 *
 *   This terminates CPU access started with dma_buf_begin_cpu_access().
 *
 * Input Parameters:
 *   dmabuf    - buffer to prepare cpu access for.
 *   direction - direction of access.
 *
 * Returned Value:
 *   return negative error values, returns 0 on success.
 *
 ****************************************************************************/

int dma_buf_end_cpu_access(FAR struct dma_buf *dmabuf,
                           enum dma_data_direction direction);

/****************************************************************************
 * Name: dma_buf_map_attachment
 *
 * Description:
 *   Returns the pointer of address of the attachment, mapped into _device_
 *   address space. Is a wrapper for map_dma_buf() of the dma_buf_ops.
 *
 * Input Parameters:
 *   attach    - attachment whose scatterlist is to be returned
 *   direction - direction of DMA transfer
 *
 * Returned Value:
 *   Returns the attachment's physical address on success, 0 on failure.
 *
 *   A mapping must be unmapped by using dma_buf_unmap_attachment().
 *   Note that the underlying backing storage is pinned for as long as a
 *   mapping exists, therefore users/importers should not hold onto a
 *   mapping for undue amounts of time.
 *
 ****************************************************************************/

uintptr_t dma_buf_map_attachment(FAR struct dma_buf_attachment *attach,
                                 enum dma_data_direction direction);

/****************************************************************************
 * Name: dma_buf_unmap_attachment
 *
 * Description:
 *   unmaps and decreases usecount of the buffer;
 *   Is a wrapper for unmap_dma_buf() of dma_buf_ops.
 *
 * Input Parameters:
 *   attach    - attachment to unmap buffer from
 *   buf       - address returned by dma_buf_map_attachment
 *   direction - direction of DMA transfer
 *
 * Returned Value:
 *   Returns, on success, a newly created struct dma_buf object, which wraps
 *   the supplied private data and operations for struct dma_buf_ops.
 *   On either missing ops, or error in allocating struct dma_buf, will
 *   return negative error.
 *
 ****************************************************************************/

void dma_buf_unmap_attachment(FAR struct dma_buf_attachment *attach,
                              uintptr_t buf,
                              enum dma_data_direction direction);

/****************************************************************************
 * Name: dma_buf_export
 *
 * Description:
 *   Creates a new dma_buf, and associates an anon file with this buffer,
 *   so it can be exported.
 *   Also connect the allocator specific data and ops to the buffer.
 *   Additionally, provide a name string for exporter; useful in debugging.
 *
 * Input Parameters:
 *   exp_info - holds all the export related information provided by the
 *              exporter. see &struct dma_buf_export_info for further
 *              details.
 *
 * Returned Value:
 *   Returns, on success, a newly created struct dma_buf object, which wraps
 *   the supplied private data and operations for struct dma_buf_ops.
 *   On either missing ops, or error in allocating struct dma_buf, will
 *   return negative error.
 *
 ****************************************************************************/

FAR struct dma_buf *
dma_buf_export(FAR const struct dma_buf_export_info *exp_info);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_DMA_BUF_H */

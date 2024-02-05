/****************************************************************************
 * include/nuttx/tee.h
 *
 * Copyright (c) 2015-2016, Linaro Limited
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following
 * conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_TEE_H
#define __INCLUDE_NUTTX_TEE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This file describes the API provided by a TEE driver to user space.
 *
 * Each TEE driver defines a TEE specific protocol which is used for the
 * data passed back and forth using TEE_IOC_CMD.
 */

/* Helpers to make the ioctl defines */

#define TEE_IOC_MAGIC   0xa4
#define TEE_IOC_BASE    0

#define TEE_MAX_ARG_SIZE        1024

#define TEE_GEN_CAP_GP          (1 << 0) /* GlobalPlatform compliant TEE */
#define TEE_GEN_CAP_PRIVILEGED  (1 << 1) /* Privileged device (for supplicant) */
#define TEE_GEN_CAP_REG_MEM     (1 << 2) /* Supports registering shared memory */
#define TEE_GEN_CAP_MEMREF_NULL (1 << 3) /* NULL MemRef support */

#define TEE_MEMREF_NULL         ((uint64_t)-1) /* NULL MemRef Buffer */

/* TEE Implementation ID */

#define TEE_IMPL_ID_OPTEE       1
#define TEE_IMPL_ID_AMDTEE      2

/* OP-TEE specific capabilities */

#define TEE_OPTEE_CAP_TZ        (1 << 0)

/* struct tee_ioctl_version_data - TEE version
 * impl_id:    [out] TEE implementation id
 * impl_caps:  [out] Implementation specific capabilities
 * gen_caps:   [out] Generic capabilities, defined by TEE_GEN_CAPS_* above
 *
 * Identifies the TEE implementation, impl_id is one of TEE_IMPL_ID_* above.
 * impl_caps is implementation specific, for example TEE_OPTEE_CAP_*
 * is valid when impl_id == TEE_IMPL_ID_OPTEE.
 */

struct tee_ioctl_version_data
{
  uint32_t impl_id;
  uint32_t impl_caps;
  uint32_t gen_caps;
};

/* TEE_IOC_VERSION - query version of TEE
 *
 * Takes a tee_ioctl_version_data struct and returns with the TEE version
 * data filled in.
 */
#define TEE_IOC_VERSION  _IOC(TEE_IOC_MAGIC << 8, TEE_IOC_BASE + 0)

/* struct tee_ioctl_shm_alloc_data - Shared memory allocate argument
 * size:       [in/out] Size of shared memory to allocate
 * flags:      [in/out] Flags to/from allocation.
 * id:         [out] Identifier of the shared memory
 *
 * The flags field should currently be zero as input. Updated by the call
 * with actual flags as defined by TEE_IOCTL_SHM_* above.
 * This structure is used as argument for TEE_IOC_SHM_ALLOC below.
 */

struct tee_ioctl_shm_alloc_data
{
  uint64_t size;
  uint32_t flags;
  int32_t id;
};

/* TEE_IOC_SHM_ALLOC - allocate shared memory
 *
 * Allocates shared memory between the user space process and secure OS.
 *
 * Returns a file descriptor on success or < 0 on failure
 *
 * The returned file descriptor is used to map the shared memory into user
 * space. The shared memory is freed when the descriptor is closed and the
 * memory is unmapped.
 */

#define TEE_IOC_SHM_ALLOC _IOC(TEE_IOC_MAGIC << 8, TEE_IOC_BASE + 1)

/* struct tee_ioctl_shm_register_fd_data - Shared memory registering argument
 * fd:         [in] file descriptor identifying the shared memory
 * size:       [out] Size of shared memory to allocate
 * flags:      [in] Flags to/from allocation.
 * id:         [out] Identifier of the shared memory
 *
 * The flags field should currently be zero as input. Updated by the call
 * with actual flags as defined by TEE_IOCTL_SHM_* above.
 * This structure is used as argument for TEE_IOC_SHM_ALLOC below.
 */

struct tee_ioctl_shm_register_fd_data
{
  int64_t fd;
  uint64_t size;
  uint32_t flags;
  int32_t id;
};

/* Attributes for struct tee_ioctl_param, selects field in the union */

#define TEE_IOCTL_PARAM_ATTR_TYPE_NONE          0       /* parameter not used */

/* These defines value parameters (struct tee_ioctl_param_value) */

#define TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT   1
#define TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT  2
#define TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT   3       /* input and output */

/* These defines shared memory reference parameters (struct
 * tee_ioctl_param_memref)
 */

#define TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT  5
#define TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT 6
#define TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT  7       /* input and output */

/* Mask for the type part of the attribute, leaves room for more types */

#define TEE_IOCTL_PARAM_ATTR_TYPE_MASK          0xff

/* Meta parameter carrying extra information about the message. */

#define TEE_IOCTL_PARAM_ATTR_META               0x100

/* Mask of all known attr bits */
#define TEE_IOCTL_PARAM_ATTR_MASK \
        (TEE_IOCTL_PARAM_ATTR_TYPE_MASK | TEE_IOCTL_PARAM_ATTR_META)

/* Matches TEEC_LOGIN_* in GP TEE Client API
 * Are only defined for GP compliant TEEs
 */

#define TEE_IOCTL_LOGIN_PUBLIC                  0
#define TEE_IOCTL_LOGIN_USER                    1
#define TEE_IOCTL_LOGIN_GROUP                   2
#define TEE_IOCTL_LOGIN_APPLICATION             4
#define TEE_IOCTL_LOGIN_USER_APPLICATION        5
#define TEE_IOCTL_LOGIN_GROUP_APPLICATION       6

/* Disallow user-space to use GP implementation specific login
 * method range (0x80000000 - 0xBFFFFFFF). This range is rather
 * being reserved for REE kernel clients or TEE implementation.
 */

#define TEE_IOCTL_LOGIN_REE_KERNEL_MIN          0x80000000
#define TEE_IOCTL_LOGIN_REE_KERNEL_MAX          0xBFFFFFFF

/* Private login method for REE kernel clients */

#define TEE_IOCTL_LOGIN_REE_KERNEL              0x80000000

/* struct tee_ioctl_param - parameter
 * attr: attributes
 * a: if a memref, offset into the shared memory object, else a value
 * parameter
 * b: if a memref, size of the buffer, else a value parameter
 * c: if a memref, shared memory identifier, else a value parameter
 *
 * attr & TEE_PARAM_ATTR_TYPE_MASK indicates if memref or value is used in
 * the union. TEE_PARAM_ATTR_TYPE_VALUE_* indicates value and
 * TEE_PARAM_ATTR_TYPE_MEMREF_* indicates memref. TEE_PARAM_ATTR_TYPE_NONE
 * indicates that none of the members are used.
 *
 * Shared memory is allocated with TEE_IOC_SHM_ALLOC which returns an
 * identifier representing the shared memory object. A memref can reference
 * a part of a shared memory by specifying an offset (a) and size (b) of
 * the object. To supply the entire shared memory object set the offset
 * (a) to 0 and size (b) to the previously returned size of the object.
 */

struct tee_ioctl_param
{
  uint64_t attr;
  uint64_t a;
  uint64_t b;
  uint64_t c;
};

#define TEE_IOCTL_UUID_LEN  16

/* struct tee_ioctl_open_session_arg - Open session argument
 * uuid:       [in] UUID of the Trusted Application
 * clnt_uuid:  [in] UUID of client
 * clnt_login: [in] Login class of client, TEE_IOCTL_LOGIN_* above
 * cancel_id:  [in] Cancellation id, a unique value to identify this request
 * session:    [out] Session id
 * ret:        [out] return value
 * ret_origin  [out] origin of the return value
 * num_params  [in] number of parameters following this struct
 */

struct tee_ioctl_open_session_arg
{
  uint8_t uuid[TEE_IOCTL_UUID_LEN];
  uint8_t clnt_uuid[TEE_IOCTL_UUID_LEN];
  uint32_t clnt_login;
  uint32_t cancel_id;
  uint32_t session;
  uint32_t ret;
  uint32_t ret_origin;
  uint32_t num_params;

  /* num_params tells the actual number of element in params */

  struct tee_ioctl_param params[];
};

/* TEE_IOC_OPEN_SESSION - opens a session to a Trusted Application
 *
 * Takes a struct tee_ioctl_buf_data which contains a struct
 * tee_ioctl_open_session_arg followed by any array of struct
 * tee_ioctl_param
 */

#define TEE_IOC_OPEN_SESSION _IOC(TEE_IOC_MAGIC << 8, TEE_IOC_BASE + 2)

/* struct tee_ioctl_invoke_func_arg - Invokes a function in a Trusted
 * Application
 * func:       [in] Trusted Application function, specific to the TA
 * session:    [in] Session id
 * cancel_id:  [in] Cancellation id, a unique value to identify this request
 * ret:        [out] return value
 * ret_origin  [out] origin of the return value
 * num_params  [in] number of parameters following this struct
 */

struct tee_ioctl_invoke_arg
{
  uint32_t func;
  uint32_t session;
  uint32_t cancel_id;
  uint32_t ret;
  uint32_t ret_origin;
  uint32_t num_params;

  /* num_params tells the actual number of element in params */

  struct tee_ioctl_param params[];
};

/* TEE_IOC_INVOKE - Invokes a function in a Trusted Application
 *
 * Takes a struct tee_ioctl_buf_data which contains a struct
 * tee_invoke_func_arg followed by any array of struct tee_param
 */

#define TEE_IOC_INVOKE  _IOC(TEE_IOC_MAGIC << 8, TEE_IOC_BASE + 3)

/* struct tee_ioctl_cancel_arg - Cancels an open session or invoke ioctl
 * cancel_id:  [in] Cancellation id, a unique value to identify this request
 * session:    [in] Session id, if the session is opened, else set to 0
 */

struct tee_ioctl_cancel_arg
{
  uint32_t cancel_id;
  uint32_t session;
};

/* TEE_IOC_CANCEL - Cancels an open session or invoke */

#define TEE_IOC_CANCEL  _IOC(TEE_IOC_MAGIC << 4, TEE_IOC_BASE + 0)

/* struct tee_ioctl_close_session_arg - Closes an open session
 * session: [in] Session id
 */

struct tee_ioctl_close_session_arg
{
  uint32_t session;
};

/* TEE_IOC_CLOSE_SESSION - Closes a session */

#define TEE_IOC_CLOSE_SESSION _IOC(TEE_IOC_MAGIC << 8, TEE_IOC_BASE + 5)

/* struct tee_iocl_supp_recv_arg - Receive a request for a supplicant
 * function
 * func:       [in] supplicant function
 * num_params  [in/out] number of parameters following this struct
 *
 * num_params is the number of params that tee-supplicant has room to
 * receive when input, num_params is the number of actual params
 * tee-supplicant receives when output.
 */

struct tee_iocl_supp_recv_arg
{
  uint32_t func;
  uint32_t num_params;

  /* num_params tells the actual number of element in params */

  struct tee_ioctl_param params[];
};

/* TEE_IOC_SUPPL_RECV - Receive a request for a supplicant function
 *
 * Takes a struct tee_ioctl_buf_data which contains a struct
 * tee_iocl_supp_recv_arg followed by any array of struct tee_param
 */

#define TEE_IOC_SUPPL_RECV _IOC(TEE_IOC_MAGIC << 8, TEE_IOC_BASE + 6)

/* struct tee_iocl_supp_send_arg - Send a response to a received request
 * ret:        [out] return value
 * num_params  [in] number of parameters following this struct
 */

struct tee_iocl_supp_send_arg
{
  uint32_t ret;
  uint32_t num_params;

  /* num_params tells the actual number of element in params */

  struct tee_ioctl_param params[];
};

/* TEE_IOC_SUPPL_SEND - Receive a request for a supplicant function
 *
 * Takes a struct tee_ioctl_buf_data which contains a struct
 * tee_iocl_supp_send_arg followed by any array of struct tee_param
 */

#define TEE_IOC_SUPPL_SEND _IOC(TEE_IOC_MAGIC << 8, TEE_IOC_BASE + 7)

/* struct tee_ioctl_shm_register_data - Shared memory register argument
 * addr:      [in] Start address of shared memory to register
 * length:    [in/out] Length of shared memory to register
 * flags:     [in/out] Flags to/from registration.
 * id:        [out] Identifier of the shared memory
 *
 * The flags field should currently be zero as input. Updated by the call
 * with actual flags as defined by TEE_IOCTL_SHM_* above.
 * This structure is used as argument for TEE_IOC_SHM_REGISTER below.
 */

struct tee_ioctl_shm_register_data
{
  uint64_t addr;
  uint64_t length;
  uint32_t flags;
  int32_t id;
};

/* TEE_IOC_SHM_REGISTER_FD - register a shared memory from a file descriptor
 *
 * Returns a file descriptor on success or < 0 on failure
 *
 * The returned file descriptor refers to the shared memory object in kernel
 * land. The shared memory is freed when the descriptor is closed.
 */

#define TEE_IOC_SHM_REGISTER_FD _IOC(TEE_IOC_MAGIC << 8, TEE_IOC_BASE + 8)

/* struct tee_ioctl_buf_data - Variable sized buffer
 * buf_ptr:    [in] A __user pointer to a buffer
 * buf_len:    [in] Length of the buffer above
 *
 * Used as argument for TEE_IOC_OPEN_SESSION, TEE_IOC_INVOKE,
 * TEE_IOC_SUPPL_RECV, and TEE_IOC_SUPPL_SEND below.
 */

struct tee_ioctl_buf_data
{
  uint64_t buf_ptr;
  uint64_t buf_len;
};

/* TEE_IOC_SHM_REGISTER - Register shared memory argument
 *
 * Registers shared memory between the user space process and secure OS.
 *
 * Returns a file descriptor on success or < 0 on failure
 *
 * The shared memory is unregisterred when the descriptor is closed.
 */

#define TEE_IOC_SHM_REGISTER   _IOC(TEE_IOC_MAGIC << 8, TEE_IOC_BASE + 9)

/* Five syscalls are used when communicating with the TEE driver.
 * open(): opens the device associated with the driver
 * ioctl(): as described above operating on the file descriptor from open()
 * close(): two cases
 *   - closes the device file descriptor
 *   - closes a file descriptor connected to allocated shared memory
 * mmap(): maps shared memory into user space using information from struct
 *    tee_ioctl_shm_alloc_data
 * munmap(): unmaps previously shared memory
 */

#endif /* __INCLUDE_NUTTX_TEE_H */

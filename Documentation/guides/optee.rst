=======================
Interfacing with OP-TEE
=======================

Overview
========

NuttX supports basic interfacing with OP-TEE OS through three different
transports: local network, RPMsg, and native Secure Monitor Calls (SMCs)
on arm. Tasks can interface with the OP-TEE driver (and in turn with the
OP-TEE OS) via IOCTLs on the TEE (``/dev/tee#``) character device. This
interface should allow use-of/integration-with libteec, although this is
not officially supported by NuttX, and is out of the scope of this guide.

The driver supports opening and closing sessions, allocating and registering
shared memory, and invoking functions on OP-TEE Trusted Applications (TAs).
The driver also supports, reverse direction commands called RPCs
(TA -> Normal World). Some of the RPCs are handled completely by the kernel
driver while others require the TEE supplicant userspace process to be running
by having opened (``/dev/teepriv#``). Similarly to libteec, the supplicant
is not officially supported.

.. note::
   ``/dev/teepriv#`` is reserved solely for the supplicant and shouldn't be
   used by any other NuttX application.


Enabling the OP-TEE Driver
==========================

The driver is enabled using one of:

- ``CONFIG_DEV_OPTEE_LOCAL``
- ``CONFIG_DEV_OPTEE_RPMSG``
- ``CONFIG_DEV_OPTEE_SMC``

All of the above require also ``CONFIG_ALLOW_BSD_COMPONENTS`` and
``CONFIG_FS_ANONMAP``. So, at a bare minimum, to enable the driver
one would need something like the following:

.. code-block::

  CONFIG_ALLOW_BSD_COMPONENTS=y
  CONFIG_DEV_OPTEE_SMC=y
  CONFIG_FS_ANONMAP=y

Each implementation (local, RPMsg, or SMC) may have further dependencies
(e.g. RPMsg requires ``CONFIG_NET_RPMSG`` and more) and may have further
parameters to configure (e.g. RPMsg remote CPU name through
``CONFIG_OPTEE_REMOTE_CPU_NAME``).

.. warning::
  ``CONFIG_DEV_OPTEE_SMC`` has only been tested on arm64. Also, please note
  that in configurations with ``CONFIG_ARM*_DCACHE_DISABLE=y`` you might
  encounter issues with shared memory depending on the state of the data
  cache in Secure World.

If ``CONFIG_DEV_OPTEE_SMC`` is enabled we can also enable the kernel driver
for the TEE supplicant by using ``CONFIG_DEV_OPTEE_SUPPLICANT``.

Successful registration of the driver can be verified by looking into
``/dev/tee0`` and ``/dev/teepriv0`` (for the supplicant). For instance,
incompatibility with the TEE OS running in the system, will prevent the
``/dev/tee0`` character device from being registered.

IOCTLs supported
================

All IOCTLs return negative error codes on failure. All of them return 0
on success unless otherwise specified (see ``TEE_IOC_SHM_ALLOC``).

- ``TEE_IOC_VERSION`` : Query the version and capabilities of the TEE driver.

  - Use the ``struct tee_ioctl_version_data`` to get the version and
    capabilities. This driver supports OP-TEE so you should expect to
    receive only ``TEE_IMPL_ID_OPTEE`` in ``.impl_id`` and ``TEE_OPTEE_CAP_TZ``
    in ``.impl_caps``. The driver is GlobalPlatform compliant, and you should
    always expect to receive ``TEE_GEN_CAP_GP | TEE_GEN_CAP_MEMREF_NULL`` in
    ``.gen_caps``. If using the SMC implementation, the driver supports also
    shared memory registration, so you can expect also ``TEE_GEN_CAP_REG_MEM``
    in ``.gen_caps``.

- ``TEE_IOC_OPEN_SESSION`` :  Open a session with a Trusted Application.

  - Expects a ``struct tee_ioctl_buf_data`` pointer, pointing to a
    ``struct tee_ioctl_open_session_arg`` instance with at minimum, the ``.uuid``
    set. You can typically use ``uuid_enc_be()`` to encode a ``uuid_t`` struct
    to the raw byte buffer expected in the ``.uuid`` field. After a successful
    call, you can expect to get a session identifier back in the ``.session``
    field.

- ``TEE_IOC_CLOSE_SESSION`` : Close a session with a Trusted Application.

  - Expects a pointer to a ``struct tee_ioctl_close_session_arg`` with the
    ``.session`` field set to the identifier of the session to close.

- ``TEE_IOC_INVOKE`` : Invoke a function on a previously opened session to a Trusted Application.

  - Expects a ``struct tee_ioctl_buf_data`` pointer, pointing to a
    ``struct tee_ioctl_invoke_arg`` instance. You can use the
    ``TEE_IOCTL_PARAM_SIZE()`` macro to calculate the size of the
    variable-length array of ``struct tee_ioctl_param`` parameters in the
    invoke arguments struct. At minimum, the interface expects the fields
    ``.func``, ``.session``, ``.num_params``, and ``.params`` to be set.
    ``.cancel_id`` can be optionally set to enable later canceling of this
    command if needed.
    You might notice that ``struct tee_ioctl_param`` has rather obscure field
    names (``.a``, ``.b``, ``.c``). This can be improved with a union in the
    future, but until then, please refer to ``include/nuttx/tee.h`` for details.
    In short, for shared memory references, ``.a`` is the offset into the
    shared memory buffer, ``.b`` is the size of the buffer, and ``.c`` is the
    the shared memory identifier (``.id`` field returned by
    ``TEE_IOC_SHM_ALLOC`` or ``TEE_IOC_SHM_REGISTER``).

- ``TEE_IOC_CANCEL`` : Cancel a currently invoked command.

  - Expects a ``struct tee_ioctl_cancel_arg`` pointer with the ``.session``
    and ``.cancel_id`` fields set.

- ``TEE_IOC_SHM_ALLOC`` : Allocate shared memory between the user space and the secure OS.

  - Expects a ``struct tee_ioctl_shm_alloc_data`` pointer with the ``.size``
    field set, and ignoring the ``.flags`` field. Upon successful return,
    it returns the memory file descriptor one can use ``mmap()`` on (with
    ``MAP_SHARED``). It also returns an identifier for use in memory references
    (``tee_ioctl_param.c`` field) in ``.id``.

- ``TEE_IOC_SHM_REGISTER`` : Register a shared memory reference with the secure OS.

  - Expects a pointer to a ``struct tee_ioctl_shm_register_data`` instance
    with all fields set except ``.id``. ``.flags`` can be any combination of
    ``TEE_SHM_REGISTER`` and ``TEE_SHM_SEC_REGISTER`` but not ``TEE_SHM_ALLOC``.
    ``TEE_SHM_REGISTER`` registers the memory with the driver for automatic
    cleanup (not freeing!) during ``/dev/tee#`` character device close.
    ``TEE_SHM_SEC_REGISTER`` registers the memory with the secure OS for later
    use in memrefs and is automatically de-registered during driver close if
    ``TEE_SHM_REGISTER`` is also specified. ``.addr`` shall point to the (user)
    memory to register and ``.size`` shall indicate its size. One may use the
    returned ``.id`` field when specifying shared memory references
    (``tee_ioctl_param.c`` field).

- ``TEE_IOC_SUPPL_RECV`` : Receive an RPC request from the OP-TEE that needs userspace interaction from the supplicant.

  - Expects a pointer to a ``struct tee_ioctl_buf_data`` instance where the
    ``.buf_ptr`` field points to a user allocated buffer that must hold a
    ``struct tee_iocl_supp_send/recv_arg`` followed by a number of
    ``struct tee_ioctl_param`` parameters.  The ``.buf_len`` field communicates
    to the kernel the length of that buffer.  If the user passes a bigger number
    of parameters than ``OPTEE_MAX_PARAM_NUM`` or smaller number of parameters than
    the number sent by OP-TEE, the ioctl will fail. The TEE supplicant by default
    uses 5 ``struct tee_ioctl_param`` parameters.

- ``TEE_IOC_SUPPL_SEND`` : Respond to an RPC request from the OP-TEE that needed userspace interaction from the supplicant.

  - Expects a pointer to a ``struct tee_ioctl_buf_data`` instance where the
    ``.buf_ptr`` field points to a user allocated buffer that must hold a
    ``struct tee_iocl_supp_send/recv_arg`` followed by a number of
    ``struct tee_ioctl_param`` parameters.  The ``.buf_len`` field communicates
    to the kernel the length of that buffer. The number of parameters depends on
    the size of expected RPC response by the OP-TEE.

Typical usage
=============

#. Include the necessary headers:

   .. code-block:: c

     #include <stdio.h>
     #include <stdlib.h>
     #include <fcntl.h>
     #include <unistd.h>
     #include <errno.h>
     #include <sys/ioctl.h>
     #include <nuttx/tee.h>
     #include <uuid.h>

#. Open the TEE character device

   .. code-block:: c

     int fd = open("/dev/tee0", O_RDONLY | O_NONBLOCK);

#. Check the version and capabilities

   .. code-block:: c

     struct tee_ioctl_version_data ioc_ver;

     int ret = ioctl(fd, TEE_IOC_VERSION, (unsigned long)&ioc_ver);
     if (ret < 0)
       {
         printf("Failed to query TEE driver version and caps: %d, %s\n",
               ret, strerror(errno));
         return ret;
       }

     /* check ioc_ver accordingly */

#. Open a session with a Trusted Application

   .. code-block:: c

     const uuid_t *uuid = [...];
     struct tee_ioctl_open_session_arg ioc_opn = { 0 };
     struct tee_ioctl_buf_data ioc_buf;

     uuid_enc_be(&ioc_opn.uuid, uuid);

     ioc_buf.buf_ptr = (uintptr_t)&ioc_opn;
     ioc_buf.buf_len = sizeof(struct tee_ioctl_open_session_arg);

     ret = ioctl(fd, TEE_IOC_OPEN_SESSION, (unsigned long)&ioc_buf);
     if (ret < 0)
       {
         return ret;
       }

     /* use ioc_opn.session returned */

#. Invoke a function of the Trusted Application

   .. code-block:: c

     const size_t num_params = 1;
     struct tee_ioctl_invoke_arg *ioc_args;
     struct tee_ioctl_buf_data ioc_buf;
     size_t ioc_args_len;

     ioc_args_len = sizeof(struct tee_ioctl_invoke_arg) +
                    TEE_IOCTL_PARAM_SIZE(num_params);

     ioc_args = (struct tee_ioctl_invoke_arg *)calloc(1, ioc_args_len);
     if (!ioc_args)
       {
         return -ENOMEM;
       }

     ioc_args->func = <SOME_FUNCTION_ID>;
     ioc_args->session = ioc_opn.session;
     ioc_args->num_params = num_params;
     ioc_args->params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT;

     ioc_buf.buf_ptr = (uintptr_t)ioc_args;
     ioc_buf.buf_len = ioc_args_len;

     ret = ioctl(fd, TEE_IOC_INVOKE, (unsigned long)&ioc_buf);
     if (ret < 0)
       {
         goto err_with_args;
       }

     /* use result (if any) in ioc_args->params */

#. Allocate shared memory through the driver

   .. code-block:: c

     struct tee_ioctl_shm_alloc_data ioc_alloc = { 0 };
     int memfd;
     void *shm;

     ioc_alloc.size = 1024;

     memfd = ioctl(fd, TEE_IOC_SHM_ALLOC, (unsigned long)&ioc_alloc);
     if (memfd < 0)
       {
         return memfd;
       }

     shm = mmap(NULL, ioc_alloc.size, PROT_READ | PROT_WRITE, MAP_SHARED,
                memfd, 0);
     if (shm == MAP_FAILED)
       {
         close(memfd);
         return -ENOMEM;
       }

#. Register shared memory with the driver and the secure OS

   .. code-block:: c

     /* The following will fail if TEE_GEN_CAP_REG_MEM is not reported in
      * the returned `ioc_ver.gen_caps` in step 1 above
      * Note: user memory used does not have to be allocated through IOCTL
      */

     struct tee_ioctl_shm_register_data ioc_reg = { 0 };

     ioc_reg.addr = (uintptr_t)<some user memory ptr>;
     ioc_reg.length = <user memory size>;

     memfd = ioctl(fd, TEE_IOC_SHM_REGISTER, (unsigned long)&ioc_reg);
     if (memfd < 0)
       {
         return ret;
       }

     /* use ioc_reg.id returned in OP-TEE parameters (e.g. open, invoke) */

     close(memfd);

#. Use the registered shared memory in an invocation

   .. code-block:: c

     const size_t num_params = 1;
     struct tee_ioctl_invoke_arg *ioc_args;
     struct tee_ioctl_buf_data ioc_buf;
     size_t ioc_args_len;

     ioc_args_len = sizeof(struct tee_ioctl_invoke_arg) +
                    TEE_IOCTL_PARAM_SIZE(num_params);

     ioc_args = (struct tee_ioctl_invoke_arg *)calloc(1, ioc_args_len);
     if (!ioc_args)
       {
         return -ENOMEM;
       }

     ioc_args->func = <SOME_FUNCTION_ID>;
     ioc_args->session = ioc_opn.session;
     ioc_args->num_params = num_params;
     ioc_args->params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
     ioc_args->params[0].a = 0;
     ioc_args->params[0].b = ioc_reg.length;
     ioc_args->params[0].c = ioc_reg.id;

     ioc_buf.buf_ptr = (uintptr_t)ioc_args;
     ioc_buf.buf_len = ioc_args_len;

     ret = ioctl(fd, TEE_IOC_INVOKE, (unsigned long)&ioc_buf);
     if (ret < 0)
       {
         goto err_with_args;
       }

     /* use result (if any) in ioc_args->params */

#. OP-TEE secure storage support through TEE supplicant

   .. code-block:: shell

     optee_supplicant -f /data/tee > /dev/null &

This runs the OP-TEE supplicant in the background, using ``/data/tee`` as the
directory for the TEE file system. Output is redirected to ``/dev/null`` to
suppress standard output. Make sure that the userspace support for the
supplicant is enabled and that ``/data`` is mounted as read/write.

With the supplicant running, secure storage objects can be created, retrieved,
and managed by Trusted Applications (TAs). In a typical workflow, a Client
Application (CA) running on NuttX invokes a command in a TA that may need to
read from or create persistent objects. In such cases, certain RPCs generated
by OP-TEE are routed from the CA to the TEE supplicant for handling (provided
the supplicant is running in the background). Once the supplicant has processed
the request, it responds using ``TEE_IOC_SUPPL_SEND``, and the kernel driver
delivers this response back to the CA in its context.

#. OP-TEE REE time request

In this scenario, the userspace supplicant isn't needed, as the response can be
handled directly by the kernel driver.

An OP-TEE application can request the current time from the NuttX clock using:

   .. code-block:: c

     TEE_GetREETime(&t);

The NuttX kernel driver will respond to the TA with the ``CLOCK_REALTIME``
which represents the machine's best-guess as to the current wall-clock.

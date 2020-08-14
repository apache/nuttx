=========================================
Application OS vs. Internal OS Interfaces
=========================================

NuttX provides a standard, portable OS interface for use by
applications. This standard interface is controlled by the
specifications proved at `OpenGroup.org <http://opengroup.org>`__.
These application interfaces, in general, should not be used
directly by logic executing within the OS. The reason for this is
that there are certain properties of the standard application
interfaces that make them unsuitable for use within the OS These
properties include:

#. **Use of the per-thread** ``errno`` **variable**: Handling of
   return values, particularly, in the case of returned error
   indications. Most legacy POSIX OS interface return information
   via a *per-thread* ``errno``. There must be no alteration of
   the ``errno`` value that must be stable from the point of view
   of the application. So, as a general rule, internal OS logic
   must never modify the ``errno`` and particularly not by the
   inappropriate use of application OS interfaces within OS
   itself.

   Within the OS, functions do not return error information via
   the ``errno`` variable. Instead, the majority of internal OS
   function return error information as an integer value: Returned
   values greater than or equal to zero are success values;
   returned values less than zero indicate failures. Failures are
   reported by returning a negated ``errno`` value from
   ``include/errno.h``,

#. **Cancellation Points**: Many of the application OS interfaces
   are *cancellation points*, i.e., when the task is operating in
   *deferred cancellation* state, it cannot be deleted or
   cancelled until it calls an application OS interface that is a
   cancellation point.

   The POSIX specification is very specific about this, specific
   both in identifying which application OS interfaces are
   cancellation points and specific in the fact that it is
   prohibited for any OS operation other than those listed in the
   specification to generate cancellation points. If internal OS
   logic were to re-use application OS interfaces directly then it
   could very easily violate this POSIX requirement by incorrectly
   generating cancellation points on inappropriate OS operations
   and could result in very difficult to analyze application
   failures.

#. **Use of per-task Resources**: Many resources are only valid in
   the task group context in which a thread operates. Above we
   mentioned one: ``errno`` is only valid for the thread that is
   currently executing. So, for example, the ``errno`` at the time
   of a call is a completely different variable than, say, the
   ``errno`` while running in a work queue task.

   File descriptors are an even better example: An open file on
   file descriptor 5 on task A is *not* the same open file as
   might be used on file descriptor 5 on task B.

   As a result, internal OS logic may not use application OS
   interfaces that use file descriptors or any other *per-task*
   resource.

Within NuttX, this is handled by supporting equivalent internal OS
interfaces that do not break the above rules. These internal
interfaces are intended for use *only* within the OS and should
not be used by application logic. Some examples include:

-  ``nxsem_wait()``: functionally
   equivalent to the standard application interface
   ``sem_wait()``. However, ``nxsem_wait()`` will not modify the
   errno value and will not cause a cancellation point. (see
   ``include/nuttx/semaphore.h`` for other internal OS interfaces
   for semaphores).

-  ``nxsig_waitinfo()``: functionally
   equivalent to the standard application interface
   ``sigwaitinfo()``. However, ``nxsig_waitinfo()`` will not
   modify the errno value and will not cause a cancellation point
   (see ``include/nuttx/signal.h`` for other internal OS
   interfaces for signals).

-  ``nxmq_send()``: functionally equivalent
   to the standard application interface ``mq_send()``. However,
   ``nxmq_send()`` will not modify the errno value and will not
   cause a cancellation point (see ``include/nuttx/mqueue.h`` for
   other internal OS interfaces for POSIX message queues).

-  ``file_read()``: functionally equivalent
   to the standard application interface ``read()``. However,
   ``file_read()`` will not modify the errno value, will not cause
   a cancellation point, and uses a special internal data
   structure in place of the file descriptor (see
   ``include/nuttx/fs/fs.h`` for other internal OS interfaces for
   VFS functions).

-  ``psock_recvfrom()``: functionally
   equivalent to the standard application interface
   ``recvfrom()``. However, ``psock_recvfrom()`` will not modify
   the errno value, will not cause a cancellation point, and uses
   a special internal data structure in place of the socket
   descriptor (see ``include/nuttx/net/net.h`` for other internal
   OS interfaces for sockets).

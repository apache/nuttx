==============================
Named Message Queue Interfaces
==============================

NuttX supports POSIX named message queues for inter-task communication.
Any task may send or receive messages on named message queues. Interrupt
handlers may send messages via named message queues.

  - :c:func:`mq_open`
  - :c:func:`mq_close`
  - :c:func:`mq_unlink`
  - :c:func:`mq_send`
  - :c:func:`mq_timedsend`
  - :c:func:`mq_receive`
  - :c:func:`mq_timedreceive`
  - :c:func:`mq_notify`
  - :c:func:`mq_setattr`
  - :c:func:`mq_getattr`

.. c:function:: mqd_t mq_open(const char *mqName, int oflags, ...)

  Establishes a connection between a named message queue and the calling
  task. After a successful call of mq_open(), the task can reference the
  message queue using the address returned by the call. The message queue
  remains usable until it is closed by a successful call to mq_close().

  :param mqName: Name of the queue to open
  :param oflags: Open flags. These may be any combination of:

     -  ``O_RDONLY``. Open for read access.
     -  ``O_WRONLY``. Open for write access.
     -  ``O_RDWR``. Open for both read & write access.
     -  ``O_CREAT``. Create message queue if it does not already exist.
     -  ``O_EXCL``. Name must not exist when opened.
     -  ``O_NONBLOCK``. Don't wait for data.

  :param ``...``: **Optional parameters**. When the O_CREAT flag is specified,
     POSIX requires that a third and fourth parameter be supplied:

     -  ``mode``. The mode parameter is of type mode_t. In the POSIX
        specification, this mode value provides file permission bits for
        the message queue. This parameter is required but not used in the
        present implementation.
     -  ``attr``. A pointer to an mq_attr that is provided to initialize.
        the message queue. If attr is NULL, then the messages queue is
        created with implementation-defined default message queue
        attributes. If attr is non-NULL, then the message queue mq_maxmsg
        attribute is set to the corresponding value when the queue is
        created. The mq_maxmsg attribute determines the maximum number of
        messages that can be queued before addition attempts to send
        messages on the message queue fail or cause the sender to block;
        the mq_msgsize attribute determines the maximum size of a message
        that can be sent or received. Other elements of attr are ignored
        (i.e, set to default message queue attributes).

  :return: A message queue descriptor or -1 (``ERROR``)

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the full POSIX implementation include:

  -  The mq_msgsize attributes determines the maximum size of a message
     that may be sent or received. In the present implementation, this
     maximum message size is limited at 22 bytes.

.. c:function:: int mq_close(mqd_t mqdes)

  Used to indicate that the calling task is finished with the specified
  message queued ``mqdes``. The ``mq_close()`` deallocates any system
  resources allocated by the system for use by this task for its message
  queue.

  If the calling task has attached a notification request to the message
  queue via this ``mqdes`` (see ``mq_notify()``), this attachment will be
  removed and the message queue is available for another task to attach
  for notification.

  :param mqdes: Message queue descriptor.

  :return: 0 (``OK``) if the message queue is closed successfully, otherwise, -1
     (``ERROR``).

  **Assumptions/Limitations:**

    -  The behavior of a task that is blocked on either a ``mq_send()`` or
       ``mq_receive()`` is undefined when ``mq_close()`` is called.
    -  The result of using this message queue descriptor after successful
       return from ``mq_close()`` is undefined.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int mq_unlink(const char *mqName)

  Removes the message queue named by
  "mqName." If one or more tasks have the message queue open when
  ``mq_unlink()`` is called, removal of the message queue is postponed
  until all references to the message queue have been closed.

  :param mqName: Name of the message queue

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int mq_send(mqd_t mqdes, const void *msg, size_t msglen, int prio)

  Adds the specified message, ``msg``, to the message queue, ``mqdes``.
  The ``msglen`` parameter specifies the length of the message in bytes
  pointed to by ``msg``. This length must not exceed the maximum message
  length from the ``mq_getattr()``.

  If the message queue is not full, ``mq_send()`` will place the ``msg``
  in the message queue at the position indicated by the ``prio`` argument.
  Messages with higher priority will be inserted before lower priority
  messages The value of ``prio`` must not exceed ``MQ_PRIO_MAX``.

  If the specified message queue is full and ``O_NONBLOCK`` is not set in
  the message queue, then ``mq_send()`` will block until space becomes
  available to the queue the message.

  If the message queue is full and ``NON_BLOCK`` is set, the message is
  not queued and ``ERROR`` is returned.

  **NOTE**: ``mq_send()`` may be called from an interrupt handler.
  However, it behaves differently when called from the interrupt level:

  -  It does not check the size of the queue. It will always post the
     message, even if there are already too many messages in queue. This is
     because the interrupt handler does not have the option of waiting for
     the message queue to become non-full.
  -  It doesn't allocate new memory (because you cannot allocate memory
     from an interrupt handler). Instead, there is a pool of pre-allocated
     message structures that may be used just for sending messages from
     interrupt handlers. The number of such pre-allocated messages is set
     by the ``PREALLOC_MQ_IRQ_MSGS`` configuration parameter.

  :param mqdes: Message queue descriptor.
  :param msg: Message to send.
  :param msglen: The length of the message in bytes.
  :param prio: The priority of the message.
  :return: On success, ``mq_send()`` returns 0 (``OK``); on
    error, -1 (``ERROR``) is returned, with ```errno`` <#ErrnoAccess>`__ set
    to indicate the error:

    -  ``EAGAIN``. The queue was empty, and the ``O_NONBLOCK`` flag was set
       for the message queue description referred to by ``mqdes``.
    -  ``EINVAL``. Either ``msg`` or ``mqdes`` is ``NULL`` or the value of
       ``prio`` is invalid.
    -  ``EPERM``. Message queue opened not opened for writing.
    -  ``EMSGSIZE``. ``msglen`` was greater than the ``maxmsgsize``
       attribute of the message queue.
    -  ``EINTR``. The call was interrupted by a signal handler.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int mq_timedsend(mqd_t mqdes, const char *msg, size_t msglen, int prio, \
                const struct timespec *abstime);

  Adds the specified message, ``msg``, to the message queue, ``mqdes``.
  The ``msglen`` parameter specifies the length of the message in bytes
  pointed to by ``msg``. This length must not exceed the maximum message
  length from the ``mq_getattr()``.

  If the message queue is not full, ``mq_timedsend()`` will place the
  ``msg`` in the message queue at the position indicated by the ``prio``
  argument. Messages with higher priority will be inserted before lower
  priority messages The value of ``prio`` must not exceed ``MQ_PRIO_MAX``.

  If the specified message queue is full and ``O_NONBLOCK`` is not set in
  the message queue, then ``mq_send()`` will block until space becomes
  available to the queue the message or until a timeout occurs.

  ``mq_timedsend()`` behaves just like ``mq_send()``, except that if the
  queue is full and the ``O_NONBLOCK`` flag is not enabled for the message
  queue description, then ``abstime`` points to a structure which
  specifies a ceiling on the time for which the call will block. This
  ceiling is an absolute timeout in seconds and nanoseconds since the
  Epoch (midnight on the morning of 1 January 1970).

  If the message queue is full, and the timeout has already expired by the
  time of the call, ``mq_timedsend()`` returns immediately.

  :param mqdes: Message queue descriptor.
  :param msg: Message to send.
  :param msglen: The length of the message in bytes.
  :param prio: The priority of the message.
  :return: On success, ``mq_send()`` returns 0 (``OK``); on
    error, -1 (``ERROR``) is returned, with ```errno`` <#ErrnoAccess>`__ set
    to indicate the error:

    -  ``EAGAIN``. The queue was empty, and the ``O_NONBLOCK`` flag was set
       for the message queue description referred to by ``mqdes``.
    -  ``EINVAL``. Either ``msg`` or ``mqdes`` is ``NULL`` or the value of
       ``prio`` is invalid.
    -  ``EPERM``. Message queue opened not opened for writing.
    -  ``EMSGSIZE``. ``msglen`` was greater than the ``maxmsgsize``
       attribute of the message queue.
    -  ``EINTR``. The call was interrupted by a signal handler.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: ssize_t mq_receive(mqd_t mqdes, void *msg, size_t msglen, int *prio)

  Receives the oldest of the highest priority messages from the message
  queue specified by ``mqdes``. If the size of the buffer in bytes,
  ``msgLen``, is less than the ``mq_msgsize`` attribute of the message
  queue, ``mq_receive()`` will return an error. Otherwise, the selected
  message is removed from the queue and copied to ``msg``.

  If the message queue is empty and ``O_NONBLOCK`` was not set,
  ``mq_receive()`` will block until a message is added to the message
  queue. If more than one task is waiting to receive a message, only the
  task with the highest priority that has waited the longest will be
  unblocked.

  If the queue is empty and ``O_NONBLOCK`` is set, ``ERROR`` will be
  returned.

  :param mqdes: Message Queue Descriptor.
  :param msg: Buffer to receive the message.
  :param msglen: Size of the buffer in bytes.
  :param prio: If not NULL, the location to store message priority.
  :return: On success, the length of the selected message in bytes is
    returned. On failure, -1 (``ERROR``) is returned and the
    ```errno`` <#ErrnoAccess>`__ is set appropriately:

    -  ``EAGAIN`` The queue was empty and the ``O_NONBLOCK`` flag was set
       for the message queue description referred to by ``mqdes``.
    -  ``EPERM`` Message queue opened not opened for reading.
    -  ``EMSGSIZE`` ``msglen`` was less than the ``maxmsgsize`` attribute of
       the message queue.
    -  ``EINTR`` The call was interrupted by a signal handler.
    -  ``EINVAL`` Invalid ``msg`` or ``mqdes``

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: ssize_t mq_timedreceive(mqd_t mqdes, void *msg, size_t msglen, \
                               int *prio, const struct timespec *abstime);

  Receives the oldest of the highest priority messages from the message
  queue specified by ``mqdes``. If the size of the buffer in bytes,
  ``msgLen``, is less than the ``mq_msgsize`` attribute of the message
  queue, ``mq_timedreceive()`` will return an error. Otherwise, the
  selected message is removed from the queue and copied to ``msg``.

  If the message queue is empty and ``O_NONBLOCK`` was not set,
  ``mq_timedreceive()`` will block until a message is added to the message
  queue (or until a timeout occurs). If more than one task is waiting to
  receive a message, only the task with the highest priority that has
  waited the longest will be unblocked.

  ``mq_timedreceive()`` behaves just like ``mq_receive()``, except that if
  the queue is empty and the ``O_NONBLOCK`` flag is not enabled for the
  message queue description, then ``abstime`` points to a structure which
  specifies a ceiling on the time for which the call will block. This
  ceiling is an absolute timeout in seconds and nanoseconds since the
  Epoch (midnight on the morning of 1 January 1970).

  If no message is available, and the timeout has already expired by the
  time of the call, ``mq_timedreceive()`` returns immediately.

  :param mqdes: Message Queue Descriptor.
  :param msg: Buffer to receive the message.
  :param msglen: Size of the buffer in bytes.
  :param prio: If not NULL, the location to store message priority.
  :param abstime: The absolute time to wait until a timeout is declared.

  :return: On success, the length of the selected message in bytes is
    returned. On failure, -1 (``ERROR``) is returned and the
    ```errno`` <#ErrnoAccess>`__ is set appropriately:

    -  ``EAGAIN``: The queue was empty and the ``O_NONBLOCK`` flag was set
       for the message queue description referred to by ``mqdes``.
    -  ``EPERM``: Message queue opened not opened for reading.
    -  ``EMSGSIZE``: ``msglen`` was less than the ``maxmsgsize`` attribute
       of the message queue.
    -  ``EINTR``: The call was interrupted by a signal handler.
    -  ``EINVAL``: Invalid ``msg`` or ``mqdes`` or ``abstime``
    -  ``ETIMEDOUT``: The call timed out before a message could be
       transferred.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int mq_notify(mqd_t mqdes, FAR const struct sigevent *notification)

  If the ``notification`` input parameter is not ``NULL``, this function
  connects the task with the message queue such that the specified signal
  will be sent to the task whenever the message queue changes from empty
  to non-empty. One notification can be attached to a message queue.

  If ``notification``; is ``NULL``, the attached notification is detached
  (if it was held by the calling task) and the queue is available to
  attach another notification.

  When the notification is sent to the registered task, its registration
  will be removed. The message queue will then be available for
  registration.

  :param mqdes: Message queue descriptor
  :param notification: Real-time signal structure containing:

     -  ``sigev_notify``. Should be SIGEV_SIGNAL (but actually ignored)
     -  ``sigev_signo``. The signo to use for the notification
     -  ``sigev_value``. Value associated with the signal

  :return: On success ``mq_notify()`` returns 0; on error, -1
    is returned, with ``errno`` set to indicate the error:

    -  ``EBADF``. The descriptor specified in ``mqdes`` is invalid.
    -  ``EBUSY``. Another process has already registered to receive
       notification for this message queue.
    -  ``EINVAL``. ``sevp->sigev_notify`` is not one of the permitted
       values; or ``sevp->sigev_notify`` is ``SIGEV_SIGNAL`` and
       ``sevp->sigev_signo`` is not a valid signal number.
    -  ``ENOMEM``. Insufficient memory.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the full POSIX implementation include:

  -  The notification signal will be sent to the registered task even if
     another task is waiting for the message queue to become non-empty.
     This is inconsistent with the POSIX specification which states, "If a
     process has registered for notification of message arrival at a
     message queue and some process is blocked in ``mq_receive`` waiting
     to receive a message when a message arrives at the queue, the
     arriving message will satisfy the appropriate ``mq_receive()`` ...
     The resulting behavior is as if the message queue remains empty, and
     no notification will be sent."

.. c:function:: int mq_setattr(mqd_t mqdes, const struct mq_attr *mqStat, \
                      struct mq_attr *oldMqStat);

  Sets the attributes associated with the specified message queue "mqdes."
  Only the "O_NONBLOCK" bit of the "mq_flags" can be changed.

  If ``oldMqStat`` is non-null, mq_setattr() will store the previous message
  queue attributes at that location (just as would have been returned by
  mq_getattr()).

  :param mqdes: Message queue descriptor
  :param mqStat: New attributes
  :param oldMqState: Old attributes

  :return: 0 (``OK``) if attributes are set successfully, otherwise -1
     (``ERROR``).

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int mq_getattr(mqd_t mqdes, struct mq_attr *mqStat)

  Gets status information and attributes associated with the specified
  message queue.

  :param mqdes: Message queue descriptor
  :param mqStat: Buffer in which to return attributes. The returned
     attributes include:

     -  ``mq_maxmsg``. Max number of messages in queue.
     -  ``mq_msgsize``. Max message size.
     -  ``mq_flags``. Queue flags.
     -  ``mq_curmsgs``. Number of messages currently in queue.

  :return: 0 (``OK``) if attributes provided, -1 (``ERROR``) otherwise.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

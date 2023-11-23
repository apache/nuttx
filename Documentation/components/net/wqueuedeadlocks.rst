====================
Work Queue Deadlocks
====================

Use of Work Queues
==================

Most network drivers use a work queue to handle network events. This is done for
two reason: (1) Most of the example code to leverage from does it that way, and (2)
it is easier and is a more efficient use memory resources to use the work queue
rather than creating a dedicated task/thread to service the network.

High and Low Priority Work Queues
=================================

There are two work queues: A single, high priority work queue that is intended
only to service the back end interrupt processing in a semi-normal, tasking
context. And low priority work queue(s) that are similar but as then name implies
are lower in priority and not dedicated for time-critical back end interrupt
processing.

Downsides of Work Queues
========================

There are two important downsides to the use of work queues. First, the work queues
are inherently non-deterministic. The time delay from the point at which you
schedule work and the time at which the work is performed in highly random and
that delay is due not only to the strict priority scheduling but also to what
work as been queued ahead of you.

Why do you bother to use an RTOS if you rely on non-deterministic work queues to do
most of the work?

A second problem is related: Only one work queue job can be performed at a time.
That job should be brief so that it can make the work queue available again for
the next work queue job as soon as possible. And that job should never block
waiting for resources! If the job blocks, then it blocks the entire work queue
and makes the whole work queue unavailable for the duration of the wait.

Networking on Work Queues
=========================

As mentioned, most network drivers use a work queue to handle network events.
(some are even configurable to use high priority work queue... YIKES!). Most
network operations are not really suited for execution on a work queue: The
networking operations can be quite extended and also can block waiting for for
the availability of resources. So, at a minimum, networking should never use
the high priority work queue.

Deadlocks
=========

If there is only a single instance of a work queue, then it is easy to create a
deadlock on the work queue if a work job blocks on the work queue. Here is the
generic work queue deadlock scenario:

* A job runs on a work queue and waits for the availability of a resource.
* The operation that provides that resource also runs on the same work queue.
* But since the work queue is blocked waiting for the resource, the job that
  provides the resource cannot run and a deadlock results.

IOBs
====

IOBs (I/O Blocks) are small I/O buffers that can be linked together in chains to
efficiently buffer variable sized network packet data. This is a much more
efficient use of buffering space than full packet buffers since the packets
content is often much smaller than the full packet size (the MSS).

The network allocates IOBs to support TCP and UDP read-ahead buffering and write
buffering. Read-head buffering is used when TCP/UDP data is received and there is
no receiver in place waiting to accept the data. In this case, the received
payload is buffered in the IOB-based, read-ahead buffers. When the application
next calls ``revc()`` or ``recvfrom()``, the date will be removed from the read-ahead
buffer and returned to the caller immediately.

Write-buffering refers to the similar feature on the outgoing side. When application
calls ``send()`` or ``sendto()`` and the driver is not available to accept the new packet
data, then data is buffered in IOBs in the write buffer chain. When the network
driver is finally available to take more data, then packet data is removed from
the write-buffer and provided to the driver.

The IOBs are allocated with a fixed size. A fixed number of IOBs are pre-allocated
when the system starts. If the network runs out of IOBs, additional IOBs will not
be allocated dynamically, rather, the IOB allocator, ``iob_alloc()`` will block waiting
until an IOB is finally returned to pool of free IOBs. There is also a non-blocking
IOB allocator, ``iob_tryalloc()``.

Under conditions of high utilization, such as sending large amount of data at high
rates or receiving large amounts of data at high rates, it is inevitable that the
system will run out of pre-allocated IOBs. For read-ahead buffering, the packets
are simply dropped in this case. For TCP this means that there will be a subsequent
timeout on the remote peer because no ACK will be received and the remote peer will
eventually re-transmit the packet. UDP is a lossy transfer and handling of lost or
dropped datagrams must be included in any UDP design.

For write-buffering, there are three possible behaviors that can occur when the
IOB pool has been exhausted: First, if there are no available IOBs at the beginning
of a ``send()`` or ``sendto()`` transfer, then the operation will block until IOBs are again
available if ``O_NONBLOCK`` is not selected. This delay can can be a substantial amount
of time.

Second, if ``O_NONBLOCK`` is selected, the send will, of course, return immediately,
failing with errno set ``EAGAIN`` if we cannot allocate the first IOB for the transfer.

The third behavior occurs if the we run out of IOBs in the middle of the transfer.
Then the send operation will not wait but will instead send then number of bytes that
it has successfully buffered. Applications should always check the return value from
``send()`` or ``sendto()``. If it a is a byte count less then the requested transfer
size, then the send function should be called again.

The blocking iob_alloc() call is also the a common cause of work queue deadlocks.
The scenario again is:

* Some logic in the OS runs on a work queue and blocks waiting for an IOB to
  become available,
* The logic that releases the IOB also runs on the same work queue, but
* That logic that provides the IOB cannot execute, however, because the other job
  is blocked waiting for the IOB on the same work queue.

Alternatives to Work Queues
===========================

To avoid network deadlocks here is the rule: Never run the network on a singleton
work queue!

Most network implementation do just that! Here are a couple of alternatives:

#. Use Multiple Low Priority Work Queues
   Unlike the high priority work queues, the low priority work queues utilize a
   thread pool. The number of threads in the pool is controlled by the
   ``CONFIG_SCHED_LPNTHREADS``. If ``CONFIG_SCHED_LPNTHREADS`` is greater than one,
   then such deadlocks should not be possible: In that case, if a thread is busy with
   some other job (even if it is only waiting for a resource), then the job will be
   assigned to a different thread and the deadlock will be broken. The cost of the
   additional low priority work queue thread is primarily the memory set aside for
   the thread's stack.

#. Use a Dedicated Network Thread
   The best solution would be to write a custom kernel thread to handle driver
   network operations. This would be the highest performing and the most manageable.
   It would also, however, but substantially more work.

#. Interactions with Network Locks
   The network lock is a re-entrant mutex that enforces mutually exclusive access to
   the network. The network lock can also cause deadlocks and can also interact with
   the work queues to degrade performance. Consider this scenario:

     * Some network logic, perhaps running on on the application thread, takes the network
       lock then waits for an IOB to become available (on the application thread, not a
       work queue).
     * Some network related event runs on the work queue but is blocked waiting for
       the network lock.
     * Another job is queued behind that network job. This is the one that provides the
       IOB, but it cannot run because the other thread is blocked waiting for the network
       lock on the work queue.

   But the network will not be unlocked because the application logic holds the network
   lock and is waiting for the IOB which can never be released.

   Within the network, this deadlock condition is avoided using a special function
   ``net_ioballoc()``. ``net_ioballoc()`` is a wrapper around the blocking ``iob_alloc()``
   that momentarily releases the network lock while waiting for the IOB to become available.

   Similarly, the network functions ``net_lockedait()`` and ``net_timedait()`` are wrappers
   around ``nxsem_wait()`` ``nxsem_timedwait()``, respectively, and also release the network
   lock for the duration of the wait.

   Caution should be used with any of these wrapper functions. Because the network lock is
   relinquished during the wait, there could changes in the network state that occur before
   the lock is recovered. Your design should account for this possibility.




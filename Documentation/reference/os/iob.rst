=====================
I/O Buffer Management
=====================

NuttX supports generic I/O buffer management (IOB) logic. This
logic was originally added to support network I/O buffering, but
has been generalized to meet buffering requirements by all device
drivers. At the time of this writing, IOBs are currently used not
only be networking but also by logic in ``drivers/syslog`` and
``drivers/wireless``. NOTE that some of the wording in this
section still reflects those legacy roots as a part of the
networking subsystem. This objectives of this feature are:

  #. Provide common I/O buffer management logic for all drivers,
  #. Support I/O buffer allocation from both the tasking and
     interrupt level contexts.
  #. Use a fixed amount of pre-allocated memory.
  #. No costly, non-deterministic dynamic memory allocation.
  #. When the fixed number of pre-allocated I/O buffers is
     exhausted, further attempts to allocate memory from tasking
     logic will cause the task to block and wait until a an I/O
     buffer to be freed.
  #. Each I/O buffer should be small, but can be chained together to
     support buffering of larger thinks such as full size network
     packets.
  #. Support *throttling* logic to prevent lower priority tasks from
     hogging all available I/O buffering.

Configuration Options
=====================

``CONFIG_MM_IOB``
   Enables generic I/O buffer support. This setting will build the
   common I/O buffer (IOB) support library.
``CONFIG_IOB_NBUFFERS``
   Number of pre-allocated I/O buffers. Each packet is represented
   by a series of small I/O buffers in a chain. This setting
   determines the number of preallocated I/O buffers available for
   packet data. The default value is setup for network support.
   The default is 8 buffers if neither TCP/UDP or write buffering
   is enabled (neither ``CONFIG_NET_TCP_WRITE_BUFFERS`` nor
   ``CONFIG_NET_TCP``), 24 if only TCP/UDP is enabled, and 36 if
   both TCP/UDP and write buffering are enabled.
``CONFIG_IOB_BUFSIZE``
   Payload size of one I/O buffer. Each packet is represented by a
   series of small I/O buffers in a chain. This setting determines
   the data payload each preallocated I/O buffer. The default
   value is 196 bytes.
``CONFIG_IOB_NCHAINS``
   Number of pre-allocated I/O buffer chain heads. These tiny
   nodes are used as *containers* to support queueing of I/O
   buffer chains. This will limit the number of I/O transactions
   that can be *in-flight* at any give time. The default value of
   zero disables this features.
   These generic I/O buffer chain containers are not currently
   used by any logic in NuttX. That is because their other other
   specialized I/O buffer chain containers that also carry a
   payload of usage specific information. The default value is
   zero if nether TCP nor UDP is enabled (i.e., neither
   ``CONFIG_NET_TCP`` && !\ ``CONFIG_NET_UDP`` or eight if either
   is enabled.
``CONFIG_IOB_THROTTLE``
   I/O buffer throttle value. TCP write buffering and read-ahead
   buffer use the same pool of free I/O buffers. In order to
   prevent uncontrolled incoming TCP packets from hogging all of
   the available, pre-allocated I/O buffers, a throttling value is
   required. This throttle value assures that I/O buffers will be
   denied to the read-ahead logic before TCP writes are halted.
   The default 0 if neither TCP write buffering nor TCP read-ahead
   buffering is enabled. Otherwise, the default is 8.
``CONFIG_IOB_DEBUG``
   Force I/O buffer debug. This option will force debug output
   from I/O buffer logic. This is not normally something that
   would want to do but is convenient if you are debugging the I/O
   buffer logic and do not want to get overloaded with other
   un-related debug output. NOTE that this selection is not
   available if DEBUG features are not enabled
   (``CONFIG_DEBUG_FEATURES``) with IOBs are being used to syslog
   buffering logic (``CONFIG_SYSLOG_BUFFER``).

Throttling
==========

An allocation throttle was added. I/O buffer allocation logic
supports a throttle value originally for read-ahead buffering to
prevent the read-ahead logic from consuming all available I/O
buffers and blocking the write buffering logic. This throttle
logic is only needed for networking only if both write buffering
and read-ahead buffering are used. Of use of I/O buffering might
have other motivations for throttling.

Public Types
============

This structure represents one I/O buffer. A packet is contained by
one or more I/O buffers in a chain. The ``io_pktlen`` is only
valid for the I/O buffer at the head of the chain.

.. code-block:: c

   struct iob_s
   {
     /* Singly-link list support */

     FAR struct iob_s *io_flink;

     /* Payload */

   #if CONFIG_IOB_BUFSIZE < 256
     uint8_t  io_len;      /* Length of the data in the entry */
     uint8_t  io_offset;   /* Data begins at this offset */
   #else
     uint16_t io_len;      /* Length of the data in the entry */
     uint16_t io_offset;   /* Data begins at this offset */
   #endif
     uint16_t io_pktlen;   /* Total length of the packet */

     uint8_t  io_data[CONFIG_IOB_BUFSIZE];
   };

This container structure supports queuing of I/O buffer chains.
This structure is intended only for internal use by the IOB
module.

.. code-block:: c

   #if CONFIG_IOB_NCHAINS > 0
   struct iob_qentry_s
   {
     /* Singly-link list support */

     FAR struct iob_qentry_s *qe_flink;

     /* Payload -- Head of the I/O buffer chain */

     FAR struct iob_s *qe_head;
   };
   #endif /* CONFIG_IOB_NCHAINS > 0 */

The I/O buffer queue head structure.

.. code-block:: c

   #if CONFIG_IOB_NCHAINS > 0
   struct iob_queue_s
   {
     /* Head of the I/O buffer chain list */

     FAR struct iob_qentry_s *qh_head;
     FAR struct iob_qentry_s *qh_tail;
   };
   #endif /* CONFIG_IOB_NCHAINS > 0 */

Public Function Prototypes
==========================

  - :c:func:`iob_initialize()`
  - :c:func:`iob_alloc()`
  - :c:func:`iob_tryalloc()`
  - :c:func:`iob_free()`
  - :c:func:`iob_free_chain()`
  - :c:func:`iob_add_queue()`
  - :c:func:`iob_tryadd_queue()`
  - :c:func:`iob_remove_queue()`
  - :c:func:`iob_peek_queue()`
  - :c:func:`iob_free_queue()`
  - :c:func:`iob_free_queue_qentry()`
  - :c:func:`iob_get_queue_size()`
  - :c:func:`iob_copyin()`
  - :c:func:`iob_trycopyin()`
  - :c:func:`iob_copyout()`
  - :c:func:`iob_clone()`
  - :c:func:`iob_concat()`
  - :c:func:`iob_trimhead()`
  - :c:func:`iob_trimhead_queue()`
  - :c:func:`iob_trimtail()`
  - :c:func:`iob_pack()`
  - :c:func:`iob_contig()`
  - :c:func:`iob_dump()`

.. c:function:: void iob_initialize(void);

  Set up the I/O buffers for normal operations.

.. c:function:: FAR struct iob_s *iob_alloc(bool throttled);

  Allocate an I/O buffer by taking the buffer at
  the head of the free list.

.. c:function:: FAR struct iob_s *iob_tryalloc(bool throttled);

  Try to allocate an I/O buffer by taking the
  buffer at the head of the free list without waiting for a buffer
  to become free.

.. c:function:: FAR struct iob_s *iob_free(FAR struct iob_s *iob);

  Free the I/O buffer at the head of a buffer chain
  returning it to the free list. The link to the next I/O buffer in
  the chain is return.

.. c:function:: void iob_free_chain(FAR struct iob_s *iob);

  Free an entire buffer chain, starting at the
  beginning of the I/O buffer chain

.. c:function:: int iob_add_queue(FAR struct iob_s *iob, FAR struct iob_queue_s *iobq)

  Add one I/O buffer chain to the end of a queue.
  May fail due to lack of resources.

.. c:function:: void iob_tryadd_queue(FAR struct iob_s *iob, FAR struct iob_queue_s *iobq)

  Add one I/O buffer chain to the end of a queue
  without waiting for resources to become free.

.. c:function:: FAR struct iob_s *iob_remove_queue(FAR struct iob_queue_s *iobq);

  Remove and return one I/O buffer chain from the
  head of a queue.

  :return: Returns a reference to the I/O buffer chain at
    the head of the queue.

.. c:function:: FAR struct iob_s *iob_peek_queue(FAR struct iob_queue_s *iobq)

  Return a reference to the I/O buffer chain at the
  head of a queue. This is similar to iob_remove_queue except that
  the I/O buffer chain is in place at the head of the queue. The I/O
  buffer chain may safely be modified by the caller but must be
  removed from the queue before it can be freed.

  :return: Returns a reference to the I/O buffer chain at
    the head of the queue.

.. c:function:: void iob_free_queue(FAR struct iob_queue_s *qhead);

  Free an entire queue of I/O buffer chains.

.. c:function:: void iob_free_queue_qentry(FAR struct iob_s *iob, \
                  FAR struct iob_queue_s *iobq);

  Queue helper for get the iob queue buffer size.

.. c:function:: unsigned int iob_get_queue_size(FAR struct iob_queue_s *queue);

  Free an iob entire queue of I/O buffer chains.

.. c:function:: int iob_copyin(FAR struct iob_s *iob, FAR const uint8_t *src, \
                  unsigned int len, unsigned int offset, bool throttled);

  Copy data ``len`` bytes from a user buffer into
  the I/O buffer chain, starting at ``offset``, extending the chain
  as necessary.

.. c:function:: int iob_trycopyin(FAR struct iob_s *iob, FAR const uint8_t *src, \
                     unsigned int len, unsigned int offset, bool throttled);

  Copy data ``len`` bytes from a user buffer into
  the I/O buffer chain, starting at ``offset``, extending the chain
  as necessary BUT without waiting if buffers are not available.

.. c:function:: int iob_copyout(FAR uint8_t *dest, FAR const struct iob_s *iob, \
                   unsigned int len, unsigned int offset);

  Copy data ``len`` bytes of data into the user
  buffer starting at ``offset`` in the I/O buffer, returning that
  actual number of bytes copied out.

.. c:function:: int iob_clone(FAR struct iob_s *iob1, FAR struct iob_s *iob2, \
                   bool throttled, bool block);

  Duplicate (and pack) the data in ``iob1`` in
  ``iob2``. ``iob2`` must be empty.

.. c:function:: void iob_concat(FAR struct iob_s *iob1, FAR struct iob_s *iob2)

  Concatenate iob_s chain iob2 to iob1.

.. c:function:: FAR struct iob_s *iob_trimhead(FAR struct iob_s *iob, \
                   unsigned int trimlen)

  Remove bytes from the beginning of an I/O chain.
  Emptied I/O buffers are freed and, hence, the beginning of the
  chain may change.

.. c:function:: FAR struct iob_s *iob_trimhead_queue(FAR struct iob_queue_s *qhead, \
                                        unsigned int trimlen);

  Remove bytes from the beginning of an I/O chain
  at the head of the queue. Emptied I/O buffers are freed and,
  hence, the head of the queue may change.

  This function is just a wrapper around iob_trimhead() that assures
  that the iob at the head of queue is modified with the trimming
  operations.

  :return: The new iob at the head of the queue is
    returned.

.. c:function:: FAR struct iob_s *iob_trimtail(FAR struct iob_s *iob, \
                                        unsigned int trimlen);

  Remove bytes from the end of an I/O chain.
  Emptied I/O buffers are freed NULL will be returned in the special
  case where the entry I/O buffer chain is freed.

.. c:function:: FAR struct iob_s *iob_pack(FAR struct iob_s *iob);

  Pack all data in the I/O buffer chain so that the
  data offset is zero and all but the final buffer in the chain are
  filled. Any emptied buffers at the end of the chain are freed.

.. c:function:: int iob_contig(FAR struct iob_s *iob, unsigned int len);

  Ensure that there is ``len`` bytes of contiguous
  space at the beginning of the I/O buffer chain starting at
  ``iob``.

.. c:function:: void iob_dump(FAR const char *msg, FAR struct iob_s *iob, unsigned int len, \
                 unsigned int offset);

  Dump the contents of a I/O buffer chain


===============
Seqcount
===============

Summary
=======

This an efficient ``Seqlock`` (sequential lock) mechanism suitable for
concurrent scenarios with frequent reads and rare writes.
``Seqlock`` enables lock-free reading while ensuring data consistency
through sequence counting.

Core Features
-------------

1. Lock-Free Reading
~~~~~~~~~~~~~~~~~~~~

- Readers access shared data without acquiring locks, eliminating lock
  contention
- Sequence number tracking detects data modifications during read operations
- Retry mechanism guarantees read consistency

2. Write Protection
~~~~~~~~~~~~~~~~~~~

- Writers utilize atomic operations and interrupt protection for exclusive
  access
- Sequence number parity indicates write state (even: readable, odd: writing)
- SMP environments employ memory barriers for operation ordering

Main Interfaces
---------------

Initialization
~~~~~~~~~~~~~~

.. code-block:: c

   void seqlock_init(seqcount_t *s);

Read Operations
~~~~~~~~~~~~~~~

.. code-block:: c

   uint32_t read_seqbegin(const seqcount_t *s);
   uint32_t read_seqretry(const seqcount_t *s, uint32_t start);

Reader usage pattern:

.. code-block:: c

   uint32_t seq;
   do {
       seq = read_seqbegin(&seqlock);
       // Read shared data
   } while (read_seqretry(&seqlock, seq));

Write Operations
~~~~~~~~~~~~~~~~

.. code-block:: c

   irqstate_t write_seqlock_irqsave(seqcount_t *s);
   void write_sequnlock_irqrestore(seqcount_t *s, irqstate_t flags);

Writer usage pattern:

.. code-block:: c

   irqstate_t flags = write_seqlock_irqsave(&seqlock);
   // Modify shared data
   write_sequnlock_irqrestore(&seqlock, flags);

Technical Details
-----------------

1. Sequence Number Mechanism
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- Sequence counter initializes to 0 (even)
- Write start increments by 1 (becomes odd)
- Write completion increments by 1 (returns to even)

2. Memory Barriers
~~~~~~~~~~~~~~~~~~

- SMP systems utilize appropriate read/write memory barriers
- ``SMP_WMB()``: Write memory barrier
- ``SMP_RMB()``: Read memory barrier
- Ensures operation ordering and memory visibility

3. Atomic Operations
~~~~~~~~~~~~~~~~~~~~

- SMP environments employ atomic read/write and CAS operations
- Prevents data races and maintains consistency

4. Interrupt Protection
~~~~~~~~~~~~~~~~~~~~~~~

- Write operations disable interrupts
- Prevents interrupt handlers from interfering with critical sections

Applicable Scenarios
--------------------

- Read operations significantly outnumber write operations
- Readers can tolerate temporary data inconsistency
- High-performance read operations are required

Performance Advantages
----------------------

- Read operations completely lock-free with minimal overhead
- Significant performance improvement for read-intensive applications

This implementation accounts for differences between SMP and uniprocessor
environments, ensuring correct operation across various configurations.

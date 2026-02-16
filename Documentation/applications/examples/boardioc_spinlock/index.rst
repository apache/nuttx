==================================
BOARDIOC_SPINLOCK Spinlock Example
==================================

Overview
========

This example demonstrates the usage of the BOARDIOC_SPINLOCK board control
interface for managing hardware spinlock operations in NuttX. The BOARDIOC_SPINLOCK
interface provides a low-level mechanism to synchronize access to shared resources
across multiple threads or CPUs using spinlock primitives.

What is BOARDIOC_SPINLOCK?
==========================

BOARDIOC_SPINLOCK is a board control request that allows applications to perform
atomic spinlock operations through the boardctl() interface.

The BOARDIOC_SPINLOCK interface supports three primary operations:

* **BOARDIOC_SPINLOCK_LOCK** - Acquire a spinlock (blocks until available)
* **BOARDIOC_SPINLOCK_TRYLOCK** - Try to acquire a spinlock (non-blocking)
* **BOARDIOC_SPINLOCK_UNLOCK** - Release a spinlock

Prerequisites
=============

The following configuration options must be enabled:

.. code-block:: bash

    # Enable board spinlock support
    CONFIG_BOARDCTL_SPINLOCK=y


Basic Usage
===========

Header Files
------------

Include the following headers in your application:

.. code-block:: c

    #include <sys/boardctl.h>        /* For boardctl() interface */
    #include <nuttx/spinlock.h>      /* For spinlock types */

Data Structures
---------------

The BOARDIOC_SPINLOCK interface uses the following structure:

.. code-block:: c

    struct boardioc_spinlock_s
    {
        int action;                   /* Operation: LOCK, TRYLOCK, or UNLOCK */
        spinlock_t *lock;             /* Pointer to the spinlock variable */
        void *flags;                  /* Optional flags (reserved, set to NULL) */
    };

Actions
-------

The following action values are supported:

.. code-block:: c

    BOARDIOC_SPINLOCK_LOCK        /* Acquire lock (blocks if unavailable) */
    BOARDIOC_SPINLOCK_TRYLOCK     /* Try to acquire (non-blocking) */
    BOARDIOC_SPINLOCK_UNLOCK      /* Release lock */

Return Values
^^^^^^^^^^^^^

* **0** - Success
* **Negative value** - Error code (converted to errno)

Simple Lock Example
-------------------

Here's a basic example of acquiring and releasing a spinlock:

.. code-block:: c

    spinlock_t my_lock;
    struct boardioc_spinlock_s spinlock_op;
    int ret;

    /* Initialize the spinlock */
    spin_lock_init(&my_lock);

    /* Acquire the spinlock */
    spinlock_op.action = BOARDIOC_SPINLOCK_LOCK;
    spinlock_op.lock = &my_lock;
    spinlock_op.flags = NULL;
    
    ret = boardctl(BOARDIOC_SPINLOCK, (uintptr_t)&spinlock_op);
    if (ret == 0) {
        printf("Spinlock acquired successfully\n");
    } else {
        printf("Failed to acquire spinlock: %d\n", ret);
    }

    printf("Inside spinlock\n");

    /* Release the spinlock */
    spinlock_op.action = BOARDIOC_SPINLOCK_UNLOCK;
    ret = boardctl(BOARDIOC_SPINLOCK, (uintptr_t)&spinlock_op);
    if (ret == 0) {
        printf("Spinlock released successfully\n");
    }




OS Data Structures
==================

Scalar Types
************

Many of the types used to communicate with NuttX are simple scalar
types. These types are used to provide architecture independence of the
OS from the application. The scalar types used at the NuttX interface
include:

.. c:type:: pid_t
.. c:type:: size_t
.. c:type:: sigset_t
.. c:type:: time_t

Hidden Interface Structures
***************************

Several of the types used to interface with NuttX are structures that
are intended to be hidden from the application. From the standpoint of
the application, these structures (and structure pointers) should be
treated as simple handles to reference OS resources. These hidden
structures include:

.. c:struct:: tcb_s
.. c:type:: mqd_t
.. c:type:: sem_t
.. c:type:: pthread_key_t

In order to maintain portability, applications should not reference
specific elements within these hidden structures. These hidden
structures will not be described further in this user's manual.

Access to the ``errno`` Variable
********************************

A pointer to the thread-specific ``errno`` value is available through a
function call:

.. c:function:: int *__errno(void)

  ``__errno()`` returns a pointer to the thread-specific
  ``errno`` value. Note that the symbol ``errno`` is defined to be
  ``__errno()`` so that the usual access by referencing the symbol
  ``errno`` will work as expected.

  .. code-block:: c

    #include <errno.h>
    #define errno *__errno()
    int *__errno(void);

  There is a unique, private ``errno`` value for each NuttX task. However,
  the implementation of ``errno`` differs somewhat from the use of
  ``errno`` in most multi-threaded process environments: In NuttX, each
  pthread will also have its own private copy of ``errno`` and the
  ``errno`` will not be shared between pthreads. This is, perhaps,
  non-standard but promotes better thread independence.

    :return: A pointer to the thread-specific ``errno`` value.

User Interface Structures
*************************

.. c:type:: int (*main_t)(int argc, char *argv[])

:c:type:`main_t` defines the type of a task entry point. :c:type:`main_t` is declared in
``sys/types.h``.

.. c:struct:: sched_param

This structure is used to pass scheduling priorities to and from NuttX:

.. code-block:: c

  struct sched_param
  {
   int sched_priority;
  };

.. c:struct:: timespec

This structure is used to pass timing information between the NuttX and
a user application:

.. code-block:: c

  struct timespec
  {
   time_t tv_sec;  /* Seconds */
   long   tv_nsec; /* Nanoseconds */
  };

.. c:struct:: mq_attr

This structure is used to communicate message queue attributes between
NuttX and a MoBY application:

.. code-block:: c

  struct mq_attr {
   size_t       mq_maxmsg;   /* Max number of messages in queue */
   size_t       mq_msgsize;  /* Max message size */
   unsigned     mq_flags;    /* Queue flags */
   size_t       mq_curmsgs;  /* Number of messages currently in queue */
  };

.. note that this gives a warning due to https://github.com/sphinx-doc/sphinx/issues/7819 https://github.com/sphinx-doc/sphinx/pull/8313

.. c:struct:: sigaction

The following structure defines the action to take for given signal:

.. code-block:: c

  struct sigaction
  {
   union
   {
     void (*_sa_handler)(int);
     void (*_sa_sigaction)(int, siginfo_t *, void *);
   } sa_u;
   sigset_t           sa_mask;
   int                sa_flags;
  };
  #define sa_handler   sa_u._sa_handler
  #define sa_sigaction sa_u._sa_sigaction

.. c:struct:: siginfo
.. c:type:: siginfo_t

The following types is used to pass parameters to/from signal handlers:

.. code-block:: c

  typedef struct siginfo
  {
   int          si_signo;
   int          si_code;
   union sigval si_value;
  } siginfo_t;

.. c:union:: sigval

This defines the type of the struct siginfo si_value field and is used
to pass parameters with signals.

.. code-block:: c

  union sigval
  {
   int   sival_int;
   void *sival_ptr;
  };

.. c:struct:: sigevent

The following is used to attach a signal to a message queue to notify a
task when a message is available on a queue.

.. code-block:: c

  struct sigevent
  {
   int          sigev_signo;
   union sigval sigev_value;
   int          sigev_notify;
  };


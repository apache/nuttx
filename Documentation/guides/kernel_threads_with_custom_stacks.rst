=================================
Kernel Threads with Custom Stacks
=================================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Kernel+Threads+with+Custom+Stacks


Background
==========

Under certain conditions, it may be necessary to create a kernel thread whose 
stack lives in some custom memory.  This page provides and example of how that 
would be done:

Example
=======

Here is the body of some function.  It expects to have the following inputs:

1. ``taskname``:  The name of the kernel thread to be started
2. ``stacksize``:  The size of the custom stack
3. ``priority``:  The priority of the kernel thread to be started
4. ``entry_point``:  The entry point of the kernel thread to be started
5. ``argv``:  An optional array of argument strings passed to the kernel thread

.. code-block:: c 

    /* Allocate a TCB for the new kernel thread.  kmm_zalloc() is
    * used to that all fields of the new TCB will be zeroed.
    */

    tcb = (FAR struct task_tcb_s *)kmm_zalloc(sizeof(struct task_tcb_s));
    if (tcb == NULL)
    {
        return -ENOMEM;
    }

    /* Indicate (1) that this is a kernel thread and that (2) a custom
    * stack will be used.
    */

    tcb->flags = TCB_FLAG_TTYPE_KERNEL | TCB_FLAG_CUSTOM_STACK;

    /* Allocate the custom stack for the new kernel thread.
    *
    * Do whatever it takes to get a reference to the custom stack.
    * Here custom_alloc() is used as a placeholder for whatever
    * that may be.
    */

    stack = (FAR uint32_t *)custom_alloc(stacksize);
    if (stack == NULL)
    {
        kmm_free(tcb);
        return -ENOMEM;
    }

    /* Initialize the TCB.  This will initialize all remaining
    * fields of the TCB, associate the stack to the TCB, allocate
    * any additional resources needed by the kernel thread, and
    * place the TCB in a list of inactive tasks.
    */

    ret = task_init((FAR struct tcb_s *)tcb, progname, priority,
                    stack, stacksize, entry_point, argv);
    if (ret < 0)
    {
        kmm_free(tcb);
        custom_free(stack);
        return ret;
    }

    /* Then activate the kernel thread at the provided priority */

    ret = task_activate((FAR struct tcb_s *)tcb);
    if (ret < 0)
    {
        /* nxtask_unit() will undo all of the operations of nxtask_init().
        * It also has the side-effect of freeing the TCB which it assumes
        * was allocated with one of the kmm_malloc()functions.
        */

        nxtask_uninit(tcb);
        custom_free(stack);
        return ret;
    }

    return OK;


Freeing the TCB
===============

Prior to calling ``nxtask_init()``, the TCB  can be freed using the kmm 
allocator, specifically the function ``kmm_free()``.  However, after 
``nxtask_init()`` is called, additional resources will be associated with the 
TCB and you must then call ``nxtask_uninit()`` to free the TCB and all of its 
associated resources. ``kmm_free()`` will be used internally by 
``nxtask_uninit()`` to free the TCB. Note that in any event, the TCB must be 
allocated with one of the ``kmm_malloc()`` allocation functions.

You must never free the TCB after ``nxtask_activate()`` returns successfully.

Freeing the Custom Stack Memory
===============================

The effect of the ``TCB_FLAG_CUSTOM_STACK`` flag is that the OS will not 
attempt to free the custom stack memory if the kernel thread exits, crashes, 
or is killed. Does this matter in your implementation?  Could this result in 
some kind of memory leak?  If any kind of clean-up is required by your 
application to free the custom stack memory, you will probably want to use 
an ``on_exit()`` or ``atexit()`` function to get a callback when the kernel 
thread is terminated.

If ``TCB_FLAG_CUSTOM_STACK`` were not set in the TCB flags, the OS would 
attempt to free the stack using ``kmm_free()`` which is probably not what you 
want in this case.

The actual logic is a slightly more complex and somewhat redundant:

* If ``TCB_FLAG_CUSTOM_STACK`` is set in the TCB flags, no attempt will be made 
  to free the custom stack.
* If ``TCB_FLAG_CUSTOM_STACK`` is not set in the TCB flags, the stack will be 
  de-allocated for the kernel thread only if the stack lies in the kernel 
  memory pool.

So in reality ``TCB_FLAG_CUSTOM_STACK`` may not be necessary.  But the safest 
option is to include it in all cases where you do not expect the custom stack 
to be de-allocated.

You must not free the custom stack after ``nxtask_activate()`` returns 
successfully and until the kernel thread is terminated.
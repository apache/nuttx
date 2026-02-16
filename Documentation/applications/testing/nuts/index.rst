================================
NuttX Unit Test Selection (NUTS)
================================

NUTS is a selection of unit test suites that can be compiled and run on NuttX
devices to verify correct operation of the OS APIs. NUTS depends on the
:doc:`cmocka </applications/testing/cmocka/index>` testing framework in order to
run the unit tests.

Building & Deploying
--------------------

In order to build and deploy NUTS on your target device, you will need to enable
the cmocka library and ``CONFIG_TESTING_NUTS``. Once NUTS is enabled, you will
see a list of available test categories in Kconfig. You may enable the test
categories that you want to run tests for.

Once a test category is enabled, you will have access to toggles for each test
group within the category. For example, if you enable the 'devices' category,
you will then be able to choose if you want to include the tests for
``/dev/zero`` or ``/dev/null``, etc. By default, all test groups within a
category are enabled and must be opted-out of by disabling their toggle.

.. note::

   Some individual tests may have specific dependencies which need to be enabled
   first, so do not assume that the visible list of tests in Kconfig is the
   complete selection (unless you have enabled showing hidden options in
   Kconfig). For example, ``/dev/urandom`` depends on `CONFIG_DEV_URANDOM` to be
   enabled, and will not show in the menu unless the dependency is selected.

Once built and deployed to the target, you can use the following command to
see your test results (example of running ``/dev/zero`` tests only):

.. code:: console

   nsh> nuts
   [==========] /dev/zero: Running 8 test(s).
   [ RUN      ] open_rdonly
   [       OK ] open_rdonly
   [ RUN      ] open_rdwr
   [       OK ] open_rdwr
   [ RUN      ] open_wronly
   [       OK ] open_wronly
   [ RUN      ] readzero
   [       OK ] readzero
   [ RUN      ] readlarge
   [       OK ] readlarge
   [ RUN      ] writezero
   [       OK ] writezero
   [ RUN      ] writelarge
   [       OK ] writelarge
   [ RUN      ] wrrd
   [       OK ] wrrd
   [==========] /dev/zero: 8 test(s) run.
   [  PASSED  ] 8 test(s).

Included Tests
--------------

NUTS tests are split into categories. Within each category there are test
groups/suites which contain all of the individual test cases related to a
specific API.

.. toctree::
   :caption: Test categories

   devices.rst
   dstructs.rst

Test Cases
----------

The output of NUTS displays the pass/fail/skip results of each test case that
has been included in the build. Although the names of the test cases are
intended to describe what the test case is verifying, sometimes more information
is needed. Each test case in NUTS has a corresponding source-code
comment/docstring which includes a description of what the test is verifying.
You can also determine more information by looking at the assertion statements
within the test itself.

Adding New Tests
----------------

In order to add new tests, first take a look at the :doc:`cmocka
</applications/testing/cmocka/index>` API documentation.

Then, you should add a new ``.c`` file under the category that you wish to
extend. All of the test cases are static functions that take a ``void **state``
in order to pass in context from a setup function. Each test case function
should have a docstring comment which describes what the particular test case is
testing (and the name should be as descriptive as possible). Setup and teardown
functions must start with ``setup_`` and ``teardown_`` respectively, where the
suffix is a descriptive name about what is being set up/torn down.

An example of a test case taken from the circular buffer test suite is:

.. code:: c

   static int setup_empty_cbuf(void **state)
   {
     *state = &g_cbuf;
     return circbuf_init(&g_cbuf, g_buf, sizeof(g_buf));
   }

   static int teardown_cbuf(void **state)
   {
     circbuf_uninit(*state);
     if (circbuf_is_init(*state)) return -1;
     return 0;
   }

   /****************************************************************************
    * Name: empty_postinit
    *
    * Description:
    *   Tests that a circular buffer is empty right after being initialized.
    ****************************************************************************/

   static void empty_postinit(void **state)
   {
     struct circbuf_s *cbuf = *state;

     assert_true(circbuf_is_empty(cbuf));
     assert_false(circbuf_is_full(cbuf));
     assert_uint_equal(0, circbuf_used(cbuf));
     assert_uint_equal(CBUF_SIZE, circbuf_space(cbuf));
   }

Each test group must implement a public function for running all of the test
cases, following the convention:

.. code:: c

   int nuts_category_group(void)
   {
     static const struct CMUnitTests tests[] =
     {
       cmocka_unit_test(this_is_test_one),
       cmocka_unit_test_setup_teardown(this_is_test_two, setup_func,
                                       teardown_func),
     };

     return cmocka_run_group_tests_name("group name", tests, NULL, NULL);
   }

where 'category' is the name of the test category, and 'group' is the name of
the test group (i.e. category of 'devices' and group of 'devnull').

Inside the ``tests.h`` header file for the category, include the following under
the public function prototypes:

.. code:: c

   #ifdef CONFIG_TESTING_NUTS_CATEGORY_GROUP
   int nuts_category_group(void);
   #else
   #define nuts_category_group()
   #endif /* CONFIG_TESTING_NUTS_CATEGORY_GROUP */

where ``CONFIG_TESTING_NUTS_CATEGORY_GROUP`` is the Kconfig option to toggle
on/off the test group. You will have to add this to the NUTS Kconfig file.

In order to reduce the amount of special switches in the
``Makefile``/``CMakeLists.txt`` files, all source files in each category
directory are included with a wild card. Thus, to avoid compiling unselected
test groups, each test group's C file should be surrounded in the gaurd:

.. code:: c

   #ifdef CONFIG_TESTING_NUTS_CATEGORY_GROUP

   /* ... your test cases and public test runner function ... */

   #endif /* CONFIG_TESTING_NUTS_CATEGORY_GROUP */

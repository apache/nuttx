======
search
======

Hash Table Functions
====================

The ``search`` subdirectory contains implementations of hash table functions
defined in the POSIX ``search.h`` header. These functions provide a standard
interface for creating, searching, and managing hash tables.

Overview
--------

Hash tables are data structures that provide efficient key-value storage and
retrieval. The implementation in NuttX follows the SysV/XPG4 specification
and provides both reentrant (``_r`` suffix) and non-reentrant versions of
the hash table API.

The hash table uses separate chaining for collision resolution, where each
bucket in the hash table contains a singly-linked list of entries that hash
to the same value.

Functions
---------

Reentrant Functions
^^^^^^^^^^^^^^^^^^^

These functions are thread-safe and operate on user-provided hash table
structures:

``hcreate_r()``
  Create a new hash table.

  **Prototype:**

  .. code-block:: c

     int hcreate_r(size_t nel, FAR struct hsearch_data *htab);

  **Parameters:**

  - ``nel``: The number of elements in the hash table. The actual table size
    will be rounded up to the nearest power of two, with minimum size of 16
    and maximum size of ``2^(sizeof(size_t)*8-6)``.
  - ``htab``: Pointer to the hash table data structure.

  **Returns:**

  - ``1`` on success
  - ``0`` on failure with ``errno`` set to:

    - ``EINVAL``: If a table already exists in ``htab``
    - ``ENOMEM``: If memory allocation fails

  **Description:**

  Creates a hash table with at least ``nel`` buckets. The table size is
  automatically adjusted to be a power of two for efficient hash value
  modulo operations.

``hdestroy_r()``
  Destroy a hash table and free all associated memory.

  **Prototype:**

  .. code-block:: c

     void hdestroy_r(FAR struct hsearch_data *htab);

  **Parameters:**

  - ``htab``: Pointer to the hash table to destroy.

  **Description:**

  Frees all entries in the hash table and the table structure itself.
  After calling this function, the hash table pointer is set to NULL.
  If ``htab->free_entry`` is set, it will be called for each entry to
  free the key and data.

``hsearch_r()``
  Search for an entry in a hash table.

  **Prototype:**

  .. code-block:: c

     int hsearch_r(ENTRY item, ACTION action, FAR ENTRY **retval,
                   FAR struct hsearch_data *htab);

  **Parameters:**

  - ``item``: Entry containing the search key (and data for INSERT).
  - ``action``: Action to perform:

    - ``FIND``: Search for an existing entry
    - ``ENTER``: Insert entry if not found
    - ``DELETE``: Delete the entry if found

  - ``retval``: Location to store pointer to found/created entry.
  - ``htab``: Pointer to the hash table.

  **Returns:**

  - ``1`` on success (entry found, inserted, or deleted)
  - ``0`` on failure (entry not found, or allocation failed)

  **Description:**

  This function searches for an entry with a matching key. The behavior
  depends on the action parameter:

  - ``FIND``: Returns the entry if found, sets ``*retval`` to NULL if not found.
  - ``ENTER``: Returns the entry if found, creates new entry if not found.
  - ``DELETE``: Removes and frees the entry if found.

  The key comparison is done using ``strcmp()``.

``hforeach_r()``
  Iterate over all entries in a hash table.

  **Note:** This is a non-POSIX extension function.

  **Prototype:**

  .. code-block:: c

     void hforeach_r(hforeach_t handle, FAR void *data,
                     FAR struct hsearch_data *htab);

  **Parameters:**

  - ``handle``: Callback function to call for each entry.
  - ``data``: User data passed to the callback function.
  - ``htab``: Pointer to the hash table.

  **Description:**

  Calls the provided callback function for each valid entry in the hash
  table. The callback receives a pointer to the entry and the user data.

Non-Reentrant Functions
^^^^^^^^^^^^^^^^^^^^^^^

These functions operate on a global hash table and are not thread-safe:

- ``hcreate()``
- ``hdestroy()``
- ``hsearch()``

These are implemented in ``hcreate.c`` as wrappers around the reentrant
versions using a global ``hsearch_data`` structure.

Data Structures
---------------

``ENTRY``
  Represents a single hash table entry.

  .. code-block:: c

     typedef struct entry
     {
       FAR char *key;
       FAR void *data;
     } ENTRY;

``struct hsearch_data``
  Hash table control structure.

  .. code-block:: c

     struct hsearch_data
     {
       FAR struct internal_head *htable;
       size_t htablesize;
       CODE void (*free_entry)(FAR ENTRY *entry);
     };

``ACTION``
  Enumeration of possible actions for ``hsearch_r()``.

  .. code-block:: c

     typedef enum
     {
       FIND,
       ENTER,
       DELETE
     } ACTION;

Hash Function
-------------

The implementation uses a customizable hash function pointed to by the
global variable ``g_default_hash``. The default implementation is provided
in ``hash_func.c``.

Implementation Details
----------------------

**Bucket Size:**

- Minimum: 16 buckets (``MIN_BUCKETS``)
- Maximum: ``2^(sizeof(size_t)*8-6)`` buckets (``MAX_BUCKETS``)
- Table size is always a power of two

**Collision Resolution:**

The implementation uses separate chaining with singly-linked lists (``SLIST``)
to handle hash collisions.

**Memory Management:**

- Keys and data are stored as pointers in the ``ENTRY`` structure
- The caller is responsible for managing the lifetime of key and data
- When an entry is deleted, the ``free_entry`` callback is called if set
- The default ``free_entry`` function (``hfree_r``) frees both key and data

**Hash Calculation:**

.. code-block:: c

   hashval = (*g_default_hash)(item.key, strlen(item.key));
   bucket_index = hashval & (htablesize - 1);

The use of bitwise AND instead of modulo operation is possible because
the table size is always a power of two.

Usage Example
-------------

.. code-block:: c

   #include <search.h>
   #include <stdio.h>
   #include <string.h>

   int main(void)
   {
     struct hsearch_data htab = {0};
     ENTRY item;
     ENTRY *found;

     /* Create hash table with 100 elements */
     if (!hcreate_r(100, &htab))
       {
         fprintf(stderr, "Failed to create hash table\n");
         return 1;
       }

     /* Insert entries */
     item.key = strdup("key1");
     item.data = strdup("value1");
     hsearch_r(item, ENTER, &found, &htab);

     item.key = strdup("key2");
     item.data = strdup("value2");
     hsearch_r(item, ENTER, &found, &htab);

     /* Search for an entry */
     item.key = "key1";
     if (hsearch_r(item, FIND, &found, &htab))
       {
         printf("Found: %s = %s\n", found->key, (char *)found->data);
       }

     /* Delete an entry */
     item.key = "key1";
     hsearch_r(item, DELETE, &found, &htab);

     /* Destroy hash table */
     hdestroy_r(&htab);

     return 0;
   }

Standards Compliance
--------------------

**POSIX Standard Functions:**

- ``hcreate_r()``
- ``hdestroy_r()``
- ``hsearch_r()``

**Non-POSIX Extensions:**

- ``hforeach_r()`` - NuttX-specific extension for iterating over hash table entries

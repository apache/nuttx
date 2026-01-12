//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

// ============================================================================
// THREAD-SAFE EXCEPTION STORAGE FOR NUTTX
// ============================================================================
//
// This implementation provides per-thread exception storage using pthread
// thread-local storage (TLS). It is THREAD-SAFE and works correctly with:
//
// 1. Multi-threaded execution with NuttX RTOS task switching
// 2. Concurrent exception handling across different threads
// 3. Nested exceptions within the same thread
// 4. Proper cleanup when threads exit
//
// SAFETY FEATURES:
// - Uses pthread_key_create with destructor for automatic cleanup
// - Thread-safe initialization using pthread_once
// - Per-thread exception state isolation
// - No shared mutable state between threads
//
// NOTE: This implementation requires pthread support to be enabled in NuttX
// (CONFIG_PTHREAD=y)
// ============================================================================

#include "../libcxxabi/src/cxa_exception.h"
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <pthread.h>

namespace __cxxabiv1 {

// pthread key for per-thread exception storage
static pthread_key_t exception_storage_key;
static pthread_once_t key_init_once = PTHREAD_ONCE_INIT;

// Destructor called when thread exits - frees the exception storage
static void exception_storage_destructor(void *ptr) {
  if (ptr) {
    free(ptr);
  }
}

// Initialize pthread key once (thread-safe)
static void init_exception_storage_key() {
  int result =
      pthread_key_create(&exception_storage_key, exception_storage_destructor);
  assert(result == 0 && "Failed to create pthread key for exception storage");
  (void)result; // Suppress unused variable warning in release builds
}

extern "C" {

// Get per-thread exception globals, allocating if necessary
__cxa_eh_globals *__cxa_get_globals() {
  // Ensure pthread key is initialized (thread-safe, happens only once)
  pthread_once(&key_init_once, init_exception_storage_key);

  // Get per-thread exception state from pthread TLS
  __cxa_eh_globals *eh = static_cast<__cxa_eh_globals *>(
      pthread_getspecific(exception_storage_key));

  if (!eh) {
    // First exception in this thread - allocate storage
    eh = static_cast<__cxa_eh_globals *>(malloc(sizeof(__cxa_eh_globals)));
    if (!eh) {
      // Out of memory - this is fatal for exception handling
      assert(false && "Out of memory allocating exception state");
      return nullptr;
    }

    // Initialize to zero
    memset(eh, 0, sizeof(__cxa_eh_globals));

    // Store in pthread TLS for this thread
    int result = pthread_setspecific(exception_storage_key, eh);
    assert(result == 0 && "Failed to set pthread-specific exception storage");
    (void)result;
  }

  return eh;
}

// Fast version - assumes __cxa_get_globals() has been called at least once
__cxa_eh_globals *__cxa_get_globals_fast() {
  // Ensure pthread key is initialized
  pthread_once(&key_init_once, init_exception_storage_key);

  // Get per-thread exception state (may be nullptr if not yet initialized)
  return static_cast<__cxa_eh_globals *>(
      pthread_getspecific(exception_storage_key));
}

} // extern "C"

} // namespace __cxxabiv1

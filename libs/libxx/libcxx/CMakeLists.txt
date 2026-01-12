# ##############################################################################
# libs/libxx/libcxx/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

if(CONFIG_LIBCXX)
  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/libcxx)

    set(LIBCXX_VERSION ${CONFIG_LIBCXX_VERSION})

    FetchContent_Declare(
      libcxx
      DOWNLOAD_NAME "libcxx-${LIBCXX_VERSION}.src.tar.xz"
      DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
      URL "https://github.com/llvm/llvm-project/releases/download/llvmorg-${LIBCXX_VERSION}/libcxx-${LIBCXX_VERSION}.src.tar.xz"
          SOURCE_DIR
          ${CMAKE_CURRENT_LIST_DIR}/libcxx
          BINARY_DIR
          ${CMAKE_BINARY_DIR}/libs/libc/libcxx
          CONFIGURE_COMMAND
          ""
          BUILD_COMMAND
          ""
          INSTALL_COMMAND
          ""
          TEST_COMMAND
          ""
      PATCH_COMMAND
        patch -p1 -d ${CMAKE_CURRENT_LIST_DIR}/libcxx <
        ${CMAKE_CURRENT_LIST_DIR}/0001_fix_stdatomic_h_miss_typedef.patch &&
        patch -p3 -d ${CMAKE_CURRENT_LIST_DIR}/libcxx <
        ${CMAKE_CURRENT_LIST_DIR}/mbstate_t.patch && patch -p1 -d
        ${CMAKE_CURRENT_LIST_DIR}/libcxx <
        ${CMAKE_CURRENT_LIST_DIR}/0001-libcxx-remove-mach-time-h.patch && patch
        -p1 -d ${CMAKE_CURRENT_LIST_DIR}/libcxx <
        ${CMAKE_CURRENT_LIST_DIR}/0001-libcxx-fix-ld-errors.patch && patch -p1
        -d ${CMAKE_CURRENT_LIST_DIR}/libcxx <
        ${CMAKE_CURRENT_LIST_DIR}/0001-Fix-build-error-about-__GLIBC__.patch &&
        patch -p1 -d ${CMAKE_CURRENT_LIST_DIR}/libcxx <
        ${CMAKE_CURRENT_LIST_DIR}/0001-libc-Fix-failures-with-GCC-14-92663.patch
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 30)

    FetchContent_GetProperties(libcxx)

    if(NOT libcxx_POPULATED)
      FetchContent_Populate(libcxx)
    endif()

  endif()

  nuttx_create_symlink(${CMAKE_CURRENT_LIST_DIR}/libcxx/include
                       ${CMAKE_BINARY_DIR}/include/libcxx)

  nuttx_create_symlink(${CMAKE_CURRENT_LIST_DIR}/../__config_site
                       ${CMAKE_BINARY_DIR}/include/libcxx_config/__config_site)

  set_property(
    TARGET nuttx
    APPEND
    PROPERTY NUTTX_CXX_INCLUDE_DIRECTORIES
             ${CMAKE_BINARY_DIR}/include/libcxx
             ${CMAKE_BINARY_DIR}/include/libcxx_config
             ${CMAKE_CURRENT_LIST_DIR}/libcxx/test/support
             ${CMAKE_CURRENT_LIST_DIR}/libcxx/src)

  add_compile_definitions(_LIBCPP_BUILDING_LIBRARY)
  if(CONFIG_LIBSUPCXX_TOOLCHAIN)
    add_compile_definitions(__GLIBCXX__)
  endif()

  if(CONFIG_LIBSUPCXX)
    add_compile_definitions(__GLIBCXX__)
  endif()

  if(CONFIG_LIBCXXABI)
    add_compile_definitions(LIBCXX_BUILDING_LIBCXXABI)
  endif()

  file(GLOB SRCS ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/*.cpp)

  string(REGEX MATCH "[0-9]+$" CPP_STD_VER "${CONFIG_CXX_STANDARD}")
  if(${CPP_STD_VER} GREATER_EQUAL 20)
    file(GLOB SRCSTMP ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/experimental/*.cpp)
    list(APPEND SRCS ${SRCSTMP})
    file(GLOB SRCSTMP ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/filesystem/*.cpp)
    list(APPEND SRCS ${SRCSTMP})
    file(GLOB SRCSTMP ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/ryu/*.cpp)
    list(APPEND SRCS ${SRCSTMP})
  endif()

  if(${CPP_STD_VER} LESS_EQUAL 14)
    file(
      GLOB
      SRCSTMP
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/any.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/variant.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/optional.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/memory_resource.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/pstl/*.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/legacy_debug_handler.cpp)
    list(REMOVE_ITEM SRCS ${SRCSTMP})
  endif()

  if(CONFIG_CXX_NO_LOCALIZATION)
    file(
      GLOB
      SRCSTMP
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/ios.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/ios.instantiations.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/iostream.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/locale.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/regex.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/strstream.cpp)
    list(REMOVE_ITEM SRCS ${SRCSTMP})
  endif()

  if(NOT CONFIG_TRICORE_TOOLCHAIN_TASKING)
    set(FLAGS -Wno-attributes -Wno-deprecated-declarations -Wno-shadow
              -Wno-sign-compare -Wno-cpp)
  endif()

  if(GCCVER GREATER_EQUAL 12)
    list(APPEND FLAGS -Wno-maybe-uninitialized -Wno-alloc-size-larger-than)
  endif()

  nuttx_add_system_library(libcxx)
  target_sources(libcxx PRIVATE ${SRCS})
  target_compile_options(libcxx PRIVATE ${FLAGS})
  target_include_directories(libcxx BEFORE
                             PRIVATE ${CMAKE_CURRENT_LIST_DIR}/libcxx/src)

  if(CONFIG_LIBCXX_TEST)
    list(
      APPEND
      FLAGS
      -Wno-array-bounds
      -Wno-permissive
      -Wno-unused-variable
      -Wno-unused-but-set-variable
      -Wno-psabi
      -Wno-unused-local-typedefs
      -Wno-unused-result
      -Wno-unused-function
      -Wno-undef
      -Wno-return-type
      -Wno-self-move
      -Wno-class-memaccess
      -Wno-tautological-compare
      -Wno-narrowing
      -Wno-use-after-free
      -Wno-invalid-memory-model
      -Wno-mismatched-new-delete)
    add_compile_options(-UNDEBUG)
    add_compile_definitions(
      DISABLE_NEW_COUNT
      _LIBCPP_ABI_INCOMPLETE_TYPES_IN_DEQUE
      _LIBCPP_ENABLE_DEBUG_MODE
      _LIBCPP_ENABLE_CXX17_REMOVED_BINDERS
      _LIBCPP_ENABLE_CXX17_REMOVED_AUTO_PTR
      _LIBCPP_ENABLE_CXX20_REMOVED_ALLOCATOR_MEMBERS
      _LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS
      _LIBCPP_DISABLE_DEPRECATION_WARNINGS)

    set(i 0)
    set(TEST_DIR "${CMAKE_CURRENT_LIST_DIR}/libcxx/test")
    file(GLOB_RECURSE TESTS "${TEST_DIR}/*.pass.cpp")
    file(
      GLOB
      UNSUPPORTED_TESTS
      # error: ‘_wremove’ was not declared in this scope
      ${TEST_DIR}/libcxx/input.output/file.streams/fstreams/ifstream.members/open_wchar_pointer.pass.cpp
      ${TEST_DIR}/libcxx/input.output/file.streams/fstreams/ofstream.members/open_wchar_pointer.pass.cpp
      ${TEST_DIR}/libcxx/input.output/file.streams/fstreams/ofstream.cons/wchar_pointer.pass.cpp
      ${TEST_DIR}/libcxx/input.output/file.streams/fstreams/fstream.members/open_wchar_pointer.pass.cpp
      ${TEST_DIR}/libcxx/input.output/file.streams/fstreams/fstream.cons/wchar_pointer.pass.cpp
      # error: ‘x’ in ‘struct Foo’ does not name a type
      ${TEST_DIR}/libcxx/selftest/compile.pass.cpp/compile-error.compile.pass.cpp
      ${TEST_DIR}/libcxx/selftest/pass.cpp/compile-error.pass.cpp
      ${TEST_DIR}/libcxx/selftest/link.pass.cpp/compile-error.link.pass.cpp
      # error: ‘__sanitizer_verify_contiguous_container’ was not declared in
      # this scope
      ${TEST_DIR}/libcxx/containers/sequences/vector/asan.pass.cpp
      # error: attributes are not allowed on a function-definition
      ${TEST_DIR}/libcxx/thread/thread.mutex/thread_safety_requires_capability.pass.cpp
      # fatal error: Block.h: No such file or directory
      ${TEST_DIR}/libcxx/utilities/function.objects/func.blocks.pass.cpp
      # link error
      ${TEST_DIR}/libcxx/selftest/compile.pass.cpp/link-error.compile.pass.cpp
      ${TEST_DIR}/libcxx/selftest/link.pass.cpp/link-error.link.pass.cpp
      ${TEST_DIR}/libcxx/selftest/pass.cpp/link-error.pass.cpp
      # std::views::join
      ${TEST_DIR}/std/ranges/range.adaptors/range.join.view/adaptor.pass.cpp
      ${TEST_DIR}/std/library/description/conventions/customization.point.object/cpo.compile.pass.cpp
      ${TEST_DIR}/std/algorithms/alg.modifying.operations/alg.copy/ranges.copy.segmented.pass.cpp
      ${TEST_DIR}/std/algorithms/alg.modifying.operations/alg.copy/ranges.copy_backward.pass.cpp
      # no build __partition_chunks
      ${TEST_DIR}/libcxx/algorithms/pstl.libdispatch.chunk_partitions.pass.cpp
      # REQUIRES: c++98 || c++03 || c++11 || c++14
      ${TEST_DIR}/std/containers/associative/multimap/multimap.value_compare/types.pass.cpp
      ${TEST_DIR}/std/containers/associative/map/map.value_compare/types.pass.cpp
      # std::result_of
      ${TEST_DIR}/std/utilities/meta/meta.trans/meta.trans.other/result_of.pass.cpp
      ${TEST_DIR}/std/utilities/meta/meta.trans/meta.trans.other/result_of11.pass.cpp
      ${TEST_DIR}/std/utilities/function.objects/func.invoke/invoke.pass.cpp
      ${TEST_DIR}/std/utilities/function.objects/func.invoke/invoke_constexpr.pass.cpp
      # WINT_MIN undef
      ${TEST_DIR}/std/language.support/cstdint/cstdint.syn/cstdint.pass.cpp
      ${TEST_DIR}/std/input.output/file.streams/c.files/cinttypes.pass.cpp
      # static assertion failed
      ${TEST_DIR}/std/language.support/support.runtime/cstdlib.pass.cpp
      # error: static assertion failed: Types differ unexpectedly
      ${TEST_DIR}/std/strings/c.strings/cstring.pass.cpp
      ${TEST_DIR}/std/strings/c.strings/cwchar.pass.cpp
      # error: ‘is_literal_type_v’ is not a member of ‘std’
      ${TEST_DIR}/std/utilities/meta/meta.unary/meta.unary.prop/is_literal_type.pass.cpp
      ${TEST_DIR}/libcxx/containers/sequences/vector/asan.pass.cpp
      # error: static assertion failed
      ${TEST_DIR}/libcxx/utilities/utility/pairs/pairs.pair/non_trivial_copy_move_ABI.pass.cpp
      # unsport c++20
      ${TEST_DIR}/std/utilities/memory/default.allocator/allocator_types.void.compile.pass.cpp
      ${TEST_DIR}/std/utilities/memory/util.smartptr/util.smartptr.shared/util.smartptr.shared.const/auto_ptr.pass.cpp
      ${TEST_DIR}/std/utilities/memory/util.smartptr/util.smartptr.shared/util.smartptr.shared.assign/auto_ptr_Y.pass.cpp
      ${TEST_DIR}/std/utilities/utility/pairs/pairs.pair/assign_pair_cxx03.pass.cpp
      ${TEST_DIR}/std/utilities/function.objects/func.require/binary_function.pass.cpp
      ${TEST_DIR}/std/utilities/function.objects/func.require/unary_function.pass.cpp
      ${TEST_DIR}/std/utilities/function.objects/refwrap/weak_result.pass.cpp
      ${TEST_DIR}/std/utilities/function.objects/refwrap/binder_typedefs.compile.pass.cpp
      ${TEST_DIR}/std/language.support/support.dynamic/new.delete/new.delete.array/sized_delete_array11.pass.cpp
      ${TEST_DIR}/std/language.support/support.dynamic/new.delete/new.delete.single/sized_delete11.pass.cpp
      # arm build error
      ${TEST_DIR}/std/numerics/cfenv/cfenv.syn/cfenv.pass.cpp
      ${TEST_DIR}/std/utilities/format/format.formatter/format.formatter.spec/formatter.floating_point.pass.cpp
      ${TEST_DIR}/std/localization/locales/locale.convenience/conversions/conversions.buffer/seekoff.pass.cpp
    )

    file(
      GLOB_RECURSE
      SKIP_TESTS
      ${TEST_DIR}/libcxx/containers/views/mdspan/*.pass.cpp
      ${TEST_DIR}/libcxx/input.output/iostream.format/print.fun/*.pass.cpp
      ${TEST_DIR}/libcxx/ranges/range.factories/range.repeat.view/*.pass.cpp
      ${TEST_DIR}/libcxx/utilities/expected/*.pass.cpp
      # skip deprecated test
      ${TEST_DIR}/libcxx/depr/*.pass.cpp
      ${TEST_DIR}/std/depr/*.pass.cpp
      ${TEST_DIR}/std/utilities/function.objects/func.wrap/func.wrap.func/func.wrap.func.con/*.pass.cpp
      # arm atomic undef
      ${TEST_DIR}/std/atomics/atomics.types.operations/atomics.types.operations.req/*.pass.cpp
      ${TEST_DIR}/std/atomics/atomics.types.operations/atomics.types.operations.wait/*.pass.cpp
    )
    list(REMOVE_ITEM TESTS ${UNSUPPORTED_TESTS} ${SKIP_TESTS})
    foreach(TEST ${TESTS})
      if(EXISTS ${TEST})
        set(GREP_PATTERNS
            "UNSUPPORTED: c++03, c++11, c++14, c++17, c++20"
            "TEST_LIBCPP_ASSERT_FAILURE"
            "static_assert(test"
            "UNSUPPORTED: GCC-ALWAYS_INLINE-FIXME"
            "UNSUPPORTED: gcc"
            "include <sstream>"
            "std::ranges::join_view"
            "std::binary_function"
            "std::unary_function"
            "std::binary_negate"
            "std::not1"
            "std::not2"
            "std::unary_negate"
            "std::raw_storage_iterator"
            "void operator delete"
            "filesystem_test_helper.h"
            "asan_testing.h")

        set(SKIP_TEST FALSE)

        foreach(PATTERN IN LISTS GREP_PATTERNS)
          execute_process(COMMAND grep -q "${PATTERN}" ${TEST}
                          RESULT_VARIABLE grep_result)

          if(grep_result EQUAL 0)
            set(SKIP_TEST TRUE)
            break()
          endif()
        endforeach()

        if(SKIP_TEST)
          continue()
        endif()

        execute_process(COMMAND grep -q "int main" ${TEST}
                        RESULT_VARIABLE grep_result)
        if(NOT grep_result EQUAL 0)
          continue()
        endif()

        get_filename_component(TEST_NAME ${TEST} NAME_WE)
        foreach(CHARACTER "+" "-" "=" "[]")
          string(REPLACE "${CHARACTER}" "_" TEST_NAME "${TEST_NAME}")
        endforeach()

        nuttx_add_application(
          NAME
          ${TEST_NAME}_${i}
          SRCS
          ${TEST}
          STACKSIZE
          102400
          COMPILE_FLAGS
          ${FLAGS})
        set_source_files_properties(
          ${TEST_DIR}/libcxx/containers/sequences/vector/exception_safety_exceptions_disabled.pass.cpp
          PROPERTIES COMPILE_FLAGS -fno-exceptions)
        math(EXPR i "${i}+1")
      endif()
    endforeach()
  endif()

endif()

# nuttx_add_symtab
#  Generates a symbol table of undefined symbols from a set of binaries
#
# Parameters:
# - NAME: name of symtab (output will be symtab_${NAME}.c)
# - BINARIES: list of binary target names to process (dependencies will be added to these targets)
# - PREFIX: optional prefix to add to symtab variable name
# - EXCLUDE: optional list of symbols to exclude (ie: assume they are defined)

function(nuttx_add_symtab)
  nuttx_parse_function_args(
    FUNC nuttx_add_symtab
    ONE_VALUE NAME PREFIX
    OPTIONS HEADER
    MULTI_VALUE EXCLUDE BINARIES
    REQUIRED NAME BINARIES ARGN ${ARGN}
  )

  # get path to binaries
  set(BINARY_PATHS)
  foreach(binary ${BINARIES})
    # this way of getting the path will select the actual ELF binary, even when this was built
    # by means of an intermediate library on non-ELF platforms
    list(APPEND BINARY_PATHS $<TARGET_PROPERTY:${binary},ELF_BINARY>)
  endforeach()

  if (EXCLUDE)
    string(REPLACE ";" " " EXCLUDE_STRING ${EXCLUDE})
  endif()

  # generate list of undefined symbols
  add_custom_command(
    OUTPUT symtab_${NAME}.dat
    COMMAND ${CMAKE_NM} ${BINARY_PATHS} | fgrep ' U ' | sed -e "s/^[ ]*//g" | cut -d' ' -f2 | sort | uniq > symtab_${NAME}.dat
    COMMAND if [ \"${EXCLUDE}\" != \"\" ]\; then fgrep -v -x ${EXCLUDE_STRING} symtab_${NAME}.dat > symtab_${NAME}.dat2\; mv symtab_${NAME}.dat2 symtab_${NAME}.dat\; fi
    DEPENDS ${BINARIES}
  )

  add_custom_target(symtab_${NAME}_dat DEPENDS symtab_${NAME}.dat)

  # generate declarations for symbols and symtab entries as headers
  add_custom_command(
    OUTPUT symtab_${NAME}_declarations.h symtab_${NAME}_entries.h
    COMMAND sed -E 's|\(.+\)|extern void *\\1\;|g' symtab_${NAME}.dat > symtab_${NAME}_declarations.h
    COMMAND sed -E 's|\(.+\)|{ \"\\1\", \\&\\1 },|g' symtab_${NAME}.dat > symtab_${NAME}_entries.h
    DEPENDS symtab_${NAME}_dat
  )

  # generate code which instantiates the symbol table
  configure_file(${CMAKE_SOURCE_DIR}/cmake/symtab.c.in symtab_${NAME}.c @ONLY)

  # define an internal library to build the symtab file on its own
  add_library(symtab_${NAME} OBJECT ${CMAKE_CURRENT_BINARY_DIR}/symtab_${NAME}.c)

  # Make the dependance between .c and .h explicit. This is necessary since using configure_file()
  # does not seem to allow this to be automatically guessed by CMake
  set_property(SOURCE ${CMAKE_CURRENT_BINARY_DIR}/symtab_${NAME}.c APPEND PROPERTY OBJECT_DEPENDS
    ${CMAKE_CURRENT_BINARY_DIR}/symtab_${NAME}_declarations.h
    ${CMAKE_CURRENT_BINARY_DIR}/symtab_${NAME}_entries.h)

  nuttx_add_library_internal(symtab_${NAME})

  if (PREFIX)
    target_compile_definitions(symtab_${NAME} PRIVATE -DSYMTAB_PREFIX=${PREFIX})
  endif()
endfunction()

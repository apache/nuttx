# nuttx_add_romfs
#  Generates a ROMFS image in a C array, which is built to an OBJECT
#  library.
#
# Parameters:
# - NAME: determines the romfs label and name of target (romfs_${NAME})
# - PATH: the directory that will be used to create the ROMFS from
# - FILES: paths to files to copy into CROMFS
# - HEADER: option to indicate that a .h file is to be generated instead of a .c
# - PREFIX: optional prefix to add to image name (as romfs_${PREFIX}.img)
# - NONCONST: option to indicate the array should be non-const
# - DEPENDS: list of targets that should be depended on

function(nuttx_add_romfs)
  nuttx_parse_function_args(
    FUNC nuttx_add_romfs
    ONE_VALUE NAME MOUNTPOINT PATH PREFIX
    OPTIONS HEADER NONCONST
    MULTI_VALUE DEPENDS FILES
    REQUIRED NAME ARGN ${ARGN}
  )

  if (NOT PATH AND NOT FILES)
    message(FATAL_ERROR "Either PATH or FILES must be specified")
  endif()

  if (HEADER)
    set(EXTENSION h)
  else()
    set(EXTENSION c)
  endif()

  if (PREFIX)
    set(IMGNAME ${PREFIX}.img)
  else()
    set(IMGNAME romfs.img)
  endif()

  add_custom_command(
    OUTPUT romfs_${NAME}.${EXTENSION}
    COMMAND ${CMAKE_COMMAND} -E make_directory romfs_${NAME}
    COMMAND if \[ \"${PATH}\" != \"\" \]; then ${CMAKE_COMMAND} -E copy_directory ${PATH} romfs_${NAME} \; fi
    COMMAND if \[ \"${FILES}\" != \"\" \]; then ${CMAKE_COMMAND} -E copy ${FILES} romfs_${NAME} \; fi
    COMMAND genromfs -f ${IMGNAME} -d romfs_${NAME} -V ${NAME}
    COMMAND xxd -i ${IMGNAME} romfs_${NAME}.${EXTENSION}
    COMMAND ${CMAKE_COMMAND} -E remove ${IMGNAME}
    COMMAND ${CMAKE_COMMAND} -E remove_directory romfs_${NAME}
    COMMAND if ! [ -z "${NONCONST}" ]\; then sed -E -i'' -e "s/^unsigned/const unsigned/g" romfs_${NAME}.${EXTENSION} \; fi
    DEPENDS ${DEPENDS})

  if (NOT HEADER)
    add_library(romfs_${NAME} OBJECT romfs_${NAME}.c)
    nuttx_add_library_internal(romfs_${NAME})
  endif()
endfunction()

# nuttx_add_cromfs
#  Generates a CROMFS image in a C array, which is built to an OBJECT
#  library.
#
# Parameters:
# - NAME: determines the name of target (cromfs_${NAME})
# - PATH: the directory that will be used to create the CROMFS
# - FILES: paths to files to copy into CROMFS
# - DEPENDS: list of targets that should be depended on

function(nuttx_add_cromfs)
  nuttx_parse_function_args(
    FUNC nuttx_add_cromfs
    ONE_VALUE NAME MOUNTPOINT PATH
    MULTI_VALUE DEPENDS FILES
    OPTIONS REQUIRED NAME ARGN ${ARGN}
  )

  if (NOT PATH AND NOT FILES)
    message(FATAL_ERROR "Either PATH or FILES must be specified")
  endif()

  add_custom_command(
    OUTPUT cromfs_${NAME}.c
    COMMAND ${CMAKE_COMMAND} -E make_directory cromfs_${NAME}
    COMMAND if \[ \"${PATH}\" != \"\" \]; then ${CMAKE_COMMAND} -E copy_directory ${PATH} cromfs_${NAME} \; fi
    COMMAND if \[ \"${FILES}\" != \"\" \]; then ${CMAKE_COMMAND} -E copy ${FILES} cromfs_${NAME} \; fi
    COMMAND ${CMAKE_BINARY_DIR}/bin/gencromfs cromfs_${NAME} cromfs_${NAME}.c
    DEPENDS ${DEPENDS}
  )

  add_library(cromfs_${NAME} OBJECT cromfs_${NAME}.c)
  nuttx_add_library_internal(cromfs_${NAME})
endfunction()

# nuttx_add_romfs
#  Generates a ROMFS image in a C array, which is built to an OBJECT
#  library.
#
# Parameters:
# - NAME: determines the romfs label and name of target (romfs_${NAME})
# - PATH: the directory that will be used to create the ROMFS

function(nuttx_add_romfs)
  nuttx_parse_function_args(
    FUNC nuttx_add_romfs
    ONE_VALUE NAME MOUNTPOINT PATH PREFIX
    OPTIONS HEADER NONCONST
    REQUIRED NAME PATH ARGN ${ARGN}
  )

  if (HEADER)
    set(EXTENSION h)
  else()
    set(EXTENSION c)
  endif()

  if (PREFIX)
    set(IMGNAME romfs_${PREFIX}.img)
  else()
    set(IMGNAME romfs.img)
  endif()

  file(MAKE_DIRECTORY romfs_${NAME})
  file(COPY ${PATH}/ DESTINATION romfs_${NAME})

  # Do this separately, otherwise genromfs appears to complete before
  # closing the output file
  execute_process(COMMAND genromfs -f ${IMGNAME} -d romfs_${NAME} -V ${NAME} WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  execute_process(COMMAND xxd -i ${IMGNAME} romfs_${NAME}.${EXTENSION}  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

  if (NOT NONCONST)
    execute_process(COMMAND sed -ri "s/^unsigned/const unsigned/g" romfs_${NAME}.${EXTENSION} WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  endif()

  file(REMOVE ${IMGNAME})

  if (NOT HEADER)
    add_library(romfs_${NAME} OBJECT romfs_${NAME}.c)
    add_dependencies(romfs_${NAME} nuttx_context)
  endif()
endfunction()

# nuttx_add_cromfs
#  Generates a CROMFS image in a C array, which is built to an OBJECT
#  library.
#
# Parameters:
# - NAME: determines the name of target (cromfs_${NAME})
# - PATH: the directory that will be used to create the CROMFS

function(nuttx_add_cromfs)
  nuttx_parse_function_args(
    FUNC nuttx_add_cromfs
    ONE_VALUE NAME MOUNTPOINT PATH
    OPTIONS REQUIRED NAME PATH ARGN ${ARGN}
  )

  add_custom_command(
    OUTPUT cromfs_${NAME}.c
    COMMAND ${CMAKE_COMMAND} -E make_directory cromfs_${NAME}
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${PATH} cromfs_${NAME}
    COMMAND ${CMAKE_BINARY_DIR}/bin/gencromfs cromfs_${NAME} cromfs_${NAME}.c
  )

  add_library(cromfs_${NAME} OBJECT cromfs_${NAME}.c)
  add_dependencies(cromfs_${NAME} nuttx_context)
endfunction()

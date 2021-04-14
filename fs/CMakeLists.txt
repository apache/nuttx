
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})

set(SRCS)

add_subdirectory(aio)
add_subdirectory(binfs)
add_subdirectory(cromfs)
add_subdirectory(dirent)
add_subdirectory(driver)
add_subdirectory(fat)
add_subdirectory(hostfs)
add_subdirectory(inode)
add_subdirectory(mmap)
add_subdirectory(mount)
add_subdirectory(mqueue)
add_subdirectory(nfs)
add_subdirectory(nxffs)
add_subdirectory(partition)
add_subdirectory(procfs)
add_subdirectory(romfs)
add_subdirectory(semaphore)
add_subdirectory(shm)
add_subdirectory(smartfs)
add_subdirectory(spiffs)
add_subdirectory(tmpfs)
add_subdirectory(unionfs)
add_subdirectory(userfs)
add_subdirectory(vfs)

nuttx_add_library(fs
	fs_initialize.c
	${SRCS}
)
target_include_directories(fs INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

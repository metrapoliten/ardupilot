cmake_minimum_required(VERSION 3.5)
project(Ardupilot)

add_compile_options(-std=c++11)
#set(TARGET_PLATFORM sitl)
set(TARGET_PLATFORM chibios)
#set(TARGET_PLATFORM linux)

string(TOUPPER ${TARGET_PLATFORM} TARGET_PLATFORM_UPPER)
# To work with SITL
add_definitions(-DSKETCHBOOK="./")
add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_${TARGET_PLATFORM_UPPER})
if(${TARGET_PLATFORM} MATCHES sitl)
  add_definitions(-DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE)
  set(SUB_TARGET_PLATFORM sitl)
elseif(${TARGET_PLATFORM} MATCHES chibios)
  add_definitions(-DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_CHIBIOS_FMUV3)
  set(SUB_TARGET_PLATFORM fmuv3)
endif()
add_definitions(-DHAL_WITH_UAVCAN)

set(AP_VERSION_DIRECTORY_ROOT build/${SUB_TARGET_PLATFORM})
set(MAVLINK_HEADER_DIRECTORY_ROOT ${AP_VERSION_DIRECTORY_ROOT}/libraries/GCS_MAVLink)
set(MAVLINK_HEADER_DIRECTORY ${MAVLINK_HEADER_DIRECTORY_ROOT}/include/mavlink/v2.0)


# we just grab all the cpp and h files. not ideal, but works well enough
# means a "reload CMake project" when you add new files
file(GLOB_RECURSE APMrover2_FILES APMrover2/*.cpp APMrover2/*.h)
file(GLOB_RECURSE ArduCopter_FILES ArduCopter/*.cpp ArduCopter/*.h)
file(GLOB_RECURSE ArduPlane_FILES ArduPlane/*.cpp ArduPlane/*.h)
file(GLOB_RECURSE libraries_FILES libraries/*.cpp libraries/*.h)
file(GLOB_RECURSE MAVLINK_HEADER_FILES ${MAVLINK_HEADER_DIRECTORY}/*.h)

set(SOURCE_FILES ${APMrover2_FILES} ${ArduCopter_FILES} ${ArduPlane_FILES} ${libraries_FILES} ${MAVLINK_HEADER_FILES})

include_directories(APMrover2)
include_directories(ArduCopter)
include_directories(ArduPlane)
include_directories(libraries)
include_directories(modules)
include_directories(${MAVLINK_HEADER_DIRECTORY})
include_directories(build/${SUB_TARGET_PLATFORM})

add_executable(fake-target-for-clion ${SOURCE_FILES} ${MAVLINK_HEADER_FILES})
set_target_properties(fake-target-for-clion PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(fake-target-for-clion PUBLIC ${MAVLINK_HEADER_DIRECTORY} ${MAVLINK_HEADER_DIRECTORY_ROOT} ${AP_VERSION_DIRECTORY_ROOT})

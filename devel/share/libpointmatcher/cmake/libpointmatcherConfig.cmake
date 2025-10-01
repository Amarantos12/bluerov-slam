# - Config file for the libpointmatcher package
# It defines the following variables
#  libpointmatcher_INCLUDE_DIRS - include directories for pointmatcher
#  libpointmatcher_LIBRARIES    - libraries to link against

include(CMakeFindDependencyMacro)
find_dependency(libnabo REQUIRED)
find_package(Boost COMPONENTS thread filesystem system program_options date_time REQUIRED)
if (Boost_MINOR_VERSION GREATER 47)
  find_package(Boost COMPONENTS thread filesystem system program_options date_time chrono REQUIRED)
endif ()
include(${CMAKE_CURRENT_LIST_DIR}/libpointmatcher-config.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/yaml-cpp-pm-targets.cmake)

# Compute paths
get_filename_component(POINTMATCHER_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(libpointmatcher_INCLUDE_DIRS "/home/hzr/catkin_ws/devel/include;/usr/include/eigen3")
set(LIBPOINTMATCHER_INCLUDE_DIRS "/home/hzr/catkin_ws/devel/include;/usr/include/eigen3")
set(pointmatcher_INCLUDE_DIRS "/home/hzr/catkin_ws/devel/include;/usr/include/eigen3")
set(POINTMATCHER_INCLUDE_DIRS "/home/hzr/catkin_ws/devel/include;/usr/include/eigen3")
set(libpointmatcher_INCLUDE_DIR "/home/hzr/catkin_ws/devel/include;/usr/include/eigen3")

set(libpointmatcher_LIBRARIES "$<TARGET_FILE:pointmatcher>;libnabo::nabo;Threads::Threads;$<TARGET_FILE:yaml-cpp-pm>;Boost::thread;Boost::filesystem;Boost::system;Boost::program_options;Boost::date_time;Boost::chrono")
set(LIBPOINTMATCHER_LIBRARIES "$<TARGET_FILE:pointmatcher>;libnabo::nabo;Threads::Threads;$<TARGET_FILE:yaml-cpp-pm>;Boost::thread;Boost::filesystem;Boost::system;Boost::program_options;Boost::date_time;Boost::chrono")
set(pointmatcher_LIBRARIES "$<TARGET_FILE:pointmatcher>;libnabo::nabo;Threads::Threads;$<TARGET_FILE:yaml-cpp-pm>;Boost::thread;Boost::filesystem;Boost::system;Boost::program_options;Boost::date_time;Boost::chrono")
set(POINTMATCHER_LIBRARIES "$<TARGET_FILE:pointmatcher>;libnabo::nabo;Threads::Threads;$<TARGET_FILE:yaml-cpp-pm>;Boost::thread;Boost::filesystem;Boost::system;Boost::program_options;Boost::date_time;Boost::chrono")

# This causes catkin simple to link against these libraries
set(libpointmatcher_FOUND_CATKIN_PROJECT true)
set(LIBPOINTMATCHER_FOUND_CATKIN_PROJECT true)
set(pointmatcher_FOUND_CATKIN_PROJECT true)
set(POINTMATCHER_FOUND_CATKIN_PROJECT true)

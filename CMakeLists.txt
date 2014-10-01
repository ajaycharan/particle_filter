cmake_minimum_required(VERSION 2.8.3)
project(robot_localization_lab1)

find_package(OpenCV REQUIRED)



# Need both the following for debugging with ddd
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -gdwarf-2 -ggdb")
#set(CMAKE_CXX_FLAGS_DEBUG "-std=c++11 ${CMAKE_CXX_FLAGS_DEBUG} -gdwarf-2  -ggdb")
#set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 ${CMAKE_CXX_FLAGS_RELEASE} -gdwarf-2 -ggdb")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++11 ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 ${CMAKE_CXX_FLAGS_RELEASE}")
message(${CMAKE_CXX_FLAGS_RELEASE})

include_directories(
  include
)

link_directories(

)

add_library(utilities
    src/Utilities.cc
)
target_link_libraries(utilities
    ${OpenCV_LIBS}
)

add_library(particle_filter
    src/ParticleFilter.cc
)
target_link_libraries(particle_filter
    ${OpenCV_LIBS}
)

add_library(simulator
    src/RobotSimulator.cc
)
target_link_libraries(simulator
    utilities
    particle_filter
    ${OpenCV_LIBS}
)

add_executable(test_utilities
    src/TestUtilities.cc
)
target_link_libraries(test_utilities
    utilities
    ${OpenCV_LIBS}
)

add_executable(test_particle_filter
    src/TestParticleFilter.cc
)
target_link_libraries(test_particle_filter
    utilities
    particle_filter
    simulator
    ${OpenCV_LIBS}
)

#install(TARGETS vision_utils ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_bag_process
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)

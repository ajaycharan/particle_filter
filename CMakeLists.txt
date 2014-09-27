cmake_minimum_required(VERSION 2.8.3)
project(robot_localization_lab1)

find_package(OpenCV REQUIRED)



# Need both the following for debugging with ddd
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -gdwarf-2 -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++11 ${CMAKE_CXX_FLAGS_DEBUG} -gdwarf-2  -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 ${CMAKE_CXX_FLAGS_RELEASE} -gdwarf-2 -ggdb")

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

add_executable(test
  src/test.cc
)
target_link_libraries(test
  utilities
  particle_filter
  simulator
  ${OpenCV_LIBS}
)

#add_library(vision_utils
#  src/visionUtilities.cpp
#)
#target_link_libraries(vision_utils ${OpenCV_LIBS})

#add_library(${PROJECT_NAME}
#  src/visualOdometry.cpp
#  src/StereoVisualOdometry.cc
#)
#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBS}
#  vision_utils
#)

#add_executable(${PROJECT_NAME}_node
#  src/${PROJECT_NAME}.cc
#)

#set_target_properties(${PROJECT_NAME}_node
#  PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
#)

#target_link_libraries(${PROJECT_NAME}_node
#  ${catkin_LIBRARIES}
#  vision_utils
#  ${PROJECT_NAME}
#  ${OpenCV_LIBS}
#)

#add_executable(${PROJECT_NAME}_bag_process
#  src/${PROJECT_NAME}_bag_process.cc
#)
#target_link_libraries(${PROJECT_NAME}_bag_process
#  ${catkin_LIBRARIES}
#  vision_utils
#  ${PROJECT_NAME}
#  ${OpenCV_LIBS}
#)

#install(TARGETS vision_utils ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_bag_process
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)

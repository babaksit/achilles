cmake_minimum_required(VERSION 2.8.3)
project(achilles_qr_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  message_generation
  std_msgs
  sensor_msgs
  pluginlib
  cmake_modules
)

include_directories(include include/achilles_qr_detector)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Qt4 4.6 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})


add_message_files(
   FILES
   victimPos.msg
   qrPos.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   VictimImage.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs # Or other packages containing msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp  pluginlib hector_map_tools hector_nav_msgs pluginlib
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  
  )
# find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
add_definitions(${Eigen3_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(/usr/local/include)


add_library(QR_filtering src/QR_filtering.cpp)
add_library(QR_Detection src/QR_Detection.cpp)

add_library(geotiff_writer src/geotiff_writer.cpp)
target_link_libraries(geotiff_writer ${QT_LIBRARIES}  ${catkin_LIBRARIES} )
add_dependencies(geotiff_writer ${catkin_EXPORTED_TARGETS})

add_executable(QR_filtering_node src/QR_filtering_node.cpp)
target_link_libraries(QR_filtering_node QR_filtering  ${catkin_LIBRARIES})
add_dependencies(QR_filtering_node ${catkin_EXPORTED_TARGETS})

add_executable(QR_Detection_node src/QR_Detection_node.cpp)
target_link_libraries(QR_Detection_node QR_Detection -lzbar ${QT_LIBRARIES} ${OpenCV_LIBS}  ${catkin_LIBRARIES})
add_dependencies(QR_Detection_node ${catkin_EXPORTED_TARGETS})

add_executable(geotiff_saver src/geotiff_saver.cpp)
target_link_libraries(geotiff_saver geotiff_writer ${QT_LIBRARIES} ${GEOTIFF_LIBRARY}  ${catkin_LIBRARIES})

add_executable(geotiff_node src/geotiff_node.cpp)
target_link_libraries(geotiff_node geotiff_writer ${QT_LIBRARIES} ${GEOTIFF_LIBRARY}  ${catkin_LIBRARIES} )
add_dependencies(geotiff_node ${catkin_EXPORTED_TARGETS})

add_executable(scan_macher_node src/scan_macher_node.cpp )
target_link_libraries(scan_macher_node ${catkin_LIBRARIES} )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
#generate_messages()
## Generate added messages and services with any dependencies listed here


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)


 

install(TARGETS geotiff_writer geotiff_saver geotiff_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)



## Declare a cpp library
# add_library(achilles_qr_detector
#   src/${PROJECT_NAME}/achilles_qr_detector.cpp
# )

## Declare a cpp executable
# add_executable(achilles_qr_detector_node src/achilles_qr_detector_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(achilles_qr_detector_node achilles_qr_detector_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(achilles_qr_detector_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS achilles_qr_detector achilles_qr_detector_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_achilles_qr_detector.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

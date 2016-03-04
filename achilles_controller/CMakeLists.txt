cmake_minimum_required(VERSION 2.8.3)
project(achilles_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)


##QT

set(QT_SOURCE_FILES
        src/achilles_controller.cpp
)

set(QT_HEADER_FILES
    include/${PROJECT_NAME}/achilles_controller.h
)

find_package(Qt4 COMPONENTS QtCore QtNetwork)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})
include_directories(${Boost_INCLUDE_DIR})

include(${QT_USE_FILE})
qt4_automoc(${QT_SOURCE_FILES})
QT4_WRAP_CPP(qt_moc_srcs ${QT_HEADER_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  # LIBRARIES aras_visual_servo
  CATKIN_DEPENDS roscpp

#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}

)

#add_executable(achilles_controller_node src/achilles_controller_node.cpp )
#target_link_libraries(achilles_controller_node
#  ${catkin_LIBRARIES}
#)

add_executable(achilles_controller_node src/achilles_controller_node.cpp ${QT_SOURCE_FILES} ${qt_moc_srcs})
target_link_libraries(achilles_controller_node ${catkin_LIBRARIES} ${QT_LIBRARIES}  -lqextserialport -lpthread)

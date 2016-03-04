cmake_minimum_required(VERSION 2.8.3)
project(achilles_navigation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  costmap_2d
)


##QT

set(QT_SOURCE_FILES
        src/achilles_navigation.cpp
        src/left_hand.cpp
        src/grid_navigation.cpp
        src/frontier_search.cpp
)

set(QT_HEADER_FILES
        include/${PROJECT_NAME}/achilles_navigation.h
        include/${PROJECT_NAME}/left_hand.h
        include/${PROJECT_NAME}/grid_navigation.h
        include/${PROJECT_NAME}/frontier_search.h
)

find_package(Qt4 COMPONENTS QtCore QtNetwork PCL)



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
  CATKIN_DEPENDS roscpp pcl pcl_ros

#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

#add_executable(achilles_navigation_node src/achilles_navigation_node.cpp )
#target_link_libraries(achilles_navigation_node
#  ${catkin_LIBRARIES}
#)
add_definitions(${PCL_DEFINITIONS})

add_executable(achilles_navigation_node src/achilles_navigation_node.cpp
                        src/quadratic_calculator.cpp
                        src/grid_path.cpp
                        src/gradient_path.cpp
                        src/dijkstra.cpp
                        src/orientation_filter.cpp
                        ${QT_SOURCE_FILES} ${qt_moc_srcs})

target_link_libraries(achilles_navigation_node ${catkin_LIBRARIES} ${QT_LIBRARIES} ${PCL_LIBRARIES} -lpthread )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

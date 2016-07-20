cmake_minimum_required(VERSION 2.4.6)

project(ros_profiling)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
	cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

SET(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration")



find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs ros_profiling_msgs roscpp_serialization roscpp_tutorials geometry_msgs roscpp_serialization topic_tools)
find_package(Boost REQUIRED COMPONENTS thread)
catkin_package(CATKIN_DEPENDS roscpp ros_profiling_msgs roscpp_serialization roscpp_tutorials geometry_msgs)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rx           src/rx.cpp)
add_executable(tx           src/tx.cpp)
add_executable(controller   src/controller.cpp)
add_executable(checker      src/checker.cpp)


target_link_libraries(rx                yaml-cpp rt z ${Boost_LIBRARIES}  ${catkin_LIBRARIES})
target_link_libraries(tx                yaml-cpp rt z ${Boost_LIBRARIES}  ${catkin_LIBRARIES})
target_link_libraries(controller        yaml-cpp rt z boost_iostreams ${Boost_LIBRARIES}  ${catkin_LIBRARIES})
target_link_libraries(checker           yaml-cpp ncurses rt z ${Boost_LIBRARIES}  ${catkin_LIBRARIES})

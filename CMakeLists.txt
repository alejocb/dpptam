

cmake_minimum_required(VERSION 2.8.3)
project(dpptam)
add_definitions(-Wall -std=c++11 -Werror=return-type ${PCL_DEFINITIONS})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport cv_bridge tf pcl_ros)


set(CMAKE_BUILD_TYPE Debug)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge tf)

set(DENSE_SOURCE_FILES     
		src/SemiDenseMapping.cpp
                src/DenseMapping.cpp
                src/SemiDenseTracking.cpp
                src/vo_system.cpp
                src/superpixel.cpp
		src/main.cpp
)


set(DENSE_HEADER_FILES  
		include/dpptam/SemiDenseMapping.h
                include/dpptam/DenseMapping.h
                include/dpptam/SemiDenseTracking.h
                include/dpptam/vo_system.h
                include/dpptam/superpixel.h
)

find_package(OpenCV REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
#  LIBRARIES imu_3dm_gx3
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
find_package(OpenMP)
if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()



option(OPTIMIZATION "Enable optimization" TRUE)
if(OPTIMIZATION)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
message("OPTIMIZATION FLAG ACTIVATED")
else()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
message("NO OPTIMIZATION FLAGS")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra -pedantic  -Wcast-qual -Wpointer-arith -Winit-self -Wswitch-default -Wmissing-include-dirs -Wold-style-cast -Wnon-virtual-dtor -Wshadow -Wno-unknown-pragmas")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra -pedantic  -Wcast-qual -Wpointer-arith -Winit-self -Wswitch-default -Wmissing-include-dirs -Wold-style-cast -Wnon-virtual-dtor -Wshadow -Wno-unknown-pragmas")

link_directories(${PCL_LIBRARY_DIRS})
###########
## Build ##
###########
add_executable(main ${DENSE_HEADER_FILES} ${DENSE_SOURCE_FILES})
target_link_libraries(main ${OpenCV_LIBS} ${catkin_LIBRARIES})


 install(PROGRAMS
  scripts/run_bag
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

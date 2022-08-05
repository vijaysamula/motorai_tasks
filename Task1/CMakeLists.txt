cmake_minimum_required(VERSION 3.16.2)
project(sensor_data_fusion)

# set(CMAKE_CXX_STANDARD 17)
find_package(PCL REQUIRED)
find_package(OpenCV 4.0.0 REQUIRED)
find_package (yaml-cpp)
if (yaml-cpp_FOUND)
  message("YAML Libs: ${YAML_CPP_LIBRARIES}")
  message("YAML Headers: ${YAML_CPP_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "yaml-cpp NOT FOUND\n")
endif (yaml-cpp_FOUND)

include_directories(include
                ${PCL_INCLUDE_DIRS} 
                ${OpenCV_INCLUDE_DIRS}
                ${YAML_CPP_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(camera_lidar_fusion camera_lidar_fusion.cpp)
target_link_libraries(camera_lidar_fusion ${PCL_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})
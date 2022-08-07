cmake_minimum_required(VERSION 3.16.2)
project(sensor_data_fusion)

# set(CMAKE_CXX_STANDARD 17)
find_package(PCL REQUIRED)
find_package(OpenCV 4.0.0 REQUIRED)


include_directories(include
                ${PCL_INCLUDE_DIRS} 
                ${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(camera_lidar_fusion camera_lidar_fusion.cpp)
target_link_libraries(camera_lidar_fusion ${PCL_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OpenCV_LIBRARIES} )
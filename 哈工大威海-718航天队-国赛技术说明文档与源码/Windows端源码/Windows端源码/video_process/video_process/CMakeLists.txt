cmake_minimum_required(VERSION 2.8)
project(001)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_VERSION VERSION_LESS "2.8.11")
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

add_executable(1 1.cpp)
target_link_libraries(1 PRIVATE ${OpenCV_LIBS})

add_executable(2 2.cpp)
target_link_libraries(2 PRIVATE ${OpenCV_LIBS})

add_executable(3 3.cpp)
target_link_libraries(3 PRIVATE ${OpenCV_LIBS})

add_executable(4 4.cpp)
target_link_libraries(4 PRIVATE ${OpenCV_LIBS})

add_executable(5 5.cpp)
target_link_libraries(5 PRIVATE ${OpenCV_LIBS})




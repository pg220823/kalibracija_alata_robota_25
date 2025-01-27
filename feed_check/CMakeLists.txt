cmake_minimum_required(VERSION 2.8.12)

project(feed_check LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Qt5Core)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

add_executable(feed_check "main.cpp")
add_executable(snimanje_slika "main2.cpp")
add_executable(kalibracija_intrinzicna "kal_int.cpp")
add_executable(segmentacija "slike_seg.cpp")

target_link_libraries(feed_check Qt5::Core ${OpenCV_LIBS})
target_link_libraries(snimanje_slika Qt5::Core ${OpenCV_LIBS})
target_link_libraries(kalibracija_intrinzicna Qt5::Core ${OpenCV_LIBS})
target_link_libraries(segmentacija PRIVATE Qt5::Core ${OpenCV_LIBS} Threads::Threads Eigen3::Eigen)

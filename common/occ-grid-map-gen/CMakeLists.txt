cmake_minimum_required(VERSION 2.8)
project( MapGen )

find_package( OpenCV REQUIRED )

add_executable( main main.cc )
target_link_libraries( main ${OpenCV_LIBS} )
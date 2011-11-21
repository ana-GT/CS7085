 cmake_minimum_required(VERSION 2.8)


project( Homework1 )
include_directories( . )

set ( SRC_FILES Homework1.cpp )
add_executable( Homework1 ${SRC_FILES} )
add_executable( Homework1c Homework1c.cpp )

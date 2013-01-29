cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PROJ_NAME		"BachelorProject")

project(${PROJ_NAME})
set( VERSION_MAJOR "1" )
set( VERSION_MINOR "0" )
set( VERSION_PATCH "0" )

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-Wno-unknown-pragmas -Wno-deprecated -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT ) #-H )
#add_definitions(-Wno-unknown-pragmas -Wno-deprecated -DEIGEN_DONT_ALIGN )
#add_definitions(-Wno-unknown-pragmas -Wno-deprecated )

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_STATIC OFF) 
find_package(PCL 1.7 REQUIRED)
find_package(Qt4 REQUIRED)
find_package(VTK REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${PROJ_INCLUDES})
link_directories(${PCL_LIBRARY_DIRS}  ${PROJ_LIBRARIES}) 
add_definitions(${PCL_DEFINITIONS})

# Default values
SET(NESTK_USE_FREENECT ON)
SET(NESTK_USE_OPENNI ON)
SET(NESTK_USE_PCL ON)
SET(NESTK_USE_EMBEDDED_EIGEN OFF)
SET(NESTK_USE_EMBEDDED_FREENECT ON)
SET(NESTK_USE_EMBEDDED_GLEW ON)
SET(NESTK_USE_EMBEDDED_OPENNI OFF)

ADD_SUBDIRECTORY(rgbdemo/nestk)
## nestk/UseNestk.cmake.in defines variable to use nestk.
## It itself includes nestk/deps/cmake/UseEmbeddedNestkDeps.cmake that
## contains most of the cmake find packages commands.
INCLUDE("${nestk_BINARY_DIR}/UseEmbeddedNestk.cmake")
#INCLUDE("${NESTK_EMBEDDED_USE_FILE}")

##########################################################

add_executable (pairwise_registration pairwise_registration.cpp)
target_link_libraries (pairwise_registration ${PCL_LIBRARIES})


ADD_SUBDIRECTORY(dynmap)
ADD_SUBDIRECTORY(rgbdemo/viewer)
ADD_SUBDIRECTORY(rgbdemo/calibration)
ADD_SUBDIRECTORY(rgbdemo/reconstructor)


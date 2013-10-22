cmake_minimum_required (VERSION 2.6)
project (ATLANTE_FRAMEWORK)

# The version number.
# set (IMG_ACQ_2CAMSTH_VERSION_MAJOR 1)
# set (IMG_ACQ_2CAMSTH_VERSION_MINOR 0)

# Create Eclipse project file
# message ("CMAKE_GENERATOR: " ${CMAKE_GENERATOR})
# set (CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT TRUE)
# set (CMAKE_GENERATOR "Eclipse CDT4 - Unix Makefiles")
# message ("CMAKE_GENERATOR: " ${CMAKE_GENERATOR})
# message ("CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT: " ${CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT} )

# message ("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR} )
# set (PROJECT_SOURCE_DIR "${PROJECT_SOURCE_DIR}/sources" )
set (PROJECT_BINARY_DIR "${PROJECT_SOURCE_DIR}/bin" )
# message ("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR} )

# include directories
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}")

# poshlib
set (POSHLIB_PATH "${PROJECT_SOURCE_DIR}/3rdparty/poshlib" )
include_directories("${POSHLIB_PATH}") 

# set(CMAKE_SOURCE_DIR  "${SOURCE_DIR}/src/" CACHE)
set ( ATLANTE_FRAMEWORK_SRCs 
	sources/cvgException.cpp
	sources/cvgMutex.cpp
	sources/cvgString.cpp
	sources/cvgThread.cpp
	sources/cvgTimer.cpp
	sources/HTMatrix4.cpp
	sources/Matrix3.cpp
	sources/Matrix4.cpp
	sources/RotMatrix3.cpp
	sources/Vector3.cpp
	sources/Vector4.cpp 
	${POSHLIB_PATH}/posh.c )
add_library(atlante ${ATLANTE_FRAMEWORK_SRCs} )
# target_link_libraries (imacq_2camSTH atlante)

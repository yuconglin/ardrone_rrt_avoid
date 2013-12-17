cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
IF(CMAKE_COMPILER_IS_GNUCXX)
   SET(CMAKE_CXX_FLAGS "-g3 -ggdb -DTIXML_USE_TICPP") 
#SET(CMAKE_BUILD_TYPE DEBUG)
ENDIF()
find_package(OpenCV REQUIRED)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
if(EXISTS /usr/lib64/libarmadillo.so)
    set( ARMADILLO_LIBRARIES /usr/lib64/libarmadillo.so )
else()
    find_package(Armadillo REQUIRED)
endif()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
include_directories(src/Dubins2D)
include_directories(src/VirtualQuad)
rosbuild_add_library(dubin src/Dubins2D/dubins.cpp)

rosbuild_add_boost_directories()
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
set(TINYCPP_DIR src/tinycpp/)
include_directories( ${TINYCPP_DIR} )
add_library(lib_ticpp
            ${TINYCPP_DIR}/ticpp.cpp  
	    ${TINYCPP_DIR}/tinyxml.cpp
	    ${TINYCPP_DIR}/tinyxmlerror.cpp
	    ${TINYCPP_DIR}/tinyxmlparser.cpp
	    ${TINYCPP_DIR}/tinystr.cpp
	   )

	 #rosbuild_add_library(QuadConfig src/QuadConfig/QuadConfig.cpp)
	 #include_directories(src/QuadConfig)

# BEGIN: addon's for multirotor speed controller

# Adding atlante
# Option 1: include atlante source code in lib on eclipse workspace (important thing is that it is outside the projects source tree)
include_directories("src/jesus_codes/atlante") # 3rdparty/atlante/atlante")
#add_subdirectory("/home/yucong/ros_workspace/lib/atlante/atlante" "${PROJECT_SOURCE_DIR}/../bin/3rdparty/atlante-bin" )

# Adding Jesus's code
include_directories("src/jesus_codes/include")
add_library(jesus_codes                       
	  src/jesus_codes/sources/Controller_MidLevelCnt.cpp
	  src/jesus_codes/sources/Controller_MidLevel_SpeedLoop.cpp
	  src/jesus_codes/sources/datalogger.cpp
	  src/jesus_codes/sources/FilteredDerivative.cpp
	  src/jesus_codes/sources/jesus_library.cpp
	  src/jesus_codes/sources/LowPassFilter.cpp
	  src/jesus_codes/sources/matrixLib.cpp
	  src/jesus_codes/sources/MidLevelControlledsimMultirotor.cpp
	  src/jesus_codes/sources/PID.cpp
	  src/jesus_codes/sources/stringdatalogger.cpp
	  )

# END::: addon's for multirotor speed controller

# all the utils functions
include_directories("src/user_types")
include_directories("src/utils")
add_library(lib_utils          
src/utils/CircleVelocity.cpp
src/utils/DubinsSubCheck.cpp
src/utils/DubinsTotalCheck.cpp
src/utils/GetRho.cpp
src/utils/LineVelocity.cpp
src/utils/NotInRadius.cpp
src/utils/PointInPoly.cpp
src/utils/CollectCheck.cpp
src/utils/systemtime.cpp
src/utils/QuatRPY.cpp
           )

#the main part
#****************the others monitor************************#
include_directories("src/OthersMonitor")
rosbuild_add_library(other_monitor src/OthersMonitor/OthersMonitor.cpp)

#****************the execution node************************#
#execute of line,circle and a dubin's curve
rosbuild_add_library(parrot_exe src/exe_node/ParrotExe.cpp)
#the dubin test
rosbuild_add_executable(exe_dubin src/exe_node/main_dubin.cpp)

target_link_libraries(exe_dubin parrot_exe jesus_codes dubin lib_ticpp lib_utils ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES}) 
#command reaction test
rosbuild_add_executable(exe_test src/exe_node/main_test.cpp)
target_link_libraries(exe_test parrot_exe jesus_codes dubin lib_ticpp lib_utils ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES}) 

target_link_libraries(exe_dubin parrot_exe jesus_codes dubin lib_ticpp lib_utils ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES})
#the command test
rosbuild_add_executable(exe_test src/exe_node/main_test.cpp)
target_link_libraries(exe_test parrot_exe jesus_codes dubin lib_ticpp lib_utils ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES})

#fly
rosbuild_add_executable(exe_fly src/exe_node/main_fly.cpp)
#record
rosbuild_add_executable(exe_record src/exe_node/main_record.cpp)
target_link_libraries(exe_record parrot_exe jesus_codes dubin lib_ticpp lib_utils ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES})
#path following
rosbuild_add_executable(exe_path src/exe_node/main_path.cpp)
target_link_libraries(exe_path parrot_exe jesus_codes dubin lib_ticpp lib_utils ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES})
#path simulation
rosbuild_add_executable(sim_path src/exe_node/main_sim_path.cpp)
target_link_libraries(sim_path parrot_exe jesus_codes dubin lib_ticpp lib_utils ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES})
#host
rosbuild_add_executable(exe_host src/exe_node/main_host.cpp)
target_link_libraries(exe_host parrot_exe other_monitor jesus_codes dubin lib_ticpp lib_utils ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES})
#intruder
rosbuild_add_executable(exe_intruder src/exe_node/main_intruder.cpp)
target_link_libraries(exe_intruder parrot_exe other_monitor jesus_codes dubin lib_ticpp lib_utils ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES})

#the planner
include_directories("src/tree")

#uav state update test
add_library(lib_state
            src/user_types/UavState/ArdroneState.cpp
           )
rosbuild_add_executable(test_update src/user_types/UavState/test_update.cpp)
target_link_libraries(test_update lib_state ${ARMADILLO_LIBRARIES} )
  
#user_types
add_library(lib_user_types
            src/user_types/Sampler3D/Sampler3D.cpp
            src/user_types/UavBehavior/ArdroneBehavior.cpp
            src/user_types/UavConfig/ArdroneConfig.cpp
            src/user_types/UavState/ArdroneState.cpp
           )

#rrt planner
include_directories("src")
add_library(lib_clrrt
     src/YlClRRT/YlClRRT.cpp
  )

rosbuild_add_executable(test_sample src/YlClRRT/main_test.cpp)
target_link_libraries(test_sample lib_clrrt lib_user_types lib_utils lib_ticpp dubin ${ARMADILLO_LIBRARIES} )
rosbuild_add_executable(test_tree src/YlClRRT/main_tree.cpp)
target_link_libraries(test_tree lib_clrrt lib_user_types lib_utils lib_ticpp dubin ${ARMADILLO_LIBRARIES} )

#planner node
add_library(plan_node src/plan_node/ParrotPlan.cpp)
rosbuild_add_executable(plan_path src/plan_node/main_plan.cpp)
target_link_libraries(plan_path plan_node lib_clrrt lib_user_types lib_utils lib_ticpp dubin ${ARMADILLO_LIBRARIES})

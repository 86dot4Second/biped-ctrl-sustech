cmake_minimum_required(VERSION 3.12)

project(biped_new)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
        find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})


add_executable(biped_new src/robot.cpp src/robot.h src/kinematics.cpp src/kinematics.h src/plan.h src/plan.cpp src/main.cpp)#src/ "src/pland.h"main.cpp src/plan.cpp src/plan.h

if(UNIX)
    target_link_libraries(biped_new ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(biped_new ${aris_LIBRARIES})
endif(WIN32)


cmake_minimum_required(VERSION 2.8)

project(HW1)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-O3" )

#sophus
find_package( Sophus REQUIRED)

include_directories(
${EIGEN3_INCLUDE_DIR}    #Eigen
${Sophus_INCLUDE_DIRS}   #Sophus
)



add_executable(${PROJECT_NAME} "main.cpp")
target_link_libraries(${PROJECT_NAME}
${EIGEN3_LIBS}
${Sophus_LIBRARIES}
)

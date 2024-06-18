cmake_minimum_required(VERSION 3.0)

set( CMAKE_CXX_FLAGS "-std=c++17 " )

set( CMAKE_BUILD_TYPE "Debug")

# set( CMAKE_CXX_FLAGS "-ftemplate-depth=15000")

# cmake_minimum_required(VERSION 2.8)

# project(gaussNewton)

project(ceres_hello)

# set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")

# list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)

include_directories(/usr/include/eigen3)

# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIR})

find_package(Ceres REQUIRED)
# include_directories(${CERES_INCLUDE_DIRS})
# link_directories(${CERES_LIBRARY_DIRS})
# include_directories(/usr/local/include/ceres)

# find_package(G2O REQUIRED)
# include_directories(${G2O_INCLUDE_DIRS} "usr/local/include/g2o" "usr/local/bin/g2o")

# find_package(glog 0.5.0 REQUIRED)
# find_package(Gflags REQUIRED)


# add_executable(gaussNewton gaussNewton.cpp)
# target_link_libraries(gaussNewton ${OpenCV_LIBS})

# add_executable(ceresCurveFitting ceresCurveFitting.cpp)
# target_link_libraries(ceresCurveFitting ${OpenCV_LIBS} Ceres::ceres)

# add_executable(helloCeres helloCeres.cpp)
# target_link_libraries(helloCeres Ceres::ceres)

# add_executable(g2oCurveFitting g2oCurveFitting.cpp)
# target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} g2o_core g2o_stuff glog::glog)


# add_executable(hello_ceres hello_ceres.cpp)
# target_link_libraries(hello_ceres Ceres::ceres)

# install(TARGETS ceres_hello_202404 RUNTIME DESTINATION bin)

add_executable(configration configration.cpp)
target_link_libraries(configration )

add_executable(dataPrepare dataPrepare.cpp configration.cpp)
target_link_libraries(dataPrepare )

add_executable(shapeFit shapeFit.cpp dataPrepare.cpp)
target_link_libraries(shapeFit )

add_executable(sphereFit sphereFit.cpp configration.cpp dataPrepare.cpp shapeFit.cpp  )
target_link_libraries(sphereFit Ceres::ceres)

add_executable(allSphereFit allSphereFit.cpp configration.cpp dataPrepare.cpp shapeFit.cpp)
target_link_libraries(allSphereFit Ceres::ceres)

add_executable(lineFit lineFit.cpp configration.cpp dataPrepare.cpp shapeFit.cpp)
target_link_libraries(lineFit Ceres::ceres)

add_executable(eccentricityAndInclination eccentricityAndInclination.cpp configration.cpp dataPrepare.cpp shapeFit.cpp)
target_link_libraries(eccentricityAndInclination Ceres::ceres)

add_executable(oneTargetAllVerticalFit oneTargetAllVerticalFit.cpp configration.cpp dataPrepare.cpp shapeFit.cpp)
target_link_libraries(oneTargetAllVerticalFit Ceres::ceres)

add_executable(allTargetAllVerticalFit allTargetAllVerticalFit.cpp configration.cpp dataPrepare.cpp shapeFit.cpp shapeFit.cpp)
target_link_libraries(allTargetAllVerticalFit Ceres::ceres)


add_executable(circleFit circleFit.cpp dataPrepare.cpp configration.cpp)
target_link_libraries(circleFit Ceres::ceres)







add_executable(allCircleFit allCircleFit.cpp dataPrepare.cpp circleFit.cpp sphereFit.cpp configration.cpp)
target_link_libraries(allCircleFit Ceres::ceres)






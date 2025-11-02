find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# OMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

# for ubuntu 18.04, please update gcc/g++ to 9, and then unzip oneapi-tbb-2022.2.0-lin in thirdparty folder,
# then check TBB2018_INCLUDE_DIR is correct and set UBUNTU1804_TBB "ON"
option(UBUNTU1804_TBB "use custom tbb package in ubuntu18.04" ON)
if(UBUNTU1804_TBB)
    set(CUSTOM_TBB_DIR "${PROJECT_SOURCE_DIR}/thirdparty/oneapi-tbb-2022.2.0-lin/oneapi-tbb-2022.2.0")
    set(TBB2022_INCLUDE_DIR "${CUSTOM_TBB_DIR}/include")
    set(TBB2022_LIBRARY_DIR "${CUSTOM_TBB_DIR}/lib/intel64/gcc4.8")
    include_directories(${TBB2022_INCLUDE_DIR})
    link_directories(${TBB2022_LIBRARY_DIR})
else()
    find_package(TBB REQUIRED)
    include_directories(${TBB_INCLUDE_DIRS})
    link_directories(${TBB_LIBRARY_DIRS})
endif ()

if (BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
else ()
    add_definitions(-std=c++17 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif ()

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
        ${tf2_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
)

include_directories(
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/thirdparty
)


set(third_party_libs
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        glog gflags
        yaml-cpp
        tbb
)

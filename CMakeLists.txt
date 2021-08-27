CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(Smart_Markers)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

# Add *.cmake files in cmake_modules dir
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(ZED 3 REQUIRED)
message(STATUS "ZED Found: " ${ZED_FOUND})
find_package(OpenCV 3.4.1 REQUIRED)
message(STATUS "OpenCV Found: " ${OpenCV_FOUND})
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(aruco REQUIRED)
message(STATUS "aruco Found: " ${aruco_FOUND})
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 Found: " ${EIGEN3_FOUND})
find_package(G2O REQUIRED)
message(STATUS "G2O Found: " ${G2O_FOUND})
find_package(Cholmod REQUIRED)
message(STATUS "Cholmod Found: " ${CHOLMOD_FOUND})
find_package(CSparse REQUIRED)
message(STATUS "CSParse Found: " ${CSparse_FOUND})

set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)
message(STATUS "Threads Found: " ${Threads_FOUND})

IF(NOT WIN32)
    add_definitions(-Wno-format-extra-args)
    SET(SPECIAL_OS_LIBS "pthread" "X11")
ENDIF()

set (MM_SRC_DIR /usr/local/include/marker_mapper)
set (LIBSERIAL_SRC_DIR /usr/local/include/libserial)

include_directories(
    ${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${G2O_INCLUDE_DIR}
    ${CHOLMOD_INCLUDE_DIR}
    ${CSPARSE_INCLUDE_DIR}
    ${MM_SRC_DIR}
    ${LIBSERIAL_SRC_DIR}
)

list(APPEND G2O_LIBS
    ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_CLI_LIBRARY} ${G2O_SOLVER_CHOLMOD}
    ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_SOLVER_STRUCTURE_ONLY}
    ${G2O_SOLVER_EIGEN} ${G2O_TYPES_DATA} ${G2O_TYPES_ICP} ${G2O_TYPES_SBA}
    ${G2O_TYPES_SCLAM2D} ${G2O_TYPES_SIM3} ${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM3D}
    ${CHOLMOD_LIBRARIES} ${CSPARSE_LIBRARY}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories("/usr/local/lib")

FILE(GLOB_RECURSE SENSORS_SRC_DIR src/sensors/*.cpp)
add_library(sensors SHARED ${SENSORS_SRC_DIR})
target_link_libraries(sensors ${ZED_LIBS})

FILE(GLOB_RECURSE UTILS_SRC_DIR src/utils/*.cpp)
add_library(utils SHARED ${UTILS_SRC_DIR})
target_link_libraries(utils ${ZED_LIBS} aruco)

FILE(GLOB_RECURSE OPT_SRC_DIR src/optimization/*.cpp)
add_library(optimization SHARED ${OPT_SRC_DIR})
target_link_libraries(optimization ${G2O_LIBS} ${OpenCV_LIBRARIES} cholmod aruco utils libmarker_mapper.so)

ADD_EXECUTABLE(save_data_cloud_correction src/app/save_data_cloud_correction.cpp)
ADD_EXECUTABLE(test_zed src/app/test_zed.cpp)
ADD_EXECUTABLE(test_marker_mapper src/app/test_marker_mapper.cpp)
ADD_EXECUTABLE(test_sm_mapping src/app/test_sm_mapping.cpp)
ADD_EXECUTABLE(test_cam_localization src/app/test_cam_localization.cpp)

ADD_EXECUTABLE(pms_orientation_viewer src/app/pms_orientation_viewer.cpp)
ADD_EXECUTABLE(pms_infomatrix_viewer src/app/pms_infomatrix_viewer.cpp)

add_definitions(-std=c++14 -O3)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

TARGET_LINK_LIBRARIES(save_data_cloud_correction ${ZED_LIBS} ${OpenCV_LIBRARIES} sensors aruco utils optimization)
TARGET_LINK_LIBRARIES(test_zed ${ZED_LIBS} ${OpenCV_LIBS} sensors utils)
TARGET_LINK_LIBRARIES(test_marker_mapper ${ZED_LIBS} ${OpenCV_LIBS} sensors utils aruco libmarker_mapper.so)
TARGET_LINK_LIBRARIES(test_sm_mapping ${ZED_LIBS} ${OpenCV_LIBS} ${G2O_LIBS} optimization sensors utils aruco)
TARGET_LINK_LIBRARIES(test_cam_localization ${ZED_LIBS} ${OpenCV_LIBS} ${G2O_LIBS} optimization sensors utils aruco)

TARGET_LINK_LIBRARIES(pms_orientation_viewer ${OpenCV_LIBS} utils libserial.so ${CMAKE_THREAD_LIBS_INIT})
TARGET_LINK_LIBRARIES(pms_infomatrix_viewer ${OpenCV_LIBS} utils libserial.so ${CMAKE_THREAD_LIBS_INIT})
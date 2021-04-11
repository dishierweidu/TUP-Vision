#----------------------------------------------------------#
#                                                          #
#                      CMakeLists.txt                      #
#                                                          #
#----------------------------------------------------------#
# 2020.9.18 by UP

cmake_minimum_required(VERSION 2.8)
SET(CMAKE_BUILD_TYPE "Release") # "Debug" "Release"
PROJECT(SAUVisionBuff)
find_package(OpenCV 3.4.5)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")
include_directories(${OpenCV_INCLUDE_DIRS})
ADD_EXECUTABLE(SAUVisionBuff ./main.cpp
					# Debug文件
					./Debug.h
					# 能量机关文件
					./Energy/Energy.cpp
					./Energy/Energy.h
					./Energy/Params.cpp
					./Energy/Params.h
					#相机驱动文件
					./Camera/GxIAPI.h
					./Camera/DxImageProc.h
					./Camera/DaHengCamera.h
					./Camera/DaHengCamera.cpp
					# fcc
					./ImageProcess/ImageProcess.h
					./ImageProcess/ImageProcess.cpp
					#装甲板识别文件
					./Armor/ArmorDetector.cpp
					./Armor/ArmorDetector.hpp
					./Armor/Settings.hpp
					#角度解算
					./AngelSolver/AngleSolver.cpp
					./AngelSolver/AngleSolver.hpp
					#串口通信
					./Serial/CRC_Check.cpp
					./Serial/CRC_Check.h
					./Serial/serialport.cpp
					./Serial/serialport.h

)

target_link_libraries(SAUVisionBuff ${OpenCV_LIBS})
find_package(Threads)
target_link_libraries(SAUVisionBuff ${CMAKE_THREAD_LIBS_INIT})
find_package(X11 REQUIRED)
include_directories(${X11_INLUCDE_DIR})
target_link_libraries(SAUVisionBuff ${X11_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} gxiapi)  # 包含大恒动态链接库

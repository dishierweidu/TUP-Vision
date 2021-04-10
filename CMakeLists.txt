#----------------------------------------------------------#
#                                                          #
#                      CMakeLists.txt                      #
#                                                          #
#----------------------------------------------------------#
# 2020.9.18 by UP

cmake_minimum_required(VERSION 2.8)
SET(CMAKE_BUILD_TYPE "Release") # "Debug"
PROJECT(SAUVisionBuff)
find_package(OpenCV 3.4.5)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")
include_directories(${OpenCV_INCLUDE_DIRS})
ADD_EXECUTABLE(SAUVisionBuff ~/Desktop/Test_Files/SAUVisionBuff/main.cpp
					# Debug文件
					~/Desktop/Test_Files/SAUVisionBuff/Debug.h
					# 能量机关文件
					~/Desktop/Test_Files/SAUVisionBuff/Energy/Energy.cpp
					~/Desktop/Test_Files/SAUVisionBuff/Energy/Energy.h
					~/Desktop/Test_Files/SAUVisionBuff/Energy/Params.cpp
					~/Desktop/Test_Files/SAUVisionBuff/Energy/Params.h
					#相机驱动文件
					~/Desktop/Test_Files/SAUVisionBuff/Camera/GxIAPI.h
					~/Desktop/Test_Files/SAUVisionBuff/Camera/DxImageProc.h
					~/Desktop/Test_Files/SAUVisionBuff/Camera/DaHengCamera.h
					~/Desktop/Test_Files/SAUVisionBuff/Camera/DaHengCamera.cpp
					# fcc
					~/Desktop/Test_Files/SAUVisionBuff/ImageProcess/ImageProcess.h
					~/Desktop/Test_Files/SAUVisionBuff/ImageProcess/ImageProcess.cpp
					#装甲板识别文件
					~/Desktop/Test_Files/SAUVisionBuff/Armor/ArmorDetector.cpp
					~/Desktop/Test_Files/SAUVisionBuff/Armor/ArmorDetector.hpp
					~/Desktop/Test_Files/SAUVisionBuff/Armor/Settings.hpp
					#角度解算
					~/Desktop/Test_Files/SAUVisionBuff/AngelSolver/AngleSolver.cpp
					~/Desktop/Test_Files/SAUVisionBuff/AngelSolver/AngleSolver.hpp
					#串口通信
					~/Desktop/Test_Files/SAUVisionBuff/Serial/CRC_Check.cpp
					~/Desktop/Test_Files/SAUVisionBuff/Serial/CRC_Check.h
					~/Desktop/Test_Files/SAUVisionBuff/Serial/serialport.cpp
					~/Desktop/Test_Files/SAUVisionBuff/Serial/serialport.h

)
target_link_libraries(SAUVisionBuff ${OpenCV_LIBS})
find_package(Threads)
target_link_libraries(SAUVisionBuff ${CMAKE_THREAD_LIBS_INIT})
find_package(X11 REQUIRED)
include_directories(${X11_INLUCDE_DIR})
target_link_libraries(SAUVisionBuff ${X11_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} gxiapi)  # 包含大恒动态链接库

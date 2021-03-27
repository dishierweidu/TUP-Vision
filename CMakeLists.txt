#----------------------------------------------------------#
#                                                          #
#                      CMakeLists.txt                      #
#                                                          #
#----------------------------------------------------------#
# 2020.9.18 by UP

cmake_minimum_required(VERSION 2.8)
PROJECT(SAUVisionBuff)
find_package(OpenCV 3.4.5)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")
include_directories(${OpenCV_INCLUDE_DIRS})
ADD_EXECUTABLE(SAUVisionBuff /home/rangeronmars/Desktop/test_file/SAUVisionBuff/main.cpp
					# Debug文件
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Debug.h
					# 能量机关文件
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Energy/Energy.cpp
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Energy/Energy.h
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Energy/Params.cpp
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Energy/Params.h
					#相机驱动文件
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Camera/GxIAPI.h
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Camera/DxImageProc.h
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Camera/DaHengCamera.h
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Camera/DaHengCamera.cpp
					# fcc
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/ImageProcess/ImageProcess.h
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/ImageProcess/ImageProcess.cpp
					#装甲板识别文件
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Armor/ArmorDetector.cpp
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Armor/ArmorDetector.hpp
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Armor/Settings.hpp
					#角度解算
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/AngelSolver/AngleSolver.cpp
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/AngelSolver/AngleSolver.hpp
					#串口通信
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Serial/CRC_Check.cpp
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Serial/CRC_Check.h
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Serial/serialport.cpp
					/home/rangeronmars/Desktop/test_file/SAUVisionBuff/Serial/serialport.h

)
target_link_libraries(SAUVisionBuff ${OpenCV_LIBS})
find_package(Threads)
target_link_libraries(SAUVisionBuff ${CMAKE_THREAD_LIBS_INIT})
find_package(X11 REQUIRED)
include_directories(${X11_INLUCDE_DIR})
target_link_libraries(SAUVisionBuff ${X11_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} gxiapi)  # 包含大恒动态链接库

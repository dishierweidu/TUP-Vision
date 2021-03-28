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
ADD_EXECUTABLE(SAUVisionBuff /home/tup/Desktop/Test_Files/SAUVisionBuff/main.cpp
					# Debug文件
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Debug.h
					# 能量机关文件
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Energy/Energy.cpp
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Energy/Energy.h
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Energy/Params.cpp
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Energy/Params.h
					#相机驱动文件
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Camera/GxIAPI.h
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Camera/DxImageProc.h
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Camera/DaHengCamera.h
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Camera/DaHengCamera.cpp
					# fcc
					/home/tup/Desktop/Test_Files/SAUVisionBuff/ImageProcess/ImageProcess.h
					/home/tup/Desktop/Test_Files/SAUVisionBuff/ImageProcess/ImageProcess.cpp
					#装甲板识别文件
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Armor/ArmorDetector.cpp
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Armor/ArmorDetector.hpp
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Armor/Settings.hpp
					#角度解算
					/home/tup/Desktop/Test_Files/SAUVisionBuff/AngelSolver/AngleSolver.cpp
					/home/tup/Desktop/Test_Files/SAUVisionBuff/AngelSolver/AngleSolver.hpp
					#串口通信
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Serial/CRC_Check.cpp
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Serial/CRC_Check.h
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Serial/serialport.cpp
					/home/tup/Desktop/Test_Files/SAUVisionBuff/Serial/serialport.h

)
target_link_libraries(SAUVisionBuff ${OpenCV_LIBS})
find_package(Threads)
target_link_libraries(SAUVisionBuff ${CMAKE_THREAD_LIBS_INIT})
find_package(X11 REQUIRED)
include_directories(${X11_INLUCDE_DIR})
target_link_libraries(SAUVisionBuff ${X11_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} gxiapi)  # 包含大恒动态链接库

cmake_minimum_required(VERSION 2.8)
project(demo1)

# 设定C++标准
set(CMAKE_CXX_STANDARD 11)

# 添加G2O的库
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
# 设定G2O的根目录
set(G2O_ROOT /urs/local/include/g2o)

# 找包
find_package(G2O REQUIRED)
find_package(OpenCV 3.2 REQUIRED)
find_package(CSparse REQUIRED)
find_package(Cholmod REQUIRED)

# 包含头文件夹
include_directories("/urs/include/eigen3")
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${G2O_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${CHOLMOD_INCLUDE_DIR})


# 包含库
set(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )
set(LIBS ${G2O_LIBS} ${OpenCV_LIBS} ${CHOLMOD_LIBRARIES})

# 生成exe文件
add_executable(demo1 main.cpp)
target_link_libraries(demo1 ${LIBS})
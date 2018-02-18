cmake_minimum_required(VERSION 3.9)
project(demo3)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_STANDARD 11)

# 设置库的输出位置
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# 添加头文件
include_directories(${PROJECT_SOURCE_DIR}/include)

# module path
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(G2O_ROOT /usr/local/inlcude/g2o)

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})

# CSpares
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# Cholmod
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})

# Eigen
include_directories("/usr/include/eigen3")

# G2O相关lib
set(G2O_LIBS ${CSPARSE_LIBRARY} ${CHOLMOD_LIBRARY}
        g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse)

# 总体的lib
set(LIBS ${G2O_LIBS})

#------------------------------------------
# 设置生成lib的文件
set(LIBS_SRC
        include/se2.h
        include/rand.h
        include/g2o_tutorial_slam2d_api.h
        src/vertex_se2.cpp
        src/vertex_point_xy.cpp
        src/edge_se2.cpp
        src/edge_se2_point.cpp
        src/parameter_se2_offset.cpp
        src/simulator.cpp
        src/types_tutorial_slam2d.cpp)

## 生成库文件
#add_library(demo3_lib SHARED include/se2.h
#        include/rand.h
#        include/g2o_tutorial_slam2d_api.h
#        src/vertex_se2.cpp
#        src/vertex_point_xy.cpp
#        src/edge_se2.cpp
#        src/edge_se2_point.cpp
#        src/parameter_se2_offset.cpp
#        src/simulator.cpp
#        src/types_tutorial_slam2d.cpp)
#target_link_libraries(demo3_lib ${LIBS})

add_executable(demo3 main.cpp ${LIBS_SRC})
target_link_libraries(demo3 ${LIBS})
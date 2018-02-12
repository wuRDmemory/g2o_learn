cmake_minimum_required(VERSION 3.9)
project(demo2)

set(CMAKE_CXX_STANDARD 11)

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

add_executable(demo2 main.cpp)
target_link_libraries(demo2 ${LIBS})
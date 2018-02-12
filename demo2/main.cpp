#include <Eigen/StdVector>

#include <unordered_set>

#include <iostream>
#include <stdint.h>

#include "g2o/config.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/icp/types_icp.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#if defined G2O_HAVE_CHOLMOD
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#elif defined G2O_HAVE_CSPARSE
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#endif

using namespace Eigen;
using namespace std;

class Sample
{
public:
    static int uniform(int from, int to);
    static double uniform();
    static double gaussian(double sigma);
};

static double uniform_rand(double lowerBndr, double upperBndr)
{
    return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}

static double gauss_rand(double mean, double sigma)
{
    double x, y, r2;
    do {
        x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
        y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
        r2 = x * x + y * y;
    } while (r2 > 1.0 || r2 == 0.0);
    return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

int Sample::uniform(int from, int to)
{
    return static_cast<int>(uniform_rand(from, to));
}

double Sample::uniform()
{
    return uniform_rand(0., 1.);
}

double Sample::gaussian(double sigma)
{
    return gauss_rand(0., sigma);
}

// 主函数
int main() {
    /*  构建G2O  */
    // 优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    // 线性求解器
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear;
    linear = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> >();
    // 算法
    g2o::OptimizationAlgorithmLevenberg* solver
            = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear)));
    // 优化器设置算法
    optimizer.setAlgorithm(solver);

    // set up 500 points
    vector<Vector3d> true_points;
    for (size_t i=0;i<500; ++i) {
        true_points.push_back(Vector3d((Sample::uniform()-0.5)*3,
                                       Sample::uniform()-0.5,
                                       Sample::uniform()+10));
    }

    Eigen::Vector2d focal_length(500,500);    // pixels
    Eigen::Vector2d principal_point(320,240); // 640x480 image
    double baseline = 0.075;                  // 7.5 cm baseline


    // 真实的位姿
    // Isometry3d表示变换矩阵4x4
    vector<Eigen::Isometry3d, aligned_allocator<Eigen::Isometry3d> > true_poses;

    //
    // set up camera params
    // 这个函数是一个静态函数，会修改整个类的静态成员
    g2o::VertexSCam::setKcam(focal_length[0],focal_length[1],
                             principal_point[0],principal_point[1],
                             baseline);

    // 设定5个顶点，其中两个为固定点
    int vertex_id = 0;
    for (int i = 0; i < 5; ++i) {
        // 生成位移
        Vector3d trans(i*0.04-1.0f,0.0f,0.0f);
        // 旋转
        Eigen::Quaterniond q;
        q.setIdentity();
        Eigen::Isometry3d pose;
        // 对pose进行赋值
        pose = q;
        pose.translation() = trans;
        // 生成带有Camera信息的SE3
        g2o::VertexSCam* v_se3 = new g2o::VertexSCam();
        // 同样的设置ID
        v_se3->setId(vertex_id);
        // 因为VertexSCam继承自BaseVertex<6, Isometry3>,其中的measurement的类型为Isometry3
        v_se3->setEstimate(pose);
        // 这里会根据T和K生成w2n,w2i
        v_se3->setAll();            // set aux transforms
        // 设置成固定点
        if (i<2)
            v_se3->setFixed(true);
        // 优化器添加顶点
        optimizer.addVertex(v_se3);
        // 添加进向量中
        true_poses.push_back(pose);
        vertex_id++;
    }

    // 记录下随后的空间点的ID号
    int point_id=vertex_id;
    // 空间点的数量
    int point_num = 0;
    // 方差总和
    double sum_diff2 = 0;

    cout << endl;
    //
    unordered_map<int,int> pointid_2_trueid;
    unordered_set<int> inliers;

    for (size_t i=0; i<true_points.size(); ++i) {
        // 添加顶点
        g2o::VertexSBAPointXYZ* v_p = new g2o::VertexSBAPointXYZ();
        // 设置顶点的ID
        v_p->setId(point_id+i);
        v_p->setMarginalized(true);
        // 设定该顶点的估计值
        v_p->setEstimate(true_points.at(i) + Vector3d(Sample::gaussian(1), Sample::gaussian(1), Sample::gaussian(1)));

        // 判断该点能被多少帧看到
        int num_obs = 0;
        for (size_t j=0; j<true_poses.size(); ++j) {
            // 测量值
            Vector3d z;
            // 获取到该顶点的类型
            g2o::VertexSCam* vertex = dynamic_cast<g2o::VertexSCam*>(optimizer.vertex(j));
            // 把真实的空间点投影到图像坐标下
            // 这个z之所以是3个数字组成，最后的一维表示在右目下的图像系坐标
            vertex->mapPoint(z, true_points[i]);
            // 判断该点是否能被看到
            if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480) {
                ++num_obs;
            }
        }

        // 如果某个点可以被大多数点看到，则认为该点可以被作为顶点
        if (num_obs>=2) {
            // 向优化器中添加空间点的顶点
            optimizer.addVertex(v_p);
            //
            bool inlier = true;
            //
            for (size_t j=0; j<true_poses.size(); ++j) {
                Eigen::Vector3d z;
                // 获取到该顶点的类型
                g2o::VertexSCam* vertex = dynamic_cast<g2o::VertexSCam*>(optimizer.vertex(j));
                // 把真实的空间点投影到图像坐标下
                // 这个z之所以是3个数字组成，最后的一维表示在右目下的图像系坐标
                vertex->mapPoint(z, true_points[i]);
                //
                if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480) {
                    // 随机的对某个顶点进行outlier认定
                    double sam = Sample::uniform();
                    if (sam<0.3)
                    {
                        z = Vector3d(Sample::uniform(64,640), Sample::uniform(0,480), Sample::uniform(0,64)); // disparity
                        z(2) = z(0) - z(2); // px' now

                        inlier= false;
                    }
                    // 对量测值加入高斯噪声
                    z += Vector3d(Sample::gaussian(1), Sample::gaussian(1), Sample::gaussian(1/16.0));
                    // 定义一条边
                    g2o::Edge_XYZ_VSC* e = new g2o::Edge_XYZ_VSC();
                    // 设定边的两个顶点，0->空间点，1->顶点
                    e->vertices()[0] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p);
                    e->vertices()[1] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(j));
                    // 设定测量值
                    e->setMeasurement(z);
                    // 信息矩阵
                    e->information() = Matrix3d::Identity();
                    // 核
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    // 优化器添加一个边
                    optimizer.addEdge(e);
                }
            }
            // 如果是内点
            if (inlier)
            {
                // 添加空间点的ID到inlier列表中
                inliers.insert(point_id+i);
                // 计算方差
                Vector3d diff = v_p->estimate() - true_points[i];
                sum_diff2 += diff.dot(diff);
            }
            // 这个用于待会儿的寻找空间点的真值
            pointid_2_trueid.insert(make_pair(point_id+i,i));
            // 点数自加
            ++point_num;
        }
    }

    cout << endl;
    // 开始进行优化器的初始化
    optimizer.initializeOptimization();
    // 显示具体的信息
    optimizer.setVerbose(true);

    // 待了解
    bool STRUCTURE_ONLY = false;
    if (STRUCTURE_ONLY) {
        cout << "Performing structure-only BA:"   << endl;
        g2o::StructureOnlySolver<3> structure_only_ba;
        g2o::OptimizableGraph::VertexContainer points;
        for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
            g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
            if (v->dimension() == 3)
                points.push_back(v);
        }

        structure_only_ba.calc(points, 10);
    }

    cout << endl;
    cout << "Point error before optimisation (inliers only): " << sqrt(sum_diff2/point_num) << endl;
    // 开启全BA模式
    cout << "Performing full BA:" << endl;
    // 优化10次
    optimizer.optimize(10);
    // 后面要用到的变量置零
    point_num = 0;
    sum_diff2 = 0;
    // 对优化后的空间点计算方差
    for (unordered_map<int,int>::iterator it=pointid_2_trueid.begin(); it!=pointid_2_trueid.end(); ++it) {
        // 返回一个id映射到vertex的迭代器
        g2o::HyperGraph::VertexIDMap::iterator v_it = optimizer.vertices().find(it->first);

        if (v_it==optimizer.vertices().end()) {
            cerr << "Vertex " << it->first << " not in graph!" << endl;
            exit(-1);
        }

        // 返回空间点的顶点
        g2o::VertexSBAPointXYZ* v_p = dynamic_cast< g2o::VertexSBAPointXYZ*>(v_it->second);
        //
        if (v_p==0) {
            cerr << "Vertex " << it->first << "is not a PointXYZ!" << endl;
            exit(-1);
        }
        // 计算空间点与真值之间的差
        Vector3d diff = v_p->estimate()-true_points[it->second];
        // 判断是否是inlier,因为主要是看inlier之间的方差变化多少
        if (inliers.find(it->first)==inliers.end())
            continue;
        //
        sum_diff2 += diff.dot(diff);
        ++point_num;
    }
    cout << endl;
    cout << "Point error after optimisation (inliers only): " << sqrt(sum_diff2/point_num) << endl;
    cout << endl;
}
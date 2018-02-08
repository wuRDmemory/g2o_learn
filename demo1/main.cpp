#include "Eigen/StdVector"
#include "iostream"
#include "stdint.h"

#include "unordered_set"
#include "unordered_map"

/*
 * g2o的基本内容
 * optimizer----->algorithm(LM.GN,Powell)
 *                       ||
 *                       \/
 *       Matrix<-------solver----->linearSolver
 *
 *
 */

/* g2o的头文件  */
// 稀疏的优化器
#include "g2o/core/sparse_optimizer.h"
// block_solver
#include "g2o/core/block_solver.h"
// 求解器
#include "g2o/core/solver.h"
// 核
#include "g2o/core/robust_kernel_impl.h"
// 优化的算法,LM算法
#include "g2o/core/optimization_algorithm_levenberg.h"
// 线性求解器的方法
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
// 顶点和边的定义
#include "g2o/types/sba/types_six_dof_expmap.h"
//
#include "g2o/solvers/structure_only/structure_only_solver.h"

using namespace Eigen;
using namespace std;

// 模拟数据的采样过程
class Sample {
public:
    // 平均分布
    static double uniform_rand(double low, double up) {
        //
        return low + ((double)std::rand()/RAND_MAX)*(up - low);
    }
    // 高斯分布
    static double gaussian_rand(double mean, double sigma) {
        double x, y, r2;
        do {
            x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
            y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
            r2 = x * x + y * y;
        } while (r2 >= 1.0 || r2 == 0.0);
        return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
    }
    // 均匀分布
    static int uniform(int from, int to) {
        return static_cast<int>(uniform_rand(from, to));
    }
    // 0-1间的均匀分布
    static double uniform() {
        return uniform_rand(0, 1);
    }
    // 均值为0的高斯分布
    static double gaussian(double sigma) {
        return gaussian_rand(0, sigma);
    }
};

int STRUCTURE_ONLY = 0;

// 主函数
int main(int argc, char* argv[]) {
    /*  G2O的初始化  */
    // 首先声明一个优化器
    g2o::SparseOptimizer optimizer;
    // 隐藏详细的信息
    optimizer.setVerbose(false);
    // 线性求解器, 这里声明为智能指针
    // 这里，LinearSolverType（LinearSolver<PoseMatrixType>）是父类，LinearSolverCholmod是继承自LinearSolverType的
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> >();
    // 迭代算法
    // OptimizationAlgorithmLevenberg初始化必须要一个unique_ptr<Solver>
    // BlockSolver是Solver的子类,BlockSolver初始化的时候需要一个unique_ptr<linearSolver>
    g2o::OptimizationAlgorithmLevenberg* algorithmLevenberg = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3 >(std::move(linearSolver)));
    // 设定优化器的算法
    optimizer.setAlgorithm(algorithmLevenberg);
    /*   G2O基本设置完毕   */

    /*   模拟采样    */
    // 真实的点
    std::vector<Eigen::Vector3d, aligned_allocator<Eigen::Vector3d> > truePoints;
    truePoints.reserve(500);
    // 产生点值,500个
    for(int i=0;i<500;i++) {
        truePoints.push_back(Eigen::Vector3d((Sample::uniform()-0.5)*3, Sample::uniform()-0.5, Sample::uniform()+3));
    }

    // 相机的参数
    double focal_length = 1000.;
    Vector2d principal_point(320, 240);
    g2o::CameraParameters* cameraParameters = new g2o::CameraParameters(1000., principal_point, 0.);
    cameraParameters->setId(0);

    // 位姿的向量
    // aligned_allocator
    vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > truePose;
    // 生成15个位姿
    if(!optimizer.addParameter(cameraParameters)) {
        assert(false);
    }

    int vertex_idx = 0;
    for(vertex_idx=0;vertex_idx<15;vertex_idx++) {
        // 生成位移量
        Vector3d trans(vertex_idx*0.04-1.0, 0, 0);
        // 生成四元数
        Eigen::Quaterniond q;
        q.setIdentity();
        // 位姿
        g2o::SE3Quat pose(q, trans);
        // 顶点
        g2o::VertexSE3Expmap* v_se3 = new g2o::VertexSE3Expmap();
        // 设定顶点的下标
        v_se3->setId(vertex_idx);
        // 设定固定点
        if(vertex_idx < 2) {
            v_se3->setFixed(true);
        } else {
            v_se3->setFixed(false);
        }
        // 设定顶点的估计值
        v_se3->setEstimate(pose);
        // 把顶点加到优化器中
        optimizer.addVertex(v_se3);
        // 记录下当前的顶点数据
        truePose.push_back(pose);
    }

    // 生成空间点的顶点
    int point_idx = vertex_idx;
    vector<int> inliers;
    double sum_diff2 = 0;
    // 同时添加边
    for(int i=0, len=truePoints.size();i<len;i++) {
        // 拿出空间点来
        Vector3d point = truePoints[i];
        // 空间点的顶点
        g2o::VertexSBAPointXYZ* pointXYZ = new g2o::VertexSBAPointXYZ();
        // 设定空间点的id号
        pointXYZ->setId(point_idx+i);
        // schur消元
        // 先计算位姿的值
        pointXYZ->setMarginalized(true);
        // 设定该顶点的估计值
        // 这里的估计值是eigen的数据类型
        Eigen::Vector3d estimate(Sample::gaussian(1),Sample::gaussian(1),Sample::gaussian(1));
        estimate = estimate + point;
        pointXYZ->setEstimate(estimate);

        /*   下面是边与顶点对应过程   */
        int num_obs = 0;
        // 先算这个点有多少位姿能看到,这里比较绝对了，只要在视野内就算看到，其实实际上不然
        for (int j = 0; j < truePose.size(); ++j) {
            // cam_map是相机坐标系到图像坐标系的映射
            // map是空间点经过位姿映射
            Eigen::Vector2d imagePoint = cameraParameters->cam_map(truePose[j].map(point));
            // 判断该点是否在图像中
            if (imagePoint[0] > 0 && imagePoint[1] > 0 && imagePoint[0] < 640 && imagePoint[1] < 480) {
                ++num_obs;
            }
        }

        // 如果这个点被超过两个位姿看到了，则可以构成一个图优化
        // 先添加顶点，再添加边
        if(num_obs > 2) {
            // 把顶点添加进去
            optimizer.addVertex(pointXYZ);
            // 为该点添加边
            for (int k = 0; k < truePose.size(); ++k) {
                // 把位姿拿出来
                g2o::SE3Quat pose = truePose[k];
                //
                Eigen::Vector2d z = cameraParameters->cam_map(pose.map(point));
                // 判断该点是否能被这个位姿看到
                if (z[0] > 0 && z[1] > 0 && z[0] < 640 && z[1] < 480) {
                    // 如果可以看到，则加入边
                    // 0->point
                    // 1->pose
                    g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
                    // 边的设置
                    edge->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(pointXYZ));
                    edge->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(k)));
                    // 设定量测值
                    edge->setMeasurement(z);
                    // 信息矩阵
                    edge->information() = Eigen::Matrix2d::Identity();
                    // 核
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    edge->setRobustKernel(rk);
                    // 参数
                    edge->setParameterId(0,0);
                    // 把边添加到优化器中
                    optimizer.addEdge(edge);
                }
            }

            // inlier
            inliers.push_back(point_idx+i);
            // 计算方差
            Eigen::Vector3d diff = pointXYZ->estimate();
            diff -= truePoints[i];
            sum_diff2 += diff.dot(diff);
        }
    }

    //
    if (STRUCTURE_ONLY){
        g2o::StructureOnlySolver<3> structure_only_ba;
        cout << "Performing structure-only BA:"   << endl;
        g2o::OptimizableGraph::VertexContainer points;
        for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
            g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
            if (v->dimension() == 3)
                points.push_back(v);
        }
        structure_only_ba.calc(points, 10);
    }

    // 输出优化前的误差
    std::cout<<"优化前： 误差为"<<sum_diff2/inliers.size()<<endl;
    optimizer.save("testbefore.g2o");
    // 开始优化
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    cout << endl;
    cout << "Performing full BA:" << endl;
    optimizer.optimize(10);
    cout << endl;
    // 清除方差
    sum_diff2 = 0;
    int inlier_cnt = 0;
    // 计算优化之后空间点的方差
    for (auto its = inliers.begin(), ends = inliers.end(); its!=ends; its++) {
        // 查找空间点对应的数据
        g2o::HyperGraph::VertexIDMap::iterator it = optimizer.vertices().find(*its);
        // 判断是否找到了
        if(it == optimizer.vertices().end()) {
            std::cerr << "Vertex " << *its << " not in graph!" << endl;
            exit(-1);
        }
        // 如果找到的话
        // 计算方差
        g2o::VertexSBAPointXYZ* sbaPointXYZ = dynamic_cast<g2o::VertexSBAPointXYZ*>(it->second);
        Eigen::Vector3d diff = sbaPointXYZ->estimate();
        diff -= truePoints[*its - point_idx];
        sum_diff2 += diff.dot(diff);
        inlier_cnt++;
    }
    std::cout<<"优化后： 误差为"<<sum_diff2/inlier_cnt<<endl;
    optimizer.save("testafter.g2o");
    return 1;
}

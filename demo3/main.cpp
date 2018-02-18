#include <iostream>
#include <cmath>

#include "simulator.h"

#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "edge_se2.h"
#include "edge_se2_point.h"
// #include "types_tutorial_slam2d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;

int main()
{
    // simulate a robot observing landmarks while travelling on a grid
    // the transform between sensor and body
    SE2 sensorOffsetTransf(0.2, 0, 0);
    // number of pose
    int numNodes = 300;
    Simulator simulator;
    // this will generate 300 poses and map
    simulator.simulate(numNodes, sensorOffsetTransf);

    // blocksolver
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // allocating the optimizer
    SparseOptimizer optimizer;
    // linear solver
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    // use GN algorithm
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);

    // add the parameter representing the sensor offset
    ParameterSE2Offset* sensorOffset = new ParameterSE2Offset;
    sensorOffset->setOffset(sensorOffsetTransf);
    sensorOffset->setId(0);
    optimizer.addParameter(sensorOffset);

    // adding the odometry to the optimizer
    // first adding all the vertices
    cout << "Optimization: Adding robot poses ... "<<endl;
    // add vertex
    for (size_t i = 0; i < simulator.poses().size(); ++i) {
        const Simulator::GridPose& p = simulator.poses()[i];
        const SE2& t = p.simulatedPose;
        VertexSE2* robot =  new VertexSE2;
        robot->setId(p.id);
        robot->setEstimate(t);
        optimizer.addVertex(robot);
    }
    cerr << "add vertex done." << endl;

    // second add the odometry constraints
    cerr << "Optimization: Adding odometry measurements ... ";
    // pose graph's edge
    for (size_t i = 0; i < simulator.odometry().size(); ++i) {
        const Simulator::GridEdge& simEdge = simulator.odometry()[i];

        EdgeSE2* odometry = new EdgeSE2;
        odometry->vertices()[0] = optimizer.vertex(simEdge.from);
        odometry->vertices()[1] = optimizer.vertex(simEdge.to);
        odometry->setMeasurement(simEdge.simulatedTransform);
        odometry->setInformation(simEdge.information);
        optimizer.addEdge(odometry);
    }
    cerr << "done." << endl;

    // add the landmark observations
    // attention: these landmarks are observed by pose
    cerr << "Optimization: add landmark vertices ... ";
    for (size_t i = 0; i < simulator.landmarks().size(); ++i) {
        const Simulator::Landmark& l = simulator.landmarks()[i];
        VertexPointXY* landmark = new VertexPointXY;
        landmark->setId(l.id);
        landmark->setEstimate(l.simulatedPose);
        optimizer.addVertex(landmark);
    }
    cerr << "done." << endl;

    cerr << "Optimization: add landmark observations ... ";
    // add project relations
    for (size_t i = 0; i < simulator.landmarkObs().size(); ++i) {
        const Simulator::LandmarkEdge& simEdge = simulator.landmarkObs()[i];
        EdgeSE2PointXY* landmarkObservation =  new EdgeSE2PointXY;
        landmarkObservation->vertices()[0] = optimizer.vertex(simEdge.from);
        landmarkObservation->vertices()[1] = optimizer.vertex(simEdge.to);
        landmarkObservation->setMeasurement(simEdge.simulatedMeas);
        landmarkObservation->setInformation(simEdge.information);
        landmarkObservation->setParameterId(0, sensorOffset->id());
        optimizer.addEdge(landmarkObservation);
    }
    cerr << "done." << endl;

    // dump initial state to the disk
    optimizer.save("tutorial_before.g2o");

    // prepare and run the optimization
    // fix the first robot pose to account for gauge freedom
    VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(optimizer.vertex(0));
    firstRobotPose->setFixed(true);
    optimizer.setVerbose(true);

    cerr << "Optimizing" << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cerr << "done." << endl;

    optimizer.save("tutorial_after.g2o");

    // freeing the graph memory
    optimizer.clear();

    // destroy all the singletons
    Factory::destroy();
    OptimizationAlgorithmFactory::destroy();
    HyperGraphActionLibrary::destroy();

    return 0;
}

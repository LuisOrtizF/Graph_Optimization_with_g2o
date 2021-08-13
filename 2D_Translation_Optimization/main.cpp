#include <iostream>
#include <vector>
#include <cmath>

#include <g2o/stuff/sampler.h>
#include <g2o/stuff/command_args.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

using namespace std;

double gt_error(Eigen::Vector2d v1, Eigen::Vector2d v2)
{
    return sqrt((v1.x() - v2.x())*(v1.x() - v2.x()) + (v1.y() - v2.y())*(v1.y() - v2.y()));
}

int main(int argc, char** argv)
{
    float noise_std_dev;
    int num_obs;

    g2o::CommandArgs arg;
    arg.param("noise", noise_std_dev, 0.5, "standard deviation of the measurement noise");
    arg.param("obs", num_obs, 50, "perform n observations (edges) about the 2D translation");

    arg.parseArgs(argc, argv);

    //True translation
    Eigen::Vector2d transl(5.0, 0.0);

    //Typedefs
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> >  Block_Solver;
    typedef g2o::LinearSolverCSparse<Block_Solver::PoseMatrixType> Linear_Solver;

    //Setup the solver
    auto linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
    g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);
    optimizer.setAlgorithm(optimizationAlgorithm);

    //Add first node (no estimation errors and fixed)
    g2o::VertexPointXY* first = new g2o::VertexPointXY;
    first->setId(0);
    first->setFixed(true);
    first->setEstimate(Eigen::Vector2d(0.0, 0.0));
    optimizer.addVertex(first);

    //Add second node
    g2o::VertexPointXY* second = new g2o::VertexPointXY;
    second->setId(1);
    optimizer.addVertex(second);

    //Generate observations (edges) for the translation between the first and second nodes
    for(int i = 0; i < num_obs; i++)
    {
        float nx = g2o::Sampler::gaussRand(0, noise_std_dev);
        float ny = g2o::Sampler::gaussRand(0, noise_std_dev);
        
        Eigen::Vector2d meas = transl + Eigen::Vector2d(nx, ny);

        //Estimate is the first edge
        if(i == 0)
        {
            second->setEstimate(meas);
            //second->setEstimate(Eigen::Vector2d(0.0, 0.0)); //initialize with 0
        }

        g2o::EdgePointXY* e = new g2o::EdgePointXY;
        e->vertices()[0] = first;
        e->vertices()[1] = second;
        e->setMeasurement(meas);
        optimizer.addEdge(e);
    }

    double dist = 0.0;

    optimizer.computeActiveErrors();
    printf("Initial chi2: %f\n", optimizer.activeChi2());
    dist = gt_error(second->estimate(), transl);
    printf("Initial estimate: %f, %f (error: %f)\n", second->estimate().x(), second->estimate().y(), dist);

    printf("Optimizing\n");
    optimizer.initializeOptimization();
    optimizer.optimize(100);

    printf("Final chi2: %f\n", optimizer.activeChi2());
    dist = gt_error(second->estimate(), transl);
    printf("Final estimate: %f, %f (error: %f)\n", second->estimate().x(), second->estimate().y(), dist);

    return 0;
}
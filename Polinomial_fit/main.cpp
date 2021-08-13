#include <iostream>
#include <vector>
#include <cmath>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

//Solvers
#include <g2o/solvers/dense/linear_solver_dense.h>

//Utility includes
#include <g2o/stuff/sampler.h>
#include <g2o/stuff/command_args.h>

//Classes for used vertices and edges in the optimization
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>

using namespace std;

//Vertex: represents the state variables to be optimized
//  >>> Dimensions: number of dimensions of the state vector;
//      for example, a line has 2 dimensions, a plane has 3 dimensions,
//      a 3D pose has 6 dimensions, etc.
//  >>> Type: C++ type informing how the state vector is stored
//      within the vertex class;
//      for example, a line might be stored as Eigen::Vector2d,
//      a plane as Eigen::Vector3d, a 3D pose as Eigen::Affine3d
//      (or the type SE3 supplied by G2O) 
class VertexPolinomial: public g2o::BaseVertex<3, Eigen::Vector3d>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //Default constructor
        VertexPolinomial()
        {

        }

        //Set vertex state to origin
        virtual void setToOriginImpl()
        {
            //set _estimate to 0 according to the vertex type
            _estimate << 0, 0, 0;
        }

        //*** Update the vertex state
        virtual void oplusImpl(const double* update)
        {
            //apply an infinitesimal increment to the vertex state i.e.
            //add update to _estimate
            // Eigen::Vector3d v(update);
            // _estimate += v;
            _estimate += Eigen::Vector3d(update[0], update[1], update[2]);
        }

        //Read vertex from file
        //(this function should be supplied)
        virtual bool read(istream&)
        {
            cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
            return false;
        }

        //Write vertex to file
        //(this function should be supplied)
        virtual bool write(ostream&) const
        {
            cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
            return false;
        }
};

//Edge: represents a measurement for the estimated vertices
//  >>> Dimensions: number of dimensions of the edge;
//      for example, a point on a line has 2 dimensions,
//      a point on a 3D plane has 3 dimensions,
//      a relative transformation between two 3D poses has 6 dimensions, etc.
//  >>> Type: C++ type informing how the edge measurement is stored
//      within the edge class;
//      for example, a point on a line might be stored as Eigen::Vector3d,
//      a point on a 3D plane as Eigen::Vector3d, a relative transformation
//      between two 3D poses as Eigen::Affine3d (or the type SE3 supplied by G2O) 
class EdgePointOnPolinomial : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPolinomial>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //Default constructor
        EdgePointOnPolinomial()
        {

        }

        //Compute error on edge
        void computeError()
        {
            //This member function should store in the member _error
            //the difference between the edge measurement (returned by the measurement() member function or by accessing member _measurement)
            //and the estimate of the vertices linked by the edge (returned by the estimate() member function of the vertex class)

            //Get the first vertex linked by the edge
            const VertexPolinomial* v = static_cast<const VertexPolinomial*>(_vertices[0]);

            //update _error using _measurement and v->estimate() 

            const double& a = v->estimate()(0);
            const double& b = v->estimate()(1);
            const double& c = v->estimate()(2);

            double yp = a * measurement().x()*measurement().x() + b*measurement().x() + c;
            _error(0) = measurement().y() - yp;
        }

        //Read edge from file
        //(this function should be supplied)
        virtual bool read(istream&)
        {
            cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
            return false;
        }

        //Write vertex from file
        //(this function should be supplied)
        virtual bool write(ostream&) const
        {
            cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
            return false;
        }

        //Remember that numeric Jacobians are being used
        //as provided by G2O
};

int main(int argc, char** argv)
{
    float noise_std_dev;
    int num_obs;
    int max_iterations;

    g2o::CommandArgs arg;
    arg.param("n", noise_std_dev, 0.1, "standard deviation of the measurement noise");
    arg.param("obs", num_obs, 50, "number of observations");
    arg.param("iter", max_iterations, 100, "maximum number of iterations");

    arg.parseArgs(argc, argv);

    //True parameters of the estimated vertices
    double a = 1.0;
    double b = 2.0;
    double c = 3.0;

    //Generate ground truth for the line and noisy observations
    Eigen::Vector2d* points = new Eigen::Vector2d[num_obs];

    for(size_t i = 0; i < num_obs; i++)
    {
        //Sample x from the [0,10] interval
        double xi = g2o::Sampler::uniformRand(0, 10);
        //Get y from the sampled x
        double yi = a*xi*xi + b*xi + c;

        //Add Gaussian noise to the sampled point
        yi += g2o::Sampler::gaussRand(0, noise_std_dev);

        points[i].x() = xi;
        points[i].y() = yi;
    }

    //Typedefs
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> > Block_Solver;
    typedef g2o::LinearSolverDense<Block_Solver::PoseMatrixType> Linear_Solver;

    //Setup the solver
    unique_ptr<Linear_Solver> linearSolver = g2o::make_unique<Linear_Solver>();
    unique_ptr<Block_Solver> blockSolver = g2o::make_unique<Block_Solver>(std::move(linearSolver));
    g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    //Setup the optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);
    optimizer.setAlgorithm(optimizationAlgorithm);

    //Build the graph problem: 
    //-Add vertices
    VertexPolinomial* vertices = new VertexPolinomial();
    vertices->setId(0);
    // vertices->setEstimate(Eigen::Vector3d(0,0)); // es necesaria solo en un problema no lineal
    optimizer.addVertex(vertices);

    //-Add num_obs edges for the observations
    for(size_t i = 0; i < num_obs; i++)
    {
        EdgePointOnPolinomial* e = new EdgePointOnPolinomial;
        e->setVertex(0, vertices);
        e->setMeasurement(points[i]);
        e->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(e);
    }

    //Get sum of errors before optimization
    optimizer.computeActiveErrors();
    printf("Initial chi2: %f\n", optimizer.activeChi2());

    //Perform the optimization
    printf("Optimizing\n");
    optimizer.initializeOptimization();
    optimizer.optimize(max_iterations);

    printf("Final chi2: %f\n", optimizer.activeChi2());

    //Show optimized estimate: for each vertex, show its state (estimate() member function)
    cout << "a      = " << vertices->estimate()(0) << endl;
    cout << "b      = " << vertices->estimate()(1) << endl;
    cout << "c      = " << vertices->estimate()(2) << endl;
    cout << endl;

    // clean up
    delete[] points;

    return 0;
}
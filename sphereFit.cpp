#include <iostream>

#include <cmath>
#include <random>
#include <chrono>

#include "gaussNoise.h"
#include "sphereFit.h"

using namespace std;


// sphere funtion: (x - a)^2 + (y - b)^2 + (z - c)^2 = r^2

// h = (x - a)^2 + (y - b)^2 + (z - c)^2 - r^2

void SphereFit::getObsCoords(vector<double>& x_data, vector<double>& y_data, vector<double>& z_data)
{
    // the earth truth: use for simulate the sphere data
    double as = 100.0, bs = 200.0, cs = 300.0, rs = 27.0;

    // the count of point for simulate the sphere
    int N = 628;

    // simulate the sphere data ,added the GaussNoise
    for ( int i = 0; i < N; i++ )
    {
        GaussNoise gauseNoise ( 0,1 );
        double noise = gauseNoise.generateGaussNoise() / 200.0;

        double theta = i / 100.0;
        double phi = i / 100.0;

        double x = as + rs * std::cos ( theta ) * std::sin ( phi ) + noise;
        double y = bs + rs * std::sin ( theta ) * std::sin ( phi ) + noise;
        double z = cs + rs * std::cos ( phi ) + noise;

        x_data.push_back ( x );
        y_data.push_back ( y );
        z_data.push_back ( z );
    }
}

void SphereFit::setSphereParamInitail(const double x, const double y, const double z, const double r)
{
    this->sphereParam = {x,y,z,r};
}

vector<double>& SphereFit::getSpherePara()
{
    return this->sphereParam;
}

void SphereFit::fitCompute()
{
    ceres::Problem problem;
    
//     std::vector<double*>  parameterBlock= this->sphereParam; 
//     int sizeOfParameterBlock = sizeof(parameterBlock) / sizeof(parameterBlock[0]);
//     problem.AddParameterBlock(parameterBlock&,sizeOfParameterBlock,nullptr);
    
    vector<double> x_data, y_data,z_data;
    this->getObsCoords(x_data, y_data,z_data);
    
    for ( int i = 0; i < x_data.size(); i++ )
    {
        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<SphereFittingCost,1,4> (
            new SphereFittingCost ( x_data[i],y_data[i],z_data[i] ) );
        
        problem.AddResidualBlock (costFunction,new ceres::CauchyLoss ( 0.5 ),
                                  this->sphereParam.data());

    }
    
    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    solverOptions.minimizer_progress_to_stdout = true;
    
    ceres::Solver::Summary summary; 
    
    ceres::Solve(solverOptions,&problem,&summary);
    
//     cout << summary.BriefReport() << endl;
    cout << summary.FullReport() << endl;
    
    //  covariance compute
    ceres::Covariance::Options covarianceOptions;
    ceres::Covariance covariance ( covarianceOptions );

    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.push_back ( make_pair (this->getSpherePara().data(), 
                                             this->getSpherePara().data() ) );

    CHECK ( covariance.Compute(covariance_blocks, &problem ) );

    Eigen::Matrix<double,4,4,Eigen::RowMajor> covariance_abcr =
        Eigen::Matrix<double, 4,4,Eigen::RowMajor>::Zero();

    covariance.GetCovarianceBlock(this->getSpherePara().data(), this->getSpherePara().data(),
                                  covariance_abcr.data() );

    std::cout << endl << "covariance of abcr: " << endl;
    std::cout << covariance_abcr << endl;    
    
    std::vector<ceres::ResidualBlockId>* residual_blocks;
    problem.GetResidualBlocks(residual_blocks);
//     const double* values;
//     std::vector<ceres::ResidualBlockId>* residual_blocks;
//     problem.GetResidualBlocksForParameterBlock(values,residual_blocks);
//     cout << "test..." << endl;

}

int main ( int argc,char** argv )
{
    SphereFit sphereFit;
    
//     vector<double> x_data, y_data,z_data;
//     sphereFit.getObsCoords(x_data, y_data,z_data);
    
    sphereFit.setSphereParamInitail(90,210,280,26);
    
    sphereFit.fitCompute();
    
    return 0;

}

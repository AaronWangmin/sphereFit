#include <iostream>
#include <fstream>
#include <sstream>

#include <cmath>
#include <random>
#include <chrono>

#include "gaussNoise.h"
#include "sphereFit.h"

using namespace std;


// sphere funtion: (x - a)^2 + (y - b)^2 + (z - c)^2 = r^2

// h = (x - a)^2 + (y - b)^2 + (z - c)^2 - r^2

void SphereFit::getObsCoords(vector<tuple<string,vector<Point>>>& spherePointsVector,string prism)
{
    for(tuple<string,vector<Point>> prismData: spherePointsVector)
    {
        if(std::get<0>(prismData) == prism)
        {
            this->pointsVector = std::get<1>(prismData);
        }
    }
    
//     cout << endl << "prism: " << prism << " measured times: " << this->pointsVector.size() << endl; 
}

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


void SphereFit::setInitialParams()
{
    Point center;
    double radius;
    this->computeAverageCenterAndRadius(center,radius,this->pointsVector);

    this->params.resize(4,1);

    this->params = {center.x, center.y, center.z, radius};
   
}

void SphereFit::fitCompute()
{ 
    ceres::Problem problem; 
    
    for ( Point point : this->pointsVector )
    {
        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<SphereFittingCost,1,4> (
            new SphereFittingCost ( point ) );
        
        problem.AddResidualBlock (costFunction,new ceres::CauchyLoss ( 0.5 ),
                                  this->params.data());

    }
    
    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    solverOptions.minimizer_progress_to_stdout = false;
    
    ceres::Solver::Summary summary; 
    
    ceres::Solve(solverOptions,&problem,&summary);
    
//     cout << summary.BriefReport() << endl;
//     cout << summary.FullReport() << endl;
    
    //  covariance compute
    ceres::Covariance::Options covarianceOptions;
    ceres::Covariance covariance ( covarianceOptions );

    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.push_back ( make_pair (this->getParams().data(), 
                                             this->getParams().data() ) );

    CHECK ( covariance.Compute(covariance_blocks, &problem ) );

    Eigen::Matrix<double,4,4,Eigen::RowMajor> covariance_params =
        Eigen::Matrix<double, 4,4,Eigen::RowMajor>::Zero();

    covariance.GetCovarianceBlock(this->getParams().data(), this->getParams().data(),
                                  covariance_params.data() );

    std::cout << endl << "covariance of params: " << endl;
    std::cout << covariance_params << endl;    
//     
//     std::vector<ceres::ResidualBlockId>* residual_blocks;
//     problem.GetResidualBlocks(residual_blocks);
//     const double* values;
//     std::vector<ceres::ResidualBlockId>* residual_blocks;
//     problem.GetResidualBlocksForParameterBlock(values,residual_blocks);
//     cout << "test..." << endl;

    this->resultString.clear();
    if(summary.termination_type == ceres::CONVERGENCE)
    {
        stringstream ss;
        ss << this->pointsVector[0].prismName << "," 
           << this->pointsVector.size() << "," ;
            
        int countParmas = 4;
        for(int i = 0; i < countParmas; i++)
        {
            ss << this->params.data()[i] << ",";
        }
        
        for(int i = 0; i < countParmas; i++)
        {
            ss << covariance_params(i,i) << ",";
        }
        
        ss << endl;
        
        this->resultString = ss.str();            
    }

}


// int main ( int argc,char** argv )
// {
//     DataPrepare dp;
//     
//     string fileDir = "/home/vboxuser/projects/shapeFit_v3/data/0515.csv"; 
//     
//     dp.readObsFile(fileDir);
//     dp.selectPointsByPrims();
//     vector<tuple<string,vector<Point>>> spherePointsVector = dp.getSphereDataByPrims();
//     
//     vector<string> resultVector;
//     for(string prism : prismVector)
//     {
//         SphereFit fitter;         
//         fitter.getObsCoords(spherePointsVector,prism);
//         fitter.setInitialParams();
//         
//         fitter.fitCompute();
//         
//         resultVector.push_back(fitter.getResultString());
//         
//         cout << fitter.getResultString() << endl;
//     }
//     
//     string outFileDir = "../data/result_sphereFitted.txt";
//     putOutResultFile(outFileDir,resultVector);
//     
//     return 0;
//     
// }

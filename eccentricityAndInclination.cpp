#include <iostream>
#include <fstream>
#include <sstream>

#include <cmath>

#include "eccentricityAndInclination.h"

using namespace std;

void EccentricityAndInclinationFit::getObs(const vector<Point>& pointsVector)
{
    this->pointsVector = pointsVector;
}

void EccentricityAndInclinationFit::getParamsIndex(Point& point, int& index_a, int& index_b, int& index_OE, int& index_yaw, int& index_pitch)
{
    
//     vector<string> prismVector = {"A1","B1","C1","D1","A2","B2","C2","D2"};
    
//     vector<string> prismVector = {"A1"};
    auto prismIt = std::find(prismVector.begin(),prismVector.end(),point.prismName);
    int prismIndex = std::distance(prismVector.begin(), prismIt); 
    
//      8 ~ 15
    index_a = 8 + prismIndex;
//      16 ~ 23
    index_b = 8 + prismVector.size() + prismIndex;
//      24 ~ 31
    index_OE = 8 + prismVector.size() * 2 + prismIndex;
    
//     auto yawIt = std::find(yawVector.begin(),yawVector.end(),point.yaw);
//     int yawIndex = std::distance(yawVector.begin(), yawIt);
//     index_yaw = 8 + prismVector.size() * 3 + yawIndex;
//     
//     auto pitchIt = std::find(pitchVector.begin(),pitchVector.end(),point.pitch);
//     int pitchIndex = std::distance(pitchVector.begin(), pitchIt);
//     index_pitch = 8 + prismVector.size() * 3 + yawVector.size() + pitchIndex;
    
}


void EccentricityAndInclinationFit::setInitialParams()
{
    this->params.resize(dimensionParams,0.0); 
    
    this->params[0] = 953.670;
    this->params[1] = 1008.62;
    this->params[2] = 23.3908;
    
    this->params[3] = 0.0;    

    this->params[4] = degreeToRadians(0.0 / 3600);
    this->params[5] = degreeToRadians(0.0 / 3600);
    this->params[6] = degreeToRadians(0.0 / 3600);

    //  inital OA 
    this->params[7] = degreeToRadians(27.342027);
    
    //  initial a
    this->params[8 + prismVector.size() * 0 + 0] =  -2.079;
    this->params[8 + prismVector.size() * 0 + 1] =  1.791;
    this->params[8 + prismVector.size() * 0 + 2] = - 2.074;
    this->params[8 + prismVector.size() * 0 + 3] = - 1.963;
    
    this->params[8 + prismVector.size() * 0 + 4] =   2.032;  
    this->params[8 + prismVector.size() * 0 + 5] =   1.912;    
    this->params[8 + prismVector.size() * 0 + 6] = - 2.032;    
    this->params[8 + prismVector.size() * 0 + 7] = - 1.917;
    
     //  initial b    
    this->params[8 + prismVector.size() * 1 + 0] =   0.05;
    this->params[8 + prismVector.size() * 1 + 1] =   0.05;
    this->params[8 + prismVector.size() * 1 + 2] = - 0.05;
    this->params[8 + prismVector.size() * 1 + 3] = - 0.05;
    
    this->params[8 + prismVector.size() * 1 + 4] =   0.05;    
    this->params[8 + prismVector.size() * 1 + 5] =   0.05;    
    this->params[8 + prismVector.size() * 1 + 6] = - 0.05;    
    this->params[8 + prismVector.size() * 1 + 7] = - 0.05 ;
    
//  initial pitch 
    this->params[8 + prismVector.size() * 2 + 0] = degreeToRadians(353.0);
    this->params[8 + prismVector.size() * 2 + 1] = degreeToRadians(- 69);
    this->params[8 + prismVector.size() * 2 + 2] = degreeToRadians(-  7);
    this->params[8 + prismVector.size() * 2 + 3] = degreeToRadians(- 70);
    
    this->params[8 + prismVector.size() * 2 + 4] =  degreeToRadians(-  7);    
    this->params[8 + prismVector.size() * 2 + 5] =  degreeToRadians(- 69); 
    this->params[8 + prismVector.size() * 2 + 6] =  degreeToRadians(-  7);
    this->params[8 + prismVector.size() * 2 + 7] =  degreeToRadians(- 69);
    
//     for(int i = 0 ; i < pointsVector.size(); i++)
//     {
//       this->params[8 + prismVector.size() * 3 + i] = pointsVector[i].yaw * M_PI / 180.0;  
//     }
//     
//     for(int i = 0 ; i < pitchVector.size(); i++)
//     {
//       this->params[8 + prismVector.size() * 3 + prismVector.size() + i] = pointsVector[i].pitch * M_PI / 180.0;  
//     }
   
}

void EccentricityAndInclinationFit::fitCompute()
{ 
    ceres::Problem problem;     
    problem.AddParameterBlock(this->params.data(),dimensionParams);
    
//     problem.SetParameterLowerBound(this->params.data(), 0, 953.660);
//     problem.SetParameterUpperBound(this->params.data(), 0, 953.680);
//     problem.SetParameterLowerBound(this->params.data(), 1, 1008.61);
//     problem.SetParameterUpperBound(this->params.data(), 1, 1008.63);
//     problem.SetParameterLowerBound(this->params.data(), 2, 23.3907);
//     problem.SetParameterUpperBound(this->params.data(), 2, 23.3909);
    
//     problem.SetParameterLowerBound(this->params.data(), 3, 0.00000);
//     problem.SetParameterUpperBound(this->params.data(), 3, 0.000001);
    
//     problem.SetParameterLowerBound(this->params.data(), 4, 0.00000);
//     problem.SetParameterUpperBound(this->params.data(), 4, 0.000001);
//     problem.SetParameterLowerBound(this->params.data(), 5, 0.00000);
//     problem.SetParameterUpperBound(this->params.data(), 5, 0.000001);
//     problem.SetParameterLowerBound(this->params.data(), 6, 0.00000);
//     problem.SetParameterUpperBound(this->params.data(), 6, 0.000001);
    
//     problem.SetParameterLowerBound(this->params.data(), 7, degreeToRadians(0.00000));
//     problem.SetParameterUpperBound(this->params.data(), 7, degreeToRadians(2 * M_PI));
//     
//     problem.SetParameterLowerBound(this->params.data(), 10, degreeToRadians(0.00000));
//     problem.SetParameterUpperBound(this->params.data(), 10, degreeToRadians(2 * M_PI));
    
    
    int i = 0;
    for ( Point point : this->pointsVector )
    {
        auto prismIt = std::find(prismVector.begin(),prismVector.end(),point.prismName);
        if(prismIt != prismVector.end())
        {
            int index_a, index_b, index_OE ,index_yaw, index_pitch;
            this->getParamsIndex(point, index_a, index_b, index_OE, index_yaw, index_pitch); 
            
//             index_yaw = 8 + prismVector.size() * 3 + i;
//             index_pitch = 8 + prismVector.size() * 3 + prismVector.size() + i;
            
        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<SphereFittingCost,3,dimensionParams> (
            new SphereFittingCost ( point, index_a, index_b, index_OE, index_yaw, index_pitch ) );
        
            problem.AddResidualBlock (costFunction,new ceres::CauchyLoss ( 0.5 ),
                                  this->params.data());
        }
    }
    
//     ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering;
//     ordering->AddElementToGroup();
    
    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    solverOptions.minimizer_progress_to_stdout = false;
    solverOptions.max_num_iterations = 200;
    
    ceres::Solver::Summary summary; 
    
    ceres::Solve(solverOptions,&problem,&summary);
    
//     cout << summary.BriefReport() << endl;
    cout << summary.FullReport() << endl;
    
    ceres::Covariance::Options covarianceOptions;
    ceres::Covariance covariance ( covarianceOptions );

    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.push_back ( make_pair (this->getParams().data(), 
                                             this->getParams().data() ) );

    CHECK ( covariance.Compute(covariance_blocks, &problem ) );

    Eigen::Matrix<double,dimensionParams,dimensionParams,Eigen::RowMajor> covariance_params =
        Eigen::Matrix<double, dimensionParams,dimensionParams,Eigen::RowMajor>::Zero();

    covariance.GetCovarianceBlock(this->getParams().data(), this->getParams().data(),
                                  covariance_params.data() );

    std::cout << endl << "covariance of params: " << endl;
    std::cout << covariance_params << endl;  

    this->resultString.clear();
    if(summary.termination_type == ceres::CONVERGENCE)
    {
        stringstream ss;         
       
        for(int i = 0; i < dimensionParams; i++)
        {
            ss << this->params.data()[i] << ",";
        }
        
        for(int i = 0; i < dimensionParams; i++)
        {
            ss << covariance_params(i,i) << ",";
        }
        
        ss << endl;
        
        this->resultString = ss.str();   
        
        cout << endl << this->resultString;
    }

}


int main ( int argc,char** argv )
{
    DataPrepare dp;
    
    string fileDir = "/home/vboxuser/projects/shapeFit_v3/data/0515.csv"; 
    
    dp.readObsFile(fileDir);
    
    EccentricityAndInclinationFit fiter;
    fiter.getObs(dp.getAllObsVector());
    fiter.setInitialParams();
    fiter.fitCompute();
    
    
    
    return 0;
    
}

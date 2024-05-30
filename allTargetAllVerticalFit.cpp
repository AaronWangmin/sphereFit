#define EIGEN_STACK_ALLOCATION_LIMIT 0


#include "allTargetAllVerticalFit.h"

void AllTargetAllVerticalFit::getObs(const vector<Point>& obsVector)
{
    this->pointsVector = obsVector;
}

void AllTargetAllVerticalFit::setInitialParams(vector<tuple<vector<Point>,Point, double>>& horizenCircleVector)
{
    this->params.resize(dimensionParams,1);
    
//     this->params[0] = -953.672;
//     this->params[1] = -1008.62;  
//     this->params[2] = -23.3908; 
    
    this->params[0] = 0.0;
    this->params[1] = EIGEN_PI / (180 *3600.0); 
    this->params[2] = EIGEN_PI / (180 *3600.0); 
    
    Point center;
    double radius;
    this->computeAverageCenterAndRadius(center,radius,this->pointsVector);
    
    int zCoordParamIndex = 3;
    int radiusParamIndex = zCoordParamIndex + 55;    
    for(auto& horizenCircle : horizenCircleVector)
    {
        this->params[zCoordParamIndex] = std::get<1>(horizenCircle).z + this->params[2];
        
        this->params[radiusParamIndex] = std::get<2>(horizenCircle);
      
        zCoordParamIndex += 1;
        radiusParamIndex += 1;
    }
    
//     cout << "test:  setInitialParams()...." << endl;
}

void AllTargetAllVerticalFit::fitCompute(vector<tuple<vector<Point>,Point, double>>& horizenCircleVector)
{
//     cout << "test:  fitCompute()...." << endl;
    
    ceres::Problem problem; 
    
    int zCoordParamIndex = 3;
    int radiusParamIndex = zCoordParamIndex + 55; 
 
    for(auto& horizenCircle : horizenCircleVector)
    {
        for ( Point& point : std::get<0>(horizenCircle) )
        {
//              cout << "test:  fitCompute()... for ." << endl;
             
            ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AllTargetAllVerticalFittingCost,2,dimensionParams> (
            new AllTargetAllVerticalFittingCost ( point, zCoordParamIndex, radiusParamIndex ) );
        
//             cout << "test:  fitCompute()... costFunction() ." << endl;
            problem.AddResidualBlock (costFunction,new ceres::CauchyLoss ( 0.5 ),
                                this->params.data());
        }
        
        zCoordParamIndex += 1;
        radiusParamIndex += 1;
    }
    
    for(auto& horizenCircle : horizenCircleVector)
    {
        for ( Point& point : std::get<0>(horizenCircle) )
        {
//              cout << "test:  fitCompute()... for ." << endl;
             
            ceres::CostFunction* costFunction_2 = new ceres::AutoDiffCostFunction<AllTargetAllVerticalFittingCost_2,3,dimensionParams> (
            new AllTargetAllVerticalFittingCost_2 ( point, zCoordParamIndex, radiusParamIndex ) );
        
//             cout << "test:  fitCompute()... costFunction() ." << endl;
            problem.AddResidualBlock (costFunction_2,new ceres::CauchyLoss ( 0.5 ),
                                this->params.data());
        }
        
        zCoordParamIndex += 1;
        radiusParamIndex += 1;
    }
    
    
        
    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    solverOptions.minimizer_progress_to_stdout = false;
    
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
    
    covariance.Compute(covariance_blocks, &problem );

    Eigen::Matrix<double,dimensionParams,dimensionParams,Eigen::RowMajor> covariance_params =
        Eigen::Matrix<double, dimensionParams,dimensionParams,Eigen::RowMajor>::Zero();

    covariance.GetCovarianceBlock(this->getParams().data(), this->getParams().data(),
                                  covariance_params.data() );

    std::cout << endl << "covariance of params: " << endl;
    std::cout << covariance_params << endl; 
    
    this->resultString.clear();
    stringstream ss;
//     ss << get<0>(circleDatas)[0].prismName << "," 
//         << get<0>(circleDatas)[0].yaw << "_"
//         << get<0>(circleDatas)[0].pitch << ","                 
//         << get<0>(circleDatas).size() << "," ;
        

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





/*int main ( int argc,char** argv )
{  */  
//     DataPrepare dp;
//     
//     string fileDir = "/home/vboxuser/projects/shapeFit_v3/data/0515.csv"; 
//     
//     dp.readObsFile(fileDir);
//     
//     dp.prepareAllData();
//     
//     vector<tuple<vector<Point>,Point, double>> horizenCircleVector = dp.getHorienCircleDataVector();
//     horizenCircleVector.erase(horizenCircleVector.begin());     
//    
//     AllTargetAllVerticalFit fit;
//     fit.setInitialParams(horizenCircleVector);
//     
//     fit.fitCompute(horizenCircleVector);
    
//     vector<double> angleAxis = {2.24744, -2.11176, -0.591471};
//     angleAxisToEuro(angleAxis);
// }


#include "oneTargetAllVerticalFit.h"

const int OneTargetAllVerticalFit::dimensionParams = 23;

void OneTargetAllVerticalFit::getObs(const vector<Point>& obsVector, const string& prismName  )
{
    for(auto& point : obsVector)
    {
       if(point.prismName == prismName)
       { 
           this->pointsVector.push_back(point);
       }
    }
}

void OneTargetAllVerticalFit::setInitialParams()
{
    this->params.resize(dimensionParams,1);
    
    Point center;
    double radius;
    this->computeAverageCenterAndRadius(center,radius,this->pointsVector);
    
    this->params[0] = center.x;
    this->params[1] = center.y;  
    
    this->params[2] = 1.0;
    this->params[3] = 1.0; 
    this->params[4] = 1.0; 
    
    int pitchIndex = 0;    
    for(double& pitch : pitchVector)
    {
        vector<Point> hCirclePointVector;
        hCirclePointVector.clear();
        
        for(Point& point : this->pointsVector)
        {
            if(pitch == point.pitch)
            {
                hCirclePointVector.push_back(point); 
            }
        }
        
        Point hCircleCenter;
        double hCirecleRadius;
        this->computeAverageCenterAndRadius(hCircleCenter,hCirecleRadius,hCirclePointVector);
        
        this->params[5 + pitchIndex] = hCircleCenter.z;
        this->params[14 + pitchIndex] = hCirecleRadius;
        
        pitchIndex++;
    }

}

void OneTargetAllVerticalFit::fitCompute()
{
    ceres::Problem problem; 
    
    for(auto& pitch : pitchVector)
    {
        for ( Point point : this->pointsVector )
        {
            if (point.pitch == pitch)
            {
                int zCoordParamIndex,radiusParamIndex;                
                this->getZCoordAndRadiusIndex(point,zCoordParamIndex,radiusParamIndex);
        
                ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<OneTargetAllVerticalFittingCost,2,dimensionParams> (
                new OneTargetAllVerticalFittingCost ( point, zCoordParamIndex, radiusParamIndex ) );
            
                problem.AddResidualBlock (costFunction,new ceres::CauchyLoss ( 0.5 ),
                                    this->params.data());
                
            }
        }
        
        ceres::CostFunction* costFunction_2 = new ceres::AutoDiffCostFunction<OneTargetAllVerticalFittingCost_2,1,dimensionParams> (
new OneTargetAllVerticalFittingCost_2 () );

problem.AddResidualBlock (costFunction_2,new ceres::CauchyLoss ( 0.5 ),
                    this->params.data());
    }
    
    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
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
        
//     int countParmas = 7;
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

int OneTargetAllVerticalFit::getZCoordAndRadiusIndex(const Point& point, 
                                                     int& zCoordParamIndex,
                                                     int& radiusParamIndex 
                                                    ) const
{
//     vector<double> pitchVector = {5,15,25,35,45,55,65,75,85};
    
    auto pitchIt = std::find(pitchVector.begin(),pitchVector.end(),point.pitch); 
    
    int pitchIndex = std::distance(pitchVector.begin(), pitchIt);
    
    zCoordParamIndex = 5 + pitchIndex;
    
    radiusParamIndex = 14 + pitchIndex;
    
    return 0;
    
}

// int main ( int argc,char** argv )
// {
//     DataPrepare dp;
//     
//     string fileDir = "/home/vboxuser/projects/shapeFit_v3/data/0515.csv"; 
//     
//     dp.readObsFile(fileDir);
//     
//     vector<string> resultVector;
//     
//     prismVector.erase(std::remove(prismVector.begin(),prismVector.end(),"B1"),prismVector.end());
//     prismVector.erase(std::remove(prismVector.begin(),prismVector.end(),"B2"),prismVector.end());
//     prismVector.erase(std::remove(prismVector.begin(),prismVector.end(),"D1"),prismVector.end());
//     prismVector.erase(std::remove(prismVector.begin(),prismVector.end(),"D2"),prismVector.end());
//     
//     for(string prismName : prismVector)
//     {
//         OneTargetAllVerticalFit fit;
//         fit.getObs(dp.getAllObsVector(),prismName);
//         fit.setInitialParams();
//         fit.fitCompute(); 
//         
//         resultVector.push_back(fit.getResultString());
//         
//     }
//     
//     string outFileDir = "../data/result_oneTargetAllVerticalCircleFitted.txt";
//     putOutResultFile(outFileDir,resultVector); 
// }

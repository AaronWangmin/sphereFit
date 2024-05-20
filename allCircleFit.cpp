#include "allCircleFit.h"

void AllCircleFit::getObs(const vector<Point>& pointsVector)
{
    this->pointsVector = pointsVector;
}


void AllCircleFit::setInitialParams()
{
    Point center;
    double radius;
    this->computeAverageCenterAndRadius(center,radius,this->pointsVector);

    this->params.resize(dimensionParams,1);
    
    this->params[0] = center.x;
    this->params[1] = center.y;
    
    for(int i = 5; i < 23; i++)
    {
       this->params[i] = center.z; 
    }
    
}

void AllCircleFit::fitCompute()
{
    ceres::Problem problem; 
    
    for ( Point& point : this->pointsVector )
    {
        
        int startIndex = 5;
        int zParamIndex = this->getParamIndex(point,startIndex);
        
//         startIndex = 77;
        startIndex = 23;
        int radiusParamIndex = this->getParamIndex(point,startIndex);
        
        if(zParamIndex != 999999 || radiusParamIndex != 999999)
        {
            ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AllCircleFittingCost,1,dimensionParams> (
            new AllCircleFittingCost ( point, zParamIndex ,radiusParamIndex ) );
        
        problem.AddResidualBlock (costFunction,new ceres::CauchyLoss ( 0.5 ),
                                  this->params.data());
            
        }
    }
    
    //             costfunction_2
        ceres::CostFunction* costFunction_2 = new ceres::AutoDiffCostFunction<AllCircleFittingCost_2,1,dimensionParams> (
        new AllCircleFittingCost_2 ());
        
        problem.AddResidualBlock (costFunction_2,nullptr,
                                  this->params.data()); 
    
    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    solverOptions.minimizer_progress_to_stdout = false;
    
    ceres::Solver::Summary summary; 
    
    ceres::Solve(solverOptions,&problem,&summary);
    
//     cout << summary.BriefReport() << endl;
    cout << summary.FullReport() << endl;
    
    //  covariance compute
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

int AllCircleFit::getParamIndex(const Point& point,int startIndex) const
{
//     vector<string> prismVector = {"A1","B1","C1","D1","A2","B2","C2","D2"}; 
    vector<string> prismVector = {"A1","B1"}; 
    vector<double> pitchVector = {5,15,25,35,45,55,65,75,85};
    
    auto prismIt = std::find(prismVector.begin(),prismVector.end(),point.prismName);
    
    auto pitchIt = std::find(pitchVector.begin(),pitchVector.end(),point.pitch);    
    
    if(prismIt != prismVector.end() || pitchIt != pitchVector.end() )
    {
        int prismIndex = std::distance(prismVector.begin(), prismIt);  
        int pitchIndex = std::distance(pitchVector.begin(), pitchIt);
      
        int paramIndex = pitchVector.size() * prismIndex + pitchIndex + startIndex ;
    
        return paramIndex;
    }
    else
    {
        return 999999;
    }
      
}

int main ( int argc,char** argv )
{
    DataPrepare dp;
    
    string fileDir = "/home/vboxuser/projects/shapeFit_v3/data/0515.csv"; 
    
    dp.readObsFile(fileDir);
    
    AllCircleFit acf;
    acf.getObs(dp.getAllObsVector());
    acf.setInitialParams();
    acf.fitCompute();
}

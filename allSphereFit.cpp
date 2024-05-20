// #include <ceres/ceres.h>
// #include <Eigen/Dense>


#include "allSphereFit.h"

const int AllSphereFit::dimensionParams = 11;


void AllSphereFit::getObs(const vector<Point>& pointsVector)
{
    this->pointsVector = pointsVector;
}

void AllSphereFit::setInitialParams()
{
    Point center;
    double radius;
    this->computeAverageCenterAndRadius(center,radius,this->pointsVector);

    this->params.resize(dimensionParams,1);
    
    this->params[0] = center.x;
    this->params[1] = center.y;
    this->params[2] = center.z;
    this->params[3] = radius;
    
}

void AllSphereFit::fitCompute()
{
    ceres::Problem problem; 
    
    for ( Point point : this->pointsVector )
    {
        int prismIndex = this->getPrismIndex(point);
       
        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AllSphereFittingCost,1,dimensionParams> (
            new AllSphereFittingCost ( point, prismIndex ) );
        
        problem.AddResidualBlock (costFunction,new ceres::CauchyLoss ( 0.5 ),
                                  this->params.data());

    }
    
    
    
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

int AllSphereFit::getPrismIndex(const Point& point) const
{
    vector<string> prismVector = {"A1","B1","C1","D1","A2","B2","C2","D2"}; 
    
    auto it = std::find(prismVector.begin(),prismVector.end(),point.prismName);
    
    return it - prismVector.begin();    
        
}

// int main ( int argc,char** argv )
// {
//     DataPrepare dp;
//     
//     string fileDir = "/home/vboxuser/projects/shapeFit_v3/data/0515.csv"; 
//     
//     dp.readObsFile(fileDir);
//     
//     AllSphereFit asf;
//     asf.getObs(dp.getAllObsVector());
//     asf.setInitialParams();
//     asf.fitCompute();
// }

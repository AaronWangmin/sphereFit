#include <fstream>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include "lineFit.h"
#include "dataPrepare.h"

// void LineFit::getObscoords(vector<Result>& fitedCircleResultVector)
// {    
//     for(auto& result : fitedCircleResultVector)
//     {
//         this->pointsVector.push_back(Point(result.updatedParams[0],
//                                            result.updatedParams[1],
//                                            result.updatedParams[2],
//                                            result.yaw,
//                                            result.pitch,
//                                            result.fixTag,
//                                            result.primsName));
//     }
// }

void LineFit::getObs(const string dataFileDir)
{
    ifstream file(dataFileDir);
    if(!file.is_open())
    {
        cerr << "cant open file: " << dataFileDir << endl;        
    }
    
    string lineStr;
    for(int i = 0; i <3; i++)
    {
        getline(file,lineStr);
    }
    
    int indexPoint = 5;
    while(getline(file,lineStr))
    {
        string pointName = "point_" + std::to_string(indexPoint);
        
        vector<string> infoList;
        boost::split(infoList,lineStr,boost::is_any_of(",")); 
        
        string prismName = infoList[0];
        double angle = stod(infoList[1]);
        double countPoints = stod(infoList[2]);
        
        double N = stod(infoList[3]);
        double E = stod(infoList[4]);
        double H = stod(infoList[5]);
        
        Point point(pointName,prismName,N,E,H,angle,angle);
        this->pointsVector.push_back(point);
        
        indexPoint++;
        
    }
    
    file.close();
    
    cout << " count of circle-fitted: " << pointsVector.size() << endl; 
}

void LineFit::setInitialParams()
{
    this->params.resize(dimensionParams,1);
    
//     this->params[0] = 953.672;
//     this->params[1] = 1008.62;  
//     this->params[2] = 23.3908; 
    
    this->params[0] = 1;
    this->params[1] = 2;  
    this->params[2] = 3; 
}

// 
double LineFit::computeVertical(double x, double y, double z )
{
   double m = pow(x * x + y * y + z * z,0.5);
   double radian = std::acos( z / m);
   
//    unit vector 
//    radian = std::acos( z );
   
   return 180 * radian / M_PI ;
}

void LineFit::fitCompute()
{
    ceres::Problem problem;
    
//     plant cost function
    for (auto& point : this->pointsVector)
    {
//*** param: cost funtion, dimensions of residual, dimensions of unknown params 
        
        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<LineFittingCost,3,3> 
            (new LineFittingCost ( point) );
        
        problem.AddResidualBlock (costFunction,new ceres::CauchyLoss ( 0.5 ),
                                  this->params.data());
    }
        
    ceres::CostFunction* costFunction_2 = new ceres::AutoDiffCostFunction<CircleFittingCost_2,1,3> 
        (new CircleFittingCost_2 () );
    
    problem.AddResidualBlock (costFunction_2,new ceres::CauchyLoss ( 0.5 ),
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
    
    vector<double> rotation = {params.data()[0], params.data()[1], params.data()[2]};
    angleAxisToEuro(rotation);
        
}

int main ( int argc,char** argv )
{ 
    LineFit fit;    
    fit.getObs("../data/result_circleFitted.txt");
    fit.setInitialParams();    
    fit.fitCompute();
    
    
    
//     fit.report();
    
    return 0;
    
}

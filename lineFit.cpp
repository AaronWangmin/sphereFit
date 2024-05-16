#include <fstream>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include "lineFit.h"
#include "dataPrepare.h"

void LineFit::getObscoords(vector<Result>& fitedCircleResultVector)
{    
    for(auto& result : fitedCircleResultVector)
    {
        this->pointsVector.push_back(Point(result.updatedParams[0],
                                           result.updatedParams[1],
                                           result.updatedParams[2],
                                           result.yaw,
                                           result.pitch,
                                           result.fixTag,
                                           result.primsName));
    }
}

void LineFit::getObscoords(const string dataFileDir)
{
    ifstream file(dataFileDir);
    if(!file.is_open())
    {
        cerr << "cant open file: " << dataFileDir << endl;        
    }
    
    string lineStr;
//     for(int i = 0; i <5; i++)
//     {
//         getline(file,lineStr);
//     }
    
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

void LineFit::computeInitialParams()
{
    this->params.resize(6,1);
    
    int count = this->pointsVector.size();
    if(count > 0)
    {
        double sumN = 0, sumE = 0, sumH = 0;        
        for(const auto& point : this->pointsVector)
        {
            sumN += point.x;
            sumE += point.y;
            sumH += point.z; 
        }
        
//         Point circleCenterInitail(sumN / count, sumE / count, 22);
        double c_x = sumN / count;
        double c_y = sumE / count;
        double c_z = sumH / count;
        
        this->params[0] = c_x;
        this->params[1] = c_y;
        this->params[2] = c_z;
    }
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
        
        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<PlantFittingCost,2,6> 
            (new PlantFittingCost ( point) );
        
        problem.AddResidualBlock (costFunction,new ceres::CauchyLoss ( 0.5 ),
                                  this->params.data());
    }
    
//     plant constant cost function
//     ceres::CostFunction* constantCostFunction = new ceres::AutoDiffCostFunction<PlantFittingConstantCost,1,6> 
//         (new PlantFittingConstantCost () );
// 
//     problem.AddResidualBlock (constantCostFunction,new ceres::CauchyLoss ( 0.5 ),
//                                   this->params.data());
    
    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    solverOptions.minimizer_progress_to_stdout = true;
    
//     ceres::Solver::Summary summary;     
    ceres::Solve(solverOptions,&problem,&summary);    
    cout << summary.FullReport() << endl;
    
    cout << "updatedParams: " << endl;
    cout << params.data()[0] << endl;
    cout << params.data()[1] << endl;
    cout << params.data()[2] << endl;
    cout << params.data()[3] << endl;
    cout << params.data()[4] << endl;
    cout << params.data()[5] << endl;

    
//     for(auto& param : params.data)
//     {
//        cout << param << endl; 
//     }
     
    cout << "vertical: " << this->computeVertical(params[3],params[4],params[5]) << endl;
    
    //  covariance compute
    ceres::Covariance::Options covarianceOptions;
    ceres::Covariance covariance ( covarianceOptions );

    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.push_back ( make_pair (this->getParams().data(), 
                                             this->getParams().data() ) );

    CHECK ( covariance.Compute(covariance_blocks, &problem ) );

    Eigen::Matrix<double,6,6,Eigen::RowMajor> covariance_abcr =
        Eigen::Matrix<double, 6,6,Eigen::RowMajor>::Zero();

    covariance.GetCovarianceBlock(this->getParams().data(), this->getParams().data(),
                                  covariance_abcr.data() );

//***     std::cout << endl << "covariance of abcr: " << endl;
//***     std::cout << covariance_abcr << endl; 
    
//     this->result.primsName = get<0>(circleDatas)[0].prismName;
//     this->result.angle = get<0>(circleDatas)[0].pitch;
    this->result.fixTag = "pitch";
    this->result.countPoints = pointsVector.size();
    
//     this->result.initalParams = {get<1>(circleDatas).x,
//         get<1>(circleDatas).y,
//         get<1>(circleDatas).z,
//         get<2>(circleDatas) };
    
    this->result.updatedParams = {params.data()[0],
        params.data()[1],
        params.data()[2],
        params.data()[3],
        params.data()[4],
        params.data()[5] };
        
    this->result.sigamas = {covariance_abcr(0,0),
        covariance_abcr(1,1),
        covariance_abcr(2,2),
        covariance_abcr(3,3), 
        covariance_abcr(4,5),
        covariance_abcr(4,5) };
        
}


// *** test ***
// int main ( int argc,char** argv )
// {    
//     string fileDir = "/home/vboxuser/projects/shapeFit_v3/data/20240507-ZB07.csv";
//     string fileDir = "/home/vboxuser/projects/shapeFit_v3/data/20240507-ZB60_ave.csv";    
//     
//     CircleFit circleFit; 
//     circleFit.putOutAllCircleFitted(fileDir,"H");
//     
//  
//     LineFit linefit;    
//     linefit.getObscoords("../data/result_circleFitted.txt");
//     linefit.computeInitialParams();    
//     linefit.fitCompute();
//     
//     linefit.report();
//     
//     return 0;
//     
// }

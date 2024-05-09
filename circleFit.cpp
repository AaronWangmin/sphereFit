#include <iostream>
#include <fstream>

#include <cmath>
#include <chrono>

#include "circleFit.h"

using namespace std;

// circle funtion: (x - a)^2 + (y - b)^2 + (z - c)^2 = r^2

// h = (x - a)^2 + (y - b)^2 + (z - c)^2 - r^2

// void CircleFit::getObsCoords(vector<double>& x_data, 
//                              vector<double>& y_data, 
//                              vector<double>& z_data,
//                              tuple<vector<Point>,Point,double>& circleDatas)

void CircleFit::getObsCoords(tuple<vector<Point>,Point,double>& circleDatas)
{
    
    for(auto& point : get<0>(circleDatas))
    {
        x_data.push_back ( point.x );
        y_data.push_back ( point.y );
        z_data.push_back ( point.z );
    }
    
    
    Point circleCenterInitial = get<1>(circleDatas);
    this->circleParam = {circleCenterInitial.x,
                         circleCenterInitial.y,
                         circleCenterInitial.z,
        
                         get<2>(circleDatas)};
}

void CircleFit::setCircleParamInitail(const double x, 
                                      const double y, 
                                      const double z, 
                                      const double r)
{
    this->circleParam = {x,y,z,r};
}

vector<double>& CircleFit::getCirclePara()
{
    return this->circleParam;
}

void CircleFit::fitCompute(tuple<vector<Point>,Point,double>& circleDatas)
{
//     cout << "prims: " << get<0>(circleDatas)[0].prismName << endl << 
//         "angle: " << get<0>(circleDatas)[0].pitch << endl <<
//         "pointCount: " << get<0>(circleDatas).size() << endl <<
//         " circleCenterInitail x: "<<  get<1>(circleDatas).x << " , "  <<
//                                      "y: " << get<1>(circleDatas).y  << " , " << 
//                                      "z: " << get<1>(circleDatas).z  << " , " << endl <<
//         " radiumInitail : " << get<2>(circleDatas) << endl <<
//         "*****************************************" << endl ;
    
    ceres::Problem problem;
    
    this->getObsCoords(circleDatas);
    
    for ( int i = 0; i < x_data.size(); i++ )
    {
        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<CircleFittingCost,1,4> (
            new CircleFittingCost ( x_data[i],y_data[i],z_data[i] ) );
        
        problem.AddResidualBlock (costFunction,new ceres::CauchyLoss ( 0.5 ),
                                  this->circleParam.data());
    }
    
    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    solverOptions.minimizer_progress_to_stdout = false;
    
    ceres::Solver::Summary summary; 
    
    ceres::Solve(solverOptions,&problem,&summary);
    
//     cout << summary.BriefReport() << endl;
//     cout << summary.FullReport() << endl;
    
//     cout << "updated param: " << endl <<
//             "center_x: " << circleParam.data()[0] << endl <<
//             "center_y: " << circleParam.data()[1] << endl <<
//             "center_z: " << circleParam.data()[2] << endl <<
//             "radium: " << circleParam.data()[3] << endl ;
            
    
    //  covariance compute
    ceres::Covariance::Options covarianceOptions;
    ceres::Covariance covariance ( covarianceOptions );

    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.push_back ( make_pair (this->getCirclePara().data(), 
                                             this->getCirclePara().data() ) );

    CHECK ( covariance.Compute(covariance_blocks, &problem ) );

    Eigen::Matrix<double,4,4,Eigen::RowMajor> covariance_abcr =
        Eigen::Matrix<double, 4,4,Eigen::RowMajor>::Zero();

    covariance.GetCovarianceBlock(this->getCirclePara().data(), this->getCirclePara().data(),
                                  covariance_abcr.data() );

//     std::cout << endl << "covariance of abcr: " << endl;
//     std::cout << covariance_abcr << endl;  
    
// *****    outup result:
//          prims,angle,pointCount,
//          center_x_intial,center_y_initail,center_z_initail,radium_initial,
//          center_x_updated,center_y_updated,center_z_updated,radium_iupdated,  
//*****      rms_x,rmx_y,rmx_z,rmx_radium    
    cout << get<0>(circleDatas)[0].prismName << "," 
         << get<0>(circleDatas)[0].pitch << ","  
         << get<0>(circleDatas).size() << "," 
         
         << get<1>(circleDatas).x << ","  
         << get<1>(circleDatas).y << "," 
         << get<1>(circleDatas).z << "," 
         << get<2>(circleDatas) << ","  
         
         << circleParam.data()[0] << ","
         << circleParam.data()[1] << "," 
         << circleParam.data()[2] << ","  
         << circleParam.data()[3] << "," 
         
         << covariance_abcr(0,0) << "," 
         << covariance_abcr(1,1) << "," 
         << covariance_abcr(2,2) << "," 
         << covariance_abcr(3,3) << "," << endl;
    
}

int main ( int argc,char** argv )
{
    DataPrepare dp;
    
    string dataFileDir = "/home/vboxuser/projects/ceres_hello_202404/data/20240507-ZB60.csv";
    dp.readObsFile(dataFileDir);
    
    dp.prepareAllData();
    
    vector<tuple<vector<Point>,Point, double>> horienCircleDataVector = dp.getHorienCircleDataVector();  
    
    cout << "the count of horizenCircle: " << horienCircleDataVector.size() << endl;
    
    CircleFit circleFit; 
    
    for(auto& horienCircleData : horienCircleDataVector)
    {
//          circleFit.fitCompute(horienCircleDataVector[11]);
         circleFit.fitCompute(horienCircleData);
//          cout << "****************************" << endl;
       
    }
    
   
    
    return 0;

}

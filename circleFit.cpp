#include <iostream>
#include <fstream>
#include <sstream>

#include <cmath>
#include <chrono>

#include "circleFit.h"

using namespace std;

void CircleFit::getObsCoords(tuple<vector<Point>,Point,double>& circleDatas)
{
    this->pointsVector = std::get<0>(circleDatas);  
}

void CircleFit::setParamsInitail(const double x, 
                                      const double y, 
                                      const double z, 
                                      const double r)
{
    this->params.resize(7,1);
    this->params = {x,y,z,1,1,1,r};
}

void CircleFit::setParamsInitail(Point& center, double& radius)
{
    this->computeAverageCenterAndRadius(center,radius,this->pointsVector);
    
    this->params.resize(7,1);
    this->params = {center.x, center.y, center.z,1,1,1,radius};
}

void CircleFit::computeAverageCenterAndRadius(Point& center,double& radius,vector<Point>& pointsVector)
{

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

        center.x = sumN / count;
        center.y = sumE / count;
        center.z = sumH / count; 
        
        double sumRadius = 0;
        for(const auto& point : this->pointsVector)
        {
           double diff = std::sqrt(std::pow( (point.x - center.x),2) + 
                                        pow( (point.y - center.y),2) +
                                        pow( (point.z - center.z),2) );
           sumRadius += diff;
        }
        
        radius = sumRadius / count;
        
    }
    
}


vector<double>& CircleFit::getParams()
{
    return this->params;
}

int CircleFit::getCountFittedPoints()
{
    return this->pointsVector.size();
}

void CircleFit::fitCompute(tuple<vector<Point>,Point,double>& circleDatas)
{
    this->getObsCoords(circleDatas);
//     Point center;
//     double radius;
//     this->setParamsInitail(center,radius);
    
//     int countParmas = this->params.size();
    
    ceres::Problem problem;
    
    for (auto& point : this->pointsVector)
    {
//       param: cost funtion, dimensions of residual, dimensions of unknown params        
        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<CircleFittingCost,2,7> (
            new CircleFittingCost ( point) );
        
        problem.AddResidualBlock (costFunction,new ceres::CauchyLoss ( 0.5 ),
                                  this->params.data());
    }
    
//             costfunction_2
        ceres::CostFunction* costFunction_2 = new ceres::AutoDiffCostFunction<CircleFittingCost_2,1,7> (
        new CircleFittingCost_2 ());
        
        problem.AddResidualBlock (costFunction_2,nullptr,
                                  this->params.data());     
        
    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    solverOptions.minimizer_progress_to_stdout = false;
    
    ceres::Solver::Summary summary;     
    ceres::Solve(solverOptions,&problem,&summary);    
    cout << summary.BriefReport() << endl;
    
    if(summary.termination_type == ceres::CONVERGENCE)
    {
        ceres::Covariance::Options covarianceOptions;
        ceres::Covariance covariance ( covarianceOptions );

        std::vector<std::pair<const double*, const double*> > covariance_blocks;
        covariance_blocks.push_back ( make_pair (this->getParams().data(), 
                                                this->getParams().data() ) );

        CHECK ( covariance.Compute(covariance_blocks, &problem ) );

        Eigen::Matrix<double,7,7,Eigen::RowMajor> covariance_params =
            Eigen::Matrix<double, 7,7,Eigen::RowMajor>::Zero();

        covariance.GetCovarianceBlock(this->getParams().data(), this->getParams().data(),
                                    covariance_params.data() );

    //***     std::cout << endl << "covariance of abcr: " << endl;
    //***     std::cout << covariance_params << endl;
        
        this->resultString.clear();

        stringstream ss;
        ss << get<0>(circleDatas)[0].prismName << "," 
            << get<0>(circleDatas)[0].yaw << "_"
            << get<0>(circleDatas)[0].pitch << ","                 
            << get<0>(circleDatas).size() << "," ;
            
        int countParmas = 7;
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

void CircleFit::fitComputeVerticalCircle(tuple<vector<Point>,Point,double>& circleDatas)
{
    this->getObsCoords(circleDatas);
//     Point center;
//     double radius;
//     this->setParamsInitail(center,radius);
    
//     int countParmas = this->params.size();
    
    ceres::Problem problem;
    
    for (auto& point : this->pointsVector)
    {
//       param: cost funtion, dimensions of residual, dimensions of unknown params        
        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<CircleFittingCost,2,7> (
            new CircleFittingCost ( point) );
        
        problem.AddResidualBlock (costFunction,new ceres::CauchyLoss ( 0.5 ),
                                  this->params.data());
    }
    
//             costfunction_2
        ceres::CostFunction* costFunction_3 = new ceres::AutoDiffCostFunction<CircleFittingCost_3,2,7> (
        new CircleFittingCost_3 ());
        
        problem.AddResidualBlock (costFunction_3,nullptr,
                                  this->params.data());
        
//         z bound
        problem.SetParameterLowerBound(this->params.data(), 2, 23.2);
        problem.SetParameterUpperBound(this->params.data(), 2, 23.58);
        
        //         radium bound
        problem.SetParameterLowerBound(this->params.data(), 6, 0.2);
        problem.SetParameterUpperBound(this->params.data(), 6, 10);
    
    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    solverOptions.minimizer_progress_to_stdout = false;
    
    ceres::Solver::Summary summary;     
    ceres::Solve(solverOptions,&problem,&summary);    
    cout << summary.BriefReport() << endl;
    
    if(summary.termination_type == ceres::CONVERGENCE)
    {
        ceres::Covariance::Options covarianceOptions;
        ceres::Covariance covariance ( covarianceOptions );

        std::vector<std::pair<const double*, const double*> > covariance_blocks;
        covariance_blocks.push_back ( make_pair (this->getParams().data(), 
                                                this->getParams().data() ) );

        CHECK ( covariance.Compute(covariance_blocks, &problem ) );

        Eigen::Matrix<double,7,7,Eigen::RowMajor> covariance_params =
            Eigen::Matrix<double, 7,7,Eigen::RowMajor>::Zero();

        covariance.GetCovarianceBlock(this->getParams().data(), this->getParams().data(),
                                    covariance_params.data() );

    //***     std::cout << endl << "covariance of abcr: " << endl;
    //***     std::cout << covariance_params << endl;
        
        this->resultString.clear();

        stringstream ss;
        ss << get<0>(circleDatas)[0].prismName << "," 
            << get<0>(circleDatas)[0].yaw << "_"
            << get<0>(circleDatas)[0].pitch << ","                 
            << get<0>(circleDatas).size() << "," ;
            
        int countParmas = 7;
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

void CircleFit::putOutAllCircleFitted(const string& fileDir,const string& rotationAxis)
{
    DataPrepare dp;    
    dp.readObsFile(fileDir);    
    dp.prepareAllData();
    
    vector<string> resultVector;
    int countCirclePredicted = 0;    
    
    if(rotationAxis == "V")
    {
        vector<tuple<vector<Point>,Point, double>> horienCircleDataVector = dp.getHorienCircleDataVector();  
    
        countCirclePredicted = horienCircleDataVector.size();
        cout << "the count of horizenCircle: " << countCirclePredicted << endl;
        
        CircleFit circleFit; 
        resultVector.clear();
        for(auto& horienCircleData : horienCircleDataVector)
        {
            Point center;
            double radius;
            circleFit.computeAverageCenterAndRadius(center,radius,circleFit.getPointsVector());            
            circleFit.setParamsInitail(953.6736,1008.625,center.z,radius);
            
            circleFit.fitCompute(horienCircleData);
            
            resultVector.push_back(circleFit.getResultString());
        }
        
        horienCircleDataVector.clear();
    }
    
    if(rotationAxis == "H")
    { 
        vector<tuple<vector<Point>,Point, double>> verticalCircleDataVector = dp.getVerticalCircleDataVector();  
    
        countCirclePredicted = verticalCircleDataVector.size();
        cout << "the count of verticalCircle: " << countCirclePredicted << endl;
        
        CircleFit circleFit; 
        
        resultVector.clear();
        for(auto& circleData : verticalCircleDataVector)
        {
            Point center;
            double radius;
            circleFit.computeAverageCenterAndRadius(center,radius,circleFit.getPointsVector());            
            circleFit.setParamsInitail(center.x,center.y,23.3906,2);
            
            circleFit.fitComputeVerticalCircle(circleData);
            
            resultVector.push_back(circleFit.getResultString());
        }
        
        verticalCircleDataVector.clear();
    }
    
    int countCircleFitted = 0;
    for(string s : resultVector)
    {
        if(s.size() != 0)
        {
           countCircleFitted++; 
        }
        cout << "test...";
    }
    
    string outFileDir = "../data/result_circleFitted.txt"; 
    string capital = "prism, yaw-pitch, countOfPoints, updated_xyz,,,updated_abc,,,updated_r, sigma,,,,,,,";
    
    CircleFit::putOutResultFile(outFileDir,resultVector,capital);
    
}


void CircleFit::putOutResultFile(const string& outFileDir, const vector<string>& resultVector,const string& capital)
{
    int countFittedSucceed = 0;
    for(string s : resultVector)
    {
        if(s.size() != 0)
        {
           countFittedSucceed++; 
        }
    }
    
    ofstream outfile(outFileDir);
    
    if(!outfile)
    {
        cerr << "cant open file for writting: " << outFileDir << endl; 
    }

    outfile << "   count of succeed-fitted:  " << countFittedSucceed << endl;
    outfile << capital << endl;    
    
    for(auto& result : resultVector)
    {
        outfile << result;
    }
    
    outfile.close();
}



int main ( int argc,char** argv )
{
    string dataFileDir = "/home/vboxuser/projects/shapeFit_v3/data/0515.csv";
    CircleFit circleFit; 
    circleFit.putOutAllCircleFitted(dataFileDir,"V"); 
    
    return 0;

}

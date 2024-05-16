#ifndef CIRCLE_FIT_H
#define CIRCLE_FIT_H

#include <string>

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include "dataPrepare.h"

using namespace std;

struct Result
{
    vector<double> initalParams; 
    vector<double> updatedParams;
    vector<double> sigamas;
    int countPoints;
    string primsName;
    double yaw;
    double pitch;
//     double angle;
    string fixTag;
};

class CircleFit
{
public:
    
    CircleFit() {} 
    
    vector<Point>& getPointsVector() {return this->pointsVector;} 
    
    void computeInitialParams() {}

    void getObsCoords(tuple<vector<Point>,Point,double>& circleDatas);
    
    void setParamsInitail(Point& center,double& radius);
    
    void setParamsInitail(const double x, 
                               const double y, 
                               const double z, 
                               const double r);
    
    void computeAverageCenterAndRadius(Point& center,double& radius,vector<Point>& pointsVector);
    
//     horizen circle fit compute
    void fitCompute(tuple<vector<Point>,Point,double>& circleHorizenDatas);
    
    void fitComputeVerticalCircle(tuple<vector<Point>,Point,double>& circleHorizenDatas);
    
    Result getResult() const { return this->result;}
    
    string getResultString() const { return this->resultString;}
    
    void report();
    
    void putOutAllCircleFitted(const string& fileDir,const string& rotationAxis);
    
    vector<double>& getParams();
    
    void getSigma(const int countParams);
    
    
    int getCountFittedPoints();
    
    static void putOutResultFile(const string& outFileDir, const vector<string>& resultVector,const string& capital);
    
protected:
    vector<double> params;
    
    vector<Point> pointsVector;
    
    Eigen::Matrix<double,7,7,Eigen::RowMajor> covarianceParams;
    
//     result
    Result result;
    string resultString;
    
//     ceres params
//     ceres::Problem problem;
//     ceres::Solver::Summary summary;
//???     ceres::Covariance::Options covarianceOptions;
//???     ceres::Covariance covariance ( covarianceOptions );
    
};


// conditions:
//  1) plane  funtcion: a(x-x0) + b(y-y0) + c(z-z0) = 0
//  2) sphere function: (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = r^2
// 
//  3) model of normal vector: | a b c| = 1

// params : 7
//  1) plane circle center: x0,y0,z0
//  2) plane circle radium: r (sphere radium)
//  2) normal vector of plane: a, b, c  

struct CircleFittingCost
{
public:
    CircleFittingCost(const Point& point) : point(point) {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {
        residual[0] = 
            ceres::pow((T(point.x) - params[0]),2) + 
            ceres::pow((T(point.y) - params[1]),2) + 
            ceres::pow((T(point.z) - params[2]),2) - 
            ceres::pow(params[6],2) ;
           
            
        residual[1] = 
            params[3] * (T(point.x) - params[0]) +
            params[4] * (T(point.y) - params[1]) +
            params[5] * (T(point.z) - params[2]);   
            
        return true;
    }
    
private:
        const Point& point;
        
};

//  | a b c| = 1
struct CircleFittingCost_2
{
public:
    CircleFittingCost_2() {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {
        residual[0] = 
            params[3] * params[3] +
            params[4] * params[4] +
            params[5] * params[5] - T(1.0);
            
        return true;
    }
    
};

struct CircleFittingCost_3
{
public:
    CircleFittingCost_3() {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {
        residual[0] = 
            params[3] * params[3] +
            params[4] * params[4] +
            params[5] * params[5] - T(1.0);
            
        Point v(0.008,-0.0003,0.991952);
        residual[1] = 
            params[3] * T(v.x) +
            params[4] * T(v.y) +
            params[5] * T(v.z);  
            
        return true;
    }
    
};

#endif

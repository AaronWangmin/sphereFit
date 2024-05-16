#ifndef LINE_FIT_H
#define LINE_FIT_H

#include <Eigen/Dense>
#include <ceres/jet.h>

#include "circleFit.h"

class LineFit : public CircleFit
{
public:
    void getObscoords(vector<Result>& circleCenterVector);
    
    void getObscoords(const string dataFileDir);
    
    void computeInitialParams(); 
    
    double computeVertical(double x, double y, double z );
    
    void fitCompute();
        
};

//*** line function: p = p0 + t * d
//    params:   6 + n dimensions(p0: 3*1, d: 3*1 ,t: n)
//    residual: 3 dimensions
//*** 


struct LineFittingCost
{
public:
    LineFittingCost(const Point& point, const int indexPoint) : point(point),indexPoint(indexPoint) {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {
       int indexParam = 6 + indexPoint;
       
//        ceres::Jet<double,2> t;
       
//        auto t = ( (T(point.x) - params[0]) / params[3] +
//                (T(point.y) - params[1]) / params[4] +
//                (T(point.z) - params[2]) / params[4] ) / 3.0; 
//                
//        residual[0] = T(point.x) - params[0] - params[3] * t;
//        residual[1] = T(point.y) - params[1] - params[4] * t; 
//        residual[2] = T(point.z) - params[2] - params[5] * t;

       residual[0] = T(point.x) - params[0] - params[4] * params[indexParam];
       residual[1] = T(point.y) - params[1] - params[5] * params[indexParam]; 
       residual[2] = T(point.z) - params[2] - params[6] * params[indexParam];
       
       return true;
    }
    
private:
        const Point& point;
        const int indexPoint;        
};


struct PlantFittingCost
{
public:
    PlantFittingCost(const Point& point) : point(point) {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {
//        plant function: a(x-x0) + b(y-y0) + c(z-z0) = 0
        residual[0] = params[3] * (T(point.x) - params[0]) +
                      params[4] * (T(point.y) - params[1]) +
                      params[5] * (T(point.z) - params[2]);
                      
        residual[1] = ceres::sqrt(params[3] * params[3] + 
                                  params[4] * params[4] +
                                  params[5] * params[5]) -T(1.0);
                      
       return true;
    }
    
private:
        const Point& point;
        
};

// struct PlantFittingConstantCost
// {
// public:
//     PlantFittingConstantCost() {}
//     
//     template <typename T> 
//     bool operator()(const T* const params, T* residual) const
//     {
//        params constant function: ||a b c|| = 1
//         residual[0] = ceres::sqrt(params[3] * params[3] + 
//                                   params[4] * params[4] +
//                                   params[5] * params[5]) -T(1.0) ;
//        return true;
//     }
//       
// };



#endif

#ifndef LINE_FIT_H
#define LINE_FIT_H

#include <Eigen/Dense>
#include <ceres/jet.h>

// #include "circleFit.h"
#include "shapeFit.h"

class LineFit : public ShapeFit
{
public:
    static const int dimensionParams = 3;
    
    void getObs();
    
    void getObs(const string dataFileDir);
    
    void setInitialParams();

    void fitCompute(); 
    
    double computeVertical(double x, double y, double z );
        
};

//*** line function: p = p0 + t * d
//    params:   3  dimensions( d: 3*1)
//    residual: 3 dimensions
//*** 


struct LineFittingCost
{
public:
    LineFittingCost(const Point& point) : point(point) {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {
        T p0[3] = { T(953.672), T(1008.62), T(23.3908)};
       
        T t;
        t = ( (T(point.x) - p0[0]) / params[0] +
              (T(point.y) - p0[1]) / params[1] +
              (T(point.z) - p0[2]) / params[2] ) / T(3.0); 
               
       residual[0] = T(point.x) - p0[0] - params[0] * t;
       residual[1] = T(point.y) - p0[1] - params[1] * t; 
       residual[2] = T(point.z) - p0[2] - params[2] * t;
       
       return true;
    }
    
private:
        const Point& point;
//         const int indexPoint;        
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
            params[0] * params[0] +
            params[1] * params[1] +
            params[2] * params[2] - T(1.0);
            
        return true;
    }
    
};



#endif

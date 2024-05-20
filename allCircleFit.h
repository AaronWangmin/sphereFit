#ifndef ALL_CIRCLE_FIT_H
#define ALL_CIRCLE_FIT_H

#include "allSphereFit.h"


//      sphere center(x,y) + 
//      vertical-direct + 
//      targetCount * verticalCount * 1(targetVerticalCircle.z) + 
//      targetCount * verticalCount * 1(targetVerticalCircle.radius)
//      
//      2 + 3 + 8 * 9 + 8*9 = 149
//
// const int dimensionParams = 149;
const int dimensionParams = 41;


class AllCircleFit : public SphereFit
{
public:
    void getObs(const vector<Point>& );
    
    void setInitialParams();
    
    void fitCompute();
    
    int getParamIndex(const Point& point,int startIndex) const;
    
};

 

struct AllCircleFittingCost
{
public:
    AllCircleFittingCost(const Point& point,
                         const int& zCoordParamIndex,
                         const int& radiusParamIndex) 
    : point(point), 
      zCoordParamIndex(zCoordParamIndex),
      radiusParamIndex(radiusParamIndex)  {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    { 
        residual[0] = 
            ceres::pow((T(point.x) - params[0]),2) + 
            ceres::pow((T(point.y) - params[1]),2) + 
            ceres::pow((T(point.z) - params[zCoordParamIndex]),2) - 
            ceres::pow(radiusParamIndex,2) ;
            
        residual[1] = 
            params[2] * (T(point.x) - params[0]) +
            params[3] * (T(point.y) - params[1]) +
            params[4] * (T(point.z) - params[zCoordParamIndex]);   
            
        return true;
    }
    
private:
        const Point& point;
        int zCoordParamIndex;
        int radiusParamIndex;
        
};

//  | a b c| = 1
struct AllCircleFittingCost_2
{
public:
    AllCircleFittingCost_2() {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {
        residual[0] = 
            params[2] * params[2] +
            params[3] * params[3] +
            params[4] * params[4] - T(1.0);
            
        return true;
    }
    
};

#endif

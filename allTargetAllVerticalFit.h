#ifndef ALL_TARGET_ALL_VERTICAL_FIT_H
#define ALL_TARGET_ALL_VERTICAL_FIT_H

// #include "oneTargetAllVerticalFit.h"
#include "shapeFit.h"



class AllTargetAllVerticalFit : public ShapeFit
{
public:
    // dimensionParams: 3  + 110 = 113
        //  No translation (sphrer center x,y,z) : 3
        //  rotationAxis : 3  
        //  horizenCircle count * 2(zCoord + radius) : 55 * 2 =  110
   
    static const int dimensionParams = 113;

public: 
    
    void getObs(const vector<Point>&);
    
    void setInitialParams(vector<tuple<vector<Point>,Point, double>>& ); 
    
    void fitCompute(vector<tuple<vector<Point>,Point, double>>& ); 
     
    int getZCoordAndRadiusIndex(const Point& point,  int& zCoordParamIndex,
                                                     int& radiusParamIndex ) const;
                                                     
    
};

struct AllTargetAllVerticalFittingCost
{
public:
    AllTargetAllVerticalFittingCost(const Point& point, 
                                    const int& zCoordParammIndex, 
                                    const int& radiusParamIndex) 
        : point(point), 
          zCoordParammIndex(zCoordParammIndex),
          radiusParamIndex(radiusParamIndex) {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    { 
        T rotation[3] = {params[0],params[1],params[2]};
        T point_measured[3] = {T(point.x), T(point.y), T(point.z)};
        
        T point_platform[3];
        ceres::AngleAxisRotatePoint(rotation, point_measured, point_platform);
        
        point_platform[0] += T(953.672);
        point_platform[1] += T(1008.62);
        point_platform[2] += T(23.3908);
        
        residual[0] = pow( point_platform[0] - T(0.0), 2) + 
                      pow( point_platform[1] - T(0.0), 2) +
                      pow( point_platform[2] - params[this->zCoordParammIndex], 2) -
                      pow( params[this->radiusParamIndex], 2);
                      
                      
        T axisPlateform[3];
        T axisLocalCoord[3] = {T(0.0), T(0.0), T(1.0)};
        ceres::AngleAxisRotatePoint(rotation, axisLocalCoord, axisPlateform);
        
        residual[1] = axisPlateform[0] * (point_platform[0] - T(0.0)) +
                      axisPlateform[1] * (point_platform[1] - T(0.0)) +
                      axisPlateform[2] * (point_platform[2] - params[this->zCoordParammIndex]); 
        
        return true;
    }
       
private:
        Point point;         
        int zCoordParammIndex;
        int radiusParamIndex;
};

struct AllTargetAllVerticalFittingCost_2
{
public:
    AllTargetAllVerticalFittingCost_2(const Point& point, 
                                    const int& zCoordParammIndex, 
                                    const int& radiusParamIndex) 
        : point(point), 
          zCoordParammIndex(zCoordParammIndex),
          radiusParamIndex(radiusParamIndex) {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    { 
        T rotation[3] = {params[0],params[1],params[2]};
        T point_measured[3] = {T(point.x), T(point.y), T(point.z)};
        
        T point_platform[3];
        ceres::AngleAxisRotatePoint(rotation, point_measured, point_platform);
        
        point_platform[0] += T(953.672);
        point_platform[1] += T(1008.62);
        point_platform[2] += T(23.3908);
            
        residual[0] = point_platform[0];
        residual[1] = point_platform[1];
        residual[2] = point_platform[2] - params[this->zCoordParammIndex];
        
        return true;
    }
       
private:
        Point point;         
        int zCoordParammIndex;
        int radiusParamIndex;
};




#endif

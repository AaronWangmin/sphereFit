#ifndef ALL_TARGET_ALL_VERTICAL_FIT_H
#define ALL_TARGET_ALL_VERTICAL_FIT_H

#include "oneTargetAllVerticalFit.h"


class AllTargetAllVerticalFit : public OneTargetAllVerticalFit
{
public:
    // dimensionParams: 3 + 3 + 110 = 116
        //  translation (sphrer center x,y,z) : 3
        //  rotationAngleAxis : 3  
        //  
        //  horizenCircle count * 2(zCoord + radius) : 55 * 2 =  110
   
    static const int dimensionParams = 116;

public: 
    
    void getObs(const vector<Point>&);
    
    void setInitialParams(vector<tuple<vector<Point>,Point, double>>); 
    
    void fitCompute(vector<tuple<vector<Point>,Point, double>>); 
     
    int getZCoordAndRadiusIndex(const Point& point,  int& zCoordParamIndex,
                                                     int& radiusParamIndex ) const;
};

struct AllTargetAllVerticalFittingCost
{
public:
    AllTargetAllVerticalFittingCost(Point& point, 
                                    const int& zCoordParammIndex, 
                                    const int& radiusParamIndex) 
        : point(point), 
          zCoordParammIndex(zCoordParammIndex),
          radiusParamIndex(radiusParamIndex) {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {
        T* translation = new T[3]{params[0], params[1], params[2]};
        T* rotation = new T[3]{params[3], params[4], params[5]};
        
        T* point_measured =  new T[3]{T(point.x), T(point.y), T(point.z)};
        T point_platform[3];
        
        
        ceres::AngleAxisRotatePoint(rotation, point_measured, point_platform);
        
        point_platform[0] += params[0];
        point_platform[1] += params[1];
        point_platform[1] += params[2];
        
        residual[0] = pow( point_platform[0] - T(0.0), 2) + 
                      pow( point_platform[1] - T(0.0), 2) +
                      pow( point_platform[2] - params[this->zCoordParammIndex], 2) -
                      pow( params[this->radiusParamIndex], 2);
                      
        residual[1] = 
            T(0.0) * (point_platform[0] - params[0]) +
            T(0.0) * (point_platform[0] - params[1]) +
            T(1.0) * (point_platform[0] - params[this->zCoordParammIndex]);  
                      
        
        delete[] translation;
        delete[] rotation;
        delete[] point_measured;
        
        
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
    AllTargetAllVerticalFittingCost_2()  {}
    
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

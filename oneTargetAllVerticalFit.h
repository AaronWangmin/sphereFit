#ifndef ONE_TARGET_ALL_VERTICAL_FIT_H
#define ONE_TARGET_ALL_VERTICAL_FIT_H


#include "shapeFit.h"

class OneTargetAllVerticalFit : public ShapeFit
{
public:
    static const int dimensionParams;

public: 
    
    void getObs(const vector<Point>&, const string& prismName  );
    
    void setInitialParams(); 
    
    void fitCompute(); 
     
    int getZCoordAndRadiusIndex(const Point& point,  int& zCoordParamIndex,
                                                     int& radiusParamIndex ) const;
    
};

struct OneTargetAllVerticalFittingCost
{
public:
    OneTargetAllVerticalFittingCost(const Point& point, 
                                    const int& zCoordParammIndex, 
                                    const int& radiusParamIndex) 
        : point(point), 
          zCoordParammIndex(zCoordParammIndex),
          radiusParamIndex(radiusParamIndex) {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {
        residual[0] = pow( (T(point.x) - params[0]), 2) + 
                      pow( (T(point.y) - params[1]), 2) +
                      pow( (T(point.z) - params[this->zCoordParammIndex]), 2) -
                      pow(  params[this->radiusParamIndex], 2);
                      
        residual[1] = 
            params[2] * (T(point.x) - params[0]) +
            params[3] * (T(point.y) - params[1]) +
            params[4] * (T(point.z) - params[this->zCoordParammIndex]);  
                      
       return true;
    }
    
private:
        Point point;         
        int zCoordParammIndex;
        int radiusParamIndex;
};

struct OneTargetAllVerticalFittingCost_2
{
public:
    OneTargetAllVerticalFittingCost_2()  {}
    
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

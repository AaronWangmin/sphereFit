#ifndef ALL_SPHERE_FIT_H
#define ALL_SPHERE_FIT_H


#include "shapeFit.h"

// sphere center(x,yz) + everyTargetSphereRadius : 3 + 8


class AllSphereFit : public ShapeFit
{
    static const int dimensionParams;
   
public:
    
    void getObs(const vector<Point>& );
    
    void setInitialParams(); 
    
    void fitCompute(); 
    
    int getPrismIndex(const Point&) const;
};

struct AllSphereFittingCost
{
public:
    AllSphereFittingCost(const Point& point, const int& prismIndex) : point(point), prismIndex(prismIndex) {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {
        residual[0] = pow( (T(point.x) - params[0]), 2) + 
                      pow( (T(point.y) - params[1]), 2) +
                      pow( (T(point.z) - params[2]), 2) -
                      pow(  params[3 + this->prismIndex], 2);
                      
       return true;
    }
    
private:
        Point point;  
        int prismIndex;
        
};

#endif

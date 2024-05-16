#ifndef SPHERE_FIT_H
#define SPHERE_FIT_H

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include "circleFit.h"

class SphereFit : public CircleFit
{
public:
    
    SphereFit() {}
    
    void getObsCoords(vector<tuple<string,vector<Point>>>& spherePointsVector,string prism);
    
    void getObsCoords(vector<double>& x_data, vector<double>& y_data, vector<double>& z_data);
    
//     void setSphereParamInitail(const double x, const double y, const double z, const double r);  
    
    void setInitialParams(); 
    
    void fitCompute(); 
    
    void fitAllShpere();
    
    void putOutResultFile(const string& outFileDir, const vector<string>& resultVector);
    
private:
    
    
};

struct SphereFittingCost
{
public:
    SphereFittingCost(const Point& point) : point(point) {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {
        residual[0] = pow( (T(point.x) - params[0]), 2) + 
                      pow( (T(point.y) - params[1]), 2) +
                      pow( (T(point.z) - params[2]), 2) -
                      pow(  params[3], 2);
                      
       return true;
    }
    
private:
        Point point;        
        
};


#endif

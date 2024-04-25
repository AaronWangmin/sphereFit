#ifndef SPHERE_FIT_H
#define SPHERE_FIT_H

#include <ceres/ceres.h>
#include <Eigen/Dense>

class SphereFit
{
public:
    
    SphereFit() {}
     
    void getObsCoords(vector<double>& x_data, vector<double>& y_data, vector<double>& z_data);
    
    void setSphereParamInitail(const double x, const double y, const double z, const double r);  
    
    void fitCompute();
    
    void report();
    
    vector<double> & getSpherePara();
    
private:
    // 4 *1 vector: the coord of center of sphere, and radius
    vector<double> sphereParam;
    
//     void ceresInitial();
    
};

struct SphereFittingCost
{
    SphereFittingCost(const double x,const double y,const double z) : _x(x),_y(y),_z(z) {}

    template <typename T> 
    bool operator()(const T* const abcr, T* residual) const
    {
        residual[0] = 
            ceres::pow((T(_x) - abcr[0]),2) + 
            ceres::pow((T(_y) - abcr[1]),2) + 
            ceres::pow((T(_z) - abcr[2]),2) - 
            abcr[3] * abcr[3] ;            
            
        return true;
    }

    private:
        const double _x,_y,_z;
};

#endif

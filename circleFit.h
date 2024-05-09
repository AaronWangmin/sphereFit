#ifndef CIRCLE_FIT_H
#define CIRCLE_FIT_H

#include <string>

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include "dataPrepare.h"

using namespace std;

class CircleFit
{
public:
    
    CircleFit() {}
    
//     void getObsCoords(vector<double>& x_data, 
//                       vector<double>& y_data, 
//                       vector<double>& z_data,
//                       tuple<vector<Point>,Point,double>& circleDatas);
    
    void getObsCoords(tuple<vector<Point>,Point,double>& circleDatas);
    
    void setCircleParamInitail(const double x, 
                               const double y, 
                               const double z, 
                               const double r);
    
    void fitCompute(tuple<vector<Point>,Point,double>& circleHorizenDatas);
    
    void report();
    
    vector<double> & getCirclePara();
    
private:
    // 4 *1 vector: the coord of center of circle, and radius
    vector<double> circleParam; 

    // 
    vector<double> x_data;
    vector<double> y_data;
    vector<double> z_data;
};

struct CircleFittingCost
{
    CircleFittingCost(const double x,const double y,const double z) : _x(x),_y(y),_z(z) {}

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
